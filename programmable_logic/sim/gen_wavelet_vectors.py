#!/usr/bin/env python3
# =====================================================================
# gen_wavelet_vectors.py
#
# Python reference + bit-exact test vectors for the Tier-3 multirate
# wavelet (constant-Q / Morse) scalogram engine. Mirrors
# gen_stft_vectors.py / gen_lfp_fir_vectors.py: pure-Python (no numpy),
# integer fixed-point arithmetic that matches the RTL exactly.
#
# What it produces (all in programmable_logic/sim/):
#   wav_coef.hex      V voice shapes x N_TAPS complex coeffs, Q1.17, as
#                     32-bit words {im[15:0]... } -- see packing below
#   wav_hb.hex        halfband ÷2 FIR coeffs (Q1.17, signed 18b in 32b word)
#   wav_samp.hex      K lanes x int16 LFP samples, one 32-bit word per
#                     (frame, lane) carrying 2 lanes? No: one word/lane,
#                     lane0..K-1 interleaved per frame (see writer)
#   wav_exp.hex       expected (re,im) int per (lane, octave, voice), the
#                     LATEST scalogram column after all frames, as signed
#                     32-bit hex, interleaved re then im, ordered
#                     lane-major / octave / voice
#   wav_meta.txt      human-readable config echo
#
# Fixed-point contract (MUST match the RTL):
#   * input samples: int16 signed (offset-binary removed upstream)
#   * coeffs: signed Q1.17 in an 18-bit field
#   * complex MAC: acc_re = sum(coef_re[j] * x[j]); acc_im = sum(coef_im[j]*x[j])
#     (x real LFP -> 2 real MACs/tap). acc is a wide signed integer.
#   * output: round-to-nearest then >> (COEF_FRAC - OUT_GAIN_SHIFT[o]) then
#     saturate to OUT_W. Per-octave gain is a left-shift (OUT_GAIN_SHIFT) to
#     recover 1/f dynamic range; here implemented as a reduction of the
#     down-shift. Default gains chosen per octave below.
#   * halfband ÷2: integer FIR, round-to-nearest >> COEF_FRAC, saturate int16.
#
# Multirate cascade (a trous / dyadic):
#   octave 0 runs on the raw fs samples.
#   octave o (>=1) runs on samples decimated by 2^o, produced by cascading
#   the halfband ÷2: hb0 = HB(x)@fs/2 feeds octave1 AND hb1=HB(hb0)@fs/4 ...
#   Each octave applies the SAME V voice shapes (constant-Q): the center
#   frequency in Hz halves each octave because the sample rate halves.
# =====================================================================
import os, math, struct

OUT = os.path.dirname(os.path.abspath(__file__))

# ---- config (keep in lockstep with the TB localparams) ----
FS        = 3000.0     # Tier-1 LFP rate (Phase B target; Phase A raises 2->3 kHz)
K         = 4          # selected channels (lanes) -- small for a fast TB
N_OCTAVES = 4          # octaves in the cascade (TB uses a few; HW uses 8)
V         = 4          # voices per octave (constant-Q grid density)
N_TAPS    = 24         # complex taps per voice (per the Morse support; even)
HB_TAPS   = 7          # halfband ÷2 FIR taps (odd, symmetric)
DATA_W    = 16
COEF_W    = 18
COEF_FRAC = 17
OUT_W     = 18         # scalogram output width (signed); wider than 16 for headroom
N_FRAMES  = 256        # base-rate frames fed to the engine

# ---- RUNTIME active config (<= the build maxes above). The engine builds the
# COMPLETE wire packet in its results BRAM using the COMPACTED lane stride
# (nscales = ACT_OCTAVES*ACT_VOICES, only active scales, contiguous). Choosing
# ACT < build max exercises the compaction path (the whole point of moving the
# repack into the PL). ----
ACT_OCTAVES = 3        # active octaves this run (< N_OCTAVES build max -> compaction)
ACT_VOICES  = 3        # active voices/octave (< V build max -> compaction)
ACT_TAPS    = N_TAPS   # active taps (engine caps at N_TAPS; keep full here)
N_SCALES    = ACT_OCTAVES * ACT_VOICES   # compacted lane stride on the wire

# ---- wire-packet header (matches the RTL wavelet_cqt_engine + net.py) ----
WAV_MAGIC_LOW  = 0x5CA70900
WAV_MAGIC_HIGH = 0xCAFEBABE
HDR_WORDS      = 8

# Generalized Morse wavelet parameters (gamma=3 is the locked default).
GAMMA = 3.0
BETA  = 3.0            # beta sets the time-bandwidth product / Q; Q ~ sqrt(beta*gamma)

# Per-octave output gain as a LEFT shift applied before the final down-shift.
# 1/f power means low octaves (slow rhythms) carry far more energy than high
# octaves, so we shift the HIGH octaves up to fill the fixed-point range.
# octave 0 = highest freq band -> biggest boost. (Tunable host knob in HW.)
OUT_GAIN_SHIFT = [3, 2, 1, 0, 0, 0, 0, 0][:N_OCTAVES]

# ---- tiny deterministic LCG (no numpy dependency) ----
_state = 0x5EED1234
def rnd(lo, hi):
    global _state
    _state = (_state * 6364136223846793005 + 1442695040888963407) & ((1 << 64) - 1)
    return lo + (_state >> 17) % (hi - lo + 1)


# =====================================================================
# Morse voice-shape design (host side). Produces V normalized complex
# bandpass shapes, each N_TAPS long, quantized to Q1.17. The shapes are
# REUSED across octaves (constant-Q), so the center frequency as a
# fraction of the octave sample rate is the same for every octave.
# =====================================================================
def morse_voice_shapes():
    """V complex FIR voices (constant-Q Morse, gamma=GAMMA, beta=BETA),
    each N_TAPS taps, normalized then quantized to signed Q1.17.
    Returns voices[v] = list of (re_int, im_int) tuples."""
    scale = (1 << COEF_FRAC)
    lim   = (1 << (COEF_W - 1))
    # Voice v covers a center frequency fc_v as a fraction of the octave
    # sample rate. The top octave maps to the highest band; the V voices
    # tile one octave (a factor of 2 in frequency). Center frac for the
    # top voice ~0.34 (just under Nyquist-with-headroom), descending by
    # 2^(-v/V) so the V voices span one octave.
    # peak frac of the octave's Nyquist band:
    fc_top = 0.34
    voices = []
    M = N_TAPS - 1
    for v in range(V):
        fc = fc_top * (2.0 ** (-v / float(V)))   # cycles/sample in this octave
        omega_c = 2.0 * math.pi * fc
        # Morse-like envelope: a Gaussian-in-log-frequency wavelet has a
        # time-domain envelope ~ a generalized Gaussian. We synthesize the
        # analytic bandpass directly: g(t) = env(t) * exp(j*omega_c*t),
        # with env a Gaussian whose width gives the constant-Q bandwidth.
        # Q = fc / bandwidth; for a Morse wavelet Q ~ sqrt(beta*gamma).
        Q = math.sqrt(BETA * GAMMA)
        # time spread (samples): sigma_t ~ Q / (2*pi*fc) * k  (k packs the
        # support into N_TAPS). Choose sigma so +/-3 sigma fits N_TAPS.
        sigma = (N_TAPS / 6.0)
        # but also tie sigma to Q so higher Q -> longer support (constant-Q):
        sigma = min(sigma, Q / (2.0 * math.pi * fc) * 1.0)
        sigma = max(sigma, 1.5)
        taps_c = []
        re_e = im_e = 0.0
        for n in range(N_TAPS):
            t = n - M / 2.0
            env = math.exp(-0.5 * (t / sigma) ** 2)
            re = env * math.cos(omega_c * t)
            im = env * math.sin(omega_c * t)
            taps_c.append((re, im))
            re_e += re
            im_e += im
        # zero-mean (admissibility): subtract the DC leakage of the real part
        # (analytic wavelet should have ~0 mean). Use the envelope sum to
        # de-bias the cosine part so the voice rejects DC.
        env_sum = sum(math.exp(-0.5 * ((n - M/2.0)/sigma) ** 2) for n in range(N_TAPS))
        dc = re_e / env_sum if env_sum != 0 else 0.0
        taps2 = []
        for n in range(N_TAPS):
            t = n - M / 2.0
            env = math.exp(-0.5 * (t / sigma) ** 2)
            re = taps_c[n][0] - dc * env
            im = taps_c[n][1]
            taps2.append((re, im))
        # normalize to unit L1 of the complex magnitude so a unit-amplitude
        # tone at fc gives a predictable response and stays in range.
        norm = sum(math.hypot(r, i) for (r, i) in taps2) or 1.0
        q = []
        for (re, im) in taps2:
            ri = int(round(re / norm * scale))
            ii = int(round(im / norm * scale))
            ri = max(-lim, min(lim - 1, ri))
            ii = max(-lim, min(lim - 1, ii))
            q.append((ri, ii))
        voices.append(q)
    return voices


# =====================================================================
# Halfband ÷2 anti-alias FIR (host side). Symmetric windowed-sinc at
# fc = 0.25 (quarter-rate), quantized Q1.17, unity DC gain.
# =====================================================================
def halfband_coeffs():
    fc = 0.25
    M = HB_TAPS - 1
    h = []
    for n in range(HB_TAPS):
        x = n - M / 2.0
        s = 2 * fc if abs(x) < 1e-9 else math.sin(2 * math.pi * fc * x) / (math.pi * x)
        w = 0.54 - 0.46 * math.cos(2 * math.pi * n / M)   # Hamming
        h.append(s * w)
    g = sum(h) or 1.0
    scale, lim = (1 << COEF_FRAC), (1 << (COEF_W - 1))
    return [max(-lim, min(lim - 1, int(round(c / g * scale)))) for c in h]


# =====================================================================
# Synthetic LFP (multi-tone + injected ripples + a log chirp), one
# independent trace per lane. Mirrors ripple_detect_prototype.synth_lfp
# in spirit but pure-Python and per-lane differentiated.
# =====================================================================
def synth_lfp_lane(lane):
    n = N_FRAMES
    x = [0.0] * n
    # a slow theta-band rhythm + a faster gamma component, lane-shifted phase
    f_theta = 6.0 + lane            # 6..9 Hz
    f_gamma = 60.0 + 10.0 * lane    # 60..90 Hz
    ph = lane * 0.7
    for i in range(n):
        t = i / FS
        x[i] = (900.0 * math.sin(2 * math.pi * f_theta * t + ph)
                + 300.0 * math.sin(2 * math.pi * f_gamma * t))
    # inject one ripple burst (150-250 Hz) with a Gaussian envelope
    f_rip = 180.0 + 20.0 * lane
    c = n // 2 + lane * 7
    half = 24
    for j in range(-half, half):
        idx = c + j
        if 0 <= idx < n:
            env = math.exp(-0.5 * (j / (half / 2.0)) ** 2)
            x[idx] += 500.0 * env * math.sin(2 * math.pi * f_rip * j / FS)
    # quantize to int16 with a touch of deterministic noise
    out = []
    for i in range(n):
        v = int(round(x[i])) + (rnd(-3, 3))
        out.append(max(-32768, min(32767, v)))
    return out


# =====================================================================
# Fixed-point primitives matching the RTL.
# =====================================================================
def rnd_shift(acc, sh):
    """round-to-nearest then arithmetic shift right by sh (sh may be <=0)."""
    if sh <= 0:
        return acc << (-sh)
    return (acc + (1 << (sh - 1))) >> sh   # Python >> floors like Verilog >>>

def sat(v, w):
    hi = (1 << (w - 1)) - 1
    lo = -(1 << (w - 1))
    return hi if v > hi else lo if v < lo else v


# =====================================================================
# Halfband ÷2 decimator reference. Polyphase-equivalent: produce one
# output every 2 input samples = FIR convolution evaluated at even output
# indices, with the standard causal delay-line semantics (ring is 0 before
# the first sample, matching the RTL BRAM config-init).
# =====================================================================
def halfband_decimate(x, hb):
    """x: int list at rate R. Returns y at rate R/2 (one output per 2 inputs).
    Matches the RTL's causal incremental decimator exactly: octave-o output m
    is produced when input index 2*m has just been pushed, reading the newest
    HB_TAPS samples (newest = hb[0]):
        y[m] = sat16( rnd>>FRAC( sum_j hb[j] * x[2*m - j] ) ),  x[<0]=0."""
    y = []
    n = len(x)
    n_out = (n + 1) // 2          # outputs at input indices 0,2,4,... < n
    for m in range(n_out):
        newest = 2 * m            # the just-pushed sample for this output
        acc = 0
        for j in range(HB_TAPS):
            idx = newest - j
            xv = x[idx] if 0 <= idx < n else 0
            acc += hb[j] * xv
        r = rnd_shift(acc, COEF_FRAC)
        y.append(sat(r, DATA_W))
    return y


# =====================================================================
# Complex voice MAC reference at a given octave's sample stream.
# Produces the complex output of every voice for EVERY output sample, but
# the TB only checks the LAST column (most-recent), so we keep the last.
# =====================================================================
def voice_mac(stream, voices, octave):
    """stream: int list (this octave's samples). voices: V x N_TAPS complex.
    Returns last_col[v] = (re_out, im_out) using the last ACT_TAPS samples, for
    the ACT_VOICES active voices (matches the engine's runtime n_voices/n_taps)."""
    sh = COEF_FRAC - OUT_GAIN_SHIFT[octave]
    n = len(stream)
    out = []
    for v in range(ACT_VOICES):
        acc_re = acc_im = 0
        for j in range(ACT_TAPS):
            idx = (n - 1) - j         # newest sample at j=0
            xv = stream[idx] if 0 <= idx < n else 0
            cre, cim = voices[v][j]
            acc_re += cre * xv
            acc_im += cim * xv
        re = sat(rnd_shift(acc_re, sh), OUT_W)
        im = sat(rnd_shift(acc_im, sh), OUT_W)
        out.append((re, im))
    return out


def main():
    voices = morse_voice_shapes()
    hb     = halfband_coeffs()

    # ---- per-lane synthetic LFP ----
    samp = [synth_lfp_lane(l) for l in range(K)]   # samp[lane][frame]

    # ---- write coefficient hex (voice-major, tap-major; {im,re} packed) ----
    # Each complex coeff -> one 32-bit word: [31:16]=im(16 low bits of 18b?),
    # No: we keep full 18b each by packing re in low 18 and im in high... that
    # overflows 32b. Instead emit TWO 32-bit words per coeff: re then im, each
    # sign-extended into 32 bits. Simpler for $readmemh and the RTL coef RAM
    # (which stores re and im in separate 18-bit fields read on consecutive
    # addresses). The RTL coef RAM is COEF_W-wide; the TB loads re,im as a
    # pair via the upload window. We emit them interleaved re,im.
    with open(f"{OUT}/wav_coef.hex", "w") as f:
        for v in range(V):
            for j in range(N_TAPS):
                cre, cim = voices[v][j]
                f.write(format(cre & ((1 << COEF_W) - 1), '05x') + "\n")
                f.write(format(cim & ((1 << COEF_W) - 1), '05x') + "\n")

    with open(f"{OUT}/wav_hb.hex", "w") as f:
        for c in hb:
            f.write(format(c & ((1 << COEF_W) - 1), '05x') + "\n")

    # ---- samples: one 32-bit word per (frame, lane), int16 in low 16 bits ----
    with open(f"{OUT}/wav_samp.hex", "w") as f:
        for fr in range(N_FRAMES):
            for l in range(K):
                f.write(format(samp[l][fr] & 0xFFFF, '08x') + "\n")

    # ---- reference scalogram: per lane, build the octave cascade, take the
    #      most-recent column of every active (octave, voice). COMPACTED:
    #      only ACT_OCTAVES octaves x ACT_VOICES voices = N_SCALES scales,
    #      ordered scale = octave*ACT_VOICES + voice (octave-major). ----
    payload = []   # signed ints, interleaved re,im, ordered lane / octave / voice
    for l in range(K):
        stream = samp[l][:]                 # octave 0 stream
        streams = [stream]
        for o in range(1, ACT_OCTAVES):
            stream = halfband_decimate(stream, hb)
            streams.append(stream)
        for o in range(ACT_OCTAVES):
            col = voice_mac(streams[o], voices, o)   # ACT_VOICES complex bins
            for (re, im) in col:
                payload.append(re)
                payload.append(im)

    # ---- build the COMPLETE wire packet the PL now writes into the results
    #      BRAM (8-word header + compacted payload). The TB snapshots the BRAM
    #      and compares it word-for-word against this. The engine runs ONE pass
    #      per kicking frame-start that had staged data: frames 1..N_FRAMES
    #      (frame f's pass is kicked by frame f+1's frame-start; the trailing
    #      trigger frame N_FRAMES flushes the last real frame). So N_FRAMES
    #      passes complete and frame_seq -> N_FRAMES; the header (written at pass
    #      start as frame_seq+1) of the LAST pass carries N_FRAMES, which is what
    #      remains in the BRAM. ----
    SEQ = N_FRAMES
    gain_word = 0
    for o in range(N_OCTAVES):                       # 4 bits/octave, build-width
        g = OUT_GAIN_SHIFT[o] if o < len(OUT_GAIN_SHIFT) else 0
        gain_word |= (g & 0xF) << (4 * o)
    w4 = (ACT_OCTAVES & 0xFF) | ((ACT_VOICES & 0xFF) << 8) \
       | ((K & 0xFF) << 16) | ((0 & 1) << 24)        # overrun must be 0 (PASS)
    w6 = (N_SCALES & 0xFFFF) | ((ACT_TAPS & 0xFFFF) << 16)
    header = [WAV_MAGIC_LOW, WAV_MAGIC_HIGH, SEQ, 0, w4, SEQ, w6, gain_word]
    wire = header + payload

    with open(f"{OUT}/wav_exp.hex", "w") as f:
        for v in wire:
            f.write(format(v & 0xFFFFFFFF, '08x') + "\n")

    with open(f"{OUT}/wav_meta.txt", "w") as f:
        f.write(f"FS={FS} K={K} N_OCTAVES={N_OCTAVES} V={V} N_TAPS={N_TAPS} "
                f"HB_TAPS={HB_TAPS} DATA_W={DATA_W} COEF_W={COEF_W} "
                f"COEF_FRAC={COEF_FRAC} OUT_W={OUT_W} N_FRAMES={N_FRAMES}\n")
        f.write(f"GAMMA={GAMMA} BETA={BETA} OUT_GAIN_SHIFT={OUT_GAIN_SHIFT}\n")
        f.write(f"ACTIVE: oct={ACT_OCTAVES} V={ACT_VOICES} taps={ACT_TAPS} "
                f"nscales={N_SCALES} (compacted wire stride)\n")
        # center frequencies per (octave, voice), in Hz
        fc_top = 0.34
        f.write("center frequencies (Hz):\n")
        for o in range(N_OCTAVES):
            fr = FS / (2.0 ** o)
            line = "  oct%d @ %.1f Hz: " % (o, fr)
            for v in range(V):
                fc = fc_top * (2.0 ** (-v / float(V))) * fr
                line += "%.1f " % fc
            f.write(line + "\n")

    print(f"K={K} build(oct={N_OCTAVES},V={V},taps={N_TAPS}) "
          f"active(oct={ACT_OCTAVES},V={ACT_VOICES},taps={ACT_TAPS}) hb={HB_TAPS} "
          f"frames={N_FRAMES} -> nscales={N_SCALES} (compacted) "
          f"wire_words={len(wire)} = {HDR_WORDS} hdr + {len(payload)} payload "
          f"(re,im interleaved)")


if __name__ == "__main__":
    main()
