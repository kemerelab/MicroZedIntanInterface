#!/usr/bin/env python3
# =====================================================================
# wavelet_coeffs.py -- generate the EXACT wavelet bank coefficients that
# THIS branch's firmware uploads to the PL, for out-of-band analysis.
#
#   Branch:  claude/tier3-wavelet-k256   (the K=256 full-channel build)
#   Config:  V=4 voices/octave, N_TAPS=24, N_OCTAVES=8, HB_TAPS=7, K=256,
#            fs=3000 Hz, fc_top=0.34, gamma=3, beta=3, Q1.17 coeffs
#
# WAVELET FAMILY: true generalized-MORSE wavelet (ghostipy convention).
#   Frequency domain:  Psi(w) = a * w^beta * exp(-w^gamma),  w > 0
#   with  log a = ln2 + (beta/gamma)*(1 + ln gamma - ln beta)   (peak value 2,
#   "bandpass" normalization), peak angular freq  wp = (beta/gamma)^(1/gamma),
#   scale<->freq  freq = wp/scale. This is EXACTLY ghostipy's MorseWavelet
#   (gamma=3, beta=3): docs/validate_against_ghostipy.py checks each voice's
#   FIR response against ghostipy.spectral.MorseWavelet.freq_domain() (the
#   analytic formula matches ghostipy to 0.0; the 24-tap FIR matches its
#   passband shape to <2e-4). See docs/wavelet-coefficients.md.
#   (NOTE: beta is the Q / bandwidth knob; ghostipy's default is beta=20.
#    This bank uses beta=3 -- broad, so a true Morse atom fits N_TAPS taps.)
#
# These constants MIRROR the 3-layer contract for this branch:
#   firmware/include/main.h   (WAV_V / WAV_N_TAPS / WAV_N_OCTAVES / ...)
#   remote/net.py             (design_wavelet_bank defaults)
#   programmable_logic/sim/gen_wavelet_vectors.py   (the bit-exact sim reference)
# If you change the branch config, change these to match (or run --validate,
# which cross-checks against the sim reference in this repo).
#
# The PL is shape-agnostic: it runs a complex FIR (the "voice") per octave on a
# halfband-decimated copy of the 3 kHz LFP (an a-trous / dyadic octave cascade).
# net.py's design_wavelet_bank() DESIGNS the voice taps + the halfband and
# uploads them as signed Q1.17 integers; this script reproduces it bit-for-bit,
# so the complex floats here (int / 2**17) are the precise taps in silicon.
#
# Usage:
#   python3 wavelet_coeffs.py              # print the bank (Q1.17 + float + freqs)
#   python3 wavelet_coeffs.py --validate   # assert bit-exact vs the sim reference
#   import wavelet_coeffs as w; b = w.coefficients()   # use programmatically
# =====================================================================
import math

# ---- branch config (mirror of main.h / net.py for claude/tier3-wavelet-k256) ----
V         = 4         # voices per octave
N_TAPS    = 24        # complex FIR taps per voice
N_OCTAVES = 8         # octaves in the cascade
HB_TAPS   = 7         # halfband decimation FIR taps
FS        = 3000.0    # LFP sample rate feeding octave 0 (Hz)
FC_TOP    = 0.34      # top voice center, cycles/sample within an octave band
GAMMA     = 3.0       # Morse gamma
BETA      = 3.0       # Morse beta (Q / bandwidth knob; ghostipy default 20)
COEF_W    = 18        # signed coefficient width (bits)
COEF_FRAC = 17        # fractional bits  -> Q1.17, value = int / 2**17

# inverse-FT integration grid for the time-domain Morse wavelet (w > 0).
# a*w^beta*exp(-w^gamma) decays fast (gamma=3 -> negligible past w~4); these
# fixed constants make the design deterministic and bit-identical everywhere.
_UMAX, _NU = 12.0, 6000


def design_voice_bank(V=V, n_taps=N_TAPS, fc_top=FC_TOP, gamma=GAMMA,
                      beta=BETA, coef_frac=COEF_FRAC, coef_w=COEF_W):
    """The V constant-Q complex true-Morse voice shapes (reused at every octave).
    Returns a list of V voices; each voice is a list of n_taps dicts
    {'q': (re_int, im_int) Q1.17, 'c': complex float = re/2^f + j im/2^f}.
    Bit-for-bit identical to net.py design_wavelet_bank / the sim reference."""
    scale = (1 << coef_frac)
    lim   = (1 << (coef_w - 1))
    M     = n_taps - 1
    wp    = (beta / gamma) ** (1.0 / gamma)                     # peak angular freq
    log_a = math.log(2.0) + (beta / gamma) * (1.0 + math.log(gamma) - math.log(beta))
    du    = _UMAX / _NU
    us    = [k * du for k in range(1, _NU + 1)]                 # skip w=0 (w^beta=0)
    amp   = [math.exp(log_a + beta * math.log(u) - u ** gamma) for u in us]
    voices = []
    for v in range(V):
        fc = fc_top * (2.0 ** (-v / float(V)))                  # cycles/sample in octave
        s  = wp / (2.0 * math.pi * fc)                          # scale: peak -> fc
        raw = []
        for n in range(n_taps):
            tau = (n - M / 2.0) / s                             # psi_s(t) = psi_1(t/s)
            re = im = 0.0
            for k in range(_NU):                                # inverse FT over w>0
                u = us[k]; a = amp[k]
                re += a * math.cos(u * tau)
                im += a * math.sin(u * tau)
            raw.append((re * du / (2.0 * math.pi), im * du / (2.0 * math.pi)))
        mre = sum(r for r, _ in raw) / n_taps                   # zero-mean -> reject DC
        mim = sum(i for _, i in raw) / n_taps                   # (cleans FIR truncation)
        taps2 = [(r - mre, i - mim) for (r, i) in raw]
        norm = sum(math.hypot(re, im) for (re, im) in taps2) or 1.0     # unit L1
        voice = []
        for (re, im) in taps2:
            ri = max(-lim, min(lim - 1, int(round(re / norm * scale))))
            ii = max(-lim, min(lim - 1, int(round(im / norm * scale))))
            voice.append({'q': (ri, ii), 'c': complex(ri / scale, ii / scale)})
        voices.append(voice)
    return voices


def design_halfband(hb_taps=HB_TAPS, coef_frac=COEF_FRAC, coef_w=COEF_W):
    """The /2 anti-alias halfband (windowed-sinc @ 0.25, Hamming, unity DC).
    Applied repeatedly to build the octave cascade. Returns list of dicts
    {'q': int Q1.17, 'c': float}.  (Unchanged by the Morse switch.)"""
    scale, lim = (1 << coef_frac), (1 << (coef_w - 1))
    fcq, Mh = 0.25, hb_taps - 1
    h = []
    for n in range(hb_taps):
        x = n - Mh / 2.0
        s = 2 * fcq if abs(x) < 1e-9 else math.sin(2 * math.pi * fcq * x) / (math.pi * x)
        w = 0.54 - 0.46 * math.cos(2 * math.pi * n / Mh)
        h.append(s * w)
    g = sum(h) or 1.0
    out = []
    for c in h:
        qi = max(-lim, min(lim - 1, int(round(c / g * scale))))
        out.append({'q': qi, 'c': qi / scale})
    return out


def octave_center_freqs(V=V, n_octaves=N_OCTAVES, fc_top=FC_TOP, fs=FS):
    """centers[o][v] = analysis center frequency (Hz) of voice v at octave o.
    Octave o runs on the LFP decimated to fs/2^o; the voice's fractional
    center fc_top*2^(-v/V) maps to fc_top*2^(-v/V) * (fs/2^o) Hz."""
    return [[fc_top * (2.0 ** (-v / float(V))) * (fs / (2.0 ** o))
             for v in range(V)] for o in range(n_octaves)]


def coefficients():
    """Everything in one dict: voices, halfband, per-octave/voice center freqs,
    and the config. Voice/halfband 'c' fields are the exact silicon taps."""
    return {
        'config': dict(V=V, N_TAPS=N_TAPS, N_OCTAVES=N_OCTAVES, HB_TAPS=HB_TAPS,
                       FS=FS, FC_TOP=FC_TOP, GAMMA=GAMMA, BETA=BETA,
                       COEF_W=COEF_W, COEF_FRAC=COEF_FRAC),
        'voices': design_voice_bank(),
        'halfband': design_halfband(),
        'centers_hz': octave_center_freqs(),
    }


def _validate():
    """Cross-check this generator against the in-repo sim reference
    (programmable_logic/sim/gen_wavelet_vectors.py) -- the bit-exact source
    the wavelet engine TB is checked against, and which net.py's
    design_wavelet_bank() mirrors. Returns True on full match."""
    import importlib.util, os
    here = os.path.dirname(os.path.abspath(__file__))
    ref_path = os.path.join(here, "..", "programmable_logic", "sim", "gen_wavelet_vectors.py")
    spec = importlib.util.spec_from_file_location("gwv", ref_path)
    g = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(g)
    ref_v = g.morse_voice_shapes()
    ref_h = g.halfband_coeffs()
    my_v = [[t['q'] for t in voice] for voice in
            design_voice_bank(V=g.V, n_taps=g.N_TAPS, gamma=g.GAMMA, beta=g.BETA,
                              coef_frac=g.COEF_FRAC, coef_w=g.COEF_W)]
    my_h = [t['q'] for t in design_halfband(g.HB_TAPS, g.COEF_FRAC, g.COEF_W)]
    ok = (my_v == ref_v) and (my_h == ref_h)
    print("sim reference: V=%d N_TAPS=%d HB=%d gamma=%g beta=%g" %
          (g.V, g.N_TAPS, g.HB_TAPS, g.GAMMA, g.BETA))
    print("voices  bit-exact:", my_v == ref_v)
    print("halfband bit-exact:", my_h == ref_h)
    print("RESULT:", "PASS" if ok else "FAIL")
    return ok


def _print_bank():
    b = coefficients()
    c = b['config']
    print("# wavelet bank -- branch claude/tier3-wavelet-k256  (true Morse, ghostipy)")
    print("# V=%d N_TAPS=%d N_OCTAVES=%d  fs=%g Hz  fc_top=%g  gamma=%g beta=%g  Q%d.%d"
          % (c['V'], c['N_TAPS'], c['N_OCTAVES'], c['FS'], c['FC_TOP'], c['GAMMA'],
             c['BETA'], c['COEF_W'] - 1 - c['COEF_FRAC'], c['COEF_FRAC']))
    print("\n## halfband /2 (Q1.17 int | float):")
    print("  ", [t['q'] for t in b['halfband']])
    print("  ", ["%.6f" % t['c'] for t in b['halfband']])
    print("\n## center frequencies (Hz), centers[octave][voice]:")
    for o, row in enumerate(b['centers_hz']):
        print("  oct%-2d (fs=%7.2f Hz): %s" %
              (o, c['FS'] / (2 ** o), ["%8.2f" % f for f in row]))
    print("\n## voice taps (complex float = int/2^17), one row per voice:")
    for v, voice in enumerate(b['voices']):
        print("  voice %d (top-octave center %.2f Hz):" % (v, b['centers_hz'][0][v]))
        for n, t in enumerate(voice):
            print("    n=%2d  q=(%7d,%7d)  c=%+.6f%+.6fj" %
                  (n, t['q'][0], t['q'][1], t['c'].real, t['c'].imag))


if __name__ == "__main__":
    import sys
    if "--validate" in sys.argv:
        sys.exit(0 if _validate() else 1)
    _print_bank()
