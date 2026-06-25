#!/usr/bin/env python3
"""
wav_validate.py -- pure-Python (no numpy) ridge analyzer for the Tier-3 Morse
wavelet scalogram stream, plus a synthetic packet injector.

This is the HEADLESS-TESTABLE core of net.py's `wav_validate` / `wav_selftest`
commands. Everything here is hardware-free: it operates on already-decoded
scalogram columns (or raw 5004 datagrams it decodes with the SAME math as
net.py::receive_wavelet / dwt_plot.decode_wavelet_packet) so the analyzer can
be exercised with no board.

A scalogram "column" == one 5004 packet == per lane, nscales complex bins at one
time step.  nscales = n_octaves * n_voices ; scale index s = octave*n_voices +
voice (octave-major).  Bin 0 is the HIGHEST center frequency; frequency is
monotonically DECREASING with s (each octave halves fs, and within an octave a
higher voice -> lower freq).  So a tone swept LOW->HIGH in Hz lights up a ridge
that walks from a HIGH scale index toward a LOW scale index: a diagonal ridge.

The center-frequency mapping is the authoritative one from
net.py::design_wavelet_bank (octave o, voice v) -> fc_top*2^(-v/V)*(fs/2^o).
"""
import math
import struct

# --- wire constants (mirror net.py WAV_* / dwt_plot) ---
WAV_MAGIC_LOW  = 0x5CA70900
WAV_MAGIC_HIGH = 0xCAFEBABE
WAV_HDR_WORDS  = 8
WAV_HDR_BYTES  = WAV_HDR_WORDS * 4


# ---------------------------------------------------------------------------
# Center-frequency table -- identical math to net.py::design_wavelet_bank's
# `centers` return (the part the ridge analyzer needs). FLAT list of length
# n_octaves*n_voices indexed by scale s = octave*n_voices + voice. Bin 0 =
# highest freq.
# ---------------------------------------------------------------------------
def scale_center_freqs(n_octaves, n_voices, fs=3000.0, fc_top=0.34):
    centers = []
    for o in range(n_octaves):
        fr = fs / (2.0 ** o)
        for v in range(n_voices):
            centers.append(fc_top * (2.0 ** (-v / float(n_voices))) * fr)
    return centers


# ---------------------------------------------------------------------------
# Packet decode -- byte-for-byte the same as net.py::receive_wavelet and
# dwt_plot.decode_wavelet_packet. Returns dict or None.
# ---------------------------------------------------------------------------
def decode_wavelet_packet(data):
    if len(data) < WAV_HDR_BYTES:
        return None
    (mlo, mhi, ts_lo, ts_hi, w4, seq, w6, gain) = struct.unpack(
        '<IIIIIIII', data[:WAV_HDR_BYTES])
    if mlo != WAV_MAGIC_LOW or mhi != WAV_MAGIC_HIGH:
        return None
    n_oct   = w4 & 0xFF
    n_voc   = (w4 >> 8) & 0xFF
    K       = (w4 >> 16) & 0xFF
    overrun = (w4 >> 24) & 1
    nscales = w6 & 0xFFFF
    n_taps  = (w6 >> 16) & 0xFFFF
    if K == 0 or nscales == 0:
        return None
    pay = data[WAV_HDR_BYTES:]
    expect = K * nscales * 2
    if len(pay) // 4 < expect:
        return None
    vals = struct.unpack('<%di' % expect, pay[:expect * 4])
    mag = []
    for lane in range(K):
        base = lane * nscales * 2
        mag.append([math.hypot(vals[base + 2 * s], vals[base + 2 * s + 1])
                    for s in range(nscales)])
    return dict(seq=seq, ts=(ts_hi << 32) | ts_lo, n_oct=n_oct, n_voc=n_voc,
                K=K, overrun=overrun, nscales=nscales, n_taps=n_taps,
                gain=gain, mag=mag)


# ---------------------------------------------------------------------------
# Synthetic packet builder -- same shape as dwt_plot.build_packet, kept here so
# the analyzer self-test has no cross-import surprises. `active` is a dict
# {(lane, scale): amplitude_fraction}. A per-scale 1/f-ish baseline is added so
# the surface looks like a real (noisy) scalogram, not a delta.
# ---------------------------------------------------------------------------
def build_packet(seq, K=4, n_oct=8, n_voc=4, n_taps=24, gain=0,
                 active=None, amp=1 << 20, base_frac=0.05):
    nscales = n_oct * n_voc
    w4 = (n_oct & 0xFF) | ((n_voc & 0xFF) << 8) | ((K & 0xFF) << 16)
    w6 = (nscales & 0xFFFF) | ((n_taps & 0xFFFF) << 16)
    hdr = struct.pack('<IIIIIIII', WAV_MAGIC_LOW, WAV_MAGIC_HIGH,
                      seq & 0xFFFFFFFF, 0, w4, seq & 0xFFFFFFFF, w6,
                      gain & 0xFFFFFFFF)
    active = active or {}
    body = bytearray()
    for lane in range(K):
        for s in range(nscales):
            a = active.get((lane, s), 0.0)
            mag = amp / (1.0 + s) * base_frac + a * amp
            phase = 0.3 * s + 0.1 * seq
            body += struct.pack('<ii', int(mag * math.cos(phase)),
                                int(mag * math.sin(phase)))
    return hdr + bytes(body)


def synth_chirp_columns(n_frames, K=4, n_oct=8, n_voc=4, n_taps=24,
                        s_lo=None, s_hi=None, triangle=True, ridge_amp=1.0,
                        steady_scale=None, steady_amp=0.6):
    """Yield raw 5004 datagrams whose lane-`*` ridge sweeps scale index s from
    s_hi down to s_lo and (if triangle) back -- i.e. a tone swept low->high->low
    in Hz, since freq DECREASES with s. This is the KNOWN injected pattern the
    analyzer must recover. Returns (datagrams, truth) where truth[i] = the ridge
    scale index for column i."""
    nscales = n_oct * n_voc
    if s_lo is None:
        s_lo = 0
    if s_hi is None:
        s_hi = nscales - 1
    if steady_scale is None:
        steady_scale = nscales // 2
    pkts, truth = [], []
    for i in range(n_frames):
        if triangle:
            # 0..1..0 triangle over the run
            frac = i / float(max(1, n_frames - 1))
            tri = 1.0 - abs(2.0 * frac - 1.0)        # 0 at ends, 1 mid
            # low->high->low in Hz == high-s -> low-s -> high-s
            s_ridge = int(round(s_hi - tri * (s_hi - s_lo)))
        else:
            frac = i / float(max(1, n_frames - 1))
            s_ridge = int(round(s_hi - frac * (s_hi - s_lo)))
        s_ridge = max(0, min(nscales - 1, s_ridge))
        active = {}
        for lane in range(K):
            active[(lane, s_ridge)] = ridge_amp
            if steady_amp:
                active[(lane, steady_scale)] = steady_amp
        pkts.append(build_packet(i, K=K, n_oct=n_oct, n_voc=n_voc,
                                 n_taps=n_taps, active=active))
        truth.append(s_ridge)
    return pkts, truth


# ---------------------------------------------------------------------------
# Ridge analyzer core. Given decoded columns for ONE lane, find the dominant
# (peak-energy) scale bin per column and assess whether it forms a clean,
# sweeping ridge. Pure math, no I/O, so net.py's HW harness and the headless
# self-test share it.
# ---------------------------------------------------------------------------
class RidgeResult:
    def __init__(self):
        self.n_columns = 0
        self.ridge_bins = []        # per-column argmax scale index
        self.ridge_freqs = []       # per-column center freq (Hz) of that bin
        self.peak_ratio = []        # per-column peak / median (sharpness)
        self.bins_covered = 0       # distinct ridge bins visited
        self.span_bins = 0          # max-min ridge bin
        self.median_sharpness = 0.0
        self.sweeps = False         # ridge actually moved across scales
        self.monotone_frac = 0.0    # fraction of the dominant trend direction
        self.f_lo = 0.0
        self.f_hi = 0.0


def analyze_ridge(columns, n_octaves, n_voices, fs=3000.0, fc_top=0.34,
                  min_peak_ratio=2.0):
    """columns: list of per-scale magnitude lists (one lane). Returns a
    RidgeResult. The ridge bin for a column is the argmax magnitude; sharpness
    is peak/median of the column (>1 means a real ridge, not flat noise)."""
    r = RidgeResult()
    if not columns:
        return r
    nscales = n_octaves * n_voices
    centers = scale_center_freqs(n_octaves, n_voices, fs, fc_top)
    for col in columns:
        if len(col) < nscales:
            continue
        c = col[:nscales]
        peak = max(c)
        b = c.index(peak)
        srt = sorted(c)
        med = srt[len(srt) // 2] if srt else 0.0
        ratio = (peak / med) if med > 1e-9 else (peak if peak > 0 else 0.0)
        # only count this column toward the ridge if it has a real peak
        if ratio >= min_peak_ratio and peak > 0:
            r.ridge_bins.append(b)
            r.ridge_freqs.append(centers[b] if b < len(centers) else 0.0)
            r.peak_ratio.append(ratio)
    r.n_columns = len(columns)
    if not r.ridge_bins:
        return r
    r.bins_covered = len(set(r.ridge_bins))
    r.span_bins = max(r.ridge_bins) - min(r.ridge_bins)
    sp = sorted(r.peak_ratio)
    r.median_sharpness = sp[len(sp) // 2]
    rf = [f for f in r.ridge_freqs if f > 0]
    if rf:
        r.f_lo, r.f_hi = min(rf), max(rf)
    # "sweeps" == the ridge visited a meaningful range of bins
    r.sweeps = r.span_bins >= max(2, nscales // 4) and r.bins_covered >= 3
    # dominant trend direction over the ridge sequence (diagonal-ness)
    ups = downs = 0
    for a, bb in zip(r.ridge_bins, r.ridge_bins[1:]):
        if bb > a:
            ups += 1
        elif bb < a:
            downs += 1
    moves = ups + downs
    r.monotone_frac = (max(ups, downs) / moves) if moves else 0.0
    return r


def recover_truth_error(ridge_bins, truth_bins):
    """Mean |recovered - injected| ridge-bin error, aligning by index. Used by
    the headless self-test to assert the analyzer recovered the injected sweep.
    Returns (mean_abs_err, max_abs_err, n) over the overlapping columns."""
    n = min(len(ridge_bins), len(truth_bins))
    if n == 0:
        return float('inf'), float('inf'), 0
    errs = [abs(ridge_bins[i] - truth_bins[i]) for i in range(n)]
    return sum(errs) / n, max(errs), n


# ---------------------------------------------------------------------------
# Headless self-test entry point (also runnable as `python3 wav_validate.py`).
# Injects a known scale-sweep, decodes with the production decoder, runs the
# ridge analyzer, and ASSERTS the recovered ridge tracks the injected one.
# ---------------------------------------------------------------------------
def run_selftest(n_frames=240, K=4, n_oct=8, n_voc=4, lane=0, verbose=True):
    nscales = n_oct * n_voc
    pkts, truth = synth_chirp_columns(n_frames, K=K, n_oct=n_oct, n_voc=n_voc,
                                      triangle=True)
    # decode every packet with the PRODUCTION decoder (proves the wire format)
    cols = []
    decoded_bins = []
    bad = 0
    for raw in pkts:
        pkt = decode_wavelet_packet(raw)
        if pkt is None:
            bad += 1
            continue
        cols.append(pkt['mag'][lane])
    res = analyze_ridge(cols, n_oct, n_voc, fs=3000.0)
    # align analyzer's accepted columns back to truth: analyze_ridge keeps only
    # columns whose peak is sharp; with our synthetic ridge ALL columns pass, so
    # the index alignment is 1:1. Guard that here.
    decoded_bins = res.ridge_bins
    mean_err, max_err, n = recover_truth_error(decoded_bins, truth)
    centers = scale_center_freqs(n_oct, n_voc, fs=3000.0)
    ok = (bad == 0 and n == n_frames and mean_err < 0.5 and max_err <= 1
          and res.sweeps and res.bins_covered >= nscales // 2)
    if verbose:
        print("=" * 64)
        print("  WAVELET RIDGE ANALYZER -- HEADLESS SELF-TEST")
        print("=" * 64)
        print(f"  injected   : triangle sweep s={truth[0]}.."
              f"{min(truth)}..{truth[-1]} over {n_frames} columns "
              f"(K={K} n_oct={n_oct} V={n_voc} nscales={nscales})")
        print(f"  decoded    : {len(cols)}/{n_frames} columns ok, {bad} undecodable")
        print(f"  recovered  : {n} ridge columns, bins_covered={res.bins_covered}"
              f"/{nscales}  span={res.span_bins} bins  "
              f"sharpness(med)={res.median_sharpness:.1f}x")
        print(f"  freq range : {res.f_lo:.1f} .. {res.f_hi:.1f} Hz "
              f"(bin0={centers[0]:.1f}  bin{nscales-1}={centers[-1]:.1f})")
        print(f"  ridge err  : mean|recovered-injected|={mean_err:.3f} bins, "
              f"max={max_err} bins")
        print(f"  sweeps?    : {res.sweeps}   diagonal monotone_frac="
              f"{res.monotone_frac:.2f}")
        # show a few aligned samples
        show = min(8, n)
        idxs = [int(round(i * (n - 1) / (show - 1))) for i in range(show)] if show > 1 else [0]
        print("  sample columns (idx: injected_s -> recovered_s @ Hz):")
        for i in idxs:
            b = decoded_bins[i]
            print(f"     col {i:3d}: s={truth[i]:2d} -> s={b:2d} "
                  f"@ {centers[b]:7.1f} Hz")
        print("-" * 64)
        print(f"  RESULT: {'PASS' if ok else 'FAIL'} -- analyzer "
              f"{'recovered' if ok else 'did NOT recover'} the injected ridge")
        print("=" * 64)
    return ok, res, (mean_err, max_err, n)


if __name__ == "__main__":
    import sys
    ok, _, _ = run_selftest()
    sys.exit(0 if ok else 1)
