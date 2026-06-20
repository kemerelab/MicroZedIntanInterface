#!/usr/bin/env python3
"""
STFT vs. FIR-filter latency / differentiation, on the lab's real parameters.

Synthetic LFP at 3 kHz with 100 ms Gaussian-envelope events whose carrier is
either 175 Hz (ripple) or 100 Hz (high gamma). We measure, for each method and a
sweep of analysis lengths:
  - latency  : the lag (ms) of the estimated band-power behind the true event
               envelope (cross-correlation peak) = the method's group-delay-equivalent.
  - sep (dB) : ripple-band / gamma-band power during ripple events = how well it
               DIFFERENTIATES the two carriers.
Filter method = the lab setup (Hamming-window FIR bandpass + 33-tap 50 Hz LP
envelope, both causal). STFT method = causal sliding Hann window, band power
summed over the band's bins. Pure Python (no numpy/scipy).
"""
import math, random

FS = 3000.0
DUR = 10.0
N = int(DUR * FS)
RIPPLE = (150.0, 250.0)     # ripple band (event carrier 175 Hz)
GAMMA  = (60.0, 140.0)      # high-gamma band (event carrier 100 Hz)
EVENT_MS = 100.0
LP_TAPS = 33                # the lab's 50 Hz envelope smoother
LP_HZ = 50.0


# ----------------------------------------------------------------- FIR design
def fir_bandpass(taps, f1, f2, fs):
    M = taps - 1
    c1, c2 = f1 / fs, f2 / fs
    h = []
    for n in range(taps):
        x = n - M / 2.0
        v = (2 * (c2 - c1) if abs(x) < 1e-9 else
             (math.sin(2 * math.pi * c2 * x) - math.sin(2 * math.pi * c1 * x)) / (math.pi * x))
        h.append(v * (0.54 - 0.46 * math.cos(2 * math.pi * n / M)))   # Hamming
    return h

def fir_lowpass(taps, fc, fs):
    M = taps - 1; c = fc / fs; h = []
    for n in range(taps):
        x = n - M / 2.0
        v = 2 * c if abs(x) < 1e-9 else math.sin(2 * math.pi * c * x) / (math.pi * x)
        h.append(v * (0.54 - 0.46 * math.cos(2 * math.pi * n / M)))
    s = sum(h) or 1.0
    return [v / s for v in h]

def fir_apply(h, x):
    """Causal FIR; output[n] uses x[n-k]. Output aligned to input (group delay
    (len-1)/2 is the lag, which is exactly what we measure)."""
    K = len(h); out = [0.0] * len(x)
    for n in range(len(x)):
        acc = 0.0
        kmax = K if n >= K - 1 else n + 1
        for k in range(kmax):
            acc += h[k] * x[n - k]
        out[n] = acc
    return out


# --------------------------------------------------------- STFT band power
def stft_band_power(x, W, band, fs, hop=1):
    """Causal sliding Hann window; power = sum over the band's bins of |DFT|^2.
    Returns a per-sample trace (hop-held)."""
    k0 = max(1, int(round(band[0] * W / fs)))
    k1 = int(round(band[1] * W / fs))
    win = [0.5 - 0.5 * math.cos(2 * math.pi * n / (W - 1)) for n in range(W)]
    # precompute twiddles for the band bins
    cw = {k: [math.cos(-2 * math.pi * k * n / W) for n in range(W)] for k in range(k0, k1 + 1)}
    sw = {k: [math.sin(-2 * math.pi * k * n / W) for n in range(W)] for k in range(k0, k1 + 1)}
    out = [0.0] * len(x)
    last = 0.0
    for t in range(len(x)):
        if t >= W - 1 and (t % hop) == 0:
            base = t - W + 1
            p = 0.0
            for k in range(k0, k1 + 1):
                re = im = 0.0
                ck, sk = cw[k], sw[k]
                for n in range(W):
                    wx = win[n] * x[base + n]
                    re += wx * ck[n]; im += wx * sk[n]
                p += re * re + im * im
            last = p
        out[t] = last
    return out


# ------------------------------------------------------------- synthetic data
def synth():
    rng = random.Random(11)
    x = [rng.gauss(0, 1) for _ in range(N)]
    for i in range(1, N):
        x[i] = 0.8 * x[i - 1] + 0.6 * x[i]           # mild 1/f coloring
    half = int(EVENT_MS * FS / 1000 / 2)
    sig = max(1e-9, (sum(v * v for v in x) / N) ** 0.5)
    true_rip = [0.0] * N                              # ground-truth ripple envelope
    ev = []                                           # (center, carrier_hz, is_ripple)
    margin = int(0.3 * FS)
    for i in range(20):
        c = rng.randint(margin, N - margin)
        is_rip = (i % 2 == 0)
        f = 175.0 if is_rip else 100.0
        amp = 6.0 * sig
        for j in range(-half, half):
            g = math.exp(-0.5 * (j / (half / 2.0)) ** 2)
            x[c + j] += amp * g * math.sin(2 * math.pi * f * (c + j) / FS)
            if is_rip:
                true_rip[c + j] += g
        ev.append((c, f, is_rip))
    return x, true_rip, ev, half


def xcorr_lag_ms(est, ref, max_lag):
    """Lag (samples) that best aligns est behind ref (est[t] ~ ref[t-lag])."""
    best_lag, best = 0, -1e30
    n = len(ref)
    for lag in range(0, max_lag + 1):
        s = 0.0
        for t in range(lag, n):
            s += est[t] * ref[t - lag]
        if s > best:
            best, best_lag = s, lag
    return best_lag * 1000.0 / FS


def event_contrast(rip_pow, gam_pow, ev, half):
    """During ripple events: mean ripple-band power / mean gamma-band power (dB)."""
    rs = gs = 0.0; cnt = 0
    for (c, f, is_rip) in ev:
        if not is_rip:
            continue
        for t in range(c - half // 2, c + half // 2):   # near the peak
            rs += rip_pow[t]; gs += gam_pow[t]; cnt += 1
    return 10.0 * math.log10((rs / cnt) / max(gs / cnt, 1e-30))


def main():
    x, true_rip, ev, half = synth()
    max_lag = int(0.06 * FS)            # search up to 60 ms
    lp = fir_lowpass(LP_TAPS, LP_HZ, FS)
    print(f"3 kHz | 100 ms events @175 Hz (ripple) / 100 Hz (gamma) | "
          f"ripple band {RIPPLE}, gamma band {GAMMA}")
    print(f"{'method':<22}{'len':>10}{'latency_ms':>12}{'separation_dB':>14}")
    print("-" * 58)

    # ---- FIR filter method (the lab setup), sweep bandpass length ----
    for taps in (30, 64, 128):
        rip = fir_apply(lp, [abs(v) for v in fir_apply(fir_bandpass(taps, *RIPPLE, FS), x)])
        gam = fir_apply(lp, [abs(v) for v in fir_apply(fir_bandpass(taps, *GAMMA, FS), x)])
        lat = xcorr_lag_ms(rip, true_rip, max_lag)
        sep = event_contrast(rip, gam, ev, half)
        gd = ((taps - 1) / 2 + (LP_TAPS - 1) / 2) / FS * 1000
        print(f"{'FIR BP+50Hz LP':<22}{f'{taps}+{LP_TAPS}t':>10}{lat:>12.1f}{sep:>14.1f}"
              f"   (GD={gd:.1f}ms)")

    # ---- STFT method, sweep window length ----
    for W in (32, 64, 128):
        rip = stft_band_power(x, W, RIPPLE, FS)
        gam = stft_band_power(x, W, GAMMA, FS)
        lat = xcorr_lag_ms(rip, true_rip, max_lag)
        sep = event_contrast(rip, gam, ev, half)
        print(f"{'STFT (Hann)':<22}{f'W={W}({W/FS*1000:.0f}ms)':>10}{lat:>12.1f}{sep:>14.1f}"
              f"   (W/2={W/2/FS*1000:.1f}ms)")


if __name__ == "__main__":
    main()
