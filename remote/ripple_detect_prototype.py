#!/usr/bin/env python3
"""
Causal sharp-wave-ripple detector prototype  --  the algorithm spec for the
Tier-2 soft core (MicroBlaze V).  Pure Python, scalar, sample-by-sample, so it
maps 1:1 onto the on-PL microcode/C loop (and needs no numpy/scipy).

Pipeline per channel, per 2 kHz sample (mirrors docs/lfp-dsp-engine-design.md):
    ripple bandpass (cascade of biquads, SOS)  ->  rectify |.|  ->  envelope (EMA)
      ->  running baseline mean + SD (slow EMA)  ->  threshold (mean + k*SD)
      ->  detection + refractory lockout
Channels run independently; combine at the decision step (not modeled here --
this is the single-channel core).  Filters are designed with the closed-form RBJ
"constant 0 dB peak gain" bandpass biquad so the coefficients are reproducible in
C and quantizable to Q1.17 for the FIR/biquad engine.

Run:  python3 ripple_detect_prototype.py
It builds a synthetic LFP with injected ripples, runs the causal detector, scores
detections vs ground truth, and writes ripple_envelope.csv for plotting.
"""
import math, random

FS = 2000.0                 # Tier-2 runs on the 2 kHz LFP stream


# ---------------------------------------------------------------- filter design
def rbj_bandpass(f0, Q, fs):
    """RBJ Audio-EQ-Cookbook bandpass (constant 0 dB peak gain), normalized a0=1.
    Returns (b0,b1,b2,a1,a2)."""
    w0 = 2.0 * math.pi * f0 / fs
    alpha = math.sin(w0) / (2.0 * Q)
    cw = math.cos(w0)
    a0 = 1.0 + alpha
    return (alpha / a0, 0.0, -alpha / a0, (-2.0 * cw) / a0, (1.0 - alpha) / a0)


def ripple_sos(fs, lo=150.0, hi=250.0, n_sections=2):
    """Cascade of identical RBJ bandpass sections centered on the geometric mean
    of [lo,hi] with Q = f0/bandwidth.  Cascading sharpens the skirts."""
    f0 = math.sqrt(lo * hi)
    Q = f0 / (hi - lo)
    return [rbj_bandpass(f0, Q, fs) for _ in range(n_sections)], f0, Q


# ----------------------------------------------------------------- the detector
class Biquad:
    """Direct Form I -- the robust fixed-point form; 4 state words per section."""
    __slots__ = ("b0", "b1", "b2", "a1", "a2", "x1", "x2", "y1", "y2")

    def __init__(self, c):
        self.b0, self.b1, self.b2, self.a1, self.a2 = c
        self.x1 = self.x2 = self.y1 = self.y2 = 0.0

    def step(self, x):
        y = (self.b0 * x + self.b1 * self.x1 + self.b2 * self.x2
             - self.a1 * self.y1 - self.a2 * self.y2)
        self.x2, self.x1 = self.x1, x
        self.y2, self.y1 = self.y1, y
        return y


class RippleDetector:
    def __init__(self, fs=FS, lo=150.0, hi=250.0, n_sections=2,
                 tau_env=0.008, tau_stat=2.0, k=5.0, lockout_s=0.15,
                 freeze_baseline=True):
        sos, self.f0, self.Q = ripple_sos(fs, lo, hi, n_sections)
        self.sos = sos
        self.biquads = [Biquad(c) for c in sos]
        self.a_env = 1.0 - math.exp(-1.0 / (tau_env * fs))     # fast envelope EMA
        self.a_stat = 1.0 - math.exp(-1.0 / (tau_stat * fs))   # slow baseline EMA
        self.k = k
        self.lockout_reload = int(round(lockout_s * fs))
        self.freeze_baseline = freeze_baseline
        # state
        self.env = 0.0
        self.mean = 0.0
        self.var = 1.0
        self.lockout = 0

    def step(self, x):
        # 1) ripple bandpass (cascade)
        b = x
        for bq in self.biquads:
            b = bq.step(b)
        # 2) rectify + 3) envelope (one-pole EMA)
        self.env += self.a_env * (abs(b) - self.env)
        # 4) running baseline mean + variance (slow EMA), optionally frozen
        #    while a detection is active so ripples don't inflate their own bar
        active = self.env > (self.mean + self.k * math.sqrt(self.var))
        if not (self.freeze_baseline and active):
            d = self.env - self.mean
            self.mean += self.a_stat * d
            self.var += self.a_stat * (d * d - self.var)
        # 5) threshold + 6) refractory lockout
        thr = self.mean + self.k * math.sqrt(self.var)
        fire = (self.env > thr) and (self.lockout == 0)
        if fire:
            self.lockout = self.lockout_reload
        elif self.lockout > 0:
            self.lockout -= 1
        return b, self.env, thr, fire


# ------------------------------------------------------------- synthetic signal
def synth_lfp(fs=FS, dur=20.0, n_ripples=15, seed=7):
    rng = random.Random(seed)
    n = int(dur * fs)
    x = [rng.gauss(0.0, 1.0) for _ in range(n)]          # baseline (unit SD noise)
    # a little 1/f-ish coloring so it reads like LFP, not white
    for i in range(1, n):
        x[i] = 0.85 * x[i - 1] + 0.55 * x[i]
    truth = []                                            # (start, end) sample idx
    margin = int(0.4 * fs)
    for _ in range(n_ripples):
        c = rng.randint(margin, n - margin)
        dur_s = rng.uniform(0.05, 0.09)                  # 50-90 ms event
        half = int(dur_s * fs / 2)
        f = rng.uniform(160.0, 240.0)
        amp = rng.uniform(6.0, 12.0)                     # several x baseline SD
        for j in range(-half, half):
            w = math.exp(-0.5 * (j / (half / 2.0)) ** 2)  # Gaussian envelope
            x[c + j] += amp * w * math.sin(2 * math.pi * f * j / fs)
        truth.append((c - half, c + half))
    return x, sorted(truth)


# ------------------------------------------------------------------------- main
def main():
    fs = FS
    x, truth = synth_lfp(fs=fs, dur=20.0, n_ripples=15)
    det = RippleDetector(fs=fs)
    print(f"ripple band f0={det.f0:.1f} Hz  Q={det.Q:.2f}  sections={len(det.sos)}  "
          f"a_env={det.a_env:.4f}  a_stat={det.a_stat:.5f}  k={det.k}")
    print("SOS (b0,b1,b2,a1,a2), normalized a0=1 -> quantize to Q1.17 for upload:")
    for c in det.sos:
        print("   " + "  ".join(f"{v:+.6f}" for v in c))

    env_trace, fires = [], []
    for n, xn in enumerate(x):
        _, env, thr, fire = det.step(xn)
        env_trace.append((n / fs, xn, env, thr))
        if fire:
            fires.append(n)

    # score: a detection "hits" if it lands inside (or just after) a truth window
    hits, used = 0, [False] * len(truth)
    win = int(0.12 * fs)
    for f in fires:
        for i, (a, b) in enumerate(truth):
            if not used[i] and a - win <= f <= b + win:
                used[i] = True; hits += 1; break
    fp = len(fires) - hits
    base_env = sorted(e for (_, _, e, _) in env_trace)[len(env_trace) // 2]
    peak_env = max(e for (_, _, e, _) in env_trace)

    with open("ripple_envelope.csv", "w") as fcsv:
        fcsv.write("t,raw,envelope,threshold\n")
        for t, raw, env, thr in env_trace:
            fcsv.write(f"{t:.5f},{raw:.4f},{env:.4f},{thr:.4f}\n")

    print(f"\ninjected ripples : {len(truth)}")
    print(f"detected (hits)  : {hits}/{len(truth)}")
    print(f"false positives  : {fp}")
    print(f"envelope baseline~{base_env:.3f}  peak~{peak_env:.3f}  "
          f"(ripple/baseline ~{peak_env / max(base_env,1e-9):.1f}x)")
    print("wrote ripple_envelope.csv")

    ok = (hits >= len(truth) - 1) and (fp <= 2) and (peak_env > 4 * base_env)
    print("RESULT:", "PASS  (envelope tracks ripples, detector fires cleanly)"
          if ok else "FAIL  (tune bands/k/tau)")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
