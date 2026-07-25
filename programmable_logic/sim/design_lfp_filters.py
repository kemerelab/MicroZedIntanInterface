#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# SPDX-FileCopyrightText: 2025-2026 Caleb Kemere, Reet Sinha, Allen Mikhailov, Rice University
"""Design the two-stage LFP decimation chain and emit its fixed-point coefficients.

    30 kHz --[stage 1: 11-tap halfband, /2]--> 15 kHz --[stage 2: 120-tap, /5]--> 3 kHz

Stage 2 ships in two flavours, selectable at runtime by uploading coefficients:

  * linear phase  -- symmetric taps, exactly constant group delay, ~4 ms latency
  * minimum phase -- asymmetric, same magnitude response, ~4x lower latency

Why two stages: a single /10 filter would need ~245 taps to hit the same
1.2/1.8 kHz transition (the transition is a fixed *fraction* of the sample rate,
so filtering after the /2 buys you the same sharpness for half the taps), and a
245-deep delay line costs ~2x the block RAM.

The number that actually matters is the last column of the summary: WORST-CASE
ALIAS, the largest gain any out-of-band input frequency has after it folds into
the 0-1.2 kHz output band through BOTH decimations. Per-stage stopband figures
flatter you; this one doesn't.

Outputs (written next to this script):
    lfp_hb11_coefs.hex          stage 1, Q1.17
    lfp_poly120_lin_coefs.hex   stage 2 linear phase, Q1.17
    lfp_poly120_min_coefs.hex   stage 2 minimum phase, Q1.17
    lfp_filter_defaults.svh     power-on defaults for the RTL

Run:  python3 design_lfp_filters.py [--plot]
"""

import argparse
import os
import sys

import numpy as np
from scipy import signal

# ---------------------------------------------------------------------------
# Chain specification
# ---------------------------------------------------------------------------
FS_IN   = 30_000.0     # broadband sample rate (one packet per sample)
FS_MID  = 15_000.0     # after stage 1 (/2)
FS_OUT  =  3_000.0     # after stage 2 (/5)

F_PASS  = 1_200.0      # final passband edge -- the band we promise to deliver flat
F_STOP2 = FS_OUT - F_PASS   # 1800 Hz: first frequency that folds back onto F_PASS

# Stage 1: an 11-tap halfband. Its transition is forced symmetric about FS_IN/4,
# so the passband edge picks the stopband edge. We only need the passband flat
# to F_PASS (stage 2 removes everything above 1.5 kHz anyway), so we spend the
# freedom on a WIDE transition, which is what buys attenuation at 11 taps.
HB_TAPS = 11
# 2 kHz is where this stops being the limiting stage. The frequency that folds
# exactly onto the passband edge is FS_MID - F_PASS = 13.8 kHz, and stage 2 has
# full gain there, so the cascade can be no cleaner than stage 1 is at 13.8 kHz.
# Widening the transition (lowering HB_FP) buys attenuation for free until stage 2
# becomes the limit:  3.0 kHz -> 63 dB,  2.5 -> 72 dB,  2.0 -> 82 dB (stage 2's
# own stopband), 1.5 -> 82 dB (no further gain). Passband droop at 1.2 kHz over
# that whole range stays under 0.01 dB, so 2 kHz costs nothing worth having.
HB_FP   = 2_000.0                 # passband edge
HB_FST  = FS_IN / 2 - HB_FP       # 13 kHz, forced by halfband symmetry

# Stage 2: 120 taps = 5 phases x 24, so the polyphase decomposition is exact.
POLY_TAPS = 120

# Fixed point: Q1.COEF_FRAC signed, matching the DSP48E1 B port.
COEF_W    = 18
COEF_FRAC = 17
COEF_MAX  = (1 << (COEF_W - 1)) - 1
COEF_MIN  = -(1 << (COEF_W - 1))


# ---------------------------------------------------------------------------
# Design
# ---------------------------------------------------------------------------
def design_halfband(ntaps, fp, fs_rate):
    """Equiripple halfband lowpass.

    An equiripple design whose bands are symmetric about fs/4 with equal ripple
    weights is already a halfband (every second tap either side of centre is
    zero); we snap those to exact zero and pin the centre tap to 0.5 so the RTL
    can implement it as a shift plus a handful of multiplies.
    """
    if (ntaps - 3) % 4 != 0:
        raise ValueError(f"halfband length must be 4k+3, got {ntaps}")
    fst = fs_rate / 2 - fp
    h = signal.remez(ntaps, [0, fp, fst, fs_rate / 2], [1, 0],
                     weight=[1, 1], fs=fs_rate)

    c = (ntaps - 1) // 2
    for i in range(ntaps):                      # snap the structural zeros
        if i != c and (i - c) % 2 == 0:
            h[i] = 0.0
    h[c] = 0.5                                  # exact centre -> a shift, not a multiply
    others = [i for i in range(ntaps) if i != c and h[i] != 0.0]
    s = sum(h[i] for i in others)
    for i in others:                            # renormalise to unity DC gain
        h[i] *= 0.5 / s
    return h


def design_poly_linear(ntaps, fp, fst, fs_rate, weight=(1, 12)):
    """Equiripple linear-phase lowpass for the /5 stage."""
    return signal.remez(ntaps, [0, fp, fst, fs_rate / 2], [1, 0],
                        weight=list(weight), fs=fs_rate)


def design_poly_minphase(ntaps, fp, fst, fs_rate):
    """Minimum-phase lowpass with the same magnitude response as the linear one.

    Built by spectral factorisation: design a linear-phase prototype of length
    2*ntaps-1 with DOUBLE the stopband attenuation in dB, then take its
    minimum-phase factor -- the factor's magnitude is the square root of the
    prototype's, so the doubled spec comes back to the target.

    Same |H(f)|, but the energy is front-loaded instead of centred, which is
    where the latency saving comes from.
    """
    proto_len = 2 * ntaps - 1
    # Push the prototype hard: it needs ~2x the dB we ultimately want.
    proto = signal.remez(proto_len, [0, fp, fst, fs_rate / 2], [1, 0],
                         weight=[1, 200], fs=fs_rate)
    try:
        h = signal.minimum_phase(proto, method='homomorphic')
    except TypeError:                            # older scipy: no method kwarg
        h = signal.minimum_phase(proto)
    if len(h) != ntaps:                          # scipy version differences
        h = np.resize(h, ntaps)
    return h / h.sum()                           # unity DC gain


# ---------------------------------------------------------------------------
# Fixed point
# ---------------------------------------------------------------------------
def quantize(h):
    """Round to Q1.COEF_FRAC and return (integer taps, the values they represent)."""
    q = np.round(np.asarray(h) * (1 << COEF_FRAC)).astype(np.int64)
    if np.any(q > COEF_MAX) or np.any(q < COEF_MIN):
        raise ValueError(f"coefficient overflows Q1.{COEF_FRAC}: "
                         f"peak {np.max(np.abs(q))} > {COEF_MAX}")
    return q, q.astype(float) / (1 << COEF_FRAC)


# ---------------------------------------------------------------------------
# Measurement
# ---------------------------------------------------------------------------
def response(h, fs_rate, freqs):
    """|H(f)| at the given absolute frequencies."""
    w = 2 * np.pi * np.asarray(freqs) / fs_rate
    _, H = signal.freqz(h, worN=w)
    return np.abs(H)


def passband_ripple_db(h, fs_rate, f_hi=F_PASS, n=2048):
    f = np.linspace(0, f_hi, n)
    m = response(h, fs_rate, f)
    return 20 * np.log10(m.max() / m.min())


def fold(f, rate):
    """Where absolute frequency f lands after sampling at `rate` (Hz)."""
    f = np.asarray(f, dtype=float) % rate
    return np.where(f > rate / 2, rate - f, f)


def worst_case_alias_db(h1, h2, n=200_001):
    """End-to-end alias rejection of the whole cascade.

    Sweep every input frequency the 30 kHz stream can carry, push it through
    BOTH filters and BOTH foldings, and find the loudest thing that ends up
    inside the 0-1.2 kHz output band without belonging there.

    Returns (attenuation_dB, offending_input_frequency_Hz).
    """
    f = np.linspace(0, FS_IN / 2, n)
    g1 = response(h1, FS_IN, f)              # stage 1, at the input rate
    f_mid = fold(f, FS_MID)                  # /2 decimation folds the spectrum
    g2 = response(h2, FS_MID, f_mid)         # stage 2 sees the folded frequency
    f_out = fold(f_mid, FS_OUT)              # /5 decimation folds again
    gain = g1 * g2

    # "Lands in the output passband but isn't the signal we asked for."
    contaminates = (f_out <= F_PASS) & (f > F_PASS)
    if not np.any(contaminates):
        return float('inf'), None
    i = int(np.argmax(np.where(contaminates, gain, 0.0)))
    return -20 * np.log10(gain[i]), f[i]


def latency_ms(h1, h2):
    """Passband group delay of the cascade, in milliseconds.

    Stage 1 runs at 30 kHz, stage 2 at 15 kHz, so their sample counts weigh
    differently -- a stage-2 tap costs twice the wall-clock of a stage-1 tap.
    """
    def gd(h, fs_rate):
        f = np.linspace(0, F_PASS, 512)
        w = 2 * np.pi * f / fs_rate
        _, d = signal.group_delay((h, 1), w=w)
        return float(np.mean(d))
    return gd(h1, FS_IN) / FS_IN * 1e3 + gd(h2, FS_MID) / FS_MID * 1e3


# ---------------------------------------------------------------------------
# Emit
# ---------------------------------------------------------------------------
def cascade_phase_and_delay(h1, h2, freqs):
    """Unwrapped phase (rad) and group delay (ms) of the whole chain vs input frequency.

    The two stages run at different rates, so their contributions weigh
    differently: a delay of one stage-2 sample is two stage-1 samples of
    wall-clock. Phases add; delays add once converted to seconds.
    """
    f = np.asarray(freqs, dtype=float)
    w1, w2 = 2 * np.pi * f / FS_IN, 2 * np.pi * f / FS_MID
    _, H1 = signal.freqz(h1, worN=w1)
    _, H2 = signal.freqz(h2, worN=w2)
    phase = np.unwrap(np.angle(H1)) + np.unwrap(np.angle(H2))
    _, g1 = signal.group_delay((h1, 1), w=w1)
    _, g2 = signal.group_delay((h2, 1), w=w2)
    delay_ms = (g1 / FS_IN + g2 / FS_MID) * 1e3
    return phase, delay_ms


def write_hex(path, q):
    """One two's-complement COEF_W-bit value per line, for $readmemh."""
    mask = (1 << COEF_W) - 1
    digits = (COEF_W + 3) // 4
    with open(path, 'w') as fh:
        for v in q:
            fh.write(f"{int(v) & mask:0{digits}x}\n")


def write_sv_defaults(path, hb_q, lin_q):
    """Emit the power-on defaults as a package the RTL imports.

    A package rather than an `include: it compiles like any other source file,
    so neither xsim nor Vivado needs an include path configured, and it matches
    how acq_frame_pkg / unified_pkt_pkg are already consumed.
    """
    def arr(q):
        mask = (1 << COEF_W) - 1
        return ",\n        ".join(
            ", ".join(f"{COEF_W}'h{int(v) & mask:05x}" for v in q[i:i + 6])
            for i in range(0, len(q), 6))

    with open(path, 'w') as fh:
        fh.write(f"""// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2025-2026 Caleb Kemere, Reet Sinha, Allen Mikhailov, Rice University

// lfp_coef_pkg.sv
//
// GENERATED by programmable_logic/sim/design_lfp_filters.py -- do not hand-edit.
// Re-run that script to change the filters; it also reports the passband ripple,
// the end-to-end alias rejection and the latency each design actually achieves.
//
// Power-on defaults for the LFP decimation chain, Q1.{COEF_FRAC} signed. These are
// what the board filters with straight out of reset; the host can overwrite
// either stage at runtime without a rebuild.
//
//   stage 1: {len(hb_q)}-tap halfband,          30 kHz -> 15 kHz
//   stage 2: {len(lin_q)}-tap linear phase,     15 kHz -> 3 kHz, passband to {F_PASS:.0f} Hz
//
// Stage 1 is a halfband, so {sum(1 for v in hb_q if v == 0)} of its {len(hb_q)} taps are structurally zero and its
// centre tap is exactly 0.5. The engine still MACs all {len(hb_q)}: the zeros cost idle
// cycles the budget can spare, and skipping them would silently mis-compute any
// non-halfband coefficients a user uploads.
//
// A minimum-phase stage 2 with the same magnitude response but ~4.6x less
// latency is in lfp_poly120_min_coefs.hex -- upload it when latency matters
// more than constant group delay.

package lfp_coef_pkg;

    localparam int LFP_COEF_W    = {COEF_W};
    localparam int LFP_COEF_FRAC = {COEF_FRAC};
    localparam int LFP_HB_TAPS   = {len(hb_q)};
    localparam int LFP_POLY_TAPS = {len(lin_q)};

    localparam logic [{COEF_W - 1}:0] LFP_HB_DEFAULT_COEF [0:{len(hb_q) - 1}] = '{{
        {arr(hb_q)}
    }};

    localparam logic [{COEF_W - 1}:0] LFP_POLY_DEFAULT_COEF [0:{len(lin_q) - 1}] = '{{
        {arr(lin_q)}
    }};

endpackage
""")


# ---------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--plot', action='store_true', help='save a response plot')
    ap.add_argument('--outdir', default=os.path.dirname(os.path.abspath(__file__)))
    args = ap.parse_args()

    print(f"LFP decimation chain: {FS_IN/1e3:.0f} kHz --/2--> {FS_MID/1e3:.0f} kHz "
          f"--/5--> {FS_OUT/1e3:.0f} kHz,  passband 0-{F_PASS:.0f} Hz\n")

    hb = design_halfband(HB_TAPS, HB_FP, FS_IN)
    lin = design_poly_linear(POLY_TAPS, F_PASS, F_STOP2, FS_MID)
    mp = design_poly_minphase(POLY_TAPS, F_PASS, F_STOP2, FS_MID)

    hb_q, hb_f = quantize(hb)
    lin_q, lin_f = quantize(lin)
    mp_q, mp_f = quantize(mp)

    nz = int(np.count_nonzero(hb_q))
    print(f"stage 1  {HB_TAPS}-tap halfband, passband edge {HB_FP/1e3:.1f} kHz, "
          f"stopband from {HB_FST/1e3:.1f} kHz")
    print(f"         {nz} non-zero taps, centre = {hb[(HB_TAPS-1)//2]:.4f} "
          f"(a shift), {(nz - 1)//2} unique multiplies with symmetry\n")

    rows = []
    for name, h_f in (("linear phase", lin_f), ("minimum phase", mp_f)):
        atten, f_bad = worst_case_alias_db(hb_f, h_f)
        rows.append((name,
                     passband_ripple_db(hb_f, FS_IN) + passband_ripple_db(h_f, FS_MID),
                     latency_ms(hb_f, h_f),
                     atten, f_bad))

    print(f"{'stage 2':<15}{'ripple 0-1.2k':>15}{'latency':>12}"
          f"{'WORST-CASE ALIAS':>20}{'  (worst input)':<16}")
    print("-" * 78)
    for name, rip, lat, atten, f_bad in rows:
        bad = f"  ({f_bad/1e3:.2f} kHz)" if f_bad else ""
        print(f"{name:<15}{rip:>13.3f} dB{lat:>9.2f} ms{atten:>17.1f} dB{bad:<16}")

    speedup = rows[0][2] / rows[1][2] if rows[1][2] > 0 else float('nan')
    print(f"\nminimum phase cuts latency {speedup:.1f}x "
          f"({rows[0][2]:.2f} ms -> {rows[1][2]:.2f} ms) for the same magnitude response.")

    # Confirm the spectral factorisation actually produced a minimum-phase filter.
    # An equiripple stopband has zeros exactly ON the unit circle, and those stay
    # there under factorisation -- so the test is not "all zeros strictly inside"
    # but "no zeros meaningfully outside", vs the linear-phase filter whose
    # mirror-image zeros sit well outside.
    print()
    for name, h_f in (("linear phase ", lin_f), ("minimum phase", mp_f)):
        r = np.abs(np.roots(h_f))
        stop = np.linspace(F_STOP2, FS_MID / 2, 4000)
        print(f"  {name}: stopband {-20*np.log10(response(h_f, FS_MID, stop).max()):5.1f} dB"
              f" | max|zero| {r.max():.3f}"
              f" | {int(np.sum(r > 1.01)):3d} zeros outside the unit circle")

    os.makedirs(args.outdir, exist_ok=True)
    outs = [("lfp_hb11_coefs.hex", hb_q),
            ("lfp_poly120_lin_coefs.hex", lin_q),
            ("lfp_poly120_min_coefs.hex", mp_q)]
    for fn, q in outs:
        write_hex(os.path.join(args.outdir, fn), q)
    # The RTL defaults are a package, so they live with the sources, not the sims.
    pkg = os.path.abspath(os.path.join(args.outdir, "..", "src", "lfp_coef_pkg.sv"))
    write_sv_defaults(pkg, hb_q, lin_q)
    print("\nwrote " + ", ".join(fn for fn, _ in outs))
    print(f"wrote {os.path.relpath(pkg, args.outdir)}")

    if args.plot:
        try:
            import matplotlib
            matplotlib.use("Agg")
            import matplotlib.pyplot as plt
        except ImportError:
            print("matplotlib not available -- skipping plot", file=sys.stderr)
            return
        f = np.linspace(0, FS_IN / 2, 20001)
        fig, ax = plt.subplots(2, 1, figsize=(10, 8))
        ax[0].plot(f / 1e3, 20 * np.log10(response(hb_f, FS_IN, f) + 1e-12),
                   label=f"stage 1 ({HB_TAPS}-tap halfband)")
        for name, h_f in (("stage 2 linear", lin_f), ("stage 2 min phase", mp_f)):
            g = response(h_f, FS_MID, fold(f, FS_MID))
            ax[0].plot(f / 1e3, 20 * np.log10(g + 1e-12), label=name, alpha=.8)
        ax[0].set(title="Per-stage magnitude vs input frequency",
                  ylabel="dB", ylim=(-140, 5), xlim=(0, FS_IN / 2e3))
        ax[0].legend(); ax[0].grid(alpha=.3)

        for name, h_f in (("linear phase", lin_f), ("minimum phase", mp_f)):
            tot = response(hb_f, FS_IN, f) * response(h_f, FS_MID, fold(f, FS_MID))
            ax[1].plot(f / 1e3, 20 * np.log10(tot + 1e-12), label=f"cascade, {name}")
        ax[1].axvline(F_PASS / 1e3, color='k', ls=':', lw=1)
        ax[1].set(title="Cascade response (dotted = 1.2 kHz passband edge)",
                  xlabel="input frequency (kHz)", ylabel="dB",
                  ylim=(-140, 5), xlim=(0, 8))
        ax[1].legend(); ax[1].grid(alpha=.3)
        fig.tight_layout()
        p = os.path.join(args.outdir, "lfp_filter_response.png")
        fig.savefig(p, dpi=110)
        print(f"wrote {p}")

        # ---- phase / group delay / impulse response -------------------------
        fp_ = np.linspace(1.0, 1500.0, 3000)      # passband + a little past the edge
        fig2, bx = plt.subplots(3, 1, figsize=(10, 11))
        colours = {"linear phase": "tab:blue", "minimum phase": "tab:red"}

        for name, h_f in (("linear phase", lin_f), ("minimum phase", mp_f)):
            n = np.arange(len(h_f)) / FS_MID * 1e3
            bx[0].plot(n, h_f, label=f"stage 2, {name}", color=colours[name], lw=1.2)
        bx[0].axhline(0, color='k', lw=.5)
        bx[0].set(title="Stage-2 impulse response — minimum phase front-loads its energy, "
                        "which is where the latency saving comes from",
                  xlabel="time (ms)", ylabel="tap value")
        bx[0].legend(); bx[0].grid(alpha=.3)

        for name, h_f in (("linear phase", lin_f), ("minimum phase", mp_f)):
            ph, gd = cascade_phase_and_delay(hb_f, h_f, fp_)
            bx[1].plot(fp_ / 1e3, np.degrees(ph), color=colours[name],
                       label=f"cascade, {name}")
            bx[2].plot(fp_ / 1e3, gd, color=colours[name],
                       label=f"cascade, {name}")
        for a in (bx[1], bx[2]):
            a.axvline(F_PASS / 1e3, color='k', ls=':', lw=1)
            a.set_xlim(0, 1.5); a.legend(); a.grid(alpha=.3)
        bx[1].set(title="Cascade phase (dotted = 1.2 kHz passband edge) — "
                        "straight line = linear phase, curved = minimum phase",
                  ylabel="phase (degrees)")
        bx[2].set(title="Cascade group delay — the latency you actually pay, per frequency",
                  xlabel="frequency (kHz)", ylabel="group delay (ms)")
        bx[2].set_ylim(bottom=0)
        fig2.tight_layout()
        p2 = os.path.join(args.outdir, "lfp_filter_phase.png")
        fig2.savefig(p2, dpi=110)
        print(f"wrote {p2}")

        # Quantify the phase-distortion cost of choosing minimum phase.
        _, gd_lin = cascade_phase_and_delay(hb_f, lin_f, np.linspace(1, F_PASS, 500))
        _, gd_min = cascade_phase_and_delay(hb_f, mp_f, np.linspace(1, F_PASS, 500))
        print(f"\ngroup delay across the 0-{F_PASS:.0f} Hz passband:")
        print(f"  linear phase : {gd_lin.min():.2f} - {gd_lin.max():.2f} ms "
              f"(spread {gd_lin.max()-gd_lin.min():.3f} ms -- flat by construction)")
        print(f"  minimum phase: {gd_min.min():.2f} - {gd_min.max():.2f} ms "
              f"(spread {gd_min.max()-gd_min.min():.3f} ms -- dispersion across the band)")


if __name__ == '__main__':
    main()
