#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# SPDX-FileCopyrightText: 2025-2026 Caleb Kemere, Reet Sinha, Allen Mikhailov, Rice University
"""Stimulus and expected results for lfp_poly_dec5_tb.

Models the hardware's integer arithmetic exactly (same coefficients, same
accumulator, same round-and-saturate) so the RTL is checked without a tolerance
-- a tolerance would hide a truncated product, a wrong output shift, or a lost
sign, which are the failures that actually occur here.

The expected file is indexed BY CHANNEL, not by emission order: stage 2 computes
several lanes at once, so it emits out of wire order on purpose. The testbench
places each output by its out_channel and compares, which checks the channel
tagging as well as the arithmetic.

Writes, next to this script:
    lfp_poly_samples.hex  one 18-bit stage-1 sample per line, in stage-1
                          emission order (lane-major, then slot), N_FRAMES frames
    lfp_poly_exp.hex      expected 16-bit outputs, [frame][channel]

Run:  python3 gen_lfp_poly_vectors.py
"""
import os

N_LANES, N_SLOTS = 8, 32
IN_W, COEF_W, COEF_FRAC, OUT_W = 18, 18, 17, 16
N_TAPS, DECIM = 120, 5
# MUST exceed N_TAPS: with fewer input frames than taps, the tail of the delay
# line is still zero and those coefficients are never exercised -- a filter bug
# in taps 40..119 would pass unnoticed. 150 frames fully populates the history
# well before the run ends.
N_FRAMES_IN = 150                    # -> 30 output frames, history full from frame 119

OUT_SHIFT = COEF_FRAC + (IN_W - OUT_W)        # 19
OUT_MAX, OUT_MIN = (1 << (OUT_W - 1)) - 1, -(1 << (OUT_W - 1))
N_CH = N_LANES * N_SLOTS
HERE = os.path.dirname(os.path.abspath(__file__))


def s(v, w):
    v &= (1 << w) - 1
    return v - (1 << w) if v >> (w - 1) else v


def load_coefs(path, n):
    with open(path) as fh:
        c = [s(int(l, 16), COEF_W) for l in fh if l.strip()]
    assert len(c) == n, f"{path}: expected {n} coefficients, got {len(c)}"
    return c


def sample(frame, ch):
    """Distinct bounded waveform per channel, kept well clear of saturation."""
    v = (frame * 2246822519 + ch * 374761393) & 0x3FFFF
    return s(v, IN_W) // 4


def main():
    coefs = load_coefs(os.path.join(HERE, "lfp_poly120_lin_coefs.hex"), N_TAPS)

    hist = [[0] * N_TAPS for _ in range(N_CH)]
    samples, expected = [], []

    for fr in range(N_FRAMES_IN):
        for ch in range(N_CH):                 # stage-1 emission order
            x = sample(fr, ch)
            samples.append(x)
            hist[ch] = [x] + hist[ch][:N_TAPS - 1]

        if fr % DECIM == DECIM - 1:            # the engine fires every DECIM frames
            for ch in range(N_CH):
                acc = sum(hist[ch][t] * coefs[t] for t in range(N_TAPS))
                r = (acc + (1 << (OUT_SHIFT - 1))) >> OUT_SHIFT
                expected.append(max(OUT_MIN, min(OUT_MAX, r)))

    with open(os.path.join(HERE, "lfp_poly_samples.hex"), "w") as fh:
        for v in samples:
            fh.write(f"{v & ((1 << IN_W) - 1):05x}\n")
    with open(os.path.join(HERE, "lfp_poly_exp.hex"), "w") as fh:
        for v in expected:
            fh.write(f"{v & ((1 << OUT_W) - 1):04x}\n")

    print(f"{len(samples)} input samples ({N_FRAMES_IN} frames x {N_CH} channels)")
    print(f"{len(expected)} expected outputs "
          f"({N_FRAMES_IN // DECIM} frames x {N_CH} channels)")
    print(f"output range {min(expected)} .. {max(expected)} "
          f"(saturation limits {OUT_MIN} .. {OUT_MAX})")


if __name__ == "__main__":
    main()
