#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# SPDX-FileCopyrightText: 2025-2026 Caleb Kemere, Reet Sinha, Allen Mikhailov, Rice University
"""Stimulus and expected results for lfp_halfband_dec2_tb.

This is the reference the RTL is checked against, so it models the hardware's
integer arithmetic exactly -- same coefficients, same accumulator, same
round-and-saturate -- rather than computing in floating point and allowing a
tolerance. A tolerance would hide precisely the bugs worth catching (a truncated
product, a wrong shift, an unsigned operand).

Writes, next to this script:
    lfp_hb_samples.hex   one 128-bit input word per line: 8 lanes x 16-bit,
                         lane 0 in the low bits; N_PACKETS * N_SLOTS lines
    lfp_hb_exp.hex       expected 18-bit outputs, in the order the engine emits
                         them (per frame: lane 0 slots 0..31, lane 1 ...)

Run:  python3 gen_lfp_hb_vectors.py
"""
import os

N_LANES, N_SLOTS = 8, 32
DATA_W, COEF_W, COEF_FRAC, OUT_W = 16, 18, 17, 18
N_TAPS, DECIM = 11, 2
N_PACKETS = 30                      # -> 15 output frames per channel

OUT_SHIFT = COEF_FRAC - (OUT_W - DATA_W)      # 15: keep 2 fractional bits for stage 2
OUT_MAX, OUT_MIN = (1 << (OUT_W - 1)) - 1, -(1 << (OUT_W - 1))
HERE = os.path.dirname(os.path.abspath(__file__))


def s(v, w):
    """Interpret the low w bits of v as two's complement."""
    v &= (1 << w) - 1
    return v - (1 << w) if v >> (w - 1) else v


def load_coefs(path, n):
    with open(path) as fh:
        c = [s(int(l, 16), COEF_W) for l in fh if l.strip()]
    assert len(c) == n, f"{path}: expected {n} coefficients, got {len(c)}"
    return c


def sample(pkt, lane, slot):
    """A distinct bounded waveform per channel, so a channel mix-up cannot pass."""
    v = (pkt * 2654435761 + lane * 40503 + slot * 97) & 0xFFFF
    return s(v, DATA_W) // 2          # keep clear of saturation


def main():
    coefs = load_coefs(os.path.join(HERE, "lfp_hb11_coefs.hex"), N_TAPS)

    # history[lane][slot] = newest-first list of past samples
    hist = [[[0] * N_TAPS for _ in range(N_SLOTS)] for _ in range(N_LANES)]
    samples, expected = [], []

    for pkt in range(N_PACKETS):
        for slot in range(N_SLOTS):
            word = 0
            for lane in range(N_LANES):
                x = sample(pkt, lane, slot)
                word |= (x & 0xFFFF) << (lane * DATA_W)
                hist[lane][slot] = [x] + hist[lane][slot][:N_TAPS - 1]
            samples.append(word)

        # The engine fires on every DECIM-th packet, once the packet is complete.
        if pkt % DECIM == DECIM - 1:
            for lane in range(N_LANES):
                for slot in range(N_SLOTS):
                    acc = sum(hist[lane][slot][t] * coefs[t] for t in range(N_TAPS))
                    r = (acc + (1 << (OUT_SHIFT - 1))) >> OUT_SHIFT   # arithmetic: Python >> on int
                    expected.append(max(OUT_MIN, min(OUT_MAX, r)))

    with open(os.path.join(HERE, "lfp_hb_samples.hex"), "w") as fh:
        for w in samples:
            fh.write(f"{w:032x}\n")
    with open(os.path.join(HERE, "lfp_hb_exp.hex"), "w") as fh:
        for v in expected:
            fh.write(f"{v & ((1 << OUT_W) - 1):05x}\n")

    print(f"{len(samples)} input words ({N_PACKETS} packets x {N_SLOTS} slots)")
    print(f"{len(expected)} expected outputs "
          f"({N_PACKETS // DECIM} frames x {N_LANES * N_SLOTS} channels)")
    print(f"output range {min(expected)} .. {max(expected)} "
          f"(saturation limits {OUT_MIN} .. {OUT_MAX})")


if __name__ == "__main__":
    main()
