#!/usr/bin/env python3
# Bit-exact reference for cic_decimator.sv: a CIC of order N_ORDER, decimation R,
# differential delay M=1, per channel. Integrators run at input rate, combs at
# output rate. Fixed-point: integrators/combs are modular (two's complement) at
# ACC_W bits -- the classic CIC property that wraparound is exact as long as
# ACC_W >= input_W + ceil(N_ORDER*log2(R*M)). Output is the comb result
# right-shifted to OUT_W (gain normalization) then saturated.
import os

OUT = os.path.dirname(os.path.abspath(__file__))

N_LANES   = 8
N_SLOTS   = 32
DATA_W    = 16
R         = 5
N_ORDER   = 4
M         = 1
ACC_W     = 32     # >= 16 + ceil(4*log2(5)) = 16 + 10 = 26; 32 is comfy
OUT_W     = 16
K_PACKETS = 120    # input frames (30 kHz ticks)
LANE_MASK = 0b1010_0101

# CIC DC gain = (R*M)^N_ORDER. To map back to ~unity and OUT_W, shift right by
# ceil(N_ORDER*log2(R*M)). For R=5,M=1,N=4: gain=625, log2(625)=9.28 -> shift 10
# (gain 1024) keeps headroom and never overflows OUT_W for full-scale input/2.
import math
GAIN_SHIFT = math.ceil(N_ORDER * math.log2(R * M))   # = 10

_state = 0xBEEF1234
def rnd(lo, hi):
    global _state
    _state = (_state * 6364136223846793005 + 1442695040888963407) & ((1 << 64) - 1)
    return lo + (_state >> 17) % (hi - lo + 1)

def to_hex(v, bits):
    return format(v & ((1 << bits) - 1), 'x').zfill((bits + 3) // 4)

ACC_MASK = (1 << ACC_W) - 1
def sx(v, w):  # sign-extend w-bit two's complement
    v &= (1 << w) - 1
    return v - (1 << w) if (v >> (w - 1)) & 1 else v

# input samples [packet][slot][lane], signed
samp = [[[rnd(-20000, 20000) for _ in range(N_LANES)] for _ in range(N_SLOTS)]
        for _ in range(K_PACKETS)]

with open(f"{OUT}/cic_samples.hex", "w") as f:
    for p in range(K_PACKETS):
        for s in range(N_SLOTS):
            word = 0
            for l in range(N_LANES):
                word |= (samp[p][s][l] & 0xFFFF) << (16 * l)
            f.write(format(word, 'x').zfill(32) + "\n")

OUT_MAX = (1 << (OUT_W - 1)) - 1
OUT_MIN = -(1 << (OUT_W - 1))
enabled = [l for l in range(N_LANES) if (LANE_MASK >> l) & 1]

# per-channel CIC state: integ[N_ORDER], comb_prev[N_ORDER] (each chain), decim cnt
exp_val, exp_chan = [], []
# emulate the SAME schedule the RTL uses: integrators update every tick; on every
# R-th tick we run the combs on the last integrator's value and emit.
for l in enabled:
    for s in range(N_SLOTS):
        integ = [0] * N_ORDER
        comb_prev = [0] * (N_ORDER + 1)   # comb_prev[0] is the integrator-output reg
        cnt = 0
        for p in range(K_PACKETS):
            x = samp[p][s][l] & ACC_MASK
            # integrator cascade (modular at ACC_W)
            acc = x
            for i in range(N_ORDER):
                integ[i] = (integ[i] + acc) & ACC_MASK
                acc = integ[i]
            cnt += 1
            if cnt == R:
                cnt = 0
                # comb cascade on the current last-integrator value
                stage = integ[N_ORDER - 1]
                for i in range(N_ORDER):
                    diff = (stage - comb_prev[i]) & ACC_MASK
                    comb_prev[i] = stage
                    stage = diff
                # gain-normalize + saturate
                y = sx(stage, ACC_W) >> GAIN_SHIFT
                y = OUT_MAX if y > OUT_MAX else OUT_MIN if y < OUT_MIN else y
                exp_val.append(y)
                exp_chan.append(l * N_SLOTS + s)

# RTL emits in (lane asc, slot asc) per frame, but the reference above is grouped
# per-channel across all frames. Re-order to frame-major to match the collector.
frames = K_PACKETS // R
exp_val_fm, exp_chan_fm = [], []
per_ch = K_PACKETS // R
# exp_val is laid out [channel][frame]; channel index = order of (l,s) in enabled
chlist = [(l, s) for l in enabled for s in range(N_SLOTS)]
for fr in range(frames):
    for ci, (l, s) in enumerate(chlist):
        idx = ci * per_ch + fr
        exp_val_fm.append(exp_val[idx])
        exp_chan_fm.append(l * N_SLOTS + s)

with open(f"{OUT}/cic_exp_val.hex", "w") as f:
    for v in exp_val_fm:
        f.write(to_hex(v, OUT_W) + "\n")
with open(f"{OUT}/cic_exp_chan.hex", "w") as f:
    for c in exp_chan_fm:
        f.write(to_hex(c, 16) + "\n")

print(f"order={N_ORDER} R={R} M={M} ACC_W={ACC_W} GAIN_SHIFT={GAIN_SHIFT} "
      f"packets={K_PACKETS} frames={frames} enabled={enabled} outputs={len(exp_val_fm)}")
