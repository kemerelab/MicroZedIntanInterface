#!/usr/bin/env python3
# Bit-exact reference for lfp_halfband.sv (a generic time-shared /2 decimating
# FIR with host coefficients). Mirrors gen_lfp_fir_vectors.py arithmetic but with
# DECIM_R fixed at 2.
import os
OUT = os.path.dirname(os.path.abspath(__file__))

N_LANES, N_SLOTS = 8, 32
DATA_W, COEF_W, COEF_FRAC, OUT_W = 16, 18, 17, 16
NUM_TAPS  = 23          # odd halfband length
RING_DEPTH = 64
K_PACKETS = 80          # input frames at the /5 (6 kHz) rate
LANE_MASK = 0b0010_0110

_state = 0x5A5A1234
def rnd(lo, hi):
    global _state
    _state = (_state * 6364136223846793005 + 1442695040888963407) & ((1 << 64) - 1)
    return lo + (_state >> 17) % (hi - lo + 1)
def to_hex(v, bits): return format(v & ((1 << bits) - 1), 'x').zfill((bits + 3) // 4)

COEF_MIN, COEF_MAX = -(1 << (COEF_W-1)), (1 << (COEF_W-1)) - 1
coef = [max(COEF_MIN, min(COEF_MAX, rnd(-60000, 60000))) for _ in range(NUM_TAPS)]
samp = [[[rnd(-2000, 2000) for _ in range(N_LANES)] for _ in range(N_SLOTS)]
        for _ in range(K_PACKETS)]

with open(f"{OUT}/hb_coefs.hex", "w") as f:
    for j in range(NUM_TAPS): f.write(to_hex(coef[j], COEF_W) + "\n")
with open(f"{OUT}/hb_samples.hex", "w") as f:
    for p in range(K_PACKETS):
        for s in range(N_SLOTS):
            word = 0
            for l in range(N_LANES): word |= (samp[p][s][l] & 0xFFFF) << (16*l)
            f.write(format(word, 'x').zfill(32) + "\n")

ROUND = 1 << (COEF_FRAC-1)
OUT_MAX, OUT_MIN = (1 << (OUT_W-1)) - 1, -(1 << (OUT_W-1))
enabled = [l for l in range(N_LANES) if (LANE_MASK >> l) & 1]
def s_at(p,s,l): return samp[p][s][l] if p >= 0 else 0

exp_val, exp_chan = [], []
for p in range(K_PACKETS):
    if (p % 2) != 1:   # /2: trigger when decim_phase set (every 2nd tick)
        continue
    for l in enabled:
        for s in range(N_SLOTS):
            acc = sum(coef[j]*s_at(p-j,s,l) for j in range(NUM_TAPS))
            r = (acc + ROUND) >> COEF_FRAC
            r = OUT_MAX if r > OUT_MAX else OUT_MIN if r < OUT_MIN else r
            exp_val.append(r); exp_chan.append(l*N_SLOTS+s)

with open(f"{OUT}/hb_exp_val.hex","w") as f:
    for v in exp_val: f.write(to_hex(v, OUT_W)+"\n")
with open(f"{OUT}/hb_exp_chan.hex","w") as f:
    for c in exp_chan: f.write(to_hex(c, 16)+"\n")
print(f"taps={NUM_TAPS} packets={K_PACKETS} frames={K_PACKETS//2} enabled={enabled} outputs={len(exp_val)}")
