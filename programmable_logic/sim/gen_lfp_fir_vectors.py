#!/usr/bin/env python3
# Generate stimulus + bit-exact expected outputs for lfp_fir_decimator_tb.sv.
# The arithmetic here MUST match the RTL exactly: integer MAC, round-to-nearest
# (add 2^(FRAC-1)) then arithmetic shift down by FRAC, then saturate to int16.
import os

OUT = os.path.dirname(os.path.abspath(__file__))

# ---- config (keep in lockstep with the TB localparams) ----
N_LANES   = 8
N_SLOTS   = 35
DATA_W    = 16
COEF_W    = 18
COEF_FRAC = 17
OUT_W     = 16
NUM_TAPS  = 25
DECIM_R   = 15
K_PACKETS = 300                 # > 256 so the delay-line ring wrap is exercised
LANE_MASK = 0b1010_0101         # lanes 0, 2, 5, 7 (non-contiguous -> tests lane skip)

# ---- tiny deterministic LCG (no numpy dependency) ----
_state = 0xC0FFEE
def rnd(lo, hi):
    global _state
    _state = (_state * 6364136223846793005 + 1442695040888963407) & ((1 << 64) - 1)
    return lo + (_state >> 17) % (hi - lo + 1)

def to_hex(val, bits):
    return format(val & ((1 << bits) - 1), 'x').zfill((bits + 3) // 4)

COEF_MIN, COEF_MAX = -(1 << (COEF_W - 1)), (1 << (COEF_W - 1)) - 1
coef = [max(COEF_MIN, min(COEF_MAX, rnd(-80000, 80000))) for _ in range(NUM_TAPS)]
samp = [[[rnd(-2000, 2000) for _ in range(N_LANES)] for _ in range(N_SLOTS)]
        for _ in range(K_PACKETS)]

with open(f"{OUT}/lfp_coefs.hex", "w") as f:
    for j in range(NUM_TAPS):
        f.write(to_hex(coef[j], COEF_W) + "\n")

# samples: one 128-bit word per (packet, slot), lane 0 in the low 16 bits
with open(f"{OUT}/lfp_samples.hex", "w") as f:
    for p in range(K_PACKETS):
        for s in range(N_SLOTS):
            word = 0
            for l in range(N_LANES):
                word |= (samp[p][s][l] & 0xFFFF) << (16 * l)
            f.write(format(word, 'x').zfill(32) + "\n")

ROUND   = 1 << (COEF_FRAC - 1)
OUT_MAX = (1 << (OUT_W - 1)) - 1
OUT_MIN = -(1 << (OUT_W - 1))
enabled_lanes = [l for l in range(N_LANES) if (LANE_MASK >> l) & 1]

def sample_at(p, s, l):
    return samp[p][s][l] if p >= 0 else 0   # ring is 0 before any write (BRAM config-init)

exp_val, exp_chan = [], []
for p in range(K_PACKETS):
    if (p % DECIM_R) != (DECIM_R - 1):       # trigger when decim_cnt+1 >= R
        continue
    for l in enabled_lanes:                  # engine emits lanes ascending, slots ascending
        for s in range(N_SLOTS):
            acc = sum(coef[j] * sample_at(p - j, s, l) for j in range(NUM_TAPS))
            r = (acc + ROUND) >> COEF_FRAC    # Python >> on ints floors like Verilog >>>
            r = OUT_MAX if r > OUT_MAX else OUT_MIN if r < OUT_MIN else r
            exp_val.append(r)
            exp_chan.append(l * N_SLOTS + s)

with open(f"{OUT}/lfp_exp_val.hex", "w") as f:
    for v in exp_val:
        f.write(to_hex(v, OUT_W) + "\n")
with open(f"{OUT}/lfp_exp_chan.hex", "w") as f:
    for c in exp_chan:
        f.write(to_hex(c, 16) + "\n")

print(f"taps={NUM_TAPS} packets={K_PACKETS} sample_words={K_PACKETS*N_SLOTS} "
      f"frames={K_PACKETS // DECIM_R} enabled_lanes={enabled_lanes} "
      f"expected_outputs={len(exp_val)}")
