#!/usr/bin/env python3
# Vectors for lfp_dsp_block_tb.sv: proves the integration layer around the
# (already bit-exact) FIR engine -- amplifier-slot gating + the 2-cycle readback
# remap, offset-binary<->signed (^0x8000) in/out, and the 2x16-bit BRAM packing.
# Feeds ALL 35 cycle slots; aux slots {0,1,34} carry junk and MUST be ignored.
import os

OUT = os.path.dirname(os.path.abspath(__file__))

N_LANES   = 8
N_SLOTS   = 32              # amplifier channels per lane (engine slots 0..31)
FIRST_AMP = 2              # cycle_counter of amplifier channel 0 (the +2 readback offset)
N_CYCLES  = 35             # cycle_counter range 0..34
COEF_W    = 18
COEF_FRAC = 17
OUT_W     = 16
NUM_TAPS  = 131            # Phase A 3 kHz anti-alias length (odd -> partial last group)
DECIM_R   = 10             # Phase A: 30 kHz / 10 = 3 kHz
K_PACKETS = 160            # > ring-fill; 16 decimation frames
LANE_MASK = 0b0010_0101    # lanes 0, 2, 5
OFFSET    = 0x8000
JUNK      = 0xDEAD         # aux-slot filler that must never reach the output

_state = 0x1234_5678
def rnd(lo, hi):
    global _state
    _state = (_state * 6364136223846793005 + 1442695040888963407) & ((1 << 64) - 1)
    return lo + (_state >> 17) % (hi - lo + 1)

def to_hex(v, bits):
    return format(v & ((1 << bits) - 1), 'x').zfill((bits + 3) // 4)

COEF_MIN, COEF_MAX = -(1 << (COEF_W - 1)), (1 << (COEF_W - 1)) - 1
coef = [max(COEF_MIN, min(COEF_MAX, rnd(-80000, 80000))) for _ in range(NUM_TAPS)]

# signed amplifier samples [packet][engine_slot][lane]; stored as offset binary.
sig = [[[rnd(-2000, 2000) for _ in range(N_LANES)] for _ in range(N_SLOTS)]
       for _ in range(K_PACKETS)]

with open(f"{OUT}/lfp_blk_coefs.hex", "w") as f:
    for j in range(NUM_TAPS):
        f.write(to_hex(coef[j], COEF_W) + "\n")

# one 128-bit word per (packet, cycle_slot 0..34): offset-binary on amp slots,
# JUNK on aux slots {0,1,34}.
with open(f"{OUT}/lfp_blk_samples.hex", "w") as f:
    for p in range(K_PACKETS):
        for c in range(N_CYCLES):
            word = 0
            amp = FIRST_AMP <= c < FIRST_AMP + N_SLOTS
            for l in range(N_LANES):
                if amp:
                    code = (sig[p][c - FIRST_AMP][l] + OFFSET) & 0xFFFF
                else:
                    code = JUNK
                word |= code << (16 * l)
            f.write(format(word, 'x').zfill(32) + "\n")

# expected output: filter signed amp samples, round/sat, +OFFSET, pack 2/word.
ROUND   = 1 << (COEF_FRAC - 1)
SMAX, SMIN = (1 << (OUT_W - 1)) - 1, -(1 << (OUT_W - 1))
lanes = [l for l in range(N_LANES) if (LANE_MASK >> l) & 1]

def s_at(p, s, l):
    return sig[p][s][l] if p >= 0 else 0

# The PL now builds the COMPLETE LFP wire packet in BRAM: the UNIFIED 8-word
# common header AHEAD of each frame's decimated samples (docs/unified-packet-
# format.md). Header (matches lfp_dsp_block.sv + net.py):
#   w0 = MAGIC = 0xCAFEBABE
#   w1 = TYPE_VER = stream_type(2) | version(1)<<8 | flags<<16  (flags=0)
#   w2/w3 = 64-bit master timestamp of the last contributing broadband sample.
#           The decimation tick at packet p stamps master count p (the broadband
#           timestamp of packet p), so the frame triggered at p carries ts = p.
#   w4 = SEQ = PL frame sequence number (++ per emitted frame, 0-indexed)
#   w5 = AUX0 = lane_mask | (decim_R<<8) | (num_taps<<16) | (overrun<<24); overrun=0
#   w6 = AUX1 = num_samples = popcount(lane_mask) * N_SLOTS
#   w7 = RSVD = 0
UNIFIED_MAGIC  = 0xCAFEBABE
STREAM_TYPE_LFP = 2
UNIFIED_VERSION = 1
TYPE_VER = (STREAM_TYPE_LFP & 0xFF) | ((UNIFIED_VERSION & 0xFF) << 8)
CFG_WORD = (LANE_MASK & 0xFF) | ((DECIM_R & 0xFF) << 8) | ((NUM_TAPS & 0xFF) << 16)  # overrun=0
NUM_SAMPLES = bin(LANE_MASK & 0xFF).count('1') * N_SLOTS

words = []
frame_seq = 0
for p in range(K_PACKETS):
    if (p % DECIM_R) != (DECIM_R - 1):
        continue
    # ---- unified 8-word common header ----
    ts = p                                       # master count of the contributing packet
    words.append(UNIFIED_MAGIC)
    words.append(TYPE_VER)
    words.append(ts & 0xFFFFFFFF)
    words.append((ts >> 32) & 0xFFFFFFFF)
    words.append(frame_seq)
    words.append(CFG_WORD)
    words.append(NUM_SAMPLES)
    words.append(0)
    frame_seq += 1
    # ---- decimated samples (signed -> offset binary, packed 2/word) ----
    emit = []
    for l in lanes:                       # engine emit order: lane asc, slot asc
        for s in range(N_SLOTS):
            acc = sum(coef[j] * s_at(p - j, s, l) for j in range(NUM_TAPS))
            r = (acc + ROUND) >> COEF_FRAC
            r = SMAX if r > SMAX else SMIN if r < SMIN else r
            emit.append((r + OFFSET) & 0xFFFF)        # signed -> offset binary
    # pack 2x16-bit -> 32-bit {high, low}; frames have even sample counts (no straddle).
    for i in range(0, len(emit), 2):
        words.append((emit[i + 1] << 16) | emit[i])

with open(f"{OUT}/lfp_blk_exp_words.hex", "w") as f:
    for w in words:
        f.write(to_hex(w, 32) + "\n")

print(f"taps={NUM_TAPS} packets={K_PACKETS} frames={frame_seq} "
      f"lanes={lanes} bram_words={len(words)} (incl {frame_seq}x8 header words)")
