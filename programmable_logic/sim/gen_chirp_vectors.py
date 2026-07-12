#!/usr/bin/env python3
# Reference model for the analytic chirp NCO in data_generator_core.sv.
# Reproduces the dual-accumulator sweep + per-channel/per-lane phase fan-out and
# the 512-entry sine LUT lookup, bit-exactly, for chirp_chirp_tb.sv to check.
#
# The TB drives the core in debug_mode + chirp_mode, captures the data word at
# each (packet, cycle_slot), and compares lane 0..7 LUT values against these.
import math, os

OUT = os.path.dirname(os.path.abspath(__file__))

PHW          = 32
LUT_HI, LUT_LO = 31, 23          # phase_acc[31:23] = 9-bit LUT index
FSPAN_SHIFT  = 16
RATE_SHIFT   = 9
STRIDE_SHIFT = 24
MASK         = (1 << PHW) - 1

# ---- chirp config (must match chirp_tb.sv) ----
FSPAN  = 0x100      # f_max = 0x100 << 16 = 0x01000000 (~93.75 Hz step base)
RATE   = 4          # freq_acc step/packet = 4 << 9 = 2048
STRIDE = 5          # per-slot stride
N_PACKETS = 40
N_SLOTS_CYC = 35    # cycle_counter 0..34

def sine_lut():
    lut = []
    for i in range(512):
        ang = 2.0 * 3.14159265359 * i / 512.0
        lut.append(int(32767.0/16*math.sin(ang) + 32767.0))   # $rtoi truncates
    return lut

LUT = sine_lut()
FMAX  = (FSPAN << FSPAN_SHIFT) & MASK
RSTEP = (RATE  << RATE_SHIFT)

def run():
    freq_acc = 0
    phase_acc = 0
    up = True
    rows = []   # per packet: list over 35 cycles of [8 lane values]
    for p in range(N_PACKETS):
        # capture uses THIS packet's phase_acc (the accumulators advance at the
        # last state of the packet, i.e. AFTER the data word is produced).
        pkt = []
        for c in range(N_SLOTS_CYC):
            channel_offset = (c - 2) if c >= 2 else 0
            stride_prod = (channel_offset & 0x1F) * STRIDE
            ch_phase = (phase_acc + ((stride_prod << STRIDE_SHIFT) & MASK)) & MASK
            lanes = []
            for l in range(8):
                lane_phase = (ch_phase + ((l & 0x7) << (PHW - 3))) & MASK
                idx = (lane_phase >> LUT_LO) & 0x1FF
                lanes.append(LUT[idx])
            pkt.append(lanes)
        rows.append(pkt)
        # advance accumulators (end of packet)
        if up:
            if freq_acc + RSTEP >= FMAX:
                freq_acc = FMAX; up = False
            else:
                freq_acc += RSTEP
        else:
            if freq_acc <= RSTEP:
                freq_acc = 0; up = True
            else:
                freq_acc -= RSTEP
        phase_acc = (phase_acc + freq_acc) & MASK
    return rows

rows = run()
with open(f"{OUT}/chirp_exp.hex", "w") as f:
    for p in range(N_PACKETS):
        for c in range(N_SLOTS_CYC):
            # one 128-bit word: lane l in [16*l +: 16]
            word = 0
            for l in range(8):
                word |= (rows[p][c][l] & 0xFFFF) << (16 * l)
            f.write(format(word, 'x').zfill(32) + "\n")

print(f"FSPAN={FSPAN} RATE={RATE} STRIDE={STRIDE} FMAX={FMAX:#x} RSTEP={RSTEP} "
      f"packets={N_PACKETS} words={N_PACKETS*N_SLOTS_CYC}")
