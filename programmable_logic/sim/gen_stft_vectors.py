#!/usr/bin/env python3
# Vectors for stft_engine_tb.sv. Feeds K lanes of int16 LFP samples over N frames,
# triggers one STFT pass, and provides the expected windowed-DFT spectrum (float32
# bit patterns) for the Hermitian half. The behavioral FFT computes the same DFT
# in real arithmetic, so the TB compares with float tolerance.
import os, math, struct

OUT = os.path.dirname(os.path.abspath(__file__))
K = 4
N = 64
CHANS = [2, 50, 100, 200]          # selected source channels for lanes 0..K-1

_s = 0x2024
def rnd(lo, hi):
    global _s
    _s = (_s * 6364136223846793005 + 1442695040888963407) & ((1 << 64) - 1)
    return lo + (_s >> 17) % (hi - lo + 1)

# Hann window, Q15 signed
win = [int(round((0.5 - 0.5 * math.cos(2 * math.pi * n / (N - 1))) * 32767)) for n in range(N)]
# samples[frame][lane], int16
samp = [[rnd(-3000, 3000) for _ in range(K)] for _ in range(N)]

with open(f"{OUT}/stft_win.hex", "w") as f:
    for n in range(N):
        f.write(format(win[n] & 0xFFFF, '04x') + "\n")

# one 128-bit-ish line per frame: K lanes x 16-bit (lane0 low); TB drives each lane
with open(f"{OUT}/stft_samp.hex", "w") as f:
    for fr in range(N):
        word = 0
        for l in range(K):
            word |= (samp[fr][l] & 0xFFFF) << (16 * l)
        f.write(format(word, '016x') + "\n")    # 64-bit (K=4)

# expected: per lane, bins 0..N/2, complex float32 (re then im), as 32-bit hex
def f32hex(x):
    return format(struct.unpack('<I', struct.pack('<f', x))[0], '08x')

NB = N // 2 + 1
with open(f"{OUT}/stft_exp.hex", "w") as f:
    for l in range(K):
        for k in range(NB):
            re = im = 0.0
            for n in range(N):
                x = samp[n][l] * win[n]          # windowed (raw int product), matches RTL
                ang = -2.0 * math.pi * k * n / N
                re += x * math.cos(ang)
                im += x * math.sin(ang)
            f.write(f32hex(re) + "\n")            # real word
            f.write(f32hex(im) + "\n")            # imag word

print(f"K={K} N={N} chans={CHANS} bins={NB} -> {K*NB*2} expected words")
