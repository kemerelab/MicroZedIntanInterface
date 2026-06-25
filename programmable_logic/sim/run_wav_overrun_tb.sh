#!/usr/bin/env bash
# Compute-budget / OVERRUN measurement for wavelet_cqt_engine (NOT bit-accuracy).
# Drives the engine at the full build config (8 octaves, 4 voices, 24 taps).
#
# TWO measurements (the second is the authoritative real-time gate for the
# v2 work-spread engine):
#
#  (1) wav_overrun_tb  -- ISOLATED single pass (NFRAMES=2, FRAMECLKS=600000).
#      Measures the worst-case duration of ONE compute pass with NO next-frame
#      preemption. For the pre-work-spread (eager) engine this WAS the real-time
#      worst case (~990*K clk for 2-MAC step1, ~1758*K single-MAC). For the
#      v2 STEP-2 work-spread engine it is NO LONGER the real-time metric: with no
#      next frame to preempt, the engine drains EVERY enqueued octave back-to-
#      back (the un-spread cost ~990*K), which never happens at the real frame
#      rate. Kept as a diagnostic only.
#
#  (2) wav_spread_tb   -- REAL-TIME spacing (FRAMECLKS=28000, the 84 MHz / 3 kHz
#      budget; NFRAMES sweeps fcount through 0 repeatedly). This is the correct
#      gate for the work-spread engine: PASS = the overrun flag NEVER asserts AND
#      frame_seq advances once per frame (no dropped columns). Reports the busy
#      duty cycle (steady-state utilization).
#
# Usage:
#   source /opt/Xilinx/2025.1/Vivado/settings64.sh && bash run_wav_overrun_tb.sh
# Edit `parameter int K` in wav_overrun_tb.sv / wav_spread_tb.sv to sweep K.
#
# Findings (2026-06-25):
#   single-MAC eager (pre-v2):  isolated pass ~1758*K  -> max clean K=16.
#   v2 STEP 1 (2 MAC lanes):    isolated pass ~990*K   -> max clean K=16.
#   v2 STEP 2 (2 MAC + work-spread), REAL-TIME spread TB (FRAMECLKS=28000):
#     K=64 -> NO_OVERRUN (duty 53%)   K=96  -> NO_OVERRUN (duty 80%)
#     K=112-> NO_OVERRUN (duty 94%, tight)   K=128 -> OVERRUN (duty 99%).
#   So the 2-MAC + work-spread real-time-clean ceiling is K=96 (robust) / K=112
#   (tight). K=256 needs a 4-MAC (2-voices-per-cycle) datapath (not built).
set -o pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
SRC="$HERE/../src"

echo "=== (1) ISOLATED single-pass diagnostic (wav_overrun_tb) ==="
WORK="$(mktemp -d)"; cd "$WORK" || exit 99
xvlog -sv "$SRC/wavelet_halfband.sv" "$SRC/wavelet_cqt_engine.sv" \
         "$HERE/wav_overrun_tb.sv" || exit 1
xelab -debug off -timescale 1ns/1ps work.wav_overrun_tb -s tb_iso || exit 1
xsim tb_iso -R -testplusarg NFRAMES=2 -testplusarg FRAMECLKS=600000 | tee iso.log
echo

echo "=== (2) REAL-TIME work-spread gate (wav_spread_tb, FRAMECLKS=28000) ==="
WORK2="$(mktemp -d)"; cd "$WORK2" || exit 99
xvlog -sv "$SRC/wavelet_halfband.sv" "$SRC/wavelet_cqt_engine.sv" \
         "$HERE/wav_spread_tb.sv" || exit 1
xelab -debug off -timescale 1ns/1ps work.wav_spread_tb -s tb_rt || exit 1
xsim tb_rt -R -testplusarg NFRAMES=600 -testplusarg FRAMECLKS=28000 | tee rt.log
grep -q "RESULT: NO_OVERRUN" rt.log && echo "REALTIME: FIT" || echo "REALTIME: OVERRUN"
