#!/usr/bin/env bash
# Compute-budget / OVERRUN measurement for wavelet_cqt_engine (NOT bit-accuracy).
# Drives the engine at the full build config (8 octaves, 4 voices, 24 taps) and
# measures the worst-case (fcount=0, all octaves coincide) compute-pass duration
# in clocks vs the 3 kHz @ 84 MHz budget (84e6/3000 = 28000 clocks/frame), and
# reports whether the overrun flag asserts.
#
# Usage:
#   source /opt/Xilinx/2025.1/Vivado/settings64.sh && bash run_wav_overrun_tb.sh
# Edit the `parameter int K` in wav_overrun_tb.sv to sweep K (16/32/.../256).
# Plusargs: +FRAMECLKS=<n> (frame spacing) +NFRAMES=<n>.
#
# Findings (2026-06-25), full 8-oct/4-voice/24-tap config, worst-case pass:
#   K=16 : 28135 clk (1.0x over)     K=32 : 56263 (2.0x)
#   K=64 : 112519 (4.0x)             K=128: 225031 (8.0x)
#   K=256: 450055 (16.1x over budget)  -> OVERRUN. Cost ~ 1758*K clocks.
#   Max sustainable K at full config ~= 16. K=256 needs the v2 2-MAC + per-octave
#   work-spread engine (a large redesign, not attempted on this branch).
set -o pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
SRC="$HERE/../src"
WORK="$(mktemp -d)"
cd "$WORK" || exit 99
xvlog -sv "$SRC/wavelet_halfband.sv" "$SRC/wavelet_cqt_engine.sv" \
         "$HERE/wav_overrun_tb.sv" || exit 1
xelab -debug off -timescale 1ns/1ps work.wav_overrun_tb -s tb_snap || exit 1
# wide frame spacing so a single pass completes and its true duration is measured
xsim tb_snap -R -testplusarg NFRAMES=2 -testplusarg FRAMECLKS=600000 | tee sim.log
grep -q "RESULT: NO_OVERRUN" sim.log && echo "FIT" || echo "OVERRUN"
