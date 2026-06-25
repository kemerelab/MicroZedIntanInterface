#!/usr/bin/env bash
# Compute-budget / OVERRUN measurement for wavelet_cqt_engine (NOT bit-accuracy).
# Configured here for the FINE build (K=6, 8 octaves, 6 voices, 40 taps) and
# measures the worst-case (fcount=0, all octaves coincide) compute-pass duration
# in clocks vs the 3 kHz @ 84 MHz budget (84e6/3000 = 28000 clocks/frame), and
# reports whether the overrun flag asserts.
#
# Usage:
#   source /opt/Xilinx/2025.1/Vivado/settings64.sh && bash run_wav_overrun_tb.sh
# Edit the params (K/V/N_TAPS/N_OCTAVES) in wav_overrun_tb.sv to sweep config.
# Plusargs: +FRAMECLKS=<n> (frame spacing) +NFRAMES=<n>.
#
# FINE build target: K=6, 8 oct, 6 voc, 40 tap. Budget = 28000 clk/frame
# (84 MHz / 3 kHz). Expected worst-case busy ~24600 clk (~4100/lane * 6),
# clean with ~12% margin. GATE: max_busy_cycles must be < 28000.
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
