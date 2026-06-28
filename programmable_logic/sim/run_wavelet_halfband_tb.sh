#!/usr/bin/env bash
# Compile + run the wavelet_halfband ÷2 primitive unit testbench under xsim.
# Usage: source /opt/Xilinx/2025.1/Vivado/settings64.sh && bash run_wavelet_halfband_tb.sh
set -o pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
SRC="$HERE/../src"
WORK="$(mktemp -d)"
cd "$WORK" || exit 99
xvlog -sv "$SRC/wavelet_halfband.sv" "$HERE/wavelet_halfband_tb.sv" || exit 1
xelab -debug off -timescale 1ns/1ps work.wavelet_halfband_tb -s tb_snap || exit 1
xsim tb_snap -R | tee sim.log
grep -q "RESULT: PASS" sim.log && { echo "TB_PASS"; exit 0; } || { echo "TB_FAIL"; exit 1; }
