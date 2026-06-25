#!/usr/bin/env bash
# Compile + run the accel_extract_block self-checking testbench under xsim.
# Usage: source /opt/Xilinx/2025.1/Vivado/settings64.sh && bash run_accel_extract_tb.sh
set -o pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
SRC="$HERE/../src"
WORK="$(mktemp -d)"

cd "$WORK" || exit 99
xvlog -sv "$SRC/accel_extract_block.sv" "$HERE/accel_extract_block_tb.sv" || exit 1
xelab -debug off -timescale 1ns/1ps work.accel_extract_block_tb -s tb_snap || exit 1
xsim tb_snap -R | tee sim.log

grep -q "RESULT: PASS" sim.log && { echo "TB_PASS"; exit 0; } || { echo "TB_FAIL"; exit 1; }
