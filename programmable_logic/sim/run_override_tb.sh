#!/usr/bin/env bash
# Compile + run the override_layer self-checking testbench under xsim.
# Usage:  source /opt/Xilinx/2025.1/Vivado/settings64.sh && bash run_override_tb.sh
# Greps for "RESULT: PASS".
set -o pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
SRC="$HERE/../src"
WORK="$(mktemp -d)"
cd "$WORK" || exit 99

xvlog -sv "$SRC/override_layer.sv" "$HERE/override_layer_tb.sv" || exit 1
xelab -debug off -timescale 1ns/1ps work.override_layer_tb -s tb_snap || exit 1
xsim tb_snap -R | tee sim.log

grep -q "RESULT: PASS" sim.log && { echo "TB_PASS"; exit 0; } || { echo "TB_FAIL"; exit 1; }
