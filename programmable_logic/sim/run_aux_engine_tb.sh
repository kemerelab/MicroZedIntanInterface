#!/usr/bin/env bash
# Compile + run the aux_command_engine testbench under xsim.
# Usage: source /opt/Xilinx/2025.1/Vivado/settings64.sh && bash run_aux_engine_tb.sh
set -o pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
SRC="$HERE/../src"
WORK="$(mktemp -d)"
cd "$WORK" || exit 99
xvlog -sv "$SRC/acq_frame_pkg.sv" "$SRC/aux_command_engine.sv" "$SRC/aux_program.sv" "$HERE/aux_command_engine_tb.sv" || exit 1
xelab -debug off -timescale 1ns/1ps work.aux_command_engine_tb -s tb_snap || exit 1
xsim tb_snap -R | tee sim.log
grep -q "RESULT: PASS" sim.log && { echo "TB_PASS"; exit 0; } || { echo "TB_FAIL"; exit 1; }
