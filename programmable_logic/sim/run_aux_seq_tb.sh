#!/usr/bin/env bash
# Compile + run the aux_command_sequencer self-checking testbench under xsim.
# Usage:  source /opt/Xilinx/2025.1/Vivado/settings64.sh && bash run_aux_seq_tb.sh
# Greps for "RESULT: PASS". Production RTL carries no `timescale, so we supply a
# default to xelab (-timescale) to avoid the mixed-timescale elaboration error.
set -o pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
SRC="$HERE/../src"
WORK="$(mktemp -d)"
cd "$WORK" || exit 99

xvlog -sv "$SRC/aux_command_sequencer.sv" "$HERE/aux_command_sequencer_tb.sv" || exit 1
xelab -debug off -timescale 1ns/1ps work.aux_command_sequencer_tb -s tb_snap || exit 1
xsim tb_snap -R | tee sim.log

grep -q "RESULT: PASS" sim.log && { echo "TB_PASS"; exit 0; } || { echo "TB_FAIL"; exit 1; }
