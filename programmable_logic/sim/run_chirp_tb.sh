#!/usr/bin/env bash
# Compile + run the analytic-chirp NCO testbench under xsim.
# Usage: source /opt/Xilinx/2025.1/Vivado/settings64.sh && bash run_chirp_tb.sh
set -o pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
SRC="$HERE/../src"
WORK="$(mktemp -d)"

python3 "$HERE/gen_chirp_vectors.py" || exit 1
cp "$HERE/chirp_exp.hex" "$WORK/" || exit 1

cd "$WORK" || exit 99
xvlog -sv "$SRC/acq_frame_pkg.sv" "$SRC/data_generator_core.sv" "$SRC/test_signal_gen.sv" "$SRC/aux_command_engine.sv" "$SRC/aux_program.sv" \
      "$HERE/chirp_tb.sv" || exit 1
xvlog "$SRC/CIPO_phase_selector.v" || exit 1
xelab -debug off -timescale 1ns/1ps work.chirp_tb -s tb_snap || exit 1
xsim tb_snap -R | tee sim.log

grep -q "RESULT: PASS" sim.log && { echo "TB_PASS"; exit 0; } || { echo "TB_FAIL"; exit 1; }
