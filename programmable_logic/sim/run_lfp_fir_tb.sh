#!/usr/bin/env bash
# Compile + run the lfp_fir_decimator bit-accuracy testbench under xsim.
# Generates the stimulus/expected vectors with the Python reference first.
# Usage: source /opt/Xilinx/2025.1/Vivado/settings64.sh && bash run_lfp_fir_tb.sh
set -o pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
SRC="$HERE/../src"
WORK="$(mktemp -d)"

python3 "$HERE/gen_lfp_fir_vectors.py" || exit 1
cp "$HERE"/lfp_coefs.hex "$HERE"/lfp_samples.hex \
   "$HERE"/lfp_exp_val.hex "$HERE"/lfp_exp_chan.hex "$WORK/" || exit 1

cd "$WORK" || exit 99
xvlog -sv "$SRC/lfp_fir_decimator.sv" "$HERE/lfp_fir_decimator_tb.sv" || exit 1
xelab -debug off -timescale 1ns/1ps work.lfp_fir_decimator_tb -s tb_snap || exit 1
xsim tb_snap -R | tee sim.log

grep -q "RESULT: PASS" sim.log && { echo "TB_PASS"; exit 0; } || { echo "TB_FAIL"; exit 1; }
