#!/usr/bin/env bash
# Compile + run the lfp_dsp_block integration testbench under xsim.
# Usage: source /opt/Xilinx/2025.1/Vivado/settings64.sh && bash run_lfp_block_tb.sh
set -o pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
SRC="$HERE/../src"
WORK="$(mktemp -d)"

python3 "$HERE/gen_lfp_block_vectors.py" || exit 1
cp "$HERE"/lfp_blk_coefs.hex "$HERE"/lfp_blk_samples.hex "$HERE"/lfp_blk_exp_words.hex "$WORK/" || exit 1

cd "$WORK" || exit 99
xvlog -sv "$SRC/lfp_fir_decimator.sv" "$SRC/cic_decimator.sv" "$SRC/cic_to_halfband.sv" \
      "$SRC/lfp_halfband.sv" "$SRC/lfp_dsp_block.sv" "$HERE/lfp_dsp_block_tb.sv" || exit 1
xelab -debug off -timescale 1ns/1ps work.lfp_dsp_block_tb -s tb_snap || exit 1
xsim tb_snap -R | tee sim.log

grep -q "RESULT: PASS" sim.log && { echo "TB_PASS"; exit 0; } || { echo "TB_FAIL"; exit 1; }
