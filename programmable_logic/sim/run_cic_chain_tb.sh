#!/usr/bin/env bash
# Compile + run the CIC(/5)->halfband(/2) = /10 chain testbench under xsim.
# Usage: source /opt/Xilinx/2025.1/Vivado/settings64.sh && bash run_cic_chain_tb.sh
set -o pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
SRC="$HERE/../src"
WORK="$(mktemp -d)"
python3 "$HERE/gen_cic_chain_vectors.py" || exit 1
cp "$HERE"/cicch_*.hex "$WORK/" || exit 1
cd "$WORK" || exit 99
xvlog -sv "$SRC/cic_decimator.sv" "$SRC/cic_to_halfband.sv" "$SRC/lfp_halfband.sv" \
      "$HERE/cic_chain_tb.sv" || exit 1
xelab -debug off -timescale 1ns/1ps work.cic_chain_tb -s tb_snap || exit 1
xsim tb_snap -R | tee sim.log
grep -q "RESULT: PASS" sim.log && { echo "TB_PASS"; exit 0; } || { echo "TB_FAIL"; exit 1; }
