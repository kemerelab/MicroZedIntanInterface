#!/usr/bin/env bash
# Compile + run the lfp_halfband (/2) bit-accuracy testbench under xsim.
# Usage: source /opt/Xilinx/2025.1/Vivado/settings64.sh && bash run_halfband_tb.sh
set -o pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
SRC="$HERE/../src"
WORK="$(mktemp -d)"
python3 "$HERE/gen_halfband_vectors.py" || exit 1
cp "$HERE"/hb_*.hex "$WORK/" || exit 1
cd "$WORK" || exit 99
xvlog -sv "$SRC/lfp_halfband.sv" "$HERE/lfp_halfband_tb.sv" || exit 1
xelab -debug off -timescale 1ns/1ps work.lfp_halfband_tb -s tb_snap || exit 1
xsim tb_snap -R | tee sim.log
grep -q "RESULT: PASS" sim.log && { echo "TB_PASS"; exit 0; } || { echo "TB_FAIL"; exit 1; }
