#!/usr/bin/env bash
# Compile + run the cic_decimator bit-accuracy testbench under xsim.
# STATUS: the CIC comb pass is NOT yet bit-exact (see docs/PHASE_A_SUMMARY.md);
# this script is here for the follow-up fix. The shipped Phase A datapath is the
# dual-MAC FIR (run_lfp_fir_tb.sh), which IS timing-closed + bit-exact.
# Usage: source /opt/Xilinx/2025.1/Vivado/settings64.sh && bash run_cic_tb.sh
set -o pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
SRC="$HERE/../src"
WORK="$(mktemp -d)"
python3 "$HERE/gen_cic_vectors.py" || exit 1
cp "$HERE"/cic_*.hex "$WORK/" || exit 1
cd "$WORK" || exit 99
xvlog -sv "$SRC/cic_decimator.sv" "$HERE/cic_decimator_tb.sv" || exit 1
xelab -debug off -timescale 1ns/1ps work.cic_decimator_tb -s tb_snap || exit 1
xsim tb_snap -R | tee sim.log
grep -q "RESULT: PASS" sim.log && { echo "TB_PASS"; exit 0; } || { echo "TB_FAIL"; exit 1; }
