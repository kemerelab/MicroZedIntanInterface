#!/usr/bin/env bash
# SPDX-License-Identifier: MIT
# SPDX-FileCopyrightText: 2025-2026 Caleb Kemere, Reet Sinha, Allen Mikhailov, Rice University
# LFP stage 2 (polyphase /5) vs the bit-exact Python reference.
# Usage: source /opt/Xilinx/2025.1/Vivado/settings64.sh && bash run_lfp_poly_tb.sh
set -o pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
SRC="$HERE/../src"
WORK="$(mktemp -d)"

python3 "$HERE/gen_lfp_poly_vectors.py" || exit 1
cp "$HERE"/lfp_poly_samples.hex "$HERE"/lfp_poly_exp.hex "$WORK/" || exit 1
cd "$WORK" || exit 99

xvlog -sv "$SRC/lfp_coef_pkg.sv" "$SRC/lfp_poly_dec5.sv" \
      "$HERE/lfp_poly_dec5_tb.sv" || exit 1
xelab -debug off -timescale 1ns/1ps work.lfp_poly_dec5_tb -s tb_snap || exit 1
xsim tb_snap -R | tee sim.log

grep -q "RESULT: PASS" sim.log && { echo "TB_PASS"; exit 0; } || { echo "TB_FAIL"; exit 1; }
