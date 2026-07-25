#!/usr/bin/env bash
# SPDX-License-Identifier: MIT
# SPDX-FileCopyrightText: 2025-2026 Caleb Kemere, Reet Sinha, Allen Mikhailov, Rice University
# The LFP wire packet as assembled in the output BRAM (header + sample placement).
# Usage: source /opt/Xilinx/2025.1/Vivado/settings64.sh && bash run_lfp_packet_tb.sh
set -o pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
SRC="$HERE/../src"
WORK="$(mktemp -d)"
cd "$WORK" || exit 99

xvlog -sv "$SRC/unified_pkt_pkg.sv" "$SRC/lfp_coef_pkg.sv" \
      "$SRC/lfp_halfband_dec2.sv" "$SRC/lfp_poly_dec5.sv" "$SRC/lfp_dsp_block.sv" \
      "$HERE/lfp_packet_tb.sv" || exit 1
xelab -debug off -timescale 1ns/1ps work.lfp_packet_tb -s tb_snap || exit 1
xsim tb_snap -R | tee sim.log

grep -q "RESULT: PASS" sim.log && { echo "TB_PASS"; exit 0; } || { echo "TB_FAIL"; exit 1; }
