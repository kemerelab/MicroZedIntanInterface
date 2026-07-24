#!/usr/bin/env bash
# SPDX-License-Identifier: MIT
# SPDX-FileCopyrightText: 2025-2026 Caleb Kemere, Reet Sinha, Allen Mikhailov, Rice University

# Compile + run the axi_lite_registers write-handshake testbench under xsim.
# Usage:  source /opt/Xilinx/2025.1/Vivado/settings64.sh && bash run_axi_write_tb.sh
set -o pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
SRC="$HERE/../src"
WORK="$(mktemp -d)"
cd "$WORK" || exit 99

xvlog "$SRC/axi_lite_registers.v" || exit 1
xvlog -sv "$HERE/axi_lite_write_tb.sv" || exit 1
xelab -debug off -timescale 1ns/1ps work.axi_lite_write_tb -s tb_snap || exit 1
xsim tb_snap -R | tee sim.log

grep -q "RESULT: PASS" sim.log && { echo "TB_PASS"; exit 0; } || { echo "TB_FAIL"; exit 1; }
