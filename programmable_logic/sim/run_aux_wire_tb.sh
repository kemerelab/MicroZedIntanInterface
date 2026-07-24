#!/usr/bin/env bash
# SPDX-License-Identifier: MIT
# SPDX-FileCopyrightText: 2025-2026 Caleb Kemere, Reet Sinha, Allen Mikhailov, Rice University
# Compile + run the aux wire-level integration testbench under xsim.
# Usage: source /opt/Xilinx/2025.1/Vivado/settings64.sh && bash run_aux_wire_tb.sh
set -o pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
SRC="$HERE/../src"
WORK="$(mktemp -d)"; cd "$WORK" || exit 99
xvlog -sv "$SRC/acq_frame_pkg.sv" "$SRC/aux_command_engine.sv" "$SRC/aux_program.sv" "$SRC/data_generator_core.sv" "$SRC/test_signal_gen.sv" \
      "$HERE/data_generator_aux_wire_tb.sv" || exit 1
xvlog "$SRC/CIPO_phase_selector.v" || exit 1
xelab -debug off -timescale 1ns/1ps work.data_generator_aux_wire_tb -s tb_snap || exit 1
xsim tb_snap -R | tee sim.log
grep -q "RESULT: PASS" sim.log && { echo "TB_PASS"; exit 0; } || { echo "TB_FAIL"; exit 1; }
