#!/usr/bin/env bash
# Compile + run the data_generator aux-integration testbench under xsim.
# Extracts the UNMODIFIED pre-integration core from git (main @ bbcadfe) as
# data_generator_core_legacy for the cycle-exact aux_seq_en=0 identity check.
# Usage:  source /opt/Xilinx/2025.1/Vivado/settings64.sh && bash run_aux_integration_tb.sh
set -o pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
SRC="$HERE/../src"
REPO="$(cd "$HERE/../.." && pwd)"
BASELINE_COMMIT=bbcadfe   # main: pre-aux-integration baseline
WORK="$(mktemp -d)"

# Patch the baseline for xsim WITHOUT changing synthesis semantics:
#  - rename the module
#  - hoist the transmission_active declaration (xsim rejects the baseline's
#    use-before-declaration; Vivado synth tolerates it)
#  - turn `logic name = expr;` declarations into `wire name = expr;` -- synth
#    treats both as continuous assignments, but xsim treats the logic form as
#    a one-time static initializer (stuck-at-X), which would make the
#    identity comparison vacuous.
git -C "$REPO" show "$BASELINE_COMMIT:programmable_logic/src/data_generator_core.sv" \
  | sed -e 's/\bdata_generator_core\b/data_generator_core_legacy/' \
        -e '/^logic        transmission_active;$/d' \
        -e 's|^// Extract control bits$|// Extract control bits\nlogic transmission_active;|' \
        -e '/^logic.*=.*;/s/^logic/wire/' \
  > "$WORK/data_generator_core_legacy.sv" || exit 1

cd "$WORK" || exit 99

xvlog -sv "$SRC/data_generator_core.sv" "$SRC/aux_command_sequencer.sv" \
      "$SRC/override_layer.sv" "$WORK/data_generator_core_legacy.sv" \
      "$HERE/data_generator_aux_tb.sv" || exit 1
xvlog "$SRC/CIPO_phase_selector.v" || exit 1
xelab -debug off -timescale 1ns/1ps work.data_generator_aux_tb -s tb_snap || exit 1
xsim tb_snap -R | tee sim.log

grep -q "RESULT: PASS" sim.log && { echo "TB_PASS"; exit 0; } || { echo "TB_FAIL"; exit 1; }
