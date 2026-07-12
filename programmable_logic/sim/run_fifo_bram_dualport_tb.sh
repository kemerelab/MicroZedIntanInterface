#!/usr/bin/env bash
# Compile + run the fifo_bram_interface dual-port widening testbench under xsim.
# Extracts the UNMODIFIED 64-bit packer from main (bbcadfe) as
# fifo_bram_interface_legacy for the bit-identity comparison.
# Usage:  source /opt/Xilinx/2025.1/Vivado/settings64.sh && bash run_fifo_bram_dualport_tb.sh
set -o pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
SRC="$HERE/../src"
REPO="$(cd "$HERE/../.." && pwd)"
BASELINE_COMMIT=bbcadfe
WORK="$(mktemp -d)"

# Extract the baseline packer, rename it, and (for xsim only) convert its
# block-local declared-with-initializer FIFO reads into plain declarations +
# blocking assignments placed after all declarations -- a block-local
# `logic x = expr;` is a run-once static initializer under xsim, which would
# latch stale data. Synthesis-neutral; same transform applied to the new module.
git -C "$REPO" show "$BASELINE_COMMIT:programmable_logic/src/fifo_bram_interface.sv" \
  > "$WORK/legacy_raw.sv" || exit 1

python3 - "$WORK/legacy_raw.sv" "$WORK/fifo_bram_interface_legacy.sv" <<'PY'
import sys, re
src = open(sys.argv[1]).read()
src = re.sub(r'\bfifo_bram_interface\b', 'fifo_bram_interface_legacy', src)
# strip initializers from the four block-local reads
src = src.replace('logic [68:0] fifo_entry = write_fifo[fifo_read_ptr];',
                  'logic [68:0] fifo_entry;')
src = src.replace('logic packet_end = fifo_entry[68];', 'logic packet_end;')
src = src.replace('logic [3:0] channel_mask = fifo_entry[67:64];', 'logic [3:0] channel_mask;')
src = src.replace('logic [63:0] data_word = fifo_entry[63:0];', 'logic [63:0] data_word;')
# re-insert the reads as blocking assignments after the last declaration
src = src.replace('logic [1:0] seg_count;',
                  'logic [1:0] seg_count;\n'
                  '                    fifo_entry = write_fifo[fifo_read_ptr];\n'
                  '                    packet_end = fifo_entry[68];\n'
                  '                    channel_mask = fifo_entry[67:64];\n'
                  '                    data_word = fifo_entry[63:0];')
open(sys.argv[2], 'w').write(src)
PY
[ $? -eq 0 ] || exit 1

cd "$WORK" || exit 99
xvlog -sv "$SRC/fifo_bram_interface.sv" "$WORK/fifo_bram_interface_legacy.sv" \
      "$HERE/fifo_bram_dualport_tb.sv" || exit 1
xelab -debug off -timescale 1ns/1ps work.fifo_bram_dualport_tb -s tb_snap || exit 1
xsim tb_snap -R | tee sim.log

grep -q "RESULT: PASS" sim.log && { echo "TB_PASS"; exit 0; } || { echo "TB_FAIL"; exit 1; }
