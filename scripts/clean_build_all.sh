#!/usr/bin/env bash
# Clean end-to-end build: Vivado project -> bitstream/XSA -> Vitis firmware (both cores).
# Logs everything; prints PASS/FAIL markers we can grep for.
set -o pipefail
cd "$(dirname "$0")/.." || exit 99
ROOT="$(pwd)"
echo "=== CLEAN BUILD START $(date -u +%H:%M:%S) in $ROOT ==="

echo "=== [1/3] Sourcing Vivado + recreating project ==="
source /opt/Xilinx/2025.1/Vivado/settings64.sh || { echo "BUILD_FAIL: vivado settings"; exit 1; }
# wipe any prior generated project for a truly clean run
rm -rf vivado_project .Xil vivado*.log vivado*.jou 2>/dev/null

vivado -mode batch -source scripts/create_vivado_project.tcl \
  && echo "STEP1_OK: project created" || { echo "BUILD_FAIL: create_vivado_project"; exit 1; }

echo "=== [2/3] Synth + impl + bitstream + XSA ==="
vivado -mode batch -source scripts/build_bitstream.tcl \
  && echo "STEP2_OK: bitstream built" || { echo "BUILD_FAIL: build_bitstream"; exit 1; }

if [ ! -f vivado_project/klab_project.xsa ]; then
  echo "BUILD_FAIL: XSA missing after build_bitstream"; exit 1
fi
echo "XSA_OK: $(ls -la vivado_project/klab_project.xsa)"

echo "=== [3/3] Sourcing Vitis + building firmware (both cores) ==="
source /opt/Xilinx/2025.1/Vitis/settings64.sh || { echo "BUILD_FAIL: vitis settings"; exit 1; }
rm -rf vitis_workspace 2>/dev/null
vitis -s scripts/create_vitis_project.py \
  && echo "STEP3_OK: vitis firmware built" || { echo "BUILD_FAIL: create_vitis_project"; exit 1; }

echo "=== ARTIFACTS ==="
ls -la vivado_project/klab_project.xsa 2>/dev/null
find vitis_workspace -name '*.elf' 2>/dev/null
echo "=== CLEAN BUILD DONE $(date -u +%H:%M:%S) — BUILD_PASS ==="
