#!/bin/bash
# Run the clean-AXI-read-controller stress sim (xsim). Answers whether a custom
# axi_bram_ctrl could fix the 0xFF burst corruption (PASS => the read-logic is
# not the fault; it is the PS7 M_AXI_GP master).
set -e
cd "$(dirname "$0")"
source /opt/Xilinx/2025.1/Vivado/settings64.sh
rm -rf xsim.dir webtalk* *.jou *.log *.pb .Xil 2>/dev/null || true

xvlog -sv \
  ../src/bram.sv \
  axi_read_bram_ctrl.sv \
  tb_axi_read_bram_ctrl.sv

xelab tb_axi_read_bram_ctrl -s tb_read_sim -timescale 1ns/1ps
xsim tb_read_sim -R
