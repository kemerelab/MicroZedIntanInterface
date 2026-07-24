# SPDX-License-Identifier: MIT
# SPDX-FileCopyrightText: 2025-2026 Caleb Kemere, Reet Sinha, Allen Mikhailov, Rice University

# Constraints for 8 digital inputs (carrier GPIO1..8 -> digital_in_0[0..7])
#
# Pin map updated for the new carrier PCB rev (the GPIO header routing changed).
# digital_in_0[i] (i = pin-select index 0..7 in the fast-settle/digout commands)
# now lands on these FPGA package pins:
#   [0] W16  [1] V16  [2] W20  [3] V20  [4] P20  [5] N20  [6] U19  [7] U18
#
# TODO (schematic): the JX1 schematic SYMBOL is FLIPPED, so the net labels on the
# carrier schematic read 18N/18P, 16N/16P, 14N/14P, 12N/12P -- they are ACTUALLY
# 17N/17P, 15N/15P, 13N/13P, 11N/11P. The FPGA pins below are correct; the JX1
# schematic symbol and these net labels should eventually be fixed so the labels
# match reality.

set_property PACKAGE_PIN W16 [get_ports {digital_in_0[0]}]
set_property PACKAGE_PIN V16 [get_ports {digital_in_0[1]}]
set_property PACKAGE_PIN W20 [get_ports {digital_in_0[2]}]
set_property PACKAGE_PIN V20 [get_ports {digital_in_0[3]}]
set_property PACKAGE_PIN P20 [get_ports {digital_in_0[4]}]
set_property PACKAGE_PIN N20 [get_ports {digital_in_0[5]}]
set_property PACKAGE_PIN U19 [get_ports {digital_in_0[6]}]
set_property PACKAGE_PIN U18 [get_ports {digital_in_0[7]}]

set_property DIRECTION IN [get_ports {digital_in_0[7]}]
set_property IOSTANDARD LVCMOS33 [get_ports {digital_in_0[7]}]
set_property PULLTYPE PULLDOWN [get_ports {digital_in_0[7]}]
set_property DIRECTION IN [get_ports {digital_in_0[6]}]
set_property IOSTANDARD LVCMOS33 [get_ports {digital_in_0[6]}]
set_property PULLTYPE PULLDOWN [get_ports {digital_in_0[6]}]
set_property DIRECTION IN [get_ports {digital_in_0[5]}]
set_property IOSTANDARD LVCMOS33 [get_ports {digital_in_0[5]}]
set_property PULLTYPE PULLDOWN [get_ports {digital_in_0[5]}]
set_property DIRECTION IN [get_ports {digital_in_0[4]}]
set_property IOSTANDARD LVCMOS33 [get_ports {digital_in_0[4]}]
set_property PULLTYPE PULLDOWN [get_ports {digital_in_0[4]}]
set_property DIRECTION IN [get_ports {digital_in_0[3]}]
set_property IOSTANDARD LVCMOS33 [get_ports {digital_in_0[3]}]
set_property PULLTYPE PULLDOWN [get_ports {digital_in_0[3]}]
set_property DIRECTION IN [get_ports {digital_in_0[2]}]
set_property IOSTANDARD LVCMOS33 [get_ports {digital_in_0[2]}]
set_property PULLTYPE PULLDOWN [get_ports {digital_in_0[2]}]
set_property DIRECTION IN [get_ports {digital_in_0[1]}]
set_property IOSTANDARD LVCMOS33 [get_ports {digital_in_0[1]}]
set_property PULLTYPE PULLDOWN [get_ports {digital_in_0[1]}]
set_property DIRECTION IN [get_ports {digital_in_0[0]}]
set_property IOSTANDARD LVCMOS33 [get_ports {digital_in_0[0]}]
set_property PULLTYPE PULLDOWN [get_ports {digital_in_0[0]}]
