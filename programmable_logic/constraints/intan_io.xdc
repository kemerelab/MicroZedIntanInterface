# Pin Connection Constraints for Intan Interface(s)
# SCLK- serial clock (PMOD3.5)
set_property PACKAGE_PIN R18 [get_ports sclk_0]
set_property IOSTANDARD LVCMOS25 [get_ports sclk_0]

# _CS (CSn) - Chip select (PMOD3.6)
set_property PACKAGE_PIN T17 [get_ports csn_0]
set_property IOSTANDARD LVCMOS25 [get_ports csn_0]

# COPI - Controller Out, Peripheral In (AKA MOSI) (PMOD3.7)
set_property PACKAGE_PIN W16 [get_ports copi_0]
set_property IOSTANDARD LVCMOS25 [get_ports copi_0]

# CIPO0 - first Controller In, Peripheral Out (AKA MIS0)(PMOD3.8)
set_property PACKAGE_PIN V16 [get_ports cpio0_0]
set_property IOSTANDARD LVCMOS25 [get_ports cpio0_0]

# CIPO0 - first Controller In, Peripheral Out (AKA MIS0)(PMOD3.4)
set_property PACKAGE_PIN W18 [get_ports cpio1_0]
set_property IOSTANDARD LVCMOS25 [get_ports cpio1_0]

# Differential pairs - LVDS or similar standards
set_property IOSTANDARD LVDS_25 [get_ports csn_p_0]
set_property IOSTANDARD LVDS_25 [get_ports csn_n_0]


set_property IOSTANDARD LVDS_25 [get_ports cipo0_p_0]
set_property IOSTANDARD LVDS_25 [get_ports cipo0_n_0]

set_property IOSTANDARD LVDS_25 [get_ports sclk_p_0]
set_property IOSTANDARD LVDS_25 [get_ports sclk_n_0]

set_property IOSTANDARD LVDS_25 [get_ports copi_p_0]
set_property IOSTANDARD LVDS_25 [get_ports copi_n_0]

set_property IOSTANDARD LVDS_25 [get_ports cipo1_p_0]
set_property IOSTANDARD LVDS_25 [get_ports cipo1_n_0]

# TODO: Add PACKAGE_PIN for differential pairs below (replace with actual pin names)
set_property PACKAGE_PIN D19 [get_ports csn_p_0]
set_property PACKAGE_PIN D20 [get_ports csn_n_0]

set_property PACKAGE_PIN E18 [get_ports cipo0_p_0]
set_property PACKAGE_PIN E19 [get_ports cipo0_n_0]

set_property PACKAGE_PIN E17 [get_ports sclk_p_0]
set_property PACKAGE_PIN D18 [get_ports sclk_n_0]

set_property PACKAGE_PIN F16 [get_ports copi_p_0]
set_property PACKAGE_PIN F17 [get_ports copi_n_0]


set_property PACKAGE_PIN M19 [get_ports cipo1_p_0]
set_property PACKAGE_PIN M20 [get_ports cipo1_n_0]
