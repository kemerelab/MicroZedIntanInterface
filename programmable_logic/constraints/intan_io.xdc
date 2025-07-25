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
