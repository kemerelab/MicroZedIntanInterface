
# Set dummy IOSTANDARDs to suppress NSTD-1 (IOSTANDARD not specified)
set_property IOSTANDARD LVCMOS33 [get_ports clk]
set_property IOSTANDARD LVCMOS33 [get_ports count[*]]
set_property IOSTANDARD LVCMOS33 [get_ports enable[*]]

# Remove any PACKAGE_PIN assignments if these ports are not connected physically
set_property PACKAGE_PIN "" [get_ports clk]
set_property PACKAGE_PIN "" [get_ports count[*]]
set_property PACKAGE_PIN "" [get_ports enable[*]]

# Downgrade DRC errors to warnings
set_property SEVERITY {Warning} [get_drc_checks NSTD-1]
set_property SEVERITY {Warning} [get_drc_checks UCIO-1]