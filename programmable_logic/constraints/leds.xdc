# LED Constraints
# LED0 - Heartbeat (system alive) (connected to JX2 / VCCO bank 35)
set_property PACKAGE_PIN B19 [get_ports led0]
set_property IOSTANDARD LVCMOS25 [get_ports led0]

# LED1 - Transmission active  (connected to JX2 / VCCO bank 35)
set_property PACKAGE_PIN A20 [get_ports led1]
set_property IOSTANDARD LVCMOS25 [get_ports led1]

# LED2 - TBD (connected to JX1 / VCCO bank 34)
set_property PACKAGE_PIN T14 [get_ports led2]
set_property IOSTANDARD LVCMOS25 [get_ports led2]

# LED3 - TBD  (connected to JX1 / VCCO bank 34)
set_property PACKAGE_PIN T15 [get_ports led3]
set_property IOSTANDARD LVCMOS25 [get_ports led3]