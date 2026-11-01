# Board pins from installed TUL PYNQ-Z2 board definition, revision A.0/1.0.
# PMOD B1 -> UART RX, PMOD B2 -> UART TX. Offline build only.
set_property -dict {PACKAGE_PIN W14 IOSTANDARD LVCMOS33} [get_ports uart_rx]
set_property -dict {PACKAGE_PIN Y14 IOSTANDARD LVCMOS33} [get_ports uart_tx]
set_property -dict {PACKAGE_PIN D19 IOSTANDARD LVCMOS33} [get_ports {btns_tri_i[0]}]
set_property -dict {PACKAGE_PIN D20 IOSTANDARD LVCMOS33} [get_ports {btns_tri_i[1]}]
set_property -dict {PACKAGE_PIN L20 IOSTANDARD LVCMOS33} [get_ports {btns_tri_i[2]}]
set_property -dict {PACKAGE_PIN L19 IOSTANDARD LVCMOS33} [get_ports {btns_tri_i[3]}]
# The board's UART and push-buttons are asynchronous to FCLK0. Two-stage
# synchronizers are in input_sync; interstage/internal paths remain timed.
set_false_path -from [get_ports {uart_rx btns_tri_i[*]}]
# UART TX is an asynchronous serial link, not a source-synchronous output bus.
set_false_path -to [get_ports uart_tx]
