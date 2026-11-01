# Vivado 2018.3, PYNQ-Z2. Build only; no hardware manager/download commands.
set root [file dirname [file normalize [info script]]]
set action [expr {[llength $argv] ? [lindex $argv 0] : "synth"}]
if {$action ni {create synth implement}} {error "Use create, synth or implement"}
set generated [file join $root generated]
file mkdir $generated
create_project -force smartcar_minimal [file join $generated smartcar_minimal] -part xc7z020clg400-1
set_property board_part tul.com.tw:pynq-z2:part0:1.0 [current_project]
set_property target_language Verilog [current_project]
set_property simulator_language Mixed [current_project]
add_files [glob [file join $root rtl *.v]]
create_bd_design smartcar
set ps [create_bd_cell -type ip -vlnv xilinx.com:ip:processing_system7:5.5 ps7_0]
apply_bd_automation -rule xilinx.com:bd_rule:processing_system7 -config {make_external "FIXED_IO, DDR" apply_board_preset "1" Master "Disable" Slave "Disable"} $ps
set_property -dict [list CONFIG.PCW_USE_M_AXI_GP0 {1} CONFIG.PCW_EN_CLK0_PORT {1} CONFIG.PCW_FPGA0_PERIPHERAL_FREQMHZ {100}] $ps
set fabric [create_bd_cell -type ip -vlnv xilinx.com:ip:axi_interconnect:2.1 axi_periph]
set_property -dict [list CONFIG.NUM_SI {1} CONFIG.NUM_MI {4}] $fabric
set rst [create_bd_cell -type ip -vlnv xilinx.com:ip:proc_sys_reset:5.0 reset_0]
set one [create_bd_cell -type ip -vlnv xilinx.com:ip:xlconstant:1.1 logic_one]
set_property CONFIG.CONST_VAL {1} $one
set zero [create_bd_cell -type ip -vlnv xilinx.com:ip:xlconstant:1.1 logic_zero]
set_property CONFIG.CONST_VAL {0} $zero
set uart [create_bd_cell -type ip -vlnv xilinx.com:ip:axi_uartlite:2.0 axi_uartlite_0]
set_property -dict [list CONFIG.C_BAUDRATE {115200} CONFIG.C_DATA_BITS {8} CONFIG.C_USE_PARITY {0} CONFIG.USE_BOARD_FLOW {false}] $uart
set btn [create_bd_cell -type ip -vlnv xilinx.com:ip:axi_gpio:2.0 btns_gpio]
set_property -dict [list CONFIG.C_GPIO_WIDTH {4} CONFIG.C_ALL_INPUTS {1} CONFIG.C_INTERRUPT_PRESENT {0}] $btn
set rx [create_bd_cell -type ip -vlnv xilinx.com:ip:axi_gpio:2.0 rx_level_0]
set_property -dict [list CONFIG.C_GPIO_WIDTH {1} CONFIG.C_ALL_INPUTS {1} CONFIG.C_INTERRUPT_PRESENT {0}] $rx
create_bd_cell -type module -reference sp21_axi sensor_core_0
create_bd_cell -type module -reference input_sync input_sync_0
set urx [create_bd_port -dir I uart_rx]
set utx [create_bd_port -dir O uart_tx]
set buttons [create_bd_port -dir I -from 3 -to 0 btns_tri_i]
connect_bd_net $urx [get_bd_pins input_sync_0/uart_rx_async]
connect_bd_net $buttons [get_bd_pins input_sync_0/btn_async]
connect_bd_net [get_bd_pins input_sync_0/uart_rx_sync] [get_bd_pins axi_uartlite_0/rx] [get_bd_pins rx_level_0/gpio_io_i]
connect_bd_net [get_bd_pins input_sync_0/btn_sync] [get_bd_pins btns_gpio/gpio_io_i]
connect_bd_net [get_bd_pins axi_uartlite_0/tx] $utx
connect_bd_intf_net [get_bd_intf_pins ps7_0/M_AXI_GP0] [get_bd_intf_pins axi_periph/S00_AXI]
foreach {master slave} {M00_AXI axi_uartlite_0/S_AXI M01_AXI btns_gpio/S_AXI M02_AXI rx_level_0/S_AXI M03_AXI sensor_core_0/S_AXI} {
    connect_bd_intf_net [get_bd_intf_pins axi_periph/$master] [get_bd_intf_pins $slave]
}
set clockpins {ps7_0/M_AXI_GP0_ACLK axi_periph/ACLK axi_periph/S00_ACLK reset_0/slowest_sync_clk axi_uartlite_0/s_axi_aclk btns_gpio/s_axi_aclk rx_level_0/s_axi_aclk sensor_core_0/aclk input_sync_0/clk}
for {set i 0} {$i<4} {incr i} {lappend clockpins [format "axi_periph/M%02d_ACLK" $i]}
connect_bd_net [get_bd_pins ps7_0/FCLK_CLK0] [get_bd_pins $clockpins]
connect_bd_net [get_bd_pins ps7_0/FCLK_RESET0_N] [get_bd_pins reset_0/ext_reset_in]
connect_bd_net [get_bd_pins logic_one/dout] [get_bd_pins reset_0/dcm_locked]
connect_bd_net [get_bd_pins logic_zero/dout] [get_bd_pins reset_0/aux_reset_in] [get_bd_pins reset_0/mb_debug_sys_rst]
connect_bd_net [get_bd_pins reset_0/interconnect_aresetn] [get_bd_pins axi_periph/ARESETN] [get_bd_pins axi_periph/S00_ARESETN]
set resetpins {axi_uartlite_0/s_axi_aresetn btns_gpio/s_axi_aresetn rx_level_0/s_axi_aresetn sensor_core_0/aresetn input_sync_0/resetn}
for {set i 0} {$i<4} {incr i} {lappend resetpins [format "axi_periph/M%02d_ARESETN" $i]}
connect_bd_net [get_bd_pins reset_0/peripheral_aresetn] [get_bd_pins $resetpins]
foreach {cell block offset range} {axi_uartlite_0 Reg 0x42C00000 64K btns_gpio Reg 0x41210000 64K rx_level_0 Reg 0x41260000 64K sensor_core_0 reg0 0x43C80000 4K} {
    create_bd_addr_seg -range $range -offset $offset [get_bd_addr_spaces ps7_0/Data] [get_bd_addr_segs $cell/S_AXI/$block] SEG_$cell
}
validate_bd_design
if {[get_property CONFIG.C_EXT_RESET_HIGH $rst] != 0} {error "PS reset polarity must be active low"}
save_bd_design
set bd [get_files */smartcar.bd]
generate_target all $bd
make_wrapper -files $bd -top
add_files [file join [get_property DIRECTORY [current_project]] smartcar_minimal.srcs sources_1 bd smartcar hdl smartcar_wrapper.v]
set_property top smartcar_wrapper [current_fileset]
add_files -fileset constrs_1 [file join $root constraints pynq_z2.xdc]
update_compile_order -fileset sources_1
write_bd_tcl -force [file join $generated recreate_bd.tcl]
write_project_tcl -force [file join $generated recreate_project.tcl]
if {$action eq "create"} {close_project; exit}
launch_runs synth_1 -jobs 4
wait_on_run synth_1
if {[get_property PROGRESS [get_runs synth_1]] ne "100%"} {error "Synthesis failed"}
open_run synth_1
set reports [file join $root reports]
file mkdir $reports
report_utilization -file [file join $reports utilization_synth.rpt]
report_timing_summary -file [file join $reports timing_synth.rpt]
report_clock_interaction -file [file join $reports clocks_synth.rpt]
report_drc -file [file join $reports drc_synth.rpt]
if {$action eq "implement"} {
    close_design
    launch_runs impl_1 -to_step route_design -jobs 4
    wait_on_run impl_1
    if {[get_property PROGRESS [get_runs impl_1]] ne "100%"} {error "Implementation failed"}
    open_run impl_1
    report_utilization -file [file join $reports utilization_route.rpt]
    report_timing_summary -file [file join $reports timing_route.rpt]
    report_drc -file [file join $reports drc_route.rpt]
    write_checkpoint -force [file join $generated minimal_routed.dcp]
}
close_project
puts "SMARTCAR_MINIMAL_BUILD_OK $action"
