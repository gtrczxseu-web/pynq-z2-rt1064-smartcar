# Read-only reopen of the saved minimal project; no run reset or hardware calls.
set root [file dirname [file normalize [info script]]]
open_project [file join $root generated smartcar_minimal smartcar_minimal.xpr]
if {[get_property PART [current_project]] ne "xc7z020clg400-1"} {error "Wrong part"}
open_bd_design [get_files */smartcar.bd]
validate_bd_design
if {[get_property CONFIG.C_EXT_RESET_HIGH [get_bd_cells reset_0]] != 0} {error "Reset polarity"}
foreach {cell expected} {axi_uartlite_0 0x42C00000 btns_gpio 0x41210000 rx_level_0 0x41260000 sensor_core_0 0x43C80000} {
    set s [get_bd_addr_segs ps7_0/Data/SEG_$cell]
    if {[expr {[get_property OFFSET $s]+0}] != [expr {$expected+0}]} {error "Wrong address for $cell"}
}
open_run synth_1
if {[get_property PROGRESS [get_runs synth_1]] ne "100%"} {error "Incomplete synthesis"}
set setup [get_timing_paths -delay_type max -max_paths 1]
set hold [get_timing_paths -delay_type min -max_paths 1]
set wns [get_property SLACK $setup]
set whs [get_property SLACK $hold]
if {$wns < 0 || $whs < 0} {error "Synthesis timing estimate has a violation"}
set f [open [file join $root reports saved_project_check.txt] w]
puts $f "SAVED_PROJECT_OK part=xc7z020clg400-1 synth=100% wns_ns=$wns whs_ns=$whs"
close $f
puts "SAVED_PROJECT_OK wns_ns=$wns whs_ns=$whs"
close_project
