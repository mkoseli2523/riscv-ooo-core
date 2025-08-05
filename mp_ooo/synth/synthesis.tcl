source dc_warn.tcl

if {[getenv ECE411_MIN_POWER] eq "1"} {
   set power_enable_minpower true
}
set hdlin_ff_always_sync_set_reset true
set hdlin_ff_always_async_set_reset true
set hdlin_infer_multibit default_all
set hdlin_check_no_latch true
set hdlin_while_loop_iterations 2000000000
set_host_options -max_cores [getenv ECE411_DC_CORES]
set_app_var report_default_significant_digits 6
set design_toplevel [getenv DESIGN_TOP]

define_design_lib WORK -path ./work
set alib_library_analysis_path [getenv STD_CELL_ALIB]

set symbol_library [list generic.sdb]
set synthetic_library [list dw_foundation.sldb]
set target_library [getenv STD_CELL_LIB]
set sram_library [getenv SRAM_LIB]

if {$sram_library eq ""} {
   set link_library [list "*" $target_library $synthetic_library]
} else {
   set link_library [list "*" $target_library $synthetic_library $sram_library]
}

get_license DC-Ultra-Features
get_license DC-Ultra-Opt

set pkg_src [getenv PKG_SRCS]

if {$pkg_src ne ""} {
   analyze -library WORK -format sverilog $pkg_src
}

set modules [split [getenv HDL_SRCS] " "]
foreach module $modules {
   analyze -library WORK -format sverilog "${module}"
}

elaborate $design_toplevel
current_design $design_toplevel

change_names -rules verilog -hierarchy

check_design

set_wire_load_model -name "5K_hvratio_1_1"
set_wire_load_mode enclosed

source constraints.sdc

link

eval [getenv ECE411_COMPILE_CMD]
for {set i 0} {$i < [getenv ECE411_COMPILE_ITER]} {incr i} {
    eval [getenv ECE411_COMPILE_CMD_INC]
}

current_design $design_toplevel

report_area -hier > reports/area.rpt
report_timing -delay max > reports/timing.rpt
check_design

write_file -format ddc -hierarchy -output outputs/synth.ddc
write_file -format verilog -hierarchy -output [format "outputs/%s.gate.v" $design_toplevel]

exit
