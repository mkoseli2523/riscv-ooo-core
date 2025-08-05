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

read_file -format ddc outputs/synth.ddc

read_saif -input [getenv ECE411_SAIF_FILE] -instance [getenv ECE411_SAIF_TOP]
report_power -analysis_effort high -hierarchy > [getenv ECE411_POWER_RPT_PATH]/power.rpt
report_power -analysis_effort high > [getenv ECE411_POWER_RPT_PATH]/power2.rpt
exit
