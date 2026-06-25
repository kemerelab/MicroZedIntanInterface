# build_bitstream.tcl
open_project ./vivado_project/klab_project.xpr

# The data_generator is a module-reference block synthesized OUT OF CONTEXT
# (its own child run design_1_data_generator_0_synth_1). The top synth_1 links
# that child's .dcp. If the child run is stale / not current when impl runs,
# opt_design sees data_generator as a BLACK BOX and fails (DRC INBB-3). So
# explicitly (re)synthesize the OOC child FIRST and wait for it, THEN the top
# synth_1, THEN impl. (Documented gotcha -- see CLAUDE.md.)
set ooc_run design_1_data_generator_0_synth_1
if {[llength [get_runs $ooc_run]] > 0} {
    reset_run $ooc_run
    launch_runs $ooc_run -jobs 4
    wait_on_run $ooc_run
}

reset_run synth_1
launch_runs synth_1 -jobs 4
wait_on_run synth_1

reset_run impl_1
launch_runs impl_1 -to_step write_bitstream -jobs 4
wait_on_run impl_1

# Fail loudly if implementation did not complete through write_bitstream.
if {[get_property PROGRESS [get_runs impl_1]] ne "100%" ||
    [get_property STATUS   [get_runs impl_1]] eq "opt_design ERROR"} {
    error "impl_1 did not complete: STATUS=[get_property STATUS [get_runs impl_1]] PROGRESS=[get_property PROGRESS [get_runs impl_1]]"
}

# export bitstream or hardware if needed
write_hw_platform -fixed -include_bit -force -file ./vivado_project/klab_project.xsa
