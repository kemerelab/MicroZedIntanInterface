# build_bitstream.tcl
open_project ./vivado_project/klab_project.xpr

reset_run synth_1
launch_runs synth_1 -jobs 4
wait_on_run synth_1

reset_run impl_1
launch_runs impl_1 -to_step write_bitstream -jobs 4
wait_on_run impl_1

# export bitstream or hardware if needed
write_hw_platform -fixed -include_bit -force -file ./vivado_proj/klab_project.xsa
