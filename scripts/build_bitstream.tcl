# build_bitstream.tcl
open_project ./vivado_proj/klab_project.xpr

reset_run synth_1
launch_runs synth_1 -jobs 4
wait_on_run synth_1

reset_run impl_1
launch_runs impl_1 -to_step write_bitstream -jobs 4
wait_on_run impl_1

# Optional: export bitstream or hardware if needed

