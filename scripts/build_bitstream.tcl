# SPDX-License-Identifier: MIT
# SPDX-FileCopyrightText: 2025-2026 Caleb Kemere, Reet Sinha, Allen Mikhailov, Rice University

# build_bitstream.tcl
open_project ./vivado_project/klab_project.xpr

# Each block-design module (data_generator, axi_lite_registers, ...) is synthesized
# OUT-OF-CONTEXT as its own run. `reset_run synth_1` resets ONLY the top and does NOT
# cascade to these child runs; Vivado's out-of-date detection can also miss edits to
# the underlying RTL in sources_1. The result is a stale sub-module DCP stitched into
# an otherwise-fresh top build -- SILENT: fresh timestamp/SHA, old logic. Reset every
# synthesis run so all modules re-synthesize from the current sources.
foreach r [get_runs -filter {IS_SYNTHESIS && NAME != "synth_1"}] { reset_run $r }
reset_run synth_1
launch_runs synth_1 -jobs 4
wait_on_run synth_1

reset_run impl_1
launch_runs impl_1 -to_step write_bitstream -jobs 4
wait_on_run impl_1

# export bitstream or hardware if needed
write_hw_platform -fixed -include_bit -force -file ./vivado_project/klab_project.xsa
