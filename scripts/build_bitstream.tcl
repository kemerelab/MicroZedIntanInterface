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
set ooc_runs [get_runs -filter {IS_SYNTHESIS && NAME != "synth_1"}]
foreach r $ooc_runs { reset_run $r }
reset_run synth_1

# Run the out-of-context modules to completion FIRST, and wait for each one by
# name. Letting them ride along as dependencies of synth_1 is not enough:
# `wait_on_run synth_1` returns when the TOP finishes, which can leave a child
# checkpoint not yet linked, and implementation then fails at opt_design with
# the module as an empty black box -- with no synthesis error anywhere, because
# the child run itself succeeded. Waiting explicitly makes the order certain.
if {[llength $ooc_runs]} {
    launch_runs $ooc_runs -jobs 4
    foreach r $ooc_runs { wait_on_run $r }
}

launch_runs synth_1 -jobs 4
wait_on_run synth_1

reset_run impl_1
launch_runs impl_1 -to_step write_bitstream -jobs 4
wait_on_run impl_1

# export bitstream or hardware if needed
write_hw_platform -fixed -include_bit -force -file ./vivado_project/klab_project.xsa
