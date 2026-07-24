# SPDX-License-Identifier: MIT
# SPDX-FileCopyrightText: 2025-2026 Caleb Kemere, Reet Sinha, Allen Mikhailov, Rice University

# routing_analysis.tcl - open routed DCP and emit authoritative routing/timing/congestion reports
open_checkpoint ./vivado_project/klab_project.runs/impl_1/design_1_wrapper_routed.dcp

set OUT ./routing_analysis
file mkdir $OUT

puts "##### UTILIZATION (flattened) #####"
report_utilization -file $OUT/util_full.rpt
report_utilization -hierarchical -hierarchical_depth 4 -file $OUT/util_hier.rpt

puts "##### CLOCK INTERACTION #####"
report_clock_interaction -delay_type min_max -significant_digits 3 -file $OUT/clock_interaction.rpt

puts "##### CDC #####"
report_cdc -details -file $OUT/cdc.rpt

puts "##### WORST INTER-CLOCK SETUP PATHS 175->84 #####"
report_timing -from [get_clocks clk_out2_design_1_clk_wiz_0_84M_175M_0] \
              -to   [get_clocks clk_out1_design_1_clk_wiz_0_84M_175M_0] \
              -max_paths 8 -nworst 1 -delay_type max -significant_digits 3 \
              -file $OUT/worst_175_to_84.rpt

puts "##### WORST INTRA-CLOCK 84 MHz #####"
report_timing -from [get_clocks clk_out1_design_1_clk_wiz_0_84M_175M_0] \
              -to   [get_clocks clk_out1_design_1_clk_wiz_0_84M_175M_0] \
              -max_paths 8 -nworst 1 -delay_type max -significant_digits 3 \
              -file $OUT/worst_84_intra.rpt

puts "##### CONGESTION #####"
report_design_analysis -congestion -file $OUT/congestion.rpt

puts "##### HIGH FANOUT NETS #####"
report_high_fanout_nets -max_nets 40 -file $OUT/high_fanout.rpt

puts "##### CONTROL SETS #####"
report_control_sets -verbose -file $OUT/control_sets.rpt

puts "##### DONE ROUTING ANALYSIS #####"
