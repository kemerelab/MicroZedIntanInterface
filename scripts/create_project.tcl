# Vivado Project Creation Script for Zynq 7000
# This script recreates the project from source files and exported configurations

# Set project variables
set project_name "klab_project"
set project_dir "./vivado_project"
set part_name "xc7z020clg400-1"

# Create project directory if it doesn't exist
file mkdir $project_dir

# Create the project
create_project $project_name $project_dir -part $part_name -force

# Set project properties
set_property target_language Verilog [current_project]
set_property simulator_language Mixed [current_project]

# Add source files
puts "Adding source files..."
add_files -norecurse src/custom_counter_block.v
update_compile_order -fileset sources_1

# Add constraint files
puts "Adding constraint files..."
add_files -fileset constrs_1 -norecurse constraints/top.xdc
add_files -fileset constrs_1 -norecurse constraints/ignore_unused_ports.xdc

# Create block design from exported TCL
puts "Creating block design..."
source block_design/design_1_bd.tcl

# Make the block design wrapper
puts "Creating HDL wrapper..."
make_wrapper -files [get_files design_1.bd] -top
add_files -norecurse [get_property directory [current_project]]/[current_project].srcs/sources_1/bd/design_1/hdl/design_1_wrapper.v
update_compile_order -fileset sources_1

# Set the wrapper as top module
set_property top design_1_wrapper [current_fileset]

# Generate block design
puts "Generating block design..."
generate_target all [get_files design_1.bd]

# Optional: Create runs if they don't exist
if {[llength [get_runs synth_1]] == 0} {
    create_run synth_1 -part $part_name -flow {Vivado Synthesis 2023}
}
if {[llength [get_runs impl_1]] == 0} {
    create_run impl_1 -parent_run synth_1 -flow {Vivado Implementation 2023}
}

puts "Project creation completed successfully!"
puts "Project location: [get_property directory [current_project]]"
puts ""
puts "Next steps:"
puts "1. Review the project in Vivado GUI"
puts "2. Run synthesis: launch_runs synth_1 -jobs 4"
puts "3. Run implementation: launch_runs impl_1 -jobs 4"
puts "4. Generate bitstream: launch_runs impl_1 -to_step write_bitstream -jobs 4"