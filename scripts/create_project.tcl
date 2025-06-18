# create_project.tcl
set proj_name "klab-project"
set proj_dir "./vivado_proj"
set part "xc7z020clg400-1"  ;# or whatever part you're targeting

# Create a new project
create_project $proj_name $proj_dir -part $part -force

# Source block design Tcl script
source ../block_design/design_1_bd.tcl

# Generate wrapper for the block design
make_wrapper -files [get_files design_1.bd] -top

# Add the generated wrapper file to the project
set wrapper_file [glob "$proj_dir.gen/sources_1/bd/design_1/hdl/design_1_wrapper.v"]
add_files -norecurse $wrapper_file

# Add custom IP
add_files -fileset sources_1 [glob ./src/*.v]

# Add XDC constraints
add_files -fileset constrs_1 [glob ../constraints/*.xdc]

# Set top module
set_property top design_1_wrapper [current_fileset]

# Save the project (optional)
save_project

