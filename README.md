# MicroZedProjects
Repository with work in process Microzed projects

## Steps to building and testing
0. Set the path up for Vivado command line - `source ~/Xilinx/2025.1/Vivado/settings64.sh`
1. Create a Vivado project  - `vivado -mode batch -source scripts/create_vivado_project.tcl`. This will
  create a Vivado project that you can open in the `<repository>/vivado_project/` directory.

**NOTE** I think that the only place the part is specified is in the `scripts/create_project.tcl` file.
(I made the project with a `xc7z020clg400-1`. I think you should be able to change this to 
`xc7z010clg400-1` safely????)

### Building from Within Vivado
1. Synthesis, Implementation, and Generate Bitstream - can simply click `Generate Bitstream` and these steps should be taken care of.
2. `File->Export->Export Hardware`, choose "Include Bitstream" option!

### Alternative to building from within Vivado
1. Run `vivado -mode batch -source scripts/build_bitstream.tcl`. This should end up with the exported hardware in `vivado_project/klab.xsa`


### Creating a Vitis project
1. Set the path for Vitis command line - `source ./Xilinx/2025.1/Vitis/settings64.sh`
2. From the root directory of the repository run `vitis -s scripts/create_vitis_project.py`. This will
  create a Vitis project that you can open in the `<repository>/vitis_project`. The script automatically
  sources the hardware file that was created in the previous step with Vivado. It by default also builds
  this project.
3. I'm still missing the final step of building the BIF which we can load onto an SD card. You can do this
  from Vitis in the meantime.

