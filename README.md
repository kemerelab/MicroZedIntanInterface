# MicroZedIntanInterface
Verilog and firmware for an FPGA interface to the Intan Data Acquisition protocol. We leverage the MicroZed 
Zynq7000 developmment board. The Zynq7020 combines dual Arm A9 processors with a reasonably sized programmable
logic fabric, and the development board adds DDR memory and a Gigabit Ethernet interface. We have implemented
firmware which handles the single and DDR data interfaces for two RHD2000-style ICs, connected via a single
Intan-standard Omnetics 12 pin cable. The interface streams data (up to 128 channels at 30ksps), with a 
user-programmable delay on the data lines and controlled via a TCP interface.

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

### Create a bootable SD card
Run `bootgen -image scripts/boot.bif -o BOOT.bin -w` and copy the resulting `BOOT.bin` file to the FAT32
formatted `Boot` partition on your SD card.

