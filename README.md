# MicroZedProjects
Repository with work in process Microzed projects

## Steps to building and testing
0. Set the path up for Vivado command line - `source ~/tools24/Xilinx/Vivado/2024.2/settings64.sh`
1. Create a Vivado project  - `vivado -mode batch -source scripts/create_project.tcl`. This will
  create a Vivado project that you can open in the `<repository>/vivado_project/` directory.

### Within Vivado
1. Synthesis, Implementation, and Generate Bitstream - can simply click `Generate Bitstream` and these steps should be taken care of.
2. `File->Export->Export Hardware`, choose "Include Bitstream" option!

### From Console Window
0. Set the path - `source ~/tools24/Xilinx/Petalinux/2024.2/tool/settings.sh`
1. Create the Petalinux project. From the project directory, run `petalinux-create -t project -n <Project Name> --template zynq`
2. Copy the exported hardware file into this new Petalinux directory. This is how I do it `cd <Project Name>`, `cp ..\<Exported Name> .`
  (The default `<Exported Name>` is `design_1_wrapper.xsa`.)
3. Do some configuration. Run `petalinux-config -c rootfs`. Under `Image Features`, select `serial-autologin-root` (this is not critically necessary!).
  Under `Filesystem Packages -> misc`, locate `packagegroup-core-buildessential` and select it if you want gcc to be available. There may be some desire
  to configure the kernel as well, which you can do by running `petalinux-config -c kernel`. This takes a long time to run, though, and seems unneeded at
  the moment.
5. Build Petalinux. Run `petalinux-build`. This will take a (long) time.
6. Generate the files to boot from SD Card. `petalinux-package --boot --fsbl ./images/linux/zynq_fsbl.elf --uboot ./images/linux/u-boot.elf --fpga ./images/linux/system.bit --force`.
  (This assumes your pwd is the Petalinux project directory.) (At some point, we may want to change the structure of the SD files so that there is a filesystem
  that lasts across reboots.)
7. Copy the files `images/linux/boot.scr', `images/linux/BOOT.BIN`, and `images/linux/image.ub` to the SD card.


### Testing in Petalinux
1. With the SD card in the Microzed, the jumpers need to be set as J1 (topmost) - Left, J2 - Right, J3 - Right. (This is holding the Microzed
  so that the USB connector is facting up.)
2. When you plug in, it should boot, the LED on the right should be blue. If you connect to a serial monitor, you should eventually be logged in at a prompt.
3. This project implements a counter that is stopped or started by setting the value of a register. To enable the counter, write a non-zero value:
   `devmem 0x41200000 32 0x01`. You can read back that it has changed by just running `devmem 0x41200000 32` (this should tell you `0x00000001` since its
   a 32-bit register. Now, the counter is ticking, and the counter can be read at `devmem 0x41210000 32`. It should increment at about 1 Hz. You can
   also stop the counter by writing zero to the enable register, `devmem 0x41200000 32 0x0`, and see that it stops changing.


