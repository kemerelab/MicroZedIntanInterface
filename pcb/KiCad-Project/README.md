# KiCad design files and production files

The PCB design was dervied from work by Viktor Nokolov licensed under the Creative Commons Zero license. His original project can be found here [https://github.com/viktor-nikolov/MicroZed-carrier-board](https://github.com/viktor-nikolov/MicroZed-carrier-board).

### Revision 1.0 Notes
Board revision 1.0 fixes the bug in 0.11, and transitions from 2 layer to 4 layer fabrication. This allows for
a smaller and slightly more case-friendly layout (digital IO on the same side of the PCB as Intan/Omnetics 
connectors, and USB-C power moved to the same side as the ethernet connector).

### Revision 0.11 Notes
Board revision 0.11 was used for initial testing. There was an error in the connectivity for the USB-UART
bridge. We fixed this with a bodge that is circled in the image below.

![Revision 0.11 Manufactured PCB](Rev011-PCB.png)
