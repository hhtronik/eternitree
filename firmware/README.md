HHTronik / @embedaddy's EterniTree Firmware
===========================================

This repository contains the firmware for the HHTronik / @embedaddy EterniTree

Development environment setup
-----------------------------

You need an up-to-date version of the STM32 Cube IDE. This can be downloaded [at ST micro's STM32CubeIDE page]https://www.st.com/en/development-tools/stm32cubeide.html) and follow the instructions shown over there.

Next, clone this repository and open it.

Debugging and flashing
----------------------

To connect the debug probe (you will need an STLink V2 or V3) connect the debug port pins (labeled `VCC`, `SWDIO`, `SWCLK`, `NRST` and `GND` on the PCB) to the matching pins on the debug probe.

The STM32CubeIDE should allow you compile the solution easily as well as to do real debugging on the target.

Licence
-------

This software is under GPLv3 licence. See [LICENCE](./LICENCE) file for a full copy of the terms.

The STM32 HAL and related libraries are under their own licence (Copyright 2016 STMicroelectronics, licenced under BSD 3-Clause license)