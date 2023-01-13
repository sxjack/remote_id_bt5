# Remote ID

A program for the nRF52840 that transmits opendroneid / ASTM F3411 / ASD-STAN 4709-002 UAV direct remote identification signals over Bluetooth 4 & 5 LR. 

This was my first significant program using Zephyr. It may be my last :-).

## Getting Started

This program uses the nRF Connect SDK and Zephyr.

  # Install [nRF Connect for Desktop](https://www.nordicsemi.com/Products/Development-tools/nrf-connect-for-desktop) and use it to install the Programmer and Toolchain Manager. I have version 2.2 of the toolchain.
  # Get the blink app working.
  # Clone this repository.
  # Add copies of opendroneid.c and opendroneid.h from [opendroneid](https://github.com/opendroneid/opendroneid-core-c/tree/master/libopendroneid) to the src directory.

## Boards

I have run this on the [nRF52840 dongle](https://www.nordicsemi.com/Products/Development-hardware/nrf52840-dongle) and the [Seeed XIAO nRF52840](https://www.seeedstudio.com/Seeed-XIAO-BLE-nRF52840-p-5201.html).

  # nRF dongle, put it into bootloader and use the nRF Programmer.
  # XIAO, double click the reset and use the script. You will have to edit the com port and directories.








