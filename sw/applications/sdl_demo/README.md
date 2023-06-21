 
## Summary
This application demonstrates the use of SDL2 for drawing a simple game.

It consists of an automated version of pong,
in which two rackets try to prevent the ball from leaving the field on their side.

It's designed to be deployed on an old ZYBO board.

This project does *NOT* work inside a simulator,
as the VEARS core is expected to be connected.

## Requirements

* Linux x86_64 compatible computer (tested with Ubuntu 20.04)

* ZYBO Development Board

* Basic development utilities installed (GNU Make, etc.)

* Installed RISC-V toolchain (tested with [SiFive Toolchain 2020.04](https://github.com/sifive/freedom-tools/releases/tag/v2020.04.0-Toolchain.Only)
with riscv64-unknown-elf-gcc and other tools visible to the command line.
(for example by using following command): 

```sh
export PATH=$PATH:/home/user/Downloads/riscv64-unknown-elf-gcc-8.3.0-2020.04.1-x86_64-linux-ubuntu14/bin/
```

* As well as this repository checked out and libparanut built.
The path of this repository will be referenced as $paranut_root from now on.

* Xilinx Vivado (tested with 2017.2)

* BASH compatible shell

## Preparation
It requires SDL2 to be present in $paranut_root/sw/SDL2
as well as the code of the VEARS graphics core (in $paranut_root/external/asterics).

Both can be prepared by executing prepare_sdl.sh in this directory.
Please ensure to run it from this directory, else the files will be prepared in the wrong directory.

```sh
./prepare_sdl.sh
```

Afterwards the appropriate bitfile can be built.

To do this please ensure that Vivado and Vivado HLS is sourced.

For example by doing the following:

```sh
. /opt/Xilinx/Vivado/2017.2/settings64.sh
. /opt/Xilinx/Vivado_HLS/2017.2/.settings64-Vivado_High_Level_Synthesis.sh
```

Now the bitfile can be built using following commands:

```sh
# Assuming we're still in $paranut_root
# Set required environmental variables
. settings.sh
cd ./systems/refdesign_vears/
make
```

## Building
In case this application is to be executed on hardware, 
it is suggested to build this application using the same config.mk as the hardware configuration.

To build this application make sure "settings.sh" is still sourced,
as well as the toolchain.

Enter this directory and run make:

```sh
cd ./sw/sdl_demo/ # Relative to $paranut_root
make
```

## Running
Now the demo can be loaded to the board using:

```sh
pn-flash -p sdl_demo.bin -b ../../systems/refdesign_vears/hardware/build/system.bit ../../systems/refdesign_vears/hardware/build/system.hdf ../../systems/refdesign_vears/hardware/firmware/firmware.elf
```

which should return something like:

```
ParaNut flash software 2.0.0   (C) 2019 Alexander Bahle, University of Applied Sciences Augsburg

Connected to /dev/ttyUSB2
Downloading firmware... OK
(FIRMWARE) ParaNut Proxy firmware v1.0.0 ready.

Transfer ParaNut software 'sdl_demo.bin'.
Sent: 868208 of 868208 Bytes... OK

Downloading bitfile... OK

Hello World
```

Sometimes an error indicating that no communication with the ParaNut firmware will occur.
In that case it might have been that pn-flash tried to use the wrong serial device.
Another device can be specified using the -d flag.

```sh
pn-flash -d /dev/ttyUSB2 -p sdl_demo.bin -b ../../systems/refdesign_vears/hardware/build/system.bit ../../systems/refdesign_vears/hardware/build/system.hdf ../../systems/refdesign_vears/hardware/firmware/firmware.elf
```

## Out-of-source build
To build this project from outside the git repository,
different directories will need to be copied.

Following commands will guide you through the creation of a new out-of-source project.
(As always assuming "settings.sh" is already sourced).

Please ensure to execute these in a new project folder.

```sh
pn-newproject -c
export SOURCE_DIR=$(realpath $PARANUT_TOOLS/..)
export NEW_PROJECT_DIR=$(realpath ./paranut)
cd $NEW_PROJECT_DIR
cp -R $SOURCE_DIR/sw/freertos $NEW_PROJECT_DIR/sw/freertos
cp -R $SOURCE_DIR/sw/sdl_demo $NEW_PROJECT_DIR/sw/sdl_demo
cp -R $SOURCE_DIR/external/SDL2 $NEW_PROJECT_DIR/external/SDL2
mkdir -p $NEW_PROJECT_DIR/external/asterics/ipcores/VEARS/drivers/vears_v1_0
cp -R $SOURCE_DIR/external/asterics/ipcores/VEARS/drivers/vears_v1_0/src $NEW_PROJECT_DIR/external/asterics/ipcores/VEARS/drivers/vears_v1_0/src
ln -s $NEW_PROJECT_DIR/external/asterics/ipcores/VEARS/drivers/vears_v1_0/src $NEW_PROJECT_DIR/sw/vears
```

Now the directory sdl_demo under ./sw/sdl_demo can be modified as wanted.

The steps to build the application are the same as in [Building](#building).
To run the application on the board, paths in pn-flash will need to be adjusted to point to the same files located within the source repository.

Please be aware however that this means that the project will not be in sync with the repository and it's usability and maintainability will be severely impaired.
Changes to the hardware in the repository will not be available from within the out-of-source project.

## Modifying icons/logos
The icons are currently stored as 8-bit indexed BMPs, which can be generated using GIMP.
The closest color to magenta in the palette is apparently used for color keying.
To generate these files, bin2c from the package "hxtools" has been used.
It's available from Debian as well as Ubuntu software repositories.

A file can be converted using this command:

```sh
bin2c -H paranut_logo.h paranut.bmp
```

