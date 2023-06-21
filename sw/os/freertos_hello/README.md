 
## Summary
This application demonstrates a simple FreeRTOS application,
together with ParaNut linked-mode capabilities.

Two tasks are being created. One with higher priority, one with lower priority.

Both print "!!Hello from task 1!!" or "Hello from task 2" accordingly.

"Hello from task 2" uses multiple cores to print.
As accessing the facilities provided for printing characters is not safe to be accessed from multiple cores,
sometimes a few letters will be omitted.

For more information refer to [Details](#details)

## Requirements

* Linux x86_64 compatible computer (tested with Ubuntu 20.04)

* Basic development utilities installed (GNU Make, etc.)

* Installed RISC-V toolchain (tested with [SiFive Toolchain 2020.04](https://github.com/sifive/freedom-tools/releases/tag/v2020.04.0-Toolchain.Only)
with riscv64-unknown-elf-gcc and other tools visible to the command line.
(for example by using following command): 

```sh
export PATH=$PATH:/home/user/Downloads/riscv64-unknown-elf-gcc-8.3.0-2020.04.1-x86_64-linux-ubuntu14/bin/
```

* As well as this repository checked out and libparanut built.
The path of this repository will be referenced as $paranut_root from now on.

* BASH compatible shell

## Preparation
To build this application, it's necessary to checkout the FreeRTOS sources with adjustments for the ParaNut processor.
To do this run the following command.

```sh
git submodule update --init --recursive
```

## Building
In case this application is to be executed on hardware, 
it is suggested to build this application using the same config.mk as the hardware configuration.

To build this application make sure "settings.sh" is still sourced,
as well as the toolchain.

Enter this directory and run make:

```sh
cd ./sw/os/freertos_hello/ # Relative to $paranut_root
make
```
Per default this clones the FreeRTOS-Kernel repository using SSH, if you would like to use HTTPS use:
```
make GIT_HTTPS=1
```


## Running
To test the application simply run following command from within the directory with this README (same as the one used before):

```sh
make sim
```

In case hardware has been built as mentioned in the main [README file of the ParaNut project](../../README.md#operating-a-paranut-on-real-hardware).
Following command can be used for running the built application on hardware.

```sh
pn-flash -p freertos_demo.bin -b ../../systems/refdesign/hardware/build/system.bit ../../systems/refdesign/hardware/build/system.hdf ../../systems/refdesign/hardware/firmware/firmware.elf
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
cp -R $SOURCE_DIR/sw/freertos_hello $NEW_PROJECT_DIR/sw/freertos_hello
```

Now the directory freertos_hello under ./sw/freertos_hello can be modified or renamed as wanted.

The steps to build and test the project are the the same as mentioned in [Building](#building) and [Running](#running).
Just relative to the project root instead of $paranut_root.

Please be aware however that this means that the project will not be in sync with the repository.
Changes to the hardware in the repository will not be available from within the out-of-source project.

## Details

The second task will run with the amount of cores enabled during the generation of paranut-config.h
(determined by config.mk in the project root).
This might lead to each character being printed multiple times.
In simulation this usually only occurs when more than two cores are activated.
In hardware this behaviour occurs more often.

Task 2 is running more often, while task 1 has a higher priority.
As such when task 1 is done waiting for two time units,
task 2 will get paused until task 1 is finished.

This can lead to "Hello from task 2" being paused for "!!Hello from task 1!!".
Which shows that the software and hardware can run preemptively.

This sample uses newlib to print.
Consult the more elaborate "main.c" in the "freertos" directory
for other methods to print strings in case using newlib is not wanted or required.
The "freertos" directory contains the sources required to build the FreeRTOS kernel,
that this project depends on.
