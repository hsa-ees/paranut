![ParaNut](./doc-src/figs/pn_logo_long.svg)
=====================

The *ParaNut* processor is a customizable, highly scalable, and RISC-V compatible processor architecture for FPGA-based systems.

Key aspects of the *ParaNut* architecture are:

* **Scalability and Parallelism:** A special concept of parallelism combines the advantages of SIMD vectorization and simultaneous multi-threading in one architecture with a simple programming model.

* **Security:** Speculation techniques are generally avoided in order to make the processor robust against security flaws.

* **SystemC:** The *ParaNut* hardware is modeled in *SystemC* and, except for the memory and debugging module, the same code model is used for hardware synthesis as well as for building a cycle-accurate instruction set simulator. This ensures that the simulator reflects the real hardware behaviour.

> **_NOTE:_** Currently we are in the process of unifying the synthesis/simulation 
source code and migrating to [ICSC](https://github.com/intel/systemc-compiler) for High Level Synthesis. As a result, the present version of the ParaNut Processor can 
not be synthesized for hardware. We are actively working on providing this feature 
again, the omission is only temporary. If you require hardware synthesis, please use 
[ParaNut V1.0 (65f1057)](https://github.com/hsa-ees/paranut/commit/65f105762e45f78e3c32d06737560882f6f6c390). For more information on the ICSC see section "Intel Compiler for SystemC". For more information on the current project status see Appendix C.

The design is presently used in education and research. Experiments on a Xilinx 7 platform running the *CoreMark* benchmark reveals an almost perfect speedup of 3.97 for a 4-core processor and a speedup of 7.6 for a *ParaNut* with 8 CPU cores (as of Feb. 2020).

The simulator supports hardware-software co-design through the ability to produce VCD trace containing signals of custom peripherals and of the *ParaNut* processor itself.

Software debugging is possible using the simulator together with GDB through an *OpenOCD*-compatible interface.

A list of publications and further information on the project can be found on the home page of the [Efficient Embedded Systems](https://ees.hs-augsburg.de/paranut) group at the University of Applied Sciences Augsburg.


May. 24, 2023

- Felix Wagner (<felix.wagner1@hs-augsburg.de>)
- Marco Milenkovic (<marco.milenkovic@hs-augsburg.de>)
- Lukas Bauer (<lukas.bauer@hs-augsburg.de>)
- Abdurrahman Celep (<abdurrahman.celep1@hs-augsburg.de>)
- Michael Schaeferling (<michael.schaeferling@hs-augsburg.de>)
- Gundolf Kiefer (<gundolf.kiefer@hs-augsburg.de>)


## Further information

- [The *ParaNut* Processor: Architecture Description and Reference Manual](doc/paranut-manual.pdf)
- For software development: [The *ParaNut* support library (*libparanut*)](doc/libparanut/index.html)
- For hardware and system development: [The *ParaNut* simulator library (*libparanutsim*)](doc/libparanutsim/index.html)
- For getting started or just a quick test drive: **See below.**


## Licensing

See [LICENSE](LICENSE) or license notes in the respective directories / files.



Getting Started
===============

## 1. Prerequisites

For software compilation and debugging:

- RISC-V GNU Toolchain (*riscv64-unknown-elf-gcc*) ([latest release](https://github.com/sifive/freedom-tools/releases))
- RISC-V OpenOCD (*riscv-openocd*) ([latest release](https://github.com/sifive/freedom-tools/releases))

For simulation of the *ParaNut* using its *SystemC* model:

- *SystemC* 2.2 or later ([latest release](https://www.accellera.org/downloads/standards/systemc))
  - **_NOTE:_** Per default, it is expected that the SystemC library is compiled using 'gnu++14', yet some Linux distributions already use 'gnu++17'. If this is the case, the build process may fail (the linking process will encounter undefined reference errors). To solve this issue, adapt the entry 'SYSTEMC_CPP_VER' in file 'systemc-config.mk'. To obtain the GNU-C++ standard used to build your SystemC library, you may run the following command (here, using an Ubuntu package installation): 
    ```
    $ strings /usr/lib/x86_64-linux-gnu/libsystemc-2.3.3.so | grep sc_api_version
    ```
    An example output may look like this, where 'cxx2017' tells you the GNU-C++ standard used:
    ```
    _ZN7sc_core31sc_api_version_2_3_3_cxx201703LIXadL_ZNS_34SC_DISABLE_VIRTUAL_BIND_UNDEFINED_EEEEC2ENS_16sc_writer_policyE
    ```
- GtkWave for simulation

## 2. Setting up the environment

- Set up your terminal by sourcing the *settings.sh* (or *settings.zsh* if you use a *zshell*):
    ```
    $ source settings.sh
    ```

- Make sure *SystemC* is installed on your system. If *SystemC* has been installed manually, the *SYSTEMC_HOME* environment variable must point to the top level of the *SytemC* installation (e.g. */opt/systemc*).
    ```
    $ export SYSTEMC_HOME=/opt/systemc   # adapt according to your installation
    ```
    additionally, the LD_LIBRARY_PATH environment variable must contain the path to the SystemC library file:
    ```
    $ export LD_LIBRARY_PATH=/opt/SystemC/lib-linux:$LD_LIBRARY_PATH
    ```

> **_NOTE:_** If you want to use the SystemC library provided with your ICSC installation 
you must add USE_ICSC=1 to your make call. You must use the ICSC SystemC Library 
when running high level synthesis. Sources build with either version of the library can not be 
mixed. Make sure to run make clean before choosing to change to another library version. 
You may also use ```export USE_ICSC=1``` to always use the ICSC SystemC configuration.

- Make sure the RISC-V GNU Toolchain is in the search path.

- Enter `make help` to get further information on what you can do next:
    ```
    $ make help
    ```


## 3. Running the "Hello World" example

### a) For the brave and the impatient

Enter:

```
$ make hello
```

### b) Step-by-step using the default *ParaNut* simulator

Using this very simple method allows you to make quick use of the *ParaNut* in it's standard configuration.

- Install the *ParaNut* tools on your system:

    ```
    $ make
    $ sudo make install INSTALL_PREFIX=/opt/paranut   # this is default, adapt the installation path if desired
    ```
    At this stage you may want to additionally specify *OS_INSTALL="freertos linux"* (or just linux/freertos).
    This will install further sources needed to build and run software based on these systems.

- source paranut/settings.sh:
    ```
    $ source /opt/paranut/settings.sh # assumes you installed to /opt/paranut 
    ```

- Create a directory for the "Hello World" software and copy the example code to it:

    ```
    $ mkdir hello_system
    $ cd hello_system
    $ cp -a $PARANUT_HOME/sw/hello_newlib .
    ```

- Compile the "Hello World" program using the RISC-V toolchain and run it in the *ParaNut* simulator:

    ```
    $ cd hello_newlib
    $ make build
    $ make sim
    ```

- To get more information on the options and capabilities of the simulator, just run:

    ```
    $ pn-sim -h
    ```

### c) Running "Hello World" on a customized *ParaNut* simulator

This method will create a folder *paranut* containing a default *ParaNut* configuration, a *Makefile* and some more files describing the *ParaNut* system.
You may adapt the configuration by editing *config.mk* and/or add Wishbone-compliant peripherals to the system.

- Install the *ParaNut* tools on your system, create a directory for the "Hello World" software and copy the example code to it as described above in Section b).

- Change into the directory you just created (e.g. hello_system).

- Create a local *ParaNut* project and compile your own simulator:

    ```
    $ pn-newproject -c paranut
    $ make -C paranut
    ```

- Compile the "Hello World" program using the standard RISC-V tools:

    ```
    $ cd hello_newlib
    $ make
    ```

- Run the program in your own simulator:

    ```
    $ ../paranut/hw/sysc/pn-sim hello_newlib
    ```
    If you use `make sim` instead of specifying the path to your simulator binary,
    the default pn-sim binary from inside your ParaNut installation will be used.

- To get more information on the options and capabilities of the customized simulator, just run:

    ```
    $ ../paranut/hw/sim/pn-sim -h
    ```
Intel Compiler for SystemC
===============
As previously mentioned, we are currently in the process of moving to the [ICSC](https://github.com/intel/systemc-compiler).
The ParaNut project used to rely on Xilinx Vivado for high level synthesis, as well as hardware synthesis. It is the 
aim of the ParaNut Project to eventually be designed to synthesize using only open source tools. As a first step, ICSC 
will be used for high level synthesis. Additionally, to being open source, the tool supports a major part of the 
SystemC subset, enabling us to remove VHDL entirely from the ParaNut project and use the same SystemC sources for 
hardware synthesis and simulation. Only exception to this are modules that represent Block RAM. For these modules 
we provide a verilog implementation in order to enable the Xilinx hardware synthesis to realize them using Block RAM.     


## Appendix A. Other interesting applications

There are a few other applications present in this repository.

For example:

* [An SDL demo](./sw/applications/sdl_demo/README.md)
* [An example using FreeRTOS](./sw/os/freertos_hello/README.md)

Please refer to their READMEs for more information.

## Appendix B. Additional hardware modules

There are additional hardware modules:

* [UART 16750](./hw/sysc/module_uart/README)
* [GPIO Module](./hw/sysc/module_gpio/README)

Please refer to their READMEs for more information.

## Appendix C. Additional hardware modules
Due to the recent changes not all software modules are up-to-date or able to run in the simulator. 
The following software sources are technically still able to run on pn-sim. 
Their build-setup is however not yet set up to meet the new project structure and may need manual adaptation in order to work properly.

Software sources that currently do not build natively:
* sw:
    * os:
        * linux

Software sources that are not supported in the simulator:
* sw:
    * applications:
        * bluetooth-implant_demo
        * sdl_demo
    * test_applications:
        * uart_test

## Appendix D. The config creator
The config creator was developed to easily create a ParaNut configuration file.
This tool replaces the config.mk file in the ParaNut root directory.
This behavior impacts all ParaNut systems and may not be desired.
To circumvent this when using the config creator, make a backup copy of [config.mk](./config.mk) and replace 
the config.mk inside the ParaNut System you are using, with the output of 
the config creator. 
