The ParaNut Processor
=====================

The *ParaNut* processor a customizable, highly scalable, and RISC-V compatible processor architecture for FPGA-based systems.

Key aspects of the *ParaNut* architecture are:

* **Scalability and Parallelism:** A special concept of parallelism combines the advantages of SIMD vectorization and simultaneous multi-threading in one architecture with a simple programming model.

* **Security:** Speculation techniques are generally avoided in order to make the processor robust against security flaws.

* **SystemC** as the primary language: The *ParaNut* hardware is modeled in *SystemC* and in general, the same code model is used for hardware synthesis as well as for building a cycle-accurate instruction set simulator. Only some performance-critical modules are implemented in VHDL. This ensures that the simulator reflects the real hardware behaviour.

The design is presently used in education and research. Experiments on a Xilinx 7 platform running the *CoreMark* benchmark reveals an almost perfect speedup of 3.97 for a 4-core processor and a speedup of 7.6 for a *ParaNut* with 8 CPU cores (as of Feb. 2020).

The simulator supports hardware-software co-design through the ability to produce VCD trace containing signals of custom peripherals and of the *ParaNut* processor itself.

Software debugging is possible using the simulator together with GDB through an *OpenOCD*-compatible interface.

A list of publications and further information on the project can be found on the home page of the [Efficient Embedded Systems](https://ees.hs-augsburg.de) group at the University of Applied Sciences Augsburg.


Nov. 22, 2021

- Felix Wagner (<felix.wagner1@hs-augsburg.de>)
- Christian H. Meyer (<christian.meyer@hs-augsburg.de>)
- Nico Borgsmüller (<nico.borgsmueller@hs-augsburg.de>)
- Mark Endres (<mark.endres@HS-Augsburg.de>)
- Patrick Zacharias (<patrick.zacharias@hs-augsburg.de>)
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

For the *SystemC* model and *ParaNut* simulator:

- *SystemC* 2.2 or later ([latest release](https://www.accellera.org/downloads/standards/systemc))

For the VHDL model:

- GHDL / GtkWave for simulation
- a synthesis tool (tested with Xilinx Vivado v2017.2 and 2019.1)
- an evaluation board (ZYBO 7010 or 7020)

## 2. Setting up the environment

- Set up your terminal by sourcing the *settings.sh* (or *settings.zsh* if you use a *zshell*):
    ```
    $ source settings.sh
    ```

- Make sure *SystemC* is installed on your system. If *SystemC* has been installed manually, the *SYSTEMC_HOME* environment variable must point to the top level of the *SytemC* installation (e.g. */opt/systemc*).
    ```
    $ export SYSTEMC_HOME=/opt/systemc   # adapt according to your installation
    ```

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
    $ sudo make install PREFIX=/opt/paranut   # adapt the installation path if desired
    ```

- Create a directory for the "Hello World" software and copy the example code to it:

    ```
    $ mkdir hello_system
    $ cd hello_system
    $ cp -a /opt/paranut/src/sw/hello_newlib .
    ```

- Compile the "Hello World" program using the RISC-V toolchain and run it in the *ParaNut* simulator:

    ```
    $ cd hello_newlib
    $ make PARANUT=/opt/paranut/src   # this assumes PREFIX=/opt/paranut during installation
    $ pn-sim hello_newlib
    ```

- To get more information on the options and capabilities of the simulator, just run:

    ```
    $ pn-sim
    ```

### c) Running "Hello World" on a customized *ParaNut* simulator

This method will create a folder *paranut* containing a default *ParaNut* configuration, a *Makefile* and some more files describing the *ParaNut* system.
You may adopt the configuration by editing *config.mk* and/or add Wishbone-compliant peripherals to the system.

- Install the *ParaNut* tools on your system, create a directory for the "Hello World" software and copy the example code to it as described above in Section b).

- Change into the system directory `hello_system`.

- Create a local *ParaNut* project and compile your own simulator:

    ```
    $ pn-newproject -c paranut
    $ make -C paranut
    ```

- Compile the "Hello World" program using the standard RISC-V tools:

    ```
    $ cd hello_newlib
    $ make PARANUT=../paranut
    ```

- Run the program in your own simulator:

    ```
    $ ../paranut/hw/sim/pn-sim hello_newlib
    ```

- To get more information on the options and capabilities of the customized simulator, just run:

    ```
    $ ../paranut/hw/sim/pn-sim
    ```

## 4. Operating a *ParaNut* on real hardware

The following steps generate a *Xilinx Vivado* project for the *ZYBO* evaluation board:

- Check and adapt configuration in config file (optional):

    ```
    $ nano config.mk
    ```

- Make sure Xilinx Vivado tools are in the path (check systems/refdesign/README).

- To change the used board refer to the README inside the systems/refdesign folder

    ```
    $ cd systems/refdesign
    $ make build PARANUT=../..
    $ make run
    ```
