# The ParaNut Processor 

The *ParaNut* processor a customizable, highly scalable, and RISC-V compatible processor architecture for FPGA-based systems. 

A key aspect of the ParaNut architecture is a special concept of parallelism, which combines advantages of SIMD vectorization and simultaneous multi-threading in one architecture. At the same time, the complexity of a single computing core is minimized in order to save area and power. Speculation techniques are generally avoided in order to save power and to make the processor robust against security flaws.

The design is presently used in education and research. In addition, the present implementation passes the RISC-V compliance tests (RV32IM instructions) and is thus compatible with the standard RISC-V toolchain. Preliminary experiments on a Xilinx 7 platform running the *CoreMark* benchmark reveals an almost perfect speedup of 3.97 for a 4-core processor and a speedup of 7.6 for a *ParaNut* with 8-cores.

The ParaNut hardware is modeled completely in *SystemC* and in general, the same code model is used for hardware synthesis as well as for building a cycle-accurate instruction set simulator. Only some performance-critical modules are implemented in VHDL. This ensures that the simulator reflects the real hardware behaviour. The simulator supports the development and debugging of both hardware and software through the ability to produce VCD trace files to inspect the inner workings of the processor and through an OpenOCD-compatible remote-bitbang (RBB) interface.

A list of publications and further information on the project can be found on the home page of the [Efficient Embedded Systems](https://ees.hs-augsburg.de) group at the University of Applied Sciences Augsburg.


Feb. 24, 2020 - Alexander Bahle <alexander.bahle@hs-augsburg.de> & Gundolf Kiefer <gundolf.kiefer@hs-augsburg.de>


## Directory Contents
------------------

./doc           - Documentation (e.g. Architecture Reference Manual)
./sysc          - SystemC model
./hw            - Hardware implementation
./sw            - Test software and examples for the ParaNut
./systems       - Contains code and makefiles for ParaNut FPGA systems
./tools         - Tools for development and OpenOCD configuration files
config          - Configuration file for SystemC and hardware implementation
Makefile        - Top level makefile
settings.sh     - Settings for the developing environment  


## Licensing
---------

(see LICENSE or license notes in the respective directories / files)


## Contributors & Contact
----------------------

Alexander Bahle <alexander.bahle@hs-augsburg.de>
Anna Pfuetzner <annakerstin.pfuetzner@gmail.com>
Gundolf Kiefer <gundolf.kiefer@hs-augsburg.de>
Michael Seider <michael.seider@ftst.de>
Michael Schaeferling <michael.schaeferling@hs-augsburg.de>


Getting Started
===============

## 1. Prerequisites
----------------

For software compilation/debug:
- RISC-V GNU Toolchain (riscv64-unknown-elf-gcc) (latest release: https://github.com/sifive/freedom-tools/releases)
- RISC-V OpenOCD (riscv-openocd) (latest release: https://github.com/sifive/freedom-tools/releases)

For the SystemC model:
- SystemC 2.2 or later (latest release: https://www.accellera.org/downloads/standards/systemc)

For the VHDL model:
- GHDL / GtkWave or Xilinx ISim for simulation
- a synthesis tool (tested with Xilinx Vivado v2017.2)
- an evaluation board (ZYBO 7010 or 7020)

## 2. Setting up the environment 
-------------------------------------------------------------
- Set up your terminal by sourcing the settings.sh 

```
 source settings.sh
```

## 3. Building the SystemC model (ISS)
-------------------------------------------------------------

- (Optional) Check and adapt configuration in config file
  `nano config`
- Make sure SYSTEMC_HOME environment variable is set and points to
  the top level of your SytemC installation (e.g. /opt/SystemC)

```
 make sim-sysc
```

## 4. Running "hello_world" on a ParaNut using the SystemC model (ISS)
-------------------------------------------------------------

- (Optional) Check and adapt configuration in config file:
  `nano config`
- Make sure SYSTEMC_HOME environment variable is set and points to
  the top level of your SytemC installation (e.g. /opt/SystemC)
- Make sure the RISC-V GNU Toolchain is in the path 
  (the following command assumes that the $RISCV environment variable
  points to the installation directory of the tool chain):
    `export PATH=$RISCV/bin:$PATH`

```
 cd sw/hello_newlib
 make sim
```

To build your own software for a ParaNut processor check the
ParaNut manual (doc/paranut_manual.pdf)


## 5. Running a ParaNut on real hardware
-------------------------------------------------------------
 
Generate a Vivado Project for the ZYBO board:
- (Optional) Check and adapt configuration in config file:
	` nano config `
- Make sure Xilinx Vivado tools are in the path (check systems/refdesign/README)
- To change the used board refer to the README inside the systems/refdesign folder

```
 cd systems/refdesign
 make build
 make run
```


