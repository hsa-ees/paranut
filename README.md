# The ParaNut Processor (coming soon)

The *ParaNut* processor a customizable, highly scalable, and RISC-V compatible processor architecture for FPGA-based systems. 

A key aspect of the ParaNut architecture is a special concept of parallelism, which combines advantages of SIMD vectorization and simultaneous multi-threading in one architecture. At the same time, the complexity of a single computing core is minimized in order to save area and power. Speculation techniques are generally avoided in order to save power and to make the processor robust against security flaws.

The design is presently used in education and research. In addition, the present implementation passes the RISC-V compliance tests (RV32IM instructions) and is thus compatible with the standard RISC-V toolchain. Preliminary experiments on a Xilinx 7 platform running the *CoreMark* benchmark reveals an almost perfect speedup of 3.97 for a 4-core processor and a speedup of 7.6 for a *ParaNut* with 8-cores.

The ParaNut hardware is modeled completely in *SystemC* and in general, the same code model is used for hardware synthesis as well as for building a cycle-accurate instruction set simulator. Only some performance-critical modules are implemented in VHDL. This ensures that the simulator reflects the real hardware behaviour. The simulator supports the development and debugging of both hardware and software through the ability to produce VCD trace files to inspect the inner workings of the processor and through an OpenOCD-compatible remote-bitbang (RBB) interface.

A list of publications and further information on the project can be found on the home page of the [Efficient Embedded Systems](https://ees.hs-augsburg.de) group at the University of Applied Sciences Augsburg.
