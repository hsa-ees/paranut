# mode2_demo_rust
Author: Abdurrahman Celep <abdurrahman.celep1@hs-augsburg.de> 

Date: 13.06.2023

## Summary
This application demonstrates a simple rust application, which represents the paralelity of the ParaNut processor.

## Requirements

* Linux x86_64 compatible computer (tested with Debian 11.3)

* Basic development utilities installed (GNU Make, etc.)

* Installed RISC-V toolchain (tested with [SiFive Toolchain 2020.04](https://github.com/sifive/freedom-tools/releases/tag/v2020.04.0-Toolchain.Only)
with riscv64-unknown-elf-gcc and other tools visible to the command line.
(for example by using following command): 

```sh
export PATH=$PATH:$<RISCV_TOOLCHAIN_DIR>/bin/
```
* Also set RISCV_HOME enviroment variable

```sh
export RISCV_HOME=$<RISCV_TOOLCHAIN_DIR>
```

* Installed clang compiler (tested with version 11.0.1-2)

* As well as this repository checked out and libparanut built (```make -C ../libparanut build```).
The path of this repository will be referenced as $paranut_root from now on.

* BASH compatible shell

* rustc Compiler to work with rust Code (tested with version 1.69.0)

* cargo to install the creates (tested with version 1.69)

* rustup the installer for Rust (1.26)

* If your system has no Rust installed, please install the language using

```sh
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
```
* To work with nightly Channel (tested with version 1.69)
```sh
rustup install nightly
```

* add the toolchain with
```sh
rustup component add rust-src --toolchain nightly-x86_64-unknown-linux-gnu
```

* cargo-make to use cargos own Makefile

```sh
cargo install --force cargo-make
```

## Building
In case this application is to be executed on hardware, 
it is recommended to build this application using the same config.mk as the hardware configuration.

To build this application make sure "settings.sh" is still sourced,
as well as the toolchain.

Enter this directory:

```sh
cd ./sw/applications/mode2_demo_rust/ # Relative to $paranut_root
```

next, build using cargo make:

```sh
cargo make build
```

## Running
To test the application simply run following command from within the directory with this README (same as the one used before):
Build the simulator:
```sh
make -C ../../.. build-sim
```
Run the program:
```sh
cargo make sim
```


**EXECUTION ON HARDWARE IS CURRENTLY NOT AVAILABLE...**
Still, this is how you would do it, if it was:
 
In case hardware has been built as mentioned in the main [README file of the ParaNut project](../../README.md#operating-a-paranut-on-real-hardware).
Following command can be used for running the built application on hardware.

```sh
pn-flash -p mode2_demo_rust.bin -b ../../systems/refdesign/hardware/build/system.bit ../../systems/refdesign/hardware/build/system.hdf ../../systems/refdesign/hardware/firmware/firmware.elf
```

