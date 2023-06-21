# Tools
## Directory Contents
- bin: binaries for working with the ParaNut
- etc: mainly files used for configuring/setting up a debugger
- gdbtest: gnu debugger test
- lib: contains libtnf.a
- share: files used for pn-newproject and to check environment variables
- src: source code for Zybo-board-ARM-Firmware
- toolchain: install helper for RISCV-Toolchain
- *-base.mk: see section "Build Automation"

## Build Automation
This section describes the concept behind the Build Automation present in the 
ParaNut Project. It revolves around the idea of centralizing and reusage, and 
covers all ParaNut components.

### Directory Structure
As the ParaNut Project is continually under development, changes to the 
directory structure, the names and locations of important files and other 
information needed in many of the build tools present in the project's files, 
a central file called directory-base.mk provides this information to most 
makefiles and shell scripts inside the project directory.
Most of the variables used in the makefiles throughout the project are defined 
there.


### ParaNut Components
The whole ParaNut Project can be sperated into four central components:
* SystemC Modules:
* Hardware Abstraction Libraries
* RISC-V Applications
* ParaNut Systems

### Base Makefiles
Each of the first three components requires a distinct and specialized build 
automation. Hence, this folder contains three matching *-base.mk files:
* sysc-base.mk
* hal-base.mk
* application-base.mk

The fourth component is used to collect artifacts created by these three and 
provide additional information such as a specific hardware configuration to 
the build process. It will at some point be used to synthesize for a specific 
hardware platform. As this is currently not supported, this component does not 
have a base makefile yet.

Each base makefile is intended to be used by being imported into another makefile, 
which in turn provides necessary configuration options such as source locations and 
filenames. For simple modules only very few options must be set, the base makefiles 
however do not hinder you from providing deeper configuration or overwriting certain 
targets in order to adapt the build automation.
All this is done so that changes to the tools used to create certain artifacts must only be made in a central file and not in each an every makefile in the project.


