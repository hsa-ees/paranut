#  This file is part of the ParaNut project.
#
#  Copyright (C) 2019-2021 Alexander Bahle <alexander.bahle@hs-augsburg.de>
#                          Michael Schaeferling <michael.schaeferling@hs-augsburg.de>
#      Hochschule Augsburg, University of Applied Sciences
#
#  Description:
#    This makefile provides targets to generate and build the
#    ParaNut demo hardware project, based on a given blockdesign.
#    This makefile is to be called from within the directory
#    where the project is to be built:
#     usually, within 'hardware' call
#     > make -f ./Makefile
#     or just
#     > make
#
#  Redistribution and use in source and binary forms, with or without modification,
#  are permitted provided that the following conditions are met:
#
#  1. Redistributions of source code must retain the above copyright notice, this
#     list of conditions and the following disclaimer.
#
#  2. Redistributions in binary form must reproduce the above copyright notice,
#     this list of conditions and the following disclaimer in the documentation and/or
#     other materials provided with the distribution.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
#  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
#  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
#  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
#  ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
#  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
#  ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
#  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
#  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


BUILD_DIR=./build
VIVADO_PROJECT=$(BUILD_DIR)/system.xpr

FIRMWARE_SRC=$(wildcard boards/${BOARD}/*.c)
FIRMWARE=$(patsubst boards/${BOARD}/%.c,%,$(FIRMWARE_SRC))

ifeq ($(BOARD), )
$(error BOARD is not set)
endif

ifeq ($(FIRMWARE), )
	# No firmware for selected board
	FIRMWARE_PRJ=
	FIRMWARE_ELF=
else
	# Generate/build firmware for selected board
	FIRMWARE_PRJ=$(FIRMWARE)/Makefile
	FIRMWARE_ELF=$(FIRMWARE)/$(FIRMWARE).elf
endif


.PHONY: all
all: build_project


# The target 'generate_project' calls a board dependent Makefile to
# generate the appropriate Vivado system
.PHONY: generate_project
generate_project: $(VIVADO_PROJECT)
$(VIVADO_PROJECT):
	+$(MAKE) -f boards/$(BOARD)/Makefile generate_project


# The target 'build_project' runs the steps to generate a bitfile for a
# previously generated Vivado project (either "hand-made" using Vivado-GUI
# or the 'generate' target from this Makefile).
.PHONY: build_project
build_project: $(VIVADO_PROJECT)
	( echo "open_project $(VIVADO_PROJECT)"; \
	echo "set ret 0"; \
	echo "reset_project"; \
	echo "launch_runs impl_1 -to_step write_bitstream -jobs 4"; \
	echo "wait_on_run impl_1"; \
	echo "if {[get_property PROGRESS [get_runs impl_1]] != \"100%\"} { set ret 1 }"; \
	echo "close_project"; \
	echo 'exit $$ret' ) \
	| vivado -mode tcl
	# Create symbolic links to bitfile and hardware definition
	ln -s -f system.runs/impl_1/design_1_wrapper.bit $(BUILD_DIR)/system.bit
	ln -s -f system.runs/impl_1/design_1_wrapper.sysdef $(BUILD_DIR)/system.hdf
	rm -f vivado*.log vivado*.jou
	# Generate and build firmware project
	ln -s -f $(PARANUT_TOOLS)/src/firmware.c boards/$(BOARD)/firmware.c
	+$(MAKE) build_firmware

$(BUILD_DIR)/system.%:
	+$(MAKE) build_project

# Special targets for board specific software (Firmware for ARM cores on ZYNQ 7000 Boards)
.PHONY: generate_firmware
generate_firmware: $(FIRMWARE_PRJ)

$(FIRMWARE)/Makefile: $(BUILD_DIR)/system.hdf
	rm -rf $(FIRMWARE)
	pn-xil-swtool -a $(BUILD_DIR)/system.hdf $(FIRMWARE) hello_world
	rm -rf $(FIRMWARE)/hello_world_bsp/ $(FIRMWARE)/helloworld.c
	ln -s ../$(FIRMWARE_SRC) $(FIRMWARE)/$(FIRMWARE).c
	pn-xil-swtool -b $(BUILD_DIR)/system.hdf $(FIRMWARE)/$(FIRMWARE)_bsp ../vivado_cores/
	sed -i.bak s/hello_world_bsp/$(FIRMWARE)_bsp/g $(FIRMWARE)/Makefile
	sed -i.bak s/executable.elf/$(FIRMWARE).elf/g $(FIRMWARE)/Makefile
	sed -i.bak s/'_SIZE : 0x2000'/'_SIZE : 0x2000000'/g $(FIRMWARE)/lscript.ld
	rm -rf pn-xil-swtool.tcl

.PHONY: build_firmware
build_firmware: $(FIRMWARE_ELF)

$(FIRMWARE_ELF): $(FIRMWARE_PRJ)
	$(MAKE) -j1 -C $(FIRMWARE) all


.PHONY: clean
clean:
	rm -f boards/$(BOARD)/firmware.c
	rm -rf $(BUILD_DIR) build.bd
	rm -rf $(FIRMWARE)
	rm -rf .Xil vivado*.jou vivado*.log pn-xil-swtool.tcl
