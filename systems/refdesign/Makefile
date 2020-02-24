#  This file is part of the ParaNut project.
# 
#  Copyright (C) 2019-2020 Alexander Bahle <alexander.bahle@hs-augsburg.de>
#                          Michael Schaeferling <michael.schaeferling@hs-augsburg.de>
#      Hochschule Augsburg, University of Applied Sciences
#
#  Description:
#    Main Makefile for the ParaNut demo system
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

# Set the board here (see folders in hardware/boards for valid options)
BOARD?=zybo

DIR_THIS=$(shell basename $(CURDIR))
DIR_VIVADO_CORES=vivado_cores
DIR_HW_BIN=$(PARANUT_HOME)/hw/bin

SOFTWARE_SRC?=hello_newlib

ifndef PARANUT_HOME
$(error PARANUT_HOME is not set)
endif

# The lazy target: 
# Do everything automatically, no more interaction needed.
# Just make sure to connect the board, as it will be programmed right after hardware and software are built.
.PHONY: all
all: build_and_run

 
.PHONY: software 
software: software/Makefile

software/Makefile:
	cp $(PARANUT_HOME)/sw/$(SOFTWARE_SRC)/* software/

# Generate the PARANUT IP core for Xilinx Vivado 
.PHONY: paranut_core
paranut_core: 
	# Remove old PARANUT core and create folder 
	rm -rf ${DIR_VIVADO_CORES}/PARANUT 
	mkdir -p ${DIR_VIVADO_CORES} 
	# Copy configuration to top-level
	cp -f hardware/boards/$(BOARD)/config $(PARANUT_HOME)/config
	# Genearate IP core 
	+$(MAKE) -C $(PARANUT_HOME) ip IP_TARGET_DIR=$(abspath  $(DIR_VIVADO_CORES)/PARANUT)
	
	
# Generate and build the system (hardware + software):
.PHONY: build_system
build_system:
	$(MAKE) -C hardware BOARD=${BOARD} all
	+$(MAKE) -C software all


# Run the demo on the board:
.PHONY: run
run:
	+$(MAKE) -C software flash-$(DIR_THIS)-bit


.PHONY: build_and_run
build_and_run:
	+$(MAKE) paranut_core
	+$(MAKE) build_system
	+$(MAKE) run

.PHONY: build
build: software
	+$(MAKE) paranut_core 
	+$(MAKE) build_system 

.PHONY: clean
clean:
	# clean vivado_cores and demo software:
	rm -rf ${DIR_VIVADO_CORES}
	$(MAKE) -C hardware BOARD=${BOARD} clean
	# clean software:
	$(MAKE) -C software clean

# Remove copied software 
.PHONY: clean-software
clean-software:
	find software -not -name "software" -not -name 'README' -delete
