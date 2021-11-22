#  -----------------------------------------------------------------------------
#
#  This file is part of the ParaNut project.
#
#  Copyright (C) 2010-2021 Alexander Bahle <alexander.bahle@hs-augsburg.de>
#                          Gundolf Kiefer <gundolf.kiefer@hs-augsburg.de>
#                          Felix Wagner <felix.wagner1@hs-augsburg.de>
#                          Michael Schaeferling <michael.schaeferling@hs-augsburg.de>
#      Hochschule Augsburg, University of Applied Sciences
#
#  Description:
#    ParaNut top-level Makefile.
#
#  --------------------- LICENSE -----------------------------------------------
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
#  -----------------------------------------------------------------------------





################################################################################
#                                                                              #
#     Settings                                                                 #
#                                                                              #
################################################################################


# There are two main locations of ParaNut files, which can be referenced by these environment variables:
# - PARANUT is to be set within a project / specific make files to point to the ParaNut (editable) sources to work with.
# - PARANUT_TOOLS points to the (static) tools and helpers and is set when sourcing the ParaNut settings.[z]sh file.
#
# Both of these environment variables have different meanings depending on the stage/place you work with ParaNut:
#  a) Source tree (e.g. obtained by a git clone from the ParaNut repository):
#     - PARANUT -> source tree
#     - PARANUT_TOOLS -> source tree
#
#  b) Installation (not to be used directly, as one usually creates a project from here and works within the project, see below):
#     - (PARANUT -> installation, usually not used)
#     - (PARANUT_TOOLS -> installation, usually not used)
#
#  c) Project (created by 'pn-newproject'):
#     - PARANUT -> project
#     - PARANUT_TOOLS -> installation



# Software projects and systems to build/clean ...
PN_SW_DIRS = coremarkv1.0 dhrystone dhrystone2.2 libparanut mode1_demo test_riscv hello_newlib libparanut_unittest mode2_demo

# ParaNut Xilinx IP core target directory
IP_XILINX_TARGET_DIR ?= paranut_ip

# Installation target directory ...
PREFIX ?= /opt/paranut

# This Makefile delegates work, thus we need to avoid relative paths:
PREFIX_ABS = $(abspath $(PREFIX))




################################################################################
#                                                                              #
#     Default and help targets                                                 #
#                                                                              #
################################################################################


# Default target ...
.PHONY: all
all: build


# Update version file
version.env: update-version

.PHONY: update-version
update-version:
	@echo "Updating version file ..."
	@if [ -d .git ]; then \
	  echo "# Git describe" > version.env.new; \
	  echo "PN_GITVERSION="$(shell git describe --tags --long --dirty='*')  >> version.env.new; \
	  echo "# MIMPID Version Value" >> version.env.new; \
	  echo "CFG_NUT_MIMPID="$(shell hw/tools/generate_mimpid.sh) >> version.env.new; \
	else \
	  echo "# Git describe" > version.env.new; \
	  echo "PN_GITVERSION=0.0-0*" >> version.env.new; \
	  echo "# MIMPID Version Value" >> version.env.new; \
	  echo "CFG_NUT_MIMPID=0x000001" >> version.env.new; \
	fi
	@diff -q version.env version.env.new > /dev/null 2>&1 || mv version.env.new version.env; \
	rm -f version.env.new; \


# Print usage info ...
.PHONY: help
help:
	@echo
	@echo "Usage: make [<target>] [<parameter>=<value> ...]"
	@echo
	@echo "Targets:"
	@echo
	@echo "  build       : Build SystemC simulator and the ParaNut support library,"
	@echo "                prepare for installation"
	@echo "  install     : Install ParaNut to the directory given by parameter PREFIX"
	@echo "                (default: /opt/paranut)"
	@echo "  clean       : Delete most files created during the build process"
	@echo "  veryclean   : Delete all files created during the build process"
	@echo
	@echo "  help        : Print this help"
	@echo
	@echo "  hello       : Build and simulate an example ParaNut program"
	@echo "                (from 'sw/hello_newlib')"
	@echo
	@echo "  build-all   : Build everything, including software projects and systems"
	@echo "  clean-all   : Delete most files created during the build process, including software projects and systems"
	@echo "  veryclean-all   : Delete all files created during the build process, including software projects and systems"
	@echo
	@echo "  build-sim         : Build the SystemC simulator ('libparanutsim.a' and 'pn-sim')"
	@echo "  build-lib         : Build the ParaNut support library ('libparanut.a')"
	@echo "  build-ip-xilinx   : Build a ParaNut IP core for Xilinx FPGAs"
	@echo "  build-sw          : Build all software projects (as defined by PN_SW_DIRS)"
	@echo "  build-systems     : Build all systems (may be defined by PN_SYSTEMS,"
	@echo "                      defaults to the ones declared in 'systems/Makefile')"
	@echo
	@echo "  test-sim          : Run test to verify basic functionality of simulation"
	@echo "  test-lib          : Run unittests to verify libparanut functionality"
	@echo "  test-vhdl         : Run testbenches to test vhdl code"
	@echo "  test-sysc         : Run testbenches to test sysc code"
	@echo
	@echo "Parameters:"
	@echo
	@echo "  PN_SW_DIRS           : Space-separated list of software directories that will be cleaned"
	@echo "  PN_SYSTEMS           : Space-separated list of systems (e.g. for target 'build-systems')"
	@echo "  PREFIX               : Target directory for make install"
	@echo "  IP_XILINX_TARGET_DIR : ParaNut Xilinx IP core target directory (for target 'build-ip-xilinx'),"
	@echo "                         default directory is 'hw/tools/paranut_ip'."


# Build and run "hello world" example ...
PHONY: hello
hello: build
	$(MAKE) -C sw/hello_newlib sim





################################################################################
#                                                                              #
#     Building: Simulator, Support Libraries                                   #
#                                                                              #
################################################################################


# Build everything except vendor-specific synthesis products ...
.PHONY: build
build: build-sim build-lib


.PHONY: build-sim
build-sim: config.mk
	@echo
	@echo "######### Building SystemC Simulator #########";
	@echo
	$(MAKE) -C hw/sim build


# Build the ParaNut support library/libraries ...
.PHONY: build-lib
build-lib:
	@echo
	@echo "######### Building ParaNut Support Library #########";
	@echo
	+$(MAKE) -C sw/libparanut





################################################################################
#                                                                              #
#     Building: Extra and Vendor-Specific Targets                              #
#                                                                              #
################################################################################


# Build the ParaNut Xilinx IP core (to IP_XILINX_TARGET_DIR directory) ...
.PHONY: build-ip-xilinx
build-ip-xilinx:
	@echo
	@echo "######### Generating ParaNut IP core for Xilinx FPGAs #########"
	@echo
	# Execute all HLS targets first
	+$(MAKE) -C hw/sysc copy
	# Build the IP core
	+$(MAKE) -C hw/tools -f Makefile.inc $@ IP_XILINX_TARGET_DIR=$(IP_XILINX_TARGET_DIR)



# Build the ParaNut support library/libraries ...
.PHONY: build-systems
build-systems:
	@echo
	@echo "######### Building ParaNut Systems #########";
	@echo
	+$(MAKE) -C systems build





################################################################################
#                                                                              #
#     Installing                                                               #
#                                                                              #
################################################################################

# Settings files
ROOT_FILES = settings.sh settings.zsh

# Top-Level info, config and Makefile
SRC_FILES = README.md \
            config.mk \
            Makefile


# Install the ParaNut repository to the directory specified by PREFIX ...
.PHONY: install
install: build update-version
	@echo 
	@echo "######### Installing to $(PREFIX) #########" 
	@echo 
	@if [ -d src ]; then \
	  echo "INFO: The install target is only available inside the source repository!";\
	else \
	  # Create target dir \
	    mkdir -p $(PREFIX_ABS) \
	  # Install ROOT_FILES \
	    for FILE in $(ROOT_FILES); do \
	      install -Dp -m 644 -t $(PREFIX_ABS)/`dirname $$FILE` $$FILE; \
	    done; \
	  # Install SRC_FILES \
	    for FILE in $(SRC_FILES); do \
	      install -Dp -m 644 -t $(PREFIX_ABS)/src/`dirname $$FILE` $$FILE; \
	    done; \
	  # Install DOCUMENTATION \
	    $(MAKE) -C doc-src install PREFIX=$(PREFIX_ABS); \
	  # Install SW \
	    $(MAKE) -C sw install PREFIX=$(PREFIX_ABS); \
	  # Install HW \
	    $(MAKE) -C hw install PREFIX=$(PREFIX_ABS); \
	  # Install tools \
	    $(MAKE) -C tools install PREFIX=$(PREFIX_ABS); \
	  # Add version.env \
	    cp version.env  $(PREFIX_ABS)/src/version.env; \
	fi





################################################################################
#                                                                              #
#     Testing                                                                  #
#                                                                              #
################################################################################

# test libparanut
PHONY: test-lib
test-lib:
	$(MAKE) -C sw/libparanut_unittest;

# test basic functionality of simulator
PHONY: test-sim
test-sim: test-hello_newlib

# run sysc test benches
PHONY: test-sysc
test-sysc:
	$(MAKE) -C hw/sysc/tb run;


# run vhdl test benches
PHONY: test-vhdl
test-vhdl:
	$(MAKE) -C hw/vhdl/tb build;





################################################################################
#                                                                              #
#     Cleaning                                                                 #
#                                                                              #
################################################################################

# Clean up most files created during the build process
PHONY: clean
clean:
	+$(MAKE) -C hw/sysc clean
	+$(MAKE) -C hw/sim clean
	@if [ -d doc-src ]; then \
	  $(MAKE) -C doc-src clean; \
	fi;
	@for DIR in $(PN_SW_DIRS); do \
	  if [ -d $$DIR ]; then $(MAKE) -C $$DIR PORT_DIR=paranut clean; fi; \
	done;
	$(MAKE) -C sw/libparanut clean;
	rm -f version.env

# Clean up *everything* created during the build process
PHONY: veryclean
veryclean: clean
	rm -rf hw/tools/$(IP_XILINX_TARGET_DIR)


# Clean up most files, including software and systems directories ...
PHONY: clean-all
clean-all: clean
	+$(MAKE) -C hw/sysc clean-all
	@if [ -d systems ]; then \
	  $(MAKE) -C systems clean; \
	fi;

# Clean up *everything*, including software and systems directories ...
PHONY: veryclean-all
veryclean-all: clean-all veryclean
	@if [ -d systems ]; then \
	  $(MAKE) -C systems veryclean; \
	fi;
