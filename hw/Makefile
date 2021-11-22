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


PREFIX ?= /opt/paranut

################################################################################
#                                                                              #
#     Default and help targets                                                 #
#                                                                              #
################################################################################


# Default target ...
.PHONY: all
all: help


# Print usage info ...
.PHONY: help
help:
	@echo "This makefile is mainly used to install HW sources to your system. Usually there sould be no need for you to use it."
	@echo
	@echo "Usage: make [<target>] [<parameter>=<value> ...]"
	@echo
	@echo "Targets:"
	@echo
	@echo "  install     : Install hardware source files to the directory given by parameter PREFIX"
	@echo "                (default: /opt/paranut)"


################################################################################
#                                                                              #
#     Installing                                                               #
#                                                                              #
################################################################################

# SystemC model files
SRC_FILES =  sysc/Makefile \
             sysc/base.cpp \
             sysc/dm.cpp \
             sysc/exu.cpp \
             sysc/csr.cpp \
             sysc/ifu.cpp \
             sysc/intc.cpp \
             sysc/interconnect.cpp \
             sysc/jtag_dtm.cpp \
             sysc/lfsr.cpp \
             sysc/lsu.cpp \
             sysc/memory.cpp \
             sysc/memu.cpp \
             sysc/mextension.cpp \
             sysc/nut.cpp \
             sysc/paranutsystem.cpp \
             sysc/remote_bitbang.cpp \
             sysc/base.h \
             sysc/paranut-config.template.h \
             sysc/debug_rom.h \
             sysc/dm.h \
             sysc/exu.h \
             sysc/csr.h \
             sysc/exu_csr.h \
             sysc/ifu.h \
             sysc/intc.h \
             sysc/interconnect.h \
             sysc/jtag_dtm.h \
             sysc/lfsr.h \
             sysc/lsu.h \
             sysc/memory.h \
             sysc/memu.h \
             sysc/mextension.h \
             sysc/nut.h \
             sysc/paranut-sim.h \
             sysc/paranutsystem.h \
             sysc/paranut-peripheral.h \
             sysc/remote_bitbang.h \
             sysc/dm.tcl \
             sysc/exu.tcl \
             sysc/csr.tcl \
             sysc/ifu.tcl \
             sysc/intc.tcl \
             sysc/jtag_dtm.tcl \
             sysc/lsu.tcl \

# Hardware tools files
SRC_FILES += tools/Makefile.inc \
             tools/paranut.tcl \

SRC_FILES_EXEC += tools/generate_mimpid.sh \

# VHDL files
SRC_FILES += vhdl/ParaNut.vhd \
             vhdl/swb2maxi.vhd \
             vhdl/saxi2mwb.vhd \
             vhdl/paranut/paranut.vhd \
             vhdl/paranut/dbg.vhd \
             vhdl/paranut/lsu.vhd \
             vhdl/paranut/mexu.vhd \
             vhdl/paranut/mcsr.vhd \
             vhdl/paranut/types.vhd \
             vhdl/paranut/memu_lib.vhd \
             vhdl/paranut/paranut-fdeps.mk \
             vhdl/paranut/tb_monitor.vhd \
             vhdl/paranut/mdbg.vhd \
             vhdl/paranut/mtagram.vhd \
             vhdl/paranut/exu.vhd \
             vhdl/paranut/csr.vhd \
             vhdl/paranut/mem_tech.vhd \
             vhdl/paranut/memu.vhd \
             vhdl/paranut/ifu.vhd \
             vhdl/paranut/text_io.vhd \
             vhdl/paranut/mlsu_simple.vhd \
             vhdl/paranut/mwriteport.vhd \
             vhdl/paranut/mintc.vhd \
             vhdl/paranut/mifu_bs.vhd \
             vhdl/paranut/paranut_config.template.vhd \
             vhdl/paranut/mem_inferred.vhd \
             vhdl/paranut/mbankram.vhd \
             vhdl/paranut/marbiter.vhd \
             vhdl/paranut/txt_util.vhd \
             vhdl/paranut/mhistogram.vhd \
             vhdl/paranut/mlsu.vhd \
             vhdl/paranut/mifu.vhd \
             vhdl/paranut/histogram.vhd \
             vhdl/paranut/lfsr.vhd \
             vhdl/paranut/mparanut.vhd \
             vhdl/paranut/mreadport.vhd \
             vhdl/paranut/intc.vhd \
             vhdl/paranut/mbusif.vhd \
             vhdl/paranut/paranut_lib.vhd \
             vhdl/paranut/mmemu.vhd

# Install 
.PHONY: install
install:
	@echo
	@echo "### Installing HW sources to $(PREFIX) ###"
	@if [ $(PREFIX) ]; then \
	  # Install SRC_FILES \
	  for FILE in $(SRC_FILES); do \
	    install -Dp -m 644 -t $(PREFIX)/src/hw/`dirname $$FILE` $$FILE; \
	  done; \
	  for FILE_EXEC in $(SRC_FILES_EXEC); do \
	    install -Dp -m 775 -t $(PREFIX)/src/hw/`dirname $$FILE_EXEC` $$FILE_EXEC; \
	  done; \
	  # Install libparanutsim \
	  $(MAKE) -C sysc install SUFFIX=libparanutsim; \
	  # Install sim (pn-sim) \
	  $(MAKE) -C sim install; \
	fi
