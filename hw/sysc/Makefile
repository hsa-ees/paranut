#  -----------------------------------------------------------------------------
#
#  This file is part of the ParaNut project.
#
#  Copyright (C) 2010-2020 Alexander Bahle <alexander.bahle@hs-augsburg.de>
#                          Gundolf Kiefer <gundolf.kiefer@hs-augsburg.de>
#      Hochschule Augsburg, University of Applied Sciences
#
#  Description:
#    This module contains various types, constants and helper functions
#    for the SystemC model of ParaNut.
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


# include global configuration
CONFIG ?= ../../config.mk
include $(CONFIG)


CONFIG_H = paranut-config.h
CONFIG_H_TEMPLATE = paranut-config.template.h

CFLAGS = -g -MMD -O2 -I$(SYSTEMC_HOME)/include -DSIMBUILD # -Wfatal-errors
DFLAGS ?=
LDFLAGS = -L$(SYSTEMC_HOME)/lib-linux64 -lsystemc # -Wfatal-errors

PARANUT_SRC = base.cpp lfsr.cpp ifu.cpp csr.cpp exu.cpp lsu.cpp memu.cpp intc.cpp dm.cpp jtag_dtm.cpp nut.cpp
PARANUT_OBJ = $(patsubst %.cpp, %.o, $(PARANUT_SRC))

PARANUT_SYSTEM_SRC = $(PARANUT_SRC) memory.cpp remote_bitbang.cpp interconnect.cpp paranutsystem.cpp
PARANUT_SYSTEM_OBJ = $(patsubst %.cpp, %.o, $(PARANUT_SYSTEM_SRC))
PARANUT_SYSTEM_HDR = $(patsubst %.cpp, %.h, $(PARANUT_SYSTEM_SRC)) $(CONFIG_H) paranut-peripheral.h paranut-sim.h exu_csr.h

HLS_SRCS = $(wildcard *.tcl)
HLS_TARGETS = $(patsubst %.tcl, hls-%/solution1/solution1.log, $(HLS_SRCS))
HLS_CP_TARGETS = $(patsubst %.tcl, copy-%, $(HLS_SRCS))
HLS_CP_DIR ?= ../vhdl/paranut

ifeq "$(CFG_EXU_M_EXTENSION)" "1"
	PARANUT_SRC += mextension.cpp
	PARANUT_OBJ += mextension.o
endif

PN_TB_DIRS = dm jtag_dtm memu

.PHONY: all
all: libparanutsim.a

.PHONY: debug
debug: CFLAGS += -g -Og
debug: libparanutsim.a


### Install documentation ###

# Doxygen targets (only needed in source repository)
# Target 'install-doc' is provided in the Doxygen Makefile fragment:
-include Documentation/Doxygen.mk


.PHONY: install
install: libparanutsim.a
	@echo
	@echo "### Install libparanutsim to: $(PREFIX)";
	install -Dp -m 644 -t $(PREFIX)/include/$(SUFFIX) $(PARANUT_SYSTEM_HDR)
	install -Dp -m 644 -t $(PREFIX)/lib/$(SUFFIX) libparanutsim.a


# Automatic dependencies ...
-include $(PARANUT_SYSTEM_OBJ:%.o=%.d)

%.o:	%.cpp $(CONFIG_H)
	$(CXX) -c $(CFLAGS) $(DFLAGS) $<


# Version ...

VERSION_FILE = ../../version.env

$(VERSION_FILE): update-version

.PHONY: update-version
update-version:
	$(MAKE) -C ../../ update-version


# Update local paranut-config.h
$(CONFIG_H): $(VERSION_FILE) $(CONFIG) $(CONFIG_H_TEMPLATE)
	$(eval CFG_NUT_MIMPID= $(shell grep CFG_NUT_MIMPID $(VERSION_FILE) | sed 's#^.*=##' ))
	@echo "### Updating $(CONFIG_H)..."
	@sed \
	-e 's#{CFG_NUT_SIM_CLK_SPEED}#$(CFG_NUT_SIM_CLK_SPEED)#g' \
	-e 's#{CFG_NUT_RESET_ADDR}#$(CFG_NUT_RESET_ADDR)#g' \
	-e 's#{CFG_NUT_SIM_MAX_PERIPHERY}#$(CFG_NUT_SIM_MAX_PERIPHERY)#g' \
	-e 's#{CFG_NUT_CPU_CORES_LD}#$(CFG_NUT_CPU_CORES_LD)#g' \
	-e 's#{CFG_NUT_CPU_MODE1_CORES}#$(CFG_NUT_CPU_MODE1_CORES)#g' \
	-e 's#{CFG_NUT_MEM_SIZE}#$(CFG_NUT_MEM_SIZE)#g' \
	-e 's#{CFG_NUT_EX_INT}#$(CFG_NUT_EX_INT)#g' \
	-e 's#{CFG_EXU_M_EXTENSION}#$(CFG_EXU_M_EXTENSION)#g' \
	-e 's#{CFG_EXU_A_EXTENSION}#$(CFG_EXU_A_EXTENSION)#g' \
	-e 's#{CFG_EXU_PERFCOUNT_ENABLE}#$(CFG_EXU_PERFCOUNT_ENABLE)#g' \
	-e 's#{CFG_EXU_PERFCOUNTER_BITS}#$(CFG_EXU_PERFCOUNTER_BITS)#g' \
	-e 's#{CFG_EXU_PERFCOUNTERS_LD}#$(CFG_EXU_PERFCOUNTERS_LD)#g' \
	-e 's#{CFG_MEMU_CACHE_BANKS_LD}#$(CFG_MEMU_CACHE_BANKS_LD)#g' \
	-e 's#{CFG_MEMU_CACHE_SETS_LD}#$(CFG_MEMU_CACHE_SETS_LD)#g' \
	-e 's#{CFG_MEMU_CACHE_WAYS_LD}#$(CFG_MEMU_CACHE_WAYS_LD)#g' \
	-e 's#{CFG_MEMU_CACHE_REPLACE_LRU}#$(CFG_MEMU_CACHE_REPLACE_LRU)#g' \
	-e 's#{CFG_MEMU_ARBITER_METHOD}#$(CFG_MEMU_ARBITER_METHOD)#g' \
	-e 's#{CFG_MEMU_BUSIF_WIDTH}#$(CFG_MEMU_BUSIF_WIDTH)#g' \
	-e 's#{CFG_IFU_IBUF_SIZE_LD}#$(CFG_IFU_IBUF_SIZE_LD)#g' \
	-e 's#{CFG_LSU_WBUF_SIZE_LD}#$(CFG_LSU_WBUF_SIZE_LD)#g' \
	-e 's#{CFG_PRIV_LEVELS}#$(CFG_PRIV_LEVELS)#g' \
	-e 's#{CFG_NUT_MIMPID}#$(CFG_NUT_MIMPID)#g' \
	$(CONFIG_H_TEMPLATE) > $(CONFIG_H)


# Main targets ...
libparanutsim.a: update-version $(CONFIG_H) $(PARANUT_SYSTEM_OBJ)
	@echo "### Archiving";
	ar rcs $@ $(PARANUT_SYSTEM_OBJ)


# Generate all HLS targets
.PHONY: hls
hls: $(HLS_TARGETS)

hls-%/solution1/solution1.log: %.cpp %.h %.tcl $(CONFIG_H)
	@echo "### Starting HLS for $*";
	vivado_hls -f $*.tcl


# Copy all generated VHDL files to $(HLS_CP_DIR) destination
.PHONY: copy
copy: $(HLS_CP_TARGETS)

.PHONY: copy-%
copy-%: hls-%/solution1/solution1.log
	@echo "### Copying $* VHDL to $(HLS_CP_DIR)";
	cp -f hls-$*/solution1/syn/vhdl/* $(HLS_CP_DIR)

clean::
	rm -f *.[aod] libparanutsim.a $(CONFIG_H) vivado*
	rm -f -r hls-*

clean-all: clean
	@if [ -d tb ]; then \
	  $(MAKE) -C tb clean; \
	fi;
