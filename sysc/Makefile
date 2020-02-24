# This file is part of the ParaNut project.
# 
#  Copyright (C) 2010-2020 Alexander Bahle <alexander.bahle@hs-augsburg.de>
#                          Gundolf Kiefer <gundolf.kiefer@hs-augsburg.de>
#     Hochschule Augsburg, University of Applied Sciences
# 
# Description:
#   This module contains various types, constants and helper functions
#   for the SystemC model of ParaNut.
# 
# Redistribution and use in source and binary forms, with or without modification, 
# are permitted provided that the following conditions are met:
# 
# 1. Redistributions of source code must retain the above copyright notice, this 
#    list of conditions and the following disclaimer.
# 
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation and/or
#    other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND 
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE 
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
# ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES 
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; 
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON 
# ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS 
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


# include global configuration
include ../config

CONFIG=config.h
CONFIG_TEMPLATE=config.template.h

CFLAGS = -O2 -I$(SYSTEMC_HOME)/include -DSIMBUILD # -Wfatal-errors
LDFLAGS = -L$(SYSTEMC_HOME)/lib-linux64 -lsystemc # -Wfatal-errors

MEMU_TB_SRC = base.cpp lfsr.cpp memu.cpp memory.cpp peripherals.cpp memu_tb.cpp
MEMU_TB_OBJ =  $(patsubst %.cpp, %.o, $(MEMU_TB_SRC))

JTAG_DTM_TB_SRC = base.cpp jtag_dtm.cpp jtag_dtm_tb.cpp
JTAG_DTM_TB_OBJ = $(patsubst %.cpp, %.o, $(JTAG_DTM_TB_SRC))

DM_TB_SRC = base.cpp dm.cpp dm_tb.cpp
DM_TB_OBJ = $(patsubst %.cpp, %.o, $(DM_TB_SRC))

PARANUT_SRC = base.cpp lfsr.cpp ifu.cpp exu.cpp lsu.cpp memu.cpp intc.cpp dm.cpp jtag_dtm.cpp paranut.cpp 
PARANUT_OBJ = $(patsubst %.cpp, %.o, $(PARANUT_SRC))

PARANUT_TB_SRC = $(PARANUT_SRC) memory.cpp peripherals.cpp remote_bitbang.cpp paranut_tb.cpp
PARANUT_TB_OBJ = $(patsubst %.cpp, %.o, $(PARANUT_TB_SRC))

TEST_APP = test_all
TEST_APP_DIR = ../sw/test_riscv

HLS_SRCS = $(wildcard *.tcl)
HLS_TARGETS = $(patsubst %.tcl, hls-%/solution1/solution1.log, $(HLS_SRCS))
HLS_CP_TARGETS = $(patsubst %.tcl, copy-%, $(HLS_SRCS))
HLS_CP_DIR ?= ../hw/rtl/vhdl/paranut 

ifeq "$(CFG_EXU_M_EXTENSION)" "0"
	CFG_MARCH := rv32i
else 
	CFG_MARCH := rv32im
	PARANUT_SRC += mextension.cpp
	PARANUT_OBJ += mextension.o
endif

paranut_tb: $(PARANUT_TB_OBJ)
	@echo; echo "### Linking"; echo
	g++ $(LDFLAGS) -o paranut_tb $(PARANUT_TB_OBJ)

memu_tb: $(MEMU_TB_OBJ)
	@echo; echo "### Linking"; echo
	g++ $(LDFLAGS) -o memu_tb $(MEMU_TB_OBJ)

# Undefine SIMBUILD for jtag_dtm_tb
jtag_dtm_tb: CFLAGS += -USIMBUILD 
jtag_dtm_tb: $(JTAG_DTM_TB_OBJ)
	@echo; echo "### Linking"; echo
	g++ $(LDFLAGS) -o jtag_dtm_tb $(JTAG_DTM_TB_OBJ)

dm_tb: $(DM_TB_OBJ)
	@echo; echo "### Linking"; echo
	g++ $(LDFLAGS) -o dm_tb $(DM_TB_OBJ)
	
sim: paranut_tb test_app
	@echo; echo "### Simulating..."; echo
	./paranut_tb -m 0x2000 0x203c $(TEST_APP_DIR)/$(TEST_APP)

sim_nocache: paranut_tb test_app
	@echo; echo "### Simulating..."; echo
	./paranut_tb -c -m 0x2000 0x203c $(TEST_APP_DIR)/$(TEST_APP)

test_app: paranut_tb
	@echo; echo "### Making test application"; echo
	+make -C $(TEST_APP_DIR) $(TEST_APP)

hello_newlib: paranut_tb
	@echo; echo "### Making 'hello_newlib"; echo
	+make -C ../sw/hello_newlib hello_newlib
	@echo; echo "### Simulating..."; echo
	./paranut_tb ../sw/hello_newlib/hello_newlib

libparanut_unittest: paranut_tb
	@echo; echo "### Making 'libparanut_unittest'"; echo
	+make -C ../sw/libparanut_unittest all
	@echo; echo "### Simulating..."; echo
	./paranut_tb ../sw/libparanut_unittest/libparanut_unittest
	
libparanut_unittest_debug: paranut_tb
	@echo; echo "### Making 'libparanut_unittest'"; echo
	+make -C ../sw/libparanut_unittest all
	@echo; echo "### Simulating..."; echo
	./paranut_tb -d ../sw/libparanut_unittest/libparanut_unittest

dhrystone: paranut_tb
	@echo; echo "### Making 'dhrystone'"; echo
	+make -C ../sw/dhrystone dhry CFG_CLKS_PER_SEC=$(CFG_NUT_SIM_CLK_SPEED) CFG_MARCH=$(CFG_MARCH)
	@echo; echo "### Simulating..."; echo
	./paranut_tb -t0 ../sw/dhrystone/dhry > dhrystone_out_M_$(CFG_EXU_M_EXTENSION)

coremark: paranut_tb
	@echo; echo "### Making 'coremark'"; echo
	+make -C ../sw/coremark_v1.0 compile PORT_DIR=paranut ITERATIONS=500 CFG_CLKS_PER_SEC=$(CFG_NUT_SIM_CLK_SPEED) CFG_NUT_CPU_CORES_LD=$(CFG_NUT_CPU_CORES_LD) CFG_MARCH=$(CFG_MARCH)
	@echo; echo "### Simulating..."; echo
	./paranut_tb -t0 ../sw/coremark_v1.0/coremark.exe > coremark_out_$(CFG_NUT_CPU_CORES_LD)_M_$(CFG_EXU_M_EXTENSION)
	
depend:
	makedepend -Y $(PARANUT_TB_SRC) $(MEMU_TB_SRC) # -- $(CFLAGS) -- $(PN_SRC) $(MEMU_TB_SRC)

%.o:	%.cpp 
	g++ -c $(CFLAGS) $<


config.h: ../config 
	@echo; echo "### Updating config.h..."; echo	
	sed \
	-e 's#{CFG_NUT_SIM_CLK_SPEED}#$(CFG_NUT_SIM_CLK_SPEED)#g' \
	-e 's#{CFG_NUT_SIM_MEM_ADDR}#$(CFG_NUT_SIM_MEM_ADDR)#g' \
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
	$(CONFIG_TEMPLATE) > $(CONFIG)

# Generate all HLS targets
.PHONY: hls
hls: $(HLS_TARGETS)

hls-%/solution1/solution1.log: %.cpp %.h %.tcl config.h
	@echo; echo "### Starting HLS for $*"; echo
	vivado_hls -f $*.tcl

# Copy all generated VHDL files to $(HLS_CP_DIR) destination
.PHONY: copy
copy: $(HLS_CP_TARGETS)

.PHONY: copy-% 
copy-%: hls-%/solution1/solution1.log
	@echo; echo "### Copying $* VHDL to $(HLS_CP_DIR)";
	cp -f hls-$*/solution1/syn/vhdl/* $(HLS_CP_DIR)


clean:
	rm -f *.[ao] core paranut_tb paranut_tb.vcd memu_tb memu_tb.vcd config.h vivado*
	rm -f -r hls-*

# DO NOT DELETE

base.o: base.h
lfsr.o: lfsr.h base.h
ifu.o: ifu.h base.h config.h
exu.o: exu.h base.h config.h mextension.h memory.h
lsu.o: lsu.h base.h config.h
memu.o: memu.h base.h config.h lfsr.h
paranut.o: paranut.h memu.h base.h config.h ifu.h lsu.h exu.h mextension.h intc.h dm.h jtag_dtm.h
mextension.o: mextension.h base.h
memory.o: memory.h base.h config.h
peripherals.o: peripherals.h memory.h base.h config.h 
paranut_tb.o: peripherals.h memory.h base.h config.h paranut.h
paranut_tb.o: memu.h ifu.h lsu.h exu.h mextension.h jtag_dtm.h dm.h
base.o: base.h
intc.o: intc.h base.h config.h
lfsr.o: lfsr.h base.h
memu.o: memu.h base.h config.h lfsr.h
memory.o: memory.h base.h config.h
memu_tb.o: memu.h base.h config.h peripherals.h memory.h 
dm.o: dm.h base.h config.h
jtag_dtm.o: jtag_dtm.h base.h 
remote_bitbang.o: remote_bitbang.h
