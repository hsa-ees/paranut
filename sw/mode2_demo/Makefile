# This file is part of the ParaNut project.
# 
#  Copyright (C) 2019-2020 Alexander Bahle <alexander.bahle@hs-augsburg.de>
#     Hochschule Augsburg, University of Applied Sciences
#
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

# Check for PARANUT_HOME (<ParaNut-Dir>/settings.sh)
ifndef PARANUT_HOME
$(error PARANUT_HOME is not set. Source <ParaNut-Dir>/settings.sh first)
endif

# Configuration options 
CROSS_COMPILE ?= riscv64-unknown-elf

CC      := $(CROSS_COMPILE)-gcc
GXX     := $(CROSS_COMPILE)-g++
OBJDUMP := $(CROSS_COMPILE)-objdump
OBJCOPY := $(CROSS_COMPILE)-objcopy
GDB     := $(CROSS_COMPILE)-gdb
AR      := $(CROSS_COMPILE)-ar
SIZE    := $(CROSS_COMPILE)-size

ELF = mode2_demo
SOURCES = $(wildcard *.c)
OBJECTS = $(patsubst %.c,%.o,$(SOURCES)) 
HEADERS = $(wildcard *.h)

SYSTEMS_DIR = $(PARANUT_HOME)/systems
RISCV_COMMON_DIR = $(PARANUT_HOME)/sw/riscv_common
PN_LIB_DIR	= $(PARANUT_HOME)/sw/libparanut/INSTALL

# Add libparanut to dependecies
HEADERS +=  $(PN_LIB_DIR)/libparanut.h

CFG_MARCH ?= rv32ima

# A-Extenxion enabled?
ifneq (,$(findstring a,$(CFG_MARCH)))
    ATOMIC = 1
else
    ATOMIC = 0
endif

CFLAGS = -O2 -march=$(CFG_MARCH) -mabi=ilp32 -I$(RISCV_COMMON_DIR) 
CFLAGS += -D_ATOMIC=$(ATOMIC)  -I$(PN_LIB_DIR)
LDFLAGS =  $(CFLAGS) -static -nostartfiles -lc $(RISCV_COMMON_DIR)/startup.S $(RISCV_COMMON_DIR)/syscalls.c -T $(RISCV_COMMON_DIR)/paranut.ld
LDFLAGS += -lparanut -L$(PN_LIB_DIR) 

# Software Targets
all: $(ELF) dump

$(ELF): $(OBJECTS)
	$(CC) -o $@ $^ $(LDFLAGS)

%.o:	%.c $(HEADERS)
	$(CC) -c $(CFLAGS) $<

# libparanut Target
$(PN_LIB_DIR)/libparanut.h: 
	+$(MAKE) -C $(PARANUT_HOME)/sw/libparanut all

# ParaNut Targets
.PHONY: sim
sim: $(ELF)
	+$(MAKE) -C  $(PARANUT_HOME)/sysc paranut_tb
	$(PARANUT_HOME)/sysc/paranut_tb -t0 $<

.PHONY: flash-%
flash-%: bin
	paranut_flash -c -p $(ELF).bin $(SYSTEMS_DIR)/$*/hardware/build/system.hdf $(SYSTEMS_DIR)/$*/hardware/firmware/firmware.elf
	
flash-%-bit: bin
	paranut_flash -c -b $(SYSTEMS_DIR)/$*/hardware/build/system.bit -p $(ELF).bin \
	                               $(SYSTEMS_DIR)/$*/hardware/build/system.hdf $(SYSTEMS_DIR)/$*/hardware/firmware/firmware.elf

# Misc Targets
.PHONY: dump
dump: $(ELF).dump
$(ELF).dump: $(ELF)
	$(OBJDUMP) -S -D $< > $@

.PHONY: bin
bin: $(ELF).bin
$(ELF).bin: $(ELF) 
	$(OBJCOPY) -S -O binary $< $@

.PHONY: clean
clean:
	rm -f *.o *.o.s *.c.s $(ELF) $(ELF).bin $(ELF).dump 

