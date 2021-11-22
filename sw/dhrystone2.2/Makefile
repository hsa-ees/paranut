# This file is part of the ParaNut project.
# 
#  Copyright (C) 2020 Alexander Bahle <alexander.bahle@hs-augsburg.de>
#     Hochschule Augsburg, University of Applied Sciences
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

# Root of ParaNut repository or local project  
PARANUT ?= ../..

# Flash target options
PN_FIRMWARE_ELF ?= 
PN_SYSTEM_HDF ?=
PN_SYSTEM_BIT ?=

# Configuration options 
CROSS_COMPILE ?= riscv64-unknown-elf

CC      := $(CROSS_COMPILE)-gcc
GXX     := $(CROSS_COMPILE)-g++
OBJDUMP := $(CROSS_COMPILE)-objdump
OBJCOPY := $(CROSS_COMPILE)-objcopy
GDB     := $(CROSS_COMPILE)-gdb
AR      := $(CROSS_COMPILE)-ar
SIZE    := $(CROSS_COMPILE)-size

ELF = dhry
SOURCES = $(wildcard *.c)
OBJECTS = $(patsubst %.c,%.o,$(SOURCES)) 
HEADERS = $(wildcard *.h)

PN_SYSTEMS_DIR = $(PARANUT)/systems
RISCV_COMMON_DIR = $(PARANUT)/sw/riscv_common

CFG_MARCH ?= rv32im

CFLAGS = -O2 -march=$(CFG_MARCH) -mabi=ilp32 -I$(RISCV_COMMON_DIR)
LDFLAGS =  $(CFLAGS) -static -nostartfiles -lc $(RISCV_COMMON_DIR)/startup.S $(RISCV_COMMON_DIR)/syscalls.c -T $(RISCV_COMMON_DIR)/paranut.ld


# Software Targets
all: $(ELF) dump

$(ELF): $(OBJECTS)
	$(CC) -o $@ $^ $(LDFLAGS)

%.o:	%.c $(HEADERS)
	$(CC) -c $(CFLAGS) $<


# ParaNut Targets
.PHONY: sim
sim: $(ELF)
	+$(MAKE) -C $(PARANUT)/hw/sim
	$(PARANUT)/hw/sim/pn-sim -t0 $<

# Generic Flash targets (set PN_* accordingly)
.PHONY: flash flash-bit
flash: bin
	pn-flash -c -p $(ELF).bin $(PN_SYSTEM_HDF) $(PN_FIRMWARE_ELF)

flash-bit: bin
	pn-flash -c -b $(PN_SYSTEM_BIT) -p $(ELF).bin $(PN_SYSTEM_HDF) $(PN_FIRMWARE_ELF)


# Special System Flash targets for testing inside the source repository
.PHONY: flash-%
flash-%: bin
	if [ ! -d $(PN_SYSTEMS_DIR) ]; then echo; echo "INFO: The flash targets are only for testing inside the source repository!"; echo; exit 1; fi
	pn-flash -c -p $(ELF).bin $(PN_SYSTEMS_DIR)/$*/hardware/build/system.hdf $(PN_SYSTEMS_DIR)/$*/hardware/firmware/firmware.elf

flash-%-bit: bin
	if [ ! -d $(PN_SYSTEMS_DIR) ]; then echo "INFO: The flash targets are only for testing inside the source repository!"; exit 1; fi
	pn-flash -c -b $(PN_SYSTEMS_DIR)/$*/hardware/build/system.bit -p $(ELF).bin \
                            $(PN_SYSTEMS_DIR)/$*/hardware/build/system.hdf $(PN_SYSTEMS_DIR)/$*/hardware/firmware/firmware.elf

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

