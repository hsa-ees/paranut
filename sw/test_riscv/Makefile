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

CFG_MARCH ?= rv32ima

ELF = test_all

CFLAGS = -g -O0 -march=$(CFG_MARCH) -mabi=ilp32 -nostartfiles -static -nostdlib 
LDFLAGS =  $(CFLAGS) -T $(ELF).ld


# Software Targets
all: $(ELF) dump 

$(ELF): $(ELF).S
	$(CC) $(LDFLAGS) -o $@ $^ 


# ParaNut Targets
.PHONY: sim
sim: $(ELF)
	+$(MAKE) -C  $(PARANUT_HOME)/sysc paranut_tb
	$(PARANUT_HOME)/sysc/paranut_tb -t0 -m 0x10002000 0x10002050 $<


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
	rm -f $(ELF) $(ELF).dump $(ELF).bin 
