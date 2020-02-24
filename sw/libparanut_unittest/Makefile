####################################################################################################
# Copyright (C) 2019-2020 Alexander Bahle <alexander.bahle@hs-augsburg.de>
#                         Anna Pfuetzner <annakerstin.pfuetzner@gmail.com>
#
# Redistribution and use in source and binary forms, with or without modification, are permitted 
# provided that the following conditions are met:
# 
# 1. Redistributions of source code must retain the above copyright notice, this list of conditions
# and the following disclaimer.
# 
# 2. Redistributions in binary form must reproduce the above copyright notice, this list of 
# conditions and the following disclaimer in the documentation and/or other materials provided with 
# the distribution.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR 
# IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND 
# FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR 
# CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL 
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, 
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER 
# IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT 
# OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
####################################################################################################

ifndef PARANUT_HOME
$(error PARANUT_HOME is not set)
endif

#System Configuration###############################################################################

# Call clean when changing these!

# System Parameters
NUMCORES  = 4
M2CAP_MSK = 0x0000000F
M3CAP_MSK = 0x00000001

#Compile Configuration##############################################################################

# Call clean when changing these!

# Instruction Set Architecture
# Currently available:
#    1. RISCV 32 bit - Set to RV32I
# Don't forget to change compiler when touching this!
PN_ISA = RV32I

# Compiler
# Currently available:
#    1. GCC - Available GCC for chosen ISA
PN_COMPILER = GCC

# Compile with Debug Symbols
# Default: 0
PN_DEBUG				= 1

#Internal Makefile Variables########################################################################

# Location of libparanut
PN_LIB_LOC	= ../libparanut

SYSTEMS_DIR = $(PARANUT_HOME)/systems

#Compiler Stuff#####################################################################################

ifeq ($(PN_ISA),RV32I)

	RISCV_COMMON_DIR = $(PARANUT_HOME)/sw/riscv_common
	
	ifeq ($(PN_COMPILER),GCC)
		
		# Set the list of system parameters together
		UNITT_DEFINES = -D NUMCORES=$(NUMCORES) -D M2CAP_MSK=$(M2CAP_MSK) -D M3CAP_MSK=$(M3CAP_MSK)
	
		# Actual Compiler and Object Dump
		CROSS_COMPILE ?= riscv64-unknown-elf
		CC      := $(CROSS_COMPILE)-gcc
		GXX     := $(CROSS_COMPILE)-g++
		OBJDUMP := $(CROSS_COMPILE)-objdump
		OBJCOPY := $(CROSS_COMPILE)-objcopy
		GDB     := $(CROSS_COMPILE)-gdb
		AR      := $(CROSS_COMPILE)-ar
		SIZE    := $(CROSS_COMPILE)-size

		# Compiler Flags
		CFLAGS  = -c -O3 -std=c99 -Wall -Werror -march=rv32ia -mabi=ilp32 -I./ -I$(RISCV_COMMON_DIR)
		CFLAGS += -I$(PN_LIB_LOC)/INSTALL $(UNITT_DEFINES)
		
		ifeq ($(PN_DEBUG),1)
			CFLAGS += -g
		endif
		
		CFLAGS += -o
		
		# Linker Flags
		LDFLAGS  = -static -march=rv32ia -mabi=ilp32 -nostartfiles -I$(PN_LIB_LOC)/INSTALL/ -I$(RISCV_COMMON_DIR)
		LDFLAGS += $(RISCV_COMMON_DIR)/startup.S $(RISCV_COMMON_DIR)/syscalls.c
		LDFLAGS += -T $(RISCV_COMMON_DIR)/paranut.ld
		LDFLAGS += -L$(PN_LIB_LOC)/INSTALL -lparanut
		
		ifeq ($(PN_DEBUG),1)
			LDFLAGS += -g
		endif
		
		LDFLAGS += -o
		
		# Object Dump Flags
		OBJFLAGS = -S -D -f -g -e -h -l -a -F -p -s -t -x
		RED_OBJFLAGS = -S -D
	
	else
	
		$(error No valid compiler set for the ISA that you chose.)
		
	endif
	
else
	
	$(error No valid ISA set.)
	
endif

#Object List########################################################################################

BINARY = libparanut_unittest
SOURCES = $(wildcard *.c)
OBJECTS = $(patsubst %.c,%.o,$(SOURCES)) 

#Target Magic#######################################################################################

sim: libparanut $(BINARY) 
	+$(MAKE) -C $(PARANUT_HOME)/sysc paranut_tb
	$(PARANUT_HOME)/sysc/paranut_tb -t0 $(BINARY) 
	
sim_dbg: libparanut $(BINARY)  dump
	+$(MAKE) -C $(PARANUT_HOME)/sysc paranut_tb
	cd ./Debugging_Aid && $(PARANUT_HOME)/sysc/paranut_tb -t2 ../$(BINARY) 
	
flash-%: bin
	paranut_flash -c -p $(BINARY).bin $(SYSTEMS_DIR)/$*/hardware/build/system.hdf $(SYSTEMS_DIR)/$*/hardware/firmware/firmware.elf
	
flash-%-bit: bin
	paranut_flash -c -b $(SYSTEMS_DIR)/$*/hardware/build/system.bit -p $(BINARY).bin \
	                               $(SYSTEMS_DIR)/$*/hardware/build/system.hdf $(SYSTEMS_DIR)/$*/hardware/firmware/firmware.elf
	
	
all: libparanut $(BINARY) dump

$(BINARY): libparanut $(OBJECTS)
	$(CC) $(OBJECTS) $(LDFLAGS) $@
	
%.o:	%.c libparanut
	$(CC) $(CFLAGS) $@ $<

# The other Makefile should check if libparanut is already up-to-date.
# This, sadly, also forces unittest to be re-linked, but I haven't found a more
# elegant solution yet.
.PHONY: libparanut
libparanut:
	+$(MAKE) -C $(PN_LIB_LOC) all
	
.PHONY: dump
dump: $(BINARY).dump
$(BINARY).dump: $(BINARY)
	$(OBJDUMP) $(OBJFLAGS) $< > ./Debugging_Aid/libparanut_unittest_dump
	$(OBJDUMP) $(RED_OBJFLAGS) $< > ./Debugging_Aid/libparanut_unittest_reduced_dump
	
.PHONY: bin
bin: $(BINARY).bin
%.bin: libparanut libparanut_unittest
	$(OBJCOPY) -S -O binary libparanut_unittest libparanut_unittest.bin

clean:
	rm -f *.o *.bin *.jou *.log ./Debugging_Aid/*.vcd ./Debugging_Aid/*_dump $(BINARY)

#EOF################################################################################################
