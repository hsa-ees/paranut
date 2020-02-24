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

ELF = dhry
SOURCES = $(wildcard *.c)
OBJECTS = $(patsubst %.c,%.o,$(SOURCES)) 
HEADERS = $(wildcard *.h)

SYSTEMS_DIR = $(PARANUT_HOME)/systems
RISCV_COMMON_DIR = $(PARANUT_HOME)/sw/riscv_common

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

