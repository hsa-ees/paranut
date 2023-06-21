# This file is part of the ParaNut project.
# 
#  Copyright (C) 2010-2020 Alexander Bahle <alexander.bahle@hs-augsburg.de>
#                          Gundolf Kiefer <gundolf.kiefer@hs-augsburg.de>
#                          Mark Endres <mark.endres@hs-augsburg.de>
#                          Patrick Zacharias <patrick.zacharias@hs-augsburg.de>
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


# Root of ParaNut repository or local project  
PN_PARANUT ?= ../..
PN_EXTERNAL ?= $(PN_PARANUT)/external

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

WGET    := wget

FREERTOS = ../freertos/FreeRTOS-Kernel
FREERTOS_KERNEL = $(PN_EXTERNAL)/FreeRTOS-Kernel
FREERTOS_SRC = https://ti-build.informatik.hs-augsburg.de:8443/paranut_developers/freertos-kernel.git
FREERTOS_IO = ../freertos/FreeRTOS-Plus-IO
FREERTOS_RISCV = $(FREERTOS_KERNEL)/portable/GCC/RISC-V

SDL = $(PN_PARANUT)/external/SDL2
VEARS = $(PN_PARANUT)/external/asterics/ipcores/VEARS/drivers/vears_v1_0/src

ELF = sdl_demo
SOURCES = $(wildcard *.c) $(FREERTOS_KERNEL)/tasks.c $(FREERTOS_KERNEL)/list.c $(FREERTOS_KERNEL)/queue.c $(FREERTOS_KERNEL)/portable/MemMang/heap_3.c $(FREERTOS_RISCV)/port.c ../freertos/minilib/_clzsi2.c $(FREERTOS_IO)/Device/Paranut/FreeRTOS_paranut_uart.c $(FREERTOS_IO)/Device/Paranut/FreeRTOS_paranut_DriverInterface.c $(wildcard $(FREERTOS_IO)/Common/*.c)
SOURCES_SDL = \
	$(SDL)/src/*.c \
	$(SDL)/src/atomic/*.c \
	$(SDL)/src/audio/*.c \
	$(SDL)/src/audio/dummy/*.c \
	$(SDL)/src/cpuinfo/*.c \
	$(SDL)/src/events/*.c \
	$(SDL)/src/file/*.c \
	$(SDL)/src/haptic/*.c \
	$(SDL)/src/haptic/dummy/*.c \
	$(SDL)/src/joystick/*.c \
	$(SDL)/src/joystick/dummy/*.c \
	$(SDL)/src/loadso/dummy/*.c \
	$(SDL)/src/power/*.c \
	$(SDL)/src/filesystem/dummy/*.c \
	$(SDL)/src/render/*.c \
	$(SDL)/src/render/software/*.c \
	$(SDL)/src/sensor/*.c \
	$(SDL)/src/sensor/dummy/*.c \
	$(SDL)/src/stdlib/*.c \
	$(SDL)/src/thread/*.c \
	$(SDL)/src/thread/generic/*.c \
	$(SDL)/src/timer/*.c \
	$(SDL)/src/timer/dummy/*.c \
	$(SDL)/src/video/*.c \
	$(SDL)/src/video/dummy/*.c \
	$(SDL)/src/video/yuv2rgb/*.c \
	$(SDL)/src/SDL_error.c \
	
SOURCES_VEARS = \
	$(VEARS)/ea_draw_line.c \
	$(VEARS)/ea_draw_string.c \
	$(VEARS)/ea_init.c \
	$(VEARS)/ea_set_overlay_software_address.c \
	$(VEARS)/ea_draw_circle.c \
	$(VEARS)/ea_draw_point.c \
	$(VEARS)/ea_erase.c \
	$(VEARS)/ea_present.c \
	$(VEARS)/embedded_augmenter_helpers.c \
	$(VEARS)/ea_draw_filled_circle.c \
	$(VEARS)/ea_draw_quick_line_h.c \
	$(VEARS)/ea_font.c \
	$(VEARS)/ea_set_clipped_area.c \
	$(VEARS)/vears.c \
	$(VEARS)/ea_draw_filled_rectangle.c \
	$(VEARS)/ea_draw_quick_line_v.c \
	$(VEARS)/ea_get_overlay_hardware_address.c \
	$(VEARS)/ea_set_color.c \
	$(VEARS)/ea_draw_icon.c \
	$(VEARS)/ea_draw_rectangle.c \
	$(VEARS)/ea_get_overlay_software_address.c \
	$(VEARS)/ea_set_overlay_hardware_address.c \

SOURCES += $(wildcard $(SOURCES_SDL))
SOURCES += $(SOURCES_VEARS)
SOURCES_ASM = $(FREERTOS_RISCV)/portASM.S
OBJECTS = $(RISCV_COMMON_DIR)/syscalls.o $(patsubst %.S,%.o,$(SOURCES_ASM)) $(patsubst %.c,%.o,$(SOURCES))
HEADERS = $(wildcard *.h) ../freertos/paranut-config.h $(PN_LIB_INC_DIR)/paranut.h

PN_SYSTEMS_DIR = $(PN_PARANUT)/systems
RISCV_COMMON_DIR = $(PN_PARANUT)/sw/riscv_common
PN_LIB_DIR = $(PN_PARANUT)/sw/libparanut/build
PN_LIB_LIB_DIR = $(PN_LIB_DIR)/lib
PN_LIB_INC_DIR = $(PN_LIB_DIR)/include

CFG_MARCH ?= rv32im

CFLAGS = -Os -g -march=$(CFG_MARCH) -mabi=ilp32 -I$(RISCV_COMMON_DIR) -I../freertos -I$(FREERTOS_KERNEL)/include -I$(FREERTOS_RISCV) -I$(FREERTOS_IO)/Include -I./$(FREERTOS_IO)/Device/Paranut -I$(PN_LIB_INC_DIR) -I$(SDL)/include -I$(VEARS) -I. -D__paranut__
LIBS = -Wl,--whole-archive $(PN_LIB_LIB_DIR)/libparanut.a -Wl,--no-whole-archive
LDFLAGS =  $(CFLAGS) -static -nostartfiles -lc -lm $(RISCV_COMMON_DIR)/startup.S $(LIBS) -T ../freertos/paranut.ld -Wl,--build-id=none

# Software Targets
all: bin
	
$(SDL): 
	./prepare_sdl.sh
	# Needs re-run to include up-to-date SDL_SOURCES (can't use wildcards otherwise)
	@echo SDL was not prepared, please run make again
	exit 1

$(ELF): $(OBJECTS)
	$(CC) -o $@ $^ $(LDFLAGS)

%.o:	%.c $(HEADERS) $(SDL) $(PN_LIB_INC_DIR)/paranut.h
	$(CC) -c $(CFLAGS) -o $@ $<

%.o:	%.S
	$(CC) -c $(CFLAGS) -I../freertos/chip_specific_extensions/Paranut/ -DportasmHANDLE_INTERRUPT=external_interrupt_handler -o $@ $<

# Prepare freertos by cloning it and checking out the given version
$(FREERTOS_KERNEL)/README.md:
	rm -rf $(FREERTOS_KERNEL)
	mkdir -p $(FREERTOS_KERNEL)
	git clone $(FREERTOS_SRC) $(FREERTOS_KERNEL)

# libparanut Target
$(PN_LIB_INC_DIR)/paranut.h: 
	+$(MAKE) -C $(PN_PARANUT)/sw/libparanut all
	
../freertos/paranut-config.h: build-sim $(PN_PARANUT)/hw/sysc/paranut-config.h $(FREERTOS_KERNEL)/README.md
	cp $(PN_PARANUT)/hw/sysc/paranut-config.h ../freertos/paranut-config.h

# ParaNut Targets
.PHONY: build-sim
build-sim:
	+$(MAKE) -C $(PN_PARANUT)/hw/sim

.PHONY: sim
sim: $(ELF) build-sim
	$(PN_PARANUT)/hw/sim/pn-sim -t0 $<

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
	rm -f ../freertos/paranut-config.h $(patsubst %.S,%.o,$(SOURCES_ASM)) $(patsubst %.c,%.o,$(SOURCES)) $(patsubst %.S,%.d,$(SOURCES_ASM)) $(patsubst %.c,%.d,$(SOURCES)) $(ELF) $(ELF).bin $(ELF).dump $(ELF).d

