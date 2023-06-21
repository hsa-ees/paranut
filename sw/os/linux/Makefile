#  -----------------------------------------------------------------------------
#
#  This file is part of the ParaNut project.
#
#  Copyright (C) 2021-2022 Nico Borgsmüller <nico.borgsmueller@hs-augsburg.de>
#      Hochschule Augsburg, University of Applied Sciences
#
#  Description:
#    Linux software Makefile.
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

# Root of ParaNut repository or local project
PN_PARANUT ?= ../..

# include global configuration
CONFIG_MK ?= $(PN_PARANUT)/config.mk
include $(CONFIG_MK)

# Flash target options
PN_FIRMWARE_ELF ?=
PN_SYSTEM_HDF ?=
PN_SYSTEM_BIT ?=

PN_EXTERNAL ?= $(PARANUT_TOOLS)/../external
PN_PATCHES = $(PN_EXTERNAL)/patches

# Configuration options
CROSS_COMPILE_SRC = https://toolchains.bootlin.com/downloads/releases/toolchains/riscv32-ilp32d/tarballs/riscv32-ilp32d--glibc--bleeding-edge-2021.11-1.tar.bz2
CROSS_COMPILE_ARCHIVE = $(CROSS_COMPILE_FOLDER)/toolchain.tar.bz2
CROSS_COMPILE_FOLDER = $(PN_EXTERNAL)/riscv32-cross-compiler
CROSS_COMPILE = $(CROSS_COMPILE_FOLDER)/riscv32-ilp32d--glibc--bleeding-edge-2021.11-1/bin/riscv32-linux-

CC      := $(CROSS_COMPILE)gcc
OBJDUMP := $(CROSS_COMPILE)objdump
OBJCOPY := $(CROSS_COMPILE)objcopy
NM      := $(CROSS_COMPILE)nm

FDT_PATH = paranut.dtb
DEVICE_TREE = paranut.dts
DEVICE_TREE_TEMPLATE = paranut-template.dts

LINUX = $(PN_EXTERNAL)/linux
OPENSBI = $(PN_EXTERNAL)/opensbi

LINUX_CONFIG = linux.config
LINUX_SRC = git://git.kernel.org/pub/scm/linux/kernel/git/stable/linux.git
LINUX_REV = v5.18.9
LINUX_IMAGE = $(LINUX)/arch/riscv/boot/Image
LINUX_DTC = $(LINUX)/scripts/dtc/dtc

OPENSBI_SRC = https://github.com/riscv-software-src/opensbi
OPENSBI_REV = v1.0
OPENSBI_PATCH_FILE = $(PN_PATCHES)/OpenSBI.patch

OPENSBI_PLATFORM_FLAGS = PLATFORM=generic PLATFORM_RISCV_ABI=ilp32 PLATFORM_RISCV_ISA=rv32ima PLATFORM_RISCV_XLEN=32
OPENSBI_FW_FLAGS := FW_PAYLOAD_PATH=$(abspath $(LINUX_IMAGE)) FW_TEXT_START=0x10000000 FW_FDT_PATH=$(abspath $(FDT_PATH)) FW_PIC=n FW_DYNAMIC=n FW_JUMP=n FW_PAYLOAD=y

ELF = $(OPENSBI)/build/platform/generic/firmware/fw_payload.elf

# Software Targets
all: $(ELF)

# Build final OpenSBI image and include Linux and the device tree
$(ELF): $(OPENSBI)/Makefile $(LINUX_IMAGE) $(FDT_PATH) $(CC)
	+$(MAKE) -C $(OPENSBI) CROSS_COMPILE=$(abspath $(CROSS_COMPILE)) ARCH=riscv $(OPENSBI_PLATFORM_FLAGS) $(OPENSBI_FW_FLAGS)

# Prepare OpenSBI by cloning it, checking out the given version and applying the patch file
$(OPENSBI)/Makefile: $(OPENSBI_PATCH_FILE)
	rm -rf $(OPENSBI)
	git clone -b $(OPENSBI_REV) --depth 1 $(OPENSBI_SRC) $(OPENSBI)
	cd $(OPENSBI) && git am $(abspath $(OPENSBI_PATCH_FILE))

# Linux build creates both the image and the device tree compiler
$(LINUX_IMAGE) $(LINUX_DTC): .linux_built ;

# Compile Linux
.linux_built: $(LINUX)/Makefile $(LINUX)/.config $(CC)
	+$(MAKE) -C $(LINUX) CROSS_COMPILE=$(abspath $(CROSS_COMPILE)) ARCH=riscv
	@touch $@

# Prepare Linux by cloning it and checking out the given version
$(LINUX)/Makefile:
	rm -rf $(LINUX)
	mkdir -p $(LINUX)
	git clone -b $(LINUX_REV) --depth 1 $(LINUX_SRC) $(LINUX)

# Copy Linux config in
$(LINUX)/.config: $(LINUX_CONFIG) $(LINUX)/Makefile
	cp $(LINUX_CONFIG) $(LINUX)/.config

# Build flattened device tree from device tree source
$(FDT_PATH): $(LINUX_DTC) $(DEVICE_TREE)
	$(LINUX_DTC) $(DEVICE_TREE) > $(FDT_PATH)

# Prepare device tree source by using the ParaNut config and a template
$(DEVICE_TREE): $(DEVICE_TREE_TEMPLATE) $(CONFIG_MK)
# Timer frequency is clock speed divided by timebase
	@TIMER_FREQ=$$(($(CFG_NUT_SIM_CLK_SPEED) / $(CFG_NUT_MTIMER_TIMEBASE_US))); \
# Remove 0x from mtime address to it can be used in the device tree \
	MTIME_ADDR=$$(echo $(CFG_NUT_MTIMER_ADDR) | sed -e "s/0x//g"); \
# mtimecmp is 64bit further than mtime \
	MTIMECMP_ADDR=$$(($$MTIME_ADDR + 8)); \
# Remove 0x from memory address \
	MEM_ADDR=$$(echo $(CFG_NUT_RESET_ADDR) | sed -e "s/0x//g"); \
# Build memory size (e.g. from "(256 * MB)" to "10000000") \
# First, replace "MB" with "KB * 1024" and "KB" with "KB * 1024" and remove "(" and ")" \
# Then, evaluate math expression with "$$((1 * 2))"" and convert number to hex with printf \
	MEM_SIZE=$$(printf "%x\n" $$(($$(echo "$(CFG_NUT_MEM_SIZE)" | sed -e "s/MB/1024 * KB/g" -e "s/KB/1024/g" -e "s/(//g" -e "s/)//g")))); \
# Replace variables in device tree template \
	sed \
	-e "s#{TIMER_FREQ}#$$TIMER_FREQ#g" \
	-e "s#{MTIME_ADDR}#$$MTIME_ADDR#g" \
	-e "s#{MTIMECMP_ADDR}#$$MTIMECMP_ADDR#g" \
	-e "s#{MEM_ADDR}#$$MEM_ADDR#g" \
	-e "s#{MEM_SIZE}#$$MEM_SIZE#g" \
	$(DEVICE_TREE_TEMPLATE) > $(DEVICE_TREE)

# Prepare compiler toolchain by unpacking downloaded archive
$(CC): $(CROSS_COMPILE_ARCHIVE)
	tar -x -C $(CROSS_COMPILE_FOLDER) -f $(CROSS_COMPILE_ARCHIVE)
	@touch $@

# Download compiler toolchain archive
$(CROSS_COMPILE_ARCHIVE):
	rm -rf $(CROSS_COMPILE_FOLDER)
	mkdir $(CROSS_COMPILE_FOLDER)
	wget $(CROSS_COMPILE_SRC) -O $(CROSS_COMPILE_ARCHIVE)

# ParaNut Targets
.PHONY: build-sim
build-sim:
	+$(MAKE) -C $(PN_PARANUT)/hw/sim

.PHONY: sim
sim: $(ELF) build-sim
	$(PN_PARANUT)/hw/sim/pn-sim -t0 $<

# Generic Flash targets (set PN_* accordingly)
.PHONY: flash flash-bit
flash: compressed-bin
	TOHOST=$(shell $(NM) $(ELF) | grep pn_tohost | cut -d " " -f1); \
	FROMHOST=$(shell $(NM) $(ELF) | grep pn_fromhost | cut -d " " -f1); \
	pn-flash -c -p $(ELF).bin.gz -o $$TOHOST:$$FROMHOST $(PN_SYSTEM_HDF) $(PN_FIRMWARE_ELF)

flash-bit: compressed-bin
	TOHOST=$(shell $(NM) $(ELF) | grep pn_tohost | cut -d " " -f1); \
	FROMHOST=$(shell $(NM) $(ELF) | grep pn_fromhost | cut -d " " -f1); \
	pn-flash -c -b $(PN_SYSTEM_BIT) -p $(ELF).bin.gz -o $$TOHOST:$$FROMHOST $(PN_SYSTEM_HDF) $(PN_FIRMWARE_ELF)

# Misc Targets
.PHONY: dump
dump: $(ELF).dump
$(ELF).dump: $(ELF)
	$(OBJDUMP) -S -D $< > $@

.PHONY: compressed-bin
compressed-bin: $(ELF).bin.gz
$(ELF).bin.gz: $(ELF).bin
	gzip -c $< > $@

.PHONY: bin
bin: $(ELF).bin
$(ELF).bin: $(ELF) 
	$(OBJCOPY) -S -O binary $< $@

.PHONY: clean
clean:
	rm -rf $(FDT_PATH) $(DEVICE_TREE) $(LINUX) $(OPENSBI) $(CROSS_COMPILE_FOLDER)
