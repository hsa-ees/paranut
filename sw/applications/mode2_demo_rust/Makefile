# This file is part of the ParaNut project.
# 
#  Copyright (C) 2010-2020 Abdurrahman Celep <abdurrahman.celep1@hs-augsburg.de>
#		           Alexander Bahle <alexander.bahle@hs-augsburg.de>
#                          Gundolf Kiefer <gundolf.kiefer@hs-augsburg.de>
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
RUST_PROJECT_DIR := $(abspath $(CURDIR))
PARANUT_HOME ?= $(abspath $(RUST_PROJECT_DIR)/../../..)
include $(PARANUT_HOME)/directory-base.mk

# Flash target options
PN_FIRMWARE_ELF ?= 
PN_SYSTEM_HDF ?=
PN_SYSTEM_BIT ?=

TARGET ?= riscv32paranut-unknown-elf
TARGET_DIR ?= target/$(TARGET)/debug
ELF ?= $(TARGET_DIR)/mode2_demo_rust
LIBPARANUT_DEP = $(PNS_SYSC_INC_DIR)/paranut.h

.PHONY: build
# Generic Flash targets (set PN_* accordingly)
build: $(ELF)

$(ELF): $(LIBPARANUT_DEP)
	cargo make build	 

# build paranut libraries if not present
$(LIBPARANUT_DEP):
	make -C $(PARANUT_HOME) build-lib

$(PN_SIM_BIN):
	make -C $(PARANUT_HOME) build-sim

sim: $(ELF) $(PN_SIM_BIN)
	$(PN_SIM_BIN) $(ELF)



.PHONY: bin
bin: $(ELF).bin
$(ELF).bin: $(ELF) 
	cargo make bin

### Clean target(s) ###
.PHONY: clean
clean:
	@rm -rf ./target
	@rm Cargo.lock

