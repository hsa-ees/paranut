#  -----------------------------------------------------------------------------
#
#  This file is part of the ParaNut project.
#
#  Copyright (C) 2020 Gundolf Kiefer <gundolf.kiefer@hs-augsburg.de>
#      Hochschule Augsburg, University of Applied Sciences
#
#  Description:
#    Makefile fragment for software projects.
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


# Identify the ParaNut directory ...
PARANUT_SW := $(dir $(lastword $(MAKEFILE_LIST)))
PARANUT := $(PARANUT_SW)..


# Cross tool chain ...
PARANUT_CROSS ?= riscv64-unknown-elf

PARANUT_CC   := $(PARANUT_CROSS)-gcc
PARANUT_CXX  := $(PARANUT_CROSS)-g++


# Compiler/linker flags ...
PN_RISCV_COMMON := $(PARANUT_SW)riscv_common
PN_MARCH ?= rv32ima

PARANUT_CFLAGS := -march=$(PN_MARCH) -mabi=ilp32 -I$(PARANUT_SW)include -I$(PN_RISCV_COMMON)
PARANUT_LDFLAGS := $(PARANUT_CFLAGS) -static -nostartfiles -lc $(PN_RISCV_COMMON)/startup.S $(PN_RISCV_COMMON)/syscalls.c -T $(PN_RISCV_COMMON)/paranut.ld -L$(PARANUT_SW)lib -lparanut

PARANUT_DEPEND := $(PARANUT)/sw/lib/libparanut.a

# Helper targets to update or clean libraries and headers ...
$(PARANUT)/sw/lib/libparanut.a: paranut-update

.PHONY: paranut-update
paranut-update:
	$(MAKE) -C $(PARANUT_SW)libparanut install PREFIX=$(abspath $(PARANUT_SW))

.PHONY: paranut-clean
paranut-clean:
	$(MAKE) -C $(PARANUT_SW)libparanut clean
	rm -rf $(PARANUT_SW)lib $(PARANUT_SW)include $(PARANUT_SW)doc
