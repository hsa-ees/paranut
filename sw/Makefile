#  -----------------------------------------------------------------------------
#
#  This file is part of the ParaNut project.
#
#  Copyright (C) 2010-2020 Alexander Bahle <alexander.bahle@hs-augsburg.de>
#                          Gundolf Kiefer <gundolf.kiefer@hs-augsburg.de>
#      Hochschule Augsburg, University of Applied Sciences
#
#  Description:
#    ParaNut top-level Makefile.
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


PREFIX ?= /opt/paranut

################################################################################
#                                                                              #
#     Default and help targets                                                 #
#                                                                              #
################################################################################


# Default target ...
.PHONY: all
all: help


# Print usage info ...
.PHONY: help
help:
	@echo "This makefile is mainly used to install SW sources to your system. Usually there sould be no need for you to use it."
	@echo
	@echo "Usage: make [<target>] [<parameter>=<value> ...]"
	@echo
	@echo "Targets:"
	@echo
	@echo "  install     : Install software source files to the directory given by parameter PREFIX"
	@echo "                (default: /opt/paranut)"


################################################################################
#                                                                              #
#     Installing                                                               #
#                                                                              #
################################################################################

# RISC-V software files
SRC_FILES += riscv_common/paranut.ld \
             riscv_common/encoding.h \
             riscv_common/syscalls.c \
             riscv_common/startup.S

SRC_FILES += libparanut/README \
             libparanut/Makefile \
             libparanut/paranut.h \
             libparanut/pn_config.h \
             libparanut/common/common.h \
             libparanut/common/common_RV32I.S \
             libparanut/common/custom_RV32I.S \
             libparanut/pn_base/pn_base.c \
             libparanut/pn_base/pn_base_RV32I.S \
             libparanut/pn_cache/pn_cache.c \
             libparanut/pn_cache/pn_cache_RV32I_buildscript.py \
             libparanut/pn_cache/pn_cache_RV32I_auto.S \
             libparanut/pn_cache/pn_cache_RV32I_32.S \
             libparanut/pn_cache/pn_cache_RV32I_64.S \
             libparanut/pn_cache/pn_cache_RV32I_128.S \
             libparanut/pn_cache/pn_cache_RV32I_256.S \
             libparanut/pn_cache/pn_cache_RV32I_512.S \
             libparanut/pn_cache/pn_cache_RV32I_1024.S \
             libparanut/pn_cache/pn_cache_RV32I_2048.S \
             libparanut/pn_exception/pn_exception.c \
             libparanut/pn_exception/pn_exception_RV32I.S \
             libparanut/pn_link/pn_link.c \
             libparanut/pn_link/pn_link_RV32I.S \
             libparanut/pn_spinlock/pn_spinlock.c \
             libparanut/pn_spinlock/pn_spinlock_RV32I.S \
             libparanut/pn_thread/pn_thread.c \
             libparanut/pn_thread/pn_thread_RV32I.S

SRC_FILES += hello_newlib/hello_newlib.c \
             hello_newlib/Makefile

SRC_FILES += ParaNut.mk

# Install 

.PHONY: install
install:
	@echo
	@echo "### Installing SW sources to $(PREFIX) ###"
	@if [ $(PREFIX) ]; then \
	  # Install SRC_FILES \
	  for FILE in $(SRC_FILES); do \
	    install -Dp -m 644 -t $(PREFIX)/src/sw/`dirname $$FILE` $$FILE; \
	  done; \
	  # Install libparanut lib and includes \
	  $(MAKE) -C libparanut install SUFFIX=libparanut; \
	fi

