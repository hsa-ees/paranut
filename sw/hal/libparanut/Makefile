####################################################################################################
# Copyright (C) 2019-2022 Alexander Bahle <alexander.bahle@hs-augsburg.de>
#                         Anna Pfuetzner <annakerstin.pfuetzner@gmail.com>
#                         Michael Schaeferling <michael.schaeferling@hs-augsburg.de>
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

PYTHON := python3

################# Library Configuration #########################################

# LIBRARY_SRC:               Sources that are used for Simulation as well as 
#                           synthesis. (including the testbench/sc_main file)
LIBRARY_C_SRC = 

# LIBRARY_HEADERS:          Headers provided by the library that shall be copied 
#                           to include
LIBRARY_HEADERS = paranut.h pn_config.h

# LIBRARY_DEPENDENCIES:     Other libraries this one depends on
#                           libparanut is usually omnypresent
LIBRARY_DEPENDENCIES ?= 

# LIBRARY_CFLAGS:            Additional compiler flags this module needs
LIBRARY_CFLAGS ?= -I./common

# LIBRARY_LDFLAGS:           Additional linker flags this module needs
LIBRARY_LDFLAGS ?=  

# CFG_MARCH:                 Configure the ISA
CFG_MARCH ?= rv32ima

################ ParaNut System Configuration ##################################

# Call clean when changing these!
PN_ISA                      = RV32I
# System Parameters
#   Cache Linesize in Bits, Default: auto
#   auto means all power of two linesizes from 32 to 2048
PN_CACHE_LINESIZE           = auto
#   Register Width in Bits, Default: 32
PN_RWIDTH                   = 32

# Switch for raw compilation (optimized for performance, will drop security checks)
# Default: 0
PN_COMPILE_RAW              = 0

# Chose modules - Set to 0 to switch off
# Default: 1
PN_WITH_BASE                = 1
PN_WITH_CACHE               = 1
PN_WITH_LINK                = 1
PN_WITH_THREAD              = 1
PN_WITH_EXCEPTION           = 1
PN_WITH_SPINLOCK            = 1

#Lists##############################################################################################

# Assemble object list so we know what objects to build
LIBRARY_C_SRC =
LIBRARY_ASM_SRC = 

ifeq ($(PN_WITH_BASE),1)
  PN_CONFIG_DEFINES       += -D PN_WITH_BASE
  BPATH                    = ./pn_base/pn_base
  LIBRARY_C_SRC           += $(BPATH).c
  LIBRARY_ASM_SRC         += $(BPATH)_$(PN_ISA).S
endif

ifeq ($(PN_WITH_LINK),1)
  PN_CONFIG_DEFINES       += -D PN_WITH_LINK
  LPATH                    = ./pn_link/pn_link
  LIBRARY_C_SRC           += $(LPATH).c 
  LIBRARY_ASM_SRC         += $(LPATH)_$(PN_ISA).S
endif

ifeq ($(PN_WITH_THREAD),1)
  PN_CONFIG_DEFINES       += -D PN_WITH_THREAD
  TPATH                    = ./pn_thread/pn_thread
  LIBRARY_C_SRC           += $(TPATH).c 
  LIBRARY_ASM_SRC         += $(TPATH)_$(PN_ISA).S
endif

ifeq ($(PN_WITH_CACHE),1)
  PN_CONFIG_DEFINES       += -D PN_WITH_CACHE
  CPATH                    = ./pn_cache/pn_cache
  LIBRARY_C_SRC           += $(CPATH).c 
  LIBRARY_ASM_SRC         += ./$(CPATH)_$(PN_ISA)_$(strip $(PN_CACHE_LINESIZE)).S
endif

ifeq ($(PN_WITH_EXCEPTION),1)
  PN_CONFIG_DEFINES       += -D PN_WITH_EXCEPTION
  EPATH                    = ./pn_exception/pn_exception
  LIBRARY_C_SRC           += $(EPATH).c 
  LIBRARY_ASM_SRC         += $(EPATH)_$(PN_ISA).S
endif

ifeq ($(PN_WITH_SPINLOCK),1)
  PN_CONFIG_DEFINES       += -D PN_WITH_SPINLOCK
  SPATH                    = ./pn_spinlock/pn_spinlock
  LIBRARY_C_SRC           += $(SPATH).c 
  LIBRARY_ASM_SRC         += $(SPATH)_$(PN_ISA).S
endif

# Check if object list is empty because it would not make any sense to build the library then
ifeq ($(strip $(LIBRARY_C_SRC)),)
  $(error No modules enabled.)
endif

# Add the objects for the common part
LIBRARY_ASM_SRC               += ./common/common_$(PN_ISA).S
LIBRARY_C_SRC                 += ./common/common.c
#Target Magic#######################################################################################

### Build targets ###

# Creates all cache files for RV32I anew in case of changes in buildscript.
# For development only.
.PHONY: build_cache_all
build_cache_all: ./pn_cache/pn_cache_RV32I_buildscript.py
	$(PYTHON) $< 32
	$(PYTHON) $< 64
	$(PYTHON) $< 128
	$(PYTHON) $< 256
	$(PYTHON) $< 512
	$(PYTHON) $< 1024
	$(PYTHON) $< 2048
	$(PYTHON) $< auto


# Cache Module Assembly Code for RV32I is generated at compile time
./pn_cache/pn_cache_RV32I_$(PN_CACHE_LINESIZE).S: ./pn_cache/pn_cache_RV32I_buildscript.py
	$(PYTHON) $< $(strip $(PN_CACHE_LINESIZE))

#  Generate pn_config header from system parameters ...
.PHONY: update-config
update-config: pn_config.h


pn_config.h: $(CONFIG_MK)
	@echo "Updating 'pn_config.h' ..."
	@echo "/* Automatically generated file. See Makefile. No edits here! */"  > pn_config.h.new
	@echo "#define PN_CACHE_LINESIZE $(PN_CACHE_LINESIZE)"                   >> pn_config.h.new
	@echo "#define PN_RWIDTH $(PN_RWIDTH)"                                   >> pn_config.h.new
	@if [ $(PN_COMPILE_RAW) -eq 1 ];      \
		then echo "#define PN_COMPILE_RAW"     >> pn_config.h.new; fi
	@if [ $(PN_WITH_BASE) -eq 1 ];        \
		then echo "#define PN_WITH_BASE"       >> pn_config.h.new; fi
	@if [ $(PN_WITH_CACHE) -eq 1 ];       \
		then echo "#define PN_WITH_CACHE"      >> pn_config.h.new; fi
	@if [ $(PN_WITH_EXCEPTION) -eq 1 ];   \
		then echo "#define PN_WITH_EXCEPTION"  >> pn_config.h.new; fi
	@if [ $(PN_WITH_LINK) -eq 1 ];        \
		then echo "#define PN_WITH_LINK"       >> pn_config.h.new; fi
	@if [ $(PN_WITH_THREAD) -eq 1 ];      \
		then echo "#define PN_WITH_THREAD"     >> pn_config.h.new; fi
	@if [ $(PN_WITH_SPINLOCK) -eq 1 ];    \
		then echo "#define PN_WITH_SPINLOCK"   >> pn_config.h.new; fi
	@diff -q pn_config.h pn_config.h.new > /dev/null 2>&1 || mv pn_config.h.new pn_config.h
	@rm -f pn_config.h.new


################# Libraray Makefile ##################################################################
LIBRARY_DIR := $(abspath $(CURDIR))
PARANUT_HOME ?= $(LIBRARY_DIR)/../../..

################# Automatic Name Generation ####################################
# Get Module name from folder name
LIBRARY_NAME := $(lastword $(subst /, ,$(LIBRARY_DIR)))

################# Directory Makefile Include #####################################
include $(PARANUT_HOME)/directory-base.mk

################# Master Makefile Include #####################################
include $(HAL_BASE_MK)



### Install documentation ###

# Doxygen targets (only needed in source repository)
# Target 'install-doc' is provided in the Doxygen Makefile fragment:
-include Documentation/Doxygen.mk


