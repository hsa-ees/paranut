#  -----------------------------------------------------------------------------
#
#  This file is part of the ParaNut project.
#
#  Copyright (C) 2023 Felix Wagner <felix.wagner1@hs-augsburg.de>
#      Hochschule Augsburg, University of Applied Sciences
#
#  Description:
#    This file provides variables describing the locations of relevant files 
#    in the ParaNut Project Architecture. It is included in most makefiles to 
#    ennsure consistent naming and paths. It also introduces some naming 
#    conventions for variables not defined here.
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

#################  Define important Directories ################################ 
# default for all makefies including this before defining targets
.PHONY: default
default: build

# Remember what variables already existed
VARS_BEFORE_DIRECTORY_BASE := $(.VARIABLES)

#################  Define important Directories ################################ 
# PARANUT_HOME must be given as an absolute path and set in EVERY MAKEFILE
# (It could also be set from some environment variable, but I dislike that)
export PARANUT_HOME ?= 

# Contains all general hw information
export HW_DIR ?= $(PARANUT_HOME)/hw
export HW_TOOLS_DIR ?= $(HW_DIR)/tools

# Contains systemc modules
export SYSC_DIR ?= $(HW_DIR)/sysc
# common module contains means to build paranut-config.h
export SYSC_COMMON_DIR ?= $(SYSC_DIR)/module_common

# Contains sw sources (hal, applications and tests)
export SW_DIR ?= $(PARANUT_HOME)/sw
export EXTERNAL_DIR ?= $(PARANUT_HOME)/external

# Contains software libraries abstracting the ParaNut hardware
export HAL_DIR ?= $(SW_DIR)/hal

export RISCV_COMMON_DIR ?= $(HAL_DIR)/riscv_common

# Software directories:
# Contains applications for the ParaNut
export SW_APPLICATIONS_DIR ?= $(SW_DIR)/applications
# Contains test-applications for the ParaNut
export SW_TEST_APPLICATIONS_DIR ?= $(SW_DIR)/test_applications
# Contains operating Systems for the ParaNut
export SW_OS_DIR ?= $(SW_DIR)/os

# Contains software libraries abstracting the ParaNut hardware
export TOOLS_DIR ?= $(PARANUT_HOME)/tools
export BIN_DIR ?= $(TOOLS_DIR)/bin

# Contains sources for creating documentation
export DOC_SRC_DIR ?= $(PARANUT_HOME)/doc-src

# Contains documentation
export DOC_DIR ?= $(PARANUT_HOME)/doc

# Contains the config creator
export CFG_CREATOR_DIR ?= $(PARANUT_HOME)/config-creator

# Systems define ParaNut hardware configurations and may contain additional
# hw modules as well as software
# variables concerning this structure are prefixed with PNS (ParaNutSystem)
# per default, all simulations are run using the pn-sim system
# as the ParaNut configuration affects hardware/simulator builds as well as 
# the hal-software-libraries, all libraries, binaries, headers and other artifacts 
# created when building libraries and hardware sources are stored inside 
# the corresponding system folder structure
export PN_SYSTEM_DEFAULT ?= refdesign
export PN_SYSTEM ?= $(PN_SYSTEM_DEFAULT)

export SYSTEMS_DIR ?= $(PARANUT_HOME)/systems
export PNS_DIR ?= $(SYSTEMS_DIR)/$(PN_SYSTEM)

# PNS Hardware Directories
export PNS_HW_DIR ?= $(PNS_DIR)/hw
export PNS_SYSC_DIR ?= $(PNS_HW_DIR)/sysc
export PNS_SYSC_INC_DIR ?= $(PNS_SYSC_DIR)/include
export PNS_SYSC_LIB_DIR ?= $(PNS_SYSC_DIR)/lib
export PNS_SYSC_SV_DIR ?= $(PNS_SYSC_DIR)/sv_out
export PNS_SYSC_V_DIR ?= $(PNS_SYSC_DIR)/v_out
export PNS_SYSC_BUILD_DIR ?= $(PNS_SYSC_DIR)/build

# PNS Software Directories
export PNS_SW_DIR ?= $(PNS_DIR)/sw
export PNS_HAL_INC_DIR ?= $(PNS_SW_DIR)/include
export PNS_HAL_LIB_DIR ?= $(PNS_SW_DIR)/lib

#################  Install Target Dir ########################################## 
export INSTALL_DIR_DEFAULT ?= /opt/paranut
export INSTALL_DIR ?= $(INSTALL_DIR_DEFAULT)


################# Variable conventions ######################################### 
# We define three types of automatic make directories:

# APPLICATION_DIR: containings software to be cross compiled for a ParaNut System
# LIBRARY_DIR: contains sources for a library to be used in paranut applications
# SYSC_MODULE_DIR: contains a systemc module that can be simulated and/or synthesised


#################  Define config Files ######################################### 
# The main config describing the default ParaNut System
export CONFIG_MK ?= $(PARANUT_HOME)/config.mk

# The system config that may override settings from CONFIG_MK
export PNS_CONFIG_MK ?= $(PNS_DIR)/config.mk

# The systems config.h (created upon building hardware/simulation)
export CONFIG_H ?= $(PNS_SYSC_INC_DIR)/paranut-config.h

# The systems config.h (created upon building hardware/simulation)
export VERSION_ENV ?= $(PARANUT_HOME)/version.env


#################  Define Master Make Files #################################### 
# Makefile for systemC hardware modules
export SYSC_BASE_MK ?= $(TOOLS_DIR)/sysc-base.mk

# makefile for simple riscv applications
export APPLICATION_BASE_MK ?= $(TOOLS_DIR)/application-base.mk

# Makefile for the ParaNut's HAL
export HAL_BASE_MK ?= $(TOOLS_DIR)/hal-base.mk

# Config file for SystemC
export SYSTEMC_CONFIG_MK ?= $(PARANUT_HOME)/systemc-config.mk

#################  SIMULATOR BINARY ############################################ 
export PN_SIM_BIN_NAME ?= pn-sim
export PN_SIM_BIN ?= $(PNS_SYSC_DIR)/$(PN_SIM_BIN_NAME)



#################  Target to export Variables to other system (e.g. shell) #####
# usage in a shell script:
# eval $(make -C $PARANUT_HOME --silent get-variables IN_SHELL=1)
# GET_VAR_GUARD is used to exclude the target when including this file normally, 
# this supresses warnings
ifndef GET_VAR_GUARD
.PHONY: get-variables
get-variables:
	$(foreach v,                                        \
      $(filter-out $(VARS_BEFORE_DIRECTORY_BASE) VARS_BEFORE_DIRECTORY_BASE,$(.VARIABLES)), \
      $(info $(v)="$($(v))"))
endif

export GET_VAR_GUARD ?= 1