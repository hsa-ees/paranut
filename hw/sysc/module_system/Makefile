#  -----------------------------------------------------------------------------
#
#  This file is part of the ParaNut project.
#
#  Copyright (C) 2023 Felix Wagner <felix.wagner1@hs-augsburg.de>
#      Hochschule Augsburg, University of Applied Sciences
#
#  Description:
#    TODO
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

##### EDIT HERE
################# Module Configuration #########################################

# MODULE_SRC:               Sources that are used for Simulation as well as 
#                           synthesis. (including the testbench/sc_main file)
MODULE_SRC = intc.cpp interconnect.cpp nut.cpp paranutsystem.cpp

# TESTBENCH_SRC:            Source that contains tb (and thus sc_main) 
TESTBENCH_SRC = 

# SIM_SRC:                  Sources that are only used in simulationon
SIM_SRC =

# MODULE_INTERDEPENDENCIES: Other modules this one depends on
#                           common is usually omnipresent
MODULE_INTERDEPENDENCIES = common cpu mmemu debug mtimer

# MODULE_INCLUDES:          Include directives for the compiler (e.g. -I..)
MODULE_INCLUDES = 

# MODULE_CFLAGS:            Additional compiler flags this module needs
MODULE_CFLAGS =

# MODULE_LDFLAGS:           Additional linker flags this module needs
MODULE_LDFLAGS =

# SYNTHESIS_TOPMODULE:      Defines the Top Module for synthesis as defined inside 
#							the test bench cpp
SYNTHESIS_TOP_MODULE = #TODO TESTBENCH AND SYNTHESIS


################################################################################
##### DO NOT EDIT FROM HERE ON #################################################
################################################################################

################# Module Makefile ##############################################
SYSC_MODULE_DIR := $(abspath $(CURDIR))
PARANUT_HOME ?= $(CURDIR)/../../..

################# Automatic Name Generation ####################################
# Get Module name from folder name
MODULE_NAME = $(subst module_,,$(lastword $(subst /, ,$(SYSC_MODULE_DIR))))


################# Directory Makefile Include #####################################
include $(PARANUT_HOME)/directory-base.mk

################# Master Makefile Include #####################################
include $(SYSC_BASE_MK)