####################################################################################################
# This file is part of the ParaNut project
# Copyright (C) 2021-2023 Felix Wagner <Felix.Wagner1@hs-augsburg.de>
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
# Date: 17.02.2023
####################################################################################################


################# Module Configuration #########################################

# MODULE_SRC:               Sources that are used for Simulation as well as 
#                           synthesis. (including the testbench/sc_main file)
LIBRARY_C_SRC = libgpio.c

# LIBRARY_HEADERS:          Headers provided by the library that shall be copied 
#                           to include
LIBRARY_HEADERS =  libgpio.h

# MODULE_INTERDEPENDENCIES: Other modules this one depends on
#                           common is usually omnypresent
LIBRARY_DEPENDENCIES = libparanut

# MODULE_INCLUDES:          Include directives for the compiler (e.g. -I..)
MODULE_INCLUDES = 

# MODULE_CFLAGS:            Additional compiler flags this module needs
LIBRARY_CFLAGS = 

# MODULE_LDFLAGS:           Additional linker flags this module needs
LIBRARY_LDFLAGS =  

# CUSTOM_BUILD_TARGETS:      These targets will be executed before any targets
#                            defined by the include makefile when calling
#                            make build
CUSTOM_BUILD_TARGETS ?= 


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





# includes the Doxygen.mk which creates the Doxygen Documentation
-include doc-src/Doxygen.mk