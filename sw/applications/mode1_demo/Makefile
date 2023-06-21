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
################# Software Configuration #########################################

# SOFTWARE_SRC:               Sources that are used for Simulation as well as 
#                            synthesis. (including the testbench/sc_main file)
SOFTWARE_SRC ?= mode1_demo.c

# LIBRARY_DEPENDENCIES:      Paranut specific libraries, this software 
#                            depends on. Mainly used to make shure the respective 
#                            files are built
LIBRARY_DEPENDENCIES ?= libparanut

# SOFTWARE_CFLAGS:            Additional compiler flags this software needs
SOFTWARE_CFLAGS ?= 

# SOFTWARE_LDFLAGS:           Additional linker flags this software needs
SOFTWARE_LDFLAGS ?=  

# SYSTEM:					  Defines the system the software is run on
#                             in simulation as well as on hardware
PN_SYSTEM ?= refdesign


################################################################################
##### DO NOT EDIT FROM HERE ON #################################################
################################################################################

################# Module Makefile ##############################################
APPLICATION_DIR:= $(abspath $(CURDIR))
PARANUT_HOME ?= $(CURDIR)/../../..

################# Automatic Name Generation ####################################
# Get Module name from folder name
SOFTWARE_NAME = $(lastword $(subst /, ,$(APPLICATION_DIR)))


################# Directory Makefile Include ###################################
include $(PARANUT_HOME)/directory-base.mk

################# Master Makefile Include ######################################
include $(APPLICATION_BASE_MK)

################################################################################
##### CUSTOM TARGETS ###########################################################
################################################################################
##### EDIT HERE
