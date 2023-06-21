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

################# Software Makefile ##############################################
APPLICATION_DIR:= $(abspath $(CURDIR))
PARANUT_HOME ?= $(APPLICATION_DIR)/../../..

################# Directory Makefile Include ###################################
include $(PARANUT_HOME)/directory-base.mk

##### EDIT HERE
################# Software Specific Variables ##################################
PN_EXTERNAL ?= $(PARANUT_HOME)/external
FREERTOS_KERNEL = $(PN_EXTERNAL)/FreeRTOS-Kernel
FREERTOS_LOCAL = ../freertos
FREERTOS_IO = $(FREERTOS_LOCAL)/FreeRTOS-Plus-IO

GIT_HTTPS ?= 0
ifeq ($(GIT_HTTPS),1)
PN_GIT_METHOD = https://
PN_GIT_PORT = 8443
else
PN_GIT_METHOD = ssh://git@
PN_GIT_PORT = 8022
endif

FREERTOS_SRC = $(PN_GIT_METHOD)ti-build.informatik.hs-augsburg.de:$(PN_GIT_PORT)/paranut_developers/freertos-kernel.git

FREERTOS_RISCV = $(FREERTOS_KERNEL)/portable/GCC/RISC-V

################# Software Configuration #######################################

# SOFTWARE_SRC:               Sources that are used for Simulation as well as 
#                            synthesis. (including the testbench/sc_main file)
SOFTWARE_SRC = main.c $(FREERTOS_KERNEL)/tasks.c $(FREERTOS_KERNEL)/list.c $(FREERTOS_KERNEL)/queue.c \
$(FREERTOS_RISCV)/portASM.S $(FREERTOS_KERNEL)/portable/MemMang/heap_3.c $(FREERTOS_RISCV)/port.c $(FREERTOS_LOCAL)/minilib/_clzsi2.c \
$(FREERTOS_IO)/Device/Paranut/FreeRTOS_paranut_uart.c $(FREERTOS_IO)/Device/Paranut/FreeRTOS_paranut_DriverInterface.c $(wildcard FreeRTOS-Plus-IO/Common/*.c)

# SOFTWARE_HEADERS:           Headers provided by the library that shall be copied 
#                            to include
SOFTWARE_HEADERS ?=  

# LIBRARY_DEPENDENCIES:      Paranut specific libraries, this software 
#                            depends on. Mainly used to make shure the respective 
#                            files are built
LIBRARY_DEPENDENCIES ?= libparanut

# SOFTWARE_CFLAGS:            Additional compiler flags this software needs
SOFTWARE_CFLAGS ?= -I. -I$(FREERTOS_KERNEL)/include -I$(FREERTOS_RISCV) -I$(FREERTOS_LOCAL) -I$(FREERTOS_LOCAL)/FreeRTOS-Plus-IO/Include -I$(FREERTOS_LOCAL)//FreeRTOS-Plus-IO/Device/Paranut  -D__paranut__ -MMD -I$(FREERTOS_LOCAL)//chip_specific_extensions/Paranut

# SOFTWARE_LDFLAGS:           Additional linker flags this software needs
SOFTWARE_LDFLAGS ?= 

ADDITIONAL_TARGETS ?= $(FREERTOS_KERNEL)/README.md

# CFG_MARCH:                 Configure the ISA
CFG_MARCH ?= rv32im


################# ALTERNATIVE CFLAGS AND DFLAGS  ###############################
# These are normally defined by software-base.mk
# however, the complexity of this module requires us to overwrite them
#CFLAGS = -g -O2 -march=$(CFG_MARCH) -mabi=ilp32 -I$(RISCV_COMMON_DIR) -I$(PNS_HAL_INC_DIR) 
SCFLAGS = $(CFLAGS) -DportasmHANDLE_INTERRUPT=external_interrupt_handler

################################################################################
##### DO NOT EDIT FROM HERE ON #################################################
################################################################################

################# Automatic Name Generation ####################################
# Get Module name from folder name
SOFTWARE_NAME = $(lastword $(subst /, ,$(APPLICATION_DIR)))

################# Master Makefile Include ######################################
include $(APPLICATION_BASE_MK)

################################################################################
##### CUSTOM TARGETS ###########################################################
################################################################################
##### EDIT HERE

# Prepare freertos by cloning it and checking out the given version
$(FREERTOS_KERNEL)/README.md:
	rm -rf $(FREERTOS_KERNEL); \
	mkdir -p $(FREERTOS_KERNEL); \
	git clone $(FREERTOS_SRC) $(FREERTOS_KERNEL)

software-clean:
	rm -rf $(FREERTOS_KERNEL)