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
MODULE_SRC = wb8_uart_16750.cpp uart_transmitter.cpp uart_receiver.cpp uart_interrupt.cpp \
			uart_baudgen.cpp slib_mv_filter.cpp slib_input_sync.cpp slib_input_filter.cpp \
			slib_input_filter_2.cpp slib_fifo.cpp slib_fifo_11.cpp slib_edge_detect.cpp \
			slib_counter.cpp slib_clock_div.cpp \
			slib_clock_div_1.cpp 

# TESTBENCH_SRC:            Source that contains tb (and thus sc_main) 
TESTBENCH_SRC = uart_tb/wb8_uart_16750/wb8_uart_16750_tb.cpp

# SIM_SRC:                  Sources that are only used in simulationon
SIM_SRC =

# MODULE_INTERDEPENDENCIES: Other modules this one depends on
#                           common is usually omnipresent
MODULE_INTERDEPENDENCIES = common

# MODULE_INCLUDES:          Include directives for the compiler (e.g. -I..)
MODULE_INCLUDES = 

# MODULE_CFLAGS:            Additional compiler flags this module needs

MODULE_CFLAGS =

# MODULE_LDFLAGS:           Additional linker flags this module needs
MODULE_LDFLAGS =

# SYNTHESIS_TOPMODULE:      Defines the Top Module for synthesis as defined inside 
#							the test bench cpp
SYNTHESIS_TOP_MODULE = wb8_uart_16750


#################### CONFIG_H Parsing ##########################################
# The config module deviates slightly from all other module makefiles,
# as it additionaly has the task of creating the paranut-config.h file by 
# parsing $(CONFIG_MK) [defined in include $(MASTER_MAKEFILE)] and setting the 
# according variables in $(CONFIG_H_TEMPLATE) 

# Update local paranut-config.h
$(CONFIG_H): $(VERSION_FILE) $(CONFIG_MK) $(CONFIG_H_TEMPLATE)
	$(eval CFG_NUT_MIMPID= $(shell grep CFG_NUT_MIMPID $(VERSION_FILE) | sed 's#^.*=##' ))
	@echo "### Updating $(CONFIG_H)..."
	@sed \
	-e 's#{CFG_NUT_SIM_CLK_SPEED}#$(CFG_NUT_SIM_CLK_SPEED)#g' \
	-e 's#{CFG_NUT_RESET_ADDR}#$(CFG_NUT_RESET_ADDR)#g' \
	-e 's#{CFG_NUT_SIM_MAX_PERIPHERY}#$(CFG_NUT_SIM_MAX_PERIPHERY)#g' \
	-e 's#{CFG_NUT_CPU_CORES_LD}#$(CFG_NUT_CPU_CORES_LD)#g' \
	-e 's#{CFG_NUT_CPU_CAP1_CORES}#$(CFG_NUT_CPU_CAP1_CORES)#g' \
	-e 's#{CFG_NUT_MEM_SIZE}#$(CFG_NUT_MEM_SIZE)#g' \
	-e 's#{CFG_NUT_EX_INT}#$(CFG_NUT_EX_INT)#g' \
	-e 's#{CFG_EXU_M_EXTENSION}#$(CFG_EXU_M_EXTENSION)#g' \
	-e 's#{CFG_EXU_A_EXTENSION}#$(CFG_EXU_A_EXTENSION)#g' \
	-e 's#{CFG_EXU_PERFCOUNT_ENABLE}#$(CFG_EXU_PERFCOUNT_ENABLE)#g' \
	-e 's#{CFG_EXU_PERFCOUNTER_BITS}#$(CFG_EXU_PERFCOUNTER_BITS)#g' \
	-e 's#{CFG_EXU_PERFCOUNTERS_LD}#$(CFG_EXU_PERFCOUNTERS_LD)#g' \
	-e 's#{CFG_MEMU_CACHE_BANKS_LD}#$(CFG_MEMU_CACHE_BANKS_LD)#g' \
	-e 's#{CFG_MEMU_CACHE_SETS_LD}#$(CFG_MEMU_CACHE_SETS_LD)#g' \
	-e 's#{CFG_MEMU_CACHE_WAYS_LD}#$(CFG_MEMU_CACHE_WAYS_LD)#g' \
	-e 's#{CFG_MEMU_CACHE_REPLACE_LRU}#$(CFG_MEMU_CACHE_REPLACE_LRU)#g' \
	-e 's#{CFG_MMU_TLB_ENTRIES_LD}#$(CFG_MMU_TLB_ENTRIES_LD)#g' \
	-e 's#{CFG_MMU_TLB_ENABLE}#$(CFG_MMU_TLB_ENABLE)#g' \
	-e 's#{CFG_MEMU_ARBITER_METHOD}#$(CFG_MEMU_ARBITER_METHOD)#g' \
	-e 's#{CFG_MEMU_BUSIF_WIDTH}#$(CFG_MEMU_BUSIF_WIDTH)#g' \
	-e 's#{CFG_IFU_IBUF_SIZE_LD}#$(CFG_IFU_IBUF_SIZE_LD)#g' \
	-e 's#{CFG_LSU_WBUF_SIZE_LD}#$(CFG_LSU_WBUF_SIZE_LD)#g' \
	-e 's#{CFG_PRIV_LEVELS}#$(CFG_PRIV_LEVELS)#g' \
	-e 's#{CFG_NUT_MIMPID}#$(CFG_NUT_MIMPID)#g' \
	-e 's#{CFG_NUT_MTIMER_TIMEBASE_US}#$(CFG_NUT_MTIMER_TIMEBASE_US)#g' \
	-e 's#{CFG_NUT_MTIMER_ADDR}#$(CFG_NUT_MTIMER_ADDR)#g' \
	-e 's#{CFG_UART_ENABLE}#$(CFG_UART_ENABLE)'#g \
	-e 's#{CFG_UART_BASE_ADDRESS}#$(CFG_UART_BASE_ADDRESS)'#g \
	-e 's#{CFG_GPIO_ENABLE}#$(CFG_GPIO_ENABLE)'#g \
	-e 's#{CFG_GPIO_BASE_ADDRESS}#$(CFG_GPIO_BASE_ADDRESS)'#g \
	-e 's#{CFG_GPIO_AMOUNT}#$(CFG_GPIO_AMOUNT)'#g \
	-e 's#{CFG_GPIO_OUTAMOUNT}#$(CFG_GPIO_OUTAMOUNT)'#g \
	$(CONFIG_H_TEMPLATE) > $(CONFIG_H)


################################################################################
##### DO NOT EDIT FROM HERE ON #################################################
################################################################################

################# Module Makefile ##############################################
SYSC_MODULE_DIR:= $(abspath $(CURDIR))
PARANUT_HOME ?= $(CURDIR)/../../..

################# Automatic Name Generation ####################################
# Get Module name from folder name
MODULE_NAME = $(subst module_,,$(lastword $(subst /, ,$(SYSC_MODULE_DIR))))


################# Directory Makefile Include #####################################
include $(PARANUT_HOME)/directory-base.mk

################# Master Makefile Include #####################################
include $(SYSC_BASE_MK)