#  -----------------------------------------------------------------------------
#
#  This file is part of the ParaNut project.
#
#  Copyright (C) 2010-2020 Alexander Bahle <alexander.bahle@hs-augsburg.de>
#                          Michael Schaeferling <michael.schaeferling@hs-augsburg.de>
#                          Gundolf Kiefer <gundolf.kiefer@hs-augsburg.de>
#					  2023 Elias Schuler <elias.schuler@hs-augsburg.de>
#					       Lukas Bauer <lukas.bauer@hs-aubsburg.de>
#      Hochschule Augsburg, University of Applied Sciences
#
#  Description:
#    Makefile include file for the ParaNut processor.
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


export
default: ghdl-cl

VPATH = $(VHDL_SRC_PATH): \
        $(WORKLIB)

IP_XILINX_TARGET_DIR ?= paranut_ip


# ghdl simulation
ghdl-%:
	@$(MAKE) -f $(SCRIPTS_DIR)/ghdl.mk $@

# IP core (Xilinx tools)
build-ip-xilinx: FORCE_PKG_UPDATE
	vivado -source paranut.tcl -mode tcl -tclargs --projdir "$(IP_XILINX_TARGET_DIR)"
	rm -f vivado*.log vivado*.jou


### Generate paranut_config.vhd ###

# include global configuration
-include ../../config.mk

PARANUT_CONFIG=../vhdl/paranut/paranut_config.vhd
PARANUT_CONFIG_TEMPLATE=../vhdl/paranut/paranut_config.template.vhd
UART_ADDRESS=$(shell printf "x\\\"%X\\\"" $(CFG_UART_BASE_ADDRESS))
GPIO_ADDRESS=$(shell printf "x\\\"%X\\\"" $(CFG_GPIO_BASE_ADDRESS))

../../paranut/paranut_config.vhd: FORCE_PKG_UPDATE
FORCE_PKG_UPDATE:
	@echo "### Updating paranut_config.vhd ...";

	sed \
	-e 's#{CFG_NUT_CPU_CORES_LD}#$(CFG_NUT_CPU_CORES_LD)#g' \
	-e 's#{CFG_NUT_CPU_CAP1_CORES}#$(CFG_NUT_CPU_CAP1_CORES)#g' \
	-e 's#{CFG_NUT_MEM_SIZE}#$(CFG_NUT_MEM_SIZE)#g' \
	-e 's#{CFG_NUT_EX_INT}#$(CFG_NUT_EX_INT)#g' \
	-e 's#{CFG_EXU_M_EXTENSION}#$(CFG_EXU_M_EXTENSION)#g' \
	-e 's#{CFG_EXU_PERFCOUNT_ENABLE}#$(CFG_EXU_PERFCOUNT_ENABLE)#g' \
	-e 's#{CFG_EXU_PERFCOUNTER_BITS}#$(CFG_EXU_PERFCOUNTER_BITS)#g' \
	-e 's#{CFG_EXU_PERFCOUNTERS_LD}#$(CFG_EXU_PERFCOUNTERS_LD)#g' \
	-e 's#{CFG_MEMU_CACHE_BANKS_LD}#$(CFG_MEMU_CACHE_BANKS_LD)#g' \
	-e 's#{CFG_MEMU_CACHE_SETS_LD}#$(CFG_MEMU_CACHE_SETS_LD)#g' \
	-e 's#{CFG_MEMU_CACHE_WAYS_LD}#$(CFG_MEMU_CACHE_WAYS_LD)#g' \
	-e 's#{CFG_MEMU_CACHE_REPLACE_LRU}#$(CFG_MEMU_CACHE_REPLACE_LRU)#g' \
	-e 's#{CFG_MEMU_ARBITER_METHOD}#$(CFG_MEMU_ARBITER_METHOD)#g' \
	-e 's#{CFG_MEMU_BUSIF_WIDTH}#$(CFG_MEMU_BUSIF_WIDTH)#g' \
	-e 's#{CFG_IFU_IBUF_SIZE_LD}#$(CFG_IFU_IBUF_SIZE_LD)#g' \
	-e 's#{CFG_LSU_WBUF_SIZE_LD}#$(CFG_LSU_WBUF_SIZE_LD)#g' \
	-e 's#{CFG_PRIV_LEVELS}#$(CFG_PRIV_LEVELS)#g' \
	-e 's#{CFG_UART_ENABLE}#$(CFG_UART_ENABLE)'#g \
	-e 's#{CFG_UART_BASE_ADDRESS}#$(UART_ADDRESS)'#g \
	-e 's#{CFG_GPIO_ENABLE}#$(CFG_GPIO_ENABLE)'#g \
	-e 's#{CFG_GPIO_BASE_ADDRESS}#$(GPIO_ADDRESS)'#g \
	-e 's#{CFG_GPIO_AMOUNT}#$(CFG_GPIO_AMOUNT)'#g \
	-e 's#{CFG_GPIO_OUTAMOUNT}#$(CFG_GPIO_OUTAMOUNT)'#g \
	$(PARANUT_CONFIG_TEMPLATE) > $(PARANUT_CONFIG)


##### Clean #####
clean:
	@$(MAKE) -f $(SCRIPTS_DIR)/ghdl.mk ghdl-$@
