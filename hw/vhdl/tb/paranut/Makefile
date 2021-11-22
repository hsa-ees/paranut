export
SCRIPTS_DIR=../../../tools
ENTITY_NAME=mparanut
WORKLIB=paranut

SIM_APP_DIR=apps_riscv/
SIM_APP=dhry_prog_mem.vhd

SIM_APP_TB=prog_mem.vhd

# Simulation...
VHDL_SRC_PATH=../../paranut:../../uart_16750:../../counter

# HLS_OBJs are prone to change -> using wildcards
HLS_SRC_FILES := $(wildcard ../../paranut/MLsu*.vhd) \
                 $(wildcard ../../paranut/MIfu*.vhd) \
                 $(wildcard ../../paranut/MCsr*.vhd) \
                 $(wildcard ../../paranut/MExu*.vhd) \
                 $(wildcard ../../paranut/MDebugModule*.vhd) \
                 $(wildcard ../../paranut/MDtm*.vhd) \
                 $(wildcard ../../paranut/MIntC*.vhd)
# Add Extension files according to configuration
#ifeq ($(CFG_EXU_M_EXTENSION),1)
HLS_SRC_FILES += $(wildcard ../../paranut/MMExtension*.vhd)
#endif
HLS_OBJ := $(patsubst ../../paranut/%.vhd,%.o,$(HLS_SRC_FILES))


MEMU_OBJ = mmemu.o mtagram.o mbankram.o mbusif.o mreadport.o mwriteport.o marbiter.o
PARANUT_OBJ = $(MEMU_OBJ) $(HLS_OBJ) mem_inferred.o mcsr.o mexu.o mifu.o mintc.o mlsu.o mdbg.o mparanut.o
SIM_MODULE_OBJ = $(ENTITY_NAME)_tb.o $(PARANUT_OBJ) wb_memory.o wb_uart.o slib_clock_div.o slib_counter.o slib_edge_detect.o slib_fifo.o slib_input_filter.o slib_input_sync.o slib_mv_filter.o txt_util.o uart_baudgen.o uart_interrupt.o uart_receiver.o uart_transmitter.o wb8_uart_16750.o mhistogram.o wb_counter_wrapper.o wb_counter.o counter_top.o

# Synthesis...
XST_MODULE_SRC = $(ENTITY_NAME).vhd types.vhd paranut_config.vhd paranut_lib.vhd memu.vhd mmemu.vhd intc.vhd mintc.vhd ifu.vhd mifu.vhd lsu.vhd mlsu.vhd memu_lib.vhd mem_inferred.vhd mul_inferred.vhd exu.vhd mexu.vhd mem_tech.vhd mtagram.vhd mbankram.vhd mreadport.vhd mwriteport.vhd marbiter.vhd mbusif.vhd lfsr.vhd histogram.vhd mhistogram.vhd
IOBUF="no"
XILINX_DEVICE_PART=xc5vlx110t-ff1136-1
#XILINX_DEVICE_PART=xc3s700a-fg484-4

################################################################################
include $(SCRIPTS_DIR)/Makefile.inc


################################################################################


$(SIM_APP_TB):
	ln -sf $(SIM_APP_DIR)$(SIM_APP) $@


.PHONY: clean
clean:
	rm -rf $(SIM_APP_TB)
