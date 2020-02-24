export
SCRIPTS_DIR=../../../../bin
ENTITY_NAME = mmemu
WORKLIB=paranut
# uncomment if UNISIM and/or XILINXCORELIB libraries needed
#NEED_UNISIM=1

# Simulation...
VHDL_SRC_PATH = ../../paranut:../../uart_16750:../paranut
SIM_MODULE_OBJ = $(ENTITY_NAME).o $(ENTITY_NAME)_tb.o wb_memory.o mbusif.o marbiter.o mreadport.o mwriteport.o mtagram.o mbankram.o mem_inferred.o lfsr.o

# Synthesis
XST_MODULE_SRC = $(ENTITY_NAME).vhd paranut_config.vhd paranut_lib.vhd types.vhd memu_lib.vhd memu.vhd mmemu.vhd mbusif.vhd marbiter.vhd mreadport.vhd mwriteport.vhd mtagram.vhd mbankram.vhd mem_tech.vhd mem_inferred.vhd lfsr.vhd histogram.vhd
XILINX_DEVICE_PART=xc5vlx110t-ff1136-1
#XILINX_DEVICE_PART=xc3s700a-fg484-4
GENERICS=""
IOBUF="no"

# Include this at last
include $(SCRIPTS_DIR)/Makefile.inc
