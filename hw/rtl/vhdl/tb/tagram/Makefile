export
SCRIPTS_DIR=../../../../bin
ENTITY_NAME = mtagram
WORKLIB=paranut
# uncomment if UNISIM and/or XILINXCORELIB libraries needed
#NEED_UNISIM=1

# Simulation...
VHDL_SRC_PATH = ../../paranut
SIM_MODULE_OBJ = $(ENTITY_NAME).o $(ENTITY_NAME)_tb.o mem_inferred.o

# Synthesis
XST_MODULE_SRC = $(ENTITY_NAME).vhd mem_tech.vhd mem_inferred.vhd paranut_lib.vhd types.vhd memu_lib.vhd paranut_config.vhd lfsr.vhd histogram.vhd
XILINX_DEVICE_PART=xc5vlx110t-ff1136-1
#XILINX_DEVICE_PART=xc3s700a-fg484-4
GENERICS=""
IOBUF="no"

# Include this at last
include $(SCRIPTS_DIR)/Makefile.inc
