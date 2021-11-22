export
SCRIPTS_DIR=../../../tools
ENTITY_NAME = mbankram
WORKLIB=paranut

# Simulation...
VHDL_SRC_PATH = ../../paranut
SIM_MODULE_OBJ = $(ENTITY_NAME).o $(ENTITY_NAME)_tb.o mem_inferred.o

# Synthesis
XST_MODULE_SRC = $(ENTITY_NAME).vhd mem_tech.vhd mem_inferred.vhd paranut_lib.vhd types.vhd memu_lib.vhd paranut_config.vhd histogram.vhd
XILINX_DEVICE_PART=xc5vlx110t-ff1136-1
#XILINX_DEVICE_PART=xc3s700a-fg484-4
GENERICS=""
IOBUF="no"

# Include this at last
include $(SCRIPTS_DIR)/Makefile.inc
