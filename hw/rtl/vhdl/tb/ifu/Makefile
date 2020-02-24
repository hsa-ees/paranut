export
SCRIPTS_DIR=../../../../bin
ENTITY_NAME = mifu
WORKLIB=paranut
# uncomment if UNISIM and/or XILINXCORELIB libraries needed
#NEED_UNISIM=1

# Simulation...
VHDL_SRC_PATH = ../../paranut
SIM_MODULE_OBJ = $(ENTITY_NAME).o $(ENTITY_NAME)_tb.o ifu.o paranut_lib.o types.o memu_lib.o

# Synthesis
XST_MODULE_SRC = $(ENTITY_NAME).vhd paranut_config.vhd paranut_lib.vhd ifu.vhd types.vhd memu_lib.vhd orbis32.vhd histogram.vhd
XILINX_DEVICE_PART=xc5vlx110t-ff1136-1
#XILINX_DEVICE_PART=xc3s700a-fg484-4
GENERICS=""
IOBUF="no"

# Include this at last
include $(SCRIPTS_DIR)/Makefile.inc
