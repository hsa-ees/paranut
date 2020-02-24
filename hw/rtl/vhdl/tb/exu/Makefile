export
SCRIPTS_DIR=../../../../bin
ENTITY_NAME = mexu
WORKLIB=paranut
# uncomment if UNISIM and/or XILINXCORELIB libraries needed
#NEED_UNISIM=1

# Simulation...
VHDL_SRC_PATH = ../../paranut
SIM_MODULE_OBJ = $(ENTITY_NAME).o $(ENTITY_NAME)_tb.o mem_inferred.o shift32.o mult32x32s.o mul_inferred.o mlsu.o mlsu_simple.o mifu.o mifu_bs.o rwports.o 

# Synthesis
XST_MODULE_SRC = $(ENTITY_NAME).vhd exu.vhd paranut_config.vhd paranut_lib.vhd types.vhd ifu.vhd memu_lib.vhd lsu.vhd regfile.vhd mem_tech.vhd mem_inferred.vhd shift32.vhd mult32x32s.vhd mul_tech.vhd mul_inferred.vhd orbis32.vhd mhistogram.vhd histogram.vhd
XILINX_DEVICE_PART=xc5vlx110t-ff1136-1
#XILINX_DEVICE_PART=xc3s700a-fg484-4
GENERICS=""
IOBUF="no"

# Include this at last
include $(SCRIPTS_DIR)/Makefile.inc
