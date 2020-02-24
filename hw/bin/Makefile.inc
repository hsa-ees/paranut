export
default: ghdl-cl

VPATH = $(VHDL_SRC_PATH): \
        $(WORKLIB): \
		$(UNISIM): \
		$(XCL)

IP_TARGET_DIR ?= paranut_ip

# ghdl simulation
ghdl-%:
	@$(MAKE) -f $(SCRIPTS_DIR)/ghdl.mk $@
# xilinx isim simulation
isim-%:
	@$(MAKE) -f $(SCRIPTS_DIR)/isim.mk $@
# xilinx xst synthesis
xst-%:
	@$(MAKE) -f $(SCRIPTS_DIR)/xst.mk $@

# xilinx ip core
ip: FORCE_PKG_UPDATE
	vivado -source paranut.tcl -mode tcl -tclargs --projdir "$(IP_TARGET_DIR)"
	rm -f vivado*.log vivado*.jou 

# include global configuration
-include ../../config

CONFIG=../rtl/vhdl/paranut/paranut_config.vhd
CONFIG_TEMPLATE=../rtl/vhdl/paranut/paranut_config.template.vhd

../../paranut/paranut_config.vhd: FORCE_PKG_UPDATE 
FORCE_PKG_UPDATE:
	@echo; echo "### Updating paranut_config.vhd ..."; echo	
	sed \
	-e 's#{CFG_NUT_CPU_CORES_LD}#$(CFG_NUT_CPU_CORES_LD)#g' \
	-e 's#{CFG_NUT_CPU_MODE1_CORES}#$(CFG_NUT_CPU_MODE1_CORES)#g' \
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
	$(CONFIG_TEMPLATE) > $(CONFIG)


##### Clean #####
clean:
	@$(MAKE) -f $(SCRIPTS_DIR)/ghdl.mk ghdl-$@
	@$(MAKE) -f $(SCRIPTS_DIR)/isim.mk isim-$@
	@$(MAKE) -f $(SCRIPTS_DIR)/xst.mk xst-$@
	
