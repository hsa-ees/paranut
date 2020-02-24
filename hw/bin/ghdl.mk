# Simulation (ghdl & gtkwave)
-include fdeps.mk

GHDL=ees-ghdl
GHDLFLAGS=--ieee=synopsys -fexplicit -O2 

WORKLIB?=work
UNISIM=unisim
XCL=xilinxcorelib

XILINX_VHDL_SRC_PATH=/opt/Xilinx/14.5/ISE_DS/ISE/vhdl/src

folders:
	@mkdir -p ./$(WORKLIB)
	@mkdir -p ./$(UNISIM)
	@mkdir -p ./$(XCL)

# Generic rule to analyze files
%.o: %.vhd | folders
ifeq ($(NEED_UNISIM),1)
	$(GHDL) -i --workdir=$(WORKLIB) --work=$(WORKLIB) $<
else
	$(GHDL) -a $(GHDLFLAGS) --workdir=$(WORKLIB) --work=$(WORKLIB) $<
endif

# Elaboration targets (GHDL)
$(ENTITY_NAME)_tb: $(SIM_MODULE_OBJ) | folders
ifeq ($(NEED_UNISIM),1)
	$(GHDL) -i --workdir=$(UNISIM) --work=$(UNISIM) $(XILINX_VHDL_SRC_PATH)/unisims/*.vhd
	$(GHDL) -i --workdir=$(UNISIM) --work=$(UNISIM) $(XILINX_VHDL_SRC_PATH)/unisims/primitive/*.vhd
	$(GHDL) -i --workdir=$(XCL) --work=$(XCL) $(XILINX_VHDL_SRC_PATH)/XilinxCoreLib/*.vhd
	$(GHDL) -m --workdir=$(WORKLIB) --work=$(WORKLIB) -g -P./$(UNISIM) -P./$(XCL) --syn-binding $(GHDLFLAGS) $(ENTITY_NAME)_tb
else
	$(GHDL) -e $(GHDLFLAGS) --workdir=$(WORKLIB) --work=$(WORKLIB) $@
endif
	@#$(GHDL) -r $(ENTITY_NAME)_tb --disp-tree=inst --wave=$(ENTITY_NAME)_tb.ghw
	@#$(GHDL) -r $(ENTITY_NAME)_tb --wave=$(ENTITY_NAME)_tb.ghw

$(ENTITY_NAME)_tb.ghw: $(ENTITY_NAME)_tb
	./$< --wave=$@

# Simulation targets (GHDL)
ghdl-cl: $(ENTITY_NAME)_tb
	./$< --wave=$(ENTITY_NAME)_tb.ghw --ieee-asserts=disable
	@#./$<

ghdl-gui: $(ENTITY_NAME)_tb.ghw
	gtkwave -A $< &

ghdl-clean:
	$(GHDL) --remove --work=$(WORKLIB) --workdir=$(WORKLIB)
	$(GHDL) --remove --work=$(UNISIM) --workdir=$(UNISIM)
	$(GHDL) --remove --work=$(XCL) --workdir=$(XCL)
	rm -rf $(ENTITY_NAME)_tb *.ghw *.o $(WORKLIB) $(UNISIM) $(XCL)




