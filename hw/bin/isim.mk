# Simulation (isim)
-include fdeps.mk

%.o: %.vhd
	@echo "vhdl $(WORKLIB) $<" >> $(ENTITY_NAME)_tb.prj

prj_file: 
	echo "" > $(ENTITY_NAME)_tb.prj
	#echo "vhdl work $(ENTITY_NAME)_tb.vhd" >> $(ENTITY_NAME)_tb.prj

$(ENTITY_NAME)_tb.exe: prj_file $(SIM_MODULE_OBJ)
	@source ${EIS_HOME}/etc/xilinx-settings.sh; \
	fuse -incremental -v 0 -prj $(ENTITY_NAME)_tb.prj -o $(ENTITY_NAME)_tb.exe $(WORKLIB).$(ENTITY_NAME)_tb

isim-cl: $(ENTITY_NAME)_tb.exe
	@source ${EIS_HOME}/etc/xilinx-settings.sh; \
	./$(ENTITY_NAME)_tb.exe

isim-gui: $(ENTITY_NAME)_tb.exe
	@source ${EIS_HOME}/etc/xilinx-settings.sh; \
	./$(ENTITY_NAME)_tb.exe -gui -view $(ENTITY_NAME)_tb.wcfg &

isim-clean:
	rm -rf $(ENTITY_NAME)_tb.exe fuseRelaunch.cmd isim *.wdb \
	*.log *.prj fuse.xmsgs
