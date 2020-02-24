# Synthesis & implementation (xst)
IOBUF?="yes"
XILINX_DEVICE_PART?=xc3s700a-fg484-4

$(ENTITY_NAME).bit: $(ENTITY_NAME).ngc
	eis-implement $(ENTITY_NAME).ngc $(ENTITY_NAME).ucf

$(ENTITY_NAME).ngc: $(XST_MODULE_SRC)
	@source ${EIS_HOME}/etc/xilinx-settings.sh; \
	TOP=$(ENTITY_NAME) \
	# Create .prj file... \
	echo "### Writing '$${TOP}.prj'..."; \
	echo "" > $${TOP}.prj; \
	for F in $^; do \
		if [[ $${F##*.} == "v" ]]; then \
			echo "verilog $(WORKLIB) $${F}" >> $${TOP}.prj; \
		else \
			echo "vhdl $(WORKLIB) $${F}" >> $${TOP}.prj; \
		fi; \
	done; \
	# Create .ifn file... \
	IFN=$${TOP}.ifn; \
	if [[ "" &&  -f "$${IFN}" ]]; then \
		echo "### File '$${IFN}' already exists - not touching..."; \
	else \
		echo "### Writing '$${IFN}'..."; \
		echo "set -xsthdpdir xst_work" > $${IFN}; \
		echo "run" >> $${IFN}; \
		echo "-generics {$(GENERICS)}" >> $${IFN}; \
		echo "-ifn $${TOP}.prj" >> $${IFN}; \
		echo "-ofn $${TOP}" >> $${IFN}; \
		echo "-work_lib $(WORKLIB)" >> $${IFN}; \
		echo "-top $${TOP}" >> $${IFN}; \
		echo "-iobuf $(IOBUF)" >> $${IFN}; \
		echo "-p $(XILINX_DEVICE_PART)" >> $${IFN}; \
		echo "-opt_mode Speed" >> $${IFN}; \
		echo "-opt_level 1" >> $${IFN}; \
		echo "-rtlview yes" >> $${IFN}; \
		#echo "-uc $(ENTITY_NAME).xcf" >> $${IFN}; \
		#echo "-intstyle xflow" >> $${IFN}; \
		#echo "-use_dsp48 no" >> $${IFN}; \
	fi; \
	# Run XST... \
	echo "### Running XST..."; \
	LOG=$${TOP}.log; \
	CMD="xst -ifn $${IFN} -ofn $${LOG}"; \
	echo "> $${CMD}"; \
	$${CMD}; \
	# Summarize errors and warings... \
	echo; \
	echo "### Summary of errors and warnings (see '$${LOG}' for details):"; \
	for M in ERROR WARNING INFO; do \
		echo; \
		grep $${M} $${LOG}; \
	done

xst-ngc: $(ENTITY_NAME).ngc

xst-bit: $(ENTITY_NAME).bit

xst-clean:
	rm -rf $(ENTITY_NAME)_tb *.ghw *.o *.prj *.log $(ENTITY_NAME).ngc *.ngr \
	*.bit *.ifn *.ipf xlnx_auto_* *.xrpt _xmsgs *.xmsgs *.cf *.lso xst_work implement \
