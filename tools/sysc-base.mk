#  -----------------------------------------------------------------------------
#
#  This file is part of the ParaNut project.
#
#  Copyright (C) 2023 Felix Wagner <felix.wagner1@hs-augsburg.de>
#      Hochschule Augsburg, University of Applied Sciences
#
#  Description:
#    This is a base makefile, that can be included in hardware module makefiles 
#    in oder to clean up their contents. It contains targets for creating 
#    systemc simulation binaries and synthesizing the respective modules1
#    See Makefile in the folder containing this file for an example usage. 
#
#  --------------------- LICENSE -----------------------------------------------
#  Redistribution and use in source and binary forms, with or without modification,
#  are permitted provided that the following conditions are met:
#
#  1. Redistributions of source code must retain the above copyright notice, this
#     list of conditions and the following disclaimer.
#
#  2. Redistributions in binary form must reproduce the above copyright notice,
#     this list of conditions and the following disclaimer in the documentation and/or
#     other materials provided with the distribution.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
#  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
#  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
#  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
#  ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
#  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
#  ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
#  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
#  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#  -----------------------------------------------------------------------------

# include makefile containing ParaNut directory structure information
include $(PARANUT_HOME)/directory-base.mk

# include local configuration
# variables defined in the included config.mk overwrite those
# found in the global configuration
-include $(PNS_CONFIG_MK)

# include global configuration
include $(CONFIG_MK)

################################################################################
#################                Configuration             #####################
################################################################################

################# Common Library ###############################################
CONFIG_H_TEMPLATE = $(SYSC_COMMON_DIR)/paranut-config.template.h

################# Module configuration #########################################
# MODULE_SRC:               Sources that are used for Simulation as well as 
#                           synthesis. (including the testbench/sc_main file)
MODULE_SRC ?=

# TESTBENCH_SRC:            Source that contains tb (and thus sc_main) 
TESTBENCH_SRC ?=

# SIM_SRC:                  Sources that are only used in simulationon
SIM_SRC ?= 

# MODULE_INTERDEPENDENCIES: Other modules this one depends on
#                           common is usually omnipresent
MODULE_INTERDEPENDENCIES ?=

# MODULE_INCLUDES:          Include directives for the compiler (e.g. -I..)
MODULE_INCLUDES ?= 

# MODULE_CFLAGS:            Additional compiler flags this module needs
MODULE_CFLAGS ?=

# MODULE_LDFLAGS:           Additional linker flags this module needs
MODULE_LDFLAGS ?=  

# SYNTHESIS_TOPMODULE:      Defines the Top Module for synthesis as defined inside 
#							the test bench cpp
SYNTHESIS_TOP_MODULE ?=


################# Target definitions ###########################################
SIM_TARGET ?= $(MODULE_NAME)
SCTOOL_TARGET ?= $(MODULE_NAME)_sctool
SV_TARGET ?= $(PNS_SYSC_SV_DIR)/$(MODULE_NAME).sv
V_TARGET ?= $(PNS_SYSC_V_DIR)/$(MODULE_NAME).v
SYSC_MODULE_LIB ?= $(PNS_SYSC_LIB_DIR)/lib$(MODULE_NAME).a
SYSC_MODULE_HEADERS ?= $(wildcard *.h)
SYSC_MODULE_INC := $(addprefix $(PNS_SYSC_INC_DIR)/, $(SYSC_MODULE_HEADERS))

################# Module Interdependency #######################################
# these variables ensure including and linking external modules works
# ALL MODULE FOLDERS MUST BE NAMED: module_$(MODULE_NAME)
# otherwise this will fail horribly
MODULE_INTERDEPENDENCY_DIRS = $(addprefix $(SYSC_DIR)/module_, $(MODULE_INTERDEPENDENCIES))
MODULE_INTERDEPENDENCY_INCLUDE = $(addprefix -I, $(MODULE_INTERDEPENDENCY_DIRS))
MODULE_INTERDEPENDENCY_LIBS = $(addprefix $(PNS_SYSC_LIB_DIR)/lib, $(addsuffix .a, $(MODULE_INTERDEPENDENCIES)))
MODULE_DEPENDENCY_FILE = $(PNS_SYSC_BUILD_DIR)/$(MODULE_NAME)_Makefile.deps

################# SystemC dependencies #########################################
# Overwrite the options in this section from SystemC config file
include $(SYSTEMC_CONFIG_MK)

SYSTEMC_HOME ?=
# SYSTEMC_LIB: location of libsystem.so
SYSTEMC_LIB ?=
# SYSTEMC_INCLUDE: location of  systemc.h            
SYSTEMC_INCLUDE ?=
# SYSTEMC_CPP_VER: CPP Standard Version must match that of the SystemC library 
SYSTEMC_CPP_VER ?=

################# General Compiler Configuration ###############################
CXX = g++
AR = ar
DEBUG_FLAGS ?= 
CFLAGS = -O2 -MMD $(DEBUG_FLAGS) -I$(PNS_SYSC_INC_DIR) -I$(SYSTEMC_INCLUDE) -pthread -std=$(SYSTEMC_CPP_VER) $(MODULE_INCLUDES) $(MODULE_CFLAGS)
LDFLAGS = -L$(SYSTEMC_LIB) -lsystemc $(MODULE_LDFLAGS)


################# Simulation Setup  ############################################
# SIMBUILD will supress the __SYNTHESIS__ is set info
SIM_FLAGS ?= -DSIMBUILD 
SYSC_MODULE_OBJ = $(addprefix $(PNS_SYSC_BUILD_DIR)/,$(MODULE_SRC:.cpp=.o))
SYSC_TESTBENCH_OBJ = $(addprefix $(PNS_SYSC_BUILD_DIR)/,$(TESTBENCH_SRC:.cpp=.o))
SIM_OBJ = $(addprefix $(PNS_SYSC_BUILD_DIR)/,$(SIM_SRC:.cpp=.o) $(TESTBENCH_SRC:.cpp=.o))


################# Synthesis Setup ##############################################
# SYNTH_LIB: The path derived from SYSTEMC_LIB will be correct 
#    if SYSTEMC_HOME is set to the root of an icsc installation
SYNTH_DIR ?= $(SYSC_MODULE_DIR)/sc_build
SYNTH_LIB ?= $(SYSTEMC_LIB)/libSCTool.so 
SYNTH_SCTOOL_TEMPLATE ?= $(SYSC_DIR)/sctool.template.cpp
SYNTH_SCTOOL_SRC ?= $(SYSC_MODULE_DIR)/sctool.cpp
SYNTH_SCTOOL_OBJ ?= $(SYNTH_SCTOOL_SRC:.cpp=.o)
SYNTH_SCTOOL_INC := $(addprefix $(SYNTH_DIR)/, $(SYSC_MODULE_HEADERS))
SYNTH_CONFIG_H = $(SYNTH_DIR)/$(notdir $(CONFIG_H))

SYNTH_FLAGS ?= -D__SC_TOOL__ 

################################################################################
#################                Make Targets              #####################
################################################################################

################################################################################
######             General Targets       #######################################
################################################################################

#################  Default Target = help ####################################### 
.PHONY: all
all: help

#################  Version ##################################################### 
$(VERSION_ENV):
	@$(MAKE) -C $(PARANUT_HOME) update-version

#################  Config File #################################################
# target is to be overwritten in module_common
# DISABLE_ORIGINAL_CONFIG_TARGET is used to silence target already exists warning
ifndef DISABLE_ORIGINAL_CONFIG_TARGET
$(CONFIG_H): $(CONFIG_MK) $(PNS_CONFIG_MK)
	@$(MAKE) -C $(SYSC_COMMON_DIR) build-config
endif
################################################################################
######             Simulation Targets    #######################################
################################################################################

# general compilation setup
# dependency on $(MODULE_INTERDEPENDENCY_LIBS) mainly ensures the presence 
# of the module's .h files in $(PNS_SYSC_INC_DIR)
$(PNS_SYSC_BUILD_DIR)/%.o: %.cpp $(CONFIG_H) $(SYSC_MODULE_INC) $(MODULE_INTERDEPENDENCY_LIBS)
	$(CXX) -c $(CFLAGS) $(SIM_FLAGS) $< -o $@


# phony target for human invocation
.PHONY: build-sim
build-sim: $(SIM_TARGET) 

# phony target for human invocation
.PHONY: build-tb
build-tb: build-sim

$(SIM_TARGET): $(SYSC_MODULE_LIB) 
	$(CXX) $(CFLAGS) $(LDFLAGS) -o $@ $(SYSC_MODULE_LIB) -L$(PNS_SYSC_LIB_DIR) $(addprefix -l,$(MODULE_INTERDEPENDENCIES))
#         Note on linking done here. Linking the pn_common lib must be listed after the 
#         target files themselves, as it is otherwise not searched in the nessecary order 

# install all .h files present in the module root directory in the include folder  $(PNS_SYSC_INC_DIR)
# additionally creates a folder to hold built object file
$(SYSC_MODULE_INC): $(SYSC_MODULE_HEADERS)
	@mkdir --parents $(PNS_SYSC_BUILD_DIR)
	@(install -Dp -m 644 $(SYSC_MODULE_DIR)/$(@F) $@)

# used to create missing library files from external makefile invocations
.PHONY: build-lib
build-lib: $(SYSC_MODULE_LIB) $(SYSC_MODULE_INC) 
	@echo "   --> built library for Module $(MODULE_NAME)"

# create module library and place it in $(PNS_SYSC_LIB_DIR)
# this should additionally be dependent on $(SYSTEMC_LIB)/libsystemc.so,
# but that complicates the install process
$(SYSC_MODULE_LIB): $(SYSC_MODULE_OBJ) $(SIM_OBJ) $(MODULE_DEPENDENCY_FILE)
	@mkdir --parents $(PNS_SYSC_LIB_DIR)
	$(AR)  cr $@ $(SYSC_MODULE_OBJ) $(SIM_OBJ)

# helps to create the library file of external modules
# automatically calls make in the folder named like the missing module
# this seems to block all parallelism in the makefile.
# annoying but the only solution i could find
# technically you could leave this out, some things would then be created 
# multiple times, but it would work and be a bit faster, however... ugly.
.NOTPARALLEL: $(MODULE_INTERDEPENDENCY_LIBS)
# .SECONDEXPANSION is needed to enable shell execution in target dependency
.SECONDEXPANSION:
$(PNS_SYSC_LIB_DIR)/lib%.a: $$(shell cat $$(PNS_SYSC_BUILD_DIR)/$$*_Makefile.deps 2> /dev/null) $(CONFIG_MK) $(PNS_CONFIG_MK)
	+$(MAKE) -C $(SYSC_DIR)/module_$* build-lib

#simple target to run the created executable without knowing it's name
.PHONY: run-sim
run-sim: $(SIM_TARGET)
	./$(SIM_TARGET)

.PHONY: run-tb
run-tb: run-sim

################################################################################
######             Synthesis Targets    ########################################
################################################################################

# generate lists of relevant module files
# namely the cpp files mentioned in $(MODULE_SRC)
# and all .h files present in the root of the module dir
SYNTH_MODULE_SRC_CPP := $(addprefix $(SYNTH_DIR)/, $(notdir $(MODULE_SRC)))
SYNTH_MODULE_TB_CPP := $(addprefix $(SYNTH_DIR)/, $(notdir $(TESTBENCH_SRC)))

# the module may depend on sources from another module.
# this target copies the files mentioned in the external module's MODULE_SRC 
# variable to $(SYNTH_DIR) 
.PHONY: copy-syn-files
copy-syn-files: $(SYNTH_MODULE_SRC_CPP) $(SYNTH_SCTOOL_INC) 
	@for dir in $(MODULE_INTERDEPENDENCY_DIRS); do \
		$(MAKE) -C $${dir} copy-syn-files SYNTH_DIR=$(SYNTH_DIR);\
	done

# this is called in the makefile that was used by the user on console
# it additionally copies the tb sources
.PHONY: copy-syn-files-master
copy-syn-files-master: copy-syn-files $(SYNTH_MODULE_TB_CPP)

# install all .h files present in the module root directory in the include folder  $(PNS_SYSC_INC_DIR)
# additionally creates a folder to hold built object file
$(SYNTH_SCTOOL_INC): $(SYSC_MODULE_HEADERS)
	@mkdir --parents $(SYNTH_DIR)
	@[ -L $(SYNTH_DIR)/$(@F) ] || ln -s $(abspath $(@F)) $(SYNTH_DIR)/$(@F);

# target for creating links to the sources in the dedicated 
# synthesis directory
$(SYNTH_DIR)/%: 
	@[ -d $(SYNTH_DIR) ] || mkdir --parents $(SYNTH_DIR); \
	[ -L $(SYNTH_DIR)/$(@F) ] || ln -s $(abspath $(filter %$*,$(MODULE_SRC) $(TESTBENCH_SRC))) $(SYNTH_DIR)/$*;

# SC TOOL CPP must be cleaned away before rebuilding it. Otherwise it will 
# include itself, creating circular dependencies
.PHONY: clean-sc-tool
clean-sc-tool:
	@rm -f $(SYNTH_SCTOOL_SRC)

# create sctool.cpp from template
# creating the include list is done by searching the $SYNTH_DIR for all cpp files 
# this is done up to a depth of 3 subdirectories. Should anything go deeper
# edit the wildcard search for SC_TOOL_SRC_INCLUDE
$(SYNTH_SCTOOL_SRC): clean-sc-tool copy-syn-files-master $(SYNTH_SCTOOL_TEMPLATE)
	@echo "### Updating $(SYNTH_SCTOOL_SRC)..."
	@sed \
	-e 's#{SC_TOOL_MAIN_CPP}#$(SYNTH_SCTOOL_SRC)#g' \
	-e 's#{SC_TOOL_SV_OUT}#$(SV_TARGET)#g' \
	-e 's#{SC_TOOL_TOP_MODULE}#tb.$(SYNTHESIS_TOP_MODULE)#g' \
	-e 's#{SC_TOOL_ICSC_HOME}#$(SYSTEMC_HOME)#g' \
	-e 's#{SC_TOOL_ICSC_INCLUDE}#$(SYSTEMC_INCLUDE)#g' \
	-e 's#{SC_TOOL_AUTO_INCLUDE}#$(SYNTH_DIR)#g' \
	-e 's#{SC_TOOL_WORKDIR}#$(SYNTH_DIR)#g' \
	-e 's#{SC_TOOL_SRC_INCLUDE}#$(foreach cpp,$(wildcard $(SYNTH_DIR)/*.cpp $(SYNTH_DIR)/*/*.cpp $(SYNTH_DIR)/*/*/*.cpp),\#include "$(cpp)"\n)#g' \
	-e 's#{SC_TOOL_HEADER_INCLUDE}#$(foreach include,$(MODULE_INCLUDES),"$(include)")#g' \
	$(SYNTH_SCTOOL_TEMPLATE) > $(SYNTH_SCTOOL_SRC)

# compile synthesis sources
$(SYNTH_SCTOOL_OBJ): $(SYNTH_SCTOOL_SRC) 
	$(CXX) -c $(CFLAGS) $(SYNTH_FLAGS) -o $(SYNTH_SCTOOL_OBJ) -c $(SYNTH_SCTOOL_SRC)

# build sctool for synthesis
$(SCTOOL_TARGET):  $(SYNTH_SCTOOL_OBJ)
	$(CXX) $(CFLAGS) $(LDFLAGS) -o $@ $(SYNTH_SCTOOL_OBJ) $(SYNTH_LIB)

# run synthesis
.PHONY: build-syn 
build-syn: $(SV_TARGET)

$(SYNTH_CONFIG_H): $(CONFIG_H)
	@mkdir --parents $(SYNTH_DIR); \
	[ -L $@ ] || ln -s $< $@;

# build system verilog
# must first build libcommon to provide stuff in the sim/include folder
$(SV_TARGET): $(SYNTH_CONFIG_H) $(PNS_SYSC_LIB_DIR)/libcommon.a $(SCTOOL_TARGET) $(PNS_SYSC_SV_DIR)
	./$(SCTOOL_TARGET)

# ensure presence of $(SYNTH_DIR)
$(SYNTH_DIR):
	mkdir --parents $(SYNTH_DIR)

# ensure presence of $(PNS_SYSC_SV_DIR)
$(PNS_SYSC_SV_DIR): 
	mkdir --parents $(PNS_SYSC_SV_DIR) 

# phony target for human consumption
.PHONY: build-verilog
build-verilog: $(V_TARGET)

# convert system verilog to verilog
$(V_TARGET): $(SV_TARGET) $(PNS_SYSC_V_DIR)
	sv2v $(SV_TARGET) > $(V_TARGET);\

# ensure presence of $(PNS_SYSC_V_DIR)
$(PNS_SYSC_V_DIR):
	mkdir --parents $(PNS_SYSC_V_DIR)

################################################################################
######             Helpers              ########################################
################################################################################

.PHONY: sysc-lib-info
sysc-lib-info:
	@echo "Path to SystemC-Library: $(SYSTEMC_LIB)"
	@if [ -f $(SYSTEMC_LIB)/libsystemc.so ];then \
	readelf -p .comment /data/icsc_home/lib/libsystemc.so; \
	else \
	echo ""; \
	echo "can not find library at specified path"; \
	echo "You must either set ICSC_HOME if you want to use icsc libraries"; \
	echo "or SYSTEMC_LIB and SYSTEMC_INCLUDE if you want to specify another "; \
	echo "systemc library source."; \
	fi

################# Make Dependency file for external checks #####################

$(MODULE_DEPENDENCY_FILE): $(MODULE_SRC) $(TESTBENCH_SRC) $(SYSC_MODULE_HEADERS)
	@echo $(addprefix $(SYSC_MODULE_DIR)/, $(MODULE_SRC) $(TESTBENCH_SRC) $(SYSC_MODULE_HEADERS)) > $(MODULE_DEPENDENCY_FILE)


################################################################################
######             Clean Targets    ############################################
################################################################################

# in this section there are the "normal" clean, and clean-all that only 
# depend on other targets but do nothing themselves.
# the actual work is done in the -external targets
# otherwise calling make $(external_module) clean as done in clean-interdependencies
# and clean-all-interdependencies would cause unnessecary recursive invocations

# normal clean, deletes build artefacts as well as library and include files
# basically everything to do with simulation
.PHONY: clean
clean: clean-external clean-interdependencies

#to be overwritten in the module make
.PHONY: clean-custom
clean-custom:

# see section beginning for an explaination
.PHONY: clean-external
clean-external: clean-custom clean-lib
	@rm -f *.[aod] $(SIM_OBJ) $(SIM_TARGET) $(SCTOOL_TARGET)  \
					$(SYSC_MODULE_OBJ) $(SYSC_MODULE_OBJ:%.o=%.d)
	@rm -rf $(SYNTH_DIR)
	

.PHONY: clean-lib
clean-lib:
	@rm -rf $(SYSC_MODULE_INC) $(MODULE_DEPENDENCY_FILE)
	@rm -rf $(SYSC_MODULE_LIB)

# clean all additionally deletes artifacts from synthesis
.PHONY: clean-all
clean-all: clean-all-external clean-all-interdependencies

# see section beginning for an explaination
.PHONY: clean-all-external
clean-all-external: clean-external 
	@rm -rf $(PNS_SYSC_SV_DIR)
	@rm -rf $(PNS_SYSC_V_DIR)
	@rm -rf $(PNS_SYSC_LIB_DIR)

# see section beginning for an explaination
.PHONY: clean-interdependencies
clean-interdependencies:
	@for dir in $(MODULE_INTERDEPENDENCY_DIRS); do \
		$(MAKE) -C $${dir} clean-external ;\
	done

# see section beginning for an explaination
.PHONY: clean-all-interdependencies
clean-all-interdependencies:
	@for dir in $(MODULE_INTERDEPENDENCY_DIRS); do \
		$(MAKE) -C $${dir} clean-all-external ;\
	done

#### Automatic dependencies ####################################################
# watch header files for changes by including dependency files 
# created with -MMD at compilation 
-include $(SIM_OBJ:%.o=%.d)



################################################################################
######             HEEEEELP!!!      ############################################
################################################################################
.PHONY: help
help:
	@echo "Makefile for the Memory Management Module"
	@echo "This makefile allows the user to build the libuart needed for the test benches"
	@echo "and the hls with icsc"
	@echo
	@echo "Usage: make [<target>] [<parameter>=<value> ...]"
	@echo
	@echo "Targets:"
	@echo
	@echo "  build-tb       : creates an executable testbench named $(SIM_TARGET)"
	@echo
	@echo "  build-lib      : builds the module library used to include it in higher modules"
	@echo "                  Parameters: "
	@echo "                    DEBUG_FLAGS='-g -pg'  "
	@echo "                    DEBUG_FLAGS='-g -pg'  "
	@echo 
	@echo "  build-syn      : runs the highlevel snthesis with ICSC"
	@echo "                  ICSC_HOME environment variable needs to be set"
	@echo 
	@echo "  run-tb         : executes the built simulation executable"
	@echo
	@echo "  sv2v           : converts the system verilog file created by ICSC to verilog"
	@echo "                  sc2v needs to be installed and be in the systems PATH"
	@echo 
	@echo "  sysc-lib-info"
	@echo "  clean       : Cleanup files created by build-testbench and build-sim"
	@echo "  clean-all   : Same as clean, additionally clean build-syn artefacts"
	@echo " "
