# This file is part of the ParaNut project.
# 
#  Copyright (C) 2010-2020 Alexander Bahle <alexander.bahle@hs-augsburg.de>
#                          Gundolf Kiefer <gundolf.kiefer@hs-augsburg.de>
#     Hochschule Augsburg, University of Applied Sciences
# 
# Description:
#   Prepare and run ghdl tesbenches
# 
# Redistribution and use in source and binary forms, with or without modification, 
# are permitted provided that the following conditions are met:
# 
# 1. Redistributions of source code must retain the above copyright notice, this 
#    list of conditions and the following disclaimer.
# 
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation and/or
#    other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND 
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE 
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
# ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES 
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; 
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON 
# ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS 
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

# Build the GHDL Simulation (located in the PN_HW_TB_DIR directory)...

PN_SYSC_DIR = ../../sysc
PN_HW_TB_DIR = ./paranut
PN_HW_TOOLS_DIR = ../../tools

.PHONY: all
all: help

.PHONY: build
build:  
	@echo "### Building GHDL Simulation ###"
	@if [ -f $(PN_HW_TB_DIR)/Makefile ]; then \
	  $(MAKE) -C $(PN_SYSC_DIR) copy; \
	  $(MAKE) -C $(PN_HW_TOOLS_DIR) -f Makefile.inc FORCE_PKG_UPDATE; \
	  $(MAKE) -j1 -C $(PN_HW_TB_DIR); \
	else \
		echo "INFO: The sim-ghdl target is meant for building the GHDL testbench during development inside the main repository!"; \
		echo "      Set the PN_HW_TB_DIR variable accordingly if you know what you are doing"; \
	fi

.PHONY: help
help:  
	@echo "INFO: The sim-ghdl target is meant for building the GHDL testbench during development inside the main repository!"
	@echo "      By default the testbench "paranut" is set. If you want to execute another testbench, set the PN_HW_TB_DIR variable accordingly. E.g.: make build PN_HW_TB_DIR=arbiter"


.PHONY: clean
clean:
	rm -f ../paranut/M*
