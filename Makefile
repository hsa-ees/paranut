# This file is part of the ParaNut project.
# 
#  Copyright (C) 2010-2020 Alexander Bahle <alexander.bahle@hs-augsburg.de>
#                          Gundolf Kiefer <gundolf.kiefer@hs-augsburg.de>
#     Hochschule Augsburg, University of Applied Sciences
# 
# Description:
#   Top Level Makefile
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

SYSC_FOLDER = sysc
HW_TB_FOLDER = hw/rtl/vhdl/tb/paranut
HW_BIN_FOLDER = hw/bin

IP_TARGET_DIR ?= paranut_ip

.PHONY: sim-sysc
sim-sysc: config
	@echo; echo "### Building SystemC Simulation..."; echo
	+$(MAKE) -C $(SYSC_FOLDER) paranut_tb

.PHONY: sim-ghdl
sim-ghdl:  config | $(HLS_TARGETS)
	@echo; echo "### Building GHDL Simulation..."; echo
	+$(MAKE) -C $(HW_BIN_FOLDER) -f Makefile.inc FORCE_PKG_UPDATE
	$(MAKE) -j1 -C $(HW_TB_FOLDER)

.PHONY: ip
ip: 
	@echo; echo "### Generating ParaNut IP core..."; echo
	+$(MAKE) -C $(SYSC_FOLDER) copy
	+$(MAKE) -C $(HW_BIN_FOLDER) -f Makefile.inc $@ IP_TARGET_DIR=$(IP_TARGET_DIR)


PHONY: clean-all
clean-all: 
	+$(MAKE) -C $(SYSC_FOLDER) clean
	+$(MAKE) -C $(HW_TB_FOLDER) clean
	rm -rf paranut_ip

