#  -----------------------------------------------------------------------------
#
#  This file is part of the ParaNut project.
#
#  Copyright (C) 2010-2020 Alexander Bahle <alexander.bahle@hs-augsburg.de>
#                          Michael Schaeferling <michael.schaeferling@hs-augsburg.de>
#                          Gundolf Kiefer <gundolf.kiefer@hs-augsburg.de>
#      Hochschule Augsburg, University of Applied Sciences
#
#  Description:
#    Makefile fragment for GHDL simulations.
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


# Simulation (ghdl & gtkwave)
-include fdeps.mk

GHDL=ghdl
GHDLFLAGS=--ieee=synopsys -fexplicit -O2
	# TBD: Eliminate '--ieee=synopsys', modify existing code to use 'numeric_std' instead

WORKLIB?=work


folders:
	@mkdir -p ./$(WORKLIB)

# Generic rule to analyze files
%.o: %.vhd | folders
	$(GHDL) -a $(GHDLFLAGS) --workdir=$(WORKLIB) --work=$(WORKLIB) $<

# Elaboration targets (GHDL)
$(ENTITY_NAME)_tb: $(SIM_MODULE_OBJ) | folders
	$(GHDL) -e $(GHDLFLAGS) --workdir=$(WORKLIB) --work=$(WORKLIB) $@
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
	rm -rf $(ENTITY_NAME)_tb *.ghw *.o $(WORKLIB)
