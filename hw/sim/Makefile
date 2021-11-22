#  -----------------------------------------------------------------------------
#
#  This file is part of the ParaNut project.
#
#  Copyright (C) 2020-2021 Alexander Bahle <alexander.bahle@hs-augsburg.de>
#                          Gundolf Kiefer <gundolf.kiefer@hs-augsburg.de>
#                          Michael Schaeferling <michael.schaeferling@hs-augsburg.de>
#      Hochschule Augsburg, University of Applied Sciences
#
#  Description:
#    This Makefile builds a working simulator 'pn-sim' for the local ParaNut
#    configuration.
#    At the same time, this Makefile can be used as a template for an own, customized
#    system simulator. Check the places marked with "EDIT HERE"!
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


# Default target ...
#   EDIT HERE: Change as appropriate
.PHONY: all
all: build

# Source and target files ...
#   EDIT HERE:
#   1. Give your simulator your own name as PNSIM_BIN (e.g. 'mychip-sim').
#   2. Add own source files to PNSIM_SRC.
PNSIM_BIN = pn-sim
PNSIM_SRC = $(PNSIM_BIN).cpp  # example_wb_slave.cpp
PNSIM_OBJ = $(PNSIM_SRC:%.cpp=%.o)
PNSIM_INSTALL_DIR ?= $(PREFIX)/tools/bin

HW_SIM_FILES = Makefile ParaNut.mk pn-sim.cpp example_wb_slave.h example_wb_slave.cpp
HW_SIM_SRC_INSTALL_DIR ?= $(PREFIX)/src/hw/sim/


# Default compiler and linker settings ...
#   EDIT HERE: Change options as appropriate
CXX := g++
CFLAGS := -MMD -g -O2 -Wall -Wno-unknown-pragmas
  # -MMD : Create make dependency files
  # -g   : Generate debug symbols
  # -Wall: Enable all warnings
  # -Wno-unknown-pragmas: Do not warn about HLS pragmas
  # -Og  : Optimize for debugging
  # -O2  : Optimize for speed
LDFLAGS :=


# Flags for SystemC code ...
ifneq ($(SYSTEMC_HOME),)
  # SYSTEMC_HOME set: Add search paths for the header and library files ...
  CFLAGS += -I$(SYSTEMC_HOME)/include
  LDFLAGS += -L$(SYSTEMC_HOME)/lib-linux -L$(SYSTEMC_HOME)/lib-linux64
endif
LDFLAGS += -lsystemc

# ParaNut flags ...
#   EDIT HERE: Change and add a path relative from your simulator directory, e.g.:
#      include ../paranut/hw/sim/ParaNut.mk
include ParaNut.mk
	# This defines:
	# - compiler and linker flags as $(PARANUT_CFLAGS) and $(PARANUT_LDFLAGS)
	# - a helper target 'paranut-update' to build or update the ParaNut simulation library
	# - a helper target 'paranut-clean' to clean the ParaNut simulation library

.PHONY: build
build:
	$(MAKE) paranut-update
	$(MAKE) $(PNSIM_BIN)

# Linking ...
$(PNSIM_BIN): $(PARANUT_DEPEND) $(PNSIM_OBJ)
	@echo "### Linking ...";
	$(CXX) -o $@ $(PNSIM_OBJ) $(LDFLAGS) $(PARANUT_LDFLAGS)


# Compiling ...
%.o: %.cpp
	$(CXX) -c $(CFLAGS) $(PARANUT_CFLAGS) $<


# Installing ...
.PHONY: install
install: $(PNSIM_BIN)
	@test "$(PREFIX)" != "" || ( echo "ERROR: Make variable PREFIX must be set for the 'install' target."; exit 3; )
	@echo
	@echo "### Installing simulation source files to '$(HW_SIM_SRC_INSTALL_DIR)' ###"; \
	for FILE in $(HW_SIM_FILES); do \
	  install -Dp -m 644 -t $(HW_SIM_SRC_INSTALL_DIR) $$FILE; \
	done; \
	echo "### Installing '$(PNSIM_BIN)' to '$(PNSIM_INSTALL_DIR)' ###"; \
	install -Dp -m 755 -t $(PNSIM_INSTALL_DIR) $(PNSIM_BIN)



# Cleaning ...
.PHONY: clean veryclean
clean:
	rm -rf *.o *.o.s *.c.s *.d $(PNSIM_BIN)

veryclean: clean paranut-clean
