# This file is part of the ParaNut project.
#
#  Copyright (C) 2020 Alexander Bahle <alexander.bahle@hs-augsburg.de>
#                2022-2023 Lukas Bauer <lukas.bauer@hs-augsburg.de>
#     Hochschule Augsburg, University of Applied Sciences
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

CFLAGS = -O2 -I$(SYSTEMC_HOME)/include -Ilibparanutsim/include -USIMBUILD
LDFLAGS = -L$(SYSTEMC_HOME)/lib-linux64 -lsystemc -Llibparanutsim/lib -lparanutsim
DFLAGS = -D JTAG_TB_SIM
ELF = jtag_dtm_tb
SOURCES = $(wildcard *.cpp)
OBJECTS = $(patsubst %.cpp,%.o,$(SOURCES))
HEADERS = $(wildcard *.h)
FORCE_UPDATE ?= 1

.PHONY: all
all: build

# Print usage info ...
.PHONY: help
help:
	@echo
	@echo "Usage: make [<target>] [<parameter>=<value> ...]"
	@echo
	@echo "Targets:"
	@echo
	@echo "  build       : Build the test bench binary"
	@echo "  run         : Build and run the test bench binary"
	@echo "  clean       : Install ParaNut to the directory given by parameter INSTALL_DIR"


.PHONY: run
run: $(ELF)
	./$(ELF)

.PHONY: build
build: $(ELF)

$(ELF): libparanutsim/lib/libparanutsim.a $(OBJECTS) $(HEADERS)
	@echo "### Linking"
	$(CXX) -o $@ $^ $(CFLAGS) $(LDFLAGS)

%.o: %.cpp libparanutsim/lib/libparanutsim.a
	$(CXX) -c $(CFLAGS)  $<

# libparanutsim target:
.PHONY: libparanutsim libparanutsim/lib/libparanutsim.a
libparanutsim:
libparanutsim/lib/libparanutsim.a:
	@if [ -d libparanutsim ] || [ $(FORCE_UPDATE) ]; then \
	  echo "### Update libparanutsim"; \
	  # clean current lib, we need a special version with DFLAGS \
	  $(MAKE) -C ../../.. clean; \
	  $(MAKE) -C ../../.. INSTALL_DIR=$(CURDIR)/libparanutsim DFLAGS="$(DFLAGS)" install; \
	  # clean so the special version is not used anywhere else \
	  $(MAKE) -C ../../.. clean; \
	else \
	  echo "libparanut sim already built, to force update use FORCE_UPDATE=1";\
	fi
.PHONY: clean
clean:
	rm -rf *.o *.o.s *.c.s $(ELF) libparanutsim

