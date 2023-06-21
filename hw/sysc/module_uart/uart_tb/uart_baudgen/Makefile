# This file is part of the ParaNut project.
#
#  Copyright (C) 2020 Alexander Bahle <alexander.bahle@hs-augsburg.de>
#				 2023 Elias Schuler <elias.schuler@hs-augsburg.de>
#					  Lukas Bauer <lukas.bauer@hs-aubsburg.de>
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

CFLAGS = -O2 -I$(SYSTEMC_HOME)/include -I../../.. -I../../../uart -DSIMBUILD
LDFLAGS = -L$(SYSTEMC_HOME)/lib-linux64 -lsystemc -L../../../uart -luart

ELF = uart_baudgen_tb
SOURCES = $(wildcard *.cpp)
OBJECTS = $(patsubst %.cpp,%.o,$(SOURCES))
HEADERS = $(wildcard *.h)

.PHONY: help
help:
	@echo -e 'Usage: make [command] [<PARAMETER>=<VALUE>]'
	@echo -e 'Commands:'
	@echo -e '\t all \t build the testbench'
	@echo -e '\t run \t run the testbench'
	@echo -e '\t run-t2 \t run the testbench while creating a tracefile'
	@echo -e '\t clean \t delete all creted files'
	@echo -e '\t veryclean \t delete all creted files and the created lib'

.PHONY: all
all: $(ELF)

.PHONY: run
run: $(ELF)
	./$(ELF)

.PHONY: run-t2
run-t2: $(ELF)
	./$(ELF) -t2

$(ELF): ../../../uart/libuart.a $(OBJECTS) $(HEADERS)
	@echo "### Linking";
	$(CXX) -o $@ $^ $(CFLAGS) $(LDFLAGS)

%.o: %.cpp ../../../uart/libuart.a
	$(CXX) -c $(CFLAGS)  $<

# libparanutsim target:
.PHONY: libuart ../../../uart/libuart.a
libuart:
../../../uart/libuart.a:
	@echo "### Update libuart";
	$(MAKE) -C ../../../uart build_lib

.PHONY: clean
clean:
	rm -rf *.o $(ELF)

.PHONY: veryclean
veryclean: clean
	$(MAKE) -C ../../../uart clean
	rm -f *.vcd
