# This file is part of the ParaNut project.
# 
#  Copyright (C) 2020 Alexander Bahle <alexander.bahle@hs-augsburg.de>
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

CFLAGS = -O2 -I$(SYSTEMC_HOME)/include -Ilibparanutsim/include -DSIMBUILD
LDFLAGS = -L$(SYSTEMC_HOME)/lib-linux64 -lsystemc -Llibparanutsim/lib -lparanutsim

ELF = dm_tb 
SOURCES = $(wildcard *.cpp)
OBJECTS = $(patsubst %.cpp,%.o,$(SOURCES)) 
HEADERS = $(wildcard *.h)


.PHONY: all
all: $(ELF) 

.PHONY: run
run: $(ELF)
	./$(ELF)

$(ELF): libparanutsim/lib/libparanutsim.a $(OBJECTS) $(HEADERS)
	@echo "### Linking";
	$(CXX) -o $@ $^ $(CFLAGS) $(LDFLAGS)

%.o: %.cpp libparanutsim/lib/libparanutsim.a
	$(CXX) -c $(CFLAGS)  $<
	
# libparanutsim target:
.PHONY: libparanutsim libparanutsim/lib/libparanutsim.a
libparanutsim:
libparanutsim/lib/libparanutsim.a:
	@echo "### Update libparanutsim";
	$(MAKE) -C ../.. INSTALL_DIR=$(CURDIR)/libparanutsim install

.PHONY: clean
clean: 
	rm -rf *.o *.o.s *.c.s $(ELF) libparanutsim

