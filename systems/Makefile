#  -----------------------------------------------------------------------------
#
#  This file is part of the ParaNut project.
#
#  Copyright (C) 2021 Michael Schaeferling <michael.schaeferling@hs-augsburg.de>
#      Hochschule Augsburg, University of Applied Sciences
#
#  Description:
#    ParaNut Systems Makefile.
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


# Configuration ...
PN_SYSTEMS ?= refdesign \
              soundchip


################################################################################
#                                                                              #
#     Default and help targets                                                 #
#                                                                              #
################################################################################


# Default target ...
.PHONY: all
all: help


# Print usage info ...
.PHONY: help
help:
	@echo "Makefile for the ParaNut systems directory."
	@echo "Here, we just invoke the systems Make targets to delegate work there."
	@echo
	@echo "Usage: make [<target>] [<parameter>=<value> ...]"
	@echo
	@echo "Targets:"
	@echo
	@echo "  build       : Install hardware source files to the directory given by parameter PREFIX"
	@echo
	@echo "  install     : Install system files to the directory given by parameter PREFIX"
	@echo "                (only applicable by some systems)"
	@echo
	@echo "  clean       : Cleanup systems"
	@echo "  veryclean   : Cleanup systems"
	@echo


################################################################################
#                                                                              #
#     Build                                                                    #
#                                                                              #
################################################################################

.PHONY: build
build:
	@for p in $(PN_SYSTEMS); do \
	  if [ -d "$$p" ]; then \
	    echo ""; \
	    echo "###########################################"; \
	    echo "#                                         #"; \
	    echo "#         Building ParaNut System         #"; \
	    echo "#                                         #"; \
	    echo "###########################################"; \
	    echo "System Name: '$$p'"; \
	    echo ""; \
	    $(MAKE) -C $$p build || exit 1; \
	    echo; \
	  fi; \
	done


################################################################################
#                                                                              #
#     Install                                                                  #
#                                                                              #
################################################################################

.PHONY: install
install:
	@test "$(PREFIX)" != "" || ( echo "ERROR: Make variable 'PREFIX' must be set for the 'install' target."; exit 3; )
	@echo
	@echo "### Installing system files to $(PREFIX) ###"
	for p in $(PN_SYSTEMS); do \
	  if [ -d "$$p" ]; then \
	    $(MAKE) -C $$p install PREFIX=$(PREFIX) || exit 1; \
	    echo; \
	  fi; \
	done


################################################################################
#                                                                              #
#     Clean / Veryclean                                                        #
#                                                                              #
################################################################################

.PHONY: clean
clean:
	@echo
	@echo "### Cleaning systems ###"
	@for p in $(PN_SYSTEMS); do \
	  if [ -d "$$p" ]; then \
	    $(MAKE) -C $$p clean || exit 1; \
	    echo; \
	  fi; \
	done

.PHONY: veryclean
veryclean:
	@echo
	@echo "### Cleaning systems (very clean) ###"
	@for p in $(PN_SYSTEMS); do \
	  if [ -d "$$p" ]; then \
	    $(MAKE) -C $$p veryclean || exit 1; \
	    echo; \
	  fi; \
	done
