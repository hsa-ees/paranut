#  -----------------------------------------------------------------------------
#
#  This file is part of the ParaNut project.
#
#  Copyright (C) 2010-2020 Alexander Bahle <alexander.bahle@hs-augsburg.de>
#                          Gundolf Kiefer <gundolf.kiefer@hs-augsburg.de>
#      Hochschule Augsburg, University of Applied Sciences
#
#  Description:
#    ParaNut software section Makefile.
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


PARANUT_HOME ?= $(abspath $(CURDIR)/..)
include $(PARANUT_HOME)/directory-base.mk

TARGET_DIRS := $(shell find $(SYSC_DIR)/* -type d)

################################################################################
#                                                                              #
#     Default and help targets                                                 #
#                                                                              #
################################################################################


# Default target ...
.PHONY: all
all: help

.PHONY: $(TARGET_DIRS)
$(TARGET_DIRS):
	-$(MAKE) -C $@ clean 2> /dev/null

# Print usage info ...
.PHONY: help
help:
	@echo "This makefile is mainly used to clean HW directories. Usually there sould be no need for you to use it."
	@echo
	@echo "Usage: make [<target>] [<parameter>=<value> ...]"
	@echo
	@echo "Targets:"
	@echo
	@echo "  install     : Install hardware source files to the directory given by parameter INSTALL_DIR"
	@echo "                (default: /opt/paranut)"
	@echo "  clean-all       : Clean all subdirectories (delete compilation artefacts)""
	@echo "                (default: /opt/paranut)"


################################################################################
#                                                                              #
#     Installing                                                               #
#                                                                              #
################################################################################
.PHONY: install
install:
	@echo
	@echo "### Installing HW sources to $(INSTALL_DIR) ###"
	cp -dr --no-preserve=ownership $(HW_DIR) $(INSTALL_DIR);


################################################################################
#                                                                              #
#     Cleaning                                                                 #
#                                                                              #
################################################################################
.PHONY: clean-all
clean-all: $(TARGET_DIRS)
	@echo
	@echo "### All hw directories cleaned ###"