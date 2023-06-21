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

TARGET_DIRS := $(shell find $(SYSTEMS_DIR)/* -type d)

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

.PHONY: $(PN_SYSTEM)
$(PN_SYSTEM):
	-$(MAKE) -C $@ clean 2> /dev/null

# Print usage info ...
.PHONY: help
help:
	@echo "This makefile is mainly used to clean system directories. Usually there sould be no need for you to use it."
	@echo
	@echo "Usage: make [<target>] [<parameter>=<value> ...]"
	@echo
	@echo "Targets:"
	@echo
	@echo "  install     : Install hardware source files to the directory given by parameter INSTALL_DIR"
	@echo "                (default: /opt/paranut)"
	@echo "  clean-all   : Clean all subdirectories (delete compilation artefacts)"
	@echo "                (default: /opt/paranut)"
	@echo "  clean       : Clean only PN_SYSTEM (default: $(PN_SYSTEM_DEFAULT))"
	@echo "                (default: /opt/paranut)"


################################################################################
#                                                                              #
#     Installing                                                               #
#                                                                              #
################################################################################
INSTALL_DIR_SUFFIX=$(subst $(PARANUT_HOME),,$(SYSTEMS_DIR))
INSTALL_PN_SIM_BIN=$(INSTALL_DIR)/$(subst $(PARANUT_HOME),,$(PN_SIM_BIN))
INSTALL_BIN_DIR=$(INSTALL_DIR)/$(subst $(PARANUT_HOME),,$(BIN_DIR))
.PHONY: install
install:
	@echo
	@echo "### Installing System sources to $(INSTALL_DIR) ###"
	@echo "# Note: only system $(PN_SYSTEM) will be installed #"
	mkdir -p $(INSTALL_DIR)/$(INSTALL_DIR_SUFFIX)
	cp -dr --no-preserve=ownership $(PNS_DIR) $(INSTALL_DIR)/$(INSTALL_DIR_SUFFIX)
	@echo "# Create link to pn-sim in bin dir #"
	rm -f $(INSTALL_BIN_DIR)/$(PN_SIM_BIN_NAME)
	ln -s $(INSTALL_PN_SIM_BIN) $(INSTALL_BIN_DIR)/$(PN_SIM_BIN_NAME)


################################################################################
#                                                                              #
#     Cleaning                                                                 #
#                                                                              #
################################################################################
.PHONY: clean-all
clean-all: $(TARGET_DIRS)
	@echo
	@echo "### All system directories cleaned ###"

.PHONY: clean
clean: $(PN_SYSTEM)