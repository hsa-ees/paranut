#  -----------------------------------------------------------------------------
#
#  This file is part of the ParaNut project.
#
#  Copyright (C) 2022 Michael Schaeferling <michael.schaeferling@hs-augsburg.de>
#      Hochschule Augsburg, University of Applied Sciences
#
#  Description:
#    Makefile to build a RISC-V toolchain.
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


TOOLCHAIN_VERSION = 2022.10.11

PREFIX_DEFAULT := /opt/riscv/riscv-gnu-toolchain
INSTALL_DIR ?= $(PREFIX_DEFAULT)

FORCE_OVERWRITE := no


PRESET_DEFAULT := all
PRESET ?= $(PRESET_DEFAULT)

ifeq ($(strip $(PRESET)),all)
  MULTILIB_CONF_RV32e=1
  MULTILIB_CONF_RV32i=1
  MULTILIB_CONF_RV64i=1
endif
ifeq ($(strip $(PRESET)),rv32)
  MULTILIB_CONF_RV32e=1
  MULTILIB_CONF_RV32i=1
endif
ifeq ($(strip $(PRESET)),rv32e)
  MULTILIB_CONF_RV32e=1
endif
ifeq ($(strip $(PRESET)),rv32i)
  MULTILIB_CONF_RV32i=1
endif
ifeq ($(strip $(PRESET)),rv64i)#  -----------------------------------------------------------------------------
#
#  This file is part of the ParaNut project.
#
#  Copyright (C) 2022 Michael Schaeferling <michael.schaeferling@hs-augsburg.de>
#      Hochschule Augsburg, University of Applied Sciences
#
#  Description:
#    Makefile to build a RISC-V toolchain.
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


TOOLCHAIN_VERSION = 2022.10.11

PREFIX_DEFAULT := /opt/riscv/riscv-gnu-toolchain
INSTALL_DIR ?= $(PREFIX_DEFAULT)

FORCE_OVERWRITE := no


PRESET_DEFAULT := all
PRESET ?= $(PRESET_DEFAULT)

ifeq ($(strip $(PRESET)),all)
  MULTILIB_CONF_RV32e=1
  MULTILIB_CONF_RV32i=1
  MULTILIB_CONF_RV64i=1
endif
ifeq ($(strip $(PRESET)),rv32)
  MULTILIB_CONF_RV32e=1
  MULTILIB_CONF_RV32i=1
endif
ifeq ($(strip $(PRESET)),rv32e)
  MULTILIB_CONF_RV32e=1
endif
ifeq ($(strip $(PRESET)),rv32i)
  MULTILIB_CONF_RV32i=1
endif
ifeq ($(strip $(PRESET)),rv64i)
  MULTILIB_CONF_RV64i=1
endif


# Configs are taken from SiFive releases, see assets here: https://github.com/sifive/freedom-tools/releases

## --- Old config, same as SiFive config as of April 2020 ---

# RV32e
ifeq ($(MULTILIB_CONF_RV32e),1)
MULTILIB_CONF_OLD += \
	rv32e-ilp32e--c ;\
	rv32ea-ilp32e--m ;\
	rv32em-ilp32e--c ;\
	rv32eac-ilp32e-- ;\
	rv32emac-ilp32e--
endif

# RV32i
ifeq ($(MULTILIB_CONF_RV32i),1)
MULTILIB_CONF_OLD += \
	rv32i-ilp32--c,f,fc,fd,fdc ;\
	rv32ia-ilp32-rv32ima,rv32iaf,rv32imaf,rv32iafd,rv32imafd- ;\
	rv32im-ilp32--c,f,fc,fd,fdc ;\
	rv32iac-ilp32--f,fd ;\
	rv32imac-ilp32-rv32imafc,rv32imafdc- ;\
	rv32if-ilp32f--c,d,dc ;\
	rv32iaf-ilp32f--c,d,dc ;\
	rv32imf-ilp32f--d ;\
	rv32imaf-ilp32f-rv32imafd- ;\
	rv32imfc-ilp32f--d ;\
	rv32imafc-ilp32f-rv32imafdc- ;\
	rv32ifd-ilp32d--c ;\
	rv32imfd-ilp32d--c ;\
	rv32iafd-ilp32d-rv32imafd,rv32iafdc- ;\
	rv32imafdc-ilp32d--
endif

# RV64i
ifeq ($(MULTILIB_CONF_RV64i),1)
MULTILIB_CONF_OLD += \
	rv64i-lp64--c,f,fc,fd,fdc ;\
	rv64ia-lp64-rv64ima,rv64iaf,rv64imaf,rv64iafd,rv64imafd- ;\
	rv64im-lp64--c,f,fc,fd,fdc ;\
	rv64iac-lp64--f,fd ;\
	rv64imac-lp64-rv64imafc,rv64imafdc- ;\
	rv64if-lp64f--c,d,dc ;\
	rv64iaf-lp64f--c,d,dc ;\
	rv64imf-lp64f--d ;\
	rv64imaf-lp64f-rv64imafd- ;\
	rv64imfc-lp64f--d ;\
	rv64imafc-lp64f-rv64imafdc- ;\
	rv64ifd-lp64d--c ;\
	rv64imfd-lp64d--c ;\
	rv64iafd-lp64d-rv64imafd,rv64iafdc- ;\
	rv64imafdc-lp64d--
endif


## --- New config, same as SiFive config as of August 2020 ---

# RV32e
ifeq ($(MULTILIB_CONF_RV32e),1)
MULTILIB_CONF += \
	rv32e-ilp32e--c*v*zvqmac ;\
	rv32ea-ilp32e--m*v*zvqmac ;\
	rv32em-ilp32e--c*v*zvqmac ;\
	rv32eac-ilp32e--v*zvqmac ;\
	rv32emac-ilp32e--v*zvqmac
endif

# RV32i
ifeq ($(MULTILIB_CONF_RV32i),1)
MULTILIB_CONF += \
	rv32i-ilp32--c*f*d*zfh*v*zvqmac ;\
	rv32ia-ilp32--m*f*d*v*zfh*zvqmac ;\
	rv32im-ilp32--c*f*d*zfh*v*zvqmac ;\
	rv32iac-ilp32--f*d*v*zfh*zvqmac ;\
	rv32imac-ilp32-rv32imafc,rv32imafdc,rv32imafczfh,rv32imafdczfh-v*zvqmac ;\
	rv32if-ilp32f--d*c*v*zfh*zvqmac ;\
	rv32iaf-ilp32f--d*c*v*zfh*zvqmac ;\
	rv32imf-ilp32f--d*v*zfh*zvqmac ;\
	rv32imaf-ilp32f-rv32imafd-zfh*v*zvqmac ;\
	rv32imfc-ilp32f--d*v*zfh*zvqmac ;\
	rv32imafc-ilp32f-rv32imafdc-v*zfh*zvqmac ;\
	rv32ifd-ilp32d--c*v*zfh*zvqmac ;\
	rv32imfd-ilp32d--c*v*zfh*zvqmac ;\
	rv32iafd-ilp32d-rv32imafd,rv32iafdc-v*zfh*zvqmac ;\
	rv32imafdc-ilp32d--v*zfh*zvqmac
endif

# RV64i
ifeq ($(MULTILIB_CONF_RV64i),1)
MULTILIB_CONF += \
	rv64i-lp64--f*d*c*v*zfh*zvqmac ;\
	rv64ia-lp64--m*f*d*v*zfh*zvqmac ;\
	rv64im-lp64--f*d*c*v*zfh*zvqmac ;\
	rv64iac-lp64--f*d*v*zfh*zvqmac ;\
	rv64imac-lp64-rv64imafc,rv64imafdc,rv64imafczfh,rv64imafdczfh-v*zvqmac ;\
	rv64if-lp64f--d*c*v*zfh*zvqmac ;\
	rv64iaf-lp64f--d*c*v*zfh*zvqmac ;\
	rv64imf-lp64f--d*v*zfh*zvqmac ;\
	rv64imaf-lp64f-rv64imafd-v*zfh*zvqmac ;\
	rv64imfc-lp64f--d*v*zfh*zvqmac ;\
	rv64imafc-lp64f-rv64imafdc-v*zfh*zvqmac ;\
	rv64ifd-lp64d--c*v*zfh*zvqmac ;\
	rv64imfd-lp64d--c*v*zfh*zvqmac ;\
	rv64iafd-lp64d-rv64imafd,rv64iafdc-v*zfh*zvqmac ;\
	rv64imafdc-lp64d--v*zfh*zvqmac
endif



.PHONY: all
all: build


.PHONY: help
help:
	@echo; \
	echo "This makefile helps to build and install the"; \
	echo "  'RISC-V GNU Compiler Toolchain' from 'RISC-V Software Collaboration'"; \
	echo "  from sources according to <https://github.com/riscv-collab/riscv-gnu-toolchain>."; \
	echo; \
	echo "Be aware that the installation directory (INSTALL_DIR) must be writeable";\
	echo "  by the user account that calls 'make all/build'.";\
	echo "If the installation directory already exists, the user will be asked to confirm the installation.";\
	echo "  This check can be omitted by setting 'FORCE_OVERWRITE=yes' (the installation directory will NOT be cleaned/purged).";\
	echo "  To purge the installation directory, you may run 'make veryclean' beforehand.";\
	echo; \
	echo "The architecture set (or a combination) can be chosen by setting the PRESET variable:";\
	echo "   - all   : RV32e, RV32i and RV64i";\
	echo "   - rv32  : RV32e and RV32i";\
	echo "   - rv32e : RV32e";\
	echo "   - rv32i : RV32i";\
	echo "   - rv64i : RV64i";\
	echo; \
	echo "Example: to build and install a RV32i toolchain to '/tools/riscv', use";\
	echo "         make PRESET=rv32i INSTALL_DIR=/tools/riscv";\
	echo; \
	echo; \
	echo "Overview on the Makefiles targets:";\
	echo; \
	echo "* help : Print this help text."; \
	echo; \
	echo "* all : Default target, invokes 'make build' (see below for details and arguments)."; \
	echo; \
	echo "* prepare-syspkgs : Install dependencies in terms of system packages needed to build the toolchain."; \
	echo; \
	echo "* prepare-repo : Get the repository (sources)."; \
	echo; \
	echo "* build : Build and install the toolchain where ..."; \
	echo "           - the architecture set can be specified by PRESET [default = $(PRESET_DEFAULT)]."; \
	echo "           - the installation directory can be specified by INSTALL_DIR [default = $(PREFIX_DEFAULT)]."; \
	echo "          'make [ PRESET=<arch-set> ] [ INSTALL_DIR=<installation_directory> ]'"; \
	echo; \
	echo "* clean : Cleanup the repository (sources) directory (clean from previous runs)."; \
	echo; \
	echo "* veryclean : Invokes 'make clean', additionally purges the installation (INSTALL_DIR) directory."; \
	echo;


.PHONY: prepare-syspkgs
prepare-syspkgs:
	apt-get install autoconf automake autotools-dev curl python3 libmpc-dev libmpfr-dev libgmp-dev gawk build-essential bison flex texinfo gperf libtool patchutils bc zlib1g-dev libexpat-dev

.PHONY: prepare-repo
prepare-repo: riscv-gnu-toolchain


riscv-gnu-toolchain:
	@echo "Cloning repository ..."
	git clone --branch $(TOOLCHAIN_VERSION) https://github.com/riscv/riscv-gnu-toolchain
	@echo "... done cloning."


.PHONY: check-prefix
check-prefix:
	@if [ -d $(INSTALL_DIR) ]; then \
	  if [ $(FORCE_OVERWRITE) != "yes" ]; then \
	    echo -n "Installation directory '$(INSTALL_DIR)' exists. Do you really want to install there (likely to overwrite contents)? [y/N] " && read ans && [ $${ans:-N} = y ]; \
	  fi; \
	fi;

.PHONY: build
build: check-prefix riscv-gnu-toolchain
	$(MAKE) clean && \
	cd riscv-gnu-toolchain && \
	./configure --prefix=$(INSTALL_DIR) --enable-multilib --with-multilib-generator="$(MULTILIB_CONF)" && \
	$(MAKE)


.PHONY: clean
clean:
	@if [ -f "riscv-gnu-toolchain/Makefile" ]; then \
	  echo "Cleaning up riscv-gnu-toolchain/ ..."; \
	  cd riscv-gnu-toolchain && \
	  $(MAKE) clean ; \
	fi;

.PHONY: veryclean
veryclean:
	$(MAKE) clean
	rm -rf $(INSTALL_DIR)

  MULTILIB_CONF_RV64i=1
endif


# Configs are taken from SiFive releases, see assets here: https://github.com/sifive/freedom-tools/releases

## --- Old config, same as SiFive config as of April 2020 ---

# RV32e
ifeq ($(MULTILIB_CONF_RV32e),1)
MULTILIB_CONF_OLD += \
	rv32e-ilp32e--c ;\
	rv32ea-ilp32e--m ;\
	rv32em-ilp32e--c ;\
	rv32eac-ilp32e-- ;\
	rv32emac-ilp32e--
endif

# RV32i
ifeq ($(MULTILIB_CONF_RV32i),1)
MULTILIB_CONF_OLD += \
	rv32i-ilp32--c,f,fc,fd,fdc ;\
	rv32ia-ilp32-rv32ima,rv32iaf,rv32imaf,rv32iafd,rv32imafd- ;\
	rv32im-ilp32--c,f,fc,fd,fdc ;\
	rv32iac-ilp32--f,fd ;\
	rv32imac-ilp32-rv32imafc,rv32imafdc- ;\
	rv32if-ilp32f--c,d,dc ;\
	rv32iaf-ilp32f--c,d,dc ;\
	rv32imf-ilp32f--d ;\
	rv32imaf-ilp32f-rv32imafd- ;\
	rv32imfc-ilp32f--d ;\
	rv32imafc-ilp32f-rv32imafdc- ;\
	rv32ifd-ilp32d--c ;\
	rv32imfd-ilp32d--c ;\
	rv32iafd-ilp32d-rv32imafd,rv32iafdc- ;\
	rv32imafdc-ilp32d--
endif

# RV64i
ifeq ($(MULTILIB_CONF_RV64i),1)
MULTILIB_CONF_OLD += \
	rv64i-lp64--c,f,fc,fd,fdc ;\
	rv64ia-lp64-rv64ima,rv64iaf,rv64imaf,rv64iafd,rv64imafd- ;\
	rv64im-lp64--c,f,fc,fd,fdc ;\
	rv64iac-lp64--f,fd ;\
	rv64imac-lp64-rv64imafc,rv64imafdc- ;\
	rv64if-lp64f--c,d,dc ;\
	rv64iaf-lp64f--c,d,dc ;\
	rv64imf-lp64f--d ;\
	rv64imaf-lp64f-rv64imafd- ;\
	rv64imfc-lp64f--d ;\
	rv64imafc-lp64f-rv64imafdc- ;\
	rv64ifd-lp64d--c ;\
	rv64imfd-lp64d--c ;\
	rv64iafd-lp64d-rv64imafd,rv64iafdc- ;\
	rv64imafdc-lp64d--
endif


## --- New config, same as SiFive config as of August 2020 ---

# RV32e
ifeq ($(MULTILIB_CONF_RV32e),1)
MULTILIB_CONF += \
	rv32e-ilp32e--c*v*zvqmac ;\
	rv32ea-ilp32e--m*v*zvqmac ;\
	rv32em-ilp32e--c*v*zvqmac ;\
	rv32eac-ilp32e--v*zvqmac ;\
	rv32emac-ilp32e--v*zvqmac
endif

# RV32i
ifeq ($(MULTILIB_CONF_RV32i),1)
MULTILIB_CONF += \
	rv32i-ilp32--c*f*d*zfh*v*zvqmac ;\
	rv32ia-ilp32--m*f*d*v*zfh*zvqmac ;\
	rv32im-ilp32--c*f*d*zfh*v*zvqmac ;\
	rv32iac-ilp32--f*d*v*zfh*zvqmac ;\
	rv32imac-ilp32-rv32imafc,rv32imafdc,rv32imafczfh,rv32imafdczfh-v*zvqmac ;\
	rv32if-ilp32f--d*c*v*zfh*zvqmac ;\
	rv32iaf-ilp32f--d*c*v*zfh*zvqmac ;\
	rv32imf-ilp32f--d*v*zfh*zvqmac ;\
	rv32imaf-ilp32f-rv32imafd-zfh*v*zvqmac ;\
	rv32imfc-ilp32f--d*v*zfh*zvqmac ;\
	rv32imafc-ilp32f-rv32imafdc-v*zfh*zvqmac ;\
	rv32ifd-ilp32d--c*v*zfh*zvqmac ;\
	rv32imfd-ilp32d--c*v*zfh*zvqmac ;\
	rv32iafd-ilp32d-rv32imafd,rv32iafdc-v*zfh*zvqmac ;\
	rv32imafdc-ilp32d--v*zfh*zvqmac
endif

# RV64i
ifeq ($(MULTILIB_CONF_RV64i),1)
MULTILIB_CONF += \
	rv64i-lp64--f*d*c*v*zfh*zvqmac ;\
	rv64ia-lp64--m*f*d*v*zfh*zvqmac ;\
	rv64im-lp64--f*d*c*v*zfh*zvqmac ;\
	rv64iac-lp64--f*d*v*zfh*zvqmac ;\
	rv64imac-lp64-rv64imafc,rv64imafdc,rv64imafczfh,rv64imafdczfh-v*zvqmac ;\
	rv64if-lp64f--d*c*v*zfh*zvqmac ;\
	rv64iaf-lp64f--d*c*v*zfh*zvqmac ;\
	rv64imf-lp64f--d*v*zfh*zvqmac ;\
	rv64imaf-lp64f-rv64imafd-v*zfh*zvqmac ;\
	rv64imfc-lp64f--d*v*zfh*zvqmac ;\
	rv64imafc-lp64f-rv64imafdc-v*zfh*zvqmac ;\
	rv64ifd-lp64d--c*v*zfh*zvqmac ;\
	rv64imfd-lp64d--c*v*zfh*zvqmac ;\
	rv64iafd-lp64d-rv64imafd,rv64iafdc-v*zfh*zvqmac ;\
	rv64imafdc-lp64d--v*zfh*zvqmac
endif



.PHONY: all
all: build


.PHONY: help
help:
	@echo; \
	echo "This makefile helps to build and install the"; \
	echo "  'RISC-V GNU Compiler Toolchain' from 'RISC-V Software Collaboration'"; \
	echo "  from sources according to <https://github.com/riscv-collab/riscv-gnu-toolchain>."; \
	echo; \
	echo "Be aware that the installation directory (INSTALL_DIR) must be writeable";\
	echo "  by the user account that calls 'make all/build'.";\
	echo "If the installation directory already exists, the user will be asked to confirm the installation.";\
	echo "  This check can be omitted by setting 'FORCE_OVERWRITE=yes' (the installation directory will NOT be cleaned/purged).";\
	echo "  To purge the installation directory, you may run 'make veryclean' beforehand.";\
	echo; \
	echo "The architecture set (or a combination) can be chosen by setting the PRESET variable:";\
	echo "   - all   : RV32e, RV32i and RV64i";\
	echo "   - rv32  : RV32e and RV32i";\
	echo "   - rv32e : RV32e";\
	echo "   - rv32i : RV32i";\
	echo "   - rv64i : RV64i";\
	echo; \
	echo "Example: to build and install a RV32i toolchain to '/tools/riscv', use";\
	echo "         make PRESET=rv32i INSTALL_DIR=/tools/riscv";\
	echo; \
	echo; \
	echo "Overview on the Makefiles targets:";\
	echo; \
	echo "* help : Print this help text."; \
	echo; \
	echo "* all : Default target, invokes 'make build' (see below for details and arguments)."; \
	echo; \
	echo "* prepare-syspkgs : Install dependencies in terms of system packages needed to build the toolchain."; \
	echo; \
	echo "* prepare-repo : Get the repository (sources)."; \
	echo; \
	echo "* build : Build and install the toolchain where ..."; \
	echo "           - the architecture set can be specified by PRESET [default = $(PRESET_DEFAULT)]."; \
	echo "           - the installation directory can be specified by INSTALL_DIR [default = $(PREFIX_DEFAULT)]."; \
	echo "          'make [ PRESET=<arch-set> ] [ INSTALL_DIR=<installation_directory> ]'"; \
	echo; \
	echo "* clean : Cleanup the repository (sources) directory (clean from previous runs)."; \
	echo; \
	echo "* veryclean : Invokes 'make clean', additionally purges the installation (INSTALL_DIR) directory."; \
	echo;


.PHONY: prepare-syspkgs
prepare-syspkgs:
	apt-get install autoconf automake autotools-dev curl python3 libmpc-dev libmpfr-dev libgmp-dev gawk build-essential bison flex texinfo gperf libtool patchutils bc zlib1g-dev libexpat-dev

.PHONY: prepare-repo
prepare-repo: riscv-gnu-toolchain


riscv-gnu-toolchain:
	@echo "Cloning repository ..."
	git clone --branch $(TOOLCHAIN_VERSION) https://github.com/riscv/riscv-gnu-toolchain
	@echo "... done cloning."


.PHONY: check-prefix
check-prefix:
	@if [ -d $(INSTALL_DIR) ]; then \
	  if [ $(FORCE_OVERWRITE) != "yes" ]; then \
	    echo -n "Installation directory '$(INSTALL_DIR)' exists. Do you really want to install there (likely to overwrite contents)? [y/N] " && read ans && [ $${ans:-N} = y ]; \
	  fi; \
	fi;

.PHONY: build
build: check-prefix riscv-gnu-toolchain
	$(MAKE) clean && \
	cd riscv-gnu-toolchain && \
	./configure --prefix=$(INSTALL_DIR) --enable-multilib --with-multilib-generator="$(MULTILIB_CONF)" && \
	$(MAKE)


.PHONY: clean
clean:
	@if [ -f "riscv-gnu-toolchain/Makefile" ]; then \
	  echo "Cleaning up riscv-gnu-toolchain/ ..."; \
	  cd riscv-gnu-toolchain && \
	  $(MAKE) clean ; \
	fi;

.PHONY: veryclean
veryclean:
	$(MAKE) clean
	rm -rf $(INSTALL_DIR)
