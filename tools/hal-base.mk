#  -----------------------------------------------------------------------------
#
#  This file is part of the ParaNut project.
#
#  Copyright (C) 2023 Felix Wagner <felix.wagner1@hs-augsburg.de>
#      Hochschule Augsburg, University of Applied Sciences
#
#  Description:
#    Generate hal libraries for the ParaNut
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

#################  HW-Common module - contains hw-configuration ################ 
HAL_CONFIG_H ?= $(PNS_HAL_INC_DIR)/paranut-config.h

################# Library Configuration #########################################

# LIBRARY_C_SRC:               Sources that are used for Simulation as well as 
#                           synthesis. (including the testbench/sc_main file)
LIBRARY_C_SRC ?= 
LIBRARY_ASM_SRC ?= 

# LIBRARY_HEADERS:          Headers provided by the library that shall be copied 
#                           to include
LIBRARY_HEADERS ?=  

# LIBRARY_DEPENDENCIES:     Other libraries this one depends on
#                           libparanut is usually omnypresent
LIBRARY_DEPENDENCIES ?= 

# LIBRARY_CFLAGS:            Additional compiler flags this module needs
LIBRARY_CFLAGS ?= 

# LIBRARY_LDFLAGS:           Additional linker flags this module needs
LIBRARY_LDFLAGS ?=  

# CFG_MARCH:                 Configure the ISA
CFG_MARCH ?= rv32ima

# LIBRARY_NAME:              Must be provided by including makefile. Should be 
#                            generated by parsing the folder name
LIBRARY_NAME ?=

################# Target definitions ###########################################
LIBRARY_TARGET = $(PNS_HAL_LIB_DIR)/$(LIBRARY_NAME).a
HEADER_TARGET := $(addprefix $(PNS_HAL_INC_DIR)/,$(LIBRARY_HEADERS))
LIBRARY_DEPENDENCIE_FILES := $(addprefix $(PNS_HAL_LIB_DIR)/,$(addsuffix .a,$(LIBRARY_DEPENDENCIES)))
LIBRARY_DEPENDENCY_CFLAGS := $(addprefix -l,$(LIBRARY_DEPENDENCIES:lib%=%))
################# Compile Source Setup  ############################################
# SIMBUILD will supress the __SYNTHESIS__ is set info
SIM_FLAGS ?= -DSIMBUILD 
LIB_C_OBJ := $(LIBRARY_C_SRC:.c=.o)
LIB_ASM_OBJ := $(LIBRARY_ASM_SRC:.S=.o)


################# Compiler Configuration #########################################

# Compiler
# Currently available:
#    1. GCC - Available GCC for chosen ISA
PN_COMPILER                 = GCC

# Compile with Debug Symbols
# Default: 1
PN_DEBUG                    = 1

# Actual Compiler and Assembler
CROSS_COMPILE ?= riscv64-unknown-elf
AS      := $(CROSS_COMPILE)-as
CC      := $(CROSS_COMPILE)-gcc
GXX     := $(CROSS_COMPILE)-g++
OBJDUMP := $(CROSS_COMPILE)-objdump
OBJCOPY := $(CROSS_COMPILE)-objcopy
GDB     := $(CROSS_COMPILE)-gdb
AR      := $(CROSS_COMPILE)-ar
SIZE    := $(CROSS_COMPILE)-size

# Compiler Flags
CFLAGS              = -c -O3 -Wall -Werror -I. -I$(PNS_HAL_INC_DIR) $(LIBRARY_CFLAGS)
CFLAGS             += -mabi=ilp32 -march=$(CFG_MARCH) -L$(LIBRARY_DIR) $(LIBRARY_DEPENDENCY_CFLAGS)

ARFLAGS             = 

ifeq ($(PN_DEBUG),1)
	CFLAGS           += -g
endif

################################################################################
#################                Targets                   #####################
################################################################################
.PHONY: all
all: build-lib

#################  Simple Crosscompilation for RISC-V ########################## 

# Builds objects from C source code
$(LIB_C_OBJ): %.o: %.c $(LIBRARY_DEPENDENCIE_FILES) $(LIBRARY_HEADERS) $(HAL_CONFIG_H)
	$(CC) $(CFLAGS) -o $@ $<

# Builds objects from assembler source code
$(LIB_ASM_OBJ): %.o: %.S $(LIBRARY_DEPENDENCIE_FILES) $(LIBRARY_HEADERS) $(HAL_CONFIG_H)
	$(CC) $(CFLAGS) -o $@ $<

#################  Link Library ################################################ 


# .SECONDEXPANSION is needed to enable shell execution in target dependency
.SECONDEXPANSION:
# other libraries 
$(PNS_HAL_LIB_DIR)/%.a: 
	@echo "building $*"
	$(MAKE) -C $(HAL_DIR)/$* build-lib
#find $(HAL_DIR)/src/$*/ -type f \( \! -iname "*.o" \) -print 2>/dev/null

$(PNS_HAL_LIB_DIR)/$(LIBRARY_NAME).a: $(LIB_C_OBJ) $(LIB_ASM_OBJ) $(HEADER_TARGET) Makefile.deps $(CUSTOM_BUILD_TARGETS) 
	mkdir -p $(PNS_HAL_LIB_DIR)
	$(AR) cr $(ARFLAGS) $@ $(LIB_C_OBJ) $(LIB_ASM_OBJ)

#################  Install headers ############################################# 

$(PNS_HAL_INC_DIR)/%.h: %.h
	install -Dp -m 644 $(@F) $@

#################  Standard Phony Target #######################################
.PHONY: build-lib
build-lib: $(LIBRARY_TARGET)
 
################# Make Dependency file for external checks #####################

Makefile.deps: $(LIBRARY_C_SRC) $(LIBRARY_ASM_SRC) $(LIBRARY_HEADERS)
	@echo $(addprefix $(LIBRARY_DIR)/, $(LIBRARY_C_SRC) $(LIBRARY_ASM_SRC) $(LIBRARY_HEADERS)) > Makefile.deps

#################  Config File ################################################# 
# This file contains ParaNut Hardware Parameters. It is generated and kept in the 
# /hw/sysc/module_common directory. Thus the Makefile in hw/sysc must be invoced 
# to generate it. For ease of use, we link it in include

.PHONY: paranut_config
paranut_config: $(HAL_CONFIG_H)

$(CONFIG_H): $(CONFIG_MK) $(PNS_CONFIG_MK)
	$(MAKE) -C $(SYSC_COMMON_DIR) build-config

$(HAL_CONFIG_H): $(CONFIG_H) 
	install -Dp -m 644 $(CONFIG_H) $(HAL_CONFIG_H)


#################  PN clean #################################################### 
.PHONY: clean
clean::
	rm -f $(LIB_C_OBJ) $(LIB_ASM_OBJ) $(LIBRARY_TARGET) $(HEADER_TARGET) Makefile.deps 2> /dev/null

.PHONY: clean-all
clean-all:: $(addprefix clean-,$(LIBRARY_DEPENDENCIES))
	rm -rf $(PNS_HAL_INC_DIR) $(PNS_HAL_LIB_DIR)