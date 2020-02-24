####################################################################################################
# Copyright (C) 2019-2020 Alexander Bahle <alexander.bahle@hs-augsburg.de>
#                         Anna Pfuetzner <annakerstin.pfuetzner@gmail.com>
#
# Redistribution and use in source and binary forms, with or without modification, are permitted 
# provided that the following conditions are met:
# 
# 1. Redistributions of source code must retain the above copyright notice, this list of conditions
# and the following disclaimer.
# 
# 2. Redistributions in binary form must reproduce the above copyright notice, this list of 
# conditions and the following disclaimer in the documentation and/or other materials provided with 
# the distribution.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR 
# IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND 
# FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR 
# CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL 
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, 
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER 
# IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT 
# OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
####################################################################################################

#System Configuration###############################################################################

# Call clean when changing these!

# System Parameters
PN_CACHE_LINESIZE           = auto        # Cache Linesize in Bits, Default: auto
                                          # auto means all power of two linesizes from 32 to 2048
PN_RWIDTH                   = 32          # Register Width in Bits, Default: 32

# Switch for raw compilation (optimized for performance, will drop security checks)
# Default: 0
PN_COMPILE_RAW              = 0

# Chose modules - Set to 0 to switch off
# Default: 1
PN_WITH_BASE                = 1
PN_WITH_CACHE               = 1
PN_WITH_LINK                = 1
PN_WITH_THREAD              = 1
PN_WITH_EXCEPTION           = 1
PN_WITH_SPINLOCK            = 1

#Compiler Configuration#############################################################################

# Instruction Set Architecture
# Currently available:
#    1. RISCV 32 bit - Set to RV32I
# Don't forget to change compiler when touching this!
PN_ISA                      = RV32I

# Compiler
# Currently available:
#    1. GCC - Available GCC for chosen ISA
PN_COMPILER                 = GCC

# Compile with Debug Symbols
# Default: 1
PN_DEBUG                    = 1

#Compiler and Assembler Flags#######################################################################

ifeq ($(PN_ISA),RV32I)
	
	ifeq ($(PN_COMPILER),GCC)
	
		# Set the list of system parameters together
		PN_CONFIG_DEFINES     = -D PN_CACHE_LINESIZE=$(PN_CACHE_LINESIZE) -D PN_RWIDTH=$(PN_RWIDTH) 
		PN_CONFIG_DEFINES    += -D PN_JOBQUEUE_SIZE=$(PN_JOBQUEUE_SIZE)
		
		# Put in define for raw compilation if it was set before
		ifeq ($(PN_COMPILE_RAW),1)
			PN_CONFIG_DEFINES += -D PN_COMPILE_RAW
		endif
	
		# Actual Compiler and Assembler
		CROSS_COMPILE ?= riscv64-unknown-elf
		CC      := $(CROSS_COMPILE)-gcc
		GXX     := $(CROSS_COMPILE)-g++
		OBJDUMP := $(CROSS_COMPILE)-objdump
		OBJCOPY := $(CROSS_COMPILE)-objcopy
		GDB     := $(CROSS_COMPILE)-gdb
		AR      := $(CROSS_COMPILE)-ar
		SIZE    := $(CROSS_COMPILE)-size
	
		# Compiler Flags
		CFLAGS                = -c -ansi -O3 -Wall -Werror -I./ -I./common $(PN_CONFIG_DEFINES)
		CFLAGS               += -mabi=ilp32
		
		ifeq ($(PN_WITH_SPINLOCK),0)
			ifeq ($(PN_WITH_LINK),0)
				CFLAGS         += -march=rv32i
			else
				CFLAGS         += -march=rv32im
			endif
		else
			ifeq ($(PN_WITH_LINK),0)
				CFLAGS         += -march=rv32ia
			else
				CFLAGS         += -march=rv32ima
			endif
		endif
		
		ifeq ($(PN_DEBUG),1)
			CFLAGS            += -g
		endif
		
		CFLAGS               += -o
		
		# Assembler Flags
		AFLAGS                = --fatal-warnings -mabi=ilp32 -I./common
		
		ifeq ($(PN_WITH_SPINLOCK),0)
			ifeq ($(PN_WITH_LINK),0)
				AFLAGS         += -march=rv32i
			else
				AFLAGS         += -march=rv32im
			endif
		else
			ifeq ($(PN_WITH_LINK),0)
				AFLAGS         += -march=rv32ia
			else
				AFLAGS         += -march=rv32ima
			endif
		endif
		
		ifeq ($(PN_DEBUG),1)
			AFLAGS            += -g
		endif
		
		AFLAGS               += -o
	
	else
	
		$(error No valid compiler set for the ISA that you chose.)
		
	endif
	
else
	
	$(error No valid ISA set.)
	
endif

#Lists##############################################################################################

# Assemble object list so we know what objects to build
OBJECT_LIST =

ifeq ($(PN_WITH_BASE),1)
	PN_CONFIG_DEFINES       += -D PN_WITH_BASE
	BPATH                    = ./pn_base/pn_base
	OBJECT_LIST             += $(BPATH).o $(BPATH)_$(PN_ISA).o
endif

ifeq ($(PN_WITH_LINK),1)
	PN_CONFIG_DEFINES       += -D PN_WITH_LINK
	LPATH                    = ./pn_link/pn_link
	OBJECT_LIST             += $(LPATH).o $(LPATH)_$(PN_ISA).o
endif

ifeq ($(PN_WITH_THREAD),1)
	PN_CONFIG_DEFINES       += -D PN_WITH_THREAD
	TPATH                    = ./pn_thread/pn_thread
	OBJECT_LIST             += $(TPATH).o $(TPATH)_$(PN_ISA).o
endif

ifeq ($(PN_WITH_CACHE),1)
	PN_CONFIG_DEFINES       += -D PN_WITH_CACHE
	CPATH                    = ./pn_cache/pn_cache
	OBJECT_LIST             += $(CPATH).o $(CPATH)_$(PN_ISA)_$(strip $(PN_CACHE_LINESIZE)).o
endif

ifeq ($(PN_WITH_EXCEPTION),1)
	PN_CONFIG_DEFINES       += -D PN_WITH_EXCEPTION
	EPATH                    = ./pn_exception/pn_exception
	OBJECT_LIST             += $(EPATH).o $(EPATH)_$(PN_ISA).o
endif

ifeq ($(PN_WITH_SPINLOCK),1)
	PN_CONFIG_DEFINES       += -D PN_WITH_SPINLOCK
	SPATH                    = ./pn_spinlock/pn_spinlock
	OBJECT_LIST             += $(SPATH).o $(SPATH)_$(PN_ISA).o
endif

# Check if object list is empty because it would not make any sense to build the library then
ifeq ($(strip $(OBJECT_LIST)),)
	$(error No modules enabled.)
endif

# Add the objects for the common part
OBJECT_LIST                += ./common/common_$(PN_ISA).o


# Everything in this list has to be put into INSTALL directory
INSTALL_LIST                = ./INSTALL/libparanut.a ./INSTALL/libparanut.h 
INSTALL_LIST               += ./INSTALL/pn_config.h


#Target Magic#######################################################################################

# Creates everything for usage of libparanut
all: $(INSTALL_LIST)

# Creates all cache files for RV32I anew in case of changes in buildscript.
# Important for development only.
cache: ./pn_cache/pn_cache_RV32I_buildscript.py
	python $< 32
	python $< 64
	python $< 128
	python $< 256
	python $< 512
	python $< 1024
	python $< 2048
	python $< auto

# Generates library from object files
./INSTALL/libparanut.a: $(OBJECT_LIST)
	mkdir -p INSTALL
	$(AR) cr $@ $(OBJECT_LIST)
	
# Builds objects from C source code
%.o: %.[cS] libparanut.h pn_config.h
	$(CC) $(CFLAGS) $@ $<;
	
# Cache Module Assembly Code for RV32I is generated at compile time 
./pn_cache/pn_cache_RV32I_*.S: ./pn_cache/pn_cache_RV32I_buildscript.py
	python $< $(strip $(PN_CACHE_LINESIZE))
	
# Put libparanut.h (API) into INSTALL directory
./INSTALL/libparanut.h: libparanut.h pn_config.h
	mkdir -p INSTALL
	cp libparanut.h ./INSTALL/
	
#  Generate pn_config header from system parameters - Only dependent on Makefile itself
SHELL = /bin/bash
./INSTALL/pn_config.h:
	mkdir -p INSTALL
	touch ./INSTALL/pn_config.h
	echo "/* Automatically generated file. See Makefile. No edits here! */"  > ./INSTALL/pn_config.h
	echo "#define PN_CACHE_LINESIZE $(PN_CACHE_LINESIZE)"                   >> ./INSTALL/pn_config.h
	echo "#define PN_RWIDTH $(PN_RWIDTH)"                                   >> ./INSTALL/pn_config.h
	if (($(PN_COMPILE_RAW) == 1));      \
		then echo "#define PN_COMPILE_RAW"     >> ./INSTALL/pn_config.h; fi
	if (($(PN_WITH_BASE) == 1));        \
		then echo "#define PN_WITH_BASE"       >> ./INSTALL/pn_config.h; fi
	if (($(PN_WITH_CACHE) == 1));       \
		then echo "#define PN_WITH_CACHE"      >> ./INSTALL/pn_config.h; fi
	if (($(PN_WITH_EXCEPTION) == 1));   \
		then echo "#define PN_WITH_EXCEPTION"  >> ./INSTALL/pn_config.h; fi
	if (($(PN_WITH_LINK) == 1));        \
		then echo "#define PN_WITH_LINK"       >> ./INSTALL/pn_config.h; fi
	if (($(PN_WITH_THREAD) == 1));      \
		then echo "#define PN_WITH_THREAD"     >> ./INSTALL/pn_config.h; fi
	if (($(PN_WITH_SPINLOCK) == 1));    \
		then echo "#define PN_WITH_SPINLOCK"   >> ./INSTALL/pn_config.h; fi
	
# The other INSTALL targets just have to be copied from common directory
./INSTALL/%: ./common/%
	mkdir -p INSTALL
	cp $< $ ./INSTALL/

# Removes library, object files, and auto generated files
clean:
	find . -name '*.o' -delete
	if [ -d INSTALL ]; then rm -r INSTALL; fi

#EOF################################################################################################
