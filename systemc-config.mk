#  -----------------------------------------------------------------------------
#
#  This file is part of the ParaNut project.
#
#  Copyright (C) 2023 Felix Wagner <felix.wagner1@hs-augsburg.de>
#      Hochschule Augsburg, University of Applied Sciences
#
#  Description:
#    This file provides configuration to access and link a SystemC library
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


################### INFO #######################################################
# Unfortunally, the most recent version of Accellera's SystemC library is 
# structured slightly different to the SystemC library provided with the Intel  
# Compiler for SystemC (ICSC) [most notably the folder name inside the library 
# directory containing the .so file]. 
# Depending on which library you want to use, comment out/in the following 
# sections.

# should you receive errors similar to this:
# undefined reference to `sc_core::sc_api_version_2_3_2_cxx201703L<&sc_core::SC_DISABLE_VIRTUAL_BIND_UNDEFINED_>::sc_api_version_2_3_2_cxx201703L(sc_core::sc_writer_policy)'
# check the cpp version your SystemC library was compiled for and edit SYSTEMC_CPP_VER


ifndef USE_ICSC
################### Using Accellera's SystemC Library ##########################
SYSTEMC_HOME ?=
# SYSTEMC_LIB: location of libsystem.so
SYSTEMC_LIB ?= $(SYSTEMC_HOME)/lib-linux
# SYSTEMC_INCLUDE: location of  systemc.h            
SYSTEMC_INCLUDE ?= $(SYSTEMC_HOME)/include
# SYSTEMC_CPP_VER: CPP Standard Version must match that of the SystemC library 
SYSTEMC_CPP_VER ?= gnu++14
else
# #################### Using ICSC's SystemC Library ##############################
SYSTEMC_HOME ?= $(ICSC_HOME)
# SYSTEMC_LIB: location of libsystem.so
SYSTEMC_LIB ?= $(SYSTEMC_HOME)/lib
# SYSTEMC_INCLUDE: location of  systemc.h            
SYSTEMC_INCLUDE ?= $(SYSTEMC_HOME)/include
# SYSTEMC_CPP_VER: CPP Standard Version must match that of the SystemC library 
SYSTEMC_CPP_VER ?= gnu++17
endif