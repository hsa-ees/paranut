#! /bin/bash
#
# This file is part of the ParaNut project.
# 
#  Copyright (C) 2010-2020 Alexander Bahle <alexander.bahle@hs-augsburg.de>
#                          Philip Manke <philip.manke@hs-augsburg.de>
#                          Michael Schaeferling <michael.schaeferling@hs-augsburg.de>
#     Hochschule Augsburg, University of Applied Sciencess
# 
# Description:
#    These settings are needed for proper operation of scripts and Makefiles 
#    e.g. to generate an ParaNut IP core or a complete system.
#    Usage:
#      > . settings.sh
#    or 
#      > source settings.sh
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

# Set PARANUT_HOME
PARANUT_HOME=`dirname "$(readlink -f "$0")"`
export PARANUT_HOME

# Add ParaNut python tools to PATH
PATH=$PATH:${PARANUT_HOME}/tools

# The general UAS EES-Lab environment already includes following setup steps, 
# thus they are omitted in that case:
if [ -z $EES_HOME ]; then

  # Add EES tools to PATH
  PATH=$PATH:${PARANUT_HOME}/tools/ees/

  if [ -z $XILINX_VIVADO ]; then
    echo "Vivado is not sourced! Assuming it is located in /opt/Xilinx/Vivado/2017.2/ .";
    echo "If this is not correct, either modify this file (<asterics>/settings.sh)";
    echo "or source Vivado by setting XILINX_VIVADO.";
    export EES_VIVADO_SETTINGS=/opt/Xilinx/Vivado/2017.2/settings64.sh
  else
    export EES_VIVADO_SETTINGS=$XILINX_VIVADO/settings64.sh
  fi

fi
