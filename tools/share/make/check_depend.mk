#  -----------------------------------------------------------------------------
#
#  This file is part of the ParaNut project.
#
#  Copyright (C) 2021 Michael Schaeferling <michael.schaeferling@hs-augsburg.de>
#      Hochschule Augsburg, University of Applied Sciences
#
#  Description:
#    Makefile fragment for dependency checks.
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


# TBD[MS]: Implement correct dependency checks for targets: systemc, riscv, openocd-riscv.

.PHONY: check-paranut
check-paranut:
	@test "$(PARANUT_TOOLS)" != "" || ( echo "ERROR: Variable PARANUT_TOOLS is not set. ParaNut tools are needed for this target."; exit 3; )

.PHONY: check-systemc
check-systemc:
	@test "$(shell whereis libsystemc)" != "" || ( echo "ERROR: SystemC library is not found. SystemC is needed for this target."; exit 3; )
#~ 	@if [ $(shell whereis libsystemc) == "" ]; then \
#~ 	    echo("SystemC library NOT found."); \
#~ 	  else \
#~ 	    echo("SystemC library found."); \
#~ 	  done; \
#~ 	fi

.PHONY: check-riscv
check-riscv:
	@test "$(RISCV_HOME)" != "" || ( echo "ERROR: Variable RISCV_HOME is not set. RISC-V tools are needed for this target."; exit 3; )

.PHONY: check-openocd-riscv
check-openocd-riscv:
	@test "$(OPENOCD-RV_HOME)" != "" || ( echo "ERROR: Variable OPENOCD-RV_HOME is not set. OpenOCD (with RISC-V support) is needed for this target."; exit 3; )

.PHONY: check-vivado
check-vivado:
	@test "$(XILINX_VIVADO)" != "" || ( echo "ERROR: Variable XILINX_VIVADO is not set. Xilinx tools are needed for this target."; exit 3; )
