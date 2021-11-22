#  This file is part of the ParaNut project.
#
#  Copyright (C) 2019-2021 Alexander Bahle <alexander.bahle@hs-augsburg.de>
#                          Michael Schaeferling <michael.schaeferling@hs-augsburg.de>
#      Hochschule Augsburg, University of Applied Sciences
#
#  Description:
#    This makefile provides a target to generate a hardware project,
#    based on a given blockdesign.
#    This makefile is to be called from within the directory where
#    the project is to be built:
#      usually, within '../../../hardware' call
#      > make -f boards/${BOARD}/Makefile generate_project
#
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


ifndef PARANUT_TOOLS
$(error PARANUT_TOOLS is not set)
endif


BOARD_DIR=boards/zybo

CONSTRAINTS=$(wildcard $(BOARD_DIR)/constraints/*.xdc)

.PHONY: all
all: generate_project

# The target 'generate_project' generates a Vivado project (.xpr) from
#   - a given .bd-file and
#   - a set of additional source files (e.g. constraints)
.PHONY: generate_project
generate_project:
	# Create project
	mkdir -p build.bd
	ln -s -f ../$(BOARD_DIR)/bd/design_1.bd build.bd/design_1.bd
	( echo 'create_project -force system ./build -part xc7z010clg400-1'; \
	echo 'set_property -name "board_part" -value "digilentinc.com:zybo:part0:1.0" -objects [current_project]'; \
	echo 'set_property -name "target_language" -value "VHDL" -objects [current_project]'; \
	echo 'set_property -name "ip_cache_permissions" -value "read write" -objects [current_project]'; \
	echo 'set_property -name "ip_output_repo" -value "build/design_1.cache/ip" -objects [current_project]'; \
	echo 'set_property "ip_repo_paths" "../vivado_cores" [current_fileset]'; \
	echo 'update_ip_catalog -rebuild'; \
	echo 'add_files build.bd/design_1.bd'; \
	echo 'make_wrapper -top [get_files design_1.bd]'; \
	echo 'set_property top design_1_wrapper [current_fileset]'; \
	echo 'add_files build.bd/hdl/design_1_wrapper.vhd'; \
	echo 'add_files [split "$(CONSTRAINTS)"]';) \
	| vivado -mode tcl
	rm -f vivado*.log vivado*.jou
