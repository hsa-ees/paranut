#  -----------------------------------------------------------------------------
#
#  This file is part of the ParaNut project.
#
#  Copyright (C) 2020-2021 Michael Schaeferling <michael.schaeferling@hs-augsburg.de>
#                          Gundolf Kiefer <gundolf.kiefer@hs-augsburg.de>
#      Hochschule Augsburg, University of Applied Sciences
#
#  Description:
#    Makefile for the 'sound_chip_debug' module.
#
#  Editing the hardware:
#    1. Run the Vivado script for building the DCP ('make edit').
#    2. Open the Vivado project 'build/sound_chip_debug.xpr'.
#    3. Do your work.
#
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


SOUND_CHIP_DEBUG_HDF = sound_chip_debug.hdf
SOUND_CHIP_DEBUG_DCP = sound_chip_debug.dcp

PARANUT = paranut
PARANUT_IP_CORE = ip_repo/paranut_ip


# Default target
.PHONY: all
all: build


# General/build ...
.PHONY: build
build: firmware


.PHONY: edit
edit: $(SOUND_CHIP_DEBUG_HDF)


$(SOUND_CHIP_DEBUG_HDF): $(PARANUT_IP_CORE)
	rm -fr build.bd
	mkdir -p build.bd
	ln -s ../sound_chip_debug.bd build.bd/sound_chip_debug.bd
	( echo 'create_project -force sound_chip_debug ./build -part xc7z010clg400-1'; \
	  echo 'set_property -name "target_language" -value "VHDL" -objects [current_project]'; \
	  echo 'set_property -name "ip_cache_permissions" -value "read write" -objects [current_project]'; \
	  echo 'set_property -name "ip_output_repo" -value "build/sound_chip_debug.cache/ip" -objects [current_project]'; \
	  echo 'set_property "ip_repo_paths" "ip_repo" [current_fileset]'; \
	  echo 'update_ip_catalog -rebuild'; \
	  echo 'add_files build.bd/sound_chip_debug.bd'; \
	  echo 'upgrade_ip [get_ips]'; \
	  echo 'generate_target all [get_files sound_chip_debug.bd]'; \
	  echo 'make_wrapper -top [get_files sound_chip_debug.bd]'; \
	  echo 'add_files build.bd/hdl/sound_chip_debug_wrapper.vhd'; \
	  echo 'set_property top sound_chip_debug_wrapper [current_fileset]'; \
	  echo 'set_property synth_checkpoint_mode None [get_files build.bd/sound_chip_debug.bd]'; \
	  echo 'synth_design -mode out_of_context'; \
	  echo 'opt_design'; \
	  echo 'write_checkpoint -force $(SOUND_CHIP_DEBUG_DCP)'; \
	  echo 'write_hwdef -force $(SOUND_CHIP_DEBUG_HDF)' ) \
	  | vivado -notrace -nojournal -nolog -mode tcl 2>&1


# Note: Setting CFG_MEMU_CACHE_BANKS_LD to 4 is a workaround for a bug (to be fixed):
# with the default setting of CFG_MEMU_CACHE_BANKS_LD=2, the system would build, but would not work on the board.
$(PARANUT):
	pn-newproject -c $(PARANUT) && \
	sed -i.bak1 's/CFG_NUT_CPU_CORES_LD\( *\)?=\( *\)\([0-9]\+\)/CFG_NUT_CPU_CORES_LD ?= 1/g' $(PARANUT)/config.mk && \
	sed -i.bak2 's/CFG_MEMU_CACHE_BANKS_LD\( *\)?=\( *\)\([0-9]\+\)/CFG_MEMU_CACHE_BANKS_LD ?= 4/g' $(PARANUT)/config.mk && \
	$(MAKE) -C $(PARANUT)/hw/sysc paranut-config.h


$(PARANUT_IP_CORE): $(PARANUT)
	$(MAKE) -C $(PARANUT) build-ip-xilinx && \
	cd ip_repo/ && \
	ln -sf ../$(PARANUT)/hw/tools/paranut_ip/ && \
	cd -


.PHONY: firmware
firmware:
	$(MAKE) -C firmware && \
	ln -sf firmware/firmware.elf



.PHONY: clean
clean:
	$(MAKE) -C firmware clean
	rm -fr $(PARANUT) $(PARANUT_IP_CORE)
	rm -f $(SOUND_CHIP_DEBUG_DCP) $(SOUND_CHIP_DEBUG_HDF)
	rm -fr build build.bd firmware.elf
	rm -fr drivers .Xil NA vivado*.jou vivado*.log vivado_*.str fsm_encoding.os usage_statistics_webtalk.*ml

.PHONY: veryclean
veryclean: clean



# Install ...
.PHONY: install
install: $(SOUND_CHIP_DEBUG_HDF) firmware
	@test "$(PREFIX)" != "" || ( echo "ERROR: Make variable PREFIX must be set for the 'install' target."; exit 3; )
	install -Dp -m 644 -t $(PREFIX)/share/soc_paranut zybo-paranut_soundchip.xdc sound_chip_system.vhd $(SOUND_CHIP_DEBUG_DCP) $(SOUND_CHIP_DEBUG_HDF)
	$(MAKE) -C firmware install
