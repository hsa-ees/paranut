#  -----------------------------------------------------------------------------
#
#  This file is part of the ParaNut project.
#
#  Copyright (C) 2020-2021 Michael Schaeferling <michael.schaeferling@hs-augsburg.de>
#      Hochschule Augsburg, University of Applied Sciences
#
#  Description:
#    Makefile for the debug_shell firmware.
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


FW_BUILD_DIR = fw_build
FIRMWARE = firmware
FIRMWARE_BIN= $(FIRMWARE).elf
FIRMWARE_BUILD_BIN = $(FW_BUILD_DIR)/$(FIRMWARE_BIN)

SOUND_CHIP_DEBUG_HDF_DIR = ../
SOUND_CHIP_DEBUG_HDF_FILE = sound_chip_debug.hdf
SOUND_CHIP_DEBUG_HDF = $(SOUND_CHIP_DEBUG_HDF_DIR)$(SOUND_CHIP_DEBUG_HDF_FILE)


all: firmware



.PHONY: firmware
firmware:
	$(MAKE) $(FIRMWARE_BIN)


$(SOUND_CHIP_DEBUG_HDF):
	make -C .. $(SOUND_CHIP_DEBUG_HDF_FILE)


$(FW_BUILD_DIR): $(SOUND_CHIP_DEBUG_HDF)
	@rm -rf $(FW_BUILD_DIR) && \
	pn-xil-swtool -a $(SOUND_CHIP_DEBUG_HDF) $(FW_BUILD_DIR) hello_world && \
	rm -rf $(FW_BUILD_DIR)/hello_world_bsp/ $(FW_BUILD_DIR)/helloworld.c && \
	cd $(FW_BUILD_DIR) && ln -s ../*.c ../*.h . && cd .. && \
	pn-xil-swtool -b $(SOUND_CHIP_DEBUG_HDF) $(FW_BUILD_DIR)/$(FIRMWARE)_bsp && \
	sed -i.bak s/hello_world_bsp/$(FIRMWARE)_bsp/g  $(FW_BUILD_DIR)/Makefile && \
	sed -i.bak s/executable.elf/$(FIRMWARE_BIN)/g  $(FW_BUILD_DIR)/Makefile && \
	sed -i.bak s/'_SIZE : 0x2000'/'_SIZE : 0x2000000'/g  $(FW_BUILD_DIR)/lscript.ld && \
	rm -rf pn-xil-swtool.tcl


$(FIRMWARE_BUILD_BIN): $(FW_BUILD_DIR) *.c *.h
	$(MAKE) -C $(FW_BUILD_DIR)


$(FIRMWARE_BIN): $(FIRMWARE_BUILD_BIN)
	cp $(FIRMWARE_BUILD_BIN) .


.PHONY: clean
clean:
	rm -rf $(FIRMWARE_BIN) $(FW_BUILD_DIR) pn-xil-swtool.tcl .Xil


# Install ...
.PHONY: install
install: $(FIRMWARE_BIN)
	@test "$(PREFIX)" != "" || ( echo "ERROR: Make variable PREFIX must be set for the 'install' target."; exit 3; )
	install -Dp -m 644 -t $(PREFIX)/share/soc_paranut $(FIRMWARE_BIN)
