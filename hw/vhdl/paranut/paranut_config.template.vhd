--------------------------------------------------------------------------------
--
--  This file is part of the ParaNut project.
--
--  Copyright (C) 2013-2019 Alexander Bahle <alexander.bahle@hs-augsburg.de>
--                          Michael Seider, <michael.seider@hs-augsburg.de>
--      Hochschule Augsburg, University of Applied Sciences
--
-- Description:
--  ParaNut global configuration file.
--
-- --------------------- LICENSE -----------------------------------------------
--  Redistribution and use in source and binary forms, with or without modification,
--  are permitted provided that the following conditions are met:
--
--  1. Redistributions of source code must retain the above copyright notice, this
--     list of conditions and the following disclaimer.
--
--  2. Redistributions in binary form must reproduce the above copyright notice,
--     this list of conditions and the following disclaimer in the documentation and/or
--     other materials provided with the distribution.
--
--  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
--  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
--  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
--  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
--  ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
--  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
--  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
--  ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
--  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
--  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;

package paranut_config is

  -- Do not edit
  constant KB : natural := 1024;
  constant MB : natural := 1024*1024;
  constant GB : natural := 1024*1024*1024;

  ------------------------------------------------------
  -- Debug options (simulation only)
  ------------------------------------------------------

  constant CFG_DBG_INSN_TRACE_CPU_MASK : integer := 16#0#;
  constant CFG_DBG_LSU_TRACE           : boolean := false;
  constant CFG_DBG_BUS_TRACE           : boolean := false;
  constant CFG_DBG_TRAM_TRACE          : boolean := false;
  constant CFG_DBG_BRAM_TRACE          : boolean := false;

  ------------------------------------------------------
  -- ParaNut options
  ------------------------------------------------------

  -- Number of CPU cores (ld)
  constant CFG_NUT_CPU_CORES_LD     : natural := {CFG_NUT_CPU_CORES_LD};

  -- Do not edit
  constant CFG_NUT_CPU_CORES        : natural := 2 ** CFG_NUT_CPU_CORES_LD;

  -- Number of cores with mode capability 1
  constant CFG_NUT_CPU_MODE1_CORES  : natural := {CFG_NUT_CPU_MODE1_CORES};

   -- Number of cores with mode capability 2
  constant CFG_NUT_CPU_MODE2_CORES  : natural := (CFG_NUT_CPU_CORES)-{CFG_NUT_CPU_MODE1_CORES};

  -- - For simulation, values too high (e.g. > 8MB) may result in a non-working simulation for various simulation tools
  -- - For synthesis, this must exactly match your board's main memory size
  constant CFG_NUT_MEM_SIZE         : natural := {CFG_NUT_MEM_SIZE};

  -- Number of external interrupt lines (needs to be >= 1)
  constant CFG_NUT_EX_INT           : natural := {CFG_NUT_EX_INT};

  -- Bus Interface is Little/Big Endian
  constant CFG_NUT_LITTLE_ENDIAN    : boolean := true;

  -- Generate histograms for evaluation
  constant CFG_NUT_HISTOGRAM        : boolean := false;


  ------------------------------------------------------
  -- MEMU
  ------------------------------------------------------

  -- Number of cache banks (ld)
  constant CFG_MEMU_CACHE_BANKS_LD     : natural range 1 to 8 := {CFG_MEMU_CACHE_BANKS_LD};
  -- Number of cache sets (ld)
  constant CFG_MEMU_CACHE_SETS_LD      : natural range 1 to 13 := {CFG_MEMU_CACHE_SETS_LD};
  -- Number of cache ways (ld)
  constant CFG_MEMU_CACHE_WAYS_LD      : natural range 0 to 2 := {CFG_MEMU_CACHE_WAYS_LD};
  -- Number of ports per bank. Since every bank is implemented as one Block
  -- RAM, for maximum performance this should be set to the maximum number of
  -- Block RAM ports supported by the target device (i.e. 2) but can be
  -- reduced in order to save some area.
  constant CFG_MEMU_BANK_RAM_PORTS     : natural range 1 to 2 := 2;

  -- 0 : random replacement, 1 : LRU replacement
  constant CFG_MEMU_CACHE_REPLACE_LRU  : natural range 0 to 1 := {CFG_MEMU_CACHE_REPLACE_LRU};

  -- > 0: round-robin arbitration, switches every (1 << CFG_MEMU_ARBITER_METHOD) clocks
  -- < 0: pseudo-random arbitration (LFSR-based)
  constant CFG_MEMU_ARBITER_METHOD     : integer range -1 to 15 := {CFG_MEMU_ARBITER_METHOD};

  -- Width of Busif data
  -- 64 - 64Bit data width
  -- 32 - 32Bit data width
  constant CFG_MEMU_BUSIF_WIDTH        : natural := {CFG_MEMU_BUSIF_WIDTH};

  -- Do not edit
  constant CFG_MEMU_CACHE_BANKS        : natural := 2 ** CFG_MEMU_CACHE_BANKS_LD;
  constant CFG_MEMU_CACHE_SETS         : natural := 2 ** CFG_MEMU_CACHE_SETS_LD;
  constant CFG_MEMU_CACHE_WAYS         : natural := 2 ** CFG_MEMU_CACHE_WAYS_LD;
  constant CFG_MEMU_CACHE_SIZE         : natural := CFG_MEMU_CACHE_SETS * CFG_MEMU_CACHE_WAYS * CFG_MEMU_CACHE_BANKS * 4;

  ------------------------------------------------------
  -- IFU
  ------------------------------------------------------

  -- Build bit-slice implementation of IFU
  constant CFG_IFU_BS_IMPL       : boolean := false;
  -- Size of instruction buffer (must be at least 4)
  -- (Note: The normal implementation of the IFU only supports buffer sizes
  -- in powers of two. If CFG_IFU_IBUF_SIZE is not a power of two a buffer
  -- with size 2**ceil(log2(CFG_IFU_IBUF_SIZE)) will be generated
  constant CFG_IFU_IBUF_SIZE_LD  : natural range 2 to 16 := {CFG_IFU_IBUF_SIZE_LD};
  constant CFG_IFU_IBUF_SIZE     : natural := 2 ** CFG_IFU_IBUF_SIZE_LD;

  ------------------------------------------------------
  -- LSU
  ------------------------------------------------------

  -- Build a simple LSU with no write buffer
  constant CFG_LSU_SIMPLE        : boolean := false;
  -- Size of write buffer (ld)
  constant CFG_LSU_WBUF_SIZE_LD  : natural := {CFG_LSU_WBUF_SIZE_LD};

  ------------------------------------------------------
  -- EXU
  ------------------------------------------------------

  -- 0 : serial shift, 1 : generic shifter, 2 barrell shifter
  --constant CFG_EXU_SHIFT_IMPL       : integer range 0 to 2 := 2;
  -- Number of pipeline stages for embedded multiplier
  --constant CFG_EXU_MUL_PIPE_STAGES  : integer range 1 to 5 := 3;

end package;
