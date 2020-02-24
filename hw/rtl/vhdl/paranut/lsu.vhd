--------------------------------------------------------------------------------
-- This file is part of the ParaNut project.
-- 
-- Copyright (C) 2013-2019  Alexander Bahle <alexander.bahle@hs-augsburg.de>
-- 							Michael Seider, <michael.seider@hs-augsburg.de>
-- 		 Hochschule Augsburg, University of Applied Sciences
--
-- Redistribution and use in source and binary forms, with or without modification,
-- are permitted provided that the following conditions are met:
--
-- 1. Redistributions of source code must retain the above copyright notice, this 
--    list of conditions and the following disclaimer.
--
-- 2. Redistributions in binary form must reproduce the above copyright notice,
--    this list of conditions and the following disclaimer in the documentation and/or
--    other materials provided with the distribution.
--
-- THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND 
-- ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
-- WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE 
-- DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
-- ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES 
-- (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; 
-- LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON 
-- ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
-- (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS 
-- SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
--
-- Description:
--  Component and type declarations for the mlsu and mlsu_simple modules
--
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;

library paranut;
use paranut.paranut_config.all;
use paranut.types.all;
use paranut.memu_lib.all;

package lsu is

    type lsu_in_type is record
        rd               : std_logic;
        wr               : std_logic;
        flush            : std_logic;
        cache_writeback  : std_logic;
        cache_invalidate : std_logic;
        lres_scond       : std_logic;
        width            : TLSUWidth;
        exts             : std_logic;
        adr              : TWord;
        wdata            : TWord;
        -- Histogram...
        --hist_enable      : std_logic;
    end record;
    type lsu_in_vector is array (natural range <>) of lsu_in_type;

    type lsu_out_type is record
        ack           : std_logic;
        align_err     : std_logic;
        scond_ok      : std_logic;
        rdata         : TWord;
        -- Histogram...
        --buf_fill_hist : TWord_Vec(0 to 2**CFG_LSU_WBUF_SIZE_LD+1);
    end record;
    type lsu_out_vector is array (natural range <>) of lsu_out_type;

    component mlsu_wrapper
        port (
                 clk            : in std_logic;
                 reset          : in std_logic;
                 -- to EXU...
                 lsui           : in lsu_in_type;
                 lsuo           : out lsu_out_type;
                 -- to MEMU/read port...
                 rpi            : out readport_in_type;
                 rpo            : in readport_out_type;
                 -- to MEMU/write port...
                 wpi            : out writeport_in_type;
                 wpo            : in writeport_out_type;
				 -- from CePU
				 dcache_enable  : in std_logic
             );
    end component;
    
    component mlsu_simple is
		port (
             clk            : in std_logic;
             reset          : in std_logic;
             -- to EXU...
             lsui           : in lsu_in_type;
             lsuo           : out lsu_out_type;
             -- to MEMU/read port...
             rpi            : out readport_in_type;
             rpo            : in readport_out_type;
             -- to MEMU/write port...
             wpi            : out writeport_in_type;
             wpo            : in writeport_out_type;
             -- from CePU
             dcache_enable  : in std_logic
         );
	end component;

end package;
