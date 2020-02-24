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
--  Component and type declarations for the mmemu module (see memu_lib.vhd for
--  readport, writeport, and busif type declarations)
--
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;

library paranut;
use paranut.memu_lib.all;
use paranut.histogram.all;

package memu is

    type memu_hist_ctrl_type is record
        cache_line_fill  : hist_ctrl_type;
        cache_line_wb    : hist_ctrl_type;
        cache_read_hit   : hist_ctrl_vector(0 to RPORTS-1);
        cache_read_miss  : hist_ctrl_vector(0 to RPORTS-1);
        cache_write_hit  : hist_ctrl_vector(0 to WPORTS-1);
        cache_write_miss : hist_ctrl_vector(0 to WPORTS-1);
    end record;

    type memu_in_type is record
        -- Bus interface (Wishbone)...
        bifwbi : busif_wishbone_in_type;
        -- Read ports...
        -- ports 0 .. WPORT-1 are considered to be data ports, the others to be instruction ports (with lower priority)
        rpi    : readport_in_vector(0 to RPORTS-1);
        -- Write ports...
        wpi    : writeport_in_vector(0 to WPORTS-1);
    end record;

    type memu_out_type is record
        bifwbo : busif_wishbone_out_type;
        rpo    : readport_out_vector(0 to RPORTS-1);
        wpo    : writeport_out_vector(0 to WPORTS-1);
        mhco   : memu_hist_ctrl_type;
    end record;

    component mmemu
        --generic (
        --            CFG_MEMU_CACHE_BANKS : integer := 1;
        --            RPORTS      : integer := 2;
        --            WPORTS      : integer := 1
        --        );
        port (
                 clk    : in std_logic;
                 reset  : in std_logic;
                 mi     : in memu_in_type;
                 mo     : out memu_out_type
             );
    end component;

end package;
