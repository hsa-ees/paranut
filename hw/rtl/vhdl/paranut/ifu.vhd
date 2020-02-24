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
--  Component and type declarations for the mifu module
--
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;

library paranut;
use paranut.paranut_config.all;
use paranut.types.all;
use paranut.memu_lib.all;
use paranut.histogram.all;

package ifu is

    type ifu_in_type is record
        nexti       : std_logic;
        jump        : std_logic;
        -- (next, jump) = (1, 1) lets the (current + 2)'th instruction be the jump target.
        -- Logically, 'next' is performed before 'jump'. Hence, jump instructions may either sequentially first
        -- assert 'next' and then 'jump' or both signals in the same cycle. The former way is required for JAL instructions
        -- to get the right return address, which is PC+8 (or NPC+4).
        jump_adr    : TWord;
        flush 		:  std_logic;
        -- Histogram...
        --hist_enable : std_logic;
    end record;
    type ifu_in_vector is array (natural range <>) of ifu_in_type;

    type ifu_out_type is record
        ir            : TWord; -- registered outputs
        pc            : TWord;
        npc           : TWord;
        ir_valid      : std_logic;
        npc_valid     : std_logic;
        -- Histogram...
        --buf_fill_hist : TWord_Vec(0 to CFG_IFU_IBUF_SIZE+1);
        --hist_ctrl     : hist_ctrl_type;
    end record;
    type ifu_out_vector is array (natural range <>) of ifu_out_type;

    component mifu_wrapper
        port (
                 clk            : in std_logic;
                 reset          : in std_logic;
                 -- to EXU...
                 ifui           : in ifu_in_type;
                 ifuo           : out ifu_out_type;
                 -- to MEMU (read port)...
                 rpi            : out readport_in_type;
                 rpo            : in readport_out_type;
                 -- from CePU...
                 icache_enable   : in std_logic
             );
    end component;



end package;
