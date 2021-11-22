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
--  Bank RAM module for a single bank
--
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;

library paranut;
use paranut.paranut_config.all;
use paranut.types.all;
use paranut.memu_lib.all;
use paranut.mem_tech.all;

entity mbankram is
    port (
             clk : in std_logic;
             bri : in bankram_in_type;
             bro : out bankram_out_type
         );
end mbankram;

architecture rtl of mbankram is

begin
    -- Blockram is configured to be READ_FIRST for compatibility reasons
    bram_gen_1p : if CFG_MEMU_BANK_RAM_PORTS = 1 generate
        bankram : mem_sync_sp_inferred
        generic map (AWIDTH => CFG_MEMU_CACHE_SETS_LD+CFG_MEMU_CACHE_WAYS_LD,
                     DWIDTH => 32, WRITE_MODE => READ_FIRST)
        port map (clk, bri.wiadr(0), bri.wr(0), bri.wen(0), bri.wdata(0), bro.rdata(0));
    end generate;

    -- Blockram is configured to be READ_FIRST for compatibility reasons
    bram_gen_2p : if CFG_MEMU_BANK_RAM_PORTS = 2 generate
        bankram : mem_sync_true_dp_inferred
        generic map (AWIDTH => CFG_MEMU_CACHE_SETS_LD+CFG_MEMU_CACHE_WAYS_LD,
                     DWIDTH => 32, WRITE_MODE_1 => READ_FIRST, WRITE_MODE_2 =>
                     READ_FIRST)
        port map (clk, clk, bri.wiadr(0), bri.wiadr(CFG_MEMU_BANK_RAM_PORTS-1), bri.wr(0),
        bri.wr(CFG_MEMU_BANK_RAM_PORTS-1), bri.wen(0), bri.wen(CFG_MEMU_BANK_RAM_PORTS-1), 
        bri.wdata(0), bri.wdata(CFG_MEMU_BANK_RAM_PORTS-1), bro.rdata(0),
        bro.rdata(CFG_MEMU_BANK_RAM_PORTS-1));
    end generate;

end rtl;
