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
--  Wishbone interface wrapper for the Counter module used for simulation. 
--
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library paranut;
use paranut.types.all;
use paranut.paranut_lib.all;
use paranut.counter_pkg.all;

entity wb_counter_wrapper is
    generic (
                WB_SLV_ADDR : natural := 16#F0#;
                N_COUNTER : natural := 4
            );
    port (
             -- Ports (WISHBONE slave)
             clk_i   : in std_logic;
             rst_i   : in std_logic;
             stb_i   : in std_logic;    -- strobe output
             cyc_i   : in std_logic;    -- cycle valid output
             we_i    : in std_logic;    -- indicates write transfer
             sel_i   : in TByteSel;     -- byte select outputs
             ack_o   : out std_logic;   -- normal termination
             err_o   : out std_logic;   -- termination w/ error
             rty_o   : out std_logic;   -- termination w/ retry
             adr_i   : in TWord;        -- address bus outputs
             dat_i   : in TWord;        -- input data bus
             dat_o   : out TWord        -- outout data bus
         );
end wb_counter_wrapper;

architecture behav of wb_counter_wrapper is

    signal ctr_stb_i : std_logic;
    signal ctr_cyc_i : std_logic;
    signal ctr_we_i  : std_logic;
    signal ctr_sel_i : TByteSel;
    signal ctr_ack_o : std_logic;
    signal ctr_err_o : std_logic;
    signal ctr_rty_o : std_logic;
    signal ctr_adr_i : TWord;
    signal ctr_dat_i : TWord;
    signal ctr_dat_o : TWord;

begin

    ctr_dat_i <= dat_i;
    ctr_cyc_i <= cyc_i;
    ctr_sel_i <= "0000";
    ctr_adr_i <= adr_i;

    ctr_stb_i <= stb_i when conv_integer(adr_i(31 downto 20)) = WB_SLV_ADDR else
                 '0';
    ctr_we_i <= we_i when conv_integer(adr_i(31 downto 20)) = WB_SLV_ADDR else
                '0';
    ack_o <= ctr_ack_o when conv_integer(adr_i(31 downto 20)) = WB_SLV_ADDR else
             'Z';
    dat_o <= ctr_dat_o when conv_integer(adr_i(31 downto 20)) = WB_SLV_ADDR else
             (others => 'Z');

    counter0 : wb_counter
    generic map (N_COUNTER => N_COUNTER)
    port map (clk_i, rst_i, ctr_ack_o, ctr_err_o, ctr_rty_o, ctr_dat_i, ctr_cyc_i,
    ctr_stb_i, ctr_we_i, ctr_sel_i, ctr_adr_i, ctr_dat_o);

end behav;

