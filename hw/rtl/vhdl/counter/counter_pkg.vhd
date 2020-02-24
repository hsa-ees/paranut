--------------------------------------------------------------------------------
-- This file is part of the ParaNut project.
-- 
-- Copyright (C) 2014  Michael Seider, <michael.seider@hs-augsburg.de>
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
--  Component and type declarations for the counter module
--
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;

package counter_pkg is

    component wb_counter is
        generic (N_COUNTER : integer range 1 to 16 := 1);
        port (
                 -- Ports (WISHBONE slave)
                 wb_clk     : in std_logic;
                 wb_rst     : in std_logic;
                 wb_ack_o   : out std_logic;                    -- normal termination
                 wb_err_o   : out std_logic;                    -- termination w/ error
                 wb_rty_o   : out std_logic;                    -- termination w/ retry
                 wb_dat_i   : in std_logic_vector(31 downto 0); -- input data bus
                 wb_cyc_i   : in std_logic;                     -- cycle valid input
                 wb_stb_i   : in std_logic;                     -- strobe input
                 wb_we_i    : in std_logic;                     -- indicates write transfer
                 wb_sel_i   : in std_logic_vector(3 downto 0);  -- byte select input
                 wb_adr_i   : in std_logic_vector(31 downto 0); -- address bus input
                 wb_dat_o   : out std_logic_vector(31 downto 0) -- data bus output
             );
    end component;

    component counter_top is
        port (
                 clk: in std_logic;
                 reset : in std_logic;
                 enable : in std_logic;
                 cnt_div : in std_logic_vector(31 downto 0);
                 cnt_out : out std_logic_vector(31 downto 0)
             );
    end component;

end package;
