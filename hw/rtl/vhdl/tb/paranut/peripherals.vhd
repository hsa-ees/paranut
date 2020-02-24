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
--  Component and type declarations for peripheral module for the testbench
--  (wishbone memory and wishbone uart).
--
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;

library paranut;
use paranut.paranut_config.all;
use paranut.types.all;

package peripherals is

    component wb_memory is
        generic (
                    WB_SLV_ADDR : natural := 16#00#;
                    CFG_NUT_MEM_SIZE : natural := 32 * 1024 * 1024;
                    LOAD_PROG_DATA : boolean := true;
                    PROG_DATA : mem_type
                );
        port (
                 -- Ports (WISHBONE slave)
                 clk_i   : in std_logic;
                 rst_i   : in std_logic;
                 stb_i   : in std_logic;                    -- strobe output
                 cyc_i   : in std_logic;                    -- cycle valid output
                 we_i    : in std_logic;                    -- indicates write transfer
                 sel_i   : in std_logic_vector((CFG_MEMU_BUSIF_WIDTH/8)-1 downto 0);  -- byte select outputs
                 ack_o   : out std_logic;                   -- normal termination
                 err_o   : out std_logic;                   -- termination w/ error
                 rty_o   : out std_logic;                   -- termination w/ retry
                 adr_i   : in TWord;                        -- address bus outputs
                 dat_i   : in std_logic_vector(CFG_MEMU_BUSIF_WIDTH-1 downto 0); -- input data bus
                 dat_o   : out std_logic_vector(CFG_MEMU_BUSIF_WIDTH-1 downto 0); -- output data bus
                 cti_i   : in std_logic_vector(2 downto 0); -- cycle type identifier
                 bte_i   : in std_logic_vector(1 downto 0)  -- burst type extension
             );
    end component;

    component wb_uart is
        generic (
                    WB_SLV_ADDR : natural := 16#90#
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
    end component;

    component wb_counter_wrapper is
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
    end component;

end package;
