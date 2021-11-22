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
--  Component and type declarations for the ParaNut top level module.
--
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;

library paranut;
use paranut.paranut_config.all;
use paranut.memu_lib.all;
use paranut.types.all;

package paranut_pkg is

    component mparanut
        generic (
        --            CFG_NUT_CPU_CORES   : integer := 1;
        --            CFG_MEMU_CACHE_BANKS : integer := 1
					CLK_FREQ_HZ : integer := 100_000_000
                );
        port (
                 -- Ports (WISHBONE master)
                 clk_i    : in std_logic;
                 rst_i    : in std_logic;
                 ack_i    : in std_logic;                     -- normal termination
                 err_i    : in std_logic;                     -- termination w/ error
                 rty_i    : in std_logic;                     -- termination w/ retry
                 dat_i    : in std_logic_vector(CFG_MEMU_BUSIF_WIDTH-1 downto 0);  -- input data bus
                 cyc_o    : out std_logic;                    -- cycle valid output
                 stb_o    : out std_logic;                    -- strobe output
                 we_o     : out std_logic;                    -- indicates write transfer
                 sel_o    : out std_logic_vector((CFG_MEMU_BUSIF_WIDTH/8)-1 downto 0); -- byte select outputs
                 adr_o    : out TWord;                        -- address bus outputs
                 dat_o    : out std_logic_vector(CFG_MEMU_BUSIF_WIDTH-1 downto 0); -- output data bus
                 cti_o    : out std_logic_vector(2 downto 0); -- cycle type identifier
                 bte_o    : out std_logic_vector(1 downto 0); -- burst type extension
                 
				-- Other
                 du_stall : in std_logic;
                 ex_int   : in std_logic_vector(CFG_NUT_EX_INT-1 downto 0);

                 
				 -- JTAG 
				 tck	: in std_logic;
				 tms 	: in std_logic;
				 tdi 	: in std_logic;
				 tdo 	: out std_logic
             );
    end component;

end package;
