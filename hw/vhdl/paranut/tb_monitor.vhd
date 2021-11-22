--------------------------------------------------------------------------------
-- This file is part of the ParaNut project.
-- 
-- Copyright (C) 2013  Michael Seider, <michael.seider@hs-augsburg.de>
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
--  Testbench monitor. Useful for simulation only. Provides global
--  synchronisation mechanisms used to control simulation from various parts of
--  the model.
--  Component and type declarations for simulation helpers.
--
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;

use std.textio.all;

library paranut;
use paranut.paranut_config.all;
use paranut.types.all;

package tb_monitor is

    type monitor_type is record
        halted : boolean;
        insn_issued : boolean;
        regfile : TWord_Vec(0 to 31);
        sr : TWord;
    end record;
    type monitor_vector is array (0 to CFG_NUT_CPU_CORES-1) of monitor_type;
    signal monitor : monitor_vector;

    signal sim_halt : boolean;

    shared variable tty : line;

    component orbis32_disas is
        generic (
                    CPU_ID : integer := 0
                );
        port (
                 clk : in std_logic;
                 insn : in TWord;
                 pc : in TWord
             );
    end component;

    component or1ksim_putchar is
        generic (
                    CPU_ID : integer := 0
                );
        port (
                 clk : in std_logic;
                 insn : in TWord
             );
    end component;

    component uart_putchar is
        port (
                 clk : in std_logic;
                 ifinished : in std_logic;
                 data : in std_logic_vector(7 downto 0)
             );
    end component;

end package;
