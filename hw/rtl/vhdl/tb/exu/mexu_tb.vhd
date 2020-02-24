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
--  EXU testbench. Executes the program found in prog_mem.vhd that is found in
--  this folder.
--
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;

library paranut;
use paranut.types.all;
use paranut.exu.all;
use paranut.lsu.all;
use paranut.ifu.all;
use paranut.memu_lib.all;
use paranut.tb_monitor.all;

entity mexu_tb is
    generic (MIFU_BS_IMPL : boolean := false);
end mexu_tb;

architecture tb of mexu_tb is

    signal clk           : std_logic;
    signal reset         : std_logic;
    signal ifui          : ifu_in_type;
    signal ifuo          : ifu_out_type;
    signal lsui          : lsu_in_type;
    signal lsuo          : lsu_out_type;
    signal icache_enable : std_logic;
    signal dcache_enable : std_logic;
    signal emhci : exu_memu_hist_ctrl_in_type;

    signal ifurpi : readport_in_type;
    signal ifurpo : readport_out_type;

    signal lsurpi : readport_in_type;
    signal lsurpo : readport_out_type;
    signal lsuwpi : writeport_in_type;
    signal lsuwpo : writeport_out_type;

    constant clk_period : time := 10 ns;

    component rwports_sim is
        port (
                 clk    : in std_logic;
                 reset  : in std_logic;
                 ifurpi : in readport_in_type;
                 ifurpo : out readport_out_type;
                 lsurpi : in readport_in_type;
                 lsurpo : out readport_out_type;
                 lsuwpi : in writeport_in_type;
                 lsuwpo : out writeport_out_type
             );
    end component;

begin

    uut : mexu
    port map (clk, reset, ifui, ifuo, lsui, lsuo, icache_enable, dcache_enable, '0', emhci);

    gen_mifu_bs : if MIFU_BS_IMPL generate
        ifu : mifu_bs
        port map (clk, reset, ifui, ifuo, ifurpi, ifurpo, icache_enable);
    end generate;
    gen_mifu : if not MIFU_BS_IMPL generate
        ifu : mifu
        port map (clk, reset, ifui, ifuo, ifurpi, ifurpo, icache_enable);
    end generate;


    lsu : mlsu
    port map (clk, reset, lsui, lsuo, lsurpi, lsurpo, lsuwpi, lsuwpo, dcache_enable);

    rwps : rwports_sim
    port map (clk, reset, ifurpi, ifurpo, lsurpi, lsurpo, lsuwpi, lsuwpo);

    clock: process
    begin
        while (not sim_halt) loop
            clk <= '1'; wait for clk_period/2;
            clk <= '0'; wait for clk_period/2;
        end loop;
        wait;
    end process;

    tb: process

    begin

        reset <= '1';
        wait for 5*clk_period;

        reset <= '0';
        wait until monitor(0).halted;

        wait for 60*clk_period;

        sim_halt <= true;
        report "Simulation finished";
        wait;

    end process;

end tb;
