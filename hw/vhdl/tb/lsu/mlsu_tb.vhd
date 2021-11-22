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
--  LSU testbench.
--
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;

library work;
use work.types.all;
use work.memu_lib.all;
use work.lsu.all;

entity mlsu_tb is
    end mlsu_tb;

architecture tb of mlsu_tb is

             signal clk    : std_logic;
             signal reset  : std_logic;

             signal lsui   : lsu_in_type;
             signal lsuo   : lsu_out_type;

             signal rpi    : readport_in_type;
             signal rpo    : readport_out_type;

             signal wpi    : writeport_in_type;
             signal wpo    : writeport_out_type;
             signal dcache_enable : std_logic;

    constant clk_period : time := 10 ns;

    signal halt: boolean := false;

begin

    uut: mlsu
    port map (clk, reset, lsui, lsuo, rpi, rpo, wpi, wpo, dcache_enable);

    clock: process
    begin
        while (not halt) loop
            clk <= '1'; wait for clk_period/2;
            clk <= '0'; wait for clk_period/2;
        end loop;
        wait;
    end process;

    tb: process

        procedure write(adr, data : TWord; width : std_logic_vector(1 downto 0)) is
        begin
            lsui.wr <= '1';
            lsui.adr <= adr;
            lsui.wdata <= data;
            lsui.width <= width;
            wait for clk_period;
            lsui.wr <= '0';
        end procedure;

    begin

        reset <= '1';
        lsui.rd <= '0';
        lsui.wr <= '0';
        lsui.cache_invalidate <= '0';
        lsui.cache_writeback <= '0';
        lsui.rlink_wcond <= '0';
        lsui.flush <= '0';
        lsui.exts <= '0';
        dcache_enable <= '0';
        --wait for clk_period/8;
        wait for 5*clk_period;

        reset <= '0';
        wait for clk_period;

        write(X"00003100", X"01234567", "00");
        write(X"00007200", X"89abcdef", "00");
        write(X"00027200", X"091b2d3f", "00");
        write(X"00000000", X"11111111", "00");
        write(X"00000001", X"22222222", "01");
        write(X"00000002", X"33333333", "10");
        write(X"00000004", X"44444444", "00");

        wpo.port_ack <= '1';
        write(X"00000004", X"44444444", "00");
        wait for 4*clk_period;
        wpo.port_ack <= '0';

        write(X"90000000", X"00000000", "00");
        write(X"90000002", X"11111111", "10");
        wait for 2*clk_period;
        write(X"90000000", X"22222222", "10");
        write(X"90000000", X"22222222", "10");
        wpo.port_ack <= '1';
        write(X"90000000", X"22222222", "10");
        wpo.port_ack <= '0';
        write(X"90000000", X"22222222", "10");
        write(X"90000000", X"22222222", "10");
        write(X"90000000", X"22222222", "10");
        wait for 2*clk_period;
        wpo.port_ack <= '1';
        wait for clk_period;
        wpo.port_ack <= '1';
        wait for clk_period;

        halt <= true;
        report "Simulation halted";
        wait;

    end process;

end tb;
