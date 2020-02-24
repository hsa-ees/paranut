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
--  This file simulates the behaviour of read and write ports, so there is no
--  need to the MEMU and a peripheral bus with memory system.
--
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.paranut_config.all;
use work.types.all;
use work.memu_lib.all;
use work.paranut_lib.all;
use work.prog_mem.all;

entity rwports_sim is
    port (
             clk : in std_logic;
             reset : in std_logic;
             ifurpi    : in readport_in_type;
             ifurpo    : out readport_out_type;
             lsurpi    : in readport_in_type;
             lsurpo    : out readport_out_type;
             lsuwpi    : in writeport_in_type;
             lsuwpo    : out writeport_out_type
         );
end rwports_sim;

architecture rtl of rwports_sim is

    constant ifurcntdelay : unsigned (4 downto 0) := "00001";
    constant lsurcntdelay : unsigned (4 downto 0) := "10000";
    constant lsuwcntdelay : unsigned (4 downto 0) := "10000";
    signal ifurcnt : unsigned(4 downto 0) := "00000";
    signal lsurcnt : unsigned(4 downto 0) := "00000";
    signal lsuwcnt : unsigned(4 downto 0) := "00000";

    shared variable mem : mem_type(0 to CFG_NUT_MEM_SIZE/4-1) := (others => X"00000000");

begin


    process
    begin
        mem(0 to PROG_DATA'high) := PROG_DATA;
        wait;
    end process;

    comb : process (ifurcnt, lsurcnt, lsuwcnt)
    begin
        ifurpo.port_ack <= '0';
        lsurpo.port_ack <= '0';
        lsuwpo.port_ack <= '0';
        if (ifurcnt = ifurcntdelay) then
            ifurpo.port_ack <= '1';
        end if;
        if (lsurcnt = lsurcntdelay) then
            lsurpo.port_ack <= '1';
        end if;
        if (lsuwcnt = lsuwcntdelay) then
            lsuwpo.port_ack <= '1';
        end if;
    end process;

    process (clk)
    begin
        if (clk'event and clk='1') then
            if (ifurpi.port_rd = '1') then
                ifurcnt <= ifurcnt + "00001";
            end if;
            if (ifurcnt = ifurcntdelay) then
                ifurpo.port_data <= (others => '0');
                for i in 0 to 3 loop
                    if (ifurpi.port_bsel(i) = '1') then
                        ifurpo.port_data(31-8*i downto 24-8*i) <=
                        mem(conv_integer(ifurpi.port_adr)/4)(31-8*i downto 24-8*i);
                    end if;
                end loop;
                ifurcnt <= (others => '0');
            end if;

            if (lsurpi.port_rd = '1') then
                lsurcnt <= lsurcnt + "00001";
            end if;
            if (lsurcnt = lsurcntdelay) then
                lsurpo.port_data <= (others => '0');
                for i in 0 to 3 loop
                    if (lsurpi.port_bsel(i) = '1') then
                        lsurpo.port_data(31-8*i downto 24-8*i) <=
                        mem(conv_integer(lsurpi.port_adr)/4)(31-8*i downto 24-8*i);
                    end if;
                end loop;
                lsurcnt <= (others => '0');
            end if;

            if (lsuwpi.port_wr = '1') then
                lsuwcnt <= lsuwcnt + "00001";
            end if;
            if (lsuwcnt = lsuwcntdelay) then
                for i in 0 to 3 loop
                    if (lsuwpi.port_bsel(i) = '1') then
                        mem(conv_integer(lsuwpi.port_adr)/4)(31-8*i downto 24-8*i) := lsuwpi.port_data(31-8*i downto 24-8*i);
                    end if;
                end loop;
                lsuwcnt <= (others => '0');
            end if;
        end if;
    end process;

end rtl;
