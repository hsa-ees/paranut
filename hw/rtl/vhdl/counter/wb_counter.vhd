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
--  Wishbone interface to the counter module
--
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library paranut;
use paranut.counter_pkg.all;

entity wb_counter is
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
end wb_counter;

architecture rtl of wb_counter is

    type std_logic_vector2 is array (natural range <>) of std_logic_vector(31 downto 0);

    type registers is record
        ctrl  : std_logic_vector2(0 to N_COUNTER-1);
        div   : std_logic_vector2(0 to N_COUNTER-1);
        ack_o : std_logic;
        dat_o : std_logic_vector(31 downto 0);
    end record;

    signal r, rin : registers;

    signal s_cnt  : std_logic_vector2(0 to N_COUNTER-1);

    constant RESET_BIT  : integer := 0;
    constant ENABLE_BIT : integer := 1;

begin

    wb_err_o <= '0';
    wb_rty_o <= '0';
    wb_ack_o <= r.ack_o;
    wb_dat_o <= r.dat_o;

    counters : for i in 0 to N_COUNTER-1 generate
        counter : counter_top
        port map (
                     clk => wb_clk,
                     reset => r.ctrl(i)(RESET_BIT),
                     enable => r.ctrl(i)(ENABLE_BIT),
                     cnt_div => r.div(i),
                     cnt_out => s_cnt(i)
                 );
    end generate;

    comb : process (wb_rst, wb_dat_i, wb_stb_i, wb_we_i, wb_sel_i, wb_adr_i)
        variable v : registers;
        variable cntr_adr : std_logic_vector(3 downto 0);
        variable reg_adr : std_logic_vector(1 downto 0);
    begin

        v := r;

        cntr_adr := wb_adr_i(7 downto 4);
        reg_adr := wb_adr_i(3 downto 2);

        -- auto-clear reset bits
        for i in 0 to N_COUNTER-1 loop
            v.ctrl(i)(RESET_BIT) := '0';
        end loop;

        if (wb_stb_i = '1') then
            if (wb_we_i = '1') then
                if (reg_adr = "00") then
                    v.ctrl(to_integer(unsigned(cntr_adr))) := wb_dat_i;
                elsif (reg_adr = "01") then
                    v.div(to_integer(unsigned(cntr_adr))) := wb_dat_i;
                end if;
            else
                -- 'dat_o' generation
                v.dat_o := (others => '0');
                if (reg_adr = "00") then
                    v.dat_o := r.ctrl(to_integer(unsigned(cntr_adr)));
                elsif (reg_adr = "01") then
                    v.dat_o := r.div(to_integer(unsigned(cntr_adr)));
                elsif (reg_adr = "10") then
                    v.dat_o := s_cnt(to_integer(unsigned(cntr_adr)));
                end if;
            end if;
        end if;

        if (wb_rst = '1') then
            for i in 0 to N_COUNTER-1 loop
                v.ctrl(i)(RESET_BIT) := ('1');
                v.ctrl(i)(ENABLE_BIT) := ('0');
                v.div := (others => (others => '0'));
            end loop;
        end if;

        -- 'ack_o' generation
        if (wb_rst = '1') then
            v.ack_o := '0';
        elsif (r.ack_o = '1') then
            v.ack_o := '0';
        elsif (wb_stb_i = '1' and r.ack_o = '0') then
            v.ack_o := '1';
        end if;

        rin <= v;

    end process;

    clock : process(wb_clk)
    begin
        if (wb_clk'event and wb_clk = '1') then
            r <= rin;
        end if;
    end process;

    --with wb_adr_i(1 downto 0) select
    --    wb_dat_o <= r.ctrl when "00",
    --                r.div  when "01",
    --                s_cnt  when "10",
    --                (others => '0') when others;
    
end rtl;
