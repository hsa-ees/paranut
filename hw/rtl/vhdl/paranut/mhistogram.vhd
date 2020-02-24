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
--  mhistogram module: used for registering events on a per clock basis and
--  keeping book of min, max, total clock, total count values.
--  Also can generate a histogram of events with 0 to 2**HIST_BINS_LD-4 bins.
--  - events can be:
--   - started (begin counting clks for event)
--   - stopped (stop counting clks for event and update min, max, tot, clks, hist)
--   - aborted (abort counting clks for event)
--
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library paranut;
use paranut.types.all;
use paranut.histogram.all;
use paranut.mem_tech.all;

entity mhistogram is
    generic (
                HIST_BINS_LD : integer := 6;
                ADDR_WIDTH : integer := 6
            );
    port (
             clk : in std_logic;
             reset : in std_logic;
             hi : in hist_in_type;
             ho : out hist_out_type
         );
end mhistogram;

architecture rtl of mhistogram is

type registers is record
    state : std_logic;
    cnt : unsigned(31 downto 0);
    min : unsigned(31 downto 0);
    max : unsigned(31 downto 0);
    tot_cnt : unsigned(31 downto 0);
    tot_clks : unsigned(31 downto 0);
end record;

signal r, rin : registers;

signal mem_waddr, mem_raddr : std_logic_vector(HIST_BINS_LD-1 downto 0);
signal mem_wdata, mem_rdata : TWord;
signal mem_new_data : unsigned(31 downto 0);
signal mem_we : std_logic;
signal mem_rdata_user : TWord;

begin

    no_hist_mem_gen : if (HIST_BINS_LD = 0) generate
        ho.data <= std_logic_vector(r.min) when (hi.addr(1 downto 0) = "00") else
                   std_logic_vector(r.max) when (hi.addr(1 downto 0) = "01") else
                   std_logic_vector(r.tot_cnt) when (hi.addr(1 downto 0) = "10") else
                   std_logic_vector(r.tot_clks);
    end generate;
    hist_mem_gen : if (HIST_BINS_LD /= 0) generate
        ho.data <= mem_rdata_user when (hi.addr(ADDR_WIDTH-1 downto 2) /= ones64(ADDR_WIDTH-1 downto 2)) else
                   -- last four addresses address min, max, cnt, clks...
                   std_logic_vector(r.min) when (hi.addr(1 downto 0) = "00") else
                   std_logic_vector(r.max) when (hi.addr(1 downto 0) = "01") else
                   std_logic_vector(r.tot_cnt) when (hi.addr(1 downto 0) = "10") else
                   std_logic_vector(r.tot_clks);

        mem_waddr <= std_logic_vector(r.cnt(HIST_BINS_LD-1 downto 0)) when (r.cnt < HIST_BINS_LD**2) else
                     -- Put everything greater than biggest bin into last bin
                     ones64(HIST_BINS_LD-1 downto 0);
        mem_raddr <= std_logic_vector(r.cnt(HIST_BINS_LD-1 downto 0)) when (r.cnt < HIST_BINS_LD**2) else
                     -- Put everything greater than biggest bin into last bin
                     ones64(HIST_BINS_LD-1 downto 0);
        mem_new_data <= unsigned(mem_rdata) + 1;
        mem_wdata <= std_logic_vector(mem_new_data);

        -- async RAM used to store histogram
        hist_data : mem_3p_async_dist_rfirst_inferred
        generic map (AWIDTH => HIST_BINS_LD, DWIDTH => 32)
        port map (wclk => clk, waddr => mem_waddr, wdata => mem_wdata, we =>
            mem_we, raddr1 => mem_raddr, rdata1 => mem_rdata, raddr2 =>
                hi.addr(HIST_BINS_LD-1 downto 0), rdata2 => mem_rdata_user);
    end generate;

    comb : process(r, reset, hi)
        variable v : registers;
    begin
        v := r;

        mem_we <= '0';

        if (r.state = '1') then
            v.cnt := r.cnt + 1;
        end if;

        if ((hi.ctrl.start and hi.enable) = '1') then
            -- normal start event
            v.state := '1';
            v.cnt := r.cnt + 1;
        end if;
        if ((v.state and hi.enable) = '1') then
            if (hi.ctrl.abort = '1') then
                v.state := '0';
                v.cnt := (others => '0');
            elsif (hi.ctrl.stop = '1') then
                v.state := '0';
                v.cnt := (others => '0');
                mem_we <= '1';
                if (r.cnt < r.min) then v.min := r.cnt; end if;
                if (r.cnt > r.max) then v.max := r.cnt; end if;
                v.tot_cnt := r.tot_cnt + 1;
                v.tot_clks := r.tot_clks + r.cnt + 1;
                -- new start event?
                if (r.state = '1' and hi.ctrl.start = '1') then
                    v.state := '1';
                    v.cnt := (0 => '1', others => '0');
                end if;
            end if;
        end if;

        if (reset = '1') then
            v.state := '0';
            v.min := (others => '1');
            v.max := (others => '0');
            v.cnt := (others => '0');
            v.tot_cnt := (others => '0');
            v.tot_clks := (others => '0');
        end if;
        rin <= v;
    end process;

    process(clk)
    begin
        if (clk'event and clk = '1') then
            r <= rin;
        end if;
    end process;

end rtl;
