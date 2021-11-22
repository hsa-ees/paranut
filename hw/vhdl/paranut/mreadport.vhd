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
--  Readport module for the MEMU.
--
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;

library paranut;
use paranut.paranut_config.all;
use paranut.types.all;
use paranut.memu_lib.all;
use paranut.paranut_lib.all;
use paranut.histogram.all;

entity mreadport is
        port (
                 clk    : in std_logic;
                 reset  : in std_logic;
                 -- With (CPU) port...
                 rpi    : in readport_in_type;
                 rpo    : out readport_out_type;
                 -- With cache...
                 rpci   : in readport_cache_in_type;
                 rpco   : out readport_cache_out_type;
                 -- With BUSIF...
                 rpbifi : in readport_busif_in_type;
                 rpbifo : out readport_busif_out_type;
                 -- Request & grant lines...
                 rpai   : in readport_arbiter_in_type;
                 rpao   : out readport_arbiter_out_type;
                 -- Histogram...
                 hist_cache_read_hit  : out hist_ctrl_type;
                 hist_cache_read_miss : out hist_ctrl_type
             );
end mreadport;

architecture rtl of mreadport is
    -- - Address information for tag is routed around this module from 'port_adr', for bank from 'port_adr' AND 'way_out'.

    -- - All input ports must be held until 'port_ack' is asserted, at least for one cycle.
    -- - 'port_ack' is issued for exactly one cycle (no full handshake).
    -- - 'port_ack' is issued one cycle before the data is valid (pipelining).
    -- - The next request may be issued at earliest one cycle after 'port_ack' was asserted (which is the same cycle the data is delivered).
    -- - ('port_ack' may be asserted already in the same cycle as the request was issued.) -> minumum latency on cache hit is presently 2 clocks
    -- - If 'port_direct' is set, nothing happens with the cache (i.e. no invalidation).
    --   Hence, 'port_direct' = 0/1 should not be mixed for one address.
    -- - On a 'busif_hit' the next state must set "vrpo.port_data = rpbifi.busif_data" (which is the default for all states)

    type state_type is (S_RP_INIT, S_RP_DIRECT_WAIT_BUSIF,
    S_RP_REQUEST_TAG_ONLY, S_RP_READ_TAG, S_RP_READ_BANK, S_RP_MISS_WAIT_BUSIF,
    S_RP_MISS_REQUEST_TAG, S_RP_MISS_READ_TAG, S_RP_MISS_REPLACE);

    type registers is record
        state    : state_type;
        bank_sel : std_logic_vector(CFG_MEMU_CACHE_BANKS_LD-1 downto 0);
        link_adr : TWord;
        link_valid : std_logic;
    end record;

    signal r, rin : registers;

begin

    comb : process (r, rpi, rpci, rpbifi, rpai)
        variable v : registers;
        -- Output variables
        variable vrpo    : readport_out_type;
        variable vrpbifo : readport_busif_out_type;
        variable vrpco   : readport_cache_out_type;
        variable vrpao   : readport_arbiter_out_type;
        -- Helper variables...
        variable busif_hit          : std_logic;
        variable index              : std_logic_vector(CFG_MEMU_CACHE_SETS_LD-1 downto 0);
        variable bank               : std_logic_vector(CFG_MEMU_CACHE_BANKS_LD-1 downto 0);
        variable bank_req_rd        : std_logic;
        variable tagr_gnt, bank_gnt : std_logic;
        variable link_and_port_adr_eq : std_logic;
    begin

        v := r;

        hist_cache_read_hit.start <= '0';
        hist_cache_read_hit.stop <= '0';
        hist_cache_read_hit.abort <= '0';
        hist_cache_read_miss.start <= '0';
        hist_cache_read_miss.stop <= '0';
        hist_cache_read_miss.abort <= '0';
        
        if (r.link_adr = rpi.port_adr) then 
          link_and_port_adr_eq := '1';
        else 
          link_and_port_adr_eq := '0';
        end if;

        -- Default for outputs...
        vrpo.port_ack := '0';
        vrpo.port_data := rpbifi.busif_data;
        vrpo.port_scond_ok := r.link_valid and link_and_port_adr_eq;

        vrpao.req_busif := '0';
        vrpao.req_tagr := '0';
        vrpao.req_bank := (others => '0');

        vrpbifo.busif_op := BIO_NOTHING;

        vrpco.tag_rd := '0';
        vrpco.bank_sel := r.bank_sel;
        vrpco.way_out := rpci.tag_in.way; --TODO: need a register for this?

        -- Helper variables...
        if (CFG_MEMU_BUSIF_WIDTH = 64) then
          busif_hit := rpbifi.busif_data_valid(conv_integer(rpi.port_adr(BANK_OF_ADDR_RANGE)))
                        and conv_std_logic(rpi.port_adr(31 downto 3) = rpbifi.busif_adr(31 downto 3));
        else 
          busif_hit := rpbifi.busif_data_valid(conv_integer(rpi.port_adr(BANK_OF_ADDR_RANGE)))
                        and conv_std_logic(rpi.port_adr(31 downto 2) = rpbifi.busif_adr(31 downto 2));
        end if;
        index := rpi.port_adr(INDEX_OF_ADDR_RANGE);
        bank := rpi.port_adr(BANK_OF_ADDR_RANGE);
        bank_req_rd := '0';
        tagr_gnt := rpai.gnt_tagr;
        bank_gnt := rpai.gnt_bank(conv_integer(bank));
        
        case r.state is
            when S_RP_INIT =>
                -- Initial state: On new request, initiate all actions for the first cycle
                vrpbifo.busif_op := BIO_DIRECT_READ;
                if (rpi.port_rd = '1') then
                    if (rpi.port_direct = '1') then
                        -- uncached memory access...
                        vrpao.req_busif := '1';
                        v.state := S_RP_DIRECT_WAIT_BUSIF; 
                    else
                        hist_cache_read_hit.start <= '1';
                        hist_cache_read_miss.start <= '1';
                        -- cached memory access...
                        if (busif_hit = '1') then
                            hist_cache_read_hit.abort <= '1';
                            hist_cache_read_miss.abort <= '1';
                            vrpo.port_ack := '1';
                        else
                            vrpao.req_tagr := '1';
                            vrpco.tag_rd := '1';
                            if (tagr_gnt = '1') then                             
                                v.state := S_RP_READ_TAG;
                            end if;
                        end if;
                    end if;
                    if (rpi.port_lres_scond = '1') then
                        -- Link current address (LL operation)...
                        v.link_adr := rpi.port_adr;
                        v.link_valid := '1';
                    end if;
                end if;
            when S_RP_DIRECT_WAIT_BUSIF =>
                -- Direct access: Wait for response from BUSIF
                vrpbifo.busif_op := BIO_DIRECT_READ;
                vrpao.req_busif := not busif_hit;
                if (busif_hit = '1') then
                    vrpo.port_ack := '1';
                    v.state := S_RP_INIT;
                end if;
            when S_RP_READ_TAG =>
                -- Capture the tag (which was granted last state)
                -- keep req_tagr to hold tag output
                vrpao.req_tagr := '1';
                bank_req_rd := '1';
                if (rpci.tag_in.valid = '1') then
                    hist_cache_read_miss.abort <= '1';
                    -- Cache hit
                    if (bank_gnt = '1') then
                        hist_cache_read_hit.stop <= '1';
                        --vrpao.req_tagr := '0';
                        v.bank_sel := bank;
                        vrpo.port_ack := '1';
                        v.state := S_RP_READ_BANK;
                    end if;
                else
                    hist_cache_read_hit.abort <= '1';
                    v.state := S_RP_MISS_WAIT_BUSIF;
                end if;
            when S_RP_READ_BANK =>
                -- Read the bank & complete...
                vrpo.port_data := rpci.bank_data_in;
                v.state := S_RP_INIT;
                vrpbifo.busif_op := BIO_DIRECT_READ;
                
                -- On new request, initiate all actions for the first cycle
                if (rpi.port_rd = '1') then
                    if (rpi.port_direct = '1') then
                        -- uncached memory access...
                        vrpao.req_busif := '1';
                        v.state := S_RP_DIRECT_WAIT_BUSIF; 
                    else
                        hist_cache_read_hit.start <= '1';
                        hist_cache_read_miss.start <= '1';
                        -- cached memory access...
                        if (busif_hit = '1') then
                            hist_cache_read_hit.abort <= '1';
                            hist_cache_read_miss.abort <= '1';
                            vrpo.port_ack := '1';
                        else
                            vrpao.req_tagr := '1';
                            vrpco.tag_rd := '1';
                            if (tagr_gnt = '1') then
                                v.state := S_RP_READ_TAG;
                            end if;
                        end if;
                    end if;
                end if;
            when S_RP_MISS_WAIT_BUSIF =>
                -- Cache miss detected: Wait until we get the BusIF and the
                -- BusIF becomes idle and catch an incidental BusIF hit if it
                -- happens.
                vrpao.req_busif := '1';
                if (busif_hit = '1') then
                    hist_cache_read_miss.stop <= '1';
                    vrpo.port_ack := '1';
                    v.state := S_RP_INIT;
                elsif (rpai.gnt_busif = '1' and rpbifi.busif_busy = '0') then
                    v.state := S_RP_MISS_REQUEST_TAG;
                end if;
            when S_RP_MISS_REQUEST_TAG =>
                -- Request the tag again to check for a cache hit. The BusIF
                -- might already have replaced the cache line during
                -- 'S_RP_MISS_WAIT_BUSIF' on the request of some other port,
                -- and that data may have even been modified by a write port!
                -- Hence, replacing that cache line again would lead to wrong
                -- data.
                vrpao.req_busif := '1';
                vrpao.req_tagr := '1';
                vrpco.tag_rd := '1';
                if (tagr_gnt = '1') then
                    v.state := S_RP_MISS_READ_TAG;
                end if;
            when S_RP_MISS_READ_TAG =>
                -- read the tag & check for a cache hit
                vrpao.req_busif := '1';
                -- keep req_tagr to hold tag output
                vrpao.req_tagr := '1';
                bank_req_rd := '1';
                if (rpci.tag_in.valid = '1') then
                    if (bank_gnt = '1') then
                        hist_cache_read_miss.stop <= '1';
                        v.bank_sel := bank;
                        vrpo.port_ack := '1';
                        v.state := S_RP_READ_BANK;
                    end if;
                else
                    v.state := S_RP_MISS_REPLACE;
                end if;
            when S_RP_MISS_REPLACE =>
                -- Run the replacement and wait for the BusIF hit which MUST come some time
                vrpao.req_busif := '1';
                vrpbifo.busif_op := BIO_REPLACE;
                if (busif_hit = '1') then
                    hist_cache_read_miss.stop <= '1';
                    vrpo.port_ack := '1';
                    v.state := S_RP_INIT;
                end if;
            when others =>
                v.state := S_RP_INIT;
        end case;

        -- Update link registers...
        if (rpai.snoop_stb = '1' and (rpai.snoop_adr = r.link_adr or rpai.snoop_adr = v.link_adr)) then
                v.link_valid := '0';
        end if;
        
        -- Set derived outputs...
        vrpao.req_bank(conv_integer(bank)) := bank_req_rd;
        vrpco.bank_rd := bank_req_rd;

        rpo <= vrpo;
        rpbifo <= vrpbifo;
        rpco <= vrpco;
        rpao <= vrpao;

        rin <= v;

    end process;

    regs : process (clk)
    begin
        if (rising_edge(clk)) then
            if (reset = '1') then
                r.state <= S_RP_INIT;
                r.link_valid <= '0';
            else
                r <= rin;
            end if;
        end if;
    end process;

end rtl;
