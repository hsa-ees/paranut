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

entity mwriteport is
    port (
             clk    : in std_logic;
             reset  : in std_logic;
             -- With (CPU) port...
             wpi    : in writeport_in_type;
             wpo    : out writeport_out_type;
             -- With cache...
             wpci   : in writeport_cache_in_type;
             wpco   : out writeport_cache_out_type;
             -- With BUSIF...
             wpbifi : in writeport_busif_in_type;
             wpbifo : out writeport_busif_out_type;
             -- Request & grant lines...
             wpai   : in writeport_arbiter_in_type;
             wpao   : out writeport_arbiter_out_type;
             -- Histogram...
             hist_cache_write_hit : out hist_ctrl_type;
             hist_cache_write_miss : out hist_ctrl_type
         );
end mwriteport;

architecture rtl of mwriteport is

-- - All input ports must be held until 'port_ack' is asserted, at least for one cycle.
-- - 'port_ack' is issued for exactly one cycle (no full handshake).
-- - 'port_ack' may be asserted already in the same cycle as the request was issued.
-- - The next request may be issued at earliest one cycle after 'port_ack' was asserted (TBD: allow in the ACK cycle?)
-- - If 'port_direct' is set, nothing happens with the cache (i.e. no invalidation).
--   Hence, 'port_direct' = 0/1 should not be mixed for one address.

    type state_type is (S_WP_INIT, S_WP_DIRECT, S_WP_REQUEST_LINELOCK_ONLY,
    S_WP_READ_TAG,
    S_WP_WRITE_TAG1_AND_BANK, S_WP_WRITE_TAG1, 
    S_WP_WRITE_BANK, S_WP_MISS, S_WP_REQUEST_BUSIF_ONLY,
    S_WP_RECHECK, S_WP_RECHECK_READ_TAG, S_WP_REPLACE, S_WP_REPLACE_WAIT_BUSIF,
    S_WP_SPECIAL_REQUEST_BUSIF_ONLY, S_WP_SPECIAL, S_WP_SPECIAL_WAIT_COMPLETE);

    function combine_data (bsel : TByteSel; word0, word1 : TWord) return TWord
    is
        variable ret : TWord;
    begin
        ret := word1;
        if (CFG_NUT_LITTLE_ENDIAN) then
			for n in 0 to 3 loop
				if (bsel(n) = '0') then
					ret(8*(n+1)-1 downto 8*n) := word0(8*(n+1)-1 downto 8*n);
				end if;
			end loop;
        else
			for n in 0 to 3 loop
				if (bsel(3-n) = '0') then
					ret(8*(n+1)-1 downto 8*n) := word0(8*(n+1)-1 downto 8*n);
				end if;
			end loop;
		end if;
        return (ret);
    end;

    type registers is record
        state : state_type;
        tag : cache_tag_type;
        data : TWord;
        -- Histogram...
        hist_miss : std_logic;
    end record;

    signal r, rin : registers;

begin

    comb : process (r, wpi, wpbifi, wpci, wpai)
        variable v : registers;
        -- Output variables
        variable vwpo : writeport_out_type;
        variable vwpbifo : writeport_busif_out_type;
        variable vwpco : writeport_cache_out_type;
        variable vwpao : writeport_arbiter_out_type;
        -- Helper variables
        variable tag : cache_tag_type;
        variable index : std_logic_vector(CFG_MEMU_CACHE_SETS_LD-1 downto 0);
        variable bank : std_logic_vector(CFG_MEMU_CACHE_BANKS_LD-1 downto 0);
        variable data : TWord;
        variable bank_req_rd : std_logic;
        variable bank_gnt : std_logic;
        variable link_and_port_adr_eq : std_logic;
    begin

        v := r;
        
        -- Defaults for outputs...

        hist_cache_write_hit.start <= '0';
        hist_cache_write_hit.stop <= '0';
        hist_cache_write_hit.abort <= '0';
        hist_cache_write_miss.start <= '0';
        hist_cache_write_miss.stop <= '0';
        hist_cache_write_miss.abort <= '0';

        vwpo.port_ack := '0';

        vwpbifo.busif_op := BIO_NOTHING;
        vwpbifo.busif_nolinelock := '1';

        vwpco.tag_rd := '0';
        vwpco.tag_wr := '0';
        vwpco.bank_wr := '0';
        vwpco.tag_out := r.tag;
        --vwpco.bank_data_out := (others => '-'); --TODO
        vwpco.bank_data_out :=  wpi.port_data; 
        vwpco.bank_bsel := wpi.port_bsel;

        vwpao.req_linelock := '0';
        vwpao.req_tagr := '0';
        vwpao.req_tagw := '0';
        vwpao.req_bank := (others => '0');
        vwpao.req_busif := '0';

        -- Helper variables
        index := wpi.port_adr(INDEX_OF_ADDR_RANGE);
        bank := wpi.port_adr(BANK_OF_ADDR_RANGE);
        bank_req_rd := '0';
        bank_gnt := wpai.gnt_bank(conv_integer(bank));

        -- Main "switch"...
        case r.state is
            when S_WP_INIT =>
                if (wpi.port_wr = '1') then
                    if (wpi.port_direct = '1') then
                        -- Direct (uncached) memory access...
                        vwpao.req_busif := '1';
                        vwpbifo.busif_op := BIO_DIRECT_WRITE;
                        if (wpai.gnt_busif = '1' and wpbifi.busif_busy = '0') then
                            v.state := S_WP_DIRECT;
                        end if;
                    else
                        -- Normal (cached) access...
                        if (wpi.port_lres_scond = '1' and wpi.port_scond_ok = '0') then
                            -- This is a "store conditional" (SC), which has failed right away...
                            -- (a failure may still occur later, even if this condition has not yet been fulfilled here)
                            vwpo.port_ack := '1';
                        else
                            -- Normal and (so far) successful SC cache accesses...
                            if (CFG_NUT_HISTOGRAM) then
                                v.hist_miss := '0';
                            end if;
                            hist_cache_write_hit.start <= '1';
                            hist_cache_write_miss.start <= '1';
                            vwpao.req_linelock := '1';
                            vwpao.req_tagr := '1';
                            vwpco.tag_rd := '1';
                            if (wpai.gnt_tagr = '1') then
                                if (wpai.gnt_linelock = '1') then
                                    v.state := S_WP_READ_TAG;
                                else
                                    v.state := S_WP_REQUEST_LINELOCK_ONLY;
                                end if;
                            end if;
                        end if;
                    end if;
                elsif (wpi.port_cache_op(0) = '1' or wpi.port_cache_op(1) = '1') then
                    -- Special operation...
                    vwpao.req_busif := '1';
                    vwpao.req_linelock := '1';
                    if (wpai.gnt_linelock = '1' and wpai.gnt_busif = '0') then
                        v.state := S_WP_SPECIAL_REQUEST_BUSIF_ONLY;
                    end if;
                    if (wpai.gnt_busif = '1' and wpbifi.busif_busy = '0') then
                        v.state := S_WP_SPECIAL;
                    end if;
                end if;
            -- Additional state for direct memory access...
            when S_WP_DIRECT =>
                -- Issue "direct write" operation...
                vwpao.req_busif := '1';
                vwpbifo.busif_op := BIO_DIRECT_WRITE;
                if (wpbifi.busif_busy = '1') then
                    -- Now the BUSIF is busy and has captured the data => can ack and complete
                    vwpo.port_ack := '1';
                    v.state := S_WP_INIT;
                end if;
            -- Additional states for normal access...
            when S_WP_REQUEST_LINELOCK_ONLY =>
                -- We got a grant for 'tagr', but not the 'linelock'
                -- => We must release everything except the 'linelock' to avoid a deadlock
                vwpao.req_linelock := '1';
                if (wpai.gnt_linelock = '1') then v.state := S_WP_INIT; end if;
            when S_WP_READ_TAG =>
                -- Capture the tag and request the bank...
                vwpao.req_linelock := '1';
                vwpco.tag_out.way := wpci.tag_in.way; -- for bank reading to output the correct cache way
                v.tag := wpci.tag_in;
                bank_req_rd := '1';
                if (wpci.tag_in.valid = '1') then
                    if (CFG_NUT_HISTOGRAM) then
                        if (r.hist_miss = '0') then
                            hist_cache_write_miss.abort <= '1';
                        end if;
                    end if;
                    -- Cache hit...
                    if (wpci.tag_in.dirty = '1') then
                        v.state := S_WP_WRITE_BANK;
                    else
                        v.state := S_WP_WRITE_TAG1_AND_BANK;
                    end if;
                else
                    if (CFG_NUT_HISTOGRAM) then
                        v.hist_miss := '1';
                    end if;
                    hist_cache_write_hit.abort <= '1';
                    v.state := S_WP_MISS;
                end if;
            when S_WP_WRITE_TAG1_AND_BANK =>
                vwpao.req_linelock := '1';
                vwpao.req_tagw := '1';
                vwpco.tag_out.dirty := '1';
                bank_req_rd := '1';
                --vwpco.bank_data_out := combine_data(wpi.port_bsel, r.data, wpi.port_data);
                if (bank_gnt = '1') then
                    vwpco.bank_wr := '1';
                    v.state := S_WP_WRITE_TAG1;
                end if;
                if (wpai.gnt_tagw = '1') then
                    vwpco.tag_wr := '1';
                    v.state := S_WP_WRITE_BANK;
                end if;
                if (bank_gnt = '1' and wpai.gnt_tagw = '1') then
                    vwpo.port_ack := '1'; -- can acknowledge to port now (write to bank and tag must be commited!)
                    v.state := S_WP_INIT;
                end if;
            when S_WP_WRITE_TAG1 =>
                vwpao.req_linelock := '1';
                vwpao.req_tagw := '1';
                vwpco.tag_out.dirty := '1';
                if (wpai.gnt_tagw = '1') then
                    vwpco.tag_wr := '1';
                    vwpo.port_ack := '1'; -- can acknowledge to port now (write to bank and tag must be commited!)
                    v.state := S_WP_INIT;
                end if;
            when S_WP_WRITE_BANK =>
                vwpao.req_linelock := '1';
                bank_req_rd := '1';
                --vwpco.bank_data_out := combine_data(wpi.port_bsel, r.data, wpi.port_data);
                if (bank_gnt = '1') then
                    hist_cache_write_hit.stop <= '1';
                    hist_cache_write_miss.stop <= '1';
                    --vwpao.req_linelock := '0';
                    vwpco.bank_wr := '1';
                    vwpo.port_ack := '1'; -- can acknowledge to port now (write to bank must be commited!)
                    v.state := S_WP_INIT;
                end if;
                -- Can we accept a new request already in this state?
                -- -> No, 'tagr' must not be requested while bank is held (deadlock)
    
                -- The following states handle a cache miss & replace a cache line.
            when S_WP_MISS =>
            -- Entry state for a cache miss. First, we must request and acquire the BusIf and potentially re-acquire the line lock...
                vwpao.req_busif := '1';
                vwpao.req_linelock := '1';
                if (wpai.gnt_busif = '1' and wpbifi.busif_busy = '0' and wpai.gnt_linelock = '1') then
                    v.state := S_WP_RECHECK;
                elsif (wpai.gnt_busif = '0' and wpai.gnt_linelock = '1') then
                    v.state := S_WP_REQUEST_BUSIF_ONLY;
                end if;
            when S_WP_REQUEST_BUSIF_ONLY =>
                -- Release the line lock and request the BusIf only to avoid deadlocks.
                vwpao.req_busif := '1';
                if (wpai.gnt_busif = '1') then v.state := S_WP_MISS; end if;
            when S_WP_RECHECK =>
                -- Now we have the BusIf and the line lock, and the BusIf
                -- is idle. We must re-check if there is a cache hit now,
                -- since some other port may have replaced the cache line in
                -- between.
                vwpao.req_busif := '1';
                vwpao.req_linelock := '1';
                vwpao.req_tagr := '1';
                vwpco.tag_rd := '1';
                if (wpai.gnt_tagr = '1') then
                    v.state := S_WP_RECHECK_READ_TAG;
                end if;
            when S_WP_RECHECK_READ_TAG =>
                -- Capture the tag and check it for a cache hit.
                vwpao.req_busif := '1';
                vwpao.req_linelock := '1';
                v.tag := wpci.tag_in;
                if (wpci.tag_in.valid = '1') then
                    if (wpci.tag_in.dirty = '1') then v.state := S_WP_WRITE_BANK;
                    else v.state := S_WP_WRITE_TAG1_AND_BANK;
                    end if;
                else
                    v.state := S_WP_REPLACE;
                end if;
            when S_WP_REPLACE =>
                -- Start the replacement by the BusIf.
                vwpao.req_busif := '1';
                vwpao.req_linelock := '1';
                vwpbifo.busif_op := BIO_REPLACE;
                if (wpbifi.busif_busy = '1') then v.state := S_WP_REPLACE_WAIT_BUSIF; end if;
            when S_WP_REPLACE_WAIT_BUSIF =>
                -- Wait for the BusIf to complete the replacement.
                vwpao.req_busif := '1';
                vwpao.req_linelock := '1';
                if (wpbifi.busif_busy = '0') then
                    vwpao.req_tagr := '1';
                    vwpco.tag_rd := '1';
                    if (wpai.gnt_tagr = '1') then
                        v.state := S_WP_READ_TAG;
                    else
                        v.state := S_WP_INIT;
                    end if;
                end if;
            -- States for special operations...
            when S_WP_SPECIAL_REQUEST_BUSIF_ONLY =>
                vwpao.req_busif := '1';
                if (wpai.gnt_busif = '1') then v.state := S_WP_INIT; end if;
            when S_WP_SPECIAL =>
                vwpao.req_busif := '1';
                vwpao.req_linelock := '1';
                vwpbifo.busif_op := '0' & wpi.port_cache_op;
                --if (wpi.port_writeback = '1' and wpi.port_invalidate = '1') then 
                    --vwpbifo.busif_op := BIO_FLUSH;
                --elsif (wpi.port_writeback = '1') then
                    --vwpbifo.busif_op := BIO_WRITEBACK;
                --else
                    --vwpbifo.busif_op := BIO_INVALIDATE;
                --end if;
                if (wpbifi.busif_busy = '1') then
                    v.state := S_WP_SPECIAL_WAIT_COMPLETE;
                end if;
            when S_WP_SPECIAL_WAIT_COMPLETE =>
                vwpao.req_busif := '1';
                vwpao.req_linelock := '1';
                if (wpbifi.busif_busy = '0') then
                    -- The BusIf has completed -> can ack and complete
                    --vwpao.req_linelock := '0';
                    vwpo.port_ack := '1';
                    v.state := S_WP_INIT;
                end if;
            when others =>
                v.state := S_WP_INIT;
        end case;
                
        -- Set derived outputs...
        vwpao.req_bank(conv_integer(bank)) := bank_req_rd;
        vwpco.bank_rd := bank_req_rd;

        wpao <= vwpao;
        wpco <= vwpco;
        wpo <= vwpo;
        wpbifo <= vwpbifo;

        rin <= v;

    end process;

    regs : process (clk)
    begin
        if (clk'event and clk = '1') then
            if (reset = '1') then
                r.state <= S_WP_INIT;
                r.data <= (others => '-');
            else
                r <= rin;
            end if;
        end if;
    end process;

end rtl;
