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
--  Tag RAM module for the MEMU.
--
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library paranut;
use paranut.paranut_config.all;
use paranut.types.all;
use paranut.memu_lib.all;
use paranut.mem_tech.all;
use paranut.paranut_lib.all;
use paranut.lfsr.all;

entity mtagram is
    port (
             clk   : in std_logic;
             reset : in std_logic;
             tri   : in tagram_in_type;
             tro   : out tagram_out_type
         );
end mtagram;

architecture rtl of mtagram is

    constant COUNTER_INIT : std_logic_vector(7 downto 0) := (others => '1');
    constant COUNTER_POLY : std_logic_vector(7 downto 0) := get_prime_poly(8, 0);

    constant LRU_USE_WIDTH : natural := CFG_MEMU_CACHE_WAYS_LD*(CFG_MEMU_CACHE_WAYS-1)*CFG_MEMU_CACHE_REPLACE_LRU;
    constant TAG_WIDTH : natural := 2 + TAG_ADDR_WIDTH;
    constant TAG_ENTRY_WIDTH : natural := CFG_MEMU_CACHE_WAYS*TAG_WIDTH;
    type tag_addr_vector is array (natural range <>) of std_logic_vector(TAG_ADDR_WIDTH-1 downto 0);

    -- LRU information type
    subtype lru_use_type is std_logic_vector(MAX(0, LRU_USE_WIDTH-1) downto 0);
    type lru_use_vector is array (natural range <>) of lru_use_type;

    -- Tag layout
    -- | Valid bit | Dirty bit | Tag address |
    type tag_type is record
        valid : std_logic;
        dirty : std_logic;
        taddr : std_logic_vector(TAG_ADDR_WIDTH-1 downto 0);
    end record;
    type tag_vector is array (natural range <>) of tag_type;

    -- Tag entry layout
    -- | Tags (way(0)..way(n-1)) |
    type tag_entry_type is record
        tag : tag_vector(0 to CFG_MEMU_CACHE_WAYS-1);
    end record;
    type tag_entry_vector is array (natural range <>) of tag_entry_type;   
  
    -- Vector for memory data access
    subtype tag_mem_out_type is std_logic_vector(TAG_ENTRY_WIDTH-1 downto 0);
    type tag_mem_out_vector is array (natural range <>) of tag_mem_out_type;
    subtype tag_mem_in_type is std_logic_vector(TAG_WIDTH-1 downto 0);
    type tag_mem_in_vector is array (natural range <>) of tag_mem_out_type;

    subtype index_addr_type is std_logic_vector(CFG_MEMU_CACHE_SETS_LD-1 downto 0);
    type index_addr_vector is array (natural range <>) of index_addr_type;

    type registers is record
        counter : std_logic_vector(7 downto 0);
        lru_use_wr : std_logic_vector(0 to TR_PORTS);    -- One for each reader + one for all writeports
        lru_use_iadr : index_addr_vector(0 to TR_PORTS); -- One for each reader + one for all writeports
        lru_entry : lru_use_vector(0 to TR_PORTS);       -- One for each reader + one for all writeports
        taddr : tag_addr_vector(0 to TR_PORTS-1);
        iaddr : index_addr_vector(0 to TR_PORTS-1);
        read_tag : std_logic_vector(0 to TR_PORTS-1);
        write_tag : std_logic;
        wprt : integer range 0 to TR_PORTS-1;
        wtag : cache_tag_type;
    end record;

    -- Function for easily converting tag_type into slv for storing in RAM
    function t2slv (tag : cache_tag_type) return tag_mem_in_type is
        variable tag_slv : tag_mem_in_type;
    begin
        tag_slv := tag.valid & tag.dirty & tag.taddr;
        return (tag_slv);
    end;

    -- Function for easily converting slv into tag_entry_type for reading from RAM
    function slv2te (tag_slv : tag_mem_out_type) return tag_entry_type is
        variable tage : tag_entry_type;
    begin
        for i in 0 to CFG_MEMU_CACHE_WAYS-1 loop
            tage.tag(i).valid :=  tag_slv(TAG_WIDTH*(i+1)-1); 
            tage.tag(i).dirty :=  tag_slv(TAG_WIDTH*(i+1)-2); 
            tage.tag(i).taddr :=  tag_slv(TAG_ADDR_WIDTH*i+i*2+(TAG_ADDR_WIDTH-1) downto TAG_ADDR_WIDTH*i+i*2);
        end loop;
        return (tage);
    end;

    function get_new_use (old_use, way : std_logic_vector) return std_logic_vector is
        variable new_use : std_logic_vector(old_use'high+2 downto old_use'low);
        variable ret_use : std_logic_vector(old_use'range);
        variable skip : boolean := false;
    begin
        if (CFG_MEMU_CACHE_WAYS = 1) then
            ret_use(0) := '0'; -- not really used
        elsif (CFG_MEMU_CACHE_WAYS = 2) then
            ret_use(0) := not way(0);
        elsif (CFG_MEMU_CACHE_WAYS = 4) then
            -- The 'use' word is organized as follows:
            -- - bits 1:0 represent the least recently used way (prio #0)
            -- - bits 3:2 represent way with prio #1
            -- - bits 5:4 represent way with prio #2 (future improvement: only use one bit for this)
            -- - the most recently used way is not stored (can be derived)
            -- Set missing (most recently used) way to bits 7:6;
            -- the XOR of all numbers from 00..11 is 00. Hence XOR'ing 3 different 2-bit numbers results in the missing one.
            new_use := (old_use(5 downto 4) xor old_use(3 downto 2) xor old_use(1 downto 0)) & old_use;
            -- Search for the current way in the string and remove it if found...
            for n in 0 to 2 loop
                if (new_use(2*(n+1)-1 downto 2*n) = way) then
                    skip := true;
                end if;
                if (not skip) then
                    new_use(2*(n+1)-1 downto 2*n) := new_use(2*(n+1)-1 downto 2*n);
                else
                    new_use(2*(n+1)-1 downto 2*n) := new_use(2*(n+2)-1 downto 2*(n+1));
                end if;
            end loop;
            -- Only return the uppermost bits...
            ret_use(5 downto 0) := new_use(5 downto 0);
        end if;
        return (ret_use);
    end;

    signal r, rin : registers;

    -- Signals for tag block ram
    signal tram_raddr : index_addr_vector(0 to TR_PORTS-1);
    signal tram_rdata : tag_mem_out_vector(0 to TR_PORTS-1);
    signal tram_waddr : index_addr_type;
    signal tram_waddr_way : std_logic_vector (CFG_MEMU_CACHE_SETS_LD+CFG_MEMU_CACHE_WAYS_LD-1 downto 0);
    signal tram_wdata : tag_mem_in_type;
    signal tram_we : std_logic;
    
    -- Signals for LRU block ram
    signal tram_lru_rdata : lru_use_vector(0 to TR_PORTS-1);
    signal tram_lru_waddr : index_addr_type;
    signal tram_lru_wdata : lru_use_type;
    signal tram_lru_we : std_logic;     

    type tag_init_array is array (0 to 2) of integer;
    constant TAG_INIT : tag_init_array := (
        0 => 16#0#,
        1 => 16#0#,
        2 => 16#24#
    );

begin

    comb : process (reset, r, tri, tram_rdata, tram_lru_rdata)
        variable v : registers;
        variable wentry, rentry : tag_entry_type;
        variable tag : cache_tag_type;
        variable write_lru_prt: integer range 0 to TR_PORTS;
        variable way : std_logic_vector(MAX(0, CFG_MEMU_CACHE_WAYS_LD-1) downto 0);
        variable hit : boolean;
        variable write_lru : std_logic;
        variable new_use : std_logic_vector(MAX(0, LRU_USE_WIDTH-1) downto 0);
        variable new_use_wr : std_logic_vector(0 to TR_PORTS);
    begin

        v := r;

        tro.ready <= '1';

        -- Step random counter...
        if (CFG_MEMU_CACHE_WAYS > 1 and CFG_MEMU_CACHE_REPLACE_LRU = 0) then
            v.counter := get_next_lfsr_state(r.counter, COUNTER_POLY);
        end if;

        for p in 0 to TR_PORTS-1 loop
            v.taddr(p) := tri.addr(p)(TAG_OF_ADDR_RANGE);
            v.iaddr(p) := tri.addr(p)(INDEX_OF_ADDR_RANGE);
        end loop;

        -- Read access...
        new_use_wr := (others => '0');
        v.read_tag := (others => '0');
        for p in 0 to TR_PORTS-1 loop
            if (tri.rd(p) = '1') then
                v.read_tag(p) := '1';
            end if;
            rentry := slv2te(tram_rdata(p));
            hit := false;
            way := (others => '0');
            for w in 0 to CFG_MEMU_CACHE_WAYS-1 loop
                if (rentry.tag(w).valid = '1' and rentry.tag(w).taddr = r.taddr(p)) then
                    -- pragma translate_off
                    assert not hit report "same data stored in two ways" severity error;
                    -- pragma translate_on
                    way := conv_std_logic_vector(w, MAX(1, CFG_MEMU_CACHE_WAYS_LD));
                    hit := true;
                end if;
            end loop;
            if (hit) then -- we had a hit...
                tag.valid := '1';
                if (CFG_MEMU_CACHE_REPLACE_LRU = 1 and CFG_MEMU_CACHE_WAYS > 1) then
                    new_use := get_new_use (tram_lru_rdata(p), way);
                    if (new_use /= tram_lru_rdata(p)) then
                        if (r.read_tag(p) = '1') then
                            -- All read accesses that update LRU information
                            -- must first queue up in a write buffer (Only 1
                            -- tag can be written at a time, but there can be up
                            -- to TRPORTS read accesses) Also, a tag write
                            -- access has priority and only 1 writer at a time is
                            -- allowed).
                            new_use_wr(p) := '1';
                            v.lru_entry(p) := new_use;
                            v.lru_use_iadr(p) := r.iaddr(p);
                        end if;
                    end if;
                end if;
            else -- we had a miss...
                tag.valid := '0';
                if (CFG_MEMU_CACHE_WAYS = 1) then
                    way := (others => '0');
                else
                    if (CFG_MEMU_CACHE_REPLACE_LRU = 0) then
                        -- select random way
                        way := std_logic_vector(r.counter(CFG_MEMU_CACHE_WAYS_LD-1 downto 0));
                    else
                        -- select least recently used way
                        way := tram_lru_rdata(p)(MAX(0, CFG_MEMU_CACHE_WAYS_LD-1) downto 0);
                    end if;
                end if;
            end if;
            if (p = TR_PORTS-1 and tri.rd_way = '1') then
              -- Special read for BusIf (connected to last port) 
              way := tri.way;
              tag.valid := rentry.tag(conv_integer(way)).valid;
            end if;
            
            tag.dirty := rentry.tag(conv_integer(way)).dirty;
            tag.taddr := rentry.tag(conv_integer(way)).taddr;
            tag.way := way;

            tro.tag_out(p) <= tag;
        end loop;

        -- Writing TAG...
        v.write_tag := '0';
        v.wprt := 0;
        for n in 0 to TR_PORTS-1 loop
            if (tri.wr(n) = '1') then
                -- The write tag is saved in a register in the first cycle of 'wr=1'...
                -- pragma translate_off
                assert v.write_tag = '0' report "multiple tag write signals asserted" severity error;
                -- pragma translate_on
                v.wprt := n;
                v.write_tag := '1';
                v.wtag.way := tri.tag_in(n).way;
            end if;  
        end loop;
       
        tram_waddr <= tri.waddr(v.wprt)(INDEX_OF_ADDR_RANGE);
        if (CFG_MEMU_CACHE_WAYS = 1) then
            -- tram_waddr_way is equal to tram_waddr 
            tram_waddr_way(CFG_MEMU_CACHE_SETS_LD-1 downto 0) <= tram_waddr;
        else
            tram_waddr_way <= tri.waddr(v.wprt)(INDEX_OF_ADDR_RANGE) & tri.tag_in(v.wprt).way;
        end if;
        tram_wdata <= t2slv(tri.tag_in(v.wprt));
        tram_we <= v.write_tag;
         
         
        -- Select one of updated LRU information from past read accesses for writing into TagRAM...
        write_lru := '0';
        write_lru_prt := 0;
        if (CFG_MEMU_CACHE_REPLACE_LRU = 1 and CFG_MEMU_CACHE_WAYS > 1) then
            -- Check if we need to update lru information after a write last cycle
             if (r.write_tag = '1') then
                new_use := get_new_use (tram_lru_rdata(r.wprt), r.wtag.way);
                if (new_use /= tram_lru_rdata(r.wprt)) then
                 new_use_wr(TR_PORTS) := '1';
                 v.lru_entry(TR_PORTS) := new_use;
                 v.lru_use_iadr(TR_PORTS) := r.iaddr(r.wprt);
                end if;
            end if;

        
            for n in 0 to TR_PORTS loop
                if (r.lru_use_wr(n) = '1') then
                    write_lru := '1';
                    write_lru_prt := n;
                    if (r.lru_use_iadr(n) = r.iaddr(r.wprt) and r.write_tag = '1') then
                        -- Discard writing tag if a normal tag write happens to the same address (WAW hazard!)
                        -- but watch out for a new LRU update.
                        v.lru_use_wr(n) := new_use_wr(n);
                    end if;
                else
                    v.lru_use_wr(n) := new_use_wr(n);
                end if;
            end loop;
            if (write_lru = '1' and r.write_tag = '0') then
                -- Update lru_use_wr for the writer of this cycle
                v.lru_use_wr(write_lru_prt) := new_use_wr(write_lru_prt);
            end if;
            
            tram_lru_we <= write_lru;
            tram_lru_waddr <= r.lru_use_iadr(write_lru_prt);
            tram_lru_wdata <= r.lru_entry(write_lru_prt);
           
        end if;
        

        if (reset = '1') then
            tro.ready <= '0';
            if (CFG_MEMU_CACHE_WAYS > 1) then
                if (CFG_MEMU_CACHE_REPLACE_LRU = 0) then
                    v.counter := COUNTER_INIT;
                else
                    v.lru_use_wr := (others => '0');
                end if;
            end if;
        end if;

        rin <= v;

    end process;

    -- Multiply TagRAM for every read port, with common write access
    tagrams : for p in 0 to TR_PORTS-1 generate
        tram_raddr(p) <= tri.addr(p)(INDEX_OF_ADDR_RANGE);
        tagram : mem_sync_read_wider_dp_inferred
        generic map (AWIDTHW => CFG_MEMU_CACHE_SETS_LD + CFG_MEMU_CACHE_WAYS_LD, AWIDTHR => CFG_MEMU_CACHE_SETS_LD, 
                    DWIDTHW => TAG_WIDTH, DWIDTHR => TAG_ENTRY_WIDTH
                     )
        port map (clk, tram_we, tram_waddr_way, tram_raddr(p), tram_wdata, tram_rdata(p));
        
        -- Generate blockram for least recently used data
        lru_rams : if CFG_MEMU_CACHE_REPLACE_LRU = 1  generate
            lru_ram : mem_sync_simple_dp_inferred
            generic map (AWIDTH => CFG_MEMU_CACHE_SETS_LD, DWIDTH => LRU_USE_WIDTH,
                         INITD => TAG_INIT(CFG_MEMU_CACHE_WAYS_LD*CFG_MEMU_CACHE_REPLACE_LRU))
            port map (clk, tram_raddr(p), tram_lru_waddr, tram_lru_we, tram_lru_wdata, tram_lru_rdata(p));
        end generate;
    end generate;
    
    
    regs : process (clk)
    begin
        if (clk'event and clk = '1') then
            r <= rin;
        end if;
    end process;
end rtl;

