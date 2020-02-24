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
--  Constants, component and type declarations, and helper functions for all
--  modules that interface with the memu. This includes readports, writeports,
--  and busif.
--
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library paranut;
use paranut.paranut_config.all;
use paranut.types.all;
use paranut.paranut_lib.all;
use paranut.histogram.all;

package memu_lib is

    constant TR_PORTS : natural := CFG_NUT_CPU_CORES;
    constant RPORTS   : natural := CFG_NUT_CPU_CORES*2;
    constant WPORTS   : natural := CFG_NUT_CPU_CORES;

    subtype BUS_IF_OP is std_logic_vector(2 downto 0);
    constant BIO_NOTHING      : BUS_IF_OP := "000"; -- nothing
    constant BIO_WRITEBACK    : BUS_IF_OP := "001"; -- write back cache line to main memory
    constant BIO_INVALIDATE   : BUS_IF_OP := "010"; -- just invalidate, do not write back
    constant BIO_FLUSH        : BUS_IF_OP := "011"; -- write back and invalidate afterwards
    constant BIO_REPLACE      : BUS_IF_OP := "111"; -- replace cache line
    constant BIO_DIRECT_READ  : BUS_IF_OP := "101"; -- direct (uncached) read operation
    constant BIO_DIRECT_WRITE : BUS_IF_OP := "110"; -- direct (uncached) write operation

    -- Address layout
    -- | Tag address | Index address | Bank address | word offset |
    constant TAG_ADDR_OFFSET : natural := CFG_MEMU_CACHE_SETS_LD+CFG_MEMU_CACHE_BANKS_LD+2;
    constant TAG_ADDR_WIDTH  : natural := 32-TAG_ADDR_OFFSET;

    subtype BANK_OF_ADDR_RANGE is natural range CFG_MEMU_CACHE_BANKS_LD+2-1 downto 2;
    subtype IDATA_OF_ADDR_RANGE is natural range CFG_MEMU_CACHE_BANKS_LD+2-1 downto (CFG_MEMU_BUSIF_WIDTH/32)+1;
    subtype INDEX_OF_ADDR_RANGE is natural range TAG_ADDR_OFFSET-1 downto CFG_MEMU_CACHE_BANKS_LD+2;
    subtype INDEX_OF_WAY_INDEX_RANGE is natural range CFG_MEMU_CACHE_SETS_LD-1 downto 0;
    subtype TAG_OF_ADDR_RANGE is natural range 31 downto TAG_ADDR_OFFSET;
    subtype LINE_OF_ADDR_RANGE is natural range 31 downto CFG_MEMU_CACHE_BANKS_LD+2;

    subtype way_type is std_logic_vector(MAX(0, CFG_MEMU_CACHE_WAYS_LD-1) downto 0); -- may not become size 0
    type way_vector is array (natural range <>) of way_type;

    -- Wiadr Address layout
    -- | way address | Index address |
    subtype way_index_addr_type is std_logic_vector(CFG_MEMU_CACHE_WAYS_LD+CFG_MEMU_CACHE_SETS_LD-1 downto 0);
    type way_index_addr_vector is array (natural range <>) of way_index_addr_type;
    type way_index_addr_vector_2 is array (natural range <>) of way_index_addr_vector(0 to CFG_MEMU_BANK_RAM_PORTS-1);

    function get_way_index_of_addr(addr : TWord; way : std_logic_vector) return std_logic_vector;

    function adr_is_cached(addr : TWord) return boolean;
    function adr_is_special(addr : TWord) return boolean;

    ----------------------------------------------------------------------------
    -- Tag RAM...
    ----------------------------------------------------------------------------
    type cache_tag_type is record
        valid : std_logic;
        dirty : std_logic;
        taddr : std_logic_vector(TAG_ADDR_WIDTH-1 downto 0);
        way   : way_type; -- may not become size 0
    end record;

    type cache_tag_vector is array (natural range <>) of cache_tag_type;

    type tagram_in_type is record
        rd     : std_logic_vector(0 to TR_PORTS-1);
        wr     : std_logic_vector(0 to TR_PORTS-1);
        addr   : TWord_Vec(0 to TR_PORTS-1);
        tag_in : cache_tag_vector(0 to TR_PORTS-1);
    end record;

    type tagram_out_type is record
        ready   : std_logic;
        tag_out : cache_tag_vector(0 to TR_PORTS-1);
    end record;

    component mtagram
        port (
                 clk   : in std_logic;
                 reset : in std_logic;
                 tri   : in tagram_in_type;
                 tro   : out tagram_out_type
             );
    end component;

    ----------------------------------------------------------------------------
    -- Bank RAM...
    ----------------------------------------------------------------------------
    type bankram_in_type is record
        rd    : std_logic_vector(0 to CFG_MEMU_BANK_RAM_PORTS-1);
        wr    : std_logic_vector(0 to CFG_MEMU_BANK_RAM_PORTS-1);
        wiadr : way_index_addr_vector(0 to CFG_MEMU_BANK_RAM_PORTS-1);
        wdata : TWord_Vec(0 to CFG_MEMU_BANK_RAM_PORTS-1);
    end record;

    type bankram_out_type is record
        rdata :  TWord_Vec(0 to CFG_MEMU_BANK_RAM_PORTS-1);
    end record;

    type bankram_in_vector is array (natural range <>) of bankram_in_type;
    type bankram_out_vector is array (natural range <>) of bankram_out_type;

    component mbankram
        port (
                 clk : in std_logic;
                 bri : in bankram_in_type;
                 bro : out bankram_out_type
             );
    end component;

    ----------------------------------------------------------------------------
    -- Read Port...
    ----------------------------------------------------------------------------
    type readport_in_type is record
        port_rd     : std_logic;
        port_bsel   : TByteSel;
        port_direct : std_logic;
        port_adr    : TWord;
    end record;

    type readport_out_type is record
        port_ack  : std_logic;
        port_data : TWord;
    end record;

    type readport_in_vector is array (natural range <>) of readport_in_type;
    type readport_out_vector is array (natural range <>) of readport_out_type;

    type readport_cache_in_type is record
        bank_data_in : TWord;
        tag_in       : cache_tag_type;
    end record;

    type readport_cache_out_type is record
        tag_rd   : std_logic; -- 'bank_rd' and 'cache_data' must be routed according to 'port_adr'
        bank_rd  : std_logic; -- 'bank_rd' and 'cache_data' must be routed according to 'port_adr'
        bank_sel : std_logic_vector(CFG_MEMU_CACHE_BANKS_LD-1 downto 0);
        way_out  : way_type;
    end record;

    type readport_cache_in_vector is array (natural range <>) of readport_cache_in_type;
    type readport_cache_out_vector is array (natural range <>) of readport_cache_out_type;

    type readport_busif_in_type is record
        busif_adr        : TWord; -- address BUSIF is currently work on (to check for BUSIF hits)
        busif_data       : TWord; -- must be routed here from 'busif.data_out[]' according to 'port_adr'
        busif_data_valid : std_logic_vector(0 to CFG_MEMU_CACHE_BANKS-1);
        busif_busy       : std_logic;
    end record;

    type readport_busif_out_type is record
        busif_op : BUS_IF_OP; -- This can be replaced by one bit (only 'bioReplace' is invoked)
    end record;

    type readport_busif_in_vector is array (natural range <>) of readport_busif_in_type;
    type readport_busif_out_vector is array (natural range <>) of readport_busif_out_type;

    type readport_arbiter_in_type is record
        gnt_tagr  : std_logic;
        gnt_busif : std_logic;
        gnt_bank  : std_logic_vector(0 to CFG_MEMU_CACHE_BANKS-1);
    end record;

    type readport_arbiter_out_type is record
        req_tagr  : std_logic;
        req_busif : std_logic;
        req_bank  : std_logic_vector(0 to CFG_MEMU_CACHE_BANKS-1);
    end record;

    type readport_arbiter_in_vector is array (natural range <>) of readport_arbiter_in_type;
    type readport_arbiter_out_vector is array (natural range <>) of readport_arbiter_out_type;

    component mreadport
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
    end component;

    ----------------------------------------------------------------------------
    -- Write Port...
    ----------------------------------------------------------------------------
    type writeport_in_type is record
        port_wr          : std_logic;
        port_direct      : std_logic;
        port_bsel        : TByteSel;
        port_lres_scond  : std_logic; -- for LR/SC: link current addres (wr = 0) / store only when link is valid (wr = 1)
        port_writeback   : std_logic; -- Cache control; writeback + invalidate = flush (TBD: not implementated yet)
        port_invalidate  : std_logic; -- Cache control; writeback + invalidate = flush (TBD: not implementated yet)
        port_adr         : TWord;
        port_data        : TWord;
    end record;

    type writeport_out_type is record
        port_ack      : std_logic;
        port_scond_ok : std_logic; -- for LR/SC: set, if write-conditional was successful
    end record;

    type writeport_in_vector is array (natural range <>) of writeport_in_type;
    type writeport_out_vector is array (natural range <>) of writeport_out_type;

    -- - Address information for tag must be routed from 'port_adr', for the banks from 'port_adr' and 'tag_out.way'
    type writeport_cache_in_type is record
        bank_data_in : TWord; -- must be routed according to 'port_adr'
        tag_in       : cache_tag_type;
    end record;

    type writeport_cache_out_type is record
        tag_rd        : std_logic; -- 'bank_rd', 'bank_wr' must be routed according to 'port_adr'
        tag_wr        : std_logic; -- 'bank_rd', 'bank_wr' must be routed according to 'port_adr'
        bank_rd       : std_logic; -- 'bank_rd', 'bank_wr' must be routed according to 'port_adr'
        bank_wr       : std_logic; -- 'bank_rd', 'bank_wr' must be routed according to 'port_adr'
        bank_data_out : TWord; -- must be routed according to 'port_adr'
        tag_out       : cache_tag_type;
    end record;

    type writeport_cache_in_vector is array (natural range <>) of writeport_cache_in_type;
    type writeport_cache_out_vector is array (natural range <>) of writeport_cache_out_type;

    -- Request & grant lines...
    type writeport_arbiter_in_type is record
        gnt_linelock : std_logic;
        gnt_tagr     : std_logic;
        gnt_tagw     : std_logic;
        gnt_busif    : std_logic;
        gnt_bank     : std_logic_vector(0 to CFG_MEMU_CACHE_BANKS-1);
        -- With snoop unit (arbiter)...
        snoop_adr    : TWord;
        snoop_stb    : std_logic;
    end record;

    type writeport_arbiter_out_type is record
        req_linelock : std_logic;
        req_tagr     : std_logic;
        req_tagw     : std_logic;
        req_busif    : std_logic;
        req_bank     : std_logic_vector(0 to CFG_MEMU_CACHE_BANKS-1);
    end record;

    type writeport_arbiter_in_vector is array (natural range <>) of writeport_arbiter_in_type;
    type writeport_arbiter_out_vector is array (natural range <>) of writeport_arbiter_out_type;

    type writeport_busif_in_type is record
        busif_adr  : TWord; -- address BUSIF is currently work on (to check for BUSIF hits)
        busif_busy : std_logic;
    end record;

    type writeport_busif_out_type is record
        busif_op         : BUS_IF_OP;
        busif_nolinelock : std_logic; -- if set, no line lock is acquired (for write misses: lock is already held by write port) (TBD: eliminate this signal)
    end record;

    type writeport_busif_in_vector is array (natural range <>) of writeport_busif_in_type;
    type writeport_busif_out_vector is array (natural range <>) of writeport_busif_out_type;

    component mwriteport
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
    end component;

    ----------------------------------------------------------------------------
    -- BusIF...
    ----------------------------------------------------------------------------
    type busif_wishbone_in_type is record
        ack_i : std_logic;                                           -- normal termination
        err_i : std_logic;                                           -- termination w/ error
        rty_i : std_logic;                                           -- termination w/ retry
        dat_i : std_logic_vector(CFG_MEMU_BUSIF_WIDTH-1 downto 0);   -- input data bus
    end record;

    type busif_wishbone_out_type is record
        cyc_o : std_logic;                   						  -- cycle valid output
        stb_o : std_logic;                						      -- strobe output
        we_o  : std_logic;                       					  -- indicates write transfer
        sel_o : std_logic_vector((CFG_MEMU_BUSIF_WIDTH/8)-1 downto 0);-- select outputs (may differs for busif)
        adr_o : TWord;                        					      -- address bus outputs
        dat_o : std_logic_vector(CFG_MEMU_BUSIF_WIDTH-1 downto 0);    -- output data bus
        cti_o : std_logic_vector(2 downto 0); 						  -- cycle type identifier
        bte_o : std_logic_vector(1 downto 0);					      -- burst type extension
    end record;

    type busif_in_type is record
        busif_op         : BUS_IF_OP; -- all operations are atomic & by default acquire a line lock as needed
        busif_nolinelock : std_logic;       -- if set, no line lock is acquired (for write misses: lock is already held by write port)
        busif_bsel       : TByteSel;        -- byte select (only for uncached read & write access)
                                            -- Address & data buses...
        adr_in           : TWord;
        data_in          : TWord_Vec(0 to CFG_MEMU_CACHE_BANKS-1); -- to cache banks, tag banks (bank_rd[n] = 1) or write ports (else)...
        tag_in           : cache_tag_type;
    end record;

    type busif_out_type is record
        busif_busy     : std_logic; -- if 0, the BusIF is ready for a new operation
                                    -- if 0, data write to the cache (tag & bank) are guaranteed to be completed
        -- Control lines to Tag & Cache banks...
        tag_rd         : std_logic;
        tag_wr         : std_logic;
        bank_rd        : std_logic_vector(0 to CFG_MEMU_CACHE_BANKS-1);
        bank_wr        : std_logic_vector(0 to CFG_MEMU_CACHE_BANKS-1);
        -- Address & data
        adr_out        : TWord;                          -- to cache banks, tag bank, and r/w ports to determine BUSIF hits
        data_out       : TWord_Vec(0 to CFG_MEMU_CACHE_BANKS-1);  -- data read from bus
        data_out_valid : std_logic_vector(0 to CFG_MEMU_CACHE_BANKS-1);
        tag_out        : cache_tag_type;
    end record;

    type busif_arbiter_in_type is record
        gnt_linelock : std_logic;
        gnt_tagw     : std_logic;
        gnt_tagr     : std_logic;
        gnt_bank     : std_logic_vector(0 to CFG_MEMU_CACHE_BANKS-1);
    end record;

    type busif_arbiter_out_type is record
        req_linelock : std_logic;
        req_tagw     : std_logic;
        req_tagr     : std_logic;
        req_bank     : std_logic_vector(0 to CFG_MEMU_CACHE_BANKS-1);
    end record;

    component mbusif
        port (
                 clk    : in std_logic;
                 reset  : in std_logic;
                 -- Bus interface (Wishbone)...
                 bifwbi : in busif_wishbone_in_type;
                 bifwbo : out busif_wishbone_out_type;
                 -- Cache, Readports, Writeports...
                 bifi   : in busif_in_type;
                 bifo   : out busif_out_type;
                 -- Request & grant lines...
                 bifai  : in busif_arbiter_in_type;
                 bifao  : out busif_arbiter_out_type;
                 -- Histogram...
                 hist_cache_line_fill : out hist_ctrl_type;
                 hist_cache_line_wb : out hist_ctrl_type
             );
    end component;

    ----------------------------------------------------------------------------
    -- Arbiter...
    ----------------------------------------------------------------------------
    type arbiter_in_type is record
        tagram_ready : std_logic;
        wiadr_busif  : way_index_addr_type;
        wiadr_rp     : way_index_addr_vector(0 to RPORTS-1); -- (way+index) addresses from various ports; must be kept constant as long as 'req_*' lines are held
        adr_wp       : TWord_Vec(0 to WPORTS-1);
        way_wp       : way_vector(0 to WPORTS-1);
    end record;

    type arbiter_out_type is record
        wiadr_bank : way_index_addr_vector_2(0 to CFG_MEMU_CACHE_BANKS-1);
    end record;

    component marbiter
        --generic (
        --            CFG_MEMU_CACHE_BANKS : natural := 1;
        --            RPORTS      : natural := 1;
        --            WPORTS      : natural := 1
        --        );
        port (
                 clk          : in std_logic;
                 reset        : in std_logic;
                 ai           : in arbiter_in_type;
                 ao           : out arbiter_out_type;
                 bifai        : out busif_arbiter_in_type;
                 bifao        : in busif_arbiter_out_type;
                 rpai         : out readport_arbiter_in_vector(0 to RPORTS-1);
                 rpao         : in readport_arbiter_out_vector(0 to RPORTS-1);
                 wpai         : out writeport_arbiter_in_vector(0 to WPORTS-1);
                 wpao         : in writeport_arbiter_out_vector(0 to WPORTS-1)
             );
    end component;

end package;

package body memu_lib is

    function get_way_index_of_addr(addr : TWord; way : std_logic_vector) return std_logic_vector is
        variable wiaddr : way_index_addr_type;
    begin
        if (CFG_MEMU_CACHE_WAYS = 1) then
            wiaddr(CFG_MEMU_CACHE_SETS_LD-1 downto 0) := addr(INDEX_OF_ADDR_RANGE);
        else
            wiaddr := way & addr(INDEX_OF_ADDR_RANGE);
        end if;
        return (wiaddr);
    end;

    function adr_is_cached(addr : TWord)
    return boolean is
    begin
        if (addr < std_logic_vector(to_unsigned(CFG_NUT_MEM_SIZE, addr'length))) then
            --if (unsigned(addr) < (to_unsigned(CFG_NUT_MEM_SIZE, addr'length))) then
            return true;
        else
            return false;
        end if;
    end adr_is_cached;

    function adr_is_special(addr : TWord)
    return boolean is
    begin
        --if (addr(31 downto 28) = X"9") then
        if (addr(31) = '1') then
            return true;
        else
            return false;
        end if;
    end adr_is_special;

end package body;
