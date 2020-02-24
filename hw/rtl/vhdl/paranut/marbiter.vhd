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
--   This is the arbiter for the MEMU. See description below for details.
--
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library paranut;
use paranut.paranut_config.all;
use paranut.types.all;
use paranut.memu_lib.all;
use paranut.paranut_lib.all;
use paranut.lfsr.all;

entity marbiter is
    --generic (
    --            CFG_MEMU_CACHE_BANKS : integer := 1;
    --            RPORTS      : integer := 1;
    --            WPORTS      : integer := 1
    --        );
    port (
             clk            : in std_logic;
             reset          : in std_logic;
             ai             : in arbiter_in_type;
             ao             : out arbiter_out_type;
             bifai          : out busif_arbiter_in_type;
             bifao          : in busif_arbiter_out_type;
             rpai           : out readport_arbiter_in_vector(0 to RPORTS-1);
             rpao           : in readport_arbiter_out_vector(0 to RPORTS-1);
             wpai           : out writeport_arbiter_in_vector(0 to WPORTS-1);
             wpao           : in writeport_arbiter_out_vector(0 to WPORTS-1)
         );
end marbiter;

architecture rtl of marbiter is

    -- Line lock...
    -- - Implements mutex for a single cache line (tag + all data banks).
    -- - Writers are supposed to acquire a line lock.
    -- - Readers do not acquire anything => Writers must perform their actions in a safe order.
    --  - if a line lock request is held, the associated address must not change
    --  - A line lock may be released during the clock cycle that the last bank/tag access is made,
    --    given that the bank/tag lock is still held. This allows faster write cycles without write port
    --    monopolizing a line lock (keeping it permanently up).

    -- Tag RAM...
    -- - Tag RAM must have CFG_NUT_CPU_CORES ports.
    -- - Each port is reserved for one CPU (three ports: RP #n, RP $CFG_NUT_CPU_CORES+n, WP #n).
    -- - BUSIF uses port #CFG_NUT_CPU_CORES-1.
    -- - Arbiter provides:
    --      a) Write-/Read-Lock (a write-lock excludes all readers)
    --      b) Port arbitration: Multiple readers are allowed, but only one per port (see above)
    --      c) Priority selection among a port: 0. BUSIF (if applicable), 1. Data read, 2. Insn Read, 3. Data Write
    --      EXAMINE: Would two ports/CPU bring speed improvement? Would a separate port for BUSIF bring speed improvement?
    --  - The tag RAM arbitration is also used to prevent writers from replacing/changing a cache line
    --      while it is read during a cache read hit. Hence, a reader must keep its 'tagr' lock until its bank access
    --      is completed, too. This is not necessary, if a line lock is held (i.e. for cache line reading during a write miss).

    -- Bank RAMs...
    -- - All ports of CPU n must be linked to port n % CFG_MEMU_BANK_RAM_PORTS of each bank RAM.
    -- - The BUSIF is linked to port #(CFG_MEMU_BANK_RAM_PORTS-1).
    -- - Multiple usually conflicting grant signals may be set, if the addresses match.
    -- - As long as a request signal is set, the address must not chage!
    -- - Among write ports and the BUSIF, only one grant will be given to avoid writing conflicts.

    -- BUSIF

    -- select/routing information can be derived from grant lines (only one is set at a time)

    -- Note on deadlock prevention:
    -- 1. Requests/grants must always be in the following order (-> break cyclic wait condition):
    --      busif < linelock < tagr/tagw < bank
    -- 2. (corollary to 1.) R/W ports must not request anything when waiting for BusIF (i.e. 'busif_busy')
    -- 3. tag/bank access may never be requested in a hold-and-wait manner: Either request simultaneously or use & complete serially.

    -- Mappings for the bank ports to the 'req_bank'/'gnt_bank' bus ...
    constant IDX_RP_OFF : integer := 0;             -- data read ports
    constant IDX_WP_OFF : integer := RPORTS;        -- write ports
    constant IDX_IP_OFF : integer := WPORTS;        -- insn read ports
    constant IDX_BUSIF  : integer := RPORTS+WPORTS; -- BusIF port
    constant RAMPORT_BUSIF : integer := CFG_MEMU_BANK_RAM_PORTS-1;

    constant COUNTER_INIT : std_logic_vector(15 downto 0) := (others => '1');
    constant COUNTER_POLY : std_logic_vector(15 downto 0) := get_prime_poly(16, 0);

    subtype read_req_gnt_type is std_logic_vector(0 to RPORTS+WPORTS);
    type read_req_gnt_vector is array (natural range <>) of read_req_gnt_type;

    type registers is record
        counter : unsigned(MAX(0, CFG_NUT_CPU_CORES_LD+CFG_MEMU_ARBITER_METHOD-1) downto 0);
        lfsr_counter : std_logic_vector(15 downto 0);
        linelock : std_logic_vector(0 to WPORTS);
        tagr : read_req_gnt_type;
        tagw : std_logic_vector(0 to WPORTS);
        bank : read_req_gnt_vector(0 to CFG_MEMU_CACHE_BANKS-1);
        busif : std_logic_vector(0 to RPORTS+WPORTS-1);
    end record;

    function get_prio_cpu (r : registers) return integer is
        variable index : unsigned(MAX(0, CFG_NUT_CPU_CORES_LD-1) downto 0);
    begin
        if (CFG_NUT_CPU_CORES_LD > 0) then
            if (CFG_MEMU_ARBITER_METHOD >= 0) then
                index := r.counter(CFG_MEMU_ARBITER_METHOD+CFG_NUT_CPU_CORES_LD-1 downto CFG_MEMU_ARBITER_METHOD);
            else
                index := unsigned(r.lfsr_counter(CFG_NUT_CPU_CORES_LD-1 downto 0));
            end if;
        else
            index := "0";
        end if;
        return (conv_integer(index));
    end;

    function ram_port (i : integer) return integer is
        variable index : integer range 0 to CFG_MEMU_BANK_RAM_PORTS-1;
    begin
        index := CFG_MEMU_BANK_RAM_PORTS-1;
        if (i /= IDX_BUSIF) then
            index := (i mod CFG_NUT_CPU_CORES) mod CFG_MEMU_BANK_RAM_PORTS;
        end if;
        return (index);
    end;

    signal r, rin : registers;

begin

    comb : process (reset, r, bifao, rpao, wpao, ai)
        variable v : registers;
        -- LineLockMethod
        variable req_linelock, gnt_linelock : std_logic_vector(0 to WPORTS);
        -- TagMethod
        variable req_tagr, gnt_tagr : read_req_gnt_type;
        variable req_tagw, gnt_tagw : std_logic_vector(0 to WPORTS);
        -- BankMethod
        variable req_bank, gnt_bank : read_req_gnt_type;
        variable sel_wiadr : way_index_addr_vector(0 to CFG_MEMU_BANK_RAM_PORTS-1);
        variable wiadr : way_index_addr_vector(0 to RPORTS+WPORTS);
        type sel_port_type is array (natural range <>) of integer range 0 to RPORTS+WPORTS;
        variable sel_port : sel_port_type (0 to CFG_MEMU_BANK_RAM_PORTS-1);
        variable sel_port_valid : std_logic_vector(0 to CFG_MEMU_BANK_RAM_PORTS-1);
        -- BusifMethod
        variable req_busif, gnt_busif : std_logic_vector(0 to RPORTS+WPORTS-1);
        variable i : integer range 0 to CFG_NUT_CPU_CORES-1;
        -- SnoopMethod
        variable write : boolean;
        variable writer : integer range 0 to WPORTS-1;
    begin

        v := r;

        -- LineLockMethod
        -- Current policy (to save area):
        -- - WPORT requests always exclude each other, independent of the index address
        -- - concurrent BUSIF and WPORT grants are possible, if they address different lines

        -- Collect all request signals...
        req_linelock(WPORTS) := bifao.req_linelock;
        for n in 0 to WPORTS-1 loop
            req_linelock(n) := wpao(n).req_linelock;
        end loop;
        -- Determine existing & to-keep grants...
        gnt_linelock := r.linelock and req_linelock;
        -- Handle BUSIF request (highest priority)...
        if (req_linelock(WPORTS) = '1' and gnt_linelock(WPORTS) = '0') then
            gnt_linelock(WPORTS) := '1';
            for n in 0 to WPORTS-1 loop
                if (gnt_linelock(n) = '1' and ai.adr_wp(n)(INDEX_OF_ADDR_RANGE) = ai.wiadr_busif(INDEX_OF_WAY_INDEX_RANGE)) then
                    gnt_linelock(WPORTS) := '0';
                end if;
            end loop;
        end if;
        -- Handle write port requests...
        for n in 0 to WPORTS-1 loop
            if (unsigned(gnt_linelock(0 to WPORTS-1)) /= 0) then exit; end if;
            i := (n + get_prio_cpu(r)) mod WPORTS;
            if (req_linelock(i) = '1' and (gnt_linelock(WPORTS) = '0' or ai.adr_wp(i)(INDEX_OF_ADDR_RANGE) /= ai.wiadr_busif(INDEX_OF_WAY_INDEX_RANGE))) then
                gnt_linelock(i) := '1';
            end if;
        end loop;

        -- Write results...
        v.linelock := gnt_linelock;

        bifai.gnt_linelock <= gnt_linelock(WPORTS);
        for n in 0 to WPORTS-1 loop
            wpai(n).gnt_linelock <= gnt_linelock(n);
        end loop;

        -- TagMethod
        if (ai.tagram_ready = '0') then
            gnt_tagr := (others => '0');
            gnt_tagw := (others => '0');
        else
            -- Collect all request signals...
            for n in 0 to RPORTS-1 loop req_tagr(n) := rpao(n).req_tagr; end loop;
            for n in 0 to WPORTS-1 loop
                req_tagr(n+RPORTS) := wpao(n).req_tagr;
                req_tagw(n) := wpao(n).req_tagw;
            end loop;
            req_tagr(RPORTS+WPORTS) := bifao.req_tagr;
            req_tagw(WPORTS) := bifao.req_tagw;
            -- Determine existing & to-keep grants...
            gnt_tagr := r.tagr and req_tagr;
            gnt_tagw := r.tagw and req_tagw;
            -- Handle read requests...
            if (unsigned(req_tagw) = 0) then -- Writer priority: only accept new reader if no write requests are pending...
                for n in 0 to CFG_NUT_CPU_CORES-1 loop -- Select highest priority acquired bit for each CPU
                    if ((gnt_tagr(n) or gnt_tagr(n+WPORTS) or gnt_tagr(n+2*WPORTS)) = '0'
                    and (n /= CFG_NUT_CPU_CORES-1 or gnt_tagr(RPORTS+WPORTS) = '0')) then
                        -- no existing grant...
                        if (n = CFG_NUT_CPU_CORES-1 and req_tagr(RPORTS+WPORTS) = '1') then
                            -- Prio 0: BUSIF (shares port with last CPU)
                            gnt_tagr(RPORTS+WPORTS) := '1';
                        elsif (req_tagr(n) = '1') then
                            -- Prio 1: Data read
                            gnt_tagr(n) := '1';
                        elsif (req_tagr(n+WPORTS) = '1') then
                            -- Prio 2: Insn read
                            gnt_tagr(n+WPORTS) := '1';
                        elsif (req_tagr(n+RPORTS) = '1') then
                            -- Prio 3: Data write
                            gnt_tagr(n+RPORTS) := '1';
                        end if;
                    end if;
                end loop;
            end if;

            -- Handle write requests...
            -- can only accept new writers if no other reader or writer active
            if (unsigned(gnt_tagr) = 0 and unsigned(gnt_tagw) = 0) then
                if (req_tagw(WPORTS) = '1') then
                    gnt_tagw(WPORTS) := '1'; -- give BUSIF highest priority
                else
                    for n in 0 to WPORTS-1 loop
                        i := (n + get_prio_cpu(r)) mod WPORTS;
                        if (req_tagw(i) = '1') then
                            gnt_tagw(i) := '1';
                            exit;
                        end if;
                    end loop;
                end if;
            end if;
        end if;

        -- Write results...
        v.tagr := gnt_tagr;
        v.tagw := gnt_tagw;

        for n in 0 to RPORTS-1 loop
            rpai(n).gnt_tagr <= gnt_tagr(n);
        end loop;
        for n in 0 to WPORTS-1 loop
            wpai(n).gnt_tagr <= gnt_tagr(n+RPORTS);
            wpai(n).gnt_tagw <= gnt_tagw(n);
        end loop;
        bifai.gnt_tagr <= gnt_tagr(RPORTS+WPORTS);
        bifai.gnt_tagw <= gnt_tagw(WPORTS);

        -- BankMethod
        -- Collect all way & index addresses...
        for n in 0 to CFG_NUT_CPU_CORES-1 loop
            wiadr(n+IDX_RP_OFF) := ai.wiadr_rp(n);
            wiadr(n+IDX_IP_OFF) := ai.wiadr_rp(n+CFG_NUT_CPU_CORES);
            wiadr(n+IDX_WP_OFF) := get_way_index_of_addr(ai.adr_wp(n), ai.way_wp(n));
        end loop;
        wiadr(RPORTS+WPORTS) := ai.wiadr_busif;

        for b in 0 to CFG_MEMU_CACHE_BANKS-1 loop
            -- Collect all request signals...
            for n in 0 to CFG_NUT_CPU_CORES-1 loop
                req_bank(n+IDX_RP_OFF) := rpao(n).req_bank(b);
                req_bank(n+IDX_IP_OFF) := rpao(n+CFG_NUT_CPU_CORES).req_bank(b);
                req_bank(n+IDX_WP_OFF) := wpao(n).req_bank(b);
            end loop;
            req_bank(IDX_BUSIF) := bifao.req_bank(b);
            --Determine existing & to-keep grants...
            gnt_bank := r.bank(b) and req_bank;
            -- Determine the selected ports...
            sel_port := (others => CFG_MEMU_BANK_RAM_PORTS-1);
            sel_port_valid := (others => '0');
            for n in 0 to RPORTS+WPORTS loop -- Prio -1: already granted ports
                if (gnt_bank(n) = '1') then
                    sel_port(ram_port(n)) := n;
                    sel_port_valid(ram_port(n)) := '1';
                end if;
            end loop;
            if (req_bank(IDX_BUSIF) = '1' and sel_port_valid(RAMPORT_BUSIF) = '0') then -- Prio 0: BUSIF
                sel_port(RAMPORT_BUSIF) := IDX_BUSIF;
                sel_port_valid(RAMPORT_BUSIF) := '1';
            end if;
            if (unsigned(req_bank) /= 0) then
                for n in 0 to CFG_NUT_CPU_CORES-1 loop
                    i := (get_prio_cpu(r) + n) mod CFG_NUT_CPU_CORES;
                    if (sel_port_valid(ram_port(i)) = '0') then
                        if (req_bank(i+IDX_RP_OFF) = '1') then
                            sel_port(ram_port(i)) := i+IDX_RP_OFF; -- Prio 1: Data read
                            sel_port_valid(ram_port(i)) := '1';
                        elsif (req_bank(i+IDX_IP_OFF) = '1') then
                            sel_port(ram_port(i)) := i+IDX_IP_OFF; -- Prio 2: Insn read
                            sel_port_valid(ram_port(i)) := '1';
                        elsif (req_bank(i+IDX_WP_OFF) = '1') then
                            sel_port(ram_port(i)) := i+IDX_WP_OFF; -- Prio 3: Data write
                            sel_port_valid(ram_port(i)) := '1';
                        end if;
                    end if;
                end loop;
            end if;

            -- Find selected 'wiadr's & determine all possible grant lines...
            for p in 0 to CFG_MEMU_BANK_RAM_PORTS-1 loop
                if (sel_port_valid(p) = '1') then sel_wiadr(p) := wiadr(sel_port(p));
                else sel_wiadr(p) := (others => '1'); -- TODO: don't care
                end if;
            end loop;
            for n in 0 to RPORTS+WPORTS loop
                if (sel_port_valid(ram_port(n)) = '1' and req_bank(n) = '1' and wiadr(n) = sel_wiadr(ram_port(n))) then
                    gnt_bank(n) := '1';
                end if;
            end loop;

            -- Remove all grants but one for write ports...
            -- (only one grant per bank is possible, so that the 'wdata' bus can be routed correctly)
            -- find the highest-priority write port that has been pre-granted...
            for n in 0 to CFG_NUT_CPU_CORES-1 loop
                i := (get_prio_cpu(r) + n) mod CFG_NUT_CPU_CORES;
                if (sel_port(ram_port(i)) /= IDX_BUSIF) then -- BusIF has highest prio
                    if (gnt_bank(i+IDX_WP_OFF) = '1') then
                        sel_port(ram_port(i)) := i+IDX_WP_OFF;
                        sel_port_valid(ram_port(i)) := '1';
                    end if;
                end if;
            end loop;
            -- Remove all grants except for the selected one...
            for n in 0 to CFG_NUT_CPU_CORES-1 loop
                if (sel_port(ram_port(n)) /= n+IDX_WP_OFF) then
                    gnt_bank(n+IDX_WP_OFF) := '0';
                end if;
            end loop;
            if (sel_port(RAMPORT_BUSIF) /= IDX_BUSIF) then gnt_bank(IDX_BUSIF) := '0'; end if;

            -- Write results...
            v.bank(b) := gnt_bank;
            for n in 0 to CFG_NUT_CPU_CORES-1 loop
                rpai(n).gnt_bank(b) <= gnt_bank(n+IDX_RP_OFF);
                rpai(n+CFG_NUT_CPU_CORES).gnt_bank(b) <= gnt_bank(n+IDX_IP_OFF);
                wpai(n).gnt_bank(b) <= gnt_bank(n+IDX_WP_OFF);
            end loop;
            bifai.gnt_bank(b) <= gnt_bank(IDX_BUSIF);

            for p in 0 to CFG_MEMU_BANK_RAM_PORTS-1 loop
                ao.wiadr_bank(b)(p) <= sel_wiadr(p);
            end loop;
        end loop;

        -- BusifMethod
        -- Collect all request signals...
        for n in 0 to RPORTS-1 loop req_busif(n) := rpao(n).req_busif; end loop;
        for n in 0 to WPORTS-1 loop req_busif(RPORTS+n) := wpao(n).req_busif; end loop;
        -- Determine existing & to-keep grants...
        gnt_busif := r.busif and req_busif;
        -- Handle new requests...
        for n in 0 to CFG_NUT_CPU_CORES-1 loop
            if (gnt_busif /= zero64(RPORTS+WPORTS-1 downto 0)) then exit; end if;
            i := (n + get_prio_cpu(r)) mod CFG_NUT_CPU_CORES;
            if (req_busif(i) = '1') then gnt_busif(i) := '1'; -- data read port
            elsif (req_busif(CFG_NUT_CPU_CORES+i) = '1') then gnt_busif(CFG_NUT_CPU_CORES+i) := '1'; -- insn read port
            elsif (req_busif(RPORTS+i) = '1') then gnt_busif(RPORTS+i) := '1'; end if; -- data write port
        end loop;
        --Write results...
        v.busif := gnt_busif;
        for n in 0 to RPORTS-1 loop rpai(n).gnt_busif <= gnt_busif(n); end loop;
        for n in 0 to WPORTS-1 loop wpai(n).gnt_busif <= gnt_busif(n+RPORTS); end loop;

        -- SnoopMethod
        -- Determine a/the writer...
        -- NOTE: only cached writes are supported for snooping (LL/SC)
        write := false;
        writer := WPORTS-1;
        for n in 0 to WPORTS-1 loop
            if (v.linelock(n) = '1') then -- to catch a writer to the cache
                --pragma translate_off
                -- there should be only one!
                assert not write report "more than 1 writers" severity error;
                --pragma translate_on
                write := true;
                writer := n;
            end if;
        end loop;
        
        -- Generate output signals...
        if (write) then
            for n in 0 to WPORTS-1 loop
                wpai(n).snoop_stb <= '1'; -- signal to all CPUs...
                wpai(n).snoop_adr <= ai.adr_wp(writer);
            end loop;
            wpai(writer).snoop_stb <= '0'; -- ... except to the one that caused the write to avoid race condition
        else
            for n in 0 to WPORTS-1 loop
                wpai(n).snoop_stb <= '0';
                wpai(n).snoop_adr <= (others => '1'); -- don't care
            end loop;
        end if;

        if (CFG_MEMU_ARBITER_METHOD >= 0) then
            -- round robin...
            v.counter := r.counter + 1;
        else
            -- LFSR...
            v.lfsr_counter := get_next_lfsr_state(r.lfsr_counter, COUNTER_POLY);
        end if;

        if (reset = '1') then
            if (CFG_MEMU_ARBITER_METHOD >= 0) then
                v.counter := (others => '1');
            else
                v.lfsr_counter := (others => '1');
            end if;
            v.linelock := (others => '0');
        end if;

        rin <= v;

    end process;

    process (clk)
    begin
        if (clk'event and clk = '1') then
            r <= rin;
        end if;
    end process;

end rtl;
