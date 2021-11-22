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
--   This is the memory unit (MEMU) of the ParaNut.
--   The MEMU interfaces with the main memory bus over a wishbone interface
--   and with the ParaNut CPUs over an arbitrary number of read and write
--   ports. The MEMU contains the (shared) cache and is optimize to handle
--   parallel memory accesses from different ports efficiently.
--   Also, the support for synchronization primitives is due to the MEMU.
--
--   The MEMU (entity 'mmemu') contains the following sub-modules:
--   - 1 tag RAM ('mtagram) for storing and supplying cache tag information
--   - CACHE_BANKS cache memory banks ('mbankram') for storing cached data
--   - 1 bus interface ('mbusif') for the Wishbone interconnection
--   - RPORTS read ports ('mreadport')
--   - WPORTS write ports ('mwriteport')
--   - 1 arbiter ('marbiter') for controlling the access to the caches' tag
--     and bank data
--
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;

library paranut;
use paranut.paranut_config.all;
use paranut.types.all;
use paranut.memu.all;
use paranut.memu_lib.all;
use paranut.paranut_lib.all;

-- pragma translate_off
use paranut.text_io.all;
use paranut.txt_util.all;
-- pragma translate_on

entity mmemu is
    --generic (
    --            CFG_MEMU_CACHE_BANKS : integer := 1;
    --            RPORTS      : integer := 2;
    --            WPORTS      : integer := 1
    --        );
    port (
             clk    : in std_logic;
             reset  : in std_logic;
             mi     : in memu_in_type;
             mo     : out memu_out_type
         );
end mmemu;

architecture rtl of mmemu is

    -- Tag RAM...
    signal tri : tagram_in_type;
    signal tro : tagram_out_type;

    -- Bank RAM...
    signal bri : bankram_in_vector(0 to CFG_MEMU_CACHE_BANKS-1);
    signal bro : bankram_out_vector(0 to CFG_MEMU_CACHE_BANKS-1);

    -- BusIF...
    signal bifi : busif_in_type;
    signal bifo : busif_out_type;

    -- Read ports...
    signal rpci   : readport_cache_in_vector(0 to RPORTS-1);
    signal rpco   : readport_cache_out_vector(0 to RPORTS-1);
    signal rpbifi : readport_busif_in_vector(0 to RPORTS-1);
    signal rpbifo : readport_busif_out_vector(0 to RPORTS-1);

    -- Write ports...
    signal wpci   : writeport_cache_in_vector(0 to WPORTS-1);
    signal wpco   : writeport_cache_out_vector(0 to WPORTS-1);
    signal wpbifi : writeport_busif_in_vector(0 to WPORTS-1);
    signal wpbifo : writeport_busif_out_vector(0 to WPORTS-1);

    -- Arbiter...
    signal ai    : arbiter_in_type;
    signal ao    : arbiter_out_type;
    signal bifai : busif_arbiter_in_type;
    signal bifao : busif_arbiter_out_type;
    signal rpai  : readport_arbiter_in_vector(0 to RPORTS-1);
    signal rpao  : readport_arbiter_out_vector(0 to RPORTS-1);
    signal wpai  : writeport_arbiter_in_vector(0 to WPORTS-1);
    signal wpao  : writeport_arbiter_out_vector(0 to WPORTS-1);

    -- pragma translate_off
    signal tri_reg : tagram_in_type;
    signal tro_reg : tagram_out_type;
    signal bri_reg : bankram_in_vector(0 to CFG_MEMU_CACHE_BANKS-1);
    signal bro_reg : bankram_out_vector(0 to CFG_MEMU_CACHE_BANKS-1);
    -- pragma translate_on

    type registers is record
        rp_busif_data : TWord_Vec(0 to (CFG_MEMU_BUSIF_WIDTH/32)-1);
    end record;

    signal r, rin : registers;

begin

    comb : process (r, mi.rpi, mi.wpi, tro, bro, rpai, rpbifo, wpai, wpbifo, bifai, bifo, rpco, wpco, ao)
        variable v : registers;
        variable cpu : integer;
        variable v_busif_cmd : busif_cmd_in_vector(0 to RPORTS+WPORTS);
    begin
        v := r;
        -- To Bank RAM...
        for b in 0 to CFG_MEMU_CACHE_BANKS-1 loop
            for p in 0 to CFG_MEMU_BANK_RAM_PORTS-1 loop
                -- defaults...
                bri(b).rd(p) <= '0';
                bri(b).wr(p) <= '0';
                bri(b).wdata(p) <= (others => '-');
                bri(b).wen(p) <= (others => '-');
                -- find CPU...
                cpu := p;
                while (cpu < CFG_NUT_CPU_CORES) loop
                    if (rpai(cpu).gnt_bank(b) = '1') then
                        if (rpco(cpu).bank_rd = '1') then
                            bri(b).rd(p) <= '1';
                        end if;
                    end if;
                    if (rpai(cpu+CFG_NUT_CPU_CORES).gnt_bank(b) = '1') then
                        if (rpco(cpu+CFG_NUT_CPU_CORES).bank_rd = '1') then
                            bri(b).rd(p) <= '1';
                        end if;
                    end if;
                    if (wpai(cpu).gnt_bank(b) = '1') then
                        if (wpco(cpu).bank_rd = '1') then
                            bri(b).rd(p) <= '1';
                        end if;
                        if (wpco(cpu).bank_wr = '1') then
                            bri(b).wr(p) <= '1';
                        end if;
                        bri(b).wdata(p) <= wpco(cpu).bank_data_out;
                        bri(b).wen(p) <= wpco(cpu).bank_bsel;
                    end if;
                    cpu := cpu + CFG_MEMU_BANK_RAM_PORTS;
                end loop;
            end loop;
            -- eventually link BUSIF to last port...
            if (bifai.gnt_bank(b) = '1') then
                if (bifo.bank_rd(b) = '1') then
                    bri(b).rd(CFG_MEMU_BANK_RAM_PORTS-1) <= '1';
                end if;
                if (bifo.bank_wr(b) = '1') then
                    bri(b).wr(CFG_MEMU_BANK_RAM_PORTS-1) <= '1';
                end if;
                bri(b).wdata(CFG_MEMU_BANK_RAM_PORTS-1) <= bifo.data_out(b);
                bri(b).wen(CFG_MEMU_BANK_RAM_PORTS-1) <= (others => '1');
            end if;
            -- from arbiter
            bri(b).wiadr <= ao.wiadr_bank(b);
        end loop;

        -- to BUSIF...
        -- defaults...
        v_busif_cmd(RPORTS+WPORTS).busif_op := BIO_NOTHING;
        v_busif_cmd(RPORTS+WPORTS).busif_nolinelock := '0';
        v_busif_cmd(RPORTS+WPORTS).busif_bsel := (others => '0');
        v_busif_cmd(RPORTS+WPORTS).adr := (others => '-');
        v_busif_cmd(RPORTS+WPORTS).wdata := (others => '-');
        -- from TAG RAMs...
        bifi.tag_in <= tro.tag_out(CFG_NUT_CPU_CORES-1);
        -- from BANK RAMs
        for b in 0 to CFG_MEMU_CACHE_BANKS-1 loop
            bifi.data_in(b) <= bro(b).rdata(CFG_MEMU_BANK_RAM_PORTS-1);
        end loop;     
        -- from read ports...
        for p in 0 to RPORTS-1 loop
--            if (rpai(p).gnt_busif = '1') then
                v_busif_cmd(p).busif_op := rpbifo(p).busif_op;
                v_busif_cmd(p).busif_nolinelock := '0';
                v_busif_cmd(p).busif_bsel := mi.rpi(p).port_bsel;
                v_busif_cmd(p).adr := mi.rpi(p).port_adr;
                v_busif_cmd(p).wdata := (others => '-');
--            end if;
        end loop;
        -- from write ports...
        for p in 0 to WPORTS-1 loop
--            if (wpai(p).gnt_busif = '1') then
                v_busif_cmd(RPORTS+p).busif_op := wpbifo(p).busif_op;
                v_busif_cmd(RPORTS+p).busif_nolinelock := wpbifo(p).busif_nolinelock;
                v_busif_cmd(RPORTS+p).busif_bsel := mi.wpi(p).port_bsel;
                v_busif_cmd(RPORTS+p).adr := mi.wpi(p).port_adr;
                v_busif_cmd(RPORTS+p).wdata := mi.wpi(p).port_data;
--            end if;
        end loop;
        
        bifi.busif_cmd_in <= v_busif_cmd(ao.busif_sel);

        for n in 0 to (CFG_MEMU_BUSIF_WIDTH/32)-1 loop
			-- Bank of bifo.adr_out and the next one for 64Bit 
			v.rp_busif_data(n) := bifo.data_out(conv_integer(bifo.adr_out(BANK_OF_ADDR_RANGE)+n));
		end loop;
        rin <= v;

    end process;

    process (clk)
    begin
        if (clk'event and clk = '1') then
            r <= rin;
        end if;
    end process;

    -- Tag RAM...
    tagram_interconnect : for n in 0 to CFG_NUT_CPU_CORES-1 generate
        tagram_interconnect_bif : if (n = CFG_NUT_CPU_CORES-1) generate
            tri.rd(n) <= bifo.tag_rd or rpco(n).tag_rd or rpco(CFG_NUT_CPU_CORES+n).tag_rd or wpco(n).tag_rd;
            tri.wr(n) <= bifo.tag_wr or wpco(n).tag_wr;
            tri.addr(n) <= bifo.adr_out when (bifai.gnt_tagr = '1') else
                           mi.wpi(n).port_adr when (wpai(n).gnt_tagr = '1') else
                           mi.rpi(n).port_adr when (rpai(n).gnt_tagr = '1') else
                           mi.rpi(n+CFG_NUT_CPU_CORES).port_adr;-- when rpai(n+CFG_NUT_CPU_CORES).gnt_tagr = '1' else
                           --(others => '-');
            tri.waddr(n) <= bifo.adr_out when (bifai.gnt_tagw = '1') else
                           mi.wpi(n).port_adr;
            tri.tag_in(n) <=  wpco(n).tag_out when (wpai(n).gnt_tagw = '1') else bifo.tag_out;
                            --bifo.tag_out when (bifai.gnt_tagw = '1') else
                             --wpco(n).tag_out;
            tri.rd_way <=  bifo.tag_rd_way when (bifai.gnt_tagr = '1') else '0';
        end generate;
        tagram_interconnect_no_bif : if (n /= CFG_NUT_CPU_CORES-1) generate
            tri.rd(n) <= rpco(n).tag_rd or rpco(CFG_NUT_CPU_CORES+n).tag_rd or wpco(n).tag_rd;
            tri.wr(n) <= wpco(n).tag_wr;
            tri.addr(n) <= mi.wpi(n).port_adr when (wpai(n).gnt_tagr = '1') else
                           mi.rpi(n).port_adr when (rpai(n).gnt_tagr = '1') else
                           mi.rpi(n+CFG_NUT_CPU_CORES).port_adr;-- when rpai(n+CFG_NUT_CPU_CORES).gnt_tagr = '1' else
                           --(others => '-');
            tri.waddr(n) <= mi.wpi(n).port_adr;
            tri.tag_in(n) <= wpco(n).tag_out;
        end generate;
    end generate;
    
    tri.way <= bifo.tag_out.way;

    -- To read ports...
    rp_ic : for p in 0 to RPORTS-1 generate
        rpci(p).tag_in <= tro.tag_out(p mod CFG_NUT_CPU_CORES);
        rpci(p).bank_data_in <= bro(conv_integer(rpco(p).bank_sel)).rdata((p mod CFG_NUT_CPU_CORES) mod CFG_MEMU_BANK_RAM_PORTS);
       
        rpbifi(p).busif_data <= r.rp_busif_data(0) when CFG_MEMU_BUSIF_WIDTH = 32 else -- All get the same busif_data
								r.rp_busif_data(conv_integer(mi.rpi(p).port_adr(2))); -- Either top or bottom 32Bit based on requested address
        
        --rpbifi(p).busif_data <= bifo.data_out(conv_integer(mi.rpi(p).port_adr(BANK_OF_ADDR_RANGE)));
        rpbifi(p).busif_data_valid <= bifo.data_out_valid;
        rpbifi(p).busif_adr <= bifo.adr_out;
        rpbifi(p).busif_busy <= bifo.busif_busy;
    end generate;

    -- To write ports...
    wp_ic : for p in 0 to WPORTS-1 generate
        wpci(p).tag_in <= tro.tag_out(p);
        wpci(p).bank_data_in <= bro(conv_integer(mi.wpi(p).port_adr(BANK_OF_ADDR_RANGE))).rdata(p mod CFG_MEMU_BANK_RAM_PORTS);
        wpbifi(p).busif_adr <= bifo.adr_out;
        wpbifi(p).busif_busy <= bifo.busif_busy;
    end generate;

    -- To arbiter...
    ai.wiadr_busif <= get_way_index_of_addr(bifo.adr_out, bifo.tag_out.way);
    arb_rps_ic : for p in 0 to RPORTS-1 generate
        ai.wiadr_rp(p) <= get_way_index_of_addr(mi.rpi(p).port_adr, rpco(p).way_out);
    end generate;
    arb_wps_ic : for p in 0 to WPORTS-1 generate
        ai.adr_wp(p) <= mi.wpi(p).port_adr;
        ai.way_wp(p) <= wpco(p).tag_out.way;
    end generate;
    ai.tagram_ready <= tro.ready;

    TagRam : MTagRam
    port map (
                 clk => clk,
                 reset => reset,
                 tri => tri,
                 tro => tro
             );

    -- Bank RAM...
    BankRams : for b in 0 to CFG_MEMU_CACHE_BANKS-1 generate
        BankRam : MBankRam
        port map (
                     clk => clk,
                     bri => bri(b),
                     bro => bro(b)
                 );
    end generate;

    -- BUSIF...
    BusIf : MBusIf
    port map (
                 clk => clk,
                 reset => reset,
                 bifwbi => mi.bifwbi,
                 bifwbo => mo.bifwbo,
                 bifi => bifi,
                 bifo => bifo,
                 bifai => bifai,
                 bifao => bifao,
                 hist_cache_line_fill => mo.mhco.cache_line_fill,
                 hist_cache_line_wb => mo.mhco.cache_line_wb
             );

    -- Read ports...
    ReadPorts : for n in 0 to RPORTS-1 generate
        ReadPort : MReadPort
        port map (
                     clk => clk,
                     reset => reset,
                     rpi => mi.rpi(n),
                     rpo => mo.rpo(n),
                     rpci => rpci(n),
                     rpco => rpco(n),
                     rpbifi => rpbifi(n),
                     rpbifo => rpbifo(n),
                     rpai => rpai(n),
                     rpao => rpao(n),
                     hist_cache_read_hit => mo.mhco.cache_read_hit(n),
                     hist_cache_read_miss => mo.mhco.cache_read_miss(n)
                 );
    end generate;

    -- Write ports...
    WritePorts : for n in 0 to WPORTS-1 generate
        WritePort : MWritePort
        port map (
                    clk => clk,
                    reset => reset,
                    wpi => mi.wpi(n),
                    wpo => mo.wpo(n),
                    wpci => wpci(n),
                    wpco => wpco(n),
                    wpbifi => wpbifi(n),
                    wpbifo => wpbifo(n),
                    wpai => wpai(n),
                    wpao => wpao(n),
                    hist_cache_write_hit => mo.mhco.cache_write_hit(n),
                    hist_cache_write_miss => mo.mhco.cache_write_miss(n)
                );
    end generate;

    -- Arbiter...
    Arbiter : MArbiter
    --generic map (
    --                CFG_MEMU_CACHE_BANKS => CFG_MEMU_CACHE_BANKS,
    --                RPORTS => RPORTS,
    --                WPORTS => WPORTS
    --            )
    port map (
                 clk => clk,
                 reset => reset,
                 ai => ai,
                 ao => ao,
                 bifai => bifai,
                 bifao => bifao,
                 rpai => rpai,
                 rpao => rpao,
                 wpai => wpai,
                 wpao => wpao
             );

    -- pragma translate_off
    process (clk)
    begin
        if (clk'event and clk='1') then

            if (CFG_DBG_TRAM_TRACE) then
                for n in 0 to CFG_NUT_CPU_CORES-1 loop
                    if (tri_reg.rd(n) = '1') then
                        INFO("TAGRAM  read port #" & str(n) &
                        ": " & hstr(tri_reg.addr(n)(INDEX_OF_ADDR_RANGE)) &
                        " TAG: " & hstr(tro.tag_out(n).taddr) &
                        " WAY: " & hstr(tro.tag_out(n).way) &
                        " VALID: " & str(tro.tag_out(n).valid) &
                        " DIRTY: " & str(tro.tag_out(n).dirty));
                    end if;
                    if (tri.wr(n) = '1') then
                        INFO("TAGRAM write port #" & str(n) &
                        ": " & hstr(tri.addr(n)(INDEX_OF_ADDR_RANGE)) &
                        " TAG: " & hstr(tri.tag_in(n).taddr) &
                        " WAY: " & hstr(tri.tag_in(n).way) &
                        " VALID: " & str(tri.tag_in(n).valid) &
                        " DIRTY: " & str(tri.tag_in(n).dirty));
                    end if;
                end loop;
                tri_reg <= tri;
                tro_reg <= tro;
            end if;

            if (CFG_DBG_BRAM_TRACE) then
                for b in 0 to CFG_MEMU_CACHE_BANKS-1 loop
                    for p in 0 to CFG_MEMU_BANK_RAM_PORTS-1 loop
                        if (bri_reg(b).rd(p) = '1') then
                            if (CFG_MEMU_CACHE_WAYS_LD > 0) then
                                INFO("BANKRAM(" & str(b) & ")  read port #" & str(p) &
                                ": " & hstr(bri_reg(b).wiadr(p)(CFG_MEMU_CACHE_SETS_LD-1 downto 0)) &
                                " WAY: " & str(conv_integer(bri_reg(b).wiadr(p)(CFG_MEMU_CACHE_WAYS_LD+CFG_MEMU_CACHE_SETS_LD-1 downto CFG_MEMU_CACHE_SETS_LD))) &
                                " DATA: " & hstr(bro(b).rdata(p)));
                            else
                                INFO("BANKRAM(" & str(b) & ")  read port #" & str(p) &
                                ": " & hstr(bri_reg(b).wiadr(p)) &
                                " DATA: " & hstr(bro(b).rdata(p)));
                            end if;
                        end if;
                        if (bri(b).wr(p) = '1') then
                            if (CFG_MEMU_CACHE_WAYS_LD > 0) then
                                INFO("BANKRAM(" & str(b) & ") write port #" & str(p) &
                                ": " & hstr(bri_reg(b).wiadr(p)(CFG_MEMU_CACHE_SETS_LD-1 downto 0)) &
                                " WAY: " & str(conv_integer(bri_reg(b).wiadr(p)(CFG_MEMU_CACHE_WAYS_LD+CFG_MEMU_CACHE_SETS_LD-1 downto CFG_MEMU_CACHE_SETS_LD))) &
                                " DATA: " & hstr(bri(b).wdata(p)));
                            else
                                INFO("BANKRAM(" & str(b) & ") write port #" & str(p) &
                                ": " & hstr(bri(b).wiadr(p)) &
                                " DATA: " & hstr(bri(b).wdata(p)));
                            end if;
                        end if;
                    end loop;
                end loop;
                bri_reg <= bri;
                bro_reg <= bro;
            end if;

        end if;
    end process;
    -- pragma translate_on

end rtl;
