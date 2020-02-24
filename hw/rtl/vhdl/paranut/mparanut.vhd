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
--  ParaNut top level module. Contains EXU, LSU, IFU, MEMU.
--
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library paranut;
use paranut.paranut_pkg.all;
use paranut.paranut_config.all;
use paranut.types.all;
use paranut.memu.all;
use paranut.memu_lib.all;
use paranut.ifu.all;
use paranut.lsu.all;
use paranut.exu.all;
use paranut.dbg.all;
use paranut.intc.all;

-- pragma translate_off
use paranut.text_io.all;
use paranut.txt_util.all;
use paranut.tb_monitor.all;
-- pragma translate_on

entity mparanut is
    generic (
    --            CFG_NUT_CPU_CORES   : integer := 1;
    --            CFG_MEMU_CACHE_BANKS : integer := 1
			 CLK_FREQ_HZ : integer := 100_000_000
         );
    port (
			-- Ports (WISHBONE master)
			 clk_i    : in std_logic;
			 rst_i    : in std_logic;
			 ack_i    : in std_logic;                     -- normal termination
			 err_i    : in std_logic;                     -- termination w/ error
			 rty_i    : in std_logic;                     -- termination w/ retry
			 dat_i    : in std_logic_vector(CFG_MEMU_BUSIF_WIDTH-1 downto 0);  -- input data bus
			 cyc_o    : out std_logic;                    -- cycle valid output
			 stb_o    : out std_logic;                    -- strobe output
			 we_o     : out std_logic;                    -- indicates write transfer
			 sel_o    : out std_logic_vector((CFG_MEMU_BUSIF_WIDTH/8)-1 downto 0); -- byte select outputs
			 adr_o    : out TWord;                        -- address bus outputs
			 dat_o    : out std_logic_vector(CFG_MEMU_BUSIF_WIDTH-1 downto 0); -- output data bus
			 cti_o    : out std_logic_vector(2 downto 0); -- cycle type identifier
			 bte_o    : out std_logic_vector(1 downto 0); -- burst type extension
			 -- Other
             du_stall : in std_logic;
             ex_int   : in std_logic_vector(CFG_NUT_EX_INT-1 downto 0);
             -- JTAG 
             tck	: in std_logic;
             tms 	: in std_logic;
             tdi 	: in std_logic;
             tdo 	: out std_logic
         );
end mparanut;

architecture RTL of mparanut is

    -- MEMU: busif, read ports (rp), write ports (wp)
    signal mi : memu_in_type;
    signal mo : memu_out_type;

    -- IFU
    signal ifui : ifu_in_vector(0 to CFG_NUT_CPU_CORES-1);
    signal ifuo : ifu_out_vector(0 to CFG_NUT_CPU_CORES-1);

    -- LSU
    signal lsui : lsu_in_vector(0 to CFG_NUT_CPU_CORES-1);
    signal lsuo : lsu_out_vector(0 to CFG_NUT_CPU_CORES-1);
    -- others
    --signal icache_enable, dcache_enable: std_logic;


    -- EXU signals
    signal exui : exu_in_vector(0 to CFG_NUT_CPU_CORES-1);
    signal exuo : exu_out_vector(0 to CFG_NUT_CPU_CORES-1);
    signal cepui : cepu_in_vector(0 to CFG_NUT_CPU_CORES-1);
    signal cepuo : cepu_out_type;
	
	-- INTC signals
	signal intci : intc_in_type;
	signal intco : intc_out_type;

	-- DBG signals
	signal bifwbi : busif_wishbone_out_type;     
	signal bifwbo : busif_wishbone_in_type;
	signal jtagi  : jtag_in_type;
	signal jtago  : jtag_out_type;
	signal dmo	 : dm_out_type;
	signal reset_or : STD_LOGIC;
	signal ifu_reset : STD_LOGIC_VECTOR(0 to CFG_NUT_CPU_CORES-1);
	signal dbg_addressed : STD_LOGIC;
	
    -- Histogram
--    signal emhci : exu_memu_hist_ctrl_in_vector(0 to CFG_NUT_CPU_CORES-1);

    -- pragma translate_off
--    signal lsui_reg : lsu_in_vector(0 to CFG_signal rp_rd_i : STD_LOGIC;NUT_CPU_CORES-1);
--    signal lsuo_reg : lsu_out_vector(0 to CFG_NUT_CPU_CORES-1);
    -- pragma translate_on

	function OR_reduce(d: std_logic_vector) return std_logic is
	  constant all_zeros: std_logic_vector(d'range) := (others => '0');
	begin
	  if d = all_zeros then
		return '0';
	  else
		return '1';
	  end if;
	end OR_reduce;
			
begin
	-- MemU
	dbg_addressed <= '1' when mo.bifwbo.adr_o(31 downto 16) = "0000000000000000" else '0';
	mi.bifwbi.ack_i <= bifwbo.ack_i when dbg_addressed = '1' else ack_i;
    mi.bifwbi.err_i <= bifwbo.err_i when dbg_addressed = '1' else err_i;
    mi.bifwbi.rty_i <= bifwbo.rty_i when dbg_addressed = '1' else rty_i;
    mi.bifwbi.dat_i <= bifwbo.dat_i when dbg_addressed = '1' else dat_i;
    cyc_o <= '0' when dbg_addressed = '1' else mo.bifwbo.cyc_o;
    stb_o <= '0' when dbg_addressed = '1' else mo.bifwbo.stb_o;
    we_o <= '0' when dbg_addressed = '1' else mo.bifwbo.we_o;
    bsel_le : if (CFG_NUT_LITTLE_ENDIAN) generate
        sel_o <= mo.bifwbo.sel_o;
    end generate;
    bsel_be : if (not CFG_NUT_LITTLE_ENDIAN) generate
		-- TODO: Add reverse order based on CFG_MEMU_BUSIF_WIDTH
        sel_o(3) <= mo.bifwbo.sel_o(0);
        sel_o(2) <= mo.bifwbo.sel_o(1);
        sel_o(1) <= mo.bifwbo.sel_o(2);
        sel_o(0) <= mo.bifwbo.sel_o(3);
    end generate;
    adr_o <= mo.bifwbo.adr_o;
    dat_o <= mo.bifwbo.dat_o;
    cti_o <= mo.bifwbo.cti_o;
    bte_o <= mo.bifwbo.bte_o;

    MemU : mmemu
    port map (
                 clk => clk_i,
                 reset => reset_or,
                 mi => mi,
                 mo => mo
             );
             
	-- DBGU 
	bifwbi <= mo.bifwbo;
	--~ bifwbi.cyc_o <= mo.bifwbo.cyc_o;
	--~ bifwbi.stb_o <= mo.bifwbo.stb_o;
	--~ bifwbi.we_o <= mo.bifwbo.we_o;
	--~ bifwbi.adr_o <= mo.bifwbo.adr_o;
	--~ bifwbi.dat_o <=  mo.bifwbo.dat_o;
	
	jtagi.tck <= tck;
	jtagi.tms <= tms;
	jtagi.tdi <= tdi;
	tdo <= jtago.tdo;
	
    slv_bsel_le : if (CFG_NUT_LITTLE_ENDIAN) generate
        bifwbi.sel_o <= mo.bifwbo.sel_o;
    end generate;
    slv_bsel_be : if (not CFG_NUT_LITTLE_ENDIAN) generate
		-- TODO: Add reverse order based on CFG_MEMU_BUSIF_WIDTH
        bifwbi.sel_o(3) <= mo.bifwbo.sel_o(0);
        bifwbi.sel_o(2) <= mo.bifwbo.sel_o(1);
       	bifwbi.sel_o(1) <= mo.bifwbo.sel_o(2);
        bifwbi.sel_o(0) <= mo.bifwbo.sel_o(3);
    end generate;

	DBG : dbg_wrapper
	port map (
				clk => clk_i,
				reset => rst_i,
				bifwbi => bifwbi,
				bifwbo => bifwbo,
				jtagi => jtagi,
				jtago => jtago,
				dmo => dmo
	);
	-- reset_or used in all other modules so DBGU can reset them 
	reset_or <= dmo.dbg_reset or rst_i;
	
	-- IntC
	IntC : mintc_wrapper
	port map (
				clk => clk_i,
				reset => reset_or,
				intci => intci,
				intco => intco,
				ex_int => ex_int
	);

    -- IFUs
    IFUs : for n in 0 to CFG_NUT_CPU_MODE2_CORES-1 generate
        IFU : mifu_wrapper
        port map (
                     clk => clk_i,
                     reset => ifu_reset(n),
                     ifui => ifui(n),
                     ifuo => ifuo(n),
                     rpi => mi.rpi(CFG_NUT_CPU_CORES+n),
                     rpo => mo.rpo(CFG_NUT_CPU_CORES+n),
                     icache_enable => cepuo.icache_enable
                 );
    end generate IFUs;
    IFU_RESET_SIG : for n in 0 to CFG_NUT_CPU_CORES-1 generate
			ifu_reset(n) <= reset_or or exuo(n).ifu_reset;
	end generate IFU_RESET_SIG;
	
	MODE1_RP : for n in CFG_NUT_CPU_MODE2_CORES to CFG_NUT_CPU_CORES-1 generate
        -- unuse read ports     
        mi.rpi(CFG_NUT_CPU_CORES+n).port_rd     <= '0';
        mi.rpi(CFG_NUT_CPU_CORES+n).port_bsel   <= (others => '-');
        mi.rpi(CFG_NUT_CPU_CORES+n).port_direct <= '-';
        mi.rpi(CFG_NUT_CPU_CORES+n).port_adr    <= (others => '-');
	
		--mo.rpo(CFG_NUT_CPU_CORES+n).port_ack
    end generate MODE1_RP;

    -- LSUs
    LSUs : for n in 0 to CFG_NUT_CPU_CORES-1 generate
        LSUs_SIMPLE : if (CFG_LSU_SIMPLE) generate
            LSU : mlsu_simple
            port map (
                         clk => clk_i,
                         reset => rst_i,
                         lsui => lsui(n),
                         lsuo => lsuo(n),
                         rpi => mi.rpi(n),
                         rpo => mo.rpo(n),
                         wpi => mi.wpi(n),
                         wpo => mo.wpo(n),
                         dcache_enable => cepuo.dcache_enable
                     );
        end generate;
        LSUs_COMPLEX : if (not CFG_LSU_SIMPLE) generate
            LSU : mlsu_wrapper
            port map (
                         clk => clk_i,
                         reset => reset_or,
                         lsui => lsui(n),
                         lsuo => lsuo(n),
                         rpi => mi.rpi(n),
                         rpo => mo.rpo(n),
                         wpi => mi.wpi(n),
                         wpo => mo.wpo(n),
                         dcache_enable => cepuo.dcache_enable
                     );
        end generate;
    end generate LSUs;

--    hist_ctrl_gen : if (CFG_NUT_HISTOGRAM) generate
--        hist_ctrl_in_gen : for n in 0 to CFG_NUT_CPU_CORES-1 generate
--            hist_ctrl_in_gen_cepu : if (n = 0) generate
--                emhci(n).cache_line_fill <= mo.mhco.cache_line_fill;
--                emhci(n).cache_line_wb <= mo.mhco.cache_line_wb;
--            end generate;
--            emhci(n).cache_read_hit_ifu <= mo.mhco.cache_read_hit(CFG_NUT_CPU_CORES+n);
--            emhci(n).cache_read_miss_ifu <= mo.mhco.cache_read_miss(CFG_NUT_CPU_CORES+n);
--            emhci(n).cache_read_hit_lsu <= mo.mhco.cache_read_hit(n);
--            emhci(n).cache_read_miss_lsu <= mo.mhco.cache_read_miss(n);
--            emhci(n).cache_write_hit_lsu <= mo.mhco.cache_write_hit(n);
--            emhci(n).cache_write_miss_lsu <= mo.mhco.cache_write_miss(n);
--        end generate;
--    end generate;

    -- EXUs
    EXUs : for n in 0 to CFG_NUT_CPU_CORES-1 generate
        EXUCePUs : if (n = 0) generate -- CePU
            EXUCePU : mexu_wrapper
            generic map (
                            CEPU_FLAG => true,
                            CAPABILITY_FLAG => true,
                            CPU_ID => n,
                            CLK_FREQ_HZ => CLK_FREQ_HZ
                        )
            port map (
                         clk => clk_i,
                         reset => reset_or,
                         -- to IFU
                         ifui => ifui(n),
                         ifuo => ifuo(n),
                         -- to Load/Store Unit (LSU)
                         lsui => lsui(n),
                         lsuo => lsuo(n),
                         -- to/from CePU
                         exui => exui(n),
                         exuo => exuo(n),
                         -- to/from CoPUs
                         cepui => cepui(n),
                         cepuo => cepuo,
						 -- from Debug Module
						 dbg_req => dmo.dbg_request(n),
						 -- to/from IntC
						 intco => intci,
						 intci => intco
                     );
                     
           -- route signals for CePU
           exui(n).ex_i <= OR_reduce(cepui(0).pnx);
           exui(n).m2_ir_valid <= '-';
           exui(n).m2_ir <= (others => '-');
           exui(n).m2_pc <= (others => '-');
        end generate EXUCePUs;

        EXUMode2CoPUs: if (n > 0 and n < CFG_NUT_CPU_MODE2_CORES) generate -- CoPUs
            EXUMode2CoPU: mexu_wrapper
            generic map (
                            CEPU_FLAG => false,
                            CAPABILITY_FLAG => true,
                            CPU_ID => n,
                            CLK_FREQ_HZ => CLK_FREQ_HZ
                        )
            port map (
                         clk => clk_i,
                         reset => reset_or,
                         -- to IFU
                         ifui => ifui(n),
                         ifuo => ifuo(n),
                         -- to Load/Store Unit (LSU)
                         lsui => lsui(n),
                         lsuo => lsuo(n),
                         -- to/from CePU/CoPU
                         exui => exui(n),
                         exuo => exuo(n),
                         -- to/from CoPUs
                         cepui =>  cepui(n),
                         cepuo => open,
						 -- from Debug Module
						 dbg_req => dmo.dbg_request(n),
						 -- to/from IntC
						 intco => open,
						 intci.ir_request => '-',
						 intci.ir_id => (others => '-')
                     );
                     -- don't care about cepui on CoPUs
                     cepui(n).pnhaltreq  <= (others => '-');
					 cepui(n).pnx  <= (others => '-');
					 
			-- route signals for CoPUs
			exui(n).ex_i <= exuo(0).ex_o;
		    exui(n).m2_ir_valid <= ifuo(0).ir_valid;
			exui(n).m2_ir <= ifuo(0).ir;
			exui(n).m2_pc <= ifuo(0).pc;
        end generate EXUMode2CoPUs;
        
        EXUMode1CoPUs: if (n >= CFG_NUT_CPU_MODE2_CORES) generate -- CoPUs
            EXUMode1CoPU: mexu_wrapper
            generic map (
                            CEPU_FLAG => false,
                            CAPABILITY_FLAG => false,
                            CPU_ID => n,
                            CLK_FREQ_HZ => CLK_FREQ_HZ
                        )
            port map (
                         clk => clk_i,
                         reset => reset_or,
                         -- to IFU
                         ifui => open,				-- open
                         ifuo => ifuo(0),			-- CePU IFU signals
                         -- to Load/Store Unit (LSU)
                         lsui => lsui(n),
                         lsuo => lsuo(n),
                         -- to/from CePU/CoPU
                         exui => exui(n),
                         exuo => exuo(n),
                         -- to/from CoPUs
                         cepui =>  cepui(n),
                         cepuo => open,
						 -- from Debug Module
						 dbg_req => dmo.dbg_request(n),
						 -- to/from IntC
						 intco => open,
						 intci.ir_request => '-',
						 intci.ir_id => (others => '-')
                     );
                     -- don't care about cepui on CoPUs
                     cepui(n).pnhaltreq  <= (others => '-');
					 cepui(n).pnx  <= (others => '-');
					 
			-- route signals for CoPUs
			exui(n).ex_i <= exuo(0).ex_o;
		    exui(n).m2_ir_valid <= '-';
			exui(n).m2_ir <= (others => '-');
			exui(n).m2_pc <= (others => '-');
        end generate EXUMode1CoPUs;

        -- route signals for every CPU
        exui(n).enable <= cepuo.pnce(n);
        exui(n).linked <= cepuo.pnlm(n);
        exui(n).sync_next <= exuo(0).sync_o;
		exui(n).xsel <= cepuo.pnxsel(n);
        cepui(0).pnhaltreq(n) <= exuo(n).haltreq;
		cepui(0).pnx(n) <= exuo(n).ex_o;
    end generate EXUs;
    
    
	-- route exception & sync daisy chain...
	MoreThanOne: if (CFG_NUT_CPU_CORES_LD > 0) generate 
		-- All other CPUs
		CoPUs : for n in 0 to CFG_NUT_CPU_CORES-2 generate
			exui(n).cause_i <= exuo(n+1).cause_o;
			exui(n).epc_i <= exuo(n+1).epc_o;
			exui(n).sync_i <= exuo(n+1).sync_o;
		end generate CoPUs;
		
		-- Last CoPU
		exui(CFG_NUT_CPU_CORES-1).cause_i <= (others => '-');
		exui(CFG_NUT_CPU_CORES-1).epc_i <= (others => '-');
		exui(CFG_NUT_CPU_CORES-1).sync_i <= '1';
	end generate MoreThanOne;
	
	CePUOnly: if (CFG_NUT_CPU_CORES_LD = 0) generate 
		-- CePU only:
		exui(0).cause_i <= (others => '-');
		exui(0).epc_i <= (others => '-');	
		exui(0).sync_i <= '1';
	end generate CePUOnly;
	

-- pragma translate_off
    process (clk_i)
    begin
        if (clk_i'event and clk_i='1') then
            for n in 0 to CFG_NUT_CPU_CORES-1 loop
                if (cepuo.pnce(n) = '0') then
                    monitor(n).halted <= true;
                else
                    monitor(n).halted <= false;
                end if;
            end loop;
        end if;
--            if (CFG_DBG_LSU_TRACE) then
--                for n in 0 to CFG_NUT_CPU_CORES-1 loop
--                    if (lsui_reg(n).rd = '1' and lsuo_reg(n).ack = '1') then
--                        INFO("EXU(" & str(n) & ") LSU read:  " & hstr(lsui_reg(n).adr) &
--                        " DATA: " & hstr(lsuo(n).rdata) &
--                        " WIDTH: " & hstr(lsui_reg(n).width));
--                    end if;
--                    if (lsui(n).wr = '1' and lsui_reg(n).wr = '0') then
--                        INFO("EXU(" & str(n) & ") LSU write: " & hstr(lsui(n).adr) &
--                        " DATA: " & hstr(lsui(n).wdata) &
--                        " WIDTH: " & hstr(lsui(n).width));
--                    end if;
--                end loop;
--                lsui_reg <= lsui;
--                lsuo_reg <= lsuo;
--            end if;

--            if (CFG_DBG_BUS_TRACE) then
--                if (mi.bifwbi.ack_i = '1') then
--                    if (mo.bifwbo.we_o = '0') then
--                        INFO ("MEM read:  " & hstr(mo.bifwbo.adr_o) &
--                        " DATA: " & hstr(mi.bifwbi.dat_i) &
--                        " BSEL: " & hstr(mo.bifwbo.sel_o));
--                    else
--                        INFO ("MEM write: " & hstr(mo.bifwbo.adr_o) &
--                        " DATA: " & hstr(mo.bifwbo.dat_o) &
--                        " BSEL: " & hstr(mo.bifwbo.sel_o));
--                    end if;
--                end if;
--            end if;

--        end if;
    end process;
-- pragma translate_on

end RTL;

