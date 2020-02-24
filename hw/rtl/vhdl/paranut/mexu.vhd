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
--  Execution unit (EXU) integer pipeline implementing the ORBIS32 instruction
--  set.
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library paranut;
use paranut.paranut_config.all;
use paranut.exu.all;
use paranut.types.all;
use paranut.paranut_lib.all;
use paranut.ifu.all;
use paranut.lsu.all;
use paranut.dbg.all;
use paranut.intc.all;
--use paranut.orbis32.all;
use paranut.memu_lib.all;
-- pragma translate_off
use paranut.text_io.all;
use paranut.txt_util.all;
use paranut.tb_monitor.all;
-- pragma translate_on

use paranut.histogram.all;

entity mexu_wrapper is
    generic (
                CEPU_FLAG       : boolean := true;
                CAPABILITY_FLAG : boolean := true;
                CPU_ID          : integer := 0;
                CLK_FREQ_HZ		: integer := 100_000_000
            );
    port (
             clk            : in std_logic;
             reset          : in std_logic;
             -- to IFU
             ifui           : out ifu_in_type;
             ifuo           : in ifu_out_type;
             -- to Load/Store Unit (LSU)
             lsui           : out lsu_in_type;
             lsuo           : in lsu_out_type;
			 -- CePU/CoPU control signals
			 cepuo 			: out cepu_out_type;
			 cepui			: in cepu_in_type;
			 exuo			: out exu_out_type;
			 exui			: in exu_in_type;
			 -- Debug module
			 dbg_req	  	: in std_logic;
			 -- Interrupt controller
			 intci 			: in intc_out_type;
			 intco			: out intc_in_type
				 
             --du_stall       : in std_logic;
             -- Histogram
             --emhci          : in exu_memu_hist_ctrl_in_type
             -- TBD: timer, interrupt controller ...
         );
end mexu_wrapper;

architecture rtl of mexu_wrapper is
	component MExu is
	port (
		clk : IN STD_LOGIC;
		reset : IN STD_LOGIC;
		ifu_next : OUT STD_LOGIC;
		ifu_jump : OUT STD_LOGIC;
		ifu_flush : OUT STD_LOGIC;
		ifu_reset : OUT STD_LOGIC;
		ifu_jump_adr : OUT STD_LOGIC_VECTOR (31 downto 0);
		ifu_ir_valid : IN STD_LOGIC;
		ifu_npc_valid : IN STD_LOGIC;
		ifu_ir : IN STD_LOGIC_VECTOR (31 downto 0);
		ifu_pc : IN STD_LOGIC_VECTOR (31 downto 0);
		ifu_npc : IN STD_LOGIC_VECTOR (31 downto 0);
		lsu_rd : OUT STD_LOGIC;
		lsu_wr : OUT STD_LOGIC;
		lsu_flush : OUT STD_LOGIC;
		lsu_cache_invalidate : OUT STD_LOGIC;
		lsu_cache_writeback : OUT STD_LOGIC;
		lsu_ack : IN STD_LOGIC;
		lsu_align_err : IN STD_LOGIC;
		lsu_width : OUT STD_LOGIC_VECTOR (1 downto 0);
		lsu_exts : OUT STD_LOGIC;
		lsu_adr : OUT STD_LOGIC_VECTOR (31 downto 0);
		lsu_rdata : IN STD_LOGIC_VECTOR (31 downto 0);
		lsu_wdata : OUT STD_LOGIC_VECTOR (31 downto 0);
		lsu_lres_scond : OUT STD_LOGIC;
		lsu_scond_ok : IN STD_LOGIC;
		xsel : IN STD_LOGIC;
		ex_i : IN STD_LOGIC;
		ex_o : OUT STD_LOGIC;
		epc_i : IN STD_LOGIC_VECTOR (31 downto 0);
		epc_o : OUT STD_LOGIC_VECTOR (31 downto 0);
		cause_i : IN STD_LOGIC_VECTOR (4 downto 0);
		cause_o : OUT STD_LOGIC_VECTOR (4 downto 0);
		sync_i : IN STD_LOGIC;
		sync_next : IN STD_LOGIC;
		sync_o : OUT STD_LOGIC;
		m2_ir_valid : IN STD_LOGIC;
		m2_ir : IN STD_LOGIC_VECTOR (31 downto 0);
		m2_pc : IN STD_LOGIC_VECTOR (31 downto 0);
		enable : IN STD_LOGIC;
		linked : IN STD_LOGIC;
		haltreq : OUT STD_LOGIC;
		m3_pnce : OUT STD_LOGIC_VECTOR (CFG_NUT_CPU_CORES-1 downto 0);
		m3_pnlm : OUT STD_LOGIC_VECTOR (CFG_NUT_CPU_CORES-1  downto 0);
		m3_pnxsel : OUT STD_LOGIC_VECTOR (CFG_NUT_CPU_CORES-1  downto 0);
		m3_pnhaltreq : IN STD_LOGIC_VECTOR (CFG_NUT_CPU_CORES-1  downto 0);
		m3_pnx : IN STD_LOGIC_VECTOR (CFG_NUT_CPU_CORES-1  downto 0);
		m3_ir_request : IN STD_LOGIC;
		m3_ir_id : IN STD_LOGIC_VECTOR (4 downto 0);
		m3_ir_ack : OUT STD_LOGIC;
		m3_ir_enable : OUT STD_LOGIC;
		m3_icache_enable : OUT STD_LOGIC;
		m3_dcache_enable : OUT STD_LOGIC;
		dbg_req : IN STD_LOGIC;
		hartID : IN STD_LOGIC_VECTOR (31 downto 0);
		inCePU : IN STD_LOGIC;
		mode2Cap : IN STD_LOGIC;
		ap_rst : IN STD_LOGIC;
		clock_freq_hz : IN STD_LOGIC_VECTOR (31 downto 0)
	);
	end component;


	signal hartID :  STD_LOGIC_VECTOR (31 downto 0);
    signal inCePU :  STD_LOGIC;
    signal mode2Cap :  STD_LOGIC;
    signal clkFreqHz : STD_LOGIC_VECTOR (31 downto 0);

begin

    HLSMexu : component MExu
        port map (
        clk => clk,
        reset => reset,
        -- to IFU
        ifu_next  => ifui.nexti,
        ifu_jump  => ifui.jump,
        ifu_jump_adr  =>  ifui.jump_adr,
        ifu_flush => ifui.flush,
        ifu_reset => exuo.ifu_reset,
        -- from IFU
        ifu_ir_valid  => ifuo.ir_valid ,
        ifu_npc_valid  => ifuo.npc_valid,
        ifu_ir  =>  ifuo.ir ,
        ifu_pc  =>  ifuo.pc ,
        ifu_npc  =>  ifuo.npc ,
        -- to LSU
        lsu_rd => lsui.rd,
        lsu_wr => lsui.wr,
        lsu_width => lsui.width,
        lsu_exts => lsui.exts,
        lsu_adr => lsui.adr,
        lsu_flush => lsui.flush,
        lsu_cache_invalidate => lsui.cache_invalidate,
        lsu_cache_writeback =>  lsui.cache_writeback,
		lsu_wdata => lsui.wdata,
		lsu_lres_scond => lsui.lres_scond,
        -- from LSU
        lsu_scond_ok => lsuo.scond_ok,
        lsu_ack  => lsuo.ack,
        lsu_align_err  => lsuo.align_err,
        lsu_rdata  => lsuo.rdata,
        -- from CePU
        enable  => exui.enable,
        linked  => exui.linked,
        ex_i	=> exui.ex_i,
        xsel 	=> exui.xsel,
		-- to CePU
		haltreq => exuo.haltreq,
		ex_o => exuo.ex_o,
		-- to CoPUs
        m3_pnce  => cepuo.pnce,
        m3_pnlm  => cepuo.pnlm,
        m3_icache_enable  => cepuo.icache_enable,
        m3_dcache_enable  => cepuo.dcache_enable,
    	m3_pnxsel => cepuo.pnxsel,
		-- from CoPUs
    	m3_pnhaltreq => cepui.pnhaltreq,
    	m3_pnx => cepui.pnx,
		-- from IntC
    	m3_ir_request => intci.ir_request,
		m3_ir_id  => intci.ir_id,
		-- to IntC
		m3_ir_ack => intco.ir_ack,
		m3_ir_enable => intco.ir_enable,
    
		-- exception signals
    	epc_i => exui.epc_i,
		epc_o => exuo.epc_o,
		cause_i => exui.cause_i,
		cause_o => exuo.cause_o,		
    	
		-- from Debug Module
		dbg_req => dbg_req,

		-- Mode 1 signals
		sync_i => exui.sync_i,
		sync_o => exuo.sync_o,
		sync_next => exui.sync_next,
		m2_ir_valid => exui.m2_ir_valid,
		m2_ir => exui.m2_ir,
		m2_pc => exui.m2_pc,

        ap_rst => reset,
    
		-- configuration ports 
        hartID => hartID,
        inCePU => inCePU,
        mode2Cap => mode2Cap,
        clock_freq_hz => clkFreqHz
    );
	
	-- Generic to port 
    hartID <= conv_std_logic_vector(CPU_ID, 32);
    inCePu <= conv_std_logic(CEPU_FLAG);
    mode2Cap <= conv_std_logic(CAPABILITY_FLAG);
    clkFreqHz <= conv_std_logic_vector(CLK_FREQ_HZ, 32);

end RTL;
