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
--------------------------------------------------------------------------------


library ieee;
use ieee.std_logic_1164.all;

library paranut;
use paranut.paranut_config.all;
use paranut.types.all;
use paranut.csr.all;
use paranut.ifu.all;
use paranut.lsu.all;
use paranut.histogram.all;
use paranut.dbg.all;
use paranut.intc.all;

package exu is

 
	-- special cepu in/out tpye
	type cepu_out_type is record
			pnce 			:  STD_LOGIC_VECTOR (CFG_NUT_CPU_CORES-1 downto 0);
			pnlm 			:  STD_LOGIC_VECTOR (CFG_NUT_CPU_CORES-1 downto 0);
			icache_enable 	:  STD_LOGIC;
			dcache_enable 	:  STD_LOGIC;
			pnxsel		  	:  STD_LOGIC_VECTOR (CFG_NUT_CPU_CORES-1 downto 0);
	end record;

	type cepu_in_type is record
			pnhaltreq 		:  STD_LOGIC_VECTOR (CFG_NUT_CPU_CORES-1 downto 0);
			pnx		  		:  STD_LOGIC_VECTOR (CFG_NUT_CPU_CORES-1 downto 0);
	end record;
	type cepu_in_vector is array (natural range <>) of cepu_in_type;
	
	-- generic copu/cepu in/out type
	type exu_out_type is record
			haltreq 		:  STD_LOGIC;
			ex_o 			:  STD_LOGIC;
			sync_o 			:  STD_LOGIC;
			cause_o			:  STD_LOGIC_VECTOR (4 downto 0);
			epc_o			:  STD_LOGIC_VECTOR (31 downto 0);
			ifu_reset 	    :  STD_LOGIC;
	end record;
	type exu_out_vector is array (natural range <>) of exu_out_type;

	type exu_in_type is record
	        enable 			:  STD_LOGIC;
			linked 			:  STD_LOGIC;
			ex_i   			:  STD_LOGIC;
			sync_i 			:  STD_LOGIC;
			sync_next 		:  STD_LOGIC;
			cause_i			:  STD_LOGIC_VECTOR (4 downto 0);
			epc_i			:  STD_LOGIC_VECTOR (31 downto 0);
			xsel			:  STD_LOGIC;
			m2_ir_valid 	:  STD_LOGIC;    
			m2_ir 			:  STD_LOGIC_VECTOR (31 downto 0);
			m2_pc 			:  STD_LOGIC_VECTOR (31 downto 0);
	end record;
	type exu_in_vector is array (natural range <>) of exu_in_type;

    component mexu_wrapper
        generic (
                    CEPU_FLAG       : boolean := true;
                    CAPABILITY_FLAG : boolean := true;
                    CPU_ID          : integer := 0;
                    CLK_FREQ_HZ		: integer := 100_000_000
                );
        port (
                 clk            : in std_logic;
                 reset          : in std_logic;
				 -- to CSR
				 csri			: out csr_in_type;
				 csro			: in csr_out_type;
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
				 dbg_req 	  	: in std_logic;
				 -- Interrupt controller
				 intci 			: in intc_out_type;
				 intco			: out intc_in_type
                 --du_stall       : in std_logic;
                 -- Histogram
                 --emhci          : in exu_memu_hist_ctrl_in_type
                 -- TBD: timer, interrupt controller ...
             );
    end component;


end package;
