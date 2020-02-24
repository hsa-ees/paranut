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
--  ParaNut testbench. Executes the program found in prog_mem.vhd that is found in
--  this folder.
--
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;

library paranut;
use paranut.paranut_pkg.all;
use paranut.paranut_config.all;
use paranut.types.all;
use paranut.peripherals.all;
use paranut.prog_mem.all;
use paranut.tb_monitor.all;

entity mparanut_tb is
    end mparanut_tb;

architecture tb of mparanut_tb is

    constant MEM_WB_SLV_ADDR : natural := 16#10#;
    constant UART_WB_SLV_ADDR : natural := 16#90#;
    constant COUNTER_WB_SLV_ADDR : natural := 16#412#;

    signal clk : std_logic;
    signal reset : std_logic;
    signal wb_ack : std_logic;
    signal wb_err : std_logic;
    signal wb_rty : std_logic;
    signal wb_cyc : std_logic;
    signal wb_stb : std_logic;
    signal wb_we : std_logic;
    signal wb_sel : std_logic_vector((CFG_MEMU_BUSIF_WIDTH/8)-1 downto 0);
    signal wb_adr : TWord;
    signal wb_dat_r : std_logic_vector(CFG_MEMU_BUSIF_WIDTH-1 downto 0);
    signal wb_dat_w : std_logic_vector(CFG_MEMU_BUSIF_WIDTH-1 downto 0);
    signal wb_cti : std_logic_vector(2 downto 0);
    signal wb_bte : std_logic_vector(1 downto 0);

    signal ex_int : std_logic_vector(CFG_NUT_EX_INT-1 downto 0);

    signal tms : std_logic;
    signal tck : std_logic;
    signal tdi : std_logic;
    signal tdo : std_logic;

    constant clk_period : time := 10 ns;

begin

    nut : mparanut
    --generic map (1, 1)
    port map (clk, reset, wb_ack, wb_err, wb_rty, wb_dat_r, wb_cyc, wb_stb, wb_we, wb_sel, wb_adr, wb_dat_w, wb_cti, wb_bte, '0', ex_int, 
	tms, tck, tdi, tdo);

    memory : wb_memory
    generic map (WB_SLV_ADDR => MEM_WB_SLV_ADDR, PROG_DATA => prog_data)
    port map (clk, reset, wb_stb, wb_cyc, wb_we, wb_sel, wb_ack, wb_err, wb_rty,
    wb_adr, wb_dat_w, wb_dat_r, wb_cti, wb_bte);

    --~ uart : wb_uart
    --~ generic map (WB_SLV_ADDR => UART_WB_SLV_ADDR)
    --~ port map (clk, reset, wb_stb, wb_cyc, wb_we, wb_sel, wb_ack, wb_err, wb_rty,
    --~ wb_adr, wb_dat_w, wb_dat_r);

    --~ counter : wb_counter_wrapper
    --~ generic map (WB_SLV_ADDR => COUNTER_WB_SLV_ADDR, N_COUNTER => 1)
    --~ port map (clk, reset, wb_stb, wb_cyc, wb_we, wb_sel, wb_ack, wb_err, wb_rty,
    --~ wb_adr, wb_dat_w, wb_dat_r);

    clk_gen : process
    begin
        while (not sim_halt) loop
            clk <= '1'; wait for clk_period/2;
            clk <= '0'; wait for clk_period/2;
	    end loop;
        wait;
    end process;


    tb: process
    begin
        ex_int <= (others => '0');
        reset <= '1';
        wait for 5*clk_period;
		
		reset <= '0';      
		
		-- Clear reset bit of DBGU
		--~ slv_wb_adr <= x"70000000";
		--~ slv_wb_dat_w <= x"00000000";
		--~ slv_wb_stb <= '1';
		--~ slv_wb_cyc <= '1';     
		--~ slv_wb_we <= '1';
		--~ slv_wb_sel <= x"F";
		
		--~ wait until slv_wb_ack = '1';
		
		--~ slv_wb_adr <=  (others => '0');
		--~ slv_wb_dat_w <=  (others => '0');
		--~ slv_wb_stb <= '0';
		--~ slv_wb_cyc <=  '0';
		--~ slv_wb_we <=  '0';     
		--~ slv_wb_sel <=  (others => '0');
		
        for i in 0 to CFG_NUT_CPU_CORES-1 loop
            if not monitor(i).halted then
                wait until monitor(i).halted;
            end if;
        end loop;

        wait for 60*clk_period;

        sim_halt <= true;
            
        assert false report "Simulation finished" severity note;
        wait;

    end process;


end tb;
