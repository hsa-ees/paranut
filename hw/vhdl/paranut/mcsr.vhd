--------------------------------------------------------------------------------
-- This file is part of the ParaNut project.
-- 
-- Copyright (C) 2013-2019  Alexander Bahle <alexander.bahle@hs-augsburg.de>
--                          Michael Seider, <michael.seider@hs-augsburg.de>
--     Hochschule Augsburg, University of Applied Sciences
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
--  Instruction fetch buffer module
--
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library paranut;
use paranut.paranut_config.all;
use paranut.csr.all;
use paranut.types.all;
use paranut.memu_lib.all;
use paranut.paranut_lib.all;

entity mcsr_wrapper is
    port (
             clk            : in std_logic;
             reset          : in std_logic;
             -- to EXU...
             csri           : in csr_in_type;
             csro           : out csr_out_type
         );
end mcsr_wrapper;

architecture rtl of mcsr_wrapper is

  component MCsr is
    port (
      clk : IN STD_LOGIC;
      reset : IN STD_LOGIC;
      pc : IN STD_LOGIC_VECTOR (31 downto 0);
      ir : IN STD_LOGIC_VECTOR (31 downto 0);
      exu_pop_priv_ir_stack_dreg : IN STD_LOGIC;
      ex_id_reg : IN STD_LOGIC_VECTOR (4 downto 0);
      exception : IN STD_LOGIC;
      irq_dreg : IN STD_LOGIC;
      hartID : IN STD_LOGIC_VECTOR (31 downto 0);
      clock_freq_hz : IN STD_LOGIC_VECTOR (31 downto 0);
      inCePU : IN STD_LOGIC;
      cpu_enabled : IN STD_LOGIC;
      linked : IN STD_LOGIC;
      csr_enable : IN STD_LOGIC;
      exu_cause : IN STD_LOGIC_VECTOR (4 downto 0);
      exu_dbg : IN STD_LOGIC;
      exu_dbg_reg : IN STD_LOGIC;
      exu_dbg_req : IN STD_LOGIC;
      exu_delegate_dreg : OUT STD_LOGIC;
      mret_dreg : IN STD_LOGIC;
      exu_csr_exception : OUT STD_LOGIC;
      exu_csr_rd_exception : OUT STD_LOGIC;
      exu_csr_mstatus_MIE : OUT STD_LOGIC;
      exu_csr_dcsr_step : OUT STD_LOGIC;
      exu_csr_dcsr_ebreakm : OUT STD_LOGIC;
      exu_isHalted : OUT STD_LOGIC;
      exu_csr_mepc : OUT STD_LOGIC_VECTOR (31 downto 0);
      exu_csr_dpc : OUT STD_LOGIC_VECTOR (31 downto 0);
      exu_csr_mtvec : OUT STD_LOGIC_VECTOR (31 downto 0);
      exu_csr_mcause : OUT STD_LOGIC_VECTOR (31 downto 0);
      exu_csr_sepc : OUT STD_LOGIC_VECTOR (31 downto 0);
      exu_csr_stvec : OUT STD_LOGIC_VECTOR (31 downto 0);
      exu_csr_mideleg : OUT STD_LOGIC_VECTOR (31 downto 0);
      exu_csr_mstatus_SIE : OUT STD_LOGIC;
      exu_m3_pnce : OUT STD_LOGIC_VECTOR (CFG_NUT_CPU_CORES-1 downto 0);
      exu_m3_pnlm : OUT STD_LOGIC_VECTOR (CFG_NUT_CPU_CORES-1 downto 0);
      exu_m3_pnxsel : OUT STD_LOGIC_VECTOR (CFG_NUT_CPU_CORES-1 downto 0);
      exu_m3_icache_enable : OUT STD_LOGIC;
      exu_m3_dcache_enable : OUT STD_LOGIC;
      csr_tval : IN STD_LOGIC_VECTOR (31 downto 0);
      m3_pnhaltreq : IN STD_LOGIC_VECTOR (CFG_NUT_CPU_CORES-1 downto 0);
      csr_function_reg : IN STD_LOGIC_VECTOR (2 downto 0);
      csr_rs1_reg : IN STD_LOGIC_VECTOR (4 downto 0);
      csr_adr_reg : IN STD_LOGIC_VECTOR (11 downto 0);
      csr_op_a : IN STD_LOGIC_VECTOR (31 downto 0);
      exu_csr_rdata : OUT STD_LOGIC_VECTOR (31 downto 0);
      exu_priv_mode_reg : OUT STD_LOGIC_VECTOR (1 downto 0);
      exu_epc : IN STD_LOGIC_VECTOR (31 downto 0);
      m3_pnx : IN STD_LOGIC_VECTOR (CFG_NUT_CPU_CORES-1 downto 0);
      perf_inc : IN STD_LOGIC;
      perf_addr : IN STD_LOGIC_VECTOR (2 downto 0) );
  end component;

begin

  HLSMCsr : component MCsr
  port map (
      clk => clk,
      reset => reset,
      pc => csri.pc,
      ir => csri.ir,
      exu_pop_priv_ir_stack_dreg => csri.exu_pop_priv_ir_stack_dreg,
      ex_id_reg => csri.ex_id_reg,
      exception => csri.exception,
      irq_dreg => csri.irq_dreg,
      hartID => csri.hartID,
      clock_freq_hz => csri.clock_freq_hz,
      inCePU => csri.inCePU,
      cpu_enabled => csri.cpu_enabled,
      linked => csri.linked,
      csr_enable => csri.csr_enable,
      exu_cause => csri.exu_cause,
      exu_dbg => csri.exu_dbg,
      exu_dbg_reg => csri.exu_dbg_reg,
      exu_dbg_req => csri.exu_dbg_req,
      mret_dreg => csri.mret_dreg,
      exu_epc => csri.exu_epc,
      m3_pnx => csri.m3_pnx,
      perf_inc => csri.perf_inc,
      perf_addr => csri.perf_addr,
      csr_tval => csri.csr_tval,
      m3_pnhaltreq => csri.m3_pnhaltreq,
      csr_function_reg => csri.csr_function_reg,
      csr_rs1_reg => csri.csr_rs1_reg,
      csr_adr_reg => csri.csr_adr_reg,
      csr_op_a => csri.csr_op_a,

      exu_csr_exception => csro.exu_csr_exception,
      exu_csr_rd_exception => csro.exu_csr_rd_exception,
      exu_csr_mstatus_MIE => csro.exu_csr_mstatus_MIE,
      exu_csr_dcsr_step => csro.exu_csr_dcsr_step,
      exu_csr_dcsr_ebreakm => csro.exu_csr_dcsr_ebreakm,
      exu_isHalted => csro.exu_isHalted,
      exu_csr_mepc => csro.exu_csr_mepc,
      exu_csr_dpc => csro.exu_csr_dpc,
      exu_csr_mtvec => csro.exu_csr_mtvec,
      exu_csr_mcause => csro.exu_csr_mcause,
      exu_csr_sepc => csro.exu_csr_sepc,
      exu_csr_stvec => csro.exu_csr_stvec,
      exu_csr_mideleg => csro.exu_csr_mideleg,
      exu_csr_mstatus_SIE => csro.exu_csr_mstatus_SIE,
      exu_m3_pnce => csro.exu_m3_pnce,
      exu_m3_pnlm => csro.exu_m3_pnlm,
      exu_m3_pnxsel => csro.exu_m3_pnxsel,
      exu_m3_icache_enable => csro.exu_m3_icache_enable,
      exu_m3_dcache_enable => csro.exu_m3_dcache_enable,
      exu_csr_rdata => csro.exu_csr_rdata,
      exu_priv_mode_reg => csro.exu_priv_mode_reg,
      exu_delegate_dreg => csro.exu_delegate_dreg
  );

end rtl;
