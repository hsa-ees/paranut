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
--  Component and type declarations for the csr modules
--
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;

library paranut;
use paranut.paranut_config.all;
use paranut.types.all;
use paranut.memu_lib.all;

package csr is
    
    type csr_in_type is record
        pc                          :  std_logic_vector (31 downto 0);
        ir                          :  std_logic_vector (31 downto 0);
        exu_pop_priv_ir_stack_dreg  :  std_logic;
        ex_id_reg                   :  std_logic_vector (4 downto 0);
        exception                   :  std_logic;
        irq_dreg                    :  std_logic;
        hartID                      :  std_logic_vector (31 downto 0);
        clock_freq_hz               :  std_logic_vector (31 downto 0);
        inCePU                      :  std_logic;
        cpu_enabled                 :  std_logic;
        linked                      :  std_logic;
        csr_enable                  :  std_logic;
        exu_cause                   :  std_logic_vector (4 downto 0);
        exu_dbg                     :  std_logic;
        exu_dbg_reg                 :  std_logic;
        exu_dbg_req                 :  std_logic;
        mret_dreg                   :  std_logic;
        exu_epc                     :  std_logic_vector (31 downto 0);
        m3_pnx                      :  std_logic_vector (CFG_NUT_CPU_CORES-1 downto 0);
        perf_inc                    :  std_logic;
        perf_addr                   :  std_logic_vector (2 downto 0);
        csr_tval                    :  std_logic_vector (31 downto 0);
        m3_pnhaltreq                :  std_logic_vector (CFG_NUT_CPU_CORES-1 downto 0);
        csr_function_reg            :  std_logic_vector (2 downto 0);
        csr_rs1_reg                 :  std_logic_vector (4 downto 0);
        csr_adr_reg                 :  std_logic_vector (11 downto 0);
        csr_op_a                    :  std_logic_vector (31 downto 0);
    end record;
    type csr_in_vector is array (natural range <>) of csr_in_type;

    type csr_out_type is record    
        exu_csr_exception       :  std_logic;
        exu_csr_rd_exception    :  std_logic;
        exu_csr_mstatus_MIE     :  std_logic;
        exu_csr_dcsr_step       :  std_logic;
        exu_csr_dcsr_ebreakm    :  std_logic;
        exu_isHalted            :  std_logic;
        exu_csr_mepc            :  std_logic_vector (31 downto 0);
        exu_csr_dpc             :  std_logic_vector (31 downto 0);
        exu_csr_mtvec           :  std_logic_vector (31 downto 0);
        exu_csr_mcause          :  std_logic_vector (31 downto 0);
        exu_csr_sepc            :  std_logic_vector (31 downto 0);
        exu_csr_stvec           :  std_logic_vector (31 downto 0);
        exu_csr_mideleg         :  std_logic_vector (31 downto 0);
        exu_csr_mstatus_SIE     :  std_logic;
        exu_m3_pnce             :  std_logic_vector (CFG_NUT_CPU_CORES-1 downto 0);
        exu_m3_pnlm             :  std_logic_vector (CFG_NUT_CPU_CORES-1 downto 0);
        exu_m3_pnxsel           :  std_logic_vector (CFG_NUT_CPU_CORES-1 downto 0);
        exu_m3_icache_enable    :  std_logic;
        exu_m3_dcache_enable    :  std_logic;
        exu_csr_rdata           :  std_logic_vector (31 downto 0);
        exu_priv_mode_reg       :  std_logic_vector (1 downto 0);
        exu_delegate_dreg       :  std_logic;
    end record;
    type csr_out_vector is array (natural range <>) of csr_out_type;

    component mcsr_wrapper
        port (
                 clk            : in std_logic;
                 reset          : in std_logic;

                 csri           : in csr_in_type;
                 csro           : out csr_out_type
             );
    end component;

end package;
