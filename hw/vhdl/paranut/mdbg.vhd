--------------------------------------------------------------------------------
-- This file is part of the ParaNut project.
-- 
-- Copyright (C) 2019  Alexander Bahle <alexander.bahle@hs-augsburg.de>
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
--  ParaNut Debug Unit (DBGU) 
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library paranut;
use paranut.paranut_config.all;
use paranut.memu_lib.all;
use paranut.types.all;
use paranut.dbg.all;

entity dbg_wrapper is
  port (
    clk    : in std_logic;
    reset  : in std_logic;
    -- to/from Wishbone 
    bifwbi : in busif_wishbone_out_type;     
    bifwbo : out busif_wishbone_in_type;
    -- to/from JTAG
    jtagi  : in jtag_in_type;
    jtago  : out jtag_out_type;
    -- to ExUs 
    dmo : out dm_out_type
  );
end dbg_wrapper;

architecture rtl of dbg_wrapper is
  component MDebugModule is
  port (
    clk_i : IN STD_LOGIC;
    rst_i : IN STD_LOGIC;
    stb_i : IN STD_LOGIC;
    cyc_i : IN STD_LOGIC;
    we_i : IN STD_LOGIC;
    sel_i : IN STD_LOGIC_VECTOR ((CFG_MEMU_BUSIF_WIDTH/8)-1 downto 0);
    ack_o : OUT STD_LOGIC;
    err_o : OUT STD_LOGIC;
    rty_o : OUT STD_LOGIC;
    adr_i : IN STD_LOGIC_VECTOR (31 downto 0);
    dat_i : IN STD_LOGIC_VECTOR (CFG_MEMU_BUSIF_WIDTH-1 downto 0);
    dat_o : OUT STD_LOGIC_VECTOR (CFG_MEMU_BUSIF_WIDTH-1 downto 0);
    dbg_request : OUT STD_LOGIC_VECTOR (CFG_NUT_CPU_CORES-1 downto 0);
    dbg_reset : OUT STD_LOGIC;
    dmi_adr_i : IN STD_LOGIC_VECTOR (5 downto 0);
    dmi_dat_i : IN STD_LOGIC_VECTOR (31 downto 0);
    dmi_dat_o : OUT STD_LOGIC_VECTOR (31 downto 0);
    dmi_rd : IN STD_LOGIC;
    dmi_wr : IN STD_LOGIC
  );
  end component;

  component MDtm is
  port (
    tck : IN STD_LOGIC;
    tms : IN STD_LOGIC;
    tdi : IN STD_LOGIC;
    tdo : OUT STD_LOGIC;
    reset : IN STD_LOGIC;
    dmi_adr : OUT STD_LOGIC_VECTOR (5 downto 0);
    dmi_dat_o : OUT STD_LOGIC_VECTOR (31 downto 0);
    dmi_dat_i : IN STD_LOGIC_VECTOR (31 downto 0);
    dmi_rd : OUT STD_LOGIC;
    dmi_wr : OUT STD_LOGIC
  );
  end component;

  signal dmi : dmi_type;
begin

    HLSMDebugModule : component MDebugModule
      port map (
        clk_i => clk,
        rst_i => reset,
        -- WISHBONE input
        stb_i => bifwbi.stb_o,
        cyc_i  => bifwbi.cyc_o,
        we_i => bifwbi.we_o,
        sel_i  => bifwbi.sel_o,
        adr_i => bifwbi.adr_o,
        dat_i  => bifwbi.dat_o,
        -- WISHBONE output
        ack_o => bifwbo.ack_i,
        err_o => bifwbo.err_i,
        rty_o => bifwbo.rty_i,
        dat_o => bifwbo.dat_i,
        -- to all other modules 
        dbg_reset => dmo.dbg_reset,
        -- to  EXUs
        dbg_request => dmo.dbg_request,
        -- to/from DTM
        dmi_adr_i => dmi.adr,
        dmi_dat_i => dmi.dat_o, -- MDTM is dmi master
        dmi_dat_o => dmi.dat_i, -- MDTM is dmi master
        dmi_rd => dmi.rd,
        dmi_wr => dmi.wr
      );
        
    HLSMMDTM : component MDtm
      port map (
        reset => reset,
        -- WISHBONE output
        tck => jtagi.tck,
        tms => jtagi.tms,
        tdi => jtagi.tdi,
        tdo => jtago.tdo,
        -- to/from Debug Module
        dmi_adr => dmi.adr,
        dmi_dat_i => dmi.dat_i, -- MDTM is dmi master
        dmi_dat_o => dmi.dat_o, -- MDTM is dmi master
        dmi_rd => dmi.rd,
        dmi_wr => dmi.wr
      );
end RTL;
