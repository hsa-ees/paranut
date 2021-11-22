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
--  Load/store unit with store buffer
--
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;

library paranut;
use paranut.paranut_config.all;
use paranut.lsu.all;
use paranut.types.all;
use paranut.memu_lib.all;
use paranut.paranut_lib.all;

entity mlsu_wrapper is
    port (
             clk            : in std_logic;
             reset          : in std_logic;
             -- to EXU...
             lsui           : in lsu_in_type;
             lsuo           : out lsu_out_type;
             -- to MEMU/read port...
             rpi            : out readport_in_type;
             rpo            : in readport_out_type;
             -- to MEMU/write port...
             wpi            : out writeport_in_type;
             wpo            : in writeport_out_type;
             -- from CePU
             dcache_enable  : in std_logic
         );
end mlsu_wrapper;

architecture rtl of mlsu_wrapper is


  component MLsu is
    port (
      clk : IN STD_LOGIC;
      reset : IN STD_LOGIC;
      rd : IN STD_LOGIC;
      wr : IN STD_LOGIC;
      flush : IN STD_LOGIC;
      cache_op : IN STD_LOGIC_VECTOR (2 downto 0);
      lres_scond : IN STD_LOGIC;
      ack : OUT STD_LOGIC;
      align_err : OUT STD_LOGIC;
      scond_ok : OUT STD_LOGIC;
      width : IN STD_LOGIC_VECTOR (1 downto 0);
      exts : IN STD_LOGIC;
      adr : IN STD_LOGIC_VECTOR (31 downto 0);
      rdata : OUT STD_LOGIC_VECTOR (31 downto 0);
      wdata : IN STD_LOGIC_VECTOR (31 downto 0);
      dcache_enable : IN STD_LOGIC;
      rp_rd : OUT STD_LOGIC;
      rp_bsel : OUT STD_LOGIC_VECTOR (3 downto 0);
      rp_ack : IN STD_LOGIC;
      rp_adr : OUT STD_LOGIC_VECTOR (31 downto 0);
      rp_data : IN STD_LOGIC_VECTOR (31 downto 0);
      rp_direct : OUT STD_LOGIC;
      wp_wr : OUT STD_LOGIC;
      wp_bsel : OUT STD_LOGIC_VECTOR (3 downto 0);
      wp_ack : IN STD_LOGIC;
      wp_lres_scond : OUT STD_LOGIC;
      wp_scond_ok : IN STD_LOGIC;
      wp_cache_op : OUT STD_LOGIC_VECTOR (2 downto 0);
      wp_adr : OUT STD_LOGIC_VECTOR (31 downto 0);
      wp_data : OUT STD_LOGIC_VECTOR (31 downto 0);
      wp_direct : OUT STD_LOGIC
    );
  end component;
    
  signal lres_scond :  STD_LOGIC;
  
begin

   HLSMLsu : component MLsu
   port map (
       clk => clk,
       reset => reset,
       -- from EXU
       rd  => lsui.rd,
       wr  => lsui.wr,
       flush  =>  lsui.flush,
       cache_op  => lsui.cache_op,
       width  => lsui.width ,
       adr  => lsui.adr ,
       exts  =>  lsui.exts ,
       wdata  => lsui.wdata ,
       lres_scond  => lsui.lres_scond,
       dcache_enable => dcache_enable,
       -- to EXU
       ack  => lsuo.ack,
       align_err  => lsuo.align_err ,
       rdata  => lsuo.rdata ,
       scond_ok  => lsuo.scond_ok,
       -- to Readport
       rp_rd => rpi.port_rd,
       rp_adr => rpi.port_adr,
       rp_bsel  => rpi.port_bsel,
       rp_direct => rpi.port_direct,
       -- from Readport
       rp_data => rpo.port_data,
       rp_ack => rpo.port_ack,
       -- to Writeport
       wp_wr  => wpi.port_wr,
       wp_bsel  => wpi.port_bsel,
       wp_direct => wpi.port_direct,
       wp_lres_scond  => lres_scond,
       wp_cache_op  => wpi.port_cache_op ,
       wp_adr  =>  wpi.port_adr ,
       wp_data  =>  wpi.port_data ,
       -- from Writeport
       wp_ack  =>  wpo.port_ack ,
       wp_scond_ok  => rpo.port_scond_ok
   );
   
   -- Connected RP and WP get the same lres_scond 
   rpi.port_lres_scond <= lres_scond;
   wpi.port_lres_scond <= lres_scond;
   
   wpi.port_scond_ok <= rpo.port_scond_ok;

end rtl;
