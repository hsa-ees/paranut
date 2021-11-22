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
use paranut.ifu.all;
use paranut.types.all;
use paranut.memu_lib.all;
use paranut.paranut_lib.all;

entity mifu_wrapper is
    generic (
              IFU_BUF_SIZE_MIN : integer range 4 to 16 := 4
            );
    port (
             clk            : in std_logic;
             reset          : in std_logic;
             -- to EXU...
             ifui           : in ifu_in_type;
             ifuo           : out ifu_out_type;
             -- to MEMU (read port)...
             rpi            : out readport_in_type;
             rpo            : in readport_out_type;
             -- from CePU...
             icache_enable  : in std_logic
         );
end mifu_wrapper;

architecture rtl of mifu_wrapper is

  component MIfu is
    port (
    clk : IN STD_LOGIC;
    reset : IN STD_LOGIC;
    rp_rd : OUT STD_LOGIC;
    rp_ack : IN STD_LOGIC;
    rp_adr : OUT STD_LOGIC_VECTOR (31 downto 0);
    rp_data : IN STD_LOGIC_VECTOR (31 downto 0);
    rp_direct : OUT STD_LOGIC;
    next_r : IN STD_LOGIC;
    jump : IN STD_LOGIC;
    flush : IN STD_LOGIC;
    jump_adr : IN STD_LOGIC_VECTOR (31 downto 0);
    ir : OUT STD_LOGIC_VECTOR (31 downto 0);
    pc : OUT STD_LOGIC_VECTOR (31 downto 0);
    npc : OUT STD_LOGIC_VECTOR (31 downto 0);
    ir_valid : OUT STD_LOGIC;
    npc_valid : OUT STD_LOGIC;
    icache_enable : IN STD_LOGIC
    );
  end component;

begin

  HLSMifu : component MIfu
  port map (
      clk => clk,
      reset => reset,
      -- to Readport
      rp_rd => rpi.port_rd,
      rp_adr => rpi.port_adr,
      rp_direct => rpi.port_direct,
      -- from Readport
      rp_data => rpo.port_data,
      rp_ack => rpo.port_ack,
      -- from Exu
      next_r => ifui.nexti,
      jump => ifui.jump,
      jump_adr => ifui.jump_adr,
      flush => ifui.flush,
      icache_enable => icache_enable,
      -- to Exu
      ir => ifuo.ir,
      pc => ifuo.pc,
      npc => ifuo.npc,
      ir_valid => ifuo.ir_valid,
      npc_valid => ifuo.npc_valid
  );

  -- only read words
  rpi.port_bsel <= "1111";

end rtl;
