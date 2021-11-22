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
--  Interrupt Controller 
--
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library paranut;
use paranut.paranut_config.all;
use paranut.intc.all;
use paranut.types.all;
use paranut.paranut_lib.all;

entity mintc_wrapper is
  port (
     clk            : in std_logic;
     reset          : in std_logic;
     -- from CePU
     intci          : in intc_in_type;
     -- to CePU
     intco          : out intc_out_type;
     -- External interrupt input
     ex_int          : in std_logic_vector (CFG_NUT_EX_INT-1 downto 0)
  );
end mintc_wrapper;

architecture rtl of mintc_wrapper is

  component MIntC is
    port (
      clk : IN STD_LOGIC;
      reset : IN STD_LOGIC;
      ir_request : OUT STD_LOGIC;
      ir_id : OUT STD_LOGIC_VECTOR (4 downto 0);
      ir_ack : IN STD_LOGIC;
      ir_enable : IN STD_LOGIC;
      ex_int : IN STD_LOGIC_VECTOR (CFG_NUT_EX_INT-1  downto 0)
    );
  end component;

begin

  HLSMIntC : component MIntC
  port map (
      clk => clk,
      reset => reset,
      -- to CePU
      ir_request => intco.ir_request,
      ir_id => intco.ir_id,
      -- from CePU
      ir_ack => intci.ir_ack,
      ir_enable => intci.ir_enable,
      -- External interrupt input
      ex_int => ex_int
  );

end rtl;
