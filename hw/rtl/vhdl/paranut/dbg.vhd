--------------------------------------------------------------------------------
-- This file is part of the ParaNut project.
-- 
-- Copyright (C) 2019  Alexander Bahle <alexander.bahle@hs-augsburg.de>
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
--  Component and type declarations for the ParaNut RISC-V debug support modules
--
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;

library paranut;
use paranut.memu_lib.all;
use paranut.paranut_config.all;
use paranut.types.all;

package dbg is

	-- DMI type
	type dmi_type is record
			adr : STD_LOGIC_VECTOR (5 downto 0);
			dat_i : STD_LOGIC_VECTOR (31 downto 0);
			dat_o : STD_LOGIC_VECTOR (31 downto 0);
			rd :  STD_LOGIC;
			wr : STD_LOGIC;
	end record;
	
	-- DM output type
	type dm_out_type is record
			dbg_request : STD_LOGIC_VECTOR (CFG_NUT_CPU_CORES-1 downto 0);
			dbg_reset : STD_LOGIC;			
	end record;	

 	-- JTAG input type
 	type jtag_in_type is record
			tck : STD_LOGIC;
			tms : STD_LOGIC;
			tdi : STD_LOGIC;
	end record;
	
	-- JTAG output type
	type jtag_out_type is record
			tdo : STD_LOGIC;
	end record;
	
    component dbg_wrapper
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
			    -- special dbg_reset output
			    --dbg_reset : out STD_LOGIC
             );
    end component;


end package;
