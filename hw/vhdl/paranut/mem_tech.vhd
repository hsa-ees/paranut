--------------------------------------------------------------------------------
-- This file is part of the ParaNut project.
-- 
-- Copyright (C) 2013  Michael Seider, <michael.seider@hs-augsburg.de>
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
--  Component and type declarations for the mem_inferred modules
--
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;

package mem_tech is

    constant READ_FIRST  : natural range 0 to 2 := 0;
    constant WRITE_FIRST : natural range 0 to 2 := 1;
    constant NO_CHANGE   : natural range 0 to 2 := 2;

    component mem_sync_sp_inferred is
        generic (
                    AWIDTH : integer := 10;
                    DWIDTH : integer := 32;
                    COL_NUM : integer := 4;
                    INITD  : integer := 0;
                    WRITE_MODE : natural range 0 to 2 := 0
                );
        port (
                 clk   : in std_logic;
                 addr  : in std_logic_vector(AWIDTH-1 downto 0);
                 wr    : in std_logic;
                 we    : in std_logic_vector(COL_NUM-1 downto 0);
                 wdata : in std_logic_vector(DWIDTH-1 downto 0);
                 rdata : out std_logic_vector(DWIDTH-1 downto 0)
             );
    end component;

    component mem_sync_simple_dp_inferred is
        generic (
                    AWIDTH : natural := 10;
                    DWIDTH : natural := 32;
                    INITD  : integer := 0
                );
        port (
                 clk   : in std_logic;
                 raddr : in std_logic_vector(AWIDTH-1 downto 0);
                 waddr : in std_logic_vector(AWIDTH-1 downto 0);
                 we    : in std_logic;
                 wdata : in std_logic_vector(DWIDTH-1 downto 0);
                 rdata : out std_logic_vector(DWIDTH-1 downto 0)
             );
    end component;

    component mem_sync_true_dp_inferred is
        generic (
                    AWIDTH : natural := 10;
                    DWIDTH : natural := 32;
                    INITD  : integer := 0;
                    COL_NUM : integer := 4;
                    WRITE_MODE_1 : natural range 0 to 2 := 0;
                    WRITE_MODE_2 : natural range 0 to 2 := 0
                );
        port (
                 clk1   : in std_logic;
                 clk2   : in std_logic;
                 addr1  : in std_logic_vector(AWIDTH-1 downto 0);
                 addr2  : in std_logic_vector(AWIDTH-1 downto 0);
                 wr1    : in std_logic;
                 wr2    : in std_logic;
                 we1    : in std_logic_vector(COL_NUM-1 downto 0);
                 we2    : in std_logic_vector(COL_NUM-1 downto 0);
                 wdata1 : in std_logic_vector(DWIDTH-1 downto 0);
                 wdata2 : in std_logic_vector(DWIDTH-1 downto 0);
                 rdata1 : out std_logic_vector(DWIDTH-1 downto 0);
                 rdata2 : out std_logic_vector(DWIDTH-1 downto 0)
             );
    end component;

    component mem_3p_async_dist_inferred
        generic (
                    AWIDTH : integer := 5;
                    DWIDTH : integer := 32
                );
        port (
                 wclk    : in std_logic;
                 waddr   : in std_logic_vector(AWIDTH-1 downto 0);
                 wdata   : in std_logic_vector(DWIDTH-1 downto 0);
                 we      : in std_logic;
                 raddr1  : in std_logic_vector(AWIDTH-1 downto 0);
                 rdata1  : out std_logic_vector(DWIDTH-1 downto 0);
                 raddr2  : in std_logic_vector(AWIDTH-1 downto 0);
                 rdata2  : out std_logic_vector(DWIDTH-1 downto 0)
             );
    end component;

    component mem_3p_async_dist_rfirst_inferred
        generic (
                    AWIDTH : integer := 5;
                    DWIDTH : integer := 32
                );
        port (
                 wclk    : in std_logic;
                 waddr   : in std_logic_vector(AWIDTH-1 downto 0);
                 wdata   : in std_logic_vector(DWIDTH-1 downto 0);
                 we      : in std_logic;
                 raddr1  : in std_logic_vector(AWIDTH-1 downto 0);
                 rdata1  : out std_logic_vector(DWIDTH-1 downto 0);
                 raddr2  : in std_logic_vector(AWIDTH-1 downto 0);
                 rdata2  : out std_logic_vector(DWIDTH-1 downto 0)
             );
    end component;
    
    component mem_sync_read_wider_dp_inferred is
     generic (
                  DWIDTHW : integer := 21;
                  AWIDTHW : integer := 10;
                  DWIDTHR : integer := 16;
                  AWIDTHR : integer := 84
             );
      port (
                  clk   : in std_logic;
                  we    : in std_logic;
                  waddr : in std_logic_vector(AWIDTHW - 1 downto 0);
                  raddr : in std_logic_vector(AWIDTHR - 1 downto 0);
                  wdata : in std_logic_vector(DWIDTHW - 1 downto 0);
                  rdata : out std_logic_vector(DWIDTHR - 1 downto 0)
           );
    end component;

end package;
