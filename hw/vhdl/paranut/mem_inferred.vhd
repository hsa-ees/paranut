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
--  Different types of synchronous and asynchronous memory
--
--------------------------------------------------------------------------------

-- single port synchronous memory (1 port for reading and writing)
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity mem_sync_sp_inferred is
    generic (
                AWIDTH : natural := 10;
                DWIDTH : natural := 32;
                COL_NUM : integer := 4;
                INITD  : integer := 0;
                WRITE_MODE : natural range 0 to 2 := 0 -- 0: READ_FIRST, 1: WRITE_FIRST, 2: NO_CHANGE
            );
    port (
             clk   : in std_logic;
             addr  : in std_logic_vector(AWIDTH-1 downto 0);
             wr    : in std_logic;
             we    : in std_logic_vector(COL_NUM-1 downto 0);
             wdata : in std_logic_vector(DWIDTH-1 downto 0);
             rdata : out std_logic_vector(DWIDTH-1 downto 0)
         );
end mem_sync_sp_inferred;

architecture rtl of mem_sync_sp_inferred is
    constant CWIDTH : natural := DWIDTH/COL_NUM;

    type mem_type is array (0 to 2**AWIDTH - 1) of std_logic_vector(DWIDTH-1 downto 0);
    shared variable mem : mem_type := (others => std_logic_vector(to_unsigned(INITD, DWIDTH)));

    attribute ram_style : string;
    attribute ram_style of mem : variable is "block";

begin
    process (clk)
    begin
        if (clk'event and clk = '1') then
            if (WRITE_MODE = 0) then
                rdata <= mem(to_integer(unsigned(addr)));
                if (wr = '1') then
                    for i in 0 to COL_NUM-1 loop
                        if (we(i) = '1') then
                            mem(to_integer(unsigned(addr)))((i + 1) * CWIDTH-1 downto i * CWIDTH) := wdata((i + 1) * CWIDTH-1 downto i * CWIDTH);
                        end if;
                    end loop;
                end if;
            end if;
            if (WRITE_MODE = 1) then
                if (wr = '1') then
                    for i in 0 to COL_NUM-1 loop
                       if (we(i) = '1') then
                           mem(to_integer(unsigned(addr)))((i + 1) * CWIDTH-1 downto i * CWIDTH) := wdata((i + 1) * CWIDTH-1 downto i * CWIDTH);
                       end if;
                    end loop;
                end if;
                rdata <= mem(to_integer(unsigned(addr)));
            end if;
            if (WRITE_MODE = 2) then
                if (wr = '1') then
                    for i in 0 to COL_NUM-1 loop
                       if (we(i) = '1') then
                           mem(to_integer(unsigned(addr)))((i + 1) * CWIDTH-1 downto i * CWIDTH) := wdata((i + 1) * CWIDTH-1 downto i * CWIDTH);
                       end if;
                    end loop;
                else
                    rdata <= mem(to_integer(unsigned(addr)));
                end if;
            end if;
        end if;
    end process;
end rtl;

--------------------------------------------------------------------------------

-- simple dual port synchronous memory (one port for reading only, the other
-- for writing only)
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity mem_sync_simple_dp_inferred is
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
end mem_sync_simple_dp_inferred;

architecture rtl of mem_sync_simple_dp_inferred is

    type mem_type is array (0 to 2**AWIDTH - 1) of std_logic_vector(DWIDTH-1 downto 0);
    shared variable mem : mem_type := (others => std_logic_vector(to_unsigned(INITD, DWIDTH)));

    attribute ram_style : string;
    attribute ram_style of mem : variable is "block";

begin
    process (clk)
    begin
        if (clk'event and clk = '1') then
            rdata <= mem(to_integer(unsigned(raddr)));
            if (we = '1') then
                mem(to_integer(unsigned(waddr))) := wdata;
            end if;
        end if;
    end process;
end rtl;


--------------------------------------------------------------------------------

-- simple dual port synchronous memory with asyncronous port size - read wider
-- (one port for reading only, the other for writing only)
library ieee;
use ieee.std_logic_1164.all;
use ieee.std_logic_unsigned.all;
use ieee.std_logic_arith.all;

entity mem_sync_read_wider_dp_inferred is
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
end mem_sync_read_wider_dp_inferred;

architecture behavioral of mem_sync_read_wider_dp_inferred is
  function max(L, R : INTEGER) return INTEGER is
  begin
    if L > R then
      return L;
    else
      return R;
    end if;
  end;
  
  function min(L, R : INTEGER) return INTEGER is
  begin
    if L < R then
      return L;
    else
      return R;
    end if;
  end;
  
  function log2(val : INTEGER) return natural is
    variable res : natural;
  begin
    for i in 0 to 31 loop
      if (val <= (2 ** i)) then
        res := i;
        exit;
      end if;
    end loop;
    return res;
  end function log2;
  
  constant minWIDTH : integer := min(DWIDTHW, DWIDTHR);
  constant maxWIDTH : integer := max(DWIDTHW, DWIDTHR);
  constant maxAWIDTH : integer := max(AWIDTHW, AWIDTHR);
  constant maxSIZE : integer :=  2**maxAWIDTH;
  constant RATIO : integer := maxWIDTH / minWIDTH;
  
  -- An asymmetric RAM is modeled in a similar way as a symmetric RAM, with an
  -- array of array object. Its aspect ratio corresponds to the port with the
  -- lower data width (larger depth)
  type mem_type is array (0 to maxSIZE - 1) of std_logic_vector(minWIDTH - 1 downto 0);
  signal mem : mem_type := (others => (others => '0'));
  
  attribute ram_style : string;
  attribute ram_style of mem : signal is "block";
  
begin

  -- Read process
  process(clk)
  begin
    if rising_edge(clk) then
      if (RATIO > 1) then
          for i in 0 to RATIO - 1 loop
            rdata((i + 1) * minWIDTH - 1 downto i * minWIDTH) <= mem(conv_integer(raddr & conv_std_logic_vector(i, log2(RATIO))));
          end loop;
      else 
          -- Allow same read and write size (necessary for some configurations)
          rdata <= mem(conv_integer(raddr));
      end if;
    end if;
  end process;
  
  -- Write process
  process(clk)
  begin
    if rising_edge(clk) then
          if we = '1' then
            mem(conv_integer(waddr)) <= wdata;
          end if;
    end if;
  end process;

end behavioral;
--------------------------------------------------------------------------------

-- true dual port synchronous memory (both ports can read/write with different clocks)
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity mem_sync_true_dp_inferred is
    generic (
                AWIDTH : natural := 10;
                DWIDTH : natural := 32;
                INITD  : integer := 0;
                COL_NUM : integer := 4;
                WRITE_MODE_1 : natural range 0 to 2 := 0; -- 0: READ_FIRST, 1: WRITE_FIRST, 2: NO_CHANGE
                WRITE_MODE_2 : natural range 0 to 2 := 0 -- 0: READ_FIRST, 1: WRITE_FIRST, 2: NO_CHANGE
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
end mem_sync_true_dp_inferred;

architecture rtl of mem_sync_true_dp_inferred is
    constant CWIDTH : natural := 8;

    type mem_type is array (0 to 2**AWIDTH - 1) of std_logic_vector(DWIDTH-1 downto 0);
    shared variable mem : mem_type := (others => std_logic_vector(to_unsigned(INITD, DWIDTH)));

    attribute ram_style : string;
    attribute ram_style of mem : variable is "block";

begin

    process (clk1)
    begin
        if (clk1'event and clk1 = '1') then
            if (WRITE_MODE_1 = 0) then
                rdata1 <= mem(to_integer(unsigned(addr1)));
                if wr1 = '1' then
                  for i in 0 to COL_NUM-1 loop
                      if (we1(i) = '1') then
                          mem(to_integer(unsigned(addr1)))((i + 1) * CWIDTH-1 downto i * CWIDTH) := wdata1((i + 1) * CWIDTH-1 downto i * CWIDTH);
                      end if;
                  end loop;
                end if;
            end if;
            if (WRITE_MODE_1 = 1) then
                if wr1 = '1' then
                  for i in 0 to COL_NUM-1 loop
                      if (we1(i) = '1') then
                          mem(to_integer(unsigned(addr1)))((i + 1) * CWIDTH-1 downto i * CWIDTH) := wdata1((i + 1) * CWIDTH-1 downto i * CWIDTH);
                      end if;
                  end loop;
                end if;
                rdata1 <= mem(to_integer(unsigned(addr1)));
            end if;
            if (WRITE_MODE_1 = 2) then
                if wr1 = '1' then
                    for i in 0 to COL_NUM-1 loop
                        if (we1(i) = '1') then
                            mem(to_integer(unsigned(addr1)))((i + 1) * CWIDTH-1 downto i * CWIDTH) := wdata1((i + 1) * CWIDTH-1 downto i * CWIDTH);
                        end if;
                    end loop;
                else
                    rdata1 <= mem(to_integer(unsigned(addr1)));  
                end if;
            end if;
        end if;
    end process;
    process (clk2)
    begin
        if (clk2'event and clk2 = '1') then
            if (WRITE_MODE_2 = 0) then
                rdata2 <= mem(to_integer(unsigned(addr2)));
                if wr2 = '1' then
                  for i in 0 to COL_NUM-1 loop
                      if (we2(i) = '1') then
                          mem(to_integer(unsigned(addr2)))((i + 1) * CWIDTH-1 downto i * CWIDTH) := wdata2((i + 1) * CWIDTH-1 downto i * CWIDTH);
                      end if;
                  end loop;
                end if;
            end if;
            if (WRITE_MODE_2 = 1) then
                if wr2 = '1' then
                  for i in 0 to COL_NUM-1 loop
                      if (we2(i) = '1') then
                          mem(to_integer(unsigned(addr2)))((i + 1) * CWIDTH-1 downto i * CWIDTH) := wdata2((i + 1) * CWIDTH-1 downto i * CWIDTH);
                      end if;
                  end loop;
                end if;
                rdata2 <= mem(to_integer(unsigned(addr2)));
            end if;
            if (WRITE_MODE_2 = 2) then
                if wr2 = '1' then
                  for i in 0 to COL_NUM-1 loop
                      if (we2(i) = '1') then
                          mem(to_integer(unsigned(addr2)))((i + 1) * CWIDTH-1 downto i * CWIDTH) := wdata2((i + 1) * CWIDTH-1 downto i * CWIDTH);
                      end if;
                  end loop;
                else
                  rdata2 <= mem(to_integer(unsigned(addr2)));
                end if;
            end if;
        end if;
    end process;
end rtl;

--------------------------------------------------------------------------------

-- three port asynchronous memory (2 read ports, 1 write port, used for register file)
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity mem_3p_async_dist_inferred is
    generic (
                AWIDTH : natural := 5;
                DWIDTH : natural := 32
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
end mem_3p_async_dist_inferred;

architecture rtl of mem_3p_async_dist_inferred is

    type mem_type is array (0 to 2**AWIDTH - 1) of std_logic_vector(DWIDTH-1 downto 0);
    signal mem : mem_type;

    attribute ram_style : string;
    attribute ram_style of mem : signal is "distributed";

begin
    process (wclk)
    begin
        if (wclk'event and wclk = '1') then
            if (we = '1') then
                mem(to_integer(unsigned(waddr))) <= wdata;
            end if;
        end if;
    end process;

    rdata1 <= wdata when (we = '1' and waddr = raddr1) else
              mem(to_integer(unsigned(raddr1)));
    rdata2 <= wdata when (we = '1' and waddr = raddr2) else
              mem(to_integer(unsigned(raddr2)));
end rtl;

--------------------------------------------------------------------------------

-- three port asynchronous memory (2 read ports, 1 write port) with read first mode
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity mem_3p_async_dist_rfirst_inferred is
    generic (
                AWIDTH : natural := 5;
                DWIDTH : natural := 32
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
end mem_3p_async_dist_rfirst_inferred;

architecture rtl of mem_3p_async_dist_rfirst_inferred is

    type mem_type is array (0 to 2**AWIDTH - 1) of std_logic_vector(DWIDTH-1 downto 0);
    signal mem : mem_type := (others => (others => '0'));

    attribute ram_style: string;
    attribute ram_style of mem : signal is "distributed";

begin
    process (wclk)
    begin
        if (wclk'event and wclk = '1') then
            if (we = '1') then
                mem(to_integer(unsigned(waddr))) <= wdata;
            end if;
        end if;
    end process;

    rdata1 <= mem(to_integer(unsigned(raddr1)));
    rdata2 <= mem(to_integer(unsigned(raddr2)));
end rtl;
