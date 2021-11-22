----------------------------------------------------------------------------------
--  (C) 2020 Hochschule Augsburg, University of Applied Sciences
----------------------------------------------------------------------------------
--
-- Entity:        i2s_serializer_tb
--
-- Company:       Efficient Embedded Systems Group
--                University of Applied Sciences, Augsburg, Germany
--
-- Author:        Michael Schaeferling
-- Date:          2020-07-08
--
--  Description:  Testbench for the i2s_serializer module.
--
----------------------------------------------------------------------------------
--! @file  i2s_serializer_tb.vhd
--! @brief Testbench for the i2s_serializer module.
----------------------------------------------------------------------------------


library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;


entity i2s_serializer_tb is
end i2s_serializer_tb;


architecture TESTBENCH of i2s_serializer_tb is


component i2s_serializer is
  port (
    clk: in std_logic;
    reset: in std_logic;
    
    l_data: in std_logic_vector(15 downto 0);
    r_data: in std_logic_vector(15 downto 0);
    next_sample: out std_logic;
    
    i2s_mclk: out std_logic;
    i2s_bclk: out std_logic;
    i2s_pblrc: out std_logic;
    i2s_pbdat: out std_logic;
    i2s_mute: out std_logic
  );
end component i2s_serializer;

-- Clock period ...
constant period: time := 20 ns; -- 50MHz


signal clk : std_logic;
signal reset : std_logic;

signal l_data, r_data : std_logic_vector(15 downto 0);

signal next_sample : std_logic;

signal i2s_mclk : std_logic;
signal i2s_bclk : std_logic;
signal i2s_pblrc : std_logic;
signal i2s_pbdat : std_logic;
signal i2s_mute : std_logic;


begin

  uut_i2s_serializer : i2s_serializer
  port map(
    clk => clk,
    reset => reset,
    
    l_data => l_data,
    r_data => r_data,
    next_sample => next_sample,
    
    i2s_mclk => i2s_mclk,
    i2s_bclk => i2s_bclk,
    i2s_pblrc => i2s_pblrc,
    i2s_pbdat => i2s_pbdat,
    i2s_mute => i2s_mute
  );


-- Process for applying patterns
  process

    -- Helper to perform one clock cycle...
    procedure run_cycle is
    begin
      clk <= '0';
      wait for period / 2;
      clk <= '1';
      wait for period / 2;
    end procedure;

  begin
    l_data <= x"0000";
    r_data <= x"0000";
    
    for n in 1 to 2 loop run_cycle; end loop;
    reset <= '1';
    for n in 1 to 2 loop run_cycle; end loop;
    reset <= '0';
    
    for n in 1 to 3000 loop run_cycle; end loop;
    
    l_data <= x"ffff";
    r_data <= x"aaaa";
    
    for n in 1 to 2000 loop run_cycle; end loop;
    
    l_data <= x"aaaa";
    r_data <= x"5555";
    
    for n in 1 to 2000 loop run_cycle; end loop;
    
    l_data <= x"ffff";
    r_data <= x"ffff";
    
    for n in 1 to 2000 loop run_cycle; end loop;

    l_data <= x"0000";
    r_data <= x"0000";
    
    for n in 1 to 2000 loop run_cycle; end loop;
    
    l_data <= x"ffff";
    r_data <= x"ffff";
    
    for n in 1 to 2000 loop run_cycle; end loop;
    
    -- Print a note & finish simulation...
    assert false report "Simulation finished" severity note;
    wait;

  end process;


end TESTBENCH;

