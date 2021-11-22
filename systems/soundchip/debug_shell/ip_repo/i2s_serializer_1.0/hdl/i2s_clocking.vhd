----------------------------------------------------------------------------------
--  (C) 2020 Hochschule Augsburg, University of Applied Sciences
----------------------------------------------------------------------------------
--
-- Entity:        i2s_clocking
--
-- Company:       Efficient Embedded Systems Group
--                University of Applied Sciences, Augsburg, Germany
--
-- Author:        Michael Schaeferling
-- Date:          2020-07-08
--
--  Description:  This module generates the Master Clock for an I2S Audio Module, 
--                using the input clock as reference. The output clock itself is 
--                generated using the Xilinx "MMCME2_BASE" module.
--
----------------------------------------------------------------------------------
--! @file  i2s_clocking.vhd
--! @brief Clock generator module for I2S.
----------------------------------------------------------------------------------


library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.MATH_REAL.ALL;


-- Following dependencies are needed for Xilinx specific components (also for synthesis)
library UNISIM;
use UNISIM.VComponents.all;


entity i2s_clocking is
  generic (
    CLK_IN_FREQ_MHZ : real;
    CLK_OUT_FREQ_MHZ : real -- 12.288MHz
  );
  port (
    clk_in : in  std_logic;
    reset  : in  std_logic;

    clk_out : out std_logic;
    locked : out std_logic
  );
end i2s_clocking;

architecture RTL of i2s_clocking is


signal clk_fb_out, clk_fb_in : std_logic;
signal clk_gen : std_logic;


type clk_settings_t is record
  divide_f_mclk : real;
  divide_bclk : integer; -- not used
end record clk_settings_t;

type clk_settings_array_t is array (0 to 3) of clk_settings_t;

constant c_clk_out_freq_mhz_min : real := 10.0;
constant c_clk_out_freq_mhz_max : real := 40.0;
constant c_clk_settings : clk_settings_array_t := ((60.0, 0), -- (0):  10 -  <20 MHz clk_out
                                                   (30.0, 0), -- (1):  20 -  <30 MHz clk_out
                                                   (30.0, 0), -- (2):  30 -  <40 MHz clk_out
                                                   (30.0, 0)  -- (3):         40 MHz clk_out
                                                  );

constant c_clock_out_range : integer := INTEGER(FLOOR(((CLK_OUT_FREQ_MHZ - c_clk_out_freq_mhz_min) / c_clk_out_freq_mhz_min)));
constant c_clkfbout_mult_f : real := CLK_OUT_FREQ_MHZ / CLK_IN_FREQ_MHZ * c_clk_settings(c_clock_out_range).divide_f_mclk;

constant c_clk_in_period : real := 1000.0 / CLK_IN_FREQ_MHZ;

attribute CLOCK_BUFFER_TYPE : string;
attribute CLOCK_BUFFER_TYPE of clk_gen: signal is "NONE";


begin


-- Valid VCO values for Zynq7010 (Zybo): min 600 MHz / max 1200MHz;
ASSERT ((CLK_OUT_FREQ_MHZ >= c_clk_out_freq_mhz_min) and (CLK_OUT_FREQ_MHZ <= c_clk_out_freq_mhz_max))
  REPORT "Output clock must be in range " & real'image(c_clk_out_freq_mhz_min) & " .. " & real'image(c_clk_out_freq_mhz_max) & " MHz!"
  SEVERITY FAILURE;


clken_0: MMCME2_BASE
  generic map(
    BANDWIDTH            => "OPTIMIZED",
    STARTUP_WAIT         => FALSE,

    CLKIN1_PERIOD        => c_clk_in_period,
    REF_JITTER1          => 0.010,

    DIVCLK_DIVIDE        => 1,

    CLKFBOUT_MULT_F      => c_clkfbout_mult_f,
    CLKFBOUT_PHASE       => 0.000,

    CLKOUT0_DIVIDE_F     => c_clk_settings(c_clock_out_range).divide_f_mclk,
    CLKOUT0_DUTY_CYCLE   => 0.500,
    CLKOUT0_PHASE        => 0.000
  )
  port map
  (
    RST                 => reset,
    LOCKED              => locked,
    PWRDWN              => '0',

    CLKIN1              => clk_in,

    CLKFBOUT            => clk_fb_out,
    CLKFBOUTB           => open,
    CLKFBIN             => clk_fb_in,

    CLKOUT0             => clk_out,
    CLKOUT0B            => open,

    CLKOUT1             => open,
    CLKOUT1B            => open,

    CLKOUT2             => open,
    CLKOUT2B            => open,

    CLKOUT3             => open,
    CLKOUT3B            => open,

    CLKOUT4             => open,
    CLKOUT5             => open,
    CLKOUT6             => open
  );


clk_fb_deskew: BUFR
  generic map (
    BUFR_DIVIDE => "1", 
    SIM_DEVICE => "7SERIES" 
  )
  port map (
    I => clk_fb_out,
    O => clk_fb_in,
    CE => '1',
    CLR => '0'
  );


end RTL;
