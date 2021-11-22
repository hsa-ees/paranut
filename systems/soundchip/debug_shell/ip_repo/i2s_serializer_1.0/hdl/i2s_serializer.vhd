----------------------------------------------------------------------------------
--  (C) 2020 Hochschule Augsburg, University of Applied Sciences
----------------------------------------------------------------------------------
--
-- Entity:        i2s_serializer
--
-- Company:       Efficient Embedded Systems Group
--                University of Applied Sciences, Augsburg, Germany
--
-- Author:        Michael Schaeferling
-- Date:          2020-07-08
--
--  Description:  This module interfaces I2S Audio chips.
--                The module is tested with an Analog Devices SSM2603 
--                which is used on the Zybo board.
--
--                Supported audio modes (select by generic "AUDIO_MODE"):
--                  
--                  - Mode 0:
--                    - Sample rate: 48kHz
--                    - Channels: 2
--                    - Resolution (per channel): 16bit
--                  
--                  - Mode 1:
--                    - Sample rate: 11.025kHz
--                    - Channels: 2
--                    - Resolution (per channel): 16bit
--
--
----------------------------------------------------------------------------------
--! @file  i2s_serializer.vhd
--! @brief I2S serializer module with included clock generator.
----------------------------------------------------------------------------------


library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;


entity i2s_serializer is
  generic (
    AUDIO_MODE : integer := 0
  );
  port (
    clk: in std_logic;
    reset_n: in std_logic;
    
    l_data: in std_logic_vector(15 downto 0);
    r_data: in std_logic_vector(15 downto 0);
    next_sample: out std_logic;
    
    i2s_mclk: out std_logic;
    i2s_bclk: out std_logic;
    i2s_pblrc: out std_logic;
    i2s_pbdat: out std_logic;
    i2s_mute: out std_logic
  );
end i2s_serializer;


architecture RTL of i2s_serializer is


constant c_clk_freq_mhz : real := 25.0;


type audio_settings_t is record
  mclk_freq_mhz : real;
  bclk_div_log2 : integer;
end record audio_settings_t;

type audio_mode_array_t is array (0 to 1) of audio_settings_t;

constant c_audio_settings : audio_mode_array_t := (
--         MCLK_FREQ_MHZ , BCLK_DIV_LOG2
   0  => ( 12.2880       , 3             ), -- 48.000kHz / 2ch / 16bit/ch
   1  => ( 11.2896       , 5             )  -- 11.025kHz / 2ch / 16bit/ch
);


component i2s_clocking is
  generic (
    CLK_IN_FREQ_MHZ : real;
    CLK_OUT_FREQ_MHZ : real
  );
  port (
    clk_in : in  std_logic;
    reset  : in  std_logic;

    clk_out : out std_logic;
    locked : out std_logic
  );
end component;

signal reset : std_logic := '0';

signal mclk : std_logic := '0';
signal mclk_gen : std_logic := '0';
signal r_mclk_gen_counter : unsigned(1 downto 0) := (others => '0');
signal i2s_clocking_0_locked : std_logic;

signal r_sample_lr_shadow : std_logic_vector(31 downto 0) := (others => '0');
signal r_sample_lr_shadow_sync_stage : std_logic_vector(31 downto 0) := (others => '0');
signal r_bit_counter : unsigned(4 downto 0) := (others => '0');

signal r_bclk, r_bclk_last : std_logic := '0';

signal r_bclk_gen_divider : unsigned((c_audio_settings(AUDIO_MODE).bclk_div_log2-1)-1 downto 0) := (others => '0');

signal r_next_sample_mclk : std_logic;
signal r_next_sample_sync_stage, r_next_sample, r_next_sample_last : std_logic;

begin

  reset <= not(reset_n);


  i2s_clocking_0 : i2s_clocking
  generic map(
    CLK_IN_FREQ_MHZ => c_clk_freq_mhz,
    CLK_OUT_FREQ_MHZ => c_audio_settings(AUDIO_MODE).mclk_freq_mhz
  )
  port map(
    clk_in => clk,
    reset  => reset,

    clk_out => mclk,
    locked  => i2s_clocking_0_locked
  );


  -- For simulation or when i2s_clocking cannot be used due to hardware limitations
  --~ p_mclk_gen : process (clk)
  --~ begin
    --~ if rising_edge(clk) then
      --~ --r_mclk_gen_counter <= not(r_mclk_gen_counter);
      --~ r_mclk_gen_counter <= r_mclk_gen_counter + 1;
      --~ if (r_mclk_gen_counter = 0) then
        --~ mclk_gen <= not(mclk_gen);
      --~ end if;
    --~ end if;
  --~ end process;


  p_next_sample_gen : process (clk)
  begin
    if rising_edge(clk) then
    --~ if falling_edge(clk) then
      -- default next_sample value:
      next_sample <= '0';
      
      -- sync mclk to clk domain:
      r_next_sample_sync_stage <= r_next_sample_mclk;
      r_next_sample <= r_next_sample_sync_stage;
      
      -- save last value in order to detect the rising edge of "next_sample" coming in from mclk domain:
      r_next_sample_last <= r_next_sample;
      
      -- on rising edge of r_next_sample set next_sample to 1:
      if ( (r_next_sample = '1') and (r_next_sample_last = '0') ) then
        next_sample <= '1';
      end if;
    end if;
  end process;


  p_bclk_gen : process (mclk)
  begin
    if rising_edge(mclk) then
      r_bclk_gen_divider <= r_bclk_gen_divider + 1;
      r_bclk_last <= r_bclk;
      if (r_bclk_gen_divider = 0) then
        r_bclk <= not(r_bclk);
      end if;
    end if;
  end process;


  p_sample_lr_shadow_shift : process (mclk)
  begin
    if rising_edge(mclk) then

      r_next_sample_mclk <= '0';
      r_sample_lr_shadow_sync_stage <= l_data & r_data;

      i2s_bclk <= r_bclk;

      -- On falling edge of BCLK ...
      --   .. increment bit_counter
      --   .. and shift-left the shadowed data
      if (r_bclk = '0') and (r_bclk_last = '1') then
        if (r_bit_counter = 0) then
          r_next_sample_mclk <= '1';
          r_sample_lr_shadow <= r_sample_lr_shadow_sync_stage;
        else 
          r_sample_lr_shadow <= r_sample_lr_shadow(30 downto 0) & '0';
        end if;

        r_bit_counter <= r_bit_counter + 1;

      end if;
    end if;
  end process;


i2s_mclk <= mclk;
i2s_mute <= not(reset);  -- mute is active low -> enable output by sending '1'

i2s_pblrc <= r_bit_counter(4);
i2s_pbdat <= r_sample_lr_shadow(31);


end RTL;

