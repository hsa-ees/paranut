---------------------------------------------------------------------------
--
--  This file is part of the ParaNut project.
--
--  (C) 2020 Michael Schaeferling <michael.schaeferling@hs-augsburg.de>
--           Gundolf Kiefer <gundolf.kiefer@hs-augsburg.de>
--           Efficient Embedded Systems Group
--           Hochschule Augsburg, University of Applied Sciences
--
--  Description:
--    Top-level entity for the ParaNut SoundChip system.
--
--
--
--  Main components
--  ---------------
--
--  The following submodules must be provided outside:
--
--  a) SOUND_CORE: The sound core module.
--
--  b) SOUND_CHIP_DEBUG: The debug system, provided in a separate file (.dcp).
--      The debug system provides all components needed to operate the 
--      sound core module:
--        - AXI bus infrastructure: Interconnects all debug system components:
--            - Zynq PS7
--            - ParaNut processor
--            - AXI-Wishbone-Bridge
--        - Zynq PS7: Operates the firmware in order to upload ParaNut binaries etc.
--        - ParaNut: The ParaNut processor instance, running the ParaNut binary.
--                   The ParaNut interacts with the sound core module to generate sound.
--        - AXI-Wishbone-Bridge: connects the sound core module to the AXI bus.
--        - I2S serializer module: receives sound data from the sound core module
--                   and generates I2S signals to drive an external audio chip.
--
--
---------------------------------------------------------------------------



---------------------------------------------------------------------------
--
--  Module: SOUND_CHIP_SYSTEM (Top-Level)
--
--
--  Description:
--    Top-level module for the ParaNut SoundChip system.
--
---------------------------------------------------------------------------

library IEEE;
use IEEE.STD_LOGIC_1164.ALL;


entity SOUND_CHIP_SYSTEM is
  port (
    -- Board LEDs, buttons and switches...
    led: out std_logic_vector (3 downto 0);
    btn: in std_logic_vector (3 downto 0);
    sw : in std_logic_vector (3 downto 0);

    -- Fixed I/O ports for the PS to be forwarded from PARANUT_DEBUG...
    DDR_addr : inout STD_LOGIC_VECTOR ( 14 downto 0 );
    DDR_ba : inout STD_LOGIC_VECTOR ( 2 downto 0 );
    DDR_cas_n : inout STD_LOGIC;
    DDR_ck_n : inout STD_LOGIC;
    DDR_ck_p : inout STD_LOGIC;
    DDR_cke : inout STD_LOGIC;
    DDR_cs_n : inout STD_LOGIC;
    DDR_dm : inout STD_LOGIC_VECTOR ( 3 downto 0 );
    DDR_dq : inout STD_LOGIC_VECTOR ( 31 downto 0 );
    DDR_dqs_n : inout STD_LOGIC_VECTOR ( 3 downto 0 );
    DDR_dqs_p : inout STD_LOGIC_VECTOR ( 3 downto 0 );
    DDR_odt : inout STD_LOGIC;
    DDR_ras_n : inout STD_LOGIC;
    DDR_reset_n : inout STD_LOGIC;
    DDR_we_n : inout STD_LOGIC;
    FIXED_IO_ddr_vrn : inout STD_LOGIC;
    FIXED_IO_ddr_vrp : inout STD_LOGIC;
    FIXED_IO_mio : inout STD_LOGIC_VECTOR ( 53 downto 0 );
    FIXED_IO_ps_clk : inout STD_LOGIC;
    FIXED_IO_ps_porb : inout STD_LOGIC;
    FIXED_IO_ps_srstb : inout STD_LOGIC;
    
    SSM2603_IIC_scl_io : inout STD_LOGIC;
    SSM2603_IIC_sda_io : inout STD_LOGIC;
    
    i2s_bclk:  out std_logic;
    i2s_mclk:  out std_logic;
    i2s_mute:  out std_logic;
    i2s_pbdat: out std_logic;
    i2s_pblrc: out std_logic;

    tck: in  std_logic;
    tms: in  std_logic;
    tdi: in  std_logic;
    tdo: out std_logic

    --~ debug: out std_logic_vector (7 downto 0)
  );
end SOUND_CHIP_SYSTEM;



architecture RTL of SOUND_CHIP_SYSTEM is

  ----------------------------------------------------------
  --
  --  Component declarations
  --
  ----------------------------------------------------------

  -- Module: SOUND_CORE ...
  component SOUND_CORE is
    port (
      wb_clk_i:  in  std_logic;
      wb_rst_i:  in  std_logic;
      wb_stb_i:  in  std_logic;
      wb_cyc_i:  in  std_logic;
      wb_we_i:   in  std_logic;
      wb_cti_i:  in  std_logic_vector (2 downto 0);
      wb_bte_i:  in  std_logic_vector (1 downto 0);
      wb_sel_i:  in  std_logic_vector (3 downto 0);
      wb_ack_o:  out std_logic;
      wb_err_o:  out std_logic;
      wb_rty_o:  out std_logic;
      wb_adr_i:  in  std_logic_vector (31 downto 0);
      wb_dat_i:  in  std_logic_vector (31 downto 0);
      wb_dat_o:  out std_logic_vector (31 downto 0);

      audio_l:   out std_logic_vector (15 downto 0);
      audio_r:   out std_logic_vector (15 downto 0)
    );
  end component SOUND_CORE;

  -- Module: SOUND_CHIP_DEBUG ...
  component SOUND_CHIP_DEBUG_WRAPPER is
    port (
      led: out std_logic_vector (3 downto 0);
      btn: in std_logic_vector (3 downto 0);
      sw : in std_logic_vector (3 downto 0);

      DDR_cas_n:    inout std_logic;
      DDR_cke:      inout std_logic;
      DDR_ck_n:     inout std_logic;
      DDR_ck_p:     inout std_logic;
      DDR_cs_n:     inout std_logic;
      DDR_reset_n:  inout std_logic;
      DDR_odt:      inout std_logic;
      DDR_ras_n:    inout std_logic;
      DDR_we_n:     inout std_logic;
      DDR_ba:       inout std_logic_vector (2 downto 0);
      DDR_addr:     inout std_logic_vector (14 downto 0);
      DDR_dm:       inout std_logic_vector (3 downto 0);
      DDR_dq:       inout std_logic_vector (31 downto 0);
      DDR_dqs_n:    inout std_logic_vector (3 downto 0);
      DDR_dqs_p:    inout std_logic_vector (3 downto 0);

      FIXED_IO_mio:       inout std_logic_vector (53 downto 0);
      FIXED_IO_ddr_vrn:   inout std_logic;
      FIXED_IO_ddr_vrp:   inout std_logic;
      FIXED_IO_ps_srstb:  inout std_logic;
      FIXED_IO_ps_clk:    inout std_logic;
      FIXED_IO_ps_porb:   inout std_logic;

      SSM2603_IIC_scl_io: inout std_logic;
      SSM2603_IIC_sda_io: inout std_logic;

      i2s_mclk:  out std_logic;
      i2s_bclk:  out std_logic;
      i2s_pblrc: out std_logic;
      i2s_pbdat: out std_logic;
      i2s_mute:  out std_logic;

      wb_clk_o:    out  std_logic;
      wb_rst_o:    out  std_logic;
      wb_stb_o:    out  std_logic;
      wb_cyc_o:    out  std_logic;
      wb_we_o:     out  std_logic;
      wb_cti_o:    out  std_logic_vector (2 downto 0);
      wb_bte_o:    out  std_logic_vector (1 downto 0);
      wb_sel_o:    out  std_logic_vector (3 downto 0);
      wb_ack_i:    in std_logic;
      wb_stall_i:  in std_logic;
      --wb_err_i:  in std_logic;
      --wb_rty_i:  in std_logic;
      wb_adr_o:    out  std_logic_vector (31 downto 0);
      wb_dat_o:    out  std_logic_vector (31 downto 0);
      wb_dat_i:    in std_logic_vector (31 downto 0);

      audio_l: in std_logic_vector (15 downto 0);
      audio_r: in std_logic_vector (15 downto 0);
      
      -- PN Debug:
      tck: in  std_logic;
      tdi: in  std_logic;
      tdo: out std_logic;
      tms: in  std_logic;

      -- I2S Debug:
      i2s_serializer_next_sample: out std_logic
    );
  end component SOUND_CHIP_DEBUG_WRAPPER;

  ----------------------------------------------------------
  --
  --  Local declarations...
  --
  ----------------------------------------------------------

signal wb_clk_scdbg_o: std_logic;
signal wb_rst_scdbg_o: std_logic;
signal wb_stb_scdbg_o: std_logic;
signal wb_cyc_scdbg_o: std_logic;
signal wb_we_scdbg_o:  std_logic;
signal wb_cti_scdbg_o: std_logic_vector (2 downto 0);
signal wb_bte_scdbg_o: std_logic_vector (1 downto 0);
signal wb_sel_scdbg_o: std_logic_vector (3 downto 0);
signal wb_ack_scdbg_i: std_logic;
--signal wb_err_scdbg_i: std_logic;
--signal wb_rty_scdbg_i: std_logic;
signal wb_adr_scdbg_o: std_logic_vector (31 downto 0);
signal wb_dat_scdbg_o: std_logic_vector (31 downto 0);
signal wb_dat_scdbg_i: std_logic_vector (31 downto 0);

signal audio_l: std_logic_vector (15 downto 0);
signal audio_r: std_logic_vector (15 downto 0);

signal i2s_serializer_next_sample: std_logic;

signal i2s_mclk_scdbg_o:  std_logic;
signal i2s_bclk_scdbg_o:  std_logic;
signal i2s_pblrc_scdbg_o: std_logic;
signal i2s_pbdat_scdbg_o: std_logic;
signal i2s_mute_scdbg_o:  std_logic;


begin

  ----------------------------------------------------------
  --
  --  Component instantiations
  --
  ----------------------------------------------------------

  -- Module: SOUND_CORE ...
  I_SOUND_CORE: component SOUND_CORE
    port map (
      wb_clk_i => wb_clk_scdbg_o,
      wb_rst_i => wb_rst_scdbg_o,
      wb_stb_i => wb_stb_scdbg_o,
      wb_cyc_i => wb_cyc_scdbg_o,
      wb_we_i => wb_we_scdbg_o,
      wb_cti_i => wb_cti_scdbg_o,
      wb_bte_i => wb_bte_scdbg_o,
      wb_sel_i => wb_sel_scdbg_o,
      wb_ack_o => wb_ack_scdbg_i,
      wb_err_o => open,
      wb_rty_o => open,
      wb_adr_i => wb_adr_scdbg_o,
      wb_dat_i => wb_dat_scdbg_o,
      wb_dat_o => wb_dat_scdbg_i,
      audio_l => audio_l,
      audio_r => audio_r
    );

  -- Module: PARANUT_DEBUG ...
  I_SOUND_CHIP_DEBUG: component SOUND_CHIP_DEBUG_WRAPPER
    port map (
      led => led,
      btn => btn,
      sw => sw,
      DDR_addr(14 downto 0) => DDR_addr(14 downto 0),
      DDR_ba(2 downto 0) => DDR_ba(2 downto 0),
      DDR_cas_n => DDR_cas_n,
      DDR_ck_n => DDR_ck_n,
      DDR_ck_p => DDR_ck_p,
      DDR_cke => DDR_cke,
      DDR_cs_n => DDR_cs_n,
      DDR_dm(3 downto 0) => DDR_dm(3 downto 0),
      DDR_dq(31 downto 0) => DDR_dq(31 downto 0),
      DDR_dqs_n(3 downto 0) => DDR_dqs_n(3 downto 0),
      DDR_dqs_p(3 downto 0) => DDR_dqs_p(3 downto 0),
      DDR_odt => DDR_odt,
      DDR_ras_n => DDR_ras_n,
      DDR_reset_n => DDR_reset_n,
      DDR_we_n => DDR_we_n,
      FIXED_IO_ddr_vrn => FIXED_IO_ddr_vrn,
      FIXED_IO_ddr_vrp => FIXED_IO_ddr_vrp,
      FIXED_IO_mio(53 downto 0) => FIXED_IO_mio(53 downto 0),
      FIXED_IO_ps_clk => FIXED_IO_ps_clk,
      FIXED_IO_ps_porb => FIXED_IO_ps_porb,
      FIXED_IO_ps_srstb => FIXED_IO_ps_srstb,
      
      SSM2603_IIC_scl_io => SSM2603_IIC_scl_io,
      SSM2603_IIC_sda_io => SSM2603_IIC_sda_io,
      
      i2s_mclk => i2s_mclk_scdbg_o,
      i2s_bclk => i2s_bclk_scdbg_o,
      i2s_pblrc => i2s_pblrc_scdbg_o,
      i2s_pbdat => i2s_pbdat_scdbg_o,
      i2s_mute => i2s_mute_scdbg_o,
      
      wb_clk_o => wb_clk_scdbg_o,
      wb_rst_o => wb_rst_scdbg_o,
      wb_stb_o => wb_stb_scdbg_o,
      wb_cyc_o => wb_cyc_scdbg_o,
      wb_we_o => wb_we_scdbg_o,
      wb_cti_o => wb_cti_scdbg_o,
      wb_bte_o => wb_bte_scdbg_o,
      wb_sel_o => wb_sel_scdbg_o,
      wb_ack_i => wb_ack_scdbg_i,
      wb_stall_i => '0',
      --wb_err_i => wb_err_scdbg_i,
      --wb_rty_i => wb_rty_scdbg_i,
      wb_adr_o => wb_adr_scdbg_o,
      wb_dat_o => wb_dat_scdbg_o,
      wb_dat_i => wb_dat_scdbg_i,

      audio_l => audio_l,
      audio_r => audio_r,
      
      -- PN Debug:
      tck => tck,
      tms => tms,
      tdi => tdi,
      tdo => tdo,
      
      -- I2S Debug:
      i2s_serializer_next_sample => i2s_serializer_next_sample
    );


  ----------------------------------------------------------
  --
  --  Signal connections
  --
  -------------------------------------------------------

i2s_mclk  <= i2s_mclk_scdbg_o;
i2s_bclk  <= i2s_bclk_scdbg_o;
i2s_pblrc <= i2s_pblrc_scdbg_o;
i2s_pbdat <= i2s_pbdat_scdbg_o;
i2s_mute  <= i2s_mute_scdbg_o;


  ----------------------------------------------------------
  --
  --  Debug stuff
  --
  -------------------------------------------------------

--~ debug(0) <= i2s_mclk_scdbg_o;
--~ debug(1) <= i2s_bclk_scdbg_o;
--~ debug(2) <= i2s_pblrc_scdbg_o;
--~ debug(3) <= i2s_pbdat_scdbg_o;
--~ debug(4) <= i2s_mute_scdbg_o;
--~ debug(5) <= '0';
--~ debug(6) <= i2s_serializer_next_sample;
--~ debug(7) <= wb_clk_scdbg_o;

end RTL;
