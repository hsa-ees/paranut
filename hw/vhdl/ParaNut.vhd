--------------------------------------------------------------------------------
--
--  This file is part of the ParaNut project.
--
--  Copyright (C) 2013-2019  Alexander Bahle <alexander.bahle@hs-augsburg.de>
--                           Christian Maier
--      Hochschule Augsburg, University of Applied Sciences
--
------------------------ LICENSE -----------------------------------------------
--  Redistribution and use in source and binary forms, with or without modification,
--  are permitted provided that the following conditions are met:
--
--  1. Redistributions of source code must retain the above copyright notice, this
--     list of conditions and the following disclaimer.
--
--  2. Redistributions in binary form must reproduce the above copyright notice,
--     this list of conditions and the following disclaimer in the documentation and/or
--     other materials provided with the distribution.
--
--  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
--  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
--  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
--  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
--  ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
--  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
--  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
--  ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
--  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
--  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
--------------------------------------------------------------------------------


library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library paranut;
use paranut.paranut_pkg.all;
use paranut.types.all;
use paranut.paranut_config.all;


entity ParaNut is
  generic (
    C_M_AXI_ACLK_FREQ_HZ : integer := 100_000_000;
    C_M_AXI_TARGET_SLAVE_BASE_ADDR : std_logic_vector := x"40000000";
    C_M_AXI_BURST_LEN : integer := 16;
    C_M_AXI_ID_WIDTH : integer := 1;
    C_M_AXI_ADDR_WIDTH : integer := 32;
    C_M_AXI_DATA_WIDTH : integer := 32;
    C_M_AXI_AWUSER_WIDTH : integer := 0;
    C_M_AXI_ARUSER_WIDTH : integer := 0;
    C_M_AXI_WUSER_WIDTH : integer := 0;
    C_M_AXI_RUSER_WIDTH : integer := 0;
    C_M_AXI_BUSER_WIDTH : integer := 0
  );
  port (
    ex_int : in std_logic_vector(CFG_NUT_EX_INT-1 downto 0);

    -- JTAG
    tck : in std_logic;
    tms : in std_logic;
    tdi : in std_logic;
    tdo : out std_logic;

    -- Ports of Axi Master Bus Interface M_AXI
    m_axi_aclk    : in std_logic;
    m_axi_aresetn : in std_logic;
    m_axi_awid    : out std_logic_vector(C_M_AXI_ID_WIDTH-1 downto 0);
    m_axi_awaddr  : out std_logic_vector(C_M_AXI_ADDR_WIDTH-1 downto 0);
    m_axi_awlen   : out std_logic_vector(7 downto 0);
    m_axi_awsize  : out std_logic_vector(2 downto 0);
    m_axi_awburst : out std_logic_vector(1 downto 0);
    m_axi_awlock  : out std_logic;
    m_axi_awcache : out std_logic_vector(3 downto 0);
    m_axi_awprot  : out std_logic_vector(2 downto 0);
    m_axi_awqos   : out std_logic_vector(3 downto 0);
    m_axi_awuser  : out std_logic_vector(C_M_AXI_AWUSER_WIDTH-1 downto 0);
    m_axi_awvalid : out std_logic;
    m_axi_awready : in std_logic;
    m_axi_wdata   : out std_logic_vector(C_M_AXI_DATA_WIDTH-1 downto 0);
    m_axi_wstrb   : out std_logic_vector(C_M_AXI_DATA_WIDTH/8-1 downto 0);
    m_axi_wlast   : out std_logic;
    m_axi_wuser   : out std_logic_vector(C_M_AXI_WUSER_WIDTH-1 downto 0);
    m_axi_wvalid  : out std_logic;
    m_axi_wready  : in std_logic;
    m_axi_bid     : in std_logic_vector(C_M_AXI_ID_WIDTH-1 downto 0);
    m_axi_bresp   : in std_logic_vector(1 downto 0);
    m_axi_buser   : in std_logic_vector(C_M_AXI_BUSER_WIDTH-1 downto 0);
    m_axi_bvalid  : in std_logic;
    m_axi_bready  : out std_logic;
    m_axi_arid    : out std_logic_vector(C_M_AXI_ID_WIDTH-1 downto 0);
    m_axi_araddr  : out std_logic_vector(C_M_AXI_ADDR_WIDTH-1 downto 0);
    m_axi_arlen   : out std_logic_vector(7 downto 0);
    m_axi_arsize  : out std_logic_vector(2 downto 0);
    m_axi_arburst : out std_logic_vector(1 downto 0);
    m_axi_arlock  : out std_logic;
    m_axi_arcache : out std_logic_vector(3 downto 0);
    m_axi_arprot  : out std_logic_vector(2 downto 0);
    m_axi_arqos   : out std_logic_vector(3 downto 0);
    m_axi_aruser  : out std_logic_vector(C_M_AXI_ARUSER_WIDTH-1 downto 0);
    m_axi_arvalid : out std_logic;
    m_axi_arready : in std_logic;
    m_axi_rid     : in std_logic_vector(C_M_AXI_ID_WIDTH-1 downto 0);
    m_axi_rdata   : in std_logic_vector(C_M_AXI_DATA_WIDTH-1 downto 0);
    m_axi_rresp   : in std_logic_vector(1 downto 0);
    m_axi_rlast   : in std_logic;
    m_axi_ruser   : in std_logic_vector(C_M_AXI_RUSER_WIDTH-1 downto 0);
    m_axi_rvalid  : in std_logic;
    m_axi_rready  : out std_logic
  );
end ParaNut;

architecture arch_imp of ParaNut is

  -- wishbone slave to axi master
  component swb2maxi is
    generic (
      C_M_TARGET_SLAVE_BASE_ADDR : std_logic_vector := x"40000000";
      C_M_AXI_BURST_LEN    : integer := 16;
      C_M_AXI_ID_WIDTH     : integer := 1;
      C_M_AXI_ADDR_WIDTH   : integer := 32;
      C_M_AXI_DATA_WIDTH   : integer := 32;
      C_M_AXI_AWUSER_WIDTH : integer := 0;
      C_M_AXI_ARUSER_WIDTH : integer := 0;
      C_M_AXI_WUSER_WIDTH  : integer := 0;
      C_M_AXI_RUSER_WIDTH  : integer := 0;
      C_M_AXI_BUSER_WIDTH  : integer := 0
    );
    port (
      M_AXI_ACLK    : in std_logic;
      M_AXI_ARESETN : in std_logic;
      M_AXI_AWID    : out std_logic_vector(C_M_AXI_ID_WIDTH-1 downto 0);
      M_AXI_AWADDR  : out std_logic_vector(C_M_AXI_ADDR_WIDTH-1 downto 0);
      M_AXI_AWLEN   : out std_logic_vector(7 downto 0);
      M_AXI_AWSIZE  : out std_logic_vector(2 downto 0);
      M_AXI_AWBURST : out std_logic_vector(1 downto 0);
      M_AXI_AWLOCK  : out std_logic;
      M_AXI_AWCACHE : out std_logic_vector(3 downto 0);
      M_AXI_AWPROT  : out std_logic_vector(2 downto 0);
      M_AXI_AWQOS   : out std_logic_vector(3 downto 0);
      M_AXI_AWUSER  : out std_logic_vector(C_M_AXI_AWUSER_WIDTH-1 downto 0);
      M_AXI_AWVALID : out std_logic;
      M_AXI_AWREADY : in std_logic;
      M_AXI_WDATA   : out std_logic_vector(C_M_AXI_DATA_WIDTH-1 downto 0);
      M_AXI_WSTRB   : out std_logic_vector(C_M_AXI_DATA_WIDTH/8-1 downto 0);
      M_AXI_WLAST   : out std_logic;
      M_AXI_WUSER   : out std_logic_vector(C_M_AXI_WUSER_WIDTH-1 downto 0);
      M_AXI_WVALID  : out std_logic;
      M_AXI_WREADY  : in std_logic;
      M_AXI_BID     : in std_logic_vector(C_M_AXI_ID_WIDTH-1 downto 0);
      M_AXI_BRESP   : in std_logic_vector(1 downto 0);
      M_AXI_BUSER   : in std_logic_vector(C_M_AXI_BUSER_WIDTH-1 downto 0);
      M_AXI_BVALID  : in std_logic;
      M_AXI_BREADY  : out std_logic;
      M_AXI_ARID    : out std_logic_vector(C_M_AXI_ID_WIDTH-1 downto 0);
      M_AXI_ARADDR  : out std_logic_vector(C_M_AXI_ADDR_WIDTH-1 downto 0);
      M_AXI_ARLEN   : out std_logic_vector(7 downto 0);
      M_AXI_ARSIZE  : out std_logic_vector(2 downto 0);
      M_AXI_ARBURST : out std_logic_vector(1 downto 0);
      M_AXI_ARLOCK  : out std_logic;
      M_AXI_ARCACHE : out std_logic_vector(3 downto 0);
      M_AXI_ARPROT  : out std_logic_vector(2 downto 0);
      M_AXI_ARQOS   : out std_logic_vector(3 downto 0);
      M_AXI_ARUSER  : out std_logic_vector(C_M_AXI_ARUSER_WIDTH-1 downto 0);
      M_AXI_ARVALID : out std_logic;
      M_AXI_ARREADY : in std_logic;
      M_AXI_RID     : in std_logic_vector(C_M_AXI_ID_WIDTH-1 downto 0);
      M_AXI_RDATA   : in std_logic_vector(C_M_AXI_DATA_WIDTH-1 downto 0);
      M_AXI_RRESP   : in std_logic_vector(1 downto 0);
      M_AXI_RLAST   : in std_logic;
      M_AXI_RUSER   : in std_logic_vector(C_M_AXI_RUSER_WIDTH-1 downto 0);
      M_AXI_RVALID  : in std_logic;
      M_AXI_RREADY  : out std_logic;

      ---------------------------
      -- WISHBONE Slave interface
      ---------------------------
      S_WB_DAT_IN   : in  std_logic_vector(C_M_AXI_DATA_WIDTH-1 downto 0);
      S_WB_DAT_OUT  : out std_logic_vector(C_M_AXI_DATA_WIDTH-1 downto 0);
      S_WB_ACK_O    : out std_logic;
      S_WB_ADR_I    : in  std_logic_vector(C_M_AXI_ADDR_WIDTH-1 downto 0);
      S_WB_CYC_I    : in  std_logic;
      S_WB_STB_I    : in  std_logic;
      S_WB_STALL_O  : out std_logic;
      S_WB_ERR_O    : out std_logic;
      S_WB_LOCK_I   : in  std_logic;
      S_WB_CTI_I    : in  std_logic_vector(2 downto 0);
      S_WB_BTE_I    : in  std_logic_vector(1 downto 0);
      S_WB_SEL_I    : in  std_logic_vector(C_M_AXI_DATA_WIDTH/8-1 downto 0);
      S_WB_WE_I     : in  std_logic
    );
  end component swb2maxi;


  signal wb_clk         : std_logic;
  signal wb_rst         : std_logic;

  -- Signals from paranut to paranut
  signal pip_wb_dat_i  : std_logic_vector(C_M_AXI_DATA_WIDTH-1 downto 0);
  signal pip_wb_dat_o  : std_logic_vector(C_M_AXI_DATA_WIDTH-1 downto 0);
  signal pip_wb_sel    : std_logic_vector(C_M_AXI_DATA_WIDTH/8-1 downto 0);
  signal pip_wb_adr    : std_logic_vector(C_M_AXI_ADDR_WIDTH-1 downto 0);
  signal pip_wb_cyc    : std_logic;
  signal pip_wb_stb    : std_logic;
  signal pip_wb_ack    : std_logic;
  signal pip_wb_rty    : std_logic;
  signal pip_wb_err    : std_logic;
  signal pip_wb_we     : std_logic;
  signal pip_wb_lock   : std_logic;
  signal pip_wb_cti    : std_logic_vector(2 downto 0);
  signal pip_wb_bte    : std_logic_vector(1 downto 0);
  signal pip_wb_stall  : std_logic;


begin

  wb_clk <= m_axi_aclk;
  wb_rst <= not m_axi_aresetn;

  -- Instantiation of Axi Bus Interface M_AXI
  swb2maxi_inst : swb2maxi
    generic map (
      C_M_TARGET_SLAVE_BASE_ADDR  => C_M_AXI_TARGET_SLAVE_BASE_ADDR,
      C_M_AXI_BURST_LEN  => CFG_MEMU_CACHE_BANKS / (C_M_AXI_DATA_WIDTH / 32),
      C_M_AXI_ID_WIDTH  => C_M_AXI_ID_WIDTH,
      C_M_AXI_ADDR_WIDTH  => C_M_AXI_ADDR_WIDTH,
      C_M_AXI_DATA_WIDTH  => C_M_AXI_DATA_WIDTH,
      C_M_AXI_AWUSER_WIDTH  => C_M_AXI_AWUSER_WIDTH,
      C_M_AXI_ARUSER_WIDTH  => C_M_AXI_ARUSER_WIDTH,
      C_M_AXI_WUSER_WIDTH  => C_M_AXI_WUSER_WIDTH,
      C_M_AXI_RUSER_WIDTH  => C_M_AXI_RUSER_WIDTH,
      C_M_AXI_BUSER_WIDTH  => C_M_AXI_BUSER_WIDTH
    )
    port map (
      M_AXI_ACLK  => m_axi_aclk,
      M_AXI_ARESETN  => m_axi_aresetn,
      M_AXI_AWID  => m_axi_awid,
      M_AXI_AWADDR  => m_axi_awaddr,
      M_AXI_AWLEN  => m_axi_awlen,
      M_AXI_AWSIZE  => m_axi_awsize,
      M_AXI_AWBURST  => m_axi_awburst,
      M_AXI_AWLOCK  => m_axi_awlock,
      M_AXI_AWCACHE  => m_axi_awcache,
      M_AXI_AWPROT  => m_axi_awprot,
      M_AXI_AWQOS  => m_axi_awqos,
      M_AXI_AWUSER  => m_axi_awuser,
      M_AXI_AWVALID  => m_axi_awvalid,
      M_AXI_AWREADY  => m_axi_awready,
      M_AXI_WDATA  => m_axi_wdata,
      M_AXI_WSTRB  => m_axi_wstrb,
      M_AXI_WLAST  => m_axi_wlast,
      M_AXI_WUSER  => m_axi_wuser,
      M_AXI_WVALID  => m_axi_wvalid,
      M_AXI_WREADY  => m_axi_wready,
      M_AXI_BID  => m_axi_bid,
      M_AXI_BRESP  => m_axi_bresp,
      M_AXI_BUSER  => m_axi_buser,
      M_AXI_BVALID  => m_axi_bvalid,
      M_AXI_BREADY  => m_axi_bready,
      M_AXI_ARID  => m_axi_arid,
      M_AXI_ARADDR  => m_axi_araddr,
      M_AXI_ARLEN  => m_axi_arlen,
      M_AXI_ARSIZE  => m_axi_arsize,
      M_AXI_ARBURST  => m_axi_arburst,
      M_AXI_ARLOCK  => m_axi_arlock,
      M_AXI_ARCACHE  => m_axi_arcache,
      M_AXI_ARPROT  => m_axi_arprot,
      M_AXI_ARQOS  => m_axi_arqos,
      M_AXI_ARUSER  => m_axi_aruser,
      M_AXI_ARVALID  => m_axi_arvalid,
      M_AXI_ARREADY  => m_axi_arready,
      M_AXI_RID  => m_axi_rid,
      M_AXI_RDATA  => m_axi_rdata,
      M_AXI_RRESP  => m_axi_rresp,
      M_AXI_RLAST  => m_axi_rlast,
      M_AXI_RUSER  => m_axi_ruser,
      M_AXI_RVALID  => m_axi_rvalid,
      M_AXI_RREADY  => m_axi_rready,

      ---------------------------
      -- WISHBONE Slave interface
      ---------------------------
      S_WB_DAT_IN   => pip_wb_dat_o,
      S_WB_DAT_OUT  => pip_wb_dat_i,
      S_WB_ACK_O    => pip_wb_ack,
      S_WB_ADR_I    => pip_wb_adr,
      S_WB_CYC_I    => pip_wb_cyc,
      S_WB_STB_I    => pip_wb_stb,
      S_WB_STALL_O  => pip_wb_stall,
      S_WB_ERR_O    => pip_wb_err,
      S_WB_LOCK_I   => pip_wb_lock,
      S_WB_CTI_I    => pip_wb_cti,
      S_WB_BTE_I    => pip_wb_bte,
      S_WB_SEL_I    => pip_wb_sel,
      S_WB_WE_I     => pip_wb_we
    );

  paranut_inst : mparanut
    generic map (
      --CFG_NUT_CPU_CORES   : integer := 1;
      --CFG_MEMU_CACHE_BANKS : integer := 1
      CLK_FREQ_HZ => C_M_AXI_ACLK_FREQ_HZ
      )
    port  map(
      -- Ports (WISHBONE master)
      clk_i    => wb_clk,
      rst_i    => wb_rst,
      ack_i    => pip_wb_ack,
      err_i    => pip_wb_err,
      rty_i    => pip_wb_rty,
      dat_i    => pip_wb_dat_i,
      cyc_o    => pip_wb_cyc,
      stb_o    => pip_wb_stb,
      we_o     => pip_wb_we,
      sel_o    => pip_wb_sel,
      adr_o    => pip_wb_adr,
      dat_o    => pip_wb_dat_o,
      cti_o    => pip_wb_cti,
      bte_o    => pip_wb_bte,

      -- Other
      du_stall => pip_wb_stall,
      ex_int   => ex_int,

      -- JTAG
      tck    => tck,
      tms   => tms,
      tdi   => tdi,
      tdo   => tdo
    );

end arch_imp;
