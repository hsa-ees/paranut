----------------------------------------------------------------------------------
--  This file is part of the bachelor thesis. 
----------------------------------------------------------------------------------
-- File:           saxi2mwb_v1_0.vhd
-- Entity:         saxi2mwb_v1_0
--
-- Author:         Christian Maier
--
-- Modified:       2020-08-07 Michael Schaeferling:
--                            Remove LED demo code to provide Wishbone ports
--                            externally to achieve bridging capabilities.
--
-- Description:    This module is the saxi2mwb bridge ip core.
--                 This is used for the saxi2mwb test on the system
----------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity saxi2mwb_v1_0 is
  generic (
    -- Parameters of Axi Slave Bus Interface S_AXI
    C_S_AXI_ID_WIDTH  : integer  := 1;
    C_S_AXI_DATA_WIDTH  : integer  := 32;
    C_S_AXI_ADDR_WIDTH  : integer  := 32;
    C_S_AXI_AWUSER_WIDTH  : integer  := 0;
    C_S_AXI_ARUSER_WIDTH  : integer  := 0;
    C_S_AXI_WUSER_WIDTH  : integer  := 0;
    C_S_AXI_RUSER_WIDTH  : integer  := 0;
    C_S_AXI_BUSER_WIDTH  : integer  := 0;
    C_S_AXI_LED_WIDTH : integer := 4
  );
  port (
    -- Ports of Axi Slave Bus Interface S_AXI
    s_axi_aclk  : in std_logic;
    s_axi_aresetn  : in std_logic;
    s_axi_awid  : in std_logic_vector(C_S_AXI_ID_WIDTH-1 downto 0);
    s_axi_awaddr  : in std_logic_vector(C_S_AXI_ADDR_WIDTH-1 downto 0);
    s_axi_awlen  : in std_logic_vector(7 downto 0);
    s_axi_awsize  : in std_logic_vector(2 downto 0);
    s_axi_awburst  : in std_logic_vector(1 downto 0);
    s_axi_awlock  : in std_logic;
    s_axi_awcache  : in std_logic_vector(3 downto 0);
    s_axi_awprot  : in std_logic_vector(2 downto 0);
    s_axi_awqos  : in std_logic_vector(3 downto 0);
    s_axi_awregion  : in std_logic_vector(3 downto 0);
    s_axi_awuser  : in std_logic_vector(C_S_AXI_AWUSER_WIDTH-1 downto 0);
    s_axi_awvalid  : in std_logic;
    s_axi_awready  : out std_logic;
    s_axi_wdata  : in std_logic_vector(C_S_AXI_DATA_WIDTH-1 downto 0);
    s_axi_wstrb  : in std_logic_vector((C_S_AXI_DATA_WIDTH/8)-1 downto 0);
    s_axi_wlast  : in std_logic;
    s_axi_wuser  : in std_logic_vector(C_S_AXI_WUSER_WIDTH-1 downto 0);
    s_axi_wvalid  : in std_logic;
    s_axi_wready  : out std_logic;
    s_axi_bid  : out std_logic_vector(C_S_AXI_ID_WIDTH-1 downto 0);
    s_axi_bresp  : out std_logic_vector(1 downto 0);
    s_axi_buser  : out std_logic_vector(C_S_AXI_BUSER_WIDTH-1 downto 0);
    s_axi_bvalid  : out std_logic;
    s_axi_bready  : in std_logic;
    s_axi_arid  : in std_logic_vector(C_S_AXI_ID_WIDTH-1 downto 0);
    s_axi_araddr  : in std_logic_vector(C_S_AXI_ADDR_WIDTH-1 downto 0);
    s_axi_arlen  : in std_logic_vector(7 downto 0);
    s_axi_arsize  : in std_logic_vector(2 downto 0);
    s_axi_arburst  : in std_logic_vector(1 downto 0);
    s_axi_arlock  : in std_logic;
    s_axi_arcache  : in std_logic_vector(3 downto 0);
    s_axi_arprot  : in std_logic_vector(2 downto 0);
    s_axi_arqos  : in std_logic_vector(3 downto 0);
    s_axi_arregion  : in std_logic_vector(3 downto 0);
    s_axi_aruser  : in std_logic_vector(C_S_AXI_ARUSER_WIDTH-1 downto 0);
    s_axi_arvalid  : in std_logic;
    s_axi_arready  : out std_logic;
    s_axi_rid  : out std_logic_vector(C_S_AXI_ID_WIDTH-1 downto 0);
    s_axi_rdata  : out std_logic_vector(C_S_AXI_DATA_WIDTH-1 downto 0);
    s_axi_rresp  : out std_logic_vector(1 downto 0);
    s_axi_rlast  : out std_logic;
    s_axi_ruser  : out std_logic_vector(C_S_AXI_RUSER_WIDTH-1 downto 0);
    s_axi_rvalid  : out std_logic;
    s_axi_rready  : in std_logic;
    
    -------------------------------
    -- Master wishbone interface --
    -------------------------------
    M_WB_CLK_O    : out std_logic;
    M_WB_RST_O    : out std_logic;
    M_WB_ADR_O    : out std_logic_vector(C_S_AXI_ADDR_WIDTH-1 downto 0);
    M_WB_DAT_I    : in  std_logic_vector(C_S_AXI_DATA_WIDTH-1 downto 0);
    M_WB_DAT_O    : out std_logic_vector(C_S_AXI_DATA_WIDTH-1 downto 0);
    M_WB_WE_O     : out std_logic;
    M_WB_SEL_O    : out std_logic_vector(C_S_AXI_DATA_WIDTH/8-1 downto 0);
    M_WB_STB_O    : out std_logic;
    M_WB_ACK_I    : in  std_logic;
    M_WB_CYC_O    : out std_logic;
    --M_WB_LOCK_O   : out std_logic;
    M_WB_STALL_I  : in  std_logic;
    --M_WB_ERR_I    : out std_logic;
    M_WB_CTI_O    : out std_logic_vector(2 downto 0);
    M_WB_BTE_O    : out std_logic_vector(1 downto 0)
  );
  
  
  
  
  
end saxi2mwb_v1_0;

architecture arch_imp of saxi2mwb_v1_0 is

  -- component declaration
  component saxi2mwb is
    generic (
      C_S_AXI_ID_WIDTH  : integer  := 1;
      C_S_AXI_DATA_WIDTH  : integer  := 32;
      C_S_AXI_ADDR_WIDTH  : integer  := 32;
      C_S_AXI_AWUSER_WIDTH  : integer  := 0;
      C_S_AXI_ARUSER_WIDTH  : integer  := 0;
      C_S_AXI_WUSER_WIDTH  : integer  := 0;
      C_S_AXI_RUSER_WIDTH  : integer  := 0;
      C_S_AXI_BUSER_WIDTH  : integer  := 0
    );
    port (
      S_AXI_ACLK      : in  std_logic;
      S_AXI_ARESETN   : in  std_logic;
      S_AXI_AWID      : in  std_logic_vector(C_S_AXI_ID_WIDTH-1 downto 0);
      S_AXI_AWADDR    : in  std_logic_vector(C_S_AXI_ADDR_WIDTH-1 downto 0);
      S_AXI_AWLEN     : in  std_logic_vector(7 downto 0);
      S_AXI_AWSIZE    : in  std_logic_vector(2 downto 0);
      S_AXI_AWBURST   : in  std_logic_vector(1 downto 0);
      S_AXI_AWLOCK    : in  std_logic;
      S_AXI_AWCACHE   : in  std_logic_vector(3 downto 0);
      S_AXI_AWPROT    : in  std_logic_vector(2 downto 0);
      S_AXI_AWQOS     : in  std_logic_vector(3 downto 0);
      S_AXI_AWREGION  : in  std_logic_vector(3 downto 0);
      S_AXI_AWUSER    : in  std_logic_vector(C_S_AXI_AWUSER_WIDTH-1 downto 0);
      S_AXI_AWVALID   : in  std_logic;
      S_AXI_AWREADY   : out std_logic;
      S_AXI_WDATA     : in  std_logic_vector(C_S_AXI_DATA_WIDTH-1 downto 0);
      S_AXI_WSTRB     : in  std_logic_vector((C_S_AXI_DATA_WIDTH/8)-1 downto 0);
      S_AXI_WLAST     : in  std_logic;
      S_AXI_WUSER     : in  std_logic_vector(C_S_AXI_WUSER_WIDTH-1 downto 0);
      S_AXI_WVALID    : in  std_logic;
      S_AXI_WREADY    : out std_logic;
      S_AXI_BID       : out std_logic_vector(C_S_AXI_ID_WIDTH-1 downto 0);
      S_AXI_BRESP     : out std_logic_vector(1 downto 0);
      S_AXI_BUSER     : out std_logic_vector(C_S_AXI_BUSER_WIDTH-1 downto 0);
      S_AXI_BVALID    : out std_logic;
      S_AXI_BREADY    : in  std_logic;
      S_AXI_ARID      : in  std_logic_vector(C_S_AXI_ID_WIDTH-1 downto 0);
      S_AXI_ARADDR    : in  std_logic_vector(C_S_AXI_ADDR_WIDTH-1 downto 0);
      S_AXI_ARLEN     : in  std_logic_vector(7 downto 0);
      S_AXI_ARSIZE    : in  std_logic_vector(2 downto 0);
      S_AXI_ARBURST   : in  std_logic_vector(1 downto 0);
      S_AXI_ARLOCK    : in  std_logic;
      S_AXI_ARCACHE   : in  std_logic_vector(3 downto 0);
      S_AXI_ARPROT    : in  std_logic_vector(2 downto 0);
      S_AXI_ARQOS     : in  std_logic_vector(3 downto 0);
      S_AXI_ARREGION  : in  std_logic_vector(3 downto 0);
      S_AXI_ARUSER    : in  std_logic_vector(C_S_AXI_ARUSER_WIDTH-1 downto 0);
      S_AXI_ARVALID   : in  std_logic;
      S_AXI_ARREADY   : out std_logic;
      S_AXI_RID       : out std_logic_vector(C_S_AXI_ID_WIDTH-1 downto 0);
      S_AXI_RDATA     : out std_logic_vector(C_S_AXI_DATA_WIDTH-1 downto 0);
      S_AXI_RRESP     : out std_logic_vector(1 downto 0);
      S_AXI_RLAST     : out std_logic;
      S_AXI_RUSER     : out std_logic_vector(C_S_AXI_RUSER_WIDTH-1 downto 0);
      S_AXI_RVALID    : out std_logic;
      S_AXI_RREADY    : in  std_logic;

      M_WB_ADR_O    : out std_logic_vector(C_S_AXI_ADDR_WIDTH-1 downto 0);
      M_WB_DAT_I    : in  std_logic_vector(C_S_AXI_DATA_WIDTH-1 downto 0);
      M_WB_DAT_O    : out std_logic_vector(C_S_AXI_DATA_WIDTH-1 downto 0);
      M_WB_WE_O     : out std_logic;
      M_WB_SEL_O    : out std_logic_vector(C_S_AXI_DATA_WIDTH/8-1 downto 0);
      M_WB_STB_O    : out std_logic;
      M_WB_ACK_I    : in  std_logic;
      M_WB_CYC_O    : out std_logic;
      M_WB_LOCK_O   : out std_logic;
      M_WB_STALL_I  : in  std_logic;
      --M_WB_ERR_I    : out std_logic;
      M_WB_CTI_O    : out std_logic_vector(2 downto 0);
      M_WB_BTE_O    : out std_logic_vector(1 downto 0)
      
    );
  end component saxi2mwb;


begin

-- Instantiation of Axi Bus Interface S_AXI
saxi2mwb_inst : saxi2mwb
  generic map (
    C_S_AXI_ID_WIDTH  => C_S_AXI_ID_WIDTH,
    C_S_AXI_DATA_WIDTH  => C_S_AXI_DATA_WIDTH,
    C_S_AXI_ADDR_WIDTH  => C_S_AXI_ADDR_WIDTH,
    C_S_AXI_AWUSER_WIDTH  => C_S_AXI_AWUSER_WIDTH,
    C_S_AXI_ARUSER_WIDTH  => C_S_AXI_ARUSER_WIDTH,
    C_S_AXI_WUSER_WIDTH  => C_S_AXI_WUSER_WIDTH,
    C_S_AXI_RUSER_WIDTH  => C_S_AXI_RUSER_WIDTH,
    C_S_AXI_BUSER_WIDTH  => C_S_AXI_BUSER_WIDTH
  )
  port map (
    S_AXI_ACLK      => s_axi_aclk,
    S_AXI_ARESETN   => s_axi_aresetn,
    S_AXI_AWID      => s_axi_awid,
    S_AXI_AWADDR    => s_axi_awaddr,
    S_AXI_AWLEN     => s_axi_awlen,
    S_AXI_AWSIZE    => s_axi_awsize,
    S_AXI_AWBURST   => s_axi_awburst,
    S_AXI_AWLOCK    => s_axi_awlock,
    S_AXI_AWCACHE   => s_axi_awcache,
    S_AXI_AWPROT    => s_axi_awprot,
    S_AXI_AWQOS     => s_axi_awqos,
    S_AXI_AWREGION  => s_axi_awregion,
    S_AXI_AWUSER    => s_axi_awuser,
    S_AXI_AWVALID   => s_axi_awvalid,
    S_AXI_AWREADY   => s_axi_awready,
    S_AXI_WDATA     => s_axi_wdata,
    S_AXI_WSTRB     => s_axi_wstrb,
    S_AXI_WLAST     => s_axi_wlast,
    S_AXI_WUSER     => s_axi_wuser,
    S_AXI_WVALID    => s_axi_wvalid,
    S_AXI_WREADY    => s_axi_wready,
    S_AXI_BID       => s_axi_bid,
    S_AXI_BRESP     => s_axi_bresp,
    S_AXI_BUSER     => s_axi_buser,
    S_AXI_BVALID    => s_axi_bvalid,
    S_AXI_BREADY    => s_axi_bready,
    S_AXI_ARID      => s_axi_arid,
    S_AXI_ARADDR    => s_axi_araddr,
    S_AXI_ARLEN     => s_axi_arlen,
    S_AXI_ARSIZE    => s_axi_arsize,
    S_AXI_ARBURST   => s_axi_arburst,
    S_AXI_ARLOCK    => s_axi_arlock,
    S_AXI_ARCACHE   => s_axi_arcache,
    S_AXI_ARPROT    => s_axi_arprot,
    S_AXI_ARQOS     => s_axi_arqos,
    S_AXI_ARREGION  => s_axi_arregion,
    S_AXI_ARUSER    => s_axi_aruser,
    S_AXI_ARVALID   => s_axi_arvalid,
    S_AXI_ARREADY   => s_axi_arready,
    S_AXI_RID       => s_axi_rid,
    S_AXI_RDATA     => s_axi_rdata,
    S_AXI_RRESP     => s_axi_rresp,
    S_AXI_RLAST     => s_axi_rlast,
    S_AXI_RUSER     => s_axi_ruser,
    S_AXI_RVALID    => s_axi_rvalid,
    S_AXI_RREADY    => s_axi_rready,

    M_WB_ADR_O    => M_WB_ADR_O,
    M_WB_DAT_I    => M_WB_DAT_I,
    M_WB_DAT_O    => M_WB_DAT_O,
    M_WB_WE_O     => M_WB_WE_O,
    M_WB_SEL_O    => M_WB_SEL_O,
    M_WB_STB_O    => M_WB_STB_O,
    M_WB_ACK_I    => M_WB_ACK_I,
    M_WB_CYC_O    => M_WB_CYC_O,
    M_WB_LOCK_O   => open,
    M_WB_STALL_I  => M_WB_STALL_I,
    --M_WB_ERR_I    =>
    M_WB_CTI_O    => M_WB_CTI_O,
    M_WB_BTE_O    => M_WB_BTE_O
  );

M_WB_CLK_O <= s_axi_aclk;
M_WB_RST_O <= not s_axi_aresetn;


end arch_imp;
