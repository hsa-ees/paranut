----------------------------------------------------------------------------------
--  This file is part of the bachelor thesis. 
----------------------------------------------------------------------------------
-- File:           saxi2mwb.vhd
-- Entity:         saxi2mwb
--
-- Author:         Christian Maier
--
-- Modified:
--
-- Description:    This module a bridge for slave AXI and master WISHBONE
--                 This module is able to use single beat and burst transfer
----------------------------------------------------------------------------------


library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity saxi2mwb is
	generic (

		C_S_AXI_ID_WIDTH	: integer	:= 1;
		C_S_AXI_DATA_WIDTH	: integer	:= 32;
		C_S_AXI_ADDR_WIDTH	: integer	:= 6;
		C_S_AXI_AWUSER_WIDTH	: integer	:= 0;
		C_S_AXI_ARUSER_WIDTH	: integer	:= 0;
		C_S_AXI_WUSER_WIDTH	: integer	:= 0;
		C_S_AXI_RUSER_WIDTH	: integer	:= 0;
		C_S_AXI_BUSER_WIDTH	: integer	:= 0;
		C_S_AXI_BASE_ADDR : std_logic_vector(31 downto 0) := x"40000000"
	);
	port (


		S_AXI_ACLK	: in std_logic;
		S_AXI_ARESETN	: in std_logic;
		S_AXI_AWID	: in std_logic_vector(C_S_AXI_ID_WIDTH-1 downto 0);
		S_AXI_AWADDR	: in std_logic_vector(C_S_AXI_ADDR_WIDTH-1 downto 0);
		S_AXI_AWLEN	: in std_logic_vector(7 downto 0);
		S_AXI_AWSIZE	: in std_logic_vector(2 downto 0);
		S_AXI_AWBURST	: in std_logic_vector(1 downto 0);
		S_AXI_AWLOCK	: in std_logic;
		S_AXI_AWCACHE	: in std_logic_vector(3 downto 0);
		S_AXI_AWPROT	: in std_logic_vector(2 downto 0);
		S_AXI_AWQOS	: in std_logic_vector(3 downto 0);
		S_AXI_AWREGION	: in std_logic_vector(3 downto 0);
		S_AXI_AWUSER	: in std_logic_vector(C_S_AXI_AWUSER_WIDTH-1 downto 0);
		S_AXI_AWVALID	: in std_logic;
		S_AXI_AWREADY	: out std_logic;
		S_AXI_WDATA	: in std_logic_vector(C_S_AXI_DATA_WIDTH-1 downto 0);
		S_AXI_WSTRB	: in std_logic_vector((C_S_AXI_DATA_WIDTH/8)-1 downto 0);
		S_AXI_WLAST	: in std_logic;
		S_AXI_WUSER	: in std_logic_vector(C_S_AXI_WUSER_WIDTH-1 downto 0);
		S_AXI_WVALID	: in std_logic;
		S_AXI_WREADY	: out std_logic;
		S_AXI_BID	: out std_logic_vector(C_S_AXI_ID_WIDTH-1 downto 0);
		S_AXI_BRESP	: out std_logic_vector(1 downto 0);
		S_AXI_BUSER	: out std_logic_vector(C_S_AXI_BUSER_WIDTH-1 downto 0);
		S_AXI_BVALID	: out std_logic;
		S_AXI_BREADY	: in std_logic;
		S_AXI_ARID	: in std_logic_vector(C_S_AXI_ID_WIDTH-1 downto 0);
		S_AXI_ARADDR	: in std_logic_vector(C_S_AXI_ADDR_WIDTH-1 downto 0);
		S_AXI_ARLEN	: in std_logic_vector(7 downto 0);
		S_AXI_ARSIZE	: in std_logic_vector(2 downto 0);
		S_AXI_ARBURST	: in std_logic_vector(1 downto 0);
		S_AXI_ARLOCK	: in std_logic;
		S_AXI_ARCACHE	: in std_logic_vector(3 downto 0);
		S_AXI_ARPROT	: in std_logic_vector(2 downto 0);
		S_AXI_ARQOS	: in std_logic_vector(3 downto 0);
		S_AXI_ARREGION	: in std_logic_vector(3 downto 0);
		S_AXI_ARUSER	: in std_logic_vector(C_S_AXI_ARUSER_WIDTH-1 downto 0);
		S_AXI_ARVALID	: in std_logic;
		S_AXI_ARREADY	: out std_logic;
		S_AXI_RID	: out std_logic_vector(C_S_AXI_ID_WIDTH-1 downto 0);
		S_AXI_RDATA	: out std_logic_vector(C_S_AXI_DATA_WIDTH-1 downto 0);
		S_AXI_RRESP	: out std_logic_vector(1 downto 0);
		S_AXI_RLAST	: out std_logic;
		S_AXI_RUSER	: out std_logic_vector(C_S_AXI_RUSER_WIDTH-1 downto 0);
		S_AXI_RVALID	: out std_logic;
		S_AXI_RREADY	: in std_logic;
		
    -------------------------------
		-- Master wishbone interface --
		-------------------------------
		
    M_WB_DAT_I    : in  std_logic_vector(C_S_AXI_DATA_WIDTH-1 downto 0);
    M_WB_DAT_O    : out std_logic_vector(C_S_AXI_DATA_WIDTH-1 downto 0);
    M_WB_ACK_I    : in  std_logic;
    M_WB_ADR_O    : out std_logic_vector(C_S_AXI_ADDR_WIDTH-1 downto 0);
    M_WB_CYC_O    : out std_logic;
    M_WB_LOCK_O   : out std_logic;
    M_WB_STALL_I  : in  std_logic;
    --M_WB_ERR_I    : out std_logic;
    M_WB_CTI_O    : out std_logic_vector(2 downto 0);
    M_WB_BTE_O    : out std_logic_vector(1 downto 0);
    M_WB_SEL_O    : out std_logic_vector(C_S_AXI_DATA_WIDTH/8-1 downto 0);
    M_WB_STB_O    : out std_logic;
    M_WB_WE_O     : out std_logic
		
	);
end saxi2mwb;

architecture arch_imp of saxi2mwb is

	-- AXI4FULL signals
	signal axi_awaddr	: std_logic_vector(C_S_AXI_ADDR_WIDTH-1 downto 0);
	signal axi_wdata	: std_logic_vector(C_S_AXI_DATA_WIDTH-1 downto 0);
	signal axi_awlock : std_logic;
	signal axi_wstrb  : std_logic_vector(C_S_AXI_DATA_WIDTH/8-1 downto 0);
	signal axi_awready	: std_logic;
	signal axi_wready	: std_logic;
	signal axi_bresp	: std_logic_vector(1 downto 0);
	signal axi_buser	: std_logic_vector(C_S_AXI_BUSER_WIDTH-1 downto 0);
	signal axi_bvalid	: std_logic;
	signal axi_araddr	: std_logic_vector(C_S_AXI_ADDR_WIDTH-1 downto 0);
	signal axi_arlock : std_logic;
	signal axi_arready	: std_logic;
	signal axi_rdata	: std_logic_vector(C_S_AXI_DATA_WIDTH-1 downto 0);
	signal axi_rresp	: std_logic_vector(1 downto 0);
	signal axi_rlast	: std_logic;
	signal axi_ruser	: std_logic_vector(C_S_AXI_RUSER_WIDTH-1 downto 0);
	signal axi_rvalid	: std_logic;
	-- The axi_awv_awr_flag flag marks the presence of write address valid
	signal axi_awv_awr_flag    : std_logic;
	--The axi_arv_arr_flag flag marks the presence of read address valid
	signal axi_arv_arr_flag    : std_logic;
	signal axi_burst_cnt : unsigned(7 downto 0);
	-------------------
	-- Wishbone signals
	-------------------
	signal wb_stb     : std_logic;
	signal wb_stall   : std_logic;
  --signal wb_err     : std_logic;
  signal wb_cti     : std_logic_vector(2 downto 0);
	signal wb_we      : std_logic;
	

begin
	-- I/O Connections assignments

	S_AXI_AWREADY	<= axi_awready;
	S_AXI_WREADY	<= axi_wready;
	S_AXI_BRESP	<= axi_bresp;
	S_AXI_BUSER	<= axi_buser;
	S_AXI_BVALID	<= axi_bvalid;
	S_AXI_ARREADY	<= axi_arready;
	S_AXI_RDATA	<= M_WB_DAT_I;
	S_AXI_RRESP	<= axi_rresp;
	S_AXI_RLAST	<= axi_rlast;
	S_AXI_RUSER	<= axi_ruser;
	S_AXI_RVALID	<= axi_rvalid;
	S_AXI_BID <= S_AXI_AWID;
	S_AXI_RID <= S_AXI_ARID;

  axi_wready <= M_WB_ACK_I when axi_awv_awr_flag = '1' else '0';
  axi_rvalid <= M_WB_ACK_I when axi_arv_arr_flag = '1' else '0';

  M_WB_ADR_O <= S_AXI_AWADDR when axi_awv_awr_flag = '1' else S_AXI_ARADDR when axi_arv_arr_flag = '1';
  M_WB_CYC_O <= '1' when axi_awv_awr_flag = '1' or axi_arv_arr_flag = '1' else '0';
  M_WB_LOCK_O <= S_AXI_AWLOCK when axi_awv_awr_flag = '1' else S_AXI_ARLOCK when axi_arv_arr_flag = '1' else '0';
  M_WB_DAT_O <= S_AXI_WDATA;
  M_WB_CTI_O <= wb_cti;
  M_WB_BTE_O <= (others => '0');
  M_WB_SEL_O <= S_AXI_WSTRB;
  M_WB_STB_O <= wb_stb;
  M_WB_WE_O <= '1' when axi_awv_awr_flag = '1' else '0' when axi_arv_arr_flag = '1';

  wb_stb <= S_AXI_WVALID when axi_awv_awr_flag = '1' else S_AXI_RREADY when axi_arv_arr_flag = '1' else '0';
  -- WB_CYC and address handshake

	ADR_HS : process (S_AXI_ACLK)
	begin
	  if rising_edge(S_AXI_ACLK) then 
	    if S_AXI_ARESETN = '0' then
	      axi_awready <= '0';
	      axi_awv_awr_flag <= '0';
	      
	      axi_arready <= '0';
	      axi_arv_arr_flag <= '0';

	      wb_cti <= "000";
	    else 
	      if axi_awv_awr_flag = '0' and axi_arv_arr_flag = '0' then 
	        if axi_awready = '0' and S_AXI_AWVALID = '1' then
	          axi_awready <= '1';
	        elsif S_AXI_AWVALID = '1' and axi_awready = '1' then
            axi_awready <= '0';
            axi_awv_awr_flag  <= '1';
            
            if unsigned(S_AXI_AWLEN) = 0 then
              wb_cti <= "111";
            elsif S_AXI_AWBURST = "01" then
              wb_cti <= "010";
            elsif S_AXI_AWBURST = "00" then
              wb_cti <= "001";
            else
              wb_cti <= "000";
            end if;
          end if;
          
          if axi_arready = '0' and S_AXI_ARVALID = '1' then
	          axi_arready <= '1';
	        elsif S_AXI_ARVALID = '1' and axi_arready = '1' then
            axi_arready <= '0';
            axi_arv_arr_flag <= '1';
            
            if unsigned(S_AXI_ARLEN) = 0 then
              wb_cti <= "111";
            elsif S_AXI_ARBURST = "01" then
              wb_cti <= "010";
            elsif S_AXI_ARBURST = "00" then
              wb_cti <= "001";
            else
              wb_cti <= "000";
            end if;
          end if;
	        
        elsif axi_awv_awr_flag = '1' then
          
          if unsigned(S_AXI_AWLEN) = (axi_burst_cnt+1) then
            wb_cti <= "111";
          end if;
          
          if axi_bvalid = '1' and S_AXI_BREADY = '1' then 
	          axi_awv_awr_flag  <= '0';
	        end if;
          
        elsif axi_arv_arr_flag = '1' then

          if axi_rvalid = '1' and S_AXI_RREADY = '1' and axi_rlast = '1' then 
	          axi_arv_arr_flag <= '0';
	        end if;
	        
	        if unsigned(S_AXI_ARLEN) = (axi_burst_cnt+1) then
            wb_cti <= "111";
          end if;
	        
        else
          axi_awready <= '0';
          axi_awv_awr_flag <= '0';
          
          axi_arready <= '0';
          axi_arv_arr_flag <= '0';
          
	      end if;
	    end if;
	  end if;         
	end process; 
	

	BVALID_GEN : process (S_AXI_ACLK)
	begin
	  if rising_edge(S_AXI_ACLK) then 
	    if S_AXI_ARESETN = '0' then
	      axi_bvalid  <= '0';
	      axi_bresp  <= "00";
	    else
	      if axi_bvalid = '0' and S_AXI_WVALID = '1' and axi_wready = '1' and S_AXI_WLAST = '1' then
	        axi_bvalid <= '1';
	        axi_bresp  <= "00"; 
	      elsif S_AXI_BREADY = '1' and axi_bvalid = '1' then  
	        axi_bvalid <= '0';                      
	      end if;
	    end if;
	  end if;         
	end process; 

	RLAST : process (S_AXI_ACLK)
	begin
	  if rising_edge(S_AXI_ACLK) then
	    if S_AXI_ARESETN = '0' then
	      axi_rlast <= '0';
	      axi_rresp  <= "00";
	    else  
	      if axi_rlast = '0' and (unsigned(S_AXI_ARLEN) = (axi_burst_cnt+1) or unsigned(S_AXI_ARLEN) = 0) then
	        axi_rlast <= '1';
	        axi_rresp  <= "00"; -- OKAY
	      elsif S_AXI_RREADY = '1' and axi_rvalid = '1' and axi_rlast = '1' then
	        axi_rlast <= '0';
	      end if;
	    end if;
	  end if;
	end  process;


  -- Counter for the burst
  BURST_CNT : process(S_AXI_ACLK)
  begin
    if rising_edge(S_AXI_ACLK) then
      if S_AXI_ARESETN = '0' then
        axi_burst_cnt <= (others => '0');
      else
        if axi_awv_awr_flag = '1' or axi_arv_arr_flag = '1' then
          if M_WB_ACK_I = '1' then
            axi_burst_cnt <= axi_burst_cnt + 1;
          end if;
        else
          axi_burst_cnt <= (others => '0');
        end if;
      end if;
    end if;
  end process;
  

end arch_imp;
