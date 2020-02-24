--------------------------------------------------------------------------------
-- This file is part of the ParaNut project.
-- 
-- Copyright (C) 2013-2019  Alexander Bahle <alexander.bahle@hs-augsburg.de>
-- 							Christian Maier
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
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity saxi2mwb is
	generic (

		C_S_AXI_ID_WIDTH	: integer	:= 1;
		C_S_AXI_DATA_WIDTH	: integer	:= 32;
		C_S_AXI_ADDR_WIDTH	: integer	:= 32;
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

		------------------------------
		-- Master wishbone interface --
		------------------------------

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
	signal wb_rst     : std_logic;
	signal wb_adr     : std_logic_vector(C_S_AXI_ADDR_WIDTH-1 downto 0);
	signal wb_dat_out : std_logic_vector(C_S_AXI_DATA_WIDTH-1 downto 0);
	signal wb_cyc     : std_logic;
	signal wb_stb     : std_logic;
	signal wb_stall   : std_logic;
  --signal wb_err     : std_logic;
  signal wb_cti     : std_logic_vector(2 downto 0);
  signal wb_bte     : std_logic_vector(1 downto 0);
	signal wb_we      : std_logic;
	

begin
	-- I/O Connections assignments

	S_AXI_AWREADY	<= axi_awready;
	S_AXI_WREADY	<= axi_wready;
	axi_wready      <= M_WB_ACK_I and axi_awv_awr_flag;
	S_AXI_BRESP	<= axi_bresp;
	S_AXI_BUSER	<= axi_buser;
	S_AXI_BVALID	<= axi_bvalid;
	S_AXI_ARREADY	<= axi_arready;
	S_AXI_RDATA	<= M_WB_DAT_I;
	S_AXI_RRESP	<= axi_rresp;
	S_AXI_RLAST	<= axi_rlast;
	S_AXI_RUSER	<= axi_ruser;
	S_AXI_RVALID	<= axi_rvalid;
	axi_rvalid <= M_WB_ACK_I and axi_arv_arr_flag;
	S_AXI_BID <= S_AXI_AWID;
	S_AXI_RID <= S_AXI_ARID;

  M_WB_ADR_O <= wb_adr;
  M_WB_CYC_O <= wb_cyc;
  M_WB_LOCK_O <= S_AXI_AWLOCK when axi_awv_awr_flag = '1' else S_AXI_ARLOCK when axi_arv_arr_flag = '1' else '0';
  M_WB_DAT_O <= S_AXI_WDATA;
  M_WB_CTI_O <= wb_cti;
  M_WB_BTE_O <= wb_bte;
  M_WB_SEL_O <= S_AXI_WSTRB;
  M_WB_STB_O <= S_AXI_WVALID when wb_cyc = '1' and axi_awv_awr_flag = '1' else S_AXI_RREADY when wb_cyc = '1' and axi_arv_arr_flag = '1' else '0';
  M_WB_WE_O <= wb_we;

  wb_rst   <= not S_AXI_ARESETN;

	-- Implement axi_awready generation

	-- axi_awready is asserted for one S_AXI_ACLK clock cycle when both
	-- S_AXI_AWVALID and S_AXI_WVALID are asserted. axi_awready is
	-- de-asserted when reset is low.

	AWREADY_GEN : process (S_AXI_ACLK)
	begin
	  if rising_edge(S_AXI_ACLK) then 
	    if S_AXI_ARESETN = '0' then
	      axi_awready <= '0';
	      axi_awv_awr_flag <= '0';
	    else
	      if (axi_awready = '0' and S_AXI_AWVALID = '1' and axi_awv_awr_flag = '0' and axi_arv_arr_flag = '0' and wb_cyc = '0') then
	        -- slave is ready to accept an address and
	        -- associated control signals
	        axi_awready <= '1';
	      --elsif axi_awready = '1' and S_AXI_AWVALID = '1' then
	        axi_awv_awr_flag  <= '1'; -- used for generation of bresp() and bvalid
	        --axi_awready <= '0';
	      elsif M_WB_ACK_I = '1' and unsigned(S_AXI_AWLEN) = axi_burst_cnt then 
	      -- preparing to accept next address after current write burst tx completion
	        axi_awv_awr_flag  <= '0';
	      else
	        axi_awready <= '0';
	      end if;
	    end if;
	  end if;         
	end process; 
	-- Implement axi_awaddr latching

	-- This process is used to latch the address when both 
	-- S_AXI_AWVALID and S_AXI_WVALID are valid. 
	-- Implement axi_wready generation

	-- axi_wready is asserted for one S_AXI_ACLK clock cycle when both
	-- S_AXI_AWVALID and S_AXI_WVALID are asserted. axi_wready is 
	-- de-asserted when reset is low. 

	--WREADY : process (S_AXI_ACLK)
	--begin
	--if rising_edge(S_AXI_ACLK) then 
	--  if S_AXI_ARESETN = '0' then
	--    axi_wready <= '0';
	--  else
	--    if (S_AXI_WLAST = '1' and axi_wready = '1' and S_AXI_WVALID = '1') or S_AXI_WVALID = '0' then 
	--      axi_wready <= '0';
	--    elsif axi_awv_awr_flag = '1' and wb_cyc = '1' then
	--      axi_wready <= '1';
	        -- elsif (axi_awv_awr_flag = '0') then
	--    end if;
	--  end if;
	--end if;         
	--end process; 
	-- Implement write response logic generation

	-- The write response and response valid signals are asserted by the slave 
	-- when axi_wready, S_AXI_WVALID, axi_wready and S_AXI_WVALID are asserted.  
	-- This marks the acceptance of address and indicates the status of 
	-- write transaction.

	BVALID_GEN : process (S_AXI_ACLK)
	begin
	  if rising_edge(S_AXI_ACLK) then 
	    if S_AXI_ARESETN = '0' then
	      axi_bvalid  <= '0';
	      axi_bresp  <= "00"; --need to work more on the responses
	    else
	      if axi_bvalid = '0' and (axi_wready = '1' and S_AXI_WLAST = '1' and S_AXI_WVALID = '1') and unsigned(S_AXI_AWLEN) = axi_burst_cnt then
	        axi_bvalid <= '1';
	        axi_bresp  <= "00"; 
	      elsif S_AXI_BREADY = '1' and axi_bvalid = '1' then  
	      --check if bready is asserted while bvalid is high)
	        axi_bvalid <= '0';   
	      else
	        axi_bvalid <= '0';                   
	      end if;
	    end if;
	  end if;         
	end process; 
	-- Implement axi_arready generation

	-- axi_arready is asserted for one S_AXI_ACLK clock cycle when
	-- S_AXI_ARVALID is asserted. axi_awready is 
	-- de-asserted when reset (active low) is asserted. 
	-- The read address is also latched when S_AXI_ARVALID is 
	-- asserted. axi_araddr is reset to zero on reset assertion.

	ARVALID : process (S_AXI_ACLK)
	begin
	  if rising_edge(S_AXI_ACLK) then 
	    if S_AXI_ARESETN = '0' then
	      axi_arready <= '0';
	      axi_arv_arr_flag <= '0';
	    else
	      if axi_arready = '0' and S_AXI_ARVALID = '1' and axi_awv_awr_flag = '0' and axi_arv_arr_flag = '0' and S_AXI_AWVALID = '0' and wb_cyc = '0' then
	        axi_arready <= '1';
	        axi_arv_arr_flag <= '1';
	      elsif axi_rvalid = '1' and S_AXI_RREADY = '1' and axi_rlast = '1' then 
	      -- preparing to accept next address after current read completion
	        axi_arv_arr_flag <= '0';
	      else
	        axi_arready <= '0';
	      end if;
	    end if;
	  end if;         
	end process; 

	-- Implement axi_arvalid generation

	-- axi_rvalid is asserted for one S_AXI_ACLK clock cycle when both 
	-- S_AXI_ARVALID and axi_arready are asserted. The slave registers 
	-- data are available on the axi_rdata bus at this instance. The 
	-- assertion of axi_rvalid marks the validity of read data on the 
	-- bus and axi_rresp indicates the status of read transaction.axi_rvalid 
	-- is deasserted on reset (active low). axi_rresp and axi_rdata are 
	-- cleared to zero on reset (active low).  

  

	RLAST : process (S_AXI_ACLK)
	begin
	  if rising_edge(S_AXI_ACLK) then
	    if S_AXI_ARESETN = '0' then
	      axi_rlast <= '0';
	      axi_rresp  <= "00";
	    else
	      if axi_arv_arr_flag = '1' and (unsigned(S_AXI_ARLEN) = (axi_burst_cnt+1) or unsigned(S_AXI_ARLEN) = axi_burst_cnt) and axi_rlast = '0' then
	        axi_rlast <= '1';
	        axi_rresp  <= "00"; -- 'OKAY' response
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
        if axi_awv_awr_flag = '1' then
          if S_AXI_WVALID = '1' and axi_wready = '1' then
            axi_burst_cnt <= axi_burst_cnt + 1;
          else
            axi_burst_cnt <= axi_burst_cnt;
          end if;
        elsif axi_arv_arr_flag = '1' then
          if S_AXI_RREADY = '1' and axi_rvalid = '1' then
            axi_burst_cnt <= axi_burst_cnt + 1;
          else
            axi_burst_cnt <= axi_burst_cnt;
          end if;
        else
          axi_burst_cnt <= (others => '0');
        end if;
      end if;
    end if;
  end process;
  
  
  WB : process (S_AXI_ACLK)
  begin
    if rising_edge(S_AXI_ACLK) then
      if wb_rst = '1' then
        wb_adr <= (others => '0');
        wb_cyc <= '0';
        wb_stb <= '0';
        wb_we <= '0';
        wb_cti <= "000";
        wb_bte <= "00";
      else
      
        -- WRITE
        -- wb_cyc, wb_we
        if S_AXI_AWVALID = '1' and axi_arv_arr_flag = '0' then
          wb_adr <= S_AXI_AWADDR;
          wb_cyc <= '1';
          wb_we <= '1';
        -- READ
        elsif S_AXI_ARVALID = '1' and axi_awv_awr_flag = '0' and S_AXI_AWVALID = '0' then
          wb_adr <= S_AXI_ARADDR;
          wb_cyc <= '1';
          wb_we <= '0';
        elsif M_WB_ACK_I = '1' and wb_stb = '1' then
          if wb_cti = "000" or wb_cti = "111" then
              wb_cyc <= '0';
          end if;
        end if;
          
        -- wb_adr
        if S_AXI_AWVALID = '1' and axi_arv_arr_flag = '0' then
          wb_adr <= S_AXI_AWADDR;
        elsif axi_awv_awr_flag = '1' then
          if M_WB_ACK_I = '1' and S_AXI_WVALID = '1' then
            wb_adr <= std_logic_vector(unsigned(wb_adr) + 4);
          end if;
        elsif S_AXI_ARVALID = '1' and axi_awv_awr_flag = '0' and S_AXI_AWVALID = '0' then
          wb_adr <= S_AXI_ARADDR;
        elsif axi_arv_arr_flag = '1' then
          if M_WB_ACK_I = '1' and S_AXI_RREADY = '1' then
            wb_adr <= std_logic_vector(unsigned(wb_adr) + 4);
          end if;
        end if;
          
          
        -- wb_cti
        -- WRITE
        if (S_AXI_AWVALID = '1' and axi_arv_arr_flag = '0') then 
          if  unsigned(S_AXI_AWLEN) = axi_burst_cnt then
            wb_cti <= "111";
          elsif S_AXI_AWBURST = "01" then
            wb_cti <= "010";
          elsif S_AXI_AWBURST = "00" then
            wb_cti <= "001";
          else
            wb_cti <= "000";
          end if;
        elsif axi_awv_awr_flag = '1' then
          if (unsigned(S_AXI_AWLEN) = axi_burst_cnt + 1) then
            wb_cti <= "111";
          end if;
        -- READ
        elsif S_AXI_ARVALID = '1' then
          if unsigned(S_AXI_ARLEN) = axi_burst_cnt then
            wb_cti <= "111";
          elsif S_AXI_ARBURST = "01" then
            wb_cti <= "010";
          elsif S_AXI_ARBURST = "00" then
            wb_cti <= "001";
          else
            wb_cti <= "000";
          end if;
        elsif axi_arv_arr_flag = '1' then
          if (unsigned(S_AXI_ARLEN) = axi_burst_cnt + 1) then
            wb_cti <= "111";
          end if;
        end if;
        
        
          
        -- wb_stb
        -- Read
        if S_AXI_ARVALID = '1' and axi_awv_awr_flag = '0' and S_AXI_AWVALID = '0' then
          wb_stb <= '1';
        elsif axi_arv_arr_flag = '1' then 
          if wb_cti = "000" or wb_cti = "111" then
            if M_WB_ACK_I = '1' then 
              wb_stb <= '0';
            end if;
          elsif wb_cti = "001" or wb_cti = "010" then
            if S_AXI_RREADY = '1' and axi_arv_arr_flag = '1' then
              wb_stb <= '1';
            else
              wb_stb <= '0';
            end if;
          end if;
        -- Write
        elsif S_AXI_AWVALID = '1' and axi_arv_arr_flag = '0' then
          wb_stb <= '1';
        elsif axi_awv_awr_flag = '1' then
          if wb_cti = "000" or wb_cti = "111" then
            if M_WB_ACK_I = '1' then 
              wb_stb <= '0';
            end if;
          elsif wb_cti = "001" or wb_cti = "010" then
            if S_AXI_WVALID = '1' and axi_awv_awr_flag = '1' then
              wb_stb <= '1';
            else
              wb_stb <= '0';
            end if;
          end if;
        end if;
        
        
        
      end if; -- rst
    end if; -- rising_edge
  end process;

end arch_imp;
