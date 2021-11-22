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


entity swb2maxi is
  generic (
    -- Base address of targeted slave
    C_M_TARGET_SLAVE_BASE_ADDR  : std_logic_vector  := x"40000000";
    -- Burst Length. Supports 1, 2, 4, 8, 16, 32, 64, 128, 256 burst lengths
    C_M_AXI_BURST_LEN  : integer := 16;
    -- Thread ID Width
    C_M_AXI_ID_WIDTH : integer := 1;
    -- Width of Address Bus
    C_M_AXI_ADDR_WIDTH : integer := 32;
    -- Width of Data Bus
    C_M_AXI_DATA_WIDTH : integer := 32;
    -- Width of User Write Address Bus
    C_M_AXI_AWUSER_WIDTH : integer := 0;
    -- Width of User Read Address Bus
    C_M_AXI_ARUSER_WIDTH : integer := 0;
    -- Width of User Write Data Bus
    C_M_AXI_WUSER_WIDTH : integer := 0;
    -- Width of User Read Data Bus
    C_M_AXI_RUSER_WIDTH : integer := 0;
    -- Width of User Response Bus
    C_M_AXI_BUSER_WIDTH : integer := 0
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
end swb2maxi;

architecture implementation of swb2maxi is


  -- function called clogb2 that returns an integer which has the
  --value of the ceiling of the log base 2myip_v1.0

  function clogb2 (bit_depth : integer) return integer is
     variable depth  : integer := bit_depth;
     variable count  : integer := 1;
   begin
      for i in 1 to bit_depth loop  -- Works for up to 32 bit integers
        if (bit_depth <= 2) then
          count := 1;
        else
          if(depth <= 1) then
            count := count;
          else
            depth := depth / 2;
            count := count + 1;
          end if;
        end if;
     end loop;
     return(count);
   end;

  -- C_TRANSACTIONS_NUM is the width of the index counter for
  -- number of beats in a burst write or burst read transaction.
   constant  C_TRANSACTIONS_NUM : integer := clogb2(C_M_AXI_BURST_LEN-1);
  -- Burst length for transactions, in C_M_AXI_DATA_WIDTHs.
  -- Non-2^n lengths will eventually cause bursts across 4K address boundaries.
   constant  C_MASTER_LENGTH  : integer := 12;
  -- total number of burst transfers is master length divided by burst length and burst size
   constant  C_NO_BURSTS_REQ  : integer := (C_MASTER_LENGTH-clogb2((C_M_AXI_BURST_LEN*C_M_AXI_DATA_WIDTH/8)-1));
  -- Example State machine to initialize counter, initialize write transactions,
   -- initialize read transactions and comparison of read data with the
   -- written data words.

  -- AXI4FULL signals
  --AXI4 internal temp signals
  signal axi_awvalid  : std_logic;
  signal axi_wlast  : std_logic;
  signal axi_wvalid  : std_logic;
  signal axi_bready  : std_logic;
  signal axi_arvalid  : std_logic;
  signal axi_rready  : std_logic;
  signal burst_counter  : unsigned(4 downto 0);
  signal wnext  : std_logic;
  signal rnext  : std_logic;
  signal awv_awr_flag  : std_logic;
  signal arv_arr_flag  : std_logic;
  ---------------------------
  -- WISHBONE Slave interface
  ---------------------------

  signal wb_ack      :  std_logic;
  signal wb_stall    :  std_logic;
  signal wb_err      :  std_logic;
  signal block_wb    :  std_logic;

begin

  block_wb <= '1' when S_WB_CYC_I = '0' and (awv_awr_flag = '1' or arv_arr_flag = '1') else '0';

  M_AXI_AWID  <= (others => '0');
  M_AXI_AWSIZE  <= std_logic_vector( to_unsigned(clogb2((C_M_AXI_DATA_WIDTH/8)-1), 3) );
  M_AXI_AWQOS  <= (others => '0');
  M_AXI_AWUSER  <= (others => '0');
  M_AXI_AWVALID  <= axi_awvalid;
  M_AXI_WDATA  <= S_WB_DAT_IN;
  M_AXI_WSTRB <= S_WB_SEL_I when block_wb = '0' else (others => '0');
  M_AXI_WLAST  <= axi_wlast;
  M_AXI_WUSER  <= (others => '0');
  M_AXI_WVALID  <= axi_wvalid;
  M_AXI_BREADY  <= axi_bready;

  M_AXI_ARID  <= (others => '0');
  M_AXI_ARSIZE  <= std_logic_vector( to_unsigned( clogb2((C_M_AXI_DATA_WIDTH/8)-1),3 ));
  M_AXI_ARQOS  <= (others => '0');
  M_AXI_ARUSER  <= (others => '0');
  M_AXI_ARVALID  <= axi_arvalid;
  M_AXI_RREADY  <= axi_rready;


  -----------------------------------
  -- WISHBONE Slave interface signals
  -----------------------------------
  S_WB_DAT_OUT  <= M_AXI_RDATA;
  S_WB_ACK_O    <= wb_ack;
  S_WB_STALL_O  <= wb_stall;
  S_WB_ERR_O    <= wb_err;

  -- Fix no driver warnings during synthesis
  -- TODO: Is this the wanted behaviour?
  wb_stall <= '0';
  wb_err <= '0';

  -- Burst counter
  process(M_AXI_ACLK)
  begin
    if rising_edge(M_AXI_ACLK) then
      if M_AXI_ARESETN = '0' then
        burst_counter <= (others => '0');
      else
        if wnext = '1' or rnext = '1' then
          burst_counter <= burst_counter + 1;
        elsif awv_awr_flag = '0' and arv_arr_flag = '0' then
          burst_counter <= (others => '0');
        end if;
      end if;
    end if;
  end process;

  M_AXI_AWADDR <= S_WB_ADR_I;
  M_AXI_AWLOCK <= S_WB_LOCK_I;
  axi_wvalid <= S_WB_STB_I when awv_awr_flag = '1' and axi_bready = '0' else '0';
  wnext <= M_AXI_WREADY and axi_wvalid;

  M_AXI_ARADDR <= S_WB_ADR_I;
  M_AXI_ARLOCK <= S_WB_LOCK_I;
  axi_rready <= S_WB_STB_I when arv_arr_flag = '1' else '0';
  rnext <= M_AXI_RVALID and axi_rready;

  -- Use recommended values for cache und prot
  M_AXI_AWCACHE  <= "0011";
  M_AXI_AWPROT    <= "000";

  AWADDR : process(M_AXI_ACLK)
  begin
    if rising_edge(M_AXI_ACLK) then
      if M_AXI_ARESETN = '0' then
        M_AXI_AWLEN <= (others => '0');
        M_AXI_AWBURST <= (others => '0');
      elsif S_WB_CYC_I = '1' and S_WB_WE_I = '1' and axi_bready = '0' then
        if S_WB_CTI_I = "001" then
          M_AXI_AWBURST <= "00";
          M_AXI_AWLEN <= std_logic_vector(to_unsigned(C_M_AXI_BURST_LEN-1, 8));
        elsif S_WB_CTI_I = "010" then
          M_AXI_AWBURST <= "01";
          M_AXI_AWLEN <= std_logic_vector(to_unsigned(C_M_AXI_BURST_LEN-1, 8));
        --elsif S_WB_CTI_I = "000" then
        --  M_AXI_AWBURST <= "01";
        --  M_AXI_AWLEN <= std_logic_vector(to_unsigned(C_M_AXI_BURST_LEN-1, 8));
        elsif S_WB_CTI_I = "111" or S_WB_CTI_I = "000" then
          M_AXI_AWBURST <= "01";
          M_AXI_AWLEN <= (others => '0');
        end if;
      else
          M_AXI_AWLEN <= (others => '0');
          M_AXI_AWBURST <= (others => '0');
      end if;
    end if;
  end process;


  AWVALID_GEN : process(M_AXI_ACLK)
  begin
    if (rising_edge (M_AXI_ACLK)) then
      if (M_AXI_ARESETN = '0') then
        axi_awvalid <= '0';
        awv_awr_flag <= '0';
      else
        if axi_awvalid = '0' and S_WB_WE_I = '1' and S_WB_CYC_I = '1' and awv_awr_flag = '0' then
          axi_awvalid <= '1';
        elsif (M_AXI_AWREADY = '1' and axi_awvalid = '1') then
          awv_awr_flag <= '1';
          axi_awvalid <= '0';
        elsif M_AXI_BVALID = '1' and axi_bready = '1' then
          awv_awr_flag <= '0';
        else
          axi_awvalid <= axi_awvalid;
          awv_awr_flag <= awv_awr_flag;
        end if;
      end if;
    end if;
  end process;


  BREADY_GEN : process(M_AXI_ACLK)
  begin
    if (rising_edge (M_AXI_ACLK)) then
      if M_AXI_ARESETN = '0' then
        axi_bready <= '0';
      else
        if axi_bready = '0' and wnext = '1' and axi_wlast = '1' then
          axi_bready <= '1';
        elsif M_AXI_BVALID = '1' and axi_bready = '1' then
          axi_bready <= '0';
        end if;
      end if;
    end if;
  end process;


  WLAST_GEN : process(M_AXI_ACLK)
  begin
    if (rising_edge (M_AXI_ACLK)) then
      if M_AXI_ARESETN = '0'then
        axi_wlast <= '0';
      else
        if S_WB_CYC_I = '1' and S_WB_WE_I = '1' then
          if axi_wlast = '0' and M_AXI_BVALID = '0' then
            if S_WB_CTI_I = "001" or S_WB_CTI_I = "010" then
              if burst_counter = C_M_AXI_BURST_LEN-2 then
                axi_wlast <= '1';
              end if;
            --elsif S_WB_CTI_I = "000" then
            --  if burst_counter = C_M_AXI_BURST_LEN-1 then
            --    axi_wlast <= '1';
            --  end if;
            elsif S_WB_CTI_I = "111" or S_WB_CTI_I = "000" then
              axi_wlast <= '1';
            end if;
          elsif axi_wlast = '1' and axi_wvalid = '1' and M_AXI_WREADY = '1' then
            axi_wlast <= '0';
          else
            axi_wlast <= axi_wlast;
          end if;
        else
          axi_wlast <= '0';
        end if;
      end if;
    end if;
  end process;

  -- Use recommended values for cache und prot
  M_AXI_ARCACHE    <= "0011";
  M_AXI_ARPROT    <= "000";

  ARADDR : process(M_AXI_ACLK)
  begin
    if rising_edge(M_AXI_ACLK) then
      if M_AXI_ARESETN = '0' then
        M_AXI_ARLEN <= (others => '0');
        M_AXI_ARBURST <= (others => '0');

      elsif S_WB_CYC_I = '1' and S_WB_WE_I = '0' then
        if S_WB_CTI_I = "001" then
          M_AXI_ARBURST <= "00";
          M_AXI_ARLEN <= std_logic_vector(to_unsigned(C_M_AXI_BURST_LEN-1, 8));
        elsif S_WB_CTI_I = "010" then
          M_AXI_ARBURST <= "01";
          M_AXI_ARLEN <= std_logic_vector(to_unsigned(C_M_AXI_BURST_LEN-1, 8));
        --elsif S_WB_CTI_I = "000" then
        --  M_AXI_ARBURST <= "01";
        --  M_AXI_ARLEN <= std_logic_vector(to_unsigned(C_M_AXI_BURST_LEN-1, 8));
        elsif S_WB_CTI_I = "111" or S_WB_CTI_I = "000" then
          M_AXI_ARBURST <= "01";
          M_AXI_ARLEN <= (others => '0');
        end if;
      else
        M_AXI_ARLEN <= (others => '0');
        M_AXI_ARBURST <= (others => '0');
      end if;
    end if;
  end process;

  ARVALID_GEN : process(M_AXI_ACLK)
  begin
    if rising_edge (M_AXI_ACLK) then
      if M_AXI_ARESETN = '0' then
        axi_arvalid <= '0';
        arv_arr_flag <= '0';
      else
        if axi_arvalid = '0' and S_WB_WE_I = '0' and S_WB_CYC_I = '1' and arv_arr_flag = '0' then
          axi_arvalid <= '1';
        elsif M_AXI_ARREADY = '1' and axi_arvalid = '1' then
          arv_arr_flag <= '1';
          axi_arvalid <= '0';
        elsif rnext = '1' and M_AXI_RLAST = '1' then
          arv_arr_flag <= '0';
        end if;
      end if;
    end if;
  end process;

  -- WB_ACK
  wb_ack <= M_AXI_WREADY when awv_awr_flag = '1' and (S_WB_CTI_I = "001" or S_WB_CTI_I = "010") else
            M_AXI_BVALID when awv_awr_flag = '1' and (S_WB_CTI_I = "111" or S_WB_CTI_I = "000") and M_AXI_BRESP = "00" else
            M_AXI_RVALID when arv_arr_flag = '1' else
            '0';

end implementation;
