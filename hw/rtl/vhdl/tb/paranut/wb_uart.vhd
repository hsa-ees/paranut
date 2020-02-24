--------------------------------------------------------------------------------
-- This file is part of the ParaNut project.
-- 
-- Copyright (C) 2013-2019  Alexander Bahle <alexander.bahle@hs-augsburg.de>
-- 							Michael Seider, <michael.seider@hs-augsburg.de>
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
--  Wishbone interface for the UART module for simulation. 
--
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library std;
use std.textio.all;

library paranut;
use paranut.paranut_config.all;
use paranut.types.all;
use paranut.paranut_lib.all;
use paranut.text_io.all;
use paranut.txt_util.all;

entity wb_uart is
    generic (
                WB_SLV_ADDR : natural := 16#90#
            );
    port (
             -- Ports (WISHBONE slave)
             clk_i   : in std_logic;
             rst_i   : in std_logic;
             stb_i   : in std_logic;    -- strobe output
             cyc_i   : in std_logic;    -- cycle valid output
             we_i    : in std_logic;    -- indicates write transfer
             sel_i   : in TByteSel;     -- byte select outputs
             ack_o   : out std_logic;   -- normal termination
             err_o   : out std_logic;   -- termination w/ error
             rty_o   : out std_logic;   -- termination w/ retry
             adr_i   : in TWord;        -- address bus outputs
             dat_i   : in TWord;        -- input data bus
             dat_o   : out TWord        -- outout data bus
         );
end wb_uart;

architecture behav of wb_uart is

    constant WRITE_DELAY : integer := 10;
    constant READ_DELAY : integer := 10;

    signal int                      : std_logic;
    signal baudce, rclk, baudoutn   : std_logic;
    signal out1n, out2n, rtsn, dtrn, ctsn, dsrn, dcdn, rin, sin, sout : std_logic;
    signal uart_din, uart_dout : std_logic_vector(7 downto 0);
    signal uart_cyc, uart_stb, uart_we, uart_ack : std_logic;
    signal uart_adr : TWord;

    component uart_16750 is
    port (
        CLK         : in std_logic;                             -- Clock
        RST         : in std_logic;                             -- Reset
        BAUDCE      : in std_logic;                             -- Baudrate generator clock enable
        WB_CYC      : in std_logic;                             -- Chip select
        WB_STB      : in std_logic;                             -- Chip select
        WB_WE       : in std_logic;                             -- Write/NotRead to/from UART
        WB_ADR      : in std_logic_vector(31 downto 0);         -- Address input
        WB_DIN      : in std_logic_vector(7 downto 0);          -- Data bus input
        WB_DOUT     : out std_logic_vector(7 downto 0);         -- Data bus output
        WB_ACK      : out std_logic;                            -- Transaction uart_ack
        INT         : out std_logic;                            -- Interrupt output
        OUT1N       : out std_logic;                            -- Output 1
        OUT2N       : out std_logic;                            -- Output 2
        RCLK        : in std_logic;                             -- Receiver clock (16x baudrate)
        BAUDOUTN    : out std_logic;                            -- Baudrate generator output (16x baudrate)
        RTSN        : out std_logic;                            -- RTS output
        DTRN        : out std_logic;                            -- DTR output
        CTSN        : in std_logic;                             -- CTS input
        DSRN        : in std_logic;                             -- DSR input
        DCDN        : in std_logic;                             -- DCD input
        RIN         : in std_logic;                             -- RI input
        SIN         : in std_logic;                             -- Receiver input
        SOUT        : out std_logic                             -- Transmitter output
    );
    end component;

    component slib_clock_div is
        generic (
            RATIO       : integer := 18     -- Clock divider ratio
        );
        port (
            CLK         : in std_logic;     -- Clock
            RST         : in std_logic;     -- Reset
            CE          : in std_logic;     -- Clock enable input
            Q           : out std_logic     -- New clock enable output
        );
    end component;

begin

    rty_o <= '0';
    err_o <= '0';

    process
        variable adr, val : TWord;
		variable l : line;
    begin

        while true loop
            wait until clk_i = '1';
            if (stb_i = '1' and cyc_i = '1') then
                if (conv_integer(adr_i(31 downto 24)) = WB_SLV_ADDR) then
                    if (we_i = '1') then
                        wait until uart_ack = '1';
						put_char(l,slv2ascii(dat_i));
                    else
                        wait until uart_ack = '1';
                    end if;
                    wait until clk_i = '1';
                end if;
            end if;
        end loop;
    end process;

    uart_cyc <= cyc_i when conv_integer(adr_i(31 downto 24)) = WB_SLV_ADDR else
                '0';
    uart_stb <= stb_i when conv_integer(adr_i(31 downto 24)) = WB_SLV_ADDR else
                '0';
    uart_we <= we_i;
    --uart_adr <= std_logic_vector(unsigned(adr_i) + X"00000000") when sel_i = "1000" else
    --            std_logic_vector(unsigned(adr_i) + X"00000001") when sel_i = "0100" else
    --            std_logic_vector(unsigned(adr_i) + X"00000002") when sel_i = "0010" else
    --            std_logic_vector(unsigned(adr_i) + X"00000003") when sel_i = "0001" else
    --            adr_i;
    uart_adr <= adr_i;
    uart_din <= dat_i(31 downto 24) when sel_i = "1000" else
                dat_i(23 downto 16) when sel_i = "0100" else
                dat_i(15 downto  8) when sel_i = "0010" else
                dat_i( 7 downto  0) when sel_i = "0001" else
                X"00";
    ack_o <= uart_ack when conv_integer(adr_i(31 downto 24)) = WB_SLV_ADDR else
             'Z';
    dat_o <= uart_dout & X"000000"       when sel_i = "1000" and conv_integer(adr_i(31 downto 24)) = WB_SLV_ADDR else
             X"00" & uart_dout & X"0000" when sel_i = "0100" and conv_integer(adr_i(31 downto 24)) = WB_SLV_ADDR else
             X"0000" & uart_dout & X"00" when sel_i = "0010" and conv_integer(adr_i(31 downto 24)) = WB_SLV_ADDR else
             X"000000" & uart_dout       when sel_i = "0001" and conv_integer(adr_i(31 downto 24)) = WB_SLV_ADDR else
             (others => 'Z');

    BGCE: slib_clock_div generic map (RATIO => 1) port map (clk_i, rst_i, '1', baudce);

    rclk <= baudoutn;
    ctsn <= '0';

    uart : uart_16750
    port map (  CLK     => clk_i,
                RST     => rst_i,
                BAUDCE  => BAUDCE,
                WB_CYC  => uart_cyc,
                WB_STB  => uart_stb,
                WB_WE   => we_i,
                WB_ADR  => uart_adr,
                WB_DIN  => uart_din,
                WB_DOUT => uart_dout,
                WB_ACK  => uart_ack,
                INT     => int,
                OUT1N   => out1n,
                OUT2N   => out2n,
                RCLK    => rclk,
                BAUDOUTN=> baudoutn,
                RTSN    => rtsn,
                DTRN    => dtrn,
                CTSN    => ctsn,
                DSRN    => dsrn,
                DCDN    => dcdn,
                RIN     => rin,
                SIN     => sin,
                SOUT    => sout
            );
end behav;
