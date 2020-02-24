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
--  Wishbone memory simulation module. This module supports a registered
--  feedback read/write cycles.
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

library std;
use std.textio.all;

entity wb_memory is
    generic (
                WB_SLV_ADDR : natural := 16#00#;
                CFG_NUT_MEM_SIZE : natural := 8 * MB;
                LOAD_PROG_DATA : boolean := true;
                PROG_DATA : mem_type
            );
    port (
             -- Ports (WISHBONE slave)
             clk_i   : in std_logic;
             rst_i   : in std_logic;
             stb_i   : in std_logic;                    -- strobe output
             cyc_i   : in std_logic;                    -- cycle valid output
             we_i    : in std_logic;                    -- indicates write transfer
             sel_i   : in std_logic_vector((CFG_MEMU_BUSIF_WIDTH/8)-1 downto 0);  -- byte select outputs
             ack_o   : out std_logic;                   -- normal termination
             err_o   : out std_logic;                   -- termination w/ error
             rty_o   : out std_logic;                   -- termination w/ retry
             adr_i   : in TWord;                        -- address bus outputs
             dat_i   : in std_logic_vector(CFG_MEMU_BUSIF_WIDTH-1 downto 0); -- input data bus
             dat_o   : out std_logic_vector(CFG_MEMU_BUSIF_WIDTH-1 downto 0); -- output data bus
             cti_i   : in std_logic_vector(2 downto 0); -- cycle type identifier
             bte_i   : in std_logic_vector(1 downto 0)  -- burst type extension
         );
end wb_memory;

architecture behav of wb_memory is

    constant WRITE_DELAY : integer := 5;
    constant READ_DELAY : integer := 2;

begin

    process

        variable mem : mem_type(0 to CFG_NUT_MEM_SIZE/4-1) := (others => X"00000000");

        impure function read_mem (addr : TWord; sel : TByteSel) return TWord is
            variable rdata, temp : TWord;
            variable paddr : TWord := addr(29 downto 0) & "00";
        begin
            rdata := mem(conv_integer(addr));
			if (CFG_NUT_LITTLE_ENDIAN) then
				for i in 0 to 3 loop
					temp(7+8*i downto 8*i) := rdata(31-8*i downto 24-8*i);
				end loop;
				rdata := temp;
			end if;
            for i in 0 to 3 loop
                if (sel(i) = '0') then
                    rdata(7+8*i downto 8*i) := X"00";
                end if;
            end loop;
            return rdata;
        end;

        procedure write_mem (addr : TWord; data : TWord; sel : TByteSel) is
            variable wdata,temp : TWord;
            variable paddr : TWord := addr(29 downto 0) & "00";
            
        begin
            wdata := mem(conv_integer(addr));
            if (CFG_NUT_LITTLE_ENDIAN) then
              --for i in 0 to 3 loop
              --	temp(7+8*i downto 8*i) := wdata(31-8*i downto 24-8*i);
              --end loop;
              --wdata := temp;
              if (sel = "1111") then
                for i in 0 to 3 loop
                      if (sel(i) = '1') then
                          wdata(7+8*i downto 8*i) := data(31-8*i downto 24-8*i);
                      end if;
                  end loop;
              else 
                for i in 0 to 3 loop
                      if (sel(i) = '1') then
                          wdata(31-8*i downto 24-8*i) := data(7+8*i downto 8*i);
                      end if;
                  end loop;
              end if;
            else
              for i in 0 to 3 loop
                  if (sel(i) = '1') then
                      wdata(7+8*i downto 8*i) := data(7+8*i downto 8*i);
                  end if;
              end loop;
            end if;
            mem(conv_integer(addr)) := wdata;
        end procedure;

        variable adr : TWord;
        variable data : std_logic_vector(CFG_MEMU_BUSIF_WIDTH-1 downto 0);
        variable sel : std_logic_vector((CFG_MEMU_BUSIF_WIDTH/8)-1 downto 0);
        
        constant BUSRT_ADR_OFF : natural := (CFG_MEMU_BUSIF_WIDTH/64);
        constant BURST_ADR_ADD : natural := (CFG_MEMU_BUSIF_WIDTH/32);
        
        subtype BOT_WORD_RANGE is natural range 31 downto 0;
        subtype TOP_WORD_RANGE is natural range CFG_MEMU_BUSIF_WIDTH-1 downto CFG_MEMU_BUSIF_WIDTH-32;   
        subtype BOT_SEL_RANGE is natural range 3 downto 0;
        subtype TOP_SEL_RANGE is natural range (CFG_MEMU_BUSIF_WIDTH/8)-1 downto (CFG_MEMU_BUSIF_WIDTH/8)-4;   
        
        variable l : line;
    begin

        err_o <= '0';
        rty_o <= '0';

        if (LOAD_PROG_DATA) then
            mem(0 to PROG_DATA'high) := PROG_DATA;
        end if;

        dat_o <= (others => 'Z');
        ack_o <= 'Z';
        while true loop
            wait until clk_i = '1';
            if (stb_i = '1' and cyc_i = '1') then
                adr := "00" & (adr_i(31 downto 24) xor std_logic_vector(to_unsigned(WB_SLV_ADDR, 8))) &  adr_i(23 downto 2) ;
                --INFO("ADR:  0x" & hstr(adr) & " ADRI:  0x" & hstr(adr_i));
                sel := sel_i;
                data := dat_i;
                if (unsigned(adr_i(31 downto 24)) >= WB_SLV_ADDR)  then
                    if (unsigned(adr_i) < (WB_SLV_ADDR*2**24 + CFG_NUT_MEM_SIZE)) then
                        if (we_i = '1') then
                            -- write...
                            for i in 0 to WRITE_DELAY-1 loop
                                wait until clk_i = '1';
                            end loop;
                            if (conv_integer(adr) = 16#000000C0#) then
                              -- Write to tohost-> output
                              -- TBD: This only works for now, if the tohost address changes this word
                              --      will probaply mess up the simulation
                              --INFO ("UART: 0x" & hstr(data));
                              put_char(l, slv2ascii(data));
                              ack_o <= '1';
                              wait until clk_i = '1';
                            else
                              if (cti_i = "000") then
                                  write_mem(adr, data(BOT_WORD_RANGE), sel(BOT_SEL_RANGE));
                                  if (CFG_MEMU_BUSIF_WIDTH = 64) then
                                      write_mem(adr+1, data(TOP_WORD_RANGE), sel(TOP_SEL_RANGE));
                                  end if;
                                  ack_o <= '1';
                                  wait until clk_i = '1';
                              elsif (cti_i = "010") then
                                  ack_o <= '1';
                                  wait until clk_i = '1';
                                  loop
                                      if (stb_i = '1') then
                                          data := dat_i;
                                          sel := sel_i;
                                          write_mem(adr, data(BOT_WORD_RANGE), sel(BOT_SEL_RANGE));
                                          if (CFG_MEMU_BUSIF_WIDTH = 64) then
                                              write_mem(adr+1, data(TOP_WORD_RANGE), sel(TOP_SEL_RANGE));
                                          end if;
                                          case bte_i is
                                              when "01" => adr(1+BUSRT_ADR_OFF downto 0) := adr(1+BUSRT_ADR_OFF downto 0) + BURST_ADR_ADD;
                                              when "10" => adr(2+BUSRT_ADR_OFF downto 0) := adr(2+BUSRT_ADR_OFF downto 0) + BURST_ADR_ADD;
                                              when "11" => adr(3+BUSRT_ADR_OFF downto 0) := adr(3+BUSRT_ADR_OFF downto 0) + BURST_ADR_ADD;
                                              when others => adr := adr + BURST_ADR_ADD;
                                          end case;
                                          ack_o <= '1';
                                      else
                                          ack_o <= '0';
                                      end if;
                                      exit when (cti_i = "111");
                                      wait until clk_i = '1';
                                  end loop;
                              end if;
                            end if;
                        else
                            -- read...
                            for i in 0 to READ_DELAY-1 loop
                                wait until clk_i = '1';
                            end loop;
                            if (cti_i = "000") then
                                dat_o(BOT_WORD_RANGE) <= read_mem(adr, sel(BOT_SEL_RANGE));
                                if (CFG_MEMU_BUSIF_WIDTH = 64) then
                                    dat_o(TOP_WORD_RANGE) <= read_mem(adr+1, sel(TOP_SEL_RANGE));
                                end if;
                                ack_o <= '1';
                                wait until clk_i = '1';
                            elsif (cti_i = "010") then
                                wait until clk_i = '1';
                                while (cti_i /= "111") loop
                                    if (stb_i = '1') then
                                        dat_o(BOT_WORD_RANGE) <= read_mem(adr, sel(BOT_SEL_RANGE));
                                        if (CFG_MEMU_BUSIF_WIDTH = 64) then
                                            dat_o(TOP_WORD_RANGE) <= read_mem(adr+1, sel(TOP_SEL_RANGE));
                                        end if;
                                        sel := sel_i;
                                        case bte_i is
                                            when "01" => adr(1+BUSRT_ADR_OFF downto 0) := adr(1+BUSRT_ADR_OFF downto 0) + BURST_ADR_ADD;
                                            when "10" => adr(2+BUSRT_ADR_OFF downto 0) := adr(2+BUSRT_ADR_OFF downto 0) + BURST_ADR_ADD;
                                            when "11" => adr(3+BUSRT_ADR_OFF downto 0) := adr(3+BUSRT_ADR_OFF downto 0) + BURST_ADR_ADD;
                                            when others => adr := adr + BURST_ADR_ADD;
                                        end case;
                                        ack_o <= '1';
                                    else
                                        ack_o <= '0';
                                    end if;
                                    wait until clk_i = '1';
                                end loop;
                            end if;
                        end if;
                        ack_o <= 'Z';
                        dat_o <= (others => 'Z');
                    else
                        if (we_i = '1') then
                            INFO ("MEM write to non-existing address: 0x" & hstr(adr_i) & " : 0x" & hstr(dat_i));
                        else
                            INFO ("MEM read from non-existing address: 0x" & hstr(adr_i));
                            
                            INFO ("First adr: 0x" & hstr(to_unsigned(WB_SLV_ADDR*2**24, 32)) & " Last adr: 0x" & hstr(to_unsigned(WB_SLV_ADDR*2**24 + CFG_NUT_MEM_SIZE, 32)));
                        end if;
                    end if;
                end if;
            end if;
        end loop;
    end process;

end behav;

