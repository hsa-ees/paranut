--------------------------------------------------------------------------------
-- This file is part of the ParaNut project.
-- 
-- Copyright (C) 2013  Michael Seider, <michael.seider@hs-augsburg.de>
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
--  Memory unit (MEMU) testbench.
--
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library paranut;
use paranut.paranut_config.all;
use paranut.types.all;
use paranut.paranut_lib.all;
use paranut.memu.all;
use paranut.memu_lib.all;
use paranut.peripherals.all;
use paranut.tb_monitor.all;
use paranut.text_io.all;

entity mmemu_tb is
end mmemu_tb;

architecture tb of mmemu_tb is

    constant MEM_WB_SLV_ADDR : natural := 16#00#;

    signal clk    : std_logic;
    signal reset  : std_logic;
    signal mi : memu_in_type;
    signal mo : memu_out_type;
    signal sel    : TByteSel;

    constant clk_period : time := 10 ns;

    signal halt: boolean := false;

    constant prog : mem_type(0 to 0) := (others => X"00000000");

begin

    bsel_le : if (CFG_NUT_LITTLE_ENDIAN) generate
        sel <= mo.bifwbo.sel_o;
    end generate;
    bsel_be : if (not CFG_NUT_LITTLE_ENDIAN) generate
        sel(3) <= mo.bifwbo.sel_o(0);
        sel(2) <= mo.bifwbo.sel_o(1);
        sel(1) <= mo.bifwbo.sel_o(2);
        sel(0) <= mo.bifwbo.sel_o(3);
    end generate;

    uut : mmemu
    port map (clk, reset, mi, mo);

    memory : wb_memory
    generic map (WB_SLV_ADDR => MEM_WB_SLV_ADDR, CFG_NUT_MEM_SIZE => 8 * MB, LOAD_PROG_DATA => false, PROG_DATA => prog)
    port map (clk, reset, mo.bifwbo.stb_o, mo.bifwbo.cyc_o, mo.bifwbo.we_o,
    sel, mi.bifwbi.ack_i, mi.bifwbi.err_i, mi.bifwbi.rty_i, mo.bifwbo.adr_o,
    mo.bifwbo.dat_o, mi.bifwbi.dat_i, mo.bifwbo.cti_o, mo.bifwbo.bte_o);

    --clock: process
    --begin
    --    while (not halt) loop
    --        clk <= '0'; wait for clk_period/2;
    --        clk <= '1'; wait for clk_period/2;
    --    end loop;
    --    wait;
    --end process;

    tb: process

        procedure run_cycle(n : integer := 1) is
        begin
            for i in 0 to n-1 loop
                clk <= '1'; wait for clk_period/2;
                clk <= '0'; wait for clk_period/2;
            end loop;
        end procedure;

        -- Init
        procedure clear_read_port (p : integer) is
        begin
            mi.rpi(p).port_rd <= '0';
            mi.rpi(p).port_direct <= '0';
            mi.rpi(p).port_bsel <= (others => '0');
            mi.rpi(p).port_adr <= (others => '0');
        end procedure;

        procedure clear_write_port (p : integer) is
        begin
            mi.wpi(p).port_wr <= '0';
            mi.wpi(p).port_direct <= '0';
            mi.wpi(p).port_bsel <= (others => '0');
            mi.wpi(p).port_rlink_wcond <= '0';
            mi.wpi(p).port_writeback <= '0';
            mi.wpi(p).port_invalidate <= '0';
            mi.wpi(p).port_adr <= (others => '0');
            mi.wpi(p).port_data <= (others => '0');
        end procedure;

        -- Read
        procedure read_init (p : integer; adr : TWord; bsel : TByteSel := X"f") is
        begin
            mi.rpi(p).port_adr <= adr;
            mi.rpi(p).port_bsel <= bsel;
            mi.rpi(p).port_rd <= '1';
        end procedure;

        impure function read_check_ack (p : integer) return boolean is
            variable ret : boolean := false;
        begin
            if (mo.rpo(p).port_ack = '1') then
                mi.rpi(p).port_rd <= '0';
                mi.rpi(p).port_adr <= (others => '1');
                ret := true;
            end if;
            return (ret);
        end function;

        impure function read_get_data (p : integer) return TWord is
        begin
            return (mo.rpo(p).port_data);
        end function;

        procedure read_complete (p : integer; data : out TWord) is
        begin
            loop
                run_cycle;
                exit when read_check_ack(p);
            end loop;
            run_cycle;
            data := read_get_data(p);
        end procedure;

        procedure read (p : integer; adr : TWord; data : out TWord; bsel : TByteSel := X"f") is
        begin
            read_init(p, adr, bsel);
            read_complete(p, data);
        end procedure;

        -- Write
        procedure write_init (p : integer; adr, data : TWord; bsel : TByteSel := X"f") is
        begin
            mi.wpi(p).port_adr <= adr;
            mi.wpi(p).port_data <= data;
            mi.wpi(p).port_bsel <= bsel;
            mi.wpi(p).port_wr <= '1';
        end procedure;

        procedure write_init_special (p : integer; adr : TWord; writeback, invalidate : std_logic) is
        begin
            mi.wpi(p).port_adr <= adr;
            mi.wpi(p).port_writeback <= writeback;
            mi.wpi(p).port_invalidate <= invalidate;
        end procedure;

        impure function write_try_complete (p : integer) return boolean is
            variable ret : boolean := false;
        begin
            if (mo.wpo(p).port_ack = '1') then
                mi.wpi(p).port_wr <= '0';
                mi.wpi(p).port_writeback <= '0';
                mi.wpi(p).port_invalidate <= '0';
                ret := true;
            end if;
            return (ret);
        end function;

        procedure write_complete (p : integer) is
        begin
            loop
                run_cycle;
                exit when write_try_complete(p);
            end loop;
        end procedure;

        procedure write (p : integer; adr, data : TWord; bsel : TByteSel := X"f") is
        begin
            write_init(p, adr, data, bsel);
            write_complete(p);
        end procedure;

        procedure write_special (p : integer; adr : TWord; writeback, invalidate : std_logic) is
        begin
            write_init_special(p, adr, writeback, invalidate);
            write_complete(p);
        end procedure;

        procedure run_partword_read_write (
                                              porti : in integer;
                                              id_i : in std_logic_vector(7 downto 0);
                                              base_i : in TWord;
                                              stride : in integer := 4;
                                              count : in integer := 1
                                          )
        is
            variable wdata, waddr, data_exp, base_temp : std_logic_vector(63 downto 0);
            variable data, mask, base : TWord;
            variable id : std_logic_vector(7 downto 0);
            variable factor, summand : unsigned(31 downto 0);
        begin

            id := id_i;
            base := base_i;

            -- Write & Read full words...
            INFO ("    write & read full words ...");
            factor := X"01010101";
            summand := X"00010203";
            for n in 0 to count-1 loop
                wdata := std_logic_vector(factor * to_unsigned((conv_integer(id) + 4 * n), 32) + summand);
                waddr := std_logic_vector(unsigned(base) + to_unsigned(n, 32) * stride);
                write (porti, waddr(31 downto 0), wdata(31 downto 0));
            end loop;
            for n in 0 to count-1 loop
                waddr := std_logic_vector(unsigned(base) + to_unsigned(n, 32) * stride);
                data_exp := std_logic_vector(factor * to_unsigned((conv_integer(id) + 4 * n), 32) + summand);
                read (porti, waddr(31 downto 0), data);
                assert data = data_exp(31 downto 0);
            end loop;

            -- Read half words...
            INFO ("    write words & read as half words...");
            factor := X"01010101";
            summand := X"00010203";
            for n in 0 to count-1 loop
                wdata := std_logic_vector(factor * to_unsigned((conv_integer(id) + 4 * n), 32) + summand);
                waddr := std_logic_vector(unsigned(base) + to_unsigned(n, 32) * stride);
                write (porti, waddr(31 downto 0), wdata(31 downto 0));
            end loop;
            factor := X"01010000";
            summand := X"00010000";
            mask := X"ffff0000";
            for n in 0 to count-1 loop
                waddr := std_logic_vector(unsigned(base) + to_unsigned(n, 32) * stride);
                data_exp := std_logic_vector(factor * to_unsigned((conv_integer(id) + 4 * n), 32) + summand);
                read (porti, waddr(31 downto 0), data, X"3");
                assert (data and mask) = data_exp(31 downto 0);
            end loop;
            factor := X"00000101";
            summand := X"00000203";
            mask := X"0000ffff";
            for n in 0 to count-1 loop
                waddr := std_logic_vector(unsigned(base) + to_unsigned(n, 32) * stride);
                data_exp := std_logic_vector(factor * to_unsigned((conv_integer(id) + 4 * n), 32) + summand);
                read (porti, waddr(31 downto 0), data, X"c");
                assert (data and mask) = data_exp(31 downto 0);
            end loop;
            base_temp := std_logic_vector(unsigned(base) + to_unsigned(count, 32) * stride);
            base := base_temp(31 downto 0);
            id := std_logic_vector(unsigned(id) + to_unsigned(4 * count, 8));

            -- Read bytes...
            INFO ("    write words & read as bytes...");
            factor := X"01010101";
            summand := X"00010203";
            for n in 0 to count-1 loop
                wdata := std_logic_vector(factor * to_unsigned((conv_integer(id) + 4 * n), 32) + summand);
                waddr := std_logic_vector(unsigned(base) + to_unsigned(n, 32) * stride);
                write (porti, waddr(31 downto 0), wdata(31 downto 0));
            end loop;
            factor := X"01000000";
            summand := X"00000000";
            mask := X"ff000000";
            for n in 0 to count-1 loop
                waddr := std_logic_vector(unsigned(base) + to_unsigned(n, 32) * stride);
                data_exp := std_logic_vector(factor * to_unsigned((conv_integer(id) + 4 * n), 32) + summand);
                read (porti, waddr(31 downto 0), data, X"1");
                assert (data and mask) = data_exp(31 downto 0);
            end loop;
            factor := X"00010000";
            summand := X"00010000";
            mask := X"00ff0000";
            for n in 0 to count-1 loop
                waddr := std_logic_vector(unsigned(base) + to_unsigned(n, 32) * stride);
                data_exp := std_logic_vector(factor * to_unsigned((conv_integer(id) + 4 * n), 32) + summand);
                read (porti, waddr(31 downto 0), data, X"2");
                assert (data and mask) = data_exp(31 downto 0);
            end loop;
            factor := X"00000100";
            summand := X"00000200";
            mask := X"0000ff00";
            for n in 0 to count-1 loop
                waddr := std_logic_vector(unsigned(base) + to_unsigned(n, 32) * stride);
                data_exp := std_logic_vector(factor * to_unsigned((conv_integer(id) + 4 * n), 32) + summand);
                read (porti, waddr(31 downto 0), data, X"4");
                assert (data and mask) = data_exp(31 downto 0);
            end loop;
            factor := X"00000001";
            summand := X"00000003";
            mask := X"000000ff";
            for n in 0 to count-1 loop
                waddr := std_logic_vector(unsigned(base) + to_unsigned(n, 32) * stride);
                data_exp := std_logic_vector(factor * to_unsigned((conv_integer(id) + 4 * n), 32) + summand);
                read (porti, waddr(31 downto 0), data, X"8");
                assert (data and mask) = data_exp(31 downto 0);
            end loop;
            base_temp := std_logic_vector(unsigned(base) + to_unsigned(count, 32) * stride);
            base := base_temp(31 downto 0);
            id := std_logic_vector(unsigned(id) + to_unsigned(4 * count, 8));

            -- Write half words...
            INFO ("    write half words & read as words...");
            factor := X"01010000";
            summand := X"00010000";
            for n in 0 to count-1 loop
                wdata := std_logic_vector(factor * to_unsigned((conv_integer(id) + 4 * n), 32) + summand);
                waddr := std_logic_vector(unsigned(base) + to_unsigned(n, 32) * stride);
                write (porti, waddr(31 downto 0), wdata(31 downto 0), X"3");
            end loop;
            factor := X"00000101";
            summand := X"00000203";
            for n in 0 to count-1 loop
                wdata := std_logic_vector(factor * to_unsigned((conv_integer(id) + 4 * n), 32) + summand);
                waddr := std_logic_vector(unsigned(base) + to_unsigned(n, 32) * stride);
                write (porti, waddr(31 downto 0), wdata(31 downto 0), X"c");
            end loop;
            factor := X"01010101";
            summand := X"00010203";
            for n in 0 to count-1 loop
                waddr := std_logic_vector(unsigned(base) + to_unsigned(n, 32) * stride);
                data_exp := std_logic_vector(factor * to_unsigned((conv_integer(id) + 4 * n), 32) + summand);
                read (porti, waddr(31 downto 0), data);
                assert data = data_exp(31 downto 0);
            end loop;
            base_temp := std_logic_vector(unsigned(base) + to_unsigned(count, 32) * stride);
            base := base_temp(31 downto 0);
            id := std_logic_vector(unsigned(id) + to_unsigned(4 * count, 8));

            -- Write bytes...
            INFO ("    write bytes & read as words...");
            factor := X"01000000";
            summand := X"00000000";
            for n in 0 to count-1 loop
                wdata := std_logic_vector(factor * to_unsigned((conv_integer(id) + 4 * n), 32) + summand);
                waddr := std_logic_vector(unsigned(base) + to_unsigned(n, 32) * stride);
                write (porti, waddr(31 downto 0), wdata(31 downto 0), X"1");
            end loop;
            factor := X"00010000";
            summand := X"00010000";
            for n in 0 to count-1 loop
                wdata := std_logic_vector(factor * to_unsigned((conv_integer(id) + 4 * n), 32) + summand);
                waddr := std_logic_vector(unsigned(base) + to_unsigned(n, 32) * stride);
                write (porti, waddr(31 downto 0), wdata(31 downto 0), X"2");
            end loop;
            factor := X"00000100";
            summand := X"00000200";
            for n in 0 to count-1 loop
                wdata := std_logic_vector(factor * to_unsigned((conv_integer(id) + 4 * n), 32) + summand);
                waddr := std_logic_vector(unsigned(base) + to_unsigned(n, 32) * stride);
                write (porti, waddr(31 downto 0), wdata(31 downto 0), X"4");
            end loop;
            factor := X"00000001";
            summand := X"00000003";
            for n in 0 to count-1 loop
                wdata := std_logic_vector(factor * to_unsigned((conv_integer(id) + 4 * n), 32) + summand);
                waddr := std_logic_vector(unsigned(base) + to_unsigned(n, 32) * stride);
                write (porti, waddr(31 downto 0), wdata(31 downto 0), X"8");
            end loop;
            factor := X"01010101";
            summand := X"00010203";
            for n in 0 to count-1 loop
                waddr := std_logic_vector(unsigned(base) + to_unsigned(n, 32) * stride);
                data_exp := std_logic_vector(factor * to_unsigned((conv_integer(id) + 4 * n), 32) + summand);
                read (porti, waddr(31 downto 0), data);
                assert data = data_exp(31 downto 0);
            end loop;

        end procedure;

        procedure run_parallel_read_write (
                                              id : in TWord;
                                              base0 : in TWord;
                                              stride0 : in integer;
                                              base1 : in TWord;
                                              stride1 : in integer
                                          )
        is
            variable wdata, addr, data_exp, base_temp : std_logic_vector(63 downto 0);
            variable data, mask, base : TWord;
            variable completed, now_completed : std_logic_vector(2*CFG_NUT_CPU_CORES-1 downto 0);
        begin
            -- Parallel write...
            INFO ("    parallel write...");
            for n in 0 to CFG_NUT_CPU_CORES-1 loop
                addr(31 downto 0) := std_logic_vector(unsigned(base0) + stride0 * n);
                wdata(31 downto 0) := std_logic_vector(unsigned(id) + n);
                write_init(n, addr(31 downto 0), wdata(31 downto 0));
            end loop;
            completed := (others => '0');
            while (completed(CFG_NUT_CPU_CORES-1 downto 0) /= ones64(CFG_NUT_CPU_CORES-1 downto 0)) loop
                for n in 0 to CFG_NUT_CPU_CORES-1 loop
                    if (write_try_complete(n)) then
                        completed(n) := '1';
                    end if;
                end loop;
                run_cycle;
            end loop;

            -- Parallel read (data ports)...
            INFO ("    parallel read (data ports)...");
            for n in 0 to CFG_NUT_CPU_CORES-1 loop
                addr(31 downto 0) := std_logic_vector(unsigned(base0) + stride0 * n);
                read_init(n, addr(31 downto 0));
            end loop;
            completed := (others => '0');
            while (completed(CFG_NUT_CPU_CORES-1 downto 0) /= ones64(CFG_NUT_CPU_CORES-1 downto 0)) loop
                now_completed := (others => '0');
                for n in 0 to CFG_NUT_CPU_CORES-1 loop
                    if (read_check_ack(n)) then
                        now_completed(n) := '1';
                    end if;
                end loop;
                completed := completed or now_completed;
                run_cycle;
                for n in 0 to CFG_NUT_CPU_CORES-1 loop
                    if (now_completed(n) = '1') then
                        assert read_get_data(n) = std_logic_vector((unsigned(id) + n));
                    end if;
                end loop;
            end loop;

            -- Parallel write & read (insn ports)...
            INFO ("    parallel write & read (insn ports)...");
            for n in 0 to CFG_NUT_CPU_CORES-1 loop
                addr(31 downto 0) := std_logic_vector(unsigned(base1) + stride1 * n);
                wdata(31 downto 0) := std_logic_vector(unsigned(id) + CFG_NUT_CPU_CORES + n);
                write_init(n, addr(31 downto 0), wdata(31 downto 0));
                addr(31 downto 0) := std_logic_vector(unsigned(base0) + stride0 * n);
                read_init(CFG_NUT_CPU_CORES+n, addr(31 downto 0));
            end loop;
            completed := (others => '0');
            while (completed /= ones64(2*CFG_NUT_CPU_CORES-1 downto 0)) loop
                now_completed := (others => '0');
                for n in 0 to CFG_NUT_CPU_CORES-1 loop
                    if (write_try_complete(n)) then
                        assert completed(n+CFG_NUT_CPU_CORES) = '1' report "Direct write completes before direct read: wrong priority";
                        completed(n) := '1';
                    end if;
                    if (read_check_ack(CFG_NUT_CPU_CORES+n)) then
                        now_completed(CFG_NUT_CPU_CORES+n) := '1';
                    end if;
                end loop;
                completed := completed or now_completed;
                run_cycle;
                for n in 0 to CFG_NUT_CPU_CORES-1 loop
                    if (now_completed(n+CFG_NUT_CPU_CORES) = '1') then
                        assert read_get_data(n+CFG_NUT_CPU_CORES) = std_logic_vector((unsigned(id) + n));
                    end if;
                end loop;
            end loop;

            -- Parallel read (data ports) to verify last writes...
            INFO ("    parallel read (data ports) to verify the last writes...");
            for n in 0 to CFG_NUT_CPU_CORES-1 loop
                addr(31 downto 0) := std_logic_vector(unsigned(base0) + stride0 * n);
                read_init(CFG_NUT_CPU_CORES+n, addr(31 downto 0));
                addr(31 downto 0) := std_logic_vector(unsigned(base1) + stride1 * n);
                read_init(n, addr(31 downto 0));
            end loop;
            completed := (others => '0');
            while (completed /= ones64(2*CFG_NUT_CPU_CORES-1 downto 0)) loop
                now_completed := (others => '0');
                for n in 0 to CFG_NUT_CPU_CORES-1 loop
                    if (read_check_ack(n+CFG_NUT_CPU_CORES)) then
                        now_completed(n+CFG_NUT_CPU_CORES) := '1';
                    end if;
                    if (read_check_ack(n)) then
                        now_completed(n) := '1';
                    end if;
                end loop;
                completed := completed or now_completed;
                run_cycle;
                for n in 0 to CFG_NUT_CPU_CORES-1 loop
                    if (now_completed(n+CFG_NUT_CPU_CORES) = '1') then
                        assert read_get_data(n+CFG_NUT_CPU_CORES) = std_logic_vector((unsigned(id) + n));
                    end if;
                    if (now_completed(n) = '1') then
                        assert read_get_data(n) = std_logic_vector((unsigned(id) + n + CFG_NUT_CPU_CORES));
                    end if;
                end loop;
            end loop;
        end procedure;

        procedure run_special (porti : integer; adr, id : TWord) is
            variable data : TWord;
        begin
            -- Check "invalidate"...
            INFO ("  checking 'invalidate'...");
            write(porti, adr, id); -- write 'id' to cache
            mi.wpi(porti).port_direct <= '1';
            write(porti, adr, id+1); -- write 'id+1' to main memory
            mi.wpi(porti).port_direct <= '0';
            read(porti, adr, data);
            assert (data = id) report "Direct write affects cache - cannot check 'invalidate'";
            write_special(porti, adr, '0', '1'); -- invoke 'invalidate'
            read(porti, adr, data);
            assert (data = id+1) report "Special operation 'invalidate' does not work";

            -- Check "writeback"...
            INFO ("  checking 'writeback'...");
            write(porti, adr, id+2); -- write 'id+2' to cache (main memory holds 'id+1')
            mi.rpi(porti).port_direct <= '1';
            read(porti, adr, data);
            assert (data = id+1) report "Cached write affects main memory - cannot check 'writeback'";
            mi.rpi(porti).port_direct <= '0';
            write_special(porti, adr, '1', '0'); -- invoke 'writeback'
            mi.rpi(porti).port_direct <= '1';
            read(porti, adr, data);
            assert (data = id+2) report "Special operation 'writeback' does not work";
            mi.rpi(porti).port_direct <= '0';

            -- Check "flush"...
            INFO ("  checking 'flush'...");
            write(porti, adr, id+3); -- write 'id+3' to cache (main memory holds 'id+2')
            write_special(porti, adr, '1', '1'); -- invoke 'flush'
            mi.rpi(porti).port_direct <= '1';
            read(porti, adr, data);
            assert (data = id+3) report "Special operation 'flush' does not write back";
            mi.rpi(porti).port_direct <= '0';
            mi.wpi(porti).port_direct <= '1';
            write(porti, adr, id+4); -- write 'id+4' to main memory, the next read must cause a miss and this value to be loaded
            mi.wpi(porti).port_direct <= '0';
            read(porti, adr, data);
            assert (data = id+4) report "Special operation 'flush' does not invalidate";
        end procedure;

        procedure run_ll_sc (adr, id : TWord) is
            variable data : TWord;
        begin
            -- Initialize address...
            write(0, adr, id);

            -- Simulate LL...
            mi.wpi(0).port_rlink_wcond <= '1';
            mi.wpi(0).port_adr <= adr;
            read(0, adr, data);
            assert (data = id);
            mi.wpi(0).port_rlink_wcond <= '0';

            -- Perform successful WC...
            mi.wpi(0).port_rlink_wcond <= '1';
            write(0, adr, id+1);
            mi.wpi(0).port_rlink_wcond <= '0';
            assert (mo.wpo(0).port_wcond_ok = '1') report "'wcond_ok' not set after successful write-conditional";
            read(0, adr, data);
            assert (data = id+1) report "memory not changed after successful write-conditional";

            -- Perform concurrent write through other port...
            write(1, adr, id+2);

            -- Perform failing WC...
            mi.wpi(0).port_rlink_wcond <= '1';
            write(0, adr, id+3);
            mi.wpi(0).port_rlink_wcond <= '0';
            assert (mo.wpo(0).port_wcond_ok = '0') report "'wcond_ok' is set after failing write-conditional";
            read(0, adr, data);
            assert (data = id+2) report "memory has changed after failing write-conditional";
        end procedure;

    begin

        for n in 0 to CFG_NUT_CPU_CORES-1 loop
            clear_read_port(n);
            clear_read_port(n+1);
            clear_write_port(n);
        end loop;

        INFO("Simulation starting...");
        reset <= '1';
        run_cycle(5);
        reset <= '0';

        -- Direct write & read...
        for n in 0 to WPORTS-1 loop
            mi.wpi(n).port_direct <= '1';
        end loop;
        for n in 0 to RPORTS-1 loop
            mi.rpi(n).port_direct <= '1';
        end loop;
        INFO ("Direct write & read (with single bytes)...");
        run_partword_read_write(16#0#, X"10", X"00000100");
        INFO ("Direct write & read (parallel)...");
        run_parallel_read_write(X"00000020", X"00000200", 4, X"00000300", 16);

        -- Cached write & read...
        for n in 0 to WPORTS-1 loop
            mi.wpi(n).port_direct <= '0';
        end loop;
        for n in 0 to RPORTS-1 loop
            mi.rpi(n).port_direct <= '0';
        end loop;
        INFO ("Cached write & read (port #0, with partial words)...");
        INFO ("  same cache line...");
        run_partword_read_write (16#0#, X"30", X"00000300");
        INFO ("  multiple cache lines (no conflicts)...");
        run_partword_read_write (16#0#, X"40", X"00000400", 4, 10);
        INFO ("  with conflicts (to force replacements)...");
        run_partword_read_write (16#0#, X"50", X"00000508", CFG_MEMU_CACHE_SIZE, 3);   -- provoke replacements

        INFO ("Cached write & read (parallel)...");
        run_parallel_read_write (X"00000060", X"00000600", 4, X"00000680", 16);
        run_parallel_read_write (X"00000070", X"00000700", 16, X"00000780", 4);

        -- Special operations...
        INFO ("Special operations...");
        run_special(0, X"0000ab00", X"00000013");

        -- LL/SC...
        INFO ("Load-Link and Store-Conditional...");
        run_ll_sc(X"0000ac00", X"00000020");




        sim_halt <= true;
        wait until sim_halt;

        halt <= true;
        report "Simulation finished";
        wait;

    end process;

end tb;
