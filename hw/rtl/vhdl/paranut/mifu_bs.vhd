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
--  Instruction fetch buffer module (experimental version with bit slice design)
--  (not up to date!)
--
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library paranut;
use paranut.ifu.all;
use paranut.types.all;
use paranut.memu_lib.all;
use paranut.paranut_lib.all;
use paranut.orbis32.all;

entity mifu_bs is
    generic (
                IFU_BUF_SIZE : integer range 4 to 16 := 4
            );
    port (
             clk            : in std_logic;
             reset          : in std_logic;
             -- to EXU...
             ifui           : in ifu_in_type;
             ifuo           : out ifu_out_type;
             -- to MEMU (read port)...
             rpi            : out readport_in_type;
             rpo            : in readport_out_type;
             -- from CePU...
             icache_enable   : in std_logic
         );
end mifu_bs;

architecture rtl of mifu_bs is

    -- IFU state types
    type EIfuState is (s_ifu_idle, s_ifu_reading);
    --subtype EIfuState is std_logic_vector(0 to 0);
    --constant s_ifu_idle     : EIfuState := "0";
    --constant s_ifu_reading  : EIfuState := "1";

    constant START_ADDRESS: integer := 16#100#;

    type buf_bit_slice is record
        insn_valid  : std_logic;
        adr_valid   : std_logic;
        insn_buf    : TWord;
        adr_buf     : TWord;
    end record;
    type buf_bit_slice_vector is array (0 to IFU_BUF_SIZE-1) of buf_bit_slice;

    type reg_type is record
        state_reg   : EIfuState;
        last_rp_ack : std_logic;
        rp_rd       : std_logic;
        buf         : buf_bit_slice_vector;
    end record;

    signal r, rin: reg_type;

    function pfetch_ok (rbuf, vbuf: buf_bit_slice_vector) return boolean is
        variable fetch_ok : boolean := false;
    begin
        for n in 0 to IFU_BUF_SIZE-1 loop
            fetch_ok := fetch_ok or (vbuf(n).adr_valid = '1' and rbuf(n).insn_valid = '0');
        end loop;
        return (fetch_ok);
    end function;

    function fetch_address (rbuf: buf_bit_slice_vector) return TWord is
        variable adr : TWord;
    begin
        for n in IFU_BUF_SIZE-1 downto 0 loop
            if (rbuf(n).insn_valid = '0') then
                adr := rbuf(n).adr_buf;
            end if;
        end loop;
        return (adr);
    end function;

    function is_jump (rbuf, vbuf: buf_bit_slice_vector) return boolean is
        variable opcode : std_logic_vector(5 downto 0);
        variable ret : boolean := false;
    begin
        -- Get insn_top_var-2'th opcode (we're not interested in the 0th slot
        -- because new instructions are put into slot >=1).
        for n in IFU_BUF_SIZE-1 downto 3 loop
            if ((vbuf(n).insn_valid xor vbuf(n-1).insn_valid) = '1') then
                opcode := rbuf(n-2).insn_buf(31 downto 26);
                case opcode is
                    when J | JAL | BNF | BF | JR | JALR | OTHER | RFE =>
                        ret := true;
                    when others =>
                        null;
                end case;
            end if;
        end loop;
        return ret;
    end function;

begin

    comb : process(reset, r, rpo, ifui)

        variable v : reg_type;
        variable vrpi : readport_in_type;
        variable add : integer range 0 to 1;

        --variable adr_new: TWord;
        --variable adr_new_mask: std_logic_vector(0 to IFU_BUF_SIZE-1);

    begin

        v := r;

        --adr_new := (others => '0');
        --adr_new_mask := (others => '0');

        -- Shift buffer if 'next' is asserted...
        if (ifui.nexti = '1') then
            v.buf(0 to IFU_BUF_SIZE-2) := r.buf(1 to IFU_BUF_SIZE-1);
            v.buf(IFU_BUF_SIZE-1).insn_valid := '0';
            v.buf(IFU_BUF_SIZE-1).adr_valid := '0';
        end if;

        -- Generate new address...
        if (ifui.nexti = '0') then add := 1; else add := 0; end if;
        for n in IFU_BUF_SIZE-1 downto 1 loop
            if ((v.buf(n).adr_valid xor v.buf(n-1).adr_valid) = '1') then
                v.buf(n).adr_buf := r.buf(n-add).adr_buf + 4;
                v.buf(n).adr_valid := '1';
                --adr_new := r.buf(n-add).adr_buf + 4;
                --adr_new_mask(n) := '1';
            end if;
        end loop;
        --for n in IFU_BUF_SIZE-1 downto 0 loop
        --    if (adr_new_mask(n) = '1') then
        --        v.buf(n).adr_buf := adr_new;
        --        v.buf(n).adr_valid := '1';
        --    end if;
        --end loop;

        -- Handle jump...
        if (ifui.jump = '1') then
            -- pragma translate_off
            assert (ifui.jump_adr(1 downto 0) = "00") report "jump_adr % 4 != 0";
            -- pragma translate_on
            if (v.buf(2).insn_valid = '1') then
                for n in 2 to IFU_BUF_SIZE-1 loop
                    v.buf(n).insn_valid := '0';
                end loop;
            end if;
            v.buf(2).adr_buf := ifui.jump_adr;
            for n in 0 to 2 loop
                v.buf(n).adr_valid := '1';
            end loop;
            for n in 3 to IFU_BUF_SIZE-1 loop
                v.buf(n).adr_valid := '0';
            end loop;
        end if;

        -- Store new memory data if available...
        v.last_rp_ack := rpo.port_ack;
        if (r.last_rp_ack = '1') then
            -- pragma translate_off
            assert (v.buf(IFU_BUF_SIZE-1).insn_valid /= '1') report "insn_top >= IFU_BUF_SIZE";
            -- pragma translate_on
            for n in IFU_BUF_SIZE-1 downto 1 loop
                if ((v.buf(n).insn_valid xor v.buf(n-1).insn_valid) = '1') then
                    v.buf(n).insn_buf := rpo.port_data;
                    v.buf(n).insn_valid := '1';
                end if;
            end loop;
        end if;

        case r.state_reg is
            when s_ifu_idle =>
                if (pfetch_ok(r.buf, v.buf) and v.buf(IFU_BUF_SIZE-1).insn_valid = '0'
                and not (v.buf(1).insn_valid = '1' and is_jump(r.buf, v.buf))
                -- TODO: why nexti and last_rp_ack?
                and ifui.nexti = '0' and ifui.jump = '0' and r.last_rp_ack = '0') then
                    v.rp_rd := '1';
                    v.state_reg := s_ifu_reading;
                end if;
            when s_ifu_reading =>
                if (rpo.port_ack = '1') then
                    v.rp_rd := '0';
                    v.state_reg := s_ifu_idle;
                end if;
        end case;

        if (reset = '1') then
            v.buf(0).insn_valid := '1';
            v.buf(0).adr_valid := '1';
            for n in 1 to IFU_BUF_SIZE-1 loop
                v.buf(n).insn_valid := '0';
                v.buf(n).adr_valid := '0';
            end loop;
            v.buf(0).adr_buf := conv_std_logic_vector(START_ADDRESS - 4, TWord'length);
            v.state_reg := s_ifu_idle;
            v.rp_rd := '0';
        end if;

        -- outputs

        ifuo.ir <= r.buf(1).insn_buf;
        ifuo.ppc <= r.buf(0).adr_buf;
        ifuo.pc <= r.buf(1).adr_buf;
        ifuo.npc <= r.buf(2).adr_buf;
        ifuo.ir_valid <= r.buf(1).insn_valid;
        ifuo.npc_valid <= r.buf(2).adr_valid;

        vrpi.port_rd := r.rp_rd;
        vrpi.port_adr := fetch_address(r.buf);
        vrpi.port_bsel := X"f";
        -- 'direct' lines for read ports...
        if ((icache_enable = '0') or (not adr_is_cached(vrpi.port_adr))) then
            vrpi.port_direct := '1';
        else
            vrpi.port_direct := '0';
        end if;

        rin <= v;
        rpi <= vrpi;

    end process;

    regs : process(clk)
    begin
        if (clk'event and clk = '1') then
            r <= rin;
        end if;
    end process;

end rtl;
