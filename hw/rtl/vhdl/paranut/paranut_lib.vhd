--------------------------------------------------------------------------------
-- This file is part of the ParaNut project.
-- 
-- Copyright (C) 2013  Michael Seider, <michael.seider@hs-augsburg.de>
-- 		 Hochschule Augsburg, University of Applied Sciences
--
-- Parts of this file are taken from GRLIB VHDL IP LIBRARY
--  Copyright (C) 2003 - 2008, Gaisler Research
--  Copyright (C) 2008 - 2013, Aeroflex Gaisler
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
--  Helper functions for the ParaNut project.
--
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.math_real.all;

package paranut_lib is

    function MAX (LEFT, RIGHT: INTEGER) return INTEGER;
    function MIN (LEFT, RIGHT: INTEGER) return INTEGER;

    function log2x(val : positive) return natural;

    function trunc (i, w : integer) return integer;

    function notx(d : std_logic_vector) return boolean;
    function notx(d : std_ulogic) return boolean;

    function "+" (i : integer; d : std_logic_vector) return std_logic_vector;
    function "+" (d : std_logic_vector; i : integer) return std_logic_vector;

    function conv_integer(v : std_logic_vector) return integer;
    function conv_integer(s : std_logic) return integer;
    function conv_integer(u : unsigned) return integer;
    function conv_std_logic_vector(i : integer; w : integer) return std_logic_vector;
    function conv_std_logic_vector(s : std_logic) return std_logic_vector;
    function conv_std_logic(b : boolean) return std_ulogic;

	function reverse_vector (in_vec: in std_logic_vector) return std_logic_vector;

end package;

package body paranut_lib is

    function MAX (LEFT, RIGHT: INTEGER) return INTEGER is
    begin
        if LEFT > RIGHT then return LEFT;
        else return RIGHT;
        end if;
    end MAX;

    function MIN (LEFT, RIGHT: INTEGER) return INTEGER is
    begin
        if LEFT < RIGHT then return LEFT;
        else return RIGHT;
        end if;
    end MIN;

    function log2x(val : positive) return natural is
    begin
        --report "log2x(" & integer'image(val) & ") is " & integer'image(integer(ceil(log2(real(val)))));
        return integer(ceil(log2(real(val))));
    end function;

    function trunc (i, w : integer) return integer is
    begin
        --return to_integer(to_unsigned(i, log2x(w+1))(log2x(w)-1 downto 0));
        return to_integer(to_unsigned(i, w));
    end;

    function notx(d : std_logic_vector) return boolean is
        variable res : boolean;
    begin
        res := true;
        -- pragma translate_off
        res := not is_x(d);
        -- pragma translate_on
        return (res);
    end;

    function notx(d : std_ulogic) return boolean is
        variable res : boolean;
    begin
        res := true;
        -- pragma translate_off
        res := not is_x(d);
        -- pragma translate_on
        return (res);
    end;

    function "+" (i : integer; d : std_logic_vector) return std_logic_vector is
        variable x : std_logic_vector(d'length-1 downto 0);
    begin
        -- pragma translate_off
        if notx(d) then
            -- pragma translate_on
            return(std_logic_vector(unsigned(d) + i));
        -- pragma translate_off
        else x := (others =>'X'); return(x);
        end if;
    -- pragma translate_on
    end;

    function "+" (d : std_logic_vector; i : integer) return std_logic_vector is
        variable x : std_logic_vector(d'length-1 downto 0);
    begin
        -- pragma translate_off
        if notx(d) then
            -- pragma translate_on
            return(std_logic_vector(unsigned(d) + i));
        -- pragma translate_off
        else x := (others =>'X'); return(x);
        end if;
    -- pragma translate_on
    end;

    function conv_integer(v : std_logic_vector) return integer is
    begin
        if notx(v) then return(to_integer(unsigned(v)));
        else return(0);
        end if;
    end;

    function conv_integer(s : std_logic) return integer is
    begin
        if (notx(s)) then
            if (s = '0') then return 0;
            else return 1;
            end if;
        else return 0;
        end if;
    end;

    function conv_integer(u : unsigned) return integer is
    begin
        return(to_integer(u));
    end;

    function conv_std_logic_vector(i : integer; w : integer) return std_logic_vector is
        variable tmp : std_logic_vector(w-1 downto 0);
    begin
        tmp := std_logic_vector(to_unsigned(i, w));
        return(tmp);
    end;

    function conv_std_logic_vector(s : std_logic) return std_logic_vector is
    begin
        if (notx(s)) then
            if (s = '0') then return "0";
            else return "1";
            end if;
        else return "0";
        end if;
    end;

    function conv_std_logic(b : boolean) return std_ulogic is
    begin
        if b then return('1'); else return('0'); end if;
    end;

	function reverse_vector (in_vec: in std_logic_vector)
	return std_logic_vector is
		variable out_vec: std_logic_vector(in_vec'reverse_range);
	begin
		for i in in_vec'range loop
			out_vec(i) := in_vec(i);
		end loop;
		return out_vec;
	end reverse_vector;

end package body;
