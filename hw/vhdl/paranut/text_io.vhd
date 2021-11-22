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
--  Helper functions for printing debug ouput. Also includes a table to convert
--  bytes to ASCII values for printing on stdout
--
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library std;
use std.textio.all;
use ieee.std_logic_textio.all;

package text_io is

    function slv2ascii (slv : std_logic_vector) return character;
    procedure put_char (l : inout line; c : character);

    procedure INFO (s : string);
    function to_h_string(slv : std_logic_vector) return string;
    function to_b_string(slv : std_logic_vector) return string;
    function to_ud_string(slv : std_logic_vector) return string;
    function to_sd_string(slv : std_logic_vector) return string;

    constant ascii_table : string (1 to 128) := (
        nul, soh, stx, etx, eot, enq, ack, bel, 
		bs,  ht,  lf,  vt,  ff,  cr,  so,  si, 
		dle, dc1, dc2, dc3, dc4, nak, syn, etb, 
		can, em,  sub, esc, fsp, gsp, rsp, usp, 

		' ', '!', '"', '#', '$', '%', '&', ''', 
		'(', ')', '*', '+', ',', '-', '.', '/', 
		'0', '1', '2', '3', '4', '5', '6', '7', 
		'8', '9', ':', ';', '<', '=', '>', '?', 

		'@', 'A', 'B', 'C', 'D', 'E', 'F', 'G', 
		'H', 'I', 'J', 'K', 'L', 'M', 'N', 'O', 
		'P', 'Q', 'R', 'S', 'T', 'U', 'V', 'W', 
		'X', 'Y', 'Z', '[', '\', ']', '^', '_', 

		'`', 'a', 'b', 'c', 'd', 'e', 'f', 'g', 
		'h', 'i', 'j', 'k', 'l', 'm', 'n', 'o', 
		'p', 'q', 'r', 's', 't', 'u', 'v', 'w', 
		'x', 'y', 'z', '{', '|', '}', '~', del
    );

end package;

package body text_io is

    procedure INFO (s : string) is
        variable l : line;
    begin
        write(l, string'("INFO: "));
        write(l, now);
        write(l, string'(": "));
        write(l, s);
        writeline(OUTPUT, l);
    end procedure;
        
    function to_h_string(slv : std_logic_vector) return string is
        variable l: line;
    begin
        hwrite(l, slv);
        return l.all;
    end;

    function to_b_string(slv : std_logic_vector) return string is
        variable l: line;
    begin
        write(l, slv);
        return l.all;
    end;

    function to_ud_string(slv : std_logic_vector) return string is
        variable l: line;
    begin
        write(l, to_integer(unsigned(slv)));
        return l.all;
    end;

    function to_sd_string(slv : std_logic_vector) return string is
        variable l: line;
    begin
        write(l, to_integer(signed(slv)));
        return l.all;
    end;

    function slv2ascii (slv : std_logic_vector) return character is
        variable c : character;
    begin
        return ascii_table(1+to_integer(unsigned(slv)));
    end;

    procedure put_char (l : inout line; c : character) is
    begin
        if (c = lf) then
            writeline(OUTPUT, l);
        elsif (c /= cr) then
            write(l, c);
        end if;
    end;

end text_io;

