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
--  Global type definitions for the ParaNut project.
--
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;

package types is

    subtype THalfWord is std_logic_vector(15 downto 0);
    type THalfWord_Vec is array (natural range <>) of THalfWord;

    subtype TWord is std_logic_vector(31 downto 0);
    type TWord_Vec is array (natural range <>) of TWord;

    subtype TLSUWidth is std_logic_vector(1 downto 0); -- "00" = word, "01" = byte, "10" = half word
    type TLSUWidth_Vec is array (natural range <>) of TLSUWidth;

    subtype TWBufValid is std_logic_vector(3 downto 0);
    type TWBufValid_Vec is array (natural range <>) of TWBufValid;
    subtype TByteSel is std_logic_vector(3 downto 0);
    type TByteSel_Vec is array (natural range <>) of TByteSel;

    type integer_vector is array (natural range <>) of integer;

    type mem_type is array (natural range <>) of std_logic_vector(31 downto 0);

    constant zero64 : std_logic_vector(63 downto 0) := (others => '0');
    constant ones64 : std_logic_vector(63 downto 0) := (others => '1');

end package;
