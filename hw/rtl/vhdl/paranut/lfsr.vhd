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
--  Functions for instantiating and running an LFSR counter.
--
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;

library paranut;
use paranut.types.all;

package lfsr is

    function get_prime_poly (degree : integer range 2 to 32; sel : integer range 0 to 16) return std_logic_vector;

    function get_next_lfsr_state (state : std_logic_vector; poly : std_logic_vector) return std_logic_vector;

    type prime_poly_list_xors_type is array (0 to 30) of TWord_Vec(0 to 15);

    constant polys_available : integer_vector(0 to 30) := ( 
      0 => 1, 1 => 2, 2 => 2, 3 => 6, 4 => 6, 5 => 16, 6 => 16,            --  2..8
      7 => 16, 8 => 16, 9 => 16, 10 => 16, 11 => 16, 12 => 16, 13 => 16, 14 => 16,   --  9..16
      15 => 16, 16 => 16, 17 => 16, 18 => 16, 19 => 16, 20 => 16, 21 => 16, 22 => 14,   -- 17..24
      23 => 16, 24 => 16, 25 => 16, 26 => 16, 27 => 16, 28 => 16, 29 => 16, 30 => 16   -- 25..32
    );

    constant prime_poly_list_few_xors : prime_poly_list_xors_type := (
        0 => (0 => X"00000003", others => X"ffffffff"),
        -- degree 3...
        1 => (0 => X"00000003", 1 => X"00000005", others => X"ffffffff"),

        -- degree 4...
        2 => (0 => X"00000003", 1 => X"00000009", others => X"ffffffff"),

        -- degree 5...
        3 => (0 => X"00000005", 1 => X"00000009", 2 => X"0000000f", 3 => X"00000017",
              4 => X"0000001b", 5 => X"0000001d", others => X"ffffffff"),

        -- degree 6...
        4 => (0 => X"00000003", 1 => X"0000001b", 2 => X"00000021", 3 => X"00000027",
              4 => X"0000002d", 5 => X"00000033", others => X"ffffffff"),

        -- degree 7...
        5 => (0 => X"00000003", 1 => X"00000009", 2 => X"0000000f", 3 => X"00000011",
              4 => X"0000001d", 5 => X"0000002b", 6 => X"00000039", 7 => X"0000003f",
              8 => X"00000041", 9 => X"0000004b", 10 => X"0000004d", 11 => X"00000053",
              12 => X"00000055", 13 => X"00000065", 14 => X"00000071", 15 => X"00000077"),

        -- degree 8...
        6 => (0 => X"0000001d", 1 => X"0000002b", 2 => X"0000002d", 3 => X"0000005f",
              4 => X"0000004d", 5 => X"00000063", 6 => X"00000065", 7 => X"00000069", 
              8 => X"00000071", 9 => X"0000008d", 10 => X"00000087", 11 => X"000000a9",
              12 => X"000000c3", 13 => X"000000cf", 14 => X"000000e7", 15 => X"000000f5"),

        -- degree 9...
        7 => (0 => X"00000011", 1 => X"0000002d", 2 => X"00000059", 3 => X"00000069",
              4 => X"00000077", 5 => X"00000095", 6 => X"000000d1", 7 => X"000000db", 
              8 => X"00000113", 9 => X"00000131", 10 => X"00000161", 11 => X"00000185",
              12 => X"0000018f", 13 => X"000001b9", 14 => X"000001e3", 15 => X"000001e9"),

        -- degree 10...
        8 => (0 => X"00000009", 1 => X"0000001b", 2 => X"0000006f", 3 => X"000000d7",
              4 => X"00000081", 5 => X"0000010d", 6 => X"00000119", 7 => X"00000131",
              8 => X"0000018f", 9 => X"000001a1", 10 => X"000001c7", 11 => X"00000213",
              12 => X"00000215", 13 => X"00000223", 14 => X"000002c1", 15 => X"00000393"),

        -- degree 11...
        9 => (0 => X"00000005", 1 => X"0000002b", 2 => X"00000063", 3 => X"0000008d",
              4 => X"00000113", 5 => X"00000125", 6 => X"00000145", 7 => X"00000149", 
              8 => X"00000201", 9 => X"00000207", 10 => X"00000213", 11 => X"00000215",
              12 => X"00000291", 13 => X"00000341", 14 => X"0000040d", 15 => X"00000419"),

        -- degree 12...
        10 => (0 => X"00000053", 1 => X"00000069", 2 => X"00000099", 3 => X"000000d1",
               4 => X"00000123", 5 => X"00000161", 6 => X"00000185", 7 => X"00000225",
               8 => X"000002c1", 9 => X"00000321", 10 => X"00000407", 11 => X"000006a0",
               12 => X"00000891", 13 => X"00000941", 14 => X"00000c05", 15 => X"00000c11"),

        -- degree 13...
        11 => (0 => X"0000001b", 1 => X"0000008b", 2 => X"00000143", 3 => X"00000243",
               4 => X"0000025b", 5 => X"00000289", 6 => X"00000425", 7 => X"00000521", 
               8 => X"00000941", 9 => X"00000c21", 10 => X"00001007", 11 => X"00001019",
               12 => X"000010a1", 13 => X"00001209", 14 => X"00001301", 15 => X"00001601"),

        -- degree 14...
        12 => (0 => X"0000002b", 1 => X"00000053", 2 => X"00000143", 3 => X"00000309",
               4 => X"00000443", 5 => X"00000813", 6 => X"00000843", 7 => X"00000c81",
               8 => X"000010a1", 9 => X"00001803", 10 => X"0000200d", 11 => X"00002015",
               12 => X"00002141", 13 => X"00002501", 14 => X"00002601", 15 => X"00002a01"),

        -- degree 15...
        13 => (0 => X"00000003", 1 => X"00000011", 2 => X"00000081", 3 => X"00000101",
               4 => X"00000611", 5 => X"00000801", 6 => X"00000909", 7 => X"00001091",
               8 => X"0000200b", 9 => X"00002501", 10 => X"00003401", 11 => X"00004001",
               12 => X"00004007", 13 => X"00004301", 14 => X"00005005", 15 => X"00006003"),

        -- degree 16...
        14 => (0 => X"0000002d", 1 => X"0000026d", 2 => X"00000285", 3 => X"00000483",
               4 => X"00000489", 5 => X"00000861", 6 => X"00000c21", 7 => X"0000100b",
               8 => X"00001241", 9 => X"00002241", 10 => X"00004281", 11 => X"00006801",
               12 => X"00008241", 13 => X"0000900f", 14 => X"0000a011", 15 => X"0000e013"),

        -- degree 17...
        15 => (0 => X"00000009", 1 => X"00000021", 2 => X"00000041", 3 => X"00000801",
               4 => X"00001001", 5 => X"00002023", 6 => X"00004001", 7 => X"00004091",
               8 => X"00006401", 9 => X"0000a801", 10 => X"0000d001", 11 => X"0001000d",
               12 => X"00010031", 13 => X"00010601", 14 => X"00013001", 15 => X"0001c001"),

        -- degree 18...
        16 => (0 => X"00000027", 1 => X"00000081", 2 => X"00000489", 3 => X"000004a1",
               4 => X"00000801", 5 => X"00000f0f", 6 => X"00002901", 7 => X"00002bc9",
               8 => X"00004023", 9 => X"00008025", 10 => X"00008901", 11 => X"00009bc1",
               12 => X"00012009", 13 => X"00022011", 14 => X"00032001", 15 => X"00038781"),

        -- degree 19...
        17 => (0 => X"00000027", 1 => X"00000063", 2 => X"000003c9", 3 => X"0000168a",
               4 => X"00002139", 5 => X"00008441", 6 => X"0000a201", 7 => X"00010013", 
               8 => X"00010025", 9 => X"00010049", 10 => X"00010121", 11 => X"00011201",
               12 => X"00021401", 13 => X"0002f043", 14 => X"00040181", 15 => X"00046001"),

        -- degree 20...
        18 => (0 => X"00000009", 1 => X"00000229", 2 => X"00001809", 3 => X"00003021",
               4 => X"00010023", 5 => X"00010089", 6 => X"00012201", 7 => X"00018801",
               8 => X"00020001", 9 => X"00020049", 10 => X"00022011", 11 => X"00028801",
               12 => X"00080019", 13 => X"000800c1", 14 => X"00080301", 15 => X"00088011"),

        -- degree 21...
        19 => (0 => X"00000005", 1 => X"00004221", 2 => X"00008105", 3 => X"00011081",
               4 => X"00018401", 5 => X"00020045", 6 => X"00024401", 7 => X"00040013",
               8 => X"00040481", 9 => X"00040901", 10 => X"00080001", 11 => X"000800a1",
               12 => X"00082041", 13 => X"00088011", 14 => X"00100061", 15 => X"00120009"),

        -- degree 22...
        20 => (0 => X"00000003", 1 => X"00002409", 2 => X"00020109", 3 => X"00022101",
               4 => X"00060003", 5 => X"00081201", 6 => X"00084021", 7 => X"00089001",
               8 => X"000a4001", 9 => X"0010000b", 10 => X"00100a01", 11 => X"00200001",
               12 => X"00200031", 13 => X"00200601", 14 => X"00218001", 15 => X"00280005"),

        -- degree 23...
        21 => (0 => X"00000021", 1 => X"00000201", 2 => X"00004001", 3 => X"00040001",
               4 => X"00050005", 5 => X"002000a1", 6 => X"00200501", 7 => X"00400007", 
               8 => X"00400031", 9 => X"00400061", 10 => X"00400181", 11 => X"00400c01",
               12 => X"0040c001", 13 => X"00418001", 14 => X"004c0001", 15 => X"00600003"),

        -- degree 24...
        22 => (0 => X"0000001b", 1 => X"00000087", 2 => X"0000a011", 3 => X"00010281",
               4 => X"00028101", 5 => X"00100a01", 6 => X"00180003", 7 => X"00800031", 
               8 => X"00b00001", 9 => X"00c20001", 10 => X"001ba555", 11 => X"002fe633",
               12 => X"00554bb1", 13 => X"0098cfe9", others =>
                X"ffffffff"),

        -- degree 25...
        23 => (0 => X"00000009", 1 => X"00000081", 2 => X"00040001", 3 => X"00050005",
               4 => X"00400001", 5 => X"00500005", 6 => X"00800029", 7 => X"00800281", 
               8 => X"00800a01", 9 => X"00801401", 10 => X"008a0001", 11 => X"00d00001",
               12 => X"01000601", 13 => X"01000c01", 14 => X"01003001", 15 => X"01c00001"),

        -- degree 26...
        24 => (0 => X"00000047", 1 => X"00000183", 2 => X"00220011", 3 => X"00221001",
               4 => X"00280005", 5 => X"00284001", 6 => X"00400221", 7 => X"00800013", 
               8 => X"00848001", 9 => X"00890001", 10 => X"010000a1", 11 => X"010a0001",
               12 => X"02001801", 13 => X"020c0001", 14 => X"02400009", 15 => X"03100001"),

        -- degree 27...
        25 => (0 => X"00000027", 1 => X"00000183", 2 => X"00800045", 3 => X"00800221",
               4 => X"00844001", 5 => X"01000025", 6 => X"01002401", 7 => X"01480001", 
               8 => X"02001401", 9 => X"02014001", 10 => X"02200011", 11 => X"02400009",
               12 => X"03000003", 13 => X"0400000d", 14 => X"04180001", 15 => X"06400001"),

        -- degree 28...
        26 => (0 => X"00000009", 1 => X"00000201", 2 => X"00002001", 3 => X"00008001",
               4 => X"00080001", 5 => X"00402081", 6 => X"00408201", 7 => X"00800085", 
               8 => X"00800841", 9 => X"00808401", 10 => X"00810801", 11 => X"01000023",
               12 => X"01088001", 13 => X"02000001", 14 => X"08000031", 15 => X"08000181"),

        -- degree 29...
        27 => (0 => X"00000005", 1 => X"00280005", 2 => X"01000085", 3 => X"02000045",
               4 => X"04000025", 5 => X"04001201", 6 => X"04012001", 7 => X"04090001", 
               8 => X"08000001", 9 => X"08000501", 10 => X"08000a01", 11 => X"08140001",
               12 => X"08400021", 13 => X"08800011", 14 => X"09000009", 15 => X"10c00001"),

        -- degree 30...
        28 => (0 => X"00000053", 1 => X"00018003", 2 => X"00100803", 3 => X"00902001",
               4 => X"00800205", 5 => X"00800103", 6 => X"01000821", 7 => X"04008801", 
               8 => X"04022001", 9 => X"08000121", 10 => X"08900001", 11 => X"10200081",
               12 => X"2000c001", 13 => X"20080401", 14 => X"20400081", 15 => X"25000001"),

        -- degree 31...
        29 => (0 => X"00000009", 1 => X"00000041", 2 => X"00000081", 3 => X"00002001",
               4 => X"02000001", 5 => X"10000001", 6 => X"20000501", 7 => X"20002801", 
               8 => X"2000a001", 9 => X"20280001", 10 => X"22800001", 11 => X"2a000001",
               12 => X"400000c1", 13 => X"40006001", 14 => X"40600001", 15 => X"70000001"),

        -- degree 32...
        30 => (0 => X"000000af", 1 => X"00012009", 2 => X"000a0011", 3 => X"00840021",
               4 => X"02002041", 5 => X"02408001", 6 => X"04000821", 7 => X"04080081", 
               8 => X"08004201", 9 => X"08200041", 10 => X"0a100001", 11 => X"1000a001",
               12 => X"18000003", 13 => X"20090001", 14 => X"80000031", 15 => X"ea000001")
    );

    constant prime_poly_list_many_xors : prime_poly_list_xors_type := (
        -- degree 2...
        0 => (0 => X"00000003", others => X"ffffffff"),

        -- degree 3...
        1 => (0 => X"00000003", 1 => X"00000005", others => X"ffffffff"), 

        -- degree 4...
        2 => (0 => X"00000003", 1 => X"00000009", others => X"ffffffff"), 

        -- degree 5...
        3 => (0 => X"0000000f", 1 => X"00000017", 2 => X"0000001b", 3 =>
            X"0000001d", 4 => X"00000005", 5 => X"00000009", others => X"ffffffff"), 

        -- degree 6...
        4 => (0 => X"00000027", 1 => X"0000001b", 2 => X"00000033", 3 => X"0000002d",
              4 => X"00000003", 5 => X"00000021", others => X"ffffffff"), 

        -- degree 7...
        5 => (0 => X"0000003f", 1 => X"0000006f", 2 => X"00000077", 3 => X"0000007d",
               4 => X"0000000f", 5 => X"00000027", 6 => X"0000002b", 7 => X"0000004b", 
               8 => X"00000053", 9 => X"0000001d", 10 => X"00000055", 11 => X"00000065",
               12 => X"00000039", 13 => X"00000071", 14 => X"00000003", 15 => X"00000009"), 

        -- degree 8...
        6 => (0 => X"0000005f", 1 => X"000000cf", 2 => X"000000e7", 3 => X"000000f5",
               4 => X"00000087", 5 => X"0000002b", 6 => X"00000063", 7 => X"000000c3", 
               8 => X"0000001d", 9 => X"0000002d", 10 => X"0000004d", 11 => X"0000008d",
               12 => X"00000065", 13 => X"00000069", 14 => X"000000a9", 15 => X"00000071"), 

        -- degree 9...
        7 => (0 => X"0000017f", 1 => X"000001fb", 2 => X"0000005f", 3 => X"0000011f",
               4 => X"0000006f", 5 => X"000000af", 6 => X"000000cf", 7 => X"0000014f", 
               8 => X"0000018f", 9 => X"00000077", 10 => X"000000b7", 11 => X"000001c7",
               12 => X"0000013b", 13 => X"000000db", 14 => X"0000015b", 15 => X"0000016b"), 

        -- degree 10...
        8 => (0 => X"000000ff", 1 => X"0000027f", 2 => X"000002df", 3 => X"000001f7",
               4 => X"000001fb", 5 => X"000003db", 6 => X"000003f3", 7 => X"000002fd", 
               8 => X"0000037d", 9 => X"000003f9", 10 => X"0000006f", 11 => X"0000024f",
               12 => X"0000018f", 13 => X"00000237", 14 => X"000000d7", 15 => X"00000157"), 

        -- degree 11...
        9 => (0 => X"0000017f", 1 => X"0000027f", 2 => X"000004bf", 3 => X"0000033f",
               4 => X"000002df", 5 => X"0000035f", 6 => X"0000065f", 7 => X"0000059f", 
               8 => X"0000069f", 9 => X"000002ef", 10 => X"0000036f", 11 => X"0000056f",
               12 => X"000003af", 13 => X"000006cf", 14 => X"000001f7", 15 => X"000004f7"), 

        -- degree 12...
        10 => (0 => X"000007bf", 1 => X"00000bbf", 2 => X"00000fbb", 3 => X"00000fbd",
               4 => X"0000027f", 5 => X"0000033f", 6 => X"000001df", 7 => X"00000c9f", 
               8 => X"00000b1f", 9 => X"000008ef", 10 => X"00000e2f", 11 => X"000009cf",
               12 => X"00000ccf", 13 => X"00000e8f", 14 => X"000005d7", 15 => X"00000b57"), 

        -- degree 13...
        11 => (0 => X"00000fff", 1 => X"00001bff", 2 => X"00001ff7", 3 => X"00001ffd",
               4 => X"000011ff", 5 => X"00000b7f", 6 => X"0000157f", 7 => X"00001abf", 
               8 => X"00000f3f", 9 => X"00001b3f", 10 => X"00001e3f", 11 => X"000007df",
               12 => X"00000edf", 13 => X"00001adf", 14 => X"00000f5f", 15 => X"00001f1f"), 

        -- degree 14...
        12 => (0 => X"000033ff", 1 => X"00003cff", 2 => X"00003e7f", 3 => X"00003f3f",
               4 => X"00001fdf", 5 => X"00002fdf", 6 => X"00003f9f", 7 => X"00003fe7", 
               8 => X"00003dfb", 9 => X"00003dfd", 10 => X"000005ff", 11 => X"000011ff",
               12 => X"000018ff", 13 => X"000028ff", 14 => X"0000077f", 15 => X"0000157f"), 

        -- degree 15...
        13 => (0 => X"00003fff", 1 => X"000077ff", 2 => X"00007eff", 3 => X"00007f7f",
               4 => X"00007fef", 5 => X"00007ffd", 6 => X"000017ff", 7 => X"00001bff", 
               8 => X"000063ff", 9 => X"00004dff", 10 => X"00004eff", 11 => X"000072ff",
               12 => X"00006cff", 13 => X"0000577f", 14 => X"00003d7f", 15 => X"00003e7f"), 

       -- degree 16...
       14 => (0 => X"00006fff", 1 => X"0000afff", 2 => X"0000fb7f", 3 => X"0000fdbf",
              4 => X"0000bfdf", 5 => X"0000efdf", 6 => X"0000dfef", 7 => X"0000f7ef", 
              8 => X"0000eff7", 9 => X"0000f7fb", 10 => X"0000ffeb", 11 => X"0000ffed",
              12 => X"00004bff", 13 => X"00001dff", 14 => X"000039ff", 15 => X"000059ff"), 

        -- degree 17...
        15 => (0 => X"0001dfff", 1 => X"0001efff", 2 => X"0001ffdf", 3 => X"0001ffef",
               4 => X"00014fff", 5 => X"0000d7ff", 6 => X"0001cbff", 7 => X"00015dff", 
               8 => X"0001adff", 9 => X"0001cdff", 10 => X"000175ff", 11 => X"0001e5ff",
               12 => X"0001d9ff", 13 => X"0000beff", 14 => X"00013eff", 15 => X"0001d6ff"), 

        -- degree 18...
        16 => (0 => X"00033fff", 1 => X"0001f7ff", 2 => X"0003dbff", 3 => X"00037dff",
               4 => X"0003edff", 5 => X"0003beff", 6 => X"00037f7f", 7 => X"0003df7f", 
               8 => X"0002ffbf", 9 => X"0003dfbf", 10 => X"0003fdbf", 11 => X"00037fdf",
               12 => X"0003efdf", 13 => X"0003f7df", 14 => X"0003fedf", 15 => X"0003fbef"), 

        -- degree 19...
        17 => (0 => X"0004bfff", 1 => X"0001dfff", 2 => X"00059fff", 3 => X"0005cfff",
               4 => X"000677ff", 5 => X"0003b7ff", 6 => X"0006b7ff", 7 => X"000797ff", 
               8 => X"0005e7ff", 9 => X"00057bff", 10 => X"0005bbff", 11 => X"00075bff",
               12 => X"0003ebff", 13 => X"0007cbff", 14 => X"0003f3ff", 15 => X"0005f3ff"), 

        -- degree 20...
        18 => (0 => X"0003ffff", 1 => X"000d7fff", 2 => X"000e7fff", 3 => X"000ddfff",
               4 => X"000f9fff", 5 => X"000ef7ff", 6 => X"000fd7ff", 7 => X"000fe7ff", 
               8 => X"000dfdff", 9 => X"000fddff", 10 => X"000fdeff", 11 => X"000ffcff",
               12 => X"000fef7f", 13 => X"000ff77f", 14 => X"000ffd7f", 15 => X"000fff3f"), 

        -- degree 21...
        19 => (0 => X"00137fff", 1 => X"00197fff", 2 => X"001c7fff", 3 => X"0013bfff",
               4 => X"0015bfff", 5 => X"000ddfff", 6 => X"001cdfff", 7 => X"000f5fff", 
               8 => X"00179fff", 9 => X"001f1fff", 10 => X"0015efff", 11 => X"001cefff",
               12 => X"00176fff", 13 => X"001d6fff", 14 => X"001bcfff", 15 => X"000ef7ff"), 

        -- degree 22...
        20 => (0 => X"001bffff", 1 => X"001dffff", 2 => X"002effff", 3 => X"002f7fff",
               4 => X"002fefff", 5 => X"003df7ff", 6 => X"002ffdff", 7 => X"003f7dff", 
               8 => X"003f7eff", 9 => X"003fbf7f", 10 => X"003fdf7f", 11 => X"001fffdf",
               12 => X"002fffdf", 13 => X"003ff7df", 14 => X"0037ffef", 15 => X"001ffff7"), 

        -- degree 23...
        21 => (0 => X"003fffff", 1 => X"007dffff", 2 => X"007f7fff", 3 => X"007fdfff",
               4 => X"007ffbff", 5 => X"007ffeff", 6 => X"007fffbf", 7 => X"007ffffd", 
               8 => X"0036ffff", 9 => X"003affff", 10 => X"005affff", 11 => X"0074ffff",
               12 => X"00767fff", 13 => X"002fbfff", 14 => X"0037bfff", 15 => X"0067bfff"), 

        -- degree 24...
        22 => (0 => X"00ebffff", 1 => X"007dffff", 2 => X"007f7fff", 3 => X"00bf7fff",
               4 => X"007fbfff", 5 => X"00fefdff", 6 => X"00ff7eff", 7 => X"00ffffaf", 
               8 => X"00fffdfb", 9 => X"00fffbfd", 10 => X"00fffdfd", 11 => X"00ffff7d",
               12 => X"0063ffff", 13 => X"0055ffff", 14 => X"0095ffff", 15 => X"00bc7fff"), 

        -- degree 25...
        23 => (0 => X"01dfffff", 1 => X"01feffff", 2 => X"01fffdff", 3 => X"01ffffef",
               4 => X"003fffff", 5 => X"00afffff", 6 => X"012fffff", 7 => X"0167ffff", 
               8 => X"019bffff", 9 => X"01b3ffff", 10 => X"01e3ffff", 11 => X"013dffff",
               12 => X"01d5ffff", 13 => X"01e5ffff", 14 => X"01e9ffff", 15 => X"00beffff"), 

       -- degree 26...
       24 => (0 => X"02efffff", 1 => X"01fbffff", 2 => X"037bffff", 3 => X"02fdffff",
              4 => X"03ef7fff", 5 => X"03dfbfff", 6 => X"03ff5fff", 7 => X"037fefff", 
              8 => X"03f7efff", 9 => X"03bff7ff", 10 => X"03f7f7ff", 11 => X"03ffd7ff",
              12 => X"037ffdff", 13 => X"03ff7f7f", 14 => X"03ffbf7f", 15 => X"03fff7bf"), 

        -- degree 27...
        25 => (0 => X"047fffff", 1 => X"055fffff", 2 => X"036fffff", 3 => X"03b7ffff",
               4 => X"0737ffff", 5 => X"03d7ffff", 6 => X"06d7ffff", 7 => X"0757ffff", 
               8 => X"05e7ffff", 9 => X"02fbffff", 10 => X"04fbffff", 11 => X"03f3ffff",
               12 => X"057dffff", 13 => X"076dffff", 14 => X"05f5ffff", 15 => X"06f9ffff"), 

        -- degree 28...
        26 => (0 => X"0dfdffff", 1 => X"0fddffff", 2 => X"0ff6ffff", 3 => X"0effbfff",
               4 => X"0fdfbfff", 5 => X"0fbfdfff", 6 => X"0fefdfff", 7 => X"0efff7ff", 
               8 => X"0f7ff7ff", 9 => X"0fffedff", 10 => X"0fff7eff", 11 => X"0fffbf7f",
               12 => X"0ffff77f", 13 => X"0fff7fbf", 14 => X"0ffdffdf", 15 => X"0ffdffef"), 

        -- degree 29...
        27 => (0 => X"1f7fffff", 1 => X"1ffbffff", 2 => X"1ffff7ff", 3 => X"1fffffbf",
               4 => X"05ffffff", 5 => X"09ffffff", 6 => X"0b7fffff", 7 => X"1a7fffff", 
               8 => X"16bfffff", 9 => X"0f3fffff", 10 => X"15dfffff", 11 => X"1bafffff",
               12 => X"0ef7ffff", 13 => X"1cf7ffff", 14 => X"1d77ffff", 15 => X"0fb7ffff"), 

        -- degree 30...
        28 => (0 => X"3dbfffff", 1 => X"1fdfffff", 2 => X"3f6fffff", 3 => X"37fdffff",
               4 => X"3dfeffff", 5 => X"3dff7fff", 6 => X"3fdfbfff", 7 => X"3ff7dfff", 
               8 => X"3fefefff", 9 => X"3feff7ff", 10 => X"3ffdf7ff", 11 => X"3ff7fbff",
               12 => X"3ffbfbff", 13 => X"3dfffdff", 14 => X"3ffefdff", 15 => X"3f7ffeff"), 

        -- degree 31...
        29 => (0 => X"6fffffff", 1 => X"7dffffff", 2 => X"7effffff", 3 => X"7ffbffff",
               4 => X"7fffdfff", 5 => X"7fffff7f", 6 => X"7fffffbf", 7 => X"7ffffff7", 
               8 => X"0fffffff", 9 => X"1bffffff", 10 => X"63ffffff", 11 => X"35ffffff",
               12 => X"55ffffff", 13 => X"3cffffff", 14 => X"74ffffff", 15 => X"677fffff"), 

        -- degree 32...
        30 => (0 => X"5fffffff", 1 => X"77ffffff", 2 => X"d7ffffff", 3 => X"fef7ffff",
               4 => X"7ffdffff", 5 => X"7fffefff", 6 => X"dfffefff", 7 => X"f7fff7ff", 
               8 => X"dffffdff", 9 => X"ffffdeff", 10 => X"7fffffdf", 11 => X"ffdfffdf",
               12 => X"ff7ffff7", 13 => X"ffeffff7", 14 => X"ffffffd7", 15 => X"f7fffffd")
    );

end package;

package body lfsr is

    function get_prime_poly (degree : integer range 2 to 32; sel : integer range 0 to 16)
    return std_logic_vector is
        variable vsel : integer range 0 to 16;
        variable poly : std_logic_vector(degree-1 downto 0);
    begin
        vsel := sel mod 32;
        if (vsel < 16) then
            poly := prime_poly_list_few_xors(degree - 2)(vsel mod polys_available(degree-2))(degree-1 downto 0);
        else
            vsel := vsel - 16;
            poly := prime_poly_list_many_xors(degree - 2)(vsel mod polys_available(degree-2))(degree-1 downto 0);
        end if;
        return poly;
    end;

    function get_next_lfsr_state (state : std_logic_vector; poly : std_logic_vector)
    return std_logic_vector is
        variable new_state : std_logic_vector(state'range);
    begin
        if (state(state'right) = '1') then
            new_state := state xor poly;
        else
            new_state := state;
        end if;
        new_state := state(state'right) & new_state(state'left downto 1);
        return new_state;
    end;

end package body;
