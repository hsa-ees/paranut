#  -----------------------------------------------------------------------------
#
#  This file is part of the ParaNut project.
#
#  Copyright (C) 2023      Daniel Bortkevych  <daniel.bortkevych@hs-augsburg.de>
#                          Oleg Murashko      <oleg.murashko1@hs-augsburg.de>
#                          Haris Vojic        <haris.vojic@hs-augsburg.de>
#      Hochschule Augsburg, University of Applied Sciences
#
#  Description:
#    ParaNut utilization report for board zybo 7010 are implemented in this file
#
#  --------------------- LICENSE -----------------------------------------------
#  Redistribution and use in source and binary forms, with or without 
#  modification, are permitted provided that the following conditions are met:
#
#  1. Redistributions of source code must retain the above copyright notice, 
#     this list of conditions and the following disclaimer.
#
#  2. Redistributions in binary form must reproduce the above copyright notice,
#     this list of conditions and the following disclaimer in the documentation 
#     and/or other materials provided with the distribution.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
#  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
#  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
#  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE 
#  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
#  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
#  SUBSTITUTE GOODS OR SERVICES;
#  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND 
#  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
#  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
#  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#  -----------------------------------------------------------------------------


# Bit Nr. 1 		CFG_NUT_CPU_CORES_LD
# Bit Nr. 2 		CFG_NUT_CPU_CAP1_CORES
# Bit Nr. 3 		CFG_EXU_M_EXTENSION
# Bit Nr. 4 		CFG_EXU_A_EXTENSION


class Zybo7010:
	configuration_list = [
		"cpu_cores_ld",
		"cpu_cap1_cores",
		"m_extension",
		"a_extension",
		]

	utilization_table = {
		"1000": 9303,
		"1001": 9585,
		"1010": 10559,
		"1011": 10746,
		"1100": 8662,
		"1101": 8933,
		"1110": 9837,
		"1111": 10101,
		"2000": 15712,
		"2001": 16079,
		"2010": 18476,
		"2011": 18802,
		"2100": 14995,
		"2101": 15471,
		"2110": 17106,
		"2111": 17795,
		"2200": 14669,
		"2201": 15028,
		"2210": 17021,
		"2211": 18828,
		"2300": 13641,
		"2301": 14160,
		"2310": 15801,
		"2311": 16364,
		"3000": 28714,
		"3001": 29510,
		"3010": 33816,
		"3011": 34610,
		"3100": 28034,
		"3101": 28875,
		"3110": 33146,
		"3111": 33825,
		"3200": 27454,
		"3201": 28288,
		"3210": 32550,
		"3211": 33198,
		"3300": 26793,
		"3301": 27584,
		"3310": 31728,
		"3311": 32520,
		"3400": 26019,
		"3401": 26834,
		"3410": 31028,
		"3411": 31685,
		"3500": 25397,
		"3501": 26146,
		"3510": 30347,
		"3511": 30927,
		"3600": 24616,
		"3601": 25528,
		"3610": 29584,
		"3611": 30232,
		"3700": 23769,
		"3701": 24680,
		"3710": 28412,
		"3711": 29150,
	}

