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
#    ParaNut utilization report for board zybo 7020 are implemented in this file
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

class Zybo7020:
	configuration_list = [
		"cpu_cores_ld",
		"cpu_cap1_cores",
		"m_extension",
		"a_extension"
		]
	
	utilization_table = {
		"1000": 8932,
		"1001": 9218,
		"1010": 10190,
		"1011": 10387,
		"1100": 8297,
		"1101": 8565,
		"1110": 9465,
		"1111": 9735,
		"2000": 15350,
		"2001": 15730,
		"2010": 18036,
		"2011": 18367,
		"2100": 14638,
		"2101": 15122,
		"2110": 16802,
		"2111": 17352,
		"2200": 14312,
		"2201": 14683,
		"2210": 16714,
		"2211": 17007,
		"2300": 13276,
		"2301": 13807,
		"2310": 15447,
		"2311": 16019,
		"3000": 28321,
		"3001": 29106,
		"3010": 33297,
		"3011": 34095,
		"3100": 27638,
		"3101": 28474,
		"3110": 32617,
		"3111": 33301,
		"3200": 27064,
		"3201": 27892,
		"3210": 32055,
		"3211": 32682,
		"3300": 26398,
		"3301": 27181,
		"3310": 31208,
		"3311": 32011,
		"3400": 25627,
		"3401": 26435,
		"3410": 30528,
		"3411": 31163,
		"3500": 25004,
		"3501": 25748,
		"3510": 29831,
		"3511": 30416,
		"3600": 24233,
		"3601": 25126,
		"3610": 29097,
		"3611": 29723,
		"3700": 23382,
		"3701": 24283,
		"3710": 27899,
		"3711": 28648,
	}
