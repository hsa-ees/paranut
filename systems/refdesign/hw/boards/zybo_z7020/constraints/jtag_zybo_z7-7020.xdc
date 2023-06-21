#  This file is part of the ParaNut project.
# 
#  Copyright (C) 2019-2020 Alexander Bahle <alexander.bahle@hs-augsburg.de>
#      Hochschule Augsburg, University of Applied Sciences
#
#  Description:
# 	 JTAG adapter on Pmod port JD and BTN0 as external reset button 
#    Target Board: ZYBO Board Z7-7020
#
#  Redistribution and use in source and binary forms, with or without modification, 
#  are permitted provided that the following conditions are met:
# 
#  1. Redistributions of source code must retain the above copyright notice, this 
#     list of conditions and the following disclaimer.
# 
#  2. Redistributions in binary form must reproduce the above copyright notice,
#     this list of conditions and the following disclaimer in the documentation and/or
#     other materials provided with the distribution.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND 
#  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
#  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE 
#  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
#  ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES 
#  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; 
#  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON 
#  ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
#  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS 
#  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


### JTAG on JD, lower pin row

# TDI (JD7)
set_property IOSTANDARD LVCMOS33  [get_ports tdi];
set_property PACKAGE_PIN  U14     [get_ports tdi];

# TMS (JD8)
set_property IOSTANDARD LVCMOS33  [get_ports tms];
set_property PACKAGE_PIN  U15     [get_ports tms];

# JTAG clock (JD9)
set_property IOSTANDARD LVCMOS33  [get_ports tck];
set_property PACKAGE_PIN  V17     [get_ports tck];
# JTAG clock is constrained to 42ns (is 24MHz):
create_clock -name jtag_tck -period 20.000 [get_ports tck];
set_property CLOCK_DEDICATED_ROUTE FALSE [get_nets tck];
# Mark this clock and the system clock domain asynchronous:
set_clock_groups -asynchronous -group { clk_fpga_0 } -group [get_clocks jtag_tck]

# TDO (JD10)
set_property IOSTANDARD LVCMOS33  [get_ports tdo];
set_property PACKAGE_PIN  V18     [get_ports tdo];
