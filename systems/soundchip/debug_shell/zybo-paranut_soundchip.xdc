#  This file is part of the ParaNut project.
# 
#  Copyright (C) 2019-2020 Alexander Bahle <alexander.bahle@hs-augsburg.de>
#                          Michael Schaeferling <michael.schaeferling@hs-augsburg.de>
#                          Hochschule Augsburg, University of Applied Sciences
#
#  Description:
#    - Target Board: ZYBO Board
#    - JTAG adapter on Pmod port JB and BTN0 as external reset button 
#    - I2S interface to onboard AnalogDevices SSM2603
#
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



##### ParaNut JTAG / AUX_RESET #####

### JTAG on JD, lower pin row

# JTAG clock
set_property IOSTANDARD LVCMOS33  [get_ports tck];
set_property PACKAGE_PIN  V17     [get_ports tck];
# JTAG clock is constrained to 42ns (is 24MHz):
create_clock -name jtag_tck -period 20.000 [get_ports tck];
set_property CLOCK_DEDICATED_ROUTE FALSE [get_nets tck];
# Mark this clock and the system clock domain asynchronous:
set_clock_groups -asynchronous -group { clk_fpga_0 } -group [get_clocks jtag_tck]

# TMS
set_property IOSTANDARD LVCMOS33  [get_ports tms];
set_property PACKAGE_PIN  U15     [get_ports tms];

# TDI
set_property IOSTANDARD LVCMOS33  [get_ports tdi];
set_property PACKAGE_PIN  U14     [get_ports tdi];

# TDO
set_property IOSTANDARD LVCMOS33  [get_ports tdo];
set_property PACKAGE_PIN  V18     [get_ports tdo];


### AUX_RESET on BTN0
#set_property IOSTANDARD LVCMOS33  [get_ports aux_reset_in];
#set_property PACKAGE_PIN  R18     [get_ports aux_reset_in];



##### Board LEDs, Buttons and Switches #####

#LEDs
set_property -dict { PACKAGE_PIN M14   IOSTANDARD LVCMOS33 } [get_ports { led[0] }];
set_property -dict { PACKAGE_PIN M15   IOSTANDARD LVCMOS33 } [get_ports { led[1] }];
set_property -dict { PACKAGE_PIN G14   IOSTANDARD LVCMOS33 } [get_ports { led[2] }];
set_property -dict { PACKAGE_PIN D18   IOSTANDARD LVCMOS33 } [get_ports { led[3] }];

#Buttons
set_property -dict { PACKAGE_PIN R18   IOSTANDARD LVCMOS33 } [get_ports { btn[0] }];
set_property -dict { PACKAGE_PIN P16   IOSTANDARD LVCMOS33 } [get_ports { btn[1] }];
set_property -dict { PACKAGE_PIN V16   IOSTANDARD LVCMOS33 } [get_ports { btn[2] }];
set_property -dict { PACKAGE_PIN Y16   IOSTANDARD LVCMOS33 } [get_ports { btn[3] }];

#Switches
set_property -dict { PACKAGE_PIN G15   IOSTANDARD LVCMOS33 } [get_ports { sw[0] }];
set_property -dict { PACKAGE_PIN P15   IOSTANDARD LVCMOS33 } [get_ports { sw[1] }];
set_property -dict { PACKAGE_PIN W13   IOSTANDARD LVCMOS33 } [get_ports { sw[2] }];
set_property -dict { PACKAGE_PIN T16   IOSTANDARD LVCMOS33 } [get_ports { sw[3] }];



##### I2S #####

# Clocking constraints:
create_generated_clock -name clk_i2s_mclk [get_pins I_SOUND_CHIP_DEBUG/sound_chip_debug_i/i2s_serializer_0/U0/i2s_clocking_0/clken_0/CLKOUT0]
set_clock_groups -asynchronous -group { clk_fpga_0 } -group { clk_i2s_mclk }


# Port constraints:

# I2S_MCLK
set_property  IOSTANDARD LVCMOS33 [get_ports i2s_mclk];
set_property  PACKAGE_PIN  T19    [get_ports i2s_mclk];
#set_output_delay -clock clk_i2s_bclk -min -2.000 [get_ports i2s_mclk];
#set_output_delay -clock clk_i2s_bclk -max  0.000 [get_ports i2s_mclk];

# I2S_BCLK
set_property  IOSTANDARD LVCMOS33 [get_ports i2s_bclk];
set_property  PACKAGE_PIN  K18    [get_ports i2s_bclk];
#set_output_delay -clock clk_i2s_bclk -min -2.000 [get_ports i2s_bclk];
#set_output_delay -clock clk_i2s_bclk -max  0.000 [get_ports i2s_bclk];

# I2S_PBLRC
set_property  IOSTANDARD LVCMOS33 [get_ports {i2s_pblrc}];
set_property  PACKAGE_PIN  L17    [get_ports {i2s_pblrc}];
#set_output_delay -clock clk_i2s_bclk -min -2.000 [get_ports i2s_pblrc];
#set_output_delay -clock clk_i2s_bclk -max  0.000 [get_ports i2s_pblrc];

# I2S_PBDAT
set_property  IOSTANDARD LVCMOS33 [get_ports {i2s_pbdat}];
set_property  PACKAGE_PIN  M17    [get_ports {i2s_pbdat}];
#set_output_delay -clock clk_i2s_bclk -min -2.000 [get_ports i2s_pbdat];
#set_output_delay -clock clk_i2s_bclk -max  0.000 [get_ports i2s_pbdat];

# I2S_MUTE
set_property  IOSTANDARD LVCMOS33 [get_ports {i2s_mute}];
set_property  PACKAGE_PIN  P18    [get_ports {i2s_mute}];
#set_output_delay -clock clk_i2s_bclk -min -2.000 [get_ports i2s_mute];
#set_output_delay -clock clk_i2s_bclk -max  0.000 [get_ports i2s_mute];


# I2C for SSM2603 configuration:

set_property IOSTANDARD LVCMOS33  [get_ports SSM2603_IIC_scl_io];
set_property PACKAGE_PIN  N18     [get_ports SSM2603_IIC_scl_io];
set_property IOSTANDARD LVCMOS33  [get_ports SSM2603_IIC_sda_io];
set_property PACKAGE_PIN  N17     [get_ports SSM2603_IIC_sda_io];



##### Debug #####

# General Debug on JE 1-4 and 7-10:
#set_property  IOSTANDARD LVCMOS33 [get_ports {debug[*]}];
#set_property  PACKAGE_PIN  V12    [get_ports {debug[0]}];  # JE 1
#set_property  PACKAGE_PIN  W16    [get_ports {debug[1]}];  # JE 2
#set_property  PACKAGE_PIN  J15    [get_ports {debug[2]}];  # JE 3
#set_property  PACKAGE_PIN  H15    [get_ports {debug[3]}];  # JE 4
#set_property  PACKAGE_PIN  V13    [get_ports {debug[4]}];  # JE 7
#set_property  PACKAGE_PIN  U17    [get_ports {debug[5]}];  # JE 8
#set_property  PACKAGE_PIN  T17    [get_ports {debug[6]}];  # JE 9
#set_property  PACKAGE_PIN  Y17    [get_ports {debug[7]}];  # JE 10
