# I2S Serializer
#
# Target Board: ZYBO Board (AnalogDevices SSM2603) 
#
# Author: Michael Schaeferling
# Date:   2020-07-08
# Comments: 


# Clocking constraints:

create_generated_clock -name clk_i2s_mclk [get_pins design_1_i/i2s_sound_module_0/U0/i2s_serializer_0/i2s_clocking_0/clken_0/CLKOUT0]
#create_generated_clock -name clk_i2s_bclk [get_pins design_1_i/i2s_0/U0/i2s_serializer_0/i2s_clocking_0/clkgen_0/CLKOUT1]

set_clock_groups -asynchronous -group { clk_fpga_0 } -group { clk_i2s_mclk }
#set_clock_groups -asynchronous -group { clk_fpga_0 } -group { clk_i2s_bclk }


# Port constraints:

set_property  IOSTANDARD LVCMOS33 [get_ports i2s_mclk_dbg];
set_property  PACKAGE_PIN  T14    [get_ports i2s_mclk_dbg];
#set_output_delay -clock clk_i2s_bclk -min -2.000 [get_ports i2s_mclk];
#set_output_delay -clock clk_i2s_bclk -max  0.000 [get_ports i2s_mclk];

set_property  IOSTANDARD LVCMOS33 [get_ports i2s_bclk_dbg];
set_property  PACKAGE_PIN  T15    [get_ports i2s_bclk_dbg];
#set_output_delay -clock clk_i2s_bclk -min -2.000 [get_ports i2s_bclk];
#set_output_delay -clock clk_i2s_bclk -max  0.000 [get_ports i2s_bclk];

set_property  IOSTANDARD LVCMOS33 [get_ports {i2s_pblrc_dbg}];
set_property  PACKAGE_PIN  P14    [get_ports {i2s_pblrc_dbg}];
#set_output_delay -clock clk_i2s_bclk -min -2.000 [get_ports i2s_pblrc];
#set_output_delay -clock clk_i2s_bclk -max  0.000 [get_ports i2s_pblrc];

set_property  IOSTANDARD LVCMOS33 [get_ports {i2s_pbdat_dbg}];
set_property  PACKAGE_PIN  R14    [get_ports {i2s_pbdat_dbg}];
#set_output_delay -clock clk_i2s_bclk -min -2.000 [get_ports i2s_pbdat];
#set_output_delay -clock clk_i2s_bclk -max  0.000 [get_ports i2s_pbdat];

set_property  IOSTANDARD LVCMOS33 [get_ports {i2s_mute_dbg}];
set_property  PACKAGE_PIN  U14    [get_ports {i2s_mute_dbg}];
#set_output_delay -clock clk_i2s_bclk -min -2.000 [get_ports i2s_mute];
#set_output_delay -clock clk_i2s_bclk -max  0.000 [get_ports i2s_mute];
