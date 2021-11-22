# I2S Serializer
#
# Target Board: ZYBO Board (AnalogDevices SSM2603) 
#
# Author: Michael Schaeferling
# Date:   2020-07-08
# Comments: 


# Clocking constraints:

create_generated_clock -name clk_i2s_mclk [get_pins design_1_i/i2s_serializer_0/U0/i2s_clocking_0/clken_0/CLKOUT0]

set_clock_groups -asynchronous -group { clk_fpga_0 } -group { clk_i2s_mclk }


# Port constraints:

set_property  IOSTANDARD LVCMOS33 [get_ports i2s_mclk];
set_property  PACKAGE_PIN  T19    [get_ports i2s_mclk];
#set_output_delay -clock clk_i2s_bclk -min -2.000 [get_ports i2s_mclk];
#set_output_delay -clock clk_i2s_bclk -max  0.000 [get_ports i2s_mclk];

set_property  IOSTANDARD LVCMOS33 [get_ports i2s_bclk];
set_property  PACKAGE_PIN  K18    [get_ports i2s_bclk];
#set_output_delay -clock clk_i2s_bclk -min -2.000 [get_ports i2s_bclk];
#set_output_delay -clock clk_i2s_bclk -max  0.000 [get_ports i2s_bclk];

set_property  IOSTANDARD LVCMOS33 [get_ports {i2s_pblrc}];
set_property  PACKAGE_PIN  L17    [get_ports {i2s_pblrc}];
#set_output_delay -clock clk_i2s_bclk -min -2.000 [get_ports i2s_pblrc];
#set_output_delay -clock clk_i2s_bclk -max  0.000 [get_ports i2s_pblrc];

set_property  IOSTANDARD LVCMOS33 [get_ports {i2s_pbdat}];
set_property  PACKAGE_PIN  M17    [get_ports {i2s_pbdat}];
#set_output_delay -clock clk_i2s_bclk -min -2.000 [get_ports i2s_pbdat];
#set_output_delay -clock clk_i2s_bclk -max  0.000 [get_ports i2s_pbdat];

set_property  IOSTANDARD LVCMOS33 [get_ports {i2s_mute}];
set_property  PACKAGE_PIN  P18    [get_ports {i2s_mute}];
#set_output_delay -clock clk_i2s_bclk -min -2.000 [get_ports i2s_mute];
#set_output_delay -clock clk_i2s_bclk -max  0.000 [get_ports i2s_mute];



# I2C for SSM2603 configuration:
set_property IOSTANDARD LVCMOS33  [get_ports SSM2603_IIC_scl_io];
set_property PACKAGE_PIN  N18     [get_ports SSM2603_IIC_scl_io];
set_property IOSTANDARD LVCMOS33  [get_ports SSM2603_IIC_sda_io];
set_property PACKAGE_PIN  N17     [get_ports SSM2603_IIC_sda_io];
