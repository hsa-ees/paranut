#  This file is part of the ParaNut project.
#
#  Copyright (C) 2023 Elias Schuler <elias.schuler@hs-augsburg.de>
#      Hochschule Augsburg, University of Applied Sciences
#
#  Description:
#    GPIO Modul on Pmod port JC
#    Target Board: ZYBO Board 7010
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

### The GPIO INPUT AND OUTPUT PINS
set_property IOSTANDARD LVCMOS33  [get_ports {gpios[0]}];
set_property IOSTANDARD LVCMOS33  [get_ports {gpios[7]}];
set_property IOSTANDARD LVCMOS33  [get_ports {gpios[6]}];
set_property IOSTANDARD LVCMOS33  [get_ports {gpios[5]}];
set_property IOSTANDARD LVCMOS33  [get_ports {gpios[4]}];
set_property IOSTANDARD LVCMOS33  [get_ports {gpios[3]}];
set_property IOSTANDARD LVCMOS33  [get_ports {gpios[2]}];
set_property IOSTANDARD LVCMOS33  [get_ports {gpios[1]}];
### Setup the PINS to be pulldowns so for inputs we don't have floating values

set_property PULLDOWN TRUE [get_ports {gpios[1]}];
set_property PULLDOWN TRUE [get_ports {gpios[2]}];
set_property PULLDOWN TRUE [get_ports {gpios[3]}];
set_property PULLDOWN TRUE [get_ports {gpios[4]}];
set_property PULLDOWN TRUE [get_ports {gpios[5]}];
set_property PULLDOWN TRUE [get_ports {gpios[6]}];
set_property PULLDOWN TRUE [get_ports {gpios[7]}];
set_property PULLDOWN TRUE [get_ports {gpios[0]}];

set_property PACKAGE_PIN  V15     [get_ports {gpios[7]}];
set_property PACKAGE_PIN  W15     [get_ports {gpios[6]}];
set_property PACKAGE_PIN  T11     [get_ports {gpios[5]}];
set_property PACKAGE_PIN  T10     [get_ports {gpios[4]}];
set_property PACKAGE_PIN  W14     [get_ports {gpios[3]}];
set_property PACKAGE_PIN  Y14     [get_ports {gpios[2]}];
set_property PACKAGE_PIN  T12     [get_ports {gpios[1]}];
set_property PACKAGE_PIN  U12     [get_ports {gpios[0]}];