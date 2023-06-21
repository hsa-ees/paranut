#  -----------------------------------------------------------------------------
#
#  This file is part of the ParaNut project.
#
#  Copyright (C) 2019-2022 Alexander Bahle <alexander.bahle@hs-augsburg.de>
#                     Philip Manke
#                     Christian H. Meyer <christian.meyer@hs-augsburg.de>
#                2023 Elias Schuler <elias.schuler@hs-augsburg.de>
#					            Lukas Bauer <lukas.bauer@hs-aubsburg.de>
#      Hochschule Augsburg, University of Applied Sciences
#
#  Description:
#    TCL script to be executed in Vivado's TCL mode.
#    Packages a ParaNut IP Core.
#
#  --------------------- LICENSE -----------------------------------------------
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
#  -----------------------------------------------------------------------------


# --------  Vivado IP Core Packaging script --------

#puts $argv  # Debug
#source -notrace [lindex $argv 0]

#  General Settings
set vendor_name "ees.hs-augsburg.de"
set vendor_url "http://ees.hs-augsburg.de"
set vendor_display_name "ParaNut"

#############
# IP Settings
#############
# Set the reference directory for source file relative paths (by default the value is script directory path)
set origin_dir "."

# Use origin directory path location variable, if specified in the tcl shell
if { [info exists ::origin_dir_loc] } {
  set origin_dir $::origin_dir_loc
}

set projdir $origin_dir/paranut_temp

set design "ParaNut"

# FPGA device
set partname "xc7z010clg400-1"

set constraints_files []

variable script_file
set script_file "paranut.tcl"
puts  $::argc
puts [llength $::argc]
puts [llength $argv]
if { $::argc > 0 } {
  for {set i 0} {$i < $::argc} {incr i} {
    set option [string trim [lindex $::argv $i]]
    switch -regexp -- $option {
		"--projdir" { incr i; set projdir [lindex $::argv $i]; puts  [lindex $::argv $i] }
        "--origin_dir" { incr i; set origin_dir [lindex $::argv $i]; puts  [lindex $::argv $i] }
        default {
        if { [regexp {^-} $option] } {
          puts "ERROR: Unknown option '$option' specified, please type '$script_file -tclargs --help' for usage info.\n"
          return 1
        }
      }
    }
  }
}

###########################
# Create Managed IP Project
###########################

create_project -force $design $projdir -part $partname
set_property target_language VHDL [current_project]
set_property source_mgmt_mode None [current_project]
set_property  ip_repo_paths  $origin_dir/../ip_repo [current_project]

if [info exists board_part] {
set_property "board_part" $boardpart [current_project]
}

##########################################
# Create filesets and add files to project
##########################################

#HDL
if {[string equal [get_filesets -quiet sources_1] ""]} {
    create_fileset -srcset sources_1
}

set files [list \
 "[file normalize "$origin_dir/../vhdl/paranut/types.vhd"]"\
 "[file normalize "$origin_dir/../vhdl/paranut/paranut_config.vhd"]"\
 "[file normalize "$origin_dir/../vhdl/paranut/paranut_lib.vhd"]"\
 "[file normalize "$origin_dir/../vhdl/paranut/histogram.vhd"]"\
 "[file normalize "$origin_dir/../vhdl/paranut/memu_lib.vhd"]"\
 "[file normalize "$origin_dir/../vhdl/paranut/paranut.vhd"]"\
 "[file normalize "$origin_dir/../vhdl/paranut/dbg.vhd"]"\
 "[file normalize "$origin_dir/../vhdl/paranut/ifu.vhd"]"\
 "[file normalize "$origin_dir/../vhdl/paranut/intc.vhd"]"\
 "[file normalize "$origin_dir/../vhdl/paranut/lsu.vhd"]"\
 "[file normalize "$origin_dir/../vhdl/paranut/exu.vhd"]"\
 "[file normalize "$origin_dir/../vhdl/paranut/csr.vhd"]"\
 "[file normalize "$origin_dir/../vhdl/paranut/uart.vhd"]"\
 "[file normalize "$origin_dir/../vhdl/paranut/lfsr.vhd"]"\
 "[file normalize "$origin_dir/../vhdl/paranut/mptw.vhd"]"\
 "[file normalize "$origin_dir/../vhdl/paranut/mtlb.vhd"]"\
 "[file normalize "$origin_dir/../vhdl/paranut/timer.vhd"]"\
 "[file normalize "$origin_dir/../vhdl/paranut/marbiter.vhd"]"\
 "[file normalize "$origin_dir/../vhdl/paranut/mem_tech.vhd"]"\
 "[file normalize "$origin_dir/../vhdl/paranut/mbankram.vhd"]"\
 "[file normalize "$origin_dir/../vhdl/paranut/mbusif.vhd"]"\
 "[file normalize "$origin_dir/../vhdl/paranut/mdbg.vhd"]"\
 "[file normalize "$origin_dir/../vhdl/paranut/mem_inferred.vhd"]"\
 "[file normalize "$origin_dir/../vhdl/paranut/memu.vhd"]"\
 "[file normalize "$origin_dir/../vhdl/paranut/mexu.vhd"]"\
 "[file normalize "$origin_dir/../vhdl/paranut/mcsr.vhd"]"\
 "[file normalize "$origin_dir/../vhdl/paranut/mifu.vhd"]"\
 "[file normalize "$origin_dir/../vhdl/paranut/mintc.vhd"]"\
 "[file normalize "$origin_dir/../vhdl/paranut/mlsu.vhd"]"\
 "[file normalize "$origin_dir/../vhdl/paranut/mmemu.vhd"]"\
 "[file normalize "$origin_dir/../vhdl/paranut/mparanut.vhd"]"\
 "[file normalize "$origin_dir/../vhdl/paranut/mreadport.vhd"]"\
 "[file normalize "$origin_dir/../vhdl/paranut/mtagram.vhd"]"\
 "[file normalize "$origin_dir/../vhdl/paranut/mtimer.vhd"]"\
 "[file normalize "$origin_dir/../vhdl/paranut/mwriteport.vhd"]"\
 "[file normalize "$origin_dir/../vhdl/paranut/muart.vhd"]"\
 "[file normalize "$origin_dir/../vhdl/paranut/UARTModule.v"]"\
 "[file normalize "$origin_dir/../vhdl/paranut/GPIOModule.v"]"\
 "[file normalize "$origin_dir/../vhdl/paranut/gpio.vhd"]"\
 "[file normalize "$origin_dir/../vhdl/paranut/mgpio.vhd"]"\
]
# Add all paranut library files
add_files -norecurse -fileset [get_filesets sources_1] $files
set_property library paranut [get_files  *]

# HLS files are prone to change alot
set MExuFiles [glob -nocomplain -- $origin_dir/../vhdl/paranut/MExu*.vhd]
set MIfuFiles [glob -nocomplain -- $origin_dir/../vhdl/paranut/MIfu*.vhd]
set MLsuFiles [glob -nocomplain -- $origin_dir/../vhdl/paranut/MLsu*.vhd]
set MIntCFiles [glob -nocomplain -- $origin_dir/../vhdl/paranut/MIntC*.vhd]
set MDebugModuleFiles [glob -nocomplain -- $origin_dir/../vhdl/paranut/MDebugModule*.vhd]
set MDtmFiles [glob -nocomplain -- $origin_dir/../vhdl/paranut/MDtm*.vhd]
set MMExtensionFiles [glob -nocomplain -- $origin_dir/../vhdl/paranut/MMExtension*.vhd]
set MCsrFiles [glob -nocomplain -- $origin_dir/../vhdl/paranut/MCsr*.vhd]
set MPtwFiles [glob -nocomplain -- $origin_dir/../vhdl/paranut/MPtw*.vhd]
set MTlbFiles [glob -nocomplain -- $origin_dir/../vhdl/paranut/MTlb*.vhd]
set MtimerFiles [glob -nocomplain -- $origin_dir/../vhdl/paranut/Mtimer*.vhd]

# Add all HLS files
add_files -norecurse -fileset [get_filesets sources_1] $MExuFiles
add_files -norecurse -fileset [get_filesets sources_1] $MIfuFiles
add_files -norecurse -fileset [get_filesets sources_1] $MLsuFiles
add_files -norecurse -fileset [get_filesets sources_1] $MIntCFiles
add_files -norecurse -fileset [get_filesets sources_1] $MDebugModuleFiles
add_files -norecurse -fileset [get_filesets sources_1] $MDtmFiles
add_files -norecurse -fileset [get_filesets sources_1] $MCsrFiles
add_files -norecurse -fileset [get_filesets sources_1] $MPtwFiles
add_files -norecurse -fileset [get_filesets sources_1] $MTlbFiles
add_files -norecurse -fileset [get_filesets sources_1] $MtimerFiles
if {[llength $MMExtensionFiles] != 0} {
	add_files -norecurse -fileset [get_filesets sources_1] $MMExtensionFiles
}

# Add top-level axi-wb-bridge files
set files [list \
 "[file normalize "$origin_dir/../vhdl/saxi2mwb.vhd"]"\
 "[file normalize "$origin_dir/../vhdl/swb2maxi.vhd"]"\
 "[file normalize "$origin_dir/../vhdl/ParaNut.vhd"]"\
]
add_files -norecurse -fileset [get_filesets sources_1] $files
set_property top "ParaNut" [get_filesets sources_1]

#CONSTRAINTS
if {[string equal [get_filesets -quiet constrs_1] ""]} {
  create_fileset -constrset constrs_1
}
if {[llength $constraints_files] != 0} {
    add_files -norecurse -fileset [get_filesets constrs_1] $constraints_files
}


##########################################
# Synthesize (Optional, checks for sanity)
##########################################

#set_property top $design [current_fileset]
#launch_runs synth_1 -jobs 4
#wait_on_run synth_1


#########
# Package
#########

ipx::package_project -import_files -force -root_dir $projdir -force_update_compile_order

set_property vendor              $vendor_name    [ipx::current_core]
set_property library             {user}                  [ipx::current_core]
#set_property taxonomy            {{Processing}} [ipx::current_core]
set_property vendor_display_name $vendor_display_name              [ipx::current_core]
set_property company_url         $vendor_url    [ipx::current_core]
set_property supported_families  { \
                     {zynq}       {Production} \
                     }   [ipx::current_core]


############################
# Set some options
############################
set_property driver_value 0 [ipx::get_ports m_axi_wuser -of_objects [ipx::current_core]]
set_property enablement_dependency {(spirit:decode(id('MODELPARAM_VALUE.C_M_AXI_WUSER_WIDTH')) - 1)>0} [ipx::get_ports m_axi_wuser -of_objects [ipx::current_core]]
set_property driver_value 0 [ipx::get_ports m_axi_awuser -of_objects [ipx::current_core]]
set_property enablement_dependency {(spirit:decode(id('MODELPARAM_VALUE.C_M_AXI_AWUSER_WIDTH')) - 1)>0} [ipx::get_ports m_axi_awuser -of_objects [ipx::current_core]]
set_property enablement_dependency {(spirit:decode(id('MODELPARAM_VALUE.C_M_AXI_BUSER_WIDTH')) - 1)>0} [ipx::get_ports m_axi_buser -of_objects [ipx::current_core]]


############################
# Save and Write ZIP archive
############################

ipx::create_xgui_files [ipx::current_core]
ipx::update_checksums [ipx::current_core]
ipx::save_core [ipx::current_core]
ipx::check_integrity -quiet [ipx::current_core]
set result [ipx::archive_core [concat $projdir/$design.zip] [ipx::current_core]]

if { $result > 0 } {
    puts "IP packaging completed!"
} else {
    puts "Encountered an error during IP packaging!"
}

close_project
exit
