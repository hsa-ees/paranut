/**************************************************************************
 *
 *  This file is part of the ParaNut project.
 *
 *  Copyright (C) 2020 Alexander Bahle <alexander.bahle@hs-augsburg.de>
 *      Hochschule Augsburg, University of Applied Sciences
 *
 *  Description:
 *    This is an example of a minimal ParaNut SystemC simulation testbench.
 *
 *  --------------------- LICENSE -----------------------------------------------
 *  Redistribution and use in source and binary forms, with or without modification,
 *  are permitted provided that the following conditions are met:
 *
 *  1. Redistributions of source code must retain the above copyright notice, this
 *     list of conditions and the following disclaimer.
 *
 *  2. Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation and/or
 *     other materials provided with the distribution.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 *  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 *  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 *  ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 *  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 *  ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 *  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 **************************************************************************/


#include <systemc.h>
#include "paranut-sim.h"
#define CLK_PERIOD (1000000000 / CFG_NUT_SIM_CLK_SPEED)

int sc_main (int argc, char *argv[]) {

  // Init Testbench
  sc_clock clk("clk", sc_time(CLK_PERIOD, SC_NS));
  MParaNutSystem pnsystem("tb");
  pnsystem.clk(clk);
  pnsystem.init(argc, argv);
  pnsystem.PrintConfig();


  // ***** 1. Elaboration Phase *****

  //   ... instantiate core ...
  //
  //   EDIT HERE: Instantiate own cores as required;
  //              if applicable, map external ports to local signals
  //
  //my_core i_my_core ("my_core");
  //i_my_core.user_port_a (user_a);
  //i_my_core.user_port_b (user_b);

  //   ... add own core to the system ...
  //
  //   EDIT HERE: Use MParaNutSystem::AddSlave() to add a core to the system
  //
  //   This function is provided in to versions:
  //
  //     If a core was designed by inheriting from MPeripheral use:
  //        paranut->AddSlave (MY_CORE_IO_BASE, MY_CORE_IO_SIZE, (MPeripheral *) &i_my_core);
  //
  //     If the core is standalone connect to wishbone signals manually:
  //        paranut->AddSlave (MY_CORE_IO_BASE, MY_CORE_IO_SIZE,  
  //          &(i_my_core.wb_clk_i), &(i_my_core.wb_rst_i), &(i_my_core.wb_stb_i), 
  //          &(i_my_core.wb_cyc_i), &(i_my_core.wb_we_i), &(i_my_core.wb_cti_i), 
  //          &(i_my_core.wb_bte_i), &(i_my_core.wb_sel_i), &(i_my_core.wb_ack_o), 
  //          &(i_my_core.wb_err_o), &(i_my_core.wb_rty_o), &(i_my_core.wb_adr_i), 
  //          &(i_my_core.wb_dat_i), &(i_my_core.wb_dat_o));

  //   ... open a local trace file ...
  //
  //   EDIT HERE: If desired, create a trace file and trace arbitrary signals
  //
  //sc_trace_file *tf = sc_create_vcd_trace_file ("my_chip");  
  //tf->set_time_unit(1, SC_NS);
  //tf->delta_cycles (false);

  //   ... end of elaboration: print ParaNut configuration ...
  //paranut.PrintConfig ();


  // ***** 2. Simulation Phase *****
  sc_start();
  cout <<"\n\t\t*****Simulation complete*****" << endl;

  // ***** 3. Finish *****

  //sc_close_vcd_trace_file (tf);   // EDIT HERE: Do not forget to close your trace file, if applicable
  
  return 0;
}
