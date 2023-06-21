/*************************************************************************

  This file is part of the ParaNut project.

  Copyright (C) 2022-2023 Elias Schuler <elias.schuler@hs-augsburg.de>
                2022-2023 Lukas Bauer <lukas.bauer@hs-ausgsburg.de>
      Hochschule Augsburg, University of Applied Sciences

    Description : This file is based on the work of Sebastian Witt
                  Counter

  Redistribution and use in source and binary forms, with or without modification,
  are permitted provided that the following conditions are met:

  1. Redistributions of source code must retain the above copyright notice, this
     list of conditions and the following disclaimer.

  2. Redistributions in binary form must reproduce the above copyright notice,
     this list of conditions and the following disclaimer in the documentation and/or
     other materials provided with the distribution.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
  ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
  ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 *************************************************************************/

#ifndef _SLIB_COUNTER_
#define _SLIB_COUNTER_

#include "base.h"

#include <systemc.h>

// **************** Defines *************
#define SLIB_COUNTER_WIDTH 4 //Counter width
// **************** DM Register Addresses *************
// **************** Module *************

class SlibCounter: ::sc_core::sc_module{
public:
    //Ports (Wishbone)
    sc_in_clk clk_i  {"clk_i"};                    // Clock
    sc_in<bool> rst_i  {"rst_i"};                  // Reset
    sc_in<bool> clear_i  {"clear_i"};              // Clear counter register
    sc_in<bool> load_i  {"load_i"};                // Load counter register
    sc_in<bool> enable_i  {"enable_i"};            // Enable count operation
    sc_in<bool> down_i  {"down_i"};                // Count direction down
    sc_in<sc_uint<SLIB_COUNTER_WIDTH> > d_i  {"d_i"};   // Load counter register
    sc_out<sc_uint<SLIB_COUNTER_WIDTH> > q_o  {"q_o"};  // Shift register output
    sc_out<bool> overflow_o  {"overflow_o"};       // Counter overflow
    //Ports
    //Constructor
    //Functions
    SC_HAS_PROCESS (SlibCounter);
    SlibCounter (sc_module_name name)
        : sc_module (name) {
        SC_METHOD(OutputMethod);
        sensitive << counter;
        SC_CTHREAD (CounterMethod, clk_i.pos());
            reset_signal_is(rst_i, true);

    };
    void Trace (sc_trace_file *tf, int levels = 1);
#ifndef __SYNTHESIS__
#endif
    //Processes
    void CounterMethod();
    void OutputMethod();
protected:
    //Registers
    sc_signal<sc_uint<SLIB_COUNTER_WIDTH+1> > counter  {"counter"};   //Counter register
    //Internal Signals
};
#endif