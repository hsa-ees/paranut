/*************************************************************************

  This file is part of the ParaNut project.

  Copyright (C) 2022-2023 Elias Schuler <elias.schuler@hs-augsburg.de>
                2022-2023 Lukas Bauer <lukas.bauer@hs-ausgsburg.de>
      Hochschule Augsburg, University of Applied Sciences

    Description : This file is based on the work of Sebastian Witt
                  Clock Divider (clock enable generator) (RATIO 4)

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

#ifndef _SLIB_CLOCK_DIV1_
#define _SLIB_CLOCK_DIV1_

#include "base.h"

#include <systemc.h>

// **************** Defines *************
#define SLIB_CLOCKDIV_RATIO1 8 // Sets the max value that can be reached by the internal clock
#define SLIB_CLOCKDIV_WIDTH1 4 // Min bitwidth that can represent the CLOCKDIV_RATIO
// **************** DM Register Addresses *************
// **************** Module *************

class SlibClockDiv1: ::sc_core::sc_module{
public:
    //Ports (Wishbone)
    sc_in_clk clk_i  {"clk_i"};      // Clock
    sc_in<bool> rst_i  {"rst_i"};    // Rest
    sc_in<bool> ce_i  {"ce_i"};      // Clock enable input
    sc_out<bool> q_o  {"q_o"};       // New clock enable output
    //Ports
    //Constructor
    //Functions
    SC_HAS_PROCESS (SlibClockDiv1);
    SlibClockDiv1 (sc_module_name name)
        : sc_module (name) {
        SC_METHOD(OutputMethod);
        sensitive << iq;
        SC_CTHREAD (ClockDivMethod, clk_i.pos());
            reset_signal_is(rst_i, true);

    };
    void Trace (sc_trace_file *tf, int levels = 1);
#ifndef __SYNTHESIS__
#endif
    //Processes
    void ClockDivMethod();
    void OutputMethod();
protected:
    //Registers
    //Internal Signals
    sc_signal<bool> iq;                             // Internal Q
    sc_signal<sc_uint<SLIB_CLOCKDIV_WIDTH1> > counter;    // Counter
};
#endif
