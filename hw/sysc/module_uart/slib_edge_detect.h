/*************************************************************************

  This file is part of the ParaNut project.

  Copyright (C) 2022-2023 Elias Schuler <elias.schuler@hs-augsburg.de>
                2022-2023 Lukas Bauer <lukas.bauer@hs-ausgsburg.de>
      Hochschule Augsburg, University of Applied Sciences

    Description : This file is based on the work of Sebastian Witt
                  Signal edge detect

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

 **************************************************************************/

#ifndef _SLIB_EDGE_DETECT_
#define _SLIB_EDGE_DETECT_

#include "base.h"

#include <systemc.h>

// **************** Defines *************
// **************** DM Register Addresses *************
// **************** Module *************

class SlibEdgeDetect: ::sc_core::sc_module{
public:
    //Ports (Wishbone)
    sc_in_clk clk_i  {"clk_i"};    // Clock
    sc_in<bool> rst_i  {"rst_i"};  // Reset
    sc_in<bool> d_i  {"d_i"};      // Signal input
    sc_out<bool> re_o  {"re_o"};   // Rising edge
    sc_out<bool> fe_o  {"fe_o"};   // Falling edge
    //Ports
    //Constructor
    //Functions
    SC_HAS_PROCESS (SlibEdgeDetect);
    SlibEdgeDetect (sc_module_name name)
        : sc_module (name) {
        SC_METHOD(OutputMethod);
        sensitive << idd << d_i;
        SC_CTHREAD (EdgeDetectMethod, clk_i.pos());
            reset_signal_is(rst_i, true);

    };
    void Trace (sc_trace_file *tf, int levels = 1);
#ifndef __SYNTHESIS__
#endif
    //Processes
    void EdgeDetectMethod();
    void OutputMethod();
protected:
    //Registers
    sc_signal<bool> idd  {"D_register"};
    //Internal Signals
};
#endif