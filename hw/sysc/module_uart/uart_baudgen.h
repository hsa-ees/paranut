/*************************************************************************

  This file is part of the ParaNut project.

  Copyright (C) 2022-2023 Elias Schuler <elias.schuler@hs-augsburg.de>
                2022-2023 Lukas Bauer <lukas.bauer@hs-ausgsburg.de>
      Hochschule Augsburg, University of Applied Sciences

    Description : This file is based on the work of Sebastian Witt
                  UART Baudgen Generator

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

#ifndef _UART_BAUDGEN_
#define _UART_BAUDGEN_

#include "base.h"

#include <systemc.h>

// **************** Defines *************
// **************** DM Register Addresses *************
// **************** Module *************

class UartBaudgen: ::sc_core::sc_module{
public:
    //Ports (Wishbone)
    sc_in_clk clk_i  {"clk_i"};    // Clock
    sc_in<bool> rst_i  {"rst_i"};  // Reset
    sc_in<bool> ce_i  {"ce_i"};    //chip enable
    sc_in<bool> clear_i  {"clear_i"}; //Reset generator (synchronisation)
    sc_in<sc_uint<16> > divider_i  {"divider_i"}; // Clock divider
    sc_out<bool> baudtick_o  {"baudtick_o"}; // 16xBaudrate tick
    //Ports
    //Constructor
    //Functions
    SC_HAS_PROCESS (UartBaudgen);
    UartBaudgen (sc_module_name name)
        : sc_module (name) {
        SC_CTHREAD (BaudgenMethod, clk_i.pos());
            reset_signal_is(rst_i, true);

    };
    void Trace (sc_trace_file *tf, int levels = 1);
#ifndef __SYNTHESIS__
#endif
    //Processes
    void BaudgenMethod();
protected:
    //Registerss
    sc_signal<sc_uint<16> > counter  {"counter"};
    //Internal Signals
};
#endif