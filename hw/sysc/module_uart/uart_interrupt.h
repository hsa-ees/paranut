/*************************************************************************

  This file is part of the ParaNut project.

  Copyright (C) 2022-2023 Elias Schuler <elias.schuler@hs-augsburg.de>
                2022-2023 Lukas Bauer <lukas.bauer@hs-ausgsburg.de>
      Hochschule Augsburg, University of Applied Sciences

    Description : This file is based on the work of Sebastian Witt
                  UART Interrupt Control

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

#ifndef _UART_INTERRUPT_
#define _UART_INTERRUPT_

#include "base.h"

#include <systemc.h>

// **************** Defines *************
// **************** DM Register Addresses *************
// **************** Module *************

class UartInterrupt: ::sc_core::sc_module{
public:
    //Ports (Wishbone)
    sc_in_clk clk_i  {"clk_i"};    // Clock
    sc_in<bool> rst_i  {"rst_i"};  // Reset
    sc_in<sc_uint<4> > ire_i  {"ire_i"}; // IER 3:0
    sc_in<sc_uint<5> > lsr_i  {"lsr_i"}; // LSR 4:0
    sc_in<bool> thi_i  {"thi_i"}; // Transmitter holding register empty interrupt
    sc_in<bool> rda_i  {"rda_i"}; // Receiver data available
    sc_in<bool> cti_i  {"cti_i"}; // Character timeout indication
    sc_in<bool> afe_i  {"afe_i"}; // Automatic flow control enable
    sc_in<sc_uint<4> > msr_i  {"msr_i"}; // MSR 3:0
    sc_out<sc_uint<4> > iir_o  {"iir_o"}; // IIR 3:0
    sc_out<bool> int_o   {"int_o"}; // Interrupt
    //Ports
    //Constructor
    //Functions
    SC_HAS_PROCESS (UartInterrupt);
    UartInterrupt (sc_module_name name)
        : sc_module (name) {
        SC_METHOD(OutputMethod);
        sensitive << ire_i << lsr_i << rda_i << cti_i << thi_i << msr_i << afe_i << IIR;
        SC_CTHREAD (InterruptMethod, clk_i.pos());
            reset_signal_is(rst_i, true);

    };
    void Trace (sc_trace_file *tf, int levels = 1);
#ifndef __SYNTHESIS__
#endif
    //Processes
    void InterruptMethod();
    void OutputMethod();
protected:
    //Registerss
    sc_signal<bool> RLSInterrupt  {"RLSInterrupt"}; // Receiver line status interrupt
    sc_signal<bool> RDAInterrupt  {"RDAInterrupt"}; // Receiver data available interrupt
    sc_signal<bool> CTIInterrupt  {"CTIInterrupt"}; // Character timeout indication interrupt
    sc_signal<bool> THRInterrupt  {"THRInterrupt"}; // Transmitter holding register empty interrupt
    sc_signal<bool> MSRInterrupt  {"MSRInterrupt"}; // Modem status interrupt
    sc_signal<sc_uint<4> > IIR  {"IIR"}; // IIR register
    //Internal Signals
};
#endif