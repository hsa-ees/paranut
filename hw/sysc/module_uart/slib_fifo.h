/*************************************************************************

  This file is part of the ParaNut project.

  Copyright (C) 2022-2023 Elias Schuler <elias.schuler@hs-augsburg.de>
                2022-2023 Lukas Bauer <lukas.bauer@hs-ausgsburg.de>
      Hochschule Augsburg, University of Applied Sciences

    Description : This file is based on the work of Sebastian Witt
                  FIFO (Width 8)

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
#ifndef _SLIB_FIFO_
#define _SLIB_FIFO_

#include "base.h"

#include <systemc.h>

// **************** Defines *************

#define SLIB_FIFO_WIDTH       8U
#define SLIB_FIFO_SIZE_E      6U

// **************** DM Register Addresses *************
// **************** Module *************

class SlibFifo: ::sc_core::sc_module{
public:
    //Ports (Wishbone)
    sc_in_clk clk_i  {"clk_i"};    // Clock
    sc_in<bool> rst_i  {"rst_i"};  // Reset
    sc_in<bool> clear_i  {"clear_i"}; // Clear FIFO
    sc_in<bool> write_i  {"write_i"}; // write to FIFO
    sc_in<bool> read_i  {"read_i"}; // read to FIFO
    sc_in<sc_uint<SLIB_FIFO_WIDTH> > d_i  {"d_i"};      // FIFO input
    sc_out<sc_uint<SLIB_FIFO_WIDTH> > q_o  {"q_o"};    // FIFO output
    sc_out<bool> empty_o  {"empty_o"}; // FIFO is empty
    sc_out<bool> full_o  {"full_o"}; // FIFO is full
    sc_out<sc_uint<SLIB_FIFO_SIZE_E> > usage_o  {"usage_o"}; // FIFO usage
    //Ports
    //Constructor
    //Functions
    SC_HAS_PROCESS (SlibFifo);
    SlibFifo (sc_module_name name)
        : sc_module (name) {
        SC_METHOD(OutputMethod);
        sensitive << Usage << full << empty
        << WRAddr << RDAddr;
        SC_CTHREAD (AdrMethod, clk_i.pos());
            reset_signal_is(rst_i, true);
        SC_CTHREAD(MemMethod, clk_i.pos());
            reset_signal_is(rst_i, true);
        SC_CTHREAD(UsageMethod, clk_i.pos());
            reset_signal_is(rst_i, true);

    };
    void Trace (sc_trace_file *tf, int levels = 1);
#ifndef __SYNTHESIS__
#endif
    //Processes
    void AdrMethod();
    void MemMethod();
    void UsageMethod();
    void OutputMethod();
protected:
    //Registers
    sc_signal<bool> empty  {"empty"};
    sc_signal<bool> full  {"full"};
    sc_signal<sc_uint<SLIB_FIFO_SIZE_E + 1> > WRAddr  {"WRAddr"};
    sc_signal<sc_uint<SLIB_FIFO_SIZE_E + 1> > RDAddr  {"RDAddr"};
    sc_signal<sc_uint<SLIB_FIFO_SIZE_E> > Usage  {"Usage"};
    sc_signal<sc_uint<SLIB_FIFO_WIDTH> > FifoMem [1 << SLIB_FIFO_SIZE_E];




    //Internal Signals
};
#endif
