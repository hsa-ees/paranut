/*************************************************************************

  This file is part of the ParaNut project.

  Copyright (C) 2019 Alexander Bahle <alexander.bahle@hs-augsburg.de>
      Hochschule Augsburg, University of Applied Sciences

  Description:
    This is a SystemC model of a simple Interrupt Controller for the
    ParaNut.

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


#ifndef _INTCONTROL_
#define _INTCONTROL_

#include "base.h"
#include "paranut-config.h"

#include <systemc.h>

// **************** INTC States *************
typedef enum {
    IntIdle,
    IntPending,
    IntHandled
} EIntStates;


// **************** MIntC *************
class MIntC : ::sc_core::sc_module {
public:
    // Ports ...
    sc_in<bool> clk, reset;

    //   to CePU
    // Interrupt request: CePU executes the interrupt handler
    sc_out<bool> ir_request;
    // Interrupt ID: Interrupt information, gets saved in mcause CSR
    sc_out<sc_uint<5> > ir_id;

    //   from CePU
    // Interrupt acknowledge: Set on jump to the interrupt handler
    sc_in<bool> ir_ack;
    // Interrupt enbale: Main enable for the interrupt controller
    sc_in<bool> ir_enable;

    //   from external interrupts:
    sc_in<sc_uint<CFG_NUT_EX_INT> > ex_int;


    // Constructor...
    SC_HAS_PROCESS (MIntC);
    MIntC (sc_module_name name)
        : sc_module (name) {
        SC_METHOD (OutputMethod);
            sensitive << state << id_reg;
        SC_METHOD (TransitionMethod);
            sensitive << clk.pos ();
    }

    // Functions...
    void Trace (sc_trace_file * tf, int levels = 1);

    // Processes...
    void OutputMethod ();
    void TransitionMethod ();

protected:
    // Registers ...
    sc_signal<sc_uint<2> > state;
    sc_signal<sc_uint<5> > id_reg;
    sc_signal<sc_uint<CFG_NUT_EX_INT> > irq_reg;
};


#endif
