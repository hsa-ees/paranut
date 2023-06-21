/*************************************************************************

  This file is part of the ParaNut project.

  Copyright (C) 2010-2019 Alexander Bahle <alexander.bahle@hs-augsburg.de>
                          Gundolf Kiefer <gundolf.kiefer@hs-augsburg.de>
                          Mark Endres <mark.endres@hs-augsburg.de>
      Hochschule Augsburg, University of Applied Sciences

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


#include "intc.h"
#include <exu_csr.h>

#include <assert.h>

#ifndef __SYNTHESIS__
void MIntC::Trace (sc_trace_file *tf, int level) {
    if (!tf || pn_trace_verbose) printf ("\nSignals of Module \"%s\":\n", name ());

    // Ports...
    PN_TRACE (tf, clk);
    PN_TRACE (tf, reset);
    //   to/from EXU ...
    PN_TRACE (tf, ir_request);
    PN_TRACE (tf, ir_ack);
    PN_TRACE (tf, ir_id);
    PN_TRACE (tf, ir_enable);
    //   from external interrupt sources
    PN_TRACE (tf, ex_int);
    PN_TRACE (tf, mtimer_int);
    //   internal registers...
    PN_TRACE (tf, state);
    PN_TRACE (tf, irq_reg);
    PN_TRACE (tf, mtip_reg);
    PN_TRACE (tf, meip_reg);
}
#endif

void MIntC::OutputMethod () {
    if (state.read () == IntPending) {
        ir_request = 1;
    } else {
        ir_request = 0;
    }

    ir_id = id_reg.read ();
    mip_mtip_out = mtip_reg.read();
    mip_meip_out = meip_reg.read();
}

void MIntC::TransitionMethod () {
    sc_uint<5> id;
    // all pending external interrupts
    sc_uint<CFG_NUT_EX_INT> irq_var;
    sc_uint<2> next_state;

    // Read input signals/ports
    irq_var = irq_reg.read ();
    //irq_var |= ex_int.read ();
    next_state = state.read ();

    sc_uint<CFG_NUT_EX_INT> ex_int_var = ex_int.read ();
    bool ex_mtimer_var = mtimer_int.read ();

    id = UserSoftwareInterrupt;
    mtip_reg = 0;
    meip_reg = 0;

    // interrupt prio: mtimer > external
    if (ex_mtimer_var) {
        // timer interrupt
        id = MachineTimerInterrupt; 
        mtip_reg = 1;
    }
    else if (ex_int_var) {
        // external interrupt
        irq_var |= ex_int_var;
        meip_reg = 1;
        // Determine ID (lowest wins)
        for (int n = CFG_NUT_EX_INT - 1; n >= 0; n--) {
            if (irq_var[n]) {
                id = (sc_uint<1> (1), sc_uint<4> (n));  // exception code >= 16 (platform use)
            }
        }
    }
    else {
        irq_var = 0;  
    }


    // State machine
    switch (state.read ()) {
    case IntIdle:
        id_reg = id; 
        if ((ex_mtimer_var || irq_var.or_reduce ()) & ir_enable) {
            next_state = IntPending;
        }
        break;
    case IntPending:
        if (ir_ack) {
            next_state = IntHandled;
        }
        break;
    case IntHandled:
        // if external interupt (exception code: platform use)
        if (id_reg.read() >= 16) {
            irq_var[id_reg.read () (3, 0)] = 0;
        }
        next_state = IntIdle;
        break;
    default:
        break;
    }


    // Handle reset (must dominate)...
    if (reset) {
        next_state = IntIdle;
        //id = 0;
        irq_var = 0;
        mtip_reg = 0;
        meip_reg =0;
    }

    // Write back values to signals ...
    state = next_state;
    irq_reg = irq_var;
}
