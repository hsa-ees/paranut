/*************************************************************************

  This file is part of the ParaNut project.

  Copyright (C) 2010-2020 Alexander Bahle <alexander.bahle@hs-augsburg.de>
                          Gundolf Kiefer <gundolf.kiefer@hs-augsburg.de>
      Hochschule Augsburg, University of Applied Sciences

  Description:
    This is a SystemC model of the instruction fetch unit (IFU) of the
    ParaNut. The IFU interfaces with the MEMU and the EXU and is capable
    of instruction prefetching.

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


#ifndef _IFU_
#define _IFU_

#include "base.h"
#include "config.h"

#include <systemc.h>


SC_MODULE (MIfu) {
public:
    // Ports ...
    sc_in<bool> clk, reset;

    //   to MEMU (read port)...
    sc_out<bool> rp_rd;
    sc_in<bool> rp_ack;
    sc_out<TWord> rp_adr;
    sc_in<TWord> rp_data;
    sc_out<bool> rp_direct;

    //   to EXU ...
    sc_in<bool> next, jump, flush;
    // (next, jump) = (1, 1) lets the (current + 1)'th instruction be the jump target.
    // Logically, 'jump' is performed before 'next'. Hence, jump instructions may either sequentially first
    // assert 'jump' and then 'next' or both signals in the same cycle.
    sc_in<TWord> jump_adr;
    sc_out<TWord> ir, pc, npc; // registered outputs
    sc_out<bool> ir_valid, npc_valid;
    sc_in<bool> icache_enable;

    // Constructor...
    SC_CTOR (MIfu) {
        SC_METHOD (OutputMethod);
            sensitive << insn_top << adr_top << next << sig_rp_read;
            for (int n = 0; n < CFG_IFU_IBUF_SIZE; n++) sensitive << insn_buf[n] << adr_buf[n];
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
    sc_signal<TWord> insn_buf[CFG_IFU_IBUF_SIZE];
    sc_signal<TWord> adr_buf[CFG_IFU_IBUF_SIZE];
    sc_signal<sc_uint<CFG_IFU_IBUF_SIZE_LD + 1> > insn_top, // 'insn_top': first buffer place with not-yet-known contents (insn)
                                                  adr_top; // 'adr_top': first buffer place with not-yet-known address

    sc_signal<bool> last_rp_ack;
    sc_signal<bool> rd_ack_dirty, sig_rp_read;
    sc_signal<TWord> sig_rp_adr;
};


#endif
