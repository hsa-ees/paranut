/*************************************************************************

  This file is part of the ParaNut project.

  Copyright (C) 2010-2020 Alexander Bahle <alexander.bahle@hs-augsburg.de>
                          Gundolf Kiefer <gundolf.kiefer@hs-augsburg.de>
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


#include "ifu.h"

#include <assert.h>

#ifndef __SYNTHESIS__
void MIfu::Trace (sc_trace_file *tf, int level) {
    if (!tf || trace_verbose) printf ("\nSignals of Module \"%s\":\n", name ());

    // Ports...
    TRACE (tf, clk);
    TRACE (tf, reset);
    //   to MEMU (read port)...
    TRACE (tf, rp_rd);
    TRACE (tf, rp_ack);
    TRACE (tf, rp_adr);
    TRACE (tf, rp_data);
    TRACE (tf, rp_direct);
    //   to EXU ...
    TRACE (tf, next);
    TRACE (tf, jump);
    TRACE (tf, flush);
    TRACE (tf, jump_adr);
    TRACE (tf, ir);
    TRACE (tf, pc);
    TRACE (tf, npc);
    TRACE (tf, ir_valid);
    TRACE (tf, npc_valid);
    //   internal registers...
    TRACE_BUS (tf, insn_buf, CFG_IFU_IBUF_SIZE);
    TRACE_BUS (tf, adr_buf, CFG_IFU_IBUF_SIZE);
    TRACE (tf, insn_top);
    TRACE (tf, adr_top);
    TRACE (tf, last_rp_ack);
    TRACE (tf, rd_ack_dirty);
}
#endif

static inline bool IsJump (sc_uint<7> op7) {
    return op7 == 0x63 // BEQ, BNEQ, BLT[U], BGE[U]
           | op7 == 0x67 // JALR
           | op7 == 0x6F; // JAL
}


void MIfu::OutputMethod () {
    // Towards EXU...
    ir = insn_buf[0].read ();
    ir_valid = (insn_top.read () != 0); //& !next;
    pc = adr_buf[0].read ();
    npc = adr_buf[1].read ();
    npc_valid = (adr_top.read () > 1);

    rp_rd = sig_rp_read.read ();
    rp_adr = sig_rp_adr.read ();
    rp_direct = !icache_enable | !AdrIsCached (sig_rp_adr.read ());
}


void MIfu::TransitionMethod () {
#pragma HLS ARRAY_PARTITION variable = insn_buf complete dim = 1
#pragma HLS ARRAY_PARTITION variable = adr_buf complete dim = 1
    sc_uint<CFG_IFU_IBUF_SIZE_LD + 1> adr_top_var, insn_top_var, next_insn_top_var;
    bool rd_ack_dirty_var, sig_rp_read_var;
    TWord adr_buf_var[CFG_IFU_IBUF_SIZE], insn_buf_var[CFG_IFU_IBUF_SIZE];
    sc_uint<32> rp_data_var;


    // Read input signals/ports
    insn_top_var = insn_top.read ();
    adr_top_var = adr_top.read ();
    rd_ack_dirty_var = rd_ack_dirty.read ();
    sig_rp_read_var = sig_rp_read.read ();
    rp_data_var = rp_data.read ();


    for (int n = 0; n < CFG_IFU_IBUF_SIZE; n++) {
        adr_buf_var[n] = adr_buf[n].read ();
        insn_buf_var[n] = insn_buf[n].read ();
    }

    // Generate new address...
    if (!adr_top_var[CFG_IFU_IBUF_SIZE_LD]) {
        adr_buf_var[adr_top_var] = adr_buf_var[adr_top_var - 1] + 4;
        adr_top_var++;
    }

    // Handle jump ...
    if (jump) {
        ASSERT ((jump_adr.read () & 3) == 0);
        // RISC-V does not have architecturally visible delay slot
        // todo: does this have any hazards?
        if (insn_top_var != 0) insn_top_var = 1;
        adr_buf_var[1] = jump_adr.read (); // todo: Maybe reset last_rp_ack?
        adr_top_var = 2;
        rd_ack_dirty_var = sig_rp_read_var;
    }

    // Shift buffer if 'next' is asserted...
    if (next) {
        for (int n = 0; n < CFG_IFU_IBUF_SIZE - 1; n++) {
            insn_buf_var[n] = insn_buf_var[n + 1];
            adr_buf_var[n] = adr_buf_var[n + 1];
//            INFOF(("   Buf%d: (0x%08x) = 0x%08x", n, adr_buf[n].read(), insn_buf[n].read()));
        }
//        INFOF(("   Buf%d: (0x%08x) = 0x%08x", n, adr_buf[n].read(), insn_buf[n].read()));
        if (insn_top_var > 0) insn_top_var--;
        if (adr_top_var > 0) adr_top_var--;
    }

    // Store new memory data if available...
    if (last_rp_ack && !rd_ack_dirty && !jump && !flush) {
        ASSERT (insn_top_var < CFG_IFU_IBUF_SIZE);
        insn_buf_var[insn_top_var] = rp_data_var;
        insn_top_var++;
    }
    if (last_rp_ack) {
        rd_ack_dirty_var = 0;
    }

    // Issue new memory read request if appropriate...
    next_insn_top_var = insn_top_var + rp_ack;
    bool jump_pending = (last_rp_ack & !rd_ack_dirty & IsJump (rp_data_var (6, 0))) |
                        (insn_top_var != 0 && IsJump (sc_uint<7> (insn_buf_var[0])));
    if (!flush // Not flushing?
        && !sig_rp_read_var && adr_top_var > next_insn_top_var && !next_insn_top_var[CFG_IFU_IBUF_SIZE_LD] // Address & space available?
        && !jump_pending) { // No jump pending?
        sig_rp_read = 1;
        sig_rp_adr = adr_buf_var[(__uint8_t) (next_insn_top_var (CFG_IFU_IBUF_SIZE_LD - 1, 0))]; // (0, CFG_IFU_IBUF_SIZE_LD-1) doesn't work;
    }

    // Handle flush ...
    if (flush) {
        insn_top_var = 1;
        rd_ack_dirty_var = sig_rp_read_var;
    }

    // Reset read signal on rp_ack
    if (rp_ack) sig_rp_read = 0;

    // Handle reset (must dominate)...
    if (reset) {
        insn_top_var = 0;
        for (int n = 0; n < CFG_IFU_IBUF_SIZE; n++) {
            insn_buf_var[n] = 0x0;
        }
        adr_buf_var[0] = CFG_NUT_SIM_MEM_ADDR;
        adr_top_var = 1;
        sig_rp_read = 0;
        rd_ack_dirty_var = 0;
    }

    // Write back values to ports/signals ...
    last_rp_ack = rp_ack.read ();
    insn_top = insn_top_var;
    adr_top = adr_top_var;
    rd_ack_dirty = rd_ack_dirty_var;

    for (int n = 0; n < CFG_IFU_IBUF_SIZE; n++) {
        adr_buf[n] = adr_buf_var[n];
        insn_buf[n] = insn_buf_var[n];
    }
}
