/*************************************************************************

  This file is part of the ParaNut project.

  Copyright (C) 2010-2022 Alexander Bahle <alexander.bahle@hs-augsburg.de>
                          Gundolf Kiefer <gundolf.kiefer@hs-augsburg.de>
                          Christian H. Meyer <christian.meyer@hs-augsburg.de>
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


#ifndef __SYNTHESIS__
void MIfu::Trace (sc_trace_file *tf, int level) {
    if (!tf || pn_trace_verbose) printf ("\nSignals of Module \"%s\":\n", name ());

    // Ports...
    PN_TRACE (tf, clk);
    PN_TRACE (tf, reset);
    //   to MEMU (read port)...
    PN_TRACE (tf, rp_rd);
    PN_TRACE (tf, rp_ack);
    PN_TRACE (tf, rp_adr);
    PN_TRACE (tf, rp_data);
    PN_TRACE (tf, rp_direct);
    PN_TRACE (tf, rp_ac_u);
    PN_TRACE (tf, rp_ac_x);
    //   to EXU ...
    PN_TRACE (tf, next);
    PN_TRACE (tf, jump);
    PN_TRACE (tf, flush);
    PN_TRACE (tf, jump_adr);
    PN_TRACE (tf, ir);
    PN_TRACE (tf, pc);
    PN_TRACE (tf, npc);
    PN_TRACE (tf, ir_valid);
    PN_TRACE (tf, npc_valid);
    PN_TRACE (tf, ac_u);
    PN_TRACE (tf, ac_x);
    //   internal registers...
    PN_TRACE_BUS (tf, insn_buf, CFG_IFU_IBUF_SIZE);
    PN_TRACE_BUS (tf, adr_buf, CFG_IFU_IBUF_SIZE);
    PN_TRACE_BUS (tf, ac_x_buf, CFG_IFU_IBUF_SIZE);
    PN_TRACE_BUS (tf, ac_u_buf, CFG_IFU_IBUF_SIZE);
    PN_TRACE (tf, insn_top);
    PN_TRACE (tf, adr_top);
    PN_TRACE (tf, last_rp_ack);
    PN_TRACE (tf, rd_ack_dirty);
}
#endif

static inline bool AdrIsCached (TWord adr) { return ( adr ^ CFG_NUT_RESET_ADDR) < CFG_NUT_MEM_SIZE;  }

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
    ac_x = ac_x_buf[0].read ();
    ac_u = ac_u_buf[0].read ();
    npc = adr_buf[1].read ();
    npc_valid = (adr_top.read () > 1);

    rp_rd = sig_rp_read.read ();
    rp_adr = sig_rp_adr.read ();
    rp_paging = paging.read ();
    rp_direct = !icache_enable | !AdrIsCached (sig_rp_adr.read ());
}


void MIfu::TransitionMethod () {
#pragma HLS ARRAY_PARTITION variable = insn_buf complete dim = 1
#pragma HLS ARRAY_PARTITION variable = adr_buf complete dim = 1
#pragma HLS ARRAY_PARTITION variable = ac_x_buf complete dim = 1
#pragma HLS ARRAY_PARTITION variable = ac_u_buf complete dim = 1
    sc_uint<CFG_IFU_IBUF_SIZE_LD + 1> adr_top_var, insn_top_var, next_insn_top_var;
    bool rd_ack_dirty_var, sig_rp_read_var;
    TWord adr_buf_var[CFG_IFU_IBUF_SIZE], insn_buf_var[CFG_IFU_IBUF_SIZE];
    bool ac_x_buf_var[CFG_IFU_IBUF_SIZE], ac_u_buf_var[CFG_IFU_IBUF_SIZE];
    sc_uint<32> rp_data_var;
    bool rp_ac_x_var;
    bool rp_ac_u_var;


    // Read input signals/ports
    insn_top_var = insn_top.read ();
    adr_top_var = adr_top.read ();
    rd_ack_dirty_var = rd_ack_dirty.read ();
    sig_rp_read_var = sig_rp_read.read ();
    rp_data_var = rp_data.read ();
    rp_ac_x_var = rp_ac_x.read ();
    rp_ac_u_var = rp_ac_u.read ();

    for (int n = 0; n < CFG_IFU_IBUF_SIZE; n++) {
        adr_buf_var[n] = adr_buf[n].read ();
        insn_buf_var[n] = insn_buf[n].read ();
        ac_x_buf_var[n] = ac_x_buf[n].read ();
        ac_u_buf_var[n] = ac_u_buf[n].read ();
    }

    // Generate new address...
    if (!adr_top_var[CFG_IFU_IBUF_SIZE_LD]) {
        adr_buf_var[adr_top_var] = adr_buf_var[adr_top_var - 1] + 4;
        adr_top_var++;
    }

    // Handle jump ...
    if (jump) {
        PN_ASSERT ((jump_adr.read () & 3) == 0);
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
            ac_x_buf_var[n] = ac_x_buf_var[n+1];
            ac_u_buf_var[n] = ac_u_buf_var[n+1];
//            PN_INFOF(("   Buf%d: (0x%08x) = 0x%08x", n, adr_buf[n].read(), insn_buf[n].read()));
        }
//        PN_INFOF(("   Buf%d: (0x%08x) = 0x%08x", n, adr_buf[n].read(), insn_buf[n].read()));
        if (insn_top_var > 0) insn_top_var--;
        if (adr_top_var > 0) adr_top_var--;
    }

    // Store new memory data if available...
    if (last_rp_ack && !rd_ack_dirty && !jump && !flush) {
        PN_ASSERT (insn_top_var < CFG_IFU_IBUF_SIZE);
        insn_buf_var[insn_top_var] = rp_data_var;
        ac_x_buf_var[insn_top_var] = rp_ac_x_var;
        ac_u_buf_var[insn_top_var] = rp_ac_u_var;
        insn_top_var++;
    }
    if (last_rp_ack) {
        rd_ack_dirty_var = 0;
    }

    // Issue new memory read request if appropriate...
    next_insn_top_var = insn_top_var;
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
            ac_x_buf_var[n] = 0;
            ac_u_buf_var[n] = 0;
        }
        adr_buf_var[0] = CFG_NUT_RESET_ADDR;
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
        ac_x_buf[n] = ac_x_buf_var[n];
        ac_u_buf[n] = ac_u_buf_var[n];
    }
}
