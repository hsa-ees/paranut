/*************************************************************************

  This file is part of the ParaNut project.

  Copyright (C) 2010-2020 Alexander Bahle <alexander.bahle@hs-augsburg.de>
                          Gundolf Kiefer <gundolf.kiefer@hs-augsburg.de>
      Hochschule Augsburg, University of Applied Sciences

  Description:
    This is a SystemC model of the load & store unit (LSU) of the ParaNut.
    The LSU interfaces with the EXU and the MEMU (1 load & 1 store port).
    It contains a write buffer with forwarding capabilities to the
    respective read port.

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


#ifndef _LSU_
#define _LSU_

#include "base.h"
#include "config.h"

#include <systemc.h>


// TODO:
// - support uncached access (no write buffer)

// **************** MLsu *************
SC_MODULE (MLsu) {
public:
    // Ports ...
    sc_in<bool> clk, reset;

    //   to EXU...
    sc_in<bool> rd, wr, flush, cache_writeback, cache_invalidate;
    sc_in<bool> lres_scond;
    sc_out<bool> ack, align_err, scond_ok;
    sc_in<sc_uint<2> > width; // "00" = word, "01" = byte, "10" = half word
    sc_in<bool> exts;
    sc_in<TWord> adr;
    sc_out<TWord> rdata; // Note: 'rdata' is delayed by one clock relative to 'ack' (see comment to the read port interface in 'memu.h')
    sc_in<TWord> wdata;
    sc_in<bool> dcache_enable;

    //   to MEMU/read port...
    sc_out<bool> rp_rd;
    sc_out<sc_uint<4> > rp_bsel;
    sc_in<bool> rp_ack;
    sc_out<TWord> rp_adr;
    sc_in<TWord> rp_data;
    sc_out<bool> rp_direct;

    //   to MEMU/write port...
    sc_out<bool> wp_wr;
    sc_out<sc_uint<4> > wp_bsel;
    sc_in<bool> wp_ack;
    sc_out<bool> wp_lres_scond;
    sc_in<bool> wp_scond_ok;
    sc_out<bool> wp_writeback, wp_invalidate;
    sc_out<TWord> wp_adr;
    sc_out<TWord> wp_data;
    sc_out<bool> wp_direct;

    // Constructor...
    SC_CTOR (MLsu) {
        SC_METHOD (TransitionMethod);
            sensitive << clk.pos ();
        SC_METHOD (OutputMethod);
            sensitive << rd << wr << flush << cache_writeback << cache_invalidate << lres_scond << dcache_enable;
            sensitive << width << exts << adr << wdata;
            sensitive << rp_ack << rp_data << wp_ack;
            sensitive << wbuf_dirty0;
            for (int n = 0; n < CFG_LSU_WBUF_SIZE; n++)
                sensitive << wbuf_adr[n] << wbuf_data[n] << wbuf_valid[n];
    }

    // Functions...
    void Trace (sc_trace_file * tf, int level = 1);

    // Processes...
    void OutputMethod ();
    void TransitionMethod ();

protected:
    // Registers...
    sc_signal<TWord> wbuf_adr[CFG_LSU_WBUF_SIZE]; // only bits 31:2 are relevant!!
    sc_signal<TWord> wbuf_data[CFG_LSU_WBUF_SIZE];
    sc_signal<sc_uint<4> > wbuf_valid[CFG_LSU_WBUF_SIZE];
    sc_signal<bool> wbuf_dirty0; // is '1' if entry 0 needs to be written back

    // Internal signals...
    sc_signal<TWord> sig_wbdata;
    sc_signal<sc_uint<4> > sig_wbbsel;
    sc_signal<sc_uint<CFG_LSU_WBUF_SIZE_LD + 1> > sig_wbuf_entry, sig_wbuf_entry_new;
    sc_signal<bool> sig_wbuf_remove;
    sc_signal<bool> sig_wbuf_write;

    // Helper methods...
    int FindWbufHit (TWord adr);
    int FindEmptyWbufEntry ();
};


#endif
