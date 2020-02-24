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


#include "lsu.h"
#include "config.h"


#ifndef __SYNTHESIS__
void MLsu::Trace (sc_trace_file *tf, int level) {
    if (!tf || trace_verbose) printf ("\nSignals of Module \"%s\":\n", name ());

    // Ports ...
    TRACE (tf, clk);
    TRACE (tf, reset);
    //   to EXU...
    TRACE (tf, rd);
    TRACE (tf, wr);
    TRACE (tf, flush);
    TRACE (tf, lres_scond);
    TRACE (tf, cache_writeback);
    TRACE (tf, cache_invalidate);
    TRACE (tf, ack);
    TRACE (tf, align_err);
    TRACE (tf, scond_ok);
    TRACE (tf, width);
    TRACE (tf, exts);
    TRACE (tf, adr);
    TRACE (tf, rdata);
    TRACE (tf, wdata);
    //   to MEMU/read port...
    TRACE (tf, rp_rd);
    TRACE (tf, rp_bsel);
    TRACE (tf, rp_ack);
    TRACE (tf, rp_adr);
    TRACE (tf, rp_data);
    TRACE (tf, rp_direct);
    //   to MEMU/write port...
    TRACE (tf, wp_wr);
    TRACE (tf, wp_bsel);
    TRACE (tf, wp_ack);
    TRACE (tf, wp_lres_scond);
    TRACE (tf, wp_scond_ok);
    TRACE (tf, wp_writeback);
    TRACE (tf, wp_invalidate);
    TRACE (tf, wp_adr);
    TRACE (tf, wp_data);
    TRACE (tf, wp_direct);

    // Registers...
    TRACE_BUS (tf, wbuf_adr, CFG_LSU_WBUF_SIZE);
    TRACE_BUS (tf, wbuf_data, CFG_LSU_WBUF_SIZE);
    TRACE_BUS (tf, wbuf_valid, CFG_LSU_WBUF_SIZE);
    TRACE (tf, wbuf_dirty0);

    // Internal signals...
    TRACE (tf, sig_wbdata);
    TRACE (tf, sig_wbbsel);
    TRACE (tf, sig_wbuf_entry);
    TRACE (tf, sig_wbuf_entry_new);
    TRACE (tf, sig_wbuf_remove);
    TRACE (tf, sig_wbuf_write);
}
#endif


int MLsu::FindWbufHit (TWord adr) {
    for (int n = 0; n < CFG_LSU_WBUF_SIZE; n++)
        if (wbuf_adr[n] == (adr & ~3) && wbuf_valid[n].read () != 0) return n;
    return -1;
}


int MLsu::FindEmptyWbufEntry () {
    for (int n = 0; n < CFG_LSU_WBUF_SIZE; n++)
        if (wbuf_valid[n].read () == 0) return n;
    return -1;
}

void MLsu::OutputMethod () {
    TWord rdata_var, wbdata;
    sc_uint<4> bsel;
    sc_uint<CFG_LSU_WBUF_SIZE_LD + 1> wbuf_hit, wbuf_new, wbuf_entry;
    bool wbuf_dont_change0_var, align_err_var;
    TWord adr_var = adr.read (), wdata_var = wdata.read ();

    // Set defaults (don't cares are left open)...
    ack = 0;
    align_err_var = 0;

    rp_rd = 0;
    rp_adr = (TWord) (adr_var & ~3);
    rp_direct = !dcache_enable | !AdrIsCached (adr_var);

    wp_writeback = 0;
    wp_invalidate = 0;

    // Examine wbuf...
    wbuf_hit = FindWbufHit (adr_var);
    wbuf_new = FindEmptyWbufEntry ();

    // Generate 'sig_wbdata', 'sig_wbbsel'; general alignment check...
    switch (width.read ()) {
    case 0: // "00" = word
        if ((adr_var & 3) != 0 && (rd || wr)) {
            align_err_var = 1; /*ack = 1;*/
        }
        bsel = 0xf;
        wbdata = wdata_var;
        break;
    case 1: // "01" = byte,
        bsel = 1 << (adr_var & 3);
        wbdata = wdata_var & 0xff;
        wbdata = wbdata | (wbdata << 8) | (wbdata << 16) | (wbdata << 24);
        break;
    case 2: // "10" = half word
        if ((adr_var & 1) != 0 && (rd || wr)) {
            align_err_var = 1; /*ack = 1;*/
        }
        bsel = 3 << (adr_var & 3);
        wbdata = wdata_var & 0xffff;
        wbdata = wbdata | (wbdata << 16);
        break;
    }
    if (cache_writeback == 1 || cache_invalidate == 1) bsel = 0;

    sig_wbdata = wbdata;
    sig_wbbsel = bsel;

    wp_lres_scond = lres_scond.read ();
    scond_ok = wp_scond_ok.read ();

    // NOTE: The 'rdata' signal must not depend on 'rd', since 'rdata' must still be readable one cycle after de-asserting 'rd'

    // Read request: generate 'rdata', 'rp_bsel'...
    rdata_var = rp_data; // default

    rp_bsel = bsel;
    if (wbuf_hit < CFG_LSU_WBUF_SIZE) {
        //    INFO ("LSU: Serving (partially) from the write buffer");
        for (int n = 0; n < 4; n++)
            if (wbuf_valid[wbuf_hit].read ()[n]) {
#if PN_BIG_ENDIAN == 1
                rdata_var = (rdata_var & ~(0xff000000 >> (8 * n))) |
                            (wbuf_data[wbuf_hit] & (0xff000000 >> (8 * n)));
#else
                rdata_var = (rdata_var & ~(0x000000ff << (8 * n))) |
                            (wbuf_data[wbuf_hit].read () & (0x000000ff << (8 * n)));
#endif
                //                INFOF (("LSU:   byte #%i, rdata_var = 0x%08x", n, rdata_var));
            }
    }

    switch (width.read ()) { // Format data word & generate 'rdata'...
    case 1: // "01" = byte,
//        INFOF (("LSU:   Format byte before: rdata_var = 0x%08x", rdata_var));
#if PN_BIG_ENDIAN == 1
        rdata_var = (rdata_var >> (8 * (~adr_var & 3))) & 0xff;
#else
        rdata_var = (rdata_var >> (8 * (adr_var & 3))) & 0xff;
#endif
        if (exts == 1) rdata_var = (rdata_var ^ 0x80) - 0x80;
//        INFOF (("LSU:   Format byte: rdata_var = 0x%08x, exts= %i", rdata_var, exts.read()));
        break;
    case 2: // "10" = half word
//        INFOF (("LSU:   Format HW before: rdata_var = 0x%08x", rdata_var));
#if PN_BIG_ENDIAN == 1
        rdata_var = (rdata_var >> (8 * (~adr_var & 2))) & 0xffff;
#else
        rdata_var = (rdata_var >> (8 * (adr_var & 2))) & 0xffff;
#endif
        if (exts == 1) rdata_var = (rdata_var ^ 0x8000) - 0x8000;
//        INFOF (("LSU:   Format HW: rdata_var = 0x%08x, exts= %i", rdata_var, exts.read ()));
        break;
    }
    rdata = rdata_var;

    // Read request: generate 'rp_rd', 'ack'...
    if (rd == 1 && !align_err_var) {
//        INFOF (("LSU: read request, adr = %x, bsel = 0x%x, wbuf_hit = %i", adr.read (), (int)bsel, wbuf_hit));
        if (wbuf_hit < CFG_LSU_WBUF_SIZE && (bsel & ~wbuf_valid[wbuf_hit].read ()) == 0x0) {
            // we can serve all bytes from the write buffer
            //      INFO ("LSU: Serving all bytes from the write buffer");
            rp_rd = 0; // no request to memory
            ack = 1;
        } else {
            // we either have a write buffer miss or cannot serve all bytes
            // => pass through ack from the MEMU...
            rp_rd = 1; // pass request to memory
            ack = rp_ack.read ();
        }
    }

//    INFOF (("LSU:   bsel = %x, W := wbuf_valid[wbuf_hit].read () = %x, ~W = %x, bsel & ~W = %x",
//            (int)bsel, (int)wbuf_valid[wbuf_hit].read (), (int)~wbuf_valid[wbuf_hit].read (),
//            (int)(bsel & ~wbuf_valid[wbuf_hit].read ())));

    // Handle flush mode (generate 'ack')...
    if (flush && wbuf_new == 0) // IsFlushed ())
        ack = 1;

    // Generate MEMU write port signals ...
    wp_adr = lres_scond & rd ? adr_var : wbuf_adr[0].read ();
    wp_data = wbuf_data[0].read ();
    wp_bsel = wbuf_valid[0].read ();
    if (wbuf_new == 0 && ((cache_writeback == 1 || cache_invalidate == 1) || (lres_scond && !rd))) {
        wp_adr = adr_var; // set address
        wp_data = wdata_var;
        wp_bsel = bsel;
        wp_wr = lres_scond.read (); // cannot write now
        wp_writeback = cache_writeback.read ();
        wp_invalidate = cache_invalidate.read ();
        wp_direct = 0;
        ack = wp_ack.read ();
    } else {
        wp_wr = wbuf_dirty0.read (); // (wbuf_valid[0].read () != 0);
        wp_writeback = 0;
        wp_invalidate = 0;
        wp_direct = !dcache_enable | !AdrIsCached (wbuf_adr[0].read ());
    }

    // Determine place for (eventual) new wbuf entry...
    if (wr & !lres_scond) {
        if (wbuf_hit[CFG_LSU_WBUF_SIZE_LD])
            wbuf_entry = wbuf_new;
        else
            wbuf_entry = wbuf_hit;
    } else
        wbuf_entry = -1; // no need to store new entry

    sig_wbuf_entry = wbuf_entry;

    // this prevents changes of the wbuf in 2 situations:
    // - For a read hit in wbuf slot #0:
    //     make sure the wbuf is not changed in this clock cycle so that the forwarded data is still present in the next cycle
    // - For a write hit in wbuf slot #0:
    //     don't write into slot 0 if it is already writing
    if (wbuf_dirty0 == 1 && ((rd == 1 and wbuf_hit == 0) || (wr == 1 && wbuf_entry == 0)))
        wbuf_dont_change0_var = 1;
    else
        wbuf_dont_change0_var = 0;

    // Remove oldest entry if MEMU write / cache_writeback  / cache_invalidate was completed...
    if (wbuf_dont_change0_var == 0 && (wbuf_dirty0 == 0 || wp_ack == 1) && wbuf_entry != 0) {
        // we can safely remove the data...
        sig_wbuf_remove = 1;
        wbuf_entry--; // adjust new entry index, it may now point to the last entry MAX_WBUF_SIZE-1 if buffer was previously full
    } else
        sig_wbuf_remove = 0;

    sig_wbuf_entry_new = wbuf_entry;

    // Handle write request...
    if (!align_err_var && wbuf_entry < CFG_LSU_WBUF_SIZE &&
        (!(!AdrIsCached (adr_var) && wbuf_entry != 0) && wbuf_dont_change0_var == 0)) {
        // we can write into the write buffer & the transition thread will do so
        sig_wbuf_write = 1;
        ack = 1;
    } else
        sig_wbuf_write = 0;

    align_err = align_err_var;
}

void MLsu::TransitionMethod () {
#pragma HLS ARRAY_PARTITION variable = wbuf_adr complete dim = 1
#pragma HLS ARRAY_PARTITION variable = wbuf_data complete dim = 1
#pragma HLS ARRAY_PARTITION variable = wbuf_valid complete dim = 1
    // generates all register contents: wbuf_adr, wbuf_data, wbuf_valid
    TWord data, adr_var;
    sc_uint<4> valid;
    sc_uint<CFG_LSU_WBUF_SIZE_LD + 1> sig_wbuf_entry_var, sig_wbuf_entry_new_var;

    // Defaults
    data = 0;
    valid = 0;

    // Reset...
    if (reset) {
        for (int n = 0; n < CFG_LSU_WBUF_SIZE; n++)
#pragma HLS LOOP_FLATTEN
            wbuf_valid[n] = 0;
        wbuf_dirty0 = 0;
    } else {
        // Read input signals
        sig_wbuf_entry_var = sig_wbuf_entry.read (), sig_wbuf_entry_new_var = sig_wbuf_entry_new.read ();
        adr_var = adr.read ();

        // Read old data if applicable...
        if (sig_wbuf_entry_var >= 0 && sig_wbuf_entry_var < CFG_LSU_WBUF_SIZE) {
            data = wbuf_data[sig_wbuf_entry_var].read ();
            valid = wbuf_valid[sig_wbuf_entry_var].read ();
        }

        if (wp_ack == 1) wbuf_dirty0 = 0;
        if (sig_wbuf_remove == 1) {
            wbuf_dirty0 = (wbuf_valid[1].read () != 0);
            for (int n = 0; n < CFG_LSU_WBUF_SIZE - 1; n++) {
#pragma HLS LOOP_FLATTEN
                wbuf_adr[n] = wbuf_adr[n + 1].read ();
                wbuf_data[n] = wbuf_data[n + 1].read ();
                wbuf_valid[n] = wbuf_valid[n + 1].read ();
            }
            wbuf_adr[CFG_LSU_WBUF_SIZE - 1] = 0;
            wbuf_data[CFG_LSU_WBUF_SIZE - 1] = 0;
            wbuf_valid[CFG_LSU_WBUF_SIZE - 1] = 0;
        }

        // Store new entry if applicable...
        if (sig_wbuf_write == 1) {
            wbuf_adr[sig_wbuf_entry_new_var] = adr_var & ~3;
//            INFOF (("Old data = 0x%08x, sig_wbbsel = %i%i%i%i", data, (bool)sig_wbbsel.read ()[3],
//                    (bool)sig_wbbsel.read ()[2], (bool)sig_wbbsel.read ()[1], (bool)sig_wbbsel.read ()[0]));
            for (int n = 0; n < 4; n++) {
#pragma HLS LOOP_FLATTEN
                if (sig_wbbsel.read ()[n] == 1) {
                    data = (data & ~(0x000000ff << (8 * n))) |
                           (sig_wbdata.read () & (0x000000ff << (8 * n)));
                }
//                INFOF (("New data = 0x%08x", data));
            }
            wbuf_data[sig_wbuf_entry_new_var] = data;

            wbuf_valid[sig_wbuf_entry_new_var] = valid | sig_wbbsel.read ();
            if (sig_wbuf_entry_new_var == 0) wbuf_dirty0 = 1;
//            INFOF (("LSU storing data word 0x%08x to entry #%i, bsel = %i", data, sig_wbuf_entry_new,
//                    (int)(wbuf_valid[sig_wbuf_entry_new].read () | sig_wbbsel.read ())));
        }
    }
}
