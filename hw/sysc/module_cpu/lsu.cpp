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


#include "lsu.h"
#include "paranut-config.h"


#ifndef __SYNTHESIS__
void MLsu::Trace (sc_trace_file *tf, int level) {
    if (!tf || pn_trace_verbose) printf ("\nSignals of Module \"%s\":\n", name ());

    // Ports ...
    PN_TRACE (tf, clk);
    PN_TRACE (tf, reset);
    //   to EXU...
    PN_TRACE (tf, rd);
    PN_TRACE (tf, wr);
    PN_TRACE (tf, flush);
    PN_TRACE (tf, paging);
    PN_TRACE (tf, lres_scond);
    PN_TRACE (tf, cache_op);
    PN_TRACE (tf, ack);
    PN_TRACE (tf, align_err);
    PN_TRACE (tf, scond_ok);
    PN_TRACE (tf, width);
    PN_TRACE (tf, exts);
    PN_TRACE (tf, adr);
    PN_TRACE (tf, rdata);
    PN_TRACE (tf, wdata);
    PN_TRACE (tf, ac_u);
    PN_TRACE (tf, ac_r);
    PN_TRACE (tf, ac_w);
    //   to MEMU/read port...
    PN_TRACE (tf, rp_rd);
    PN_TRACE (tf, rp_bsel);
    PN_TRACE (tf, rp_ack);
    PN_TRACE (tf, rp_adr);
    PN_TRACE (tf, rp_data);
    PN_TRACE (tf, rp_direct);
    //   to MEMU/write port...
    PN_TRACE (tf, wp_wr);
    PN_TRACE (tf, wp_bsel);
    PN_TRACE (tf, wp_ack);
    PN_TRACE (tf, wp_lres_scond);
    PN_TRACE (tf, wp_scond_ok);
    PN_TRACE (tf, wp_cache_op);
    PN_TRACE (tf, wp_adr);
    PN_TRACE (tf, wp_data);
    PN_TRACE (tf, wp_direct);
    PN_TRACE (tf, wp_paging);

    // Registers...
    PN_TRACE_BUS (tf, wbuf, CFG_LSU_WBUF_SIZE);
    PN_TRACE (tf, wbuf_dirty0);
    PN_TRACE (tf, wbuf_hit_reg);
    PN_TRACE (tf, wp_ack_reg);
    PN_TRACE (tf, paging_reg);

    // Internal signals...
    PN_TRACE (tf, sig_wbdata);
    PN_TRACE (tf, sig_wbbsel);
    PN_TRACE (tf, sig_wbuf_entry);
    PN_TRACE (tf, sig_wbuf_entry_new);
    PN_TRACE (tf, sig_wbuf_remove);
    PN_TRACE (tf, sig_wbuf_write);
    PN_TRACE (tf, sig_wbuf_hit);
    PN_TRACE (tf, adr_offset);

}
#endif


// **************** Helpers *********************

static inline bool AdrIsCached (sc_uint<32> adr) { return ( adr ^ CFG_NUT_RESET_ADDR) < CFG_NUT_MEM_SIZE;  }

int MLsu::FindWbufHit (sc_uint<30> adr) {
    SWbufEntry entry;
    // Special case for entry 0 - No hit when it will be removed from wbuf this cycle...
    entry = wbuf[0].read ();
    if (entry.valid != 0 && entry.adr == adr && wp_ack_reg == 0) return 0;

    for (int n = 1; n < CFG_LSU_WBUF_SIZE; n++) {
        #pragma HLS LOOP_FLATTEN
        entry = wbuf[n].read ();
        if (entry.valid != 0 && entry.adr == adr) return n;
    }
    return -1;
}


int MLsu::FindEmptyWbufEntry () {
    SWbufEntry entry;
    for (int n = 0; n < CFG_LSU_WBUF_SIZE; n++) {
        #pragma HLS LOOP_FLATTEN
        entry = wbuf[n].read ();
        if (entry.valid == 0 && entry.special == 0) return n;
    }
    return -1;
}


static inline sc_uint<32> ExpandWbufAdr (sc_uint<30> adr) {
    return (adr, sc_uint<2>(0));
}

// **************** MLsu******************

void MLsu::OutputMethod () {
    sc_uint<4> bsel, wbuf_valid_var;
    sc_uint<CFG_LSU_WBUF_SIZE_LD + 1> wbuf_hit, wbuf_hit_reg_var, wbuf_new, wbuf_entry;
    bool wbuf_dont_change0_var, align_err_var, adr_cached;
    sc_uint<32> adr_var, wdata_var, wbdata, rdata_var;
    sc_uint<2> width_var;
    sc_uint<8> byte;
    sc_uint<16> hword;
    SWbufEntry wbuf_var[CFG_LSU_WBUF_SIZE];

    // Read inputs/registers ...
    width_var = width.read();
    adr_var = adr.read ();
    wdata_var = wdata.read ();
    rdata_var = rp_data.read();
    wbuf_hit_reg_var = wbuf_hit_reg.read();

    for (int n = 0; n < CFG_LSU_WBUF_SIZE; ++n) {
        wbuf_var[n] = wbuf[n].read ();
    }

    // Examine wbuf & and determine if address is cacheable...
    wbuf_hit = FindWbufHit (adr_var(31, 2));
    wbuf_new = FindEmptyWbufEntry ();
    adr_cached = AdrIsCached (adr_var);

    // Set defaults (don't cares are left open)...
    ack = 0;
    scond_ok = wp_scond_ok.read ();

    rp_rd = 0;
    rp_adr = (adr_var(31, 2), sc_uint<2>(0));
    rp_direct = !dcache_enable | !adr_cached;
    rp_paging = paging_reg.read ();

    wbuf_valid_var = !wbuf_hit[CFG_LSU_WBUF_SIZE_LD] ? wbuf_var[wbuf_hit(CFG_LSU_WBUF_SIZE_LD, 0)].valid : sc_uint<4>(0);

    bsel = 0b0000;
    wbdata = 0x0;

    wbuf_entry = -1; // no need to store new entry

    // Set align error...
    if (((width_var == 0b00 && adr_var(1,0).or_reduce()) ||  // word
         (width_var == 0b10 && adr_var[0])) &&               // half word
         (rd || wr))
        align_err_var = 1;
    else
        align_err_var = 0;
    align_err = align_err_var;


    // NOTE: The 'rdata' signal must not depend on 'rd', since 'rdata' must still be readable one cycle after de-asserting 'rd'

    // Generate 'rdata_var' if we had partial wbuf hit
    if (!wbuf_hit_reg_var[CFG_LSU_WBUF_SIZE_LD]) {
        sc_uint<4> wbuf_valid_hit = wbuf_var[wbuf_hit_reg_var(CFG_LSU_WBUF_SIZE_LD, 0)].valid;
        sc_uint<32> wbuf_data_hit = wbuf_var[wbuf_hit_reg_var(CFG_LSU_WBUF_SIZE_LD, 0)].data;
        //    PN_INFO ("LSU: Serving (partially) from the write buffer");
        for (int n = 0; n < 4; n++)
            #pragma HLS LOOP_FLATTEN
            if (wbuf_valid_hit[n]) {
#if PN_BIG_ENDIAN == 1
                rdata_var = (rdata_var & ~(0xff000000 >> (8 * n))) |
                            (wbuf_data_hit & (0xff000000 >> (8 * n)));
#else
                rdata_var = (rdata_var & ~(0x000000ff << (8 * n))) |
                            (wbuf_data_hit & (0x000000ff << (8 * n)));
#endif
                //                PN_INFOF (("LSU:   byte #%i, rdata_var = 0x%08x", n, rdata_var));
            }
    }

    // Generate 'bsel', 'wbdata' &
    // Format and sign extend input word & generate final 'rdata_var'...
    switch (width_var) {
      case 0: // "00" = word
        bsel = 0b1111;
        wbdata = wdata_var;
        break;
      case 1: // "01" = byte,
        bsel[(int)adr_var(1,0)] = 1;
        wbdata = (wdata_var(7,0), wdata_var(7,0), wdata_var(7,0), wdata_var(7,0));
//        PN_INFOF (("LSU:   Format byte before: rdata_var = 0x%08x", rdata_var));
        // Using a flattened loop seems to be the  best solution for HLS currently
        // Other solutions result in shifters:
        //  byte = rdata_var(8*(adr_var(1,0)+1)-1, 8*adr_var(1,0));
        //  byte = (rdata_var >> (8 * (adr_var(1,0))) & 0xff;
        for (int n = 0; n < 4; n++)
            #pragma HLS LOOP_FLATTEN
#if PN_BIG_ENDIAN == 1
            if (n == ~adr_var(1,0))
#else
            if (n == adr_var(1,0))
#endif
                byte = rdata_var(8*(n+1)-1, 8*n); // "(rdata_var >> (8 * n)) & 0xff" works also
        if (exts == 1)
            rdata_var = (sc_uint<24>(byte[7] ? -1 : 0), byte);
        else
            rdata_var = (sc_uint<24>(0), byte);
//        PN_INFOF (("LSU:   Format byte: rdata_var = 0x%08x, exts= %i", rdata_var, exts.read()));
        break;
      case 2: // "10" = half word
        bsel = adr_var[1] ? 0b1100 : 0b0011;
        wbdata = (wdata_var(15,0), wdata_var(15,0));
//        PN_INFOF (("LSU:   Format HW before: rdata_var = 0x%08x", rdata_var));
#if PN_BIG_ENDIAN == 1
        if(!adr_var[1])
#else
        if(adr_var[1])
#endif
            hword = rdata_var(31,16);
        else
            hword = rdata_var(15,0);
        if (exts == 1)
            rdata_var = (sc_uint<16>(hword[15] ? -1: 0), hword);
        else
            rdata_var = (sc_uint<16>(0), hword);
//        PN_INFOF (("LSU:   Format HW: rdata_var = 0x%08x, exts= %i", rdata_var, exts.read ()));
        break;
      case 3: // "11" = special
        bsel = 0b0000;
        wbdata = wdata_var;
        break;
    }
    ac_u = !wbuf_hit_reg_var[CFG_LSU_WBUF_SIZE_LD] ? 1 : rp_ac_u.read (); // Note: WriteBuffer only contains values when paging is disabled 
    ac_r = !wbuf_hit_reg_var[CFG_LSU_WBUF_SIZE_LD] ? 1 : rp_ac_r.read ();
    rdata = rdata_var;
    sig_wbdata = wbdata;
    sig_wbbsel = bsel;
    rp_bsel = bsel;

    // Read request: generate 'rp_rd', 'ack'...
    if (rd == 1) {
//        PN_INFOF (("LSU: read request, adr = %x, bsel = 0x%x, wbuf_hit = %i", adr.read (), (int)bsel, wbuf_hit));
        if (!wbuf_hit[CFG_LSU_WBUF_SIZE_LD] && ((bsel & ~wbuf_valid_var) == 0x0)) {
            // we can serve all bytes from the write buffer
//            PN_INFO ("LSU: Serving all bytes from the write buffer");
            rp_rd = 0; // no request to memory
            ack = 1;
        } else {
            // we either have a write buffer miss or cannot serve all bytes
            // => pass through ack from the MEMU...
            rp_rd = !align_err_var; // pass request to memory
            ack = rp_ack.read ();
        }
    }

//    PN_INFOF (("LSU:   bsel = %x, W := wbuf_valid[wbuf_hit].read () = %x, ~W = %x, bsel & ~W = %x",
//            (int)bsel, (int)wbuf_valid[wbuf_hit].read (), (int)~wbuf_valid[wbuf_hit].read (),
//            (int)(bsel & ~wbuf_valid[wbuf_hit].read ())));

    // Handle flush mode (generate 'ack')...
    if (flush && wbuf_dirty0 == 0) // IsFlushed ())
        ack = 1;

    // Generate MEMU write port signals ...
    wp_adr = ExpandWbufAdr(wbuf_var[0].adr);
    wp_data = wbuf_var[0].data;
    wp_bsel = wbuf_var[0].valid;
    wp_cache_op = !wp_ack_reg ? wbuf_var[0].special : sc_uint<3>(0);
    wp_lres_scond = lres_scond.read ();
    wp_wr = (wbuf_var[0].valid != 0) & !wp_ack_reg;
    wp_direct = !dcache_enable | !AdrIsCached (ExpandWbufAdr(wbuf_var[0].adr));
    wp_paging = paging_reg.read ();
    wp_trap_no_u = trap_no_u.read ();
    wp_trap_u = trap_u.read ();
    ac_w = wp_ac_w.read () || !paging_reg.read ();
    
    if (paging_reg.read () && wr) {
        // when paging is enabled, only ACK when no data is left in the WriteBuffer -> effectively disable WriteBuffer
        ack = wp_ack.read ();
    } else if (wbuf_dirty0 == 1 && (lres_scond && wr)) {
        ack = wp_ack.read ();
    }

    // Determine place for (eventual) new wbuf entry...
    if (wr) {
        if (wbuf_hit[CFG_LSU_WBUF_SIZE_LD] | (cache_op.read () != 0))
            wbuf_entry = wbuf_new;
        else
            wbuf_entry = wbuf_hit;
    }
    sig_wbuf_entry = wbuf_entry;

    // this prevents changes of the wbuf in 2 situations:
    // - For a read hit in wbuf slot #0:
    //     make sure the wbuf is not changed in this clock cycle so that the forwarded data is still present in the next cycle
    // - For a write hit in wbuf slot #0:
    //     don't write into slot 0 if it is already writing
    if (wbuf_dirty0 == 1 && (rd == 1 && !wbuf_hit[CFG_LSU_WBUF_SIZE_LD]))
        wbuf_dont_change0_var = 1;
    else
        wbuf_dont_change0_var = 0;

    // Remove oldest entry if MEMU write / cache_writeback  / cache_invalidate was completed...
    if (wbuf_dont_change0_var == 0 && wp_ack_reg == 1) {
        // we can safely remove the data...
        sig_wbuf_remove = 1;
        wbuf_entry--; // adjust new entry index, it may now point to the last entry MAX_WBUF_SIZE-1 if buffer was previously full
    } else
        sig_wbuf_remove = 0;

    sig_wbuf_hit = wbuf_hit;
    sig_wbuf_entry_new = wbuf_entry;

    // Handle write request...
    if (wr == 1 && paging_reg.read () == 1) {
        sig_wbuf_write = !align_err_var && !wbuf_dirty0.read ();
    } else if (!wbuf_entry[CFG_LSU_WBUF_SIZE_LD] // wbuf not full
        && !(wr == 1 && wbuf_hit == 0)) {    //
        // we can write into the write buffer & the transition thread will do so
        sig_wbuf_write = !align_err_var;
        ack = lres_scond ? 0 : 1;
    } else {
        sig_wbuf_write = 0;
    }
}

void MLsu::TransitionMethod () {
#pragma HLS ARRAY_PARTITION variable = wbuf complete dim = 1
    // generates all register contents: wbuf_adr, wbuf_data, wbuf_valid
    sc_uint<32> data;
    sc_uint<32> adr_var;
    sc_uint<4> valid;
    sc_uint<CFG_LSU_WBUF_SIZE_LD + 1> sig_wbuf_entry_var, sig_wbuf_entry_new_var;
    SWbufEntry wbuf_var[CFG_LSU_WBUF_SIZE];

    // Defaults
    data = 0;
    valid = 0;

    for (int n = 0; n < CFG_LSU_WBUF_SIZE; ++n) {
        wbuf_var[n] = wbuf[n].read ();
    }

    // Reset...
    if (reset) {
        for (int n = 0; n < CFG_LSU_WBUF_SIZE; n++) {
#pragma HLS LOOP_FLATTEN
            wbuf_var[n].valid = 0;
            wbuf_var[n].special = 0;
        }
        wbuf_dirty0 = 0;
        wbuf_hit_reg = 0;
        wp_ack_reg = 0;
    } else {
        // Read input signals
        sig_wbuf_entry_var = sig_wbuf_entry.read ();
        sig_wbuf_entry_new_var = sig_wbuf_entry_new.read ();
        adr_var = adr.read ();

        // Read old data if applicable...
        if (!sig_wbuf_entry_var[CFG_LSU_WBUF_SIZE_LD]) {
            data = wbuf_var[sig_wbuf_entry_var].data;
            valid = wbuf_var[sig_wbuf_entry_var].valid;
        }

        if (wp_ack == 1)
            wbuf_dirty0 = (wbuf_var[1].valid != 0) || (wbuf_var[1].special != 0);

        if (!wbuf_dirty0) {
            // single bit to store paging is sufficient, since WriteBuffer is kind of disabled when paging is enabled
            paging_reg = paging.read ();
        }

        // Shift buffer
        if (sig_wbuf_remove == 1) {
            wbuf_dirty0 = (wbuf_var[1].valid != 0) || (wbuf_var[1].special != 0);
            for (int n = 0; n < CFG_LSU_WBUF_SIZE - 1; n++) {
#pragma HLS LOOP_FLATTEN
                wbuf_var[n] = wbuf_var[n + 1];
            }
            wbuf_var[CFG_LSU_WBUF_SIZE - 1].adr = 0;
            wbuf_var[CFG_LSU_WBUF_SIZE - 1].data = 0;
            wbuf_var[CFG_LSU_WBUF_SIZE - 1].valid = 0;
            wbuf_var[CFG_LSU_WBUF_SIZE - 1].special = 0;
        }

        // Store new entry if applicable...
        if (sig_wbuf_write == 1) {
            wbuf_var[sig_wbuf_entry_new_var].adr = adr_var(31, 2);
//            PN_INFOF (("Old data = 0x%08x, sig_wbbsel = %i%i%i%i", data, (bool)sig_wbbsel.read ()[3],
//                    (bool)sig_wbbsel.read ()[2], (bool)sig_wbbsel.read ()[1], (bool)sig_wbbsel.read ()[0]));
            for (int n = 0; n < 4; n++) {
#pragma HLS LOOP_FLATTEN
                if (sig_wbbsel.read ()[n] == 1) {
                    data = (data & ~(0x000000ff << (8 * n))) |
                           (sig_wbdata.read () & (0x000000ff << (8 * n)));
                }
//                PN_INFOF (("New data = 0x%08x", data));
            }
            wbuf_var[sig_wbuf_entry_new_var].data = data;

            wbuf_var[sig_wbuf_entry_new_var].valid = valid | sig_wbbsel.read ();
            wbuf_var[sig_wbuf_entry_new_var].special = cache_op.read ();
            if (sig_wbuf_entry_new_var == 0) wbuf_dirty0 = 1;

            PN_ASSERTF (!(wbuf_var[sig_wbuf_entry_new_var].valid != 0 && wbuf_var[sig_wbuf_entry_new_var].special != 0), ("wbuf[%d].valid and wbuf[%d].special set at the same time",  (TWord)sig_wbuf_entry_new_var, (TWord)sig_wbuf_entry_new_var));
//            PN_INFOF (("LSU storing data word 0x%08x to entry #%i, bsel = %i", data, sig_wbuf_entry_new,
//                    (int)(wbuf_valid[sig_wbuf_entry_new].read () | sig_wbbsel.read ())));
        }

        // Write wbuf_hit_reg and wp_ack_reg
        wbuf_hit_reg = sig_wbuf_hit.read();
        wp_ack_reg = wp_ack.read ();
    }

    // Writeback
    for (int n = 0; n < CFG_LSU_WBUF_SIZE; ++n) {
        wbuf[n] = wbuf_var[n];
    }
}
