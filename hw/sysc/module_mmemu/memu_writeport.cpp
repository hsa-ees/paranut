
#include "memu_writeport.h"
// **************** MWritePort *******************


void MWritePort::Trace (sc_trace_file *tf, int level) {
    if (!tf || pn_trace_verbose) printf ("\nSignals of Module \"%s\":\n", name ());

    // Ports ...
    PN_TRACE (tf, clk);
    PN_TRACE (tf, reset);

    //   Towards (CPU) port...
    //   - All input ports must be held until 'port_ack' is asserted.
    PN_TRACE (tf, port_wr);
    PN_TRACE (tf, port_direct);
    PN_TRACE (tf, port_bsel);
    PN_TRACE (tf, port_ack);

    PN_TRACE (tf, port_lres_scond);

    PN_TRACE (tf, port_cache_op);

    PN_TRACE (tf, port_adr);
    PN_TRACE (tf, port_data);
    PN_TRACE (tf, port_ac_w);
    PN_TRACE (tf, port_trap_u);
    PN_TRACE (tf, port_trap_no_u);

    //   Towards BUSIF...
    //   - There is no direct transfer except for uncached memory access.
    PN_TRACE (tf, busif_adr);
    PN_TRACE (tf, busif_op);
    PN_TRACE (tf, busif_nolinelock);
    PN_TRACE (tf, busif_busy);

    //   Towards cache...
    //   - Adress information for tag/banks must be routed from 'port_adr'.
    PN_TRACE (tf, tag_rd);
    PN_TRACE (tf, tag_wr);
    PN_TRACE (tf, bank_rd);
    PN_TRACE (tf, bank_wr);
    PN_TRACE (tf, bank_data_in);
    PN_TRACE (tf, bank_data_out);
    PN_TRACE (tf, bank_bsel);
    PN_TRACE (tf, tag_in);
    PN_TRACE (tf, tag_out);

    //   Request & grant lines...
    PN_TRACE (tf, req_linelock);
    PN_TRACE (tf, req_tagr);
    PN_TRACE (tf, req_tagw);
    PN_TRACE_BUS (tf, req_bank, CFG_MEMU_CACHE_BANKS);
    PN_TRACE (tf, req_busif);
    PN_TRACE (tf, gnt_linelock);
    PN_TRACE (tf, gnt_tagr);
    PN_TRACE (tf, gnt_tagw);
    PN_TRACE_BUS (tf, gnt_bank, CFG_MEMU_CACHE_BANKS);
    PN_TRACE (tf, gnt_busif);

    // Internal register & signals...
    PN_TRACE (tf, state_trace);
    PN_TRACE (tf, tag_reg);
    PN_TRACE (tf, data_reg);

    PN_TRACE (tf, next_tag_reg);
    PN_TRACE (tf, next_data_reg);
}


void MWritePort::proc_clk_writeport () {
    while (1)
    {
        state_reg = next_state.read ();
        state_trace = (int) next_state.read ();

        tag_reg = next_tag_reg.read ();
        data_reg = next_data_reg.read ();
        wait();
    }
    
    
}


// static sc_uint<32> CombineData (sc_uint<4> bsel, sc_uint<32>word0, sc_uint<32> word1) {
//     sc_uint<32> ret;

//     ret = word1;
// #if PN_BIG_ENDIAN == 1
//     if (bsel != 0xf) { // full word skipped (just to improve simulation speed)
//         for (int n = 0; n < 4; n++)
//             if (bsel[3 - n] == 0) {
//                 ret &= ~(0xff << (8 * n));
//                 ret |= (word0 & (0xff << (8 * n)));
//             }
//     }
// #else
//     if (bsel != 0xf) { // full word skipped (just to improve simulation speed)
//         for (int n = 0; n < 4; n++)
//             if (bsel[n] == 0) {
//                 ret &= ~(0xff << (8 * n));
//                 ret |= (word0 & (0xff << (8 * n)));
//             }
//     }
// #endif
//     return ret;
// }


void MWritePort::proc_cmb_writeport () {
    SCacheTag tag;
    sc_uint<32> index, bank;
    bool tagr_req_rd, bank_req_rd; // 'req' and 'rd' signals can be identical
    bool bank_gnt;

    // To avoid over-length paths, 'gnt_*' inputs only influence the next state, but not the outputs of this module.
    // Besides this, this is a Mealy-type machine for performance reasons. (Do we need more restrictions?)

    // Defaults for outputs...
    port_ack = 0;
    port_ac_w = 0;

    busif_op = bioNothing;
    busif_nolinelock = 1;

    tag_wr = 0;
    bank_wr = 0;
    tag_out = tag_reg.read ();
    bank_data_out = port_data.read ();
    bank_bsel = port_bsel.read ();

    req_linelock = 0;
    req_tagr = 0;
    req_tagw = 0;
    for (uint n = 0; n < CFG_MEMU_CACHE_BANKS; n++) req_bank[n] = 0;
    req_busif = 0;

    next_state = state_reg.read ();
    next_tag_reg = tag_reg.read ();
    next_data_reg = data_reg.read ();

    // Helper variables...
    index = GetIndexOfAdr (port_adr);
    bank = GetBankOfAdr (port_adr);
    tagr_req_rd = bank_req_rd = 0;
    bank_gnt = gnt_bank[bank];

    // Main "switch"...
    if (reset == 1) {
        next_state = s_wp_init;
    } else
        switch (state_reg.read ()) {

        case s_wp_init: // 0
            if (port_wr == 1) {
                if (port_direct == 1) {
                    // Direct (uncached) memory access...
                    req_busif = 1;
                    busif_op = bioDirectWrite;
                    if (gnt_busif == 1 && busif_busy == 0) next_state = s_wp_direct;
                } else {
                    // Normal (cached) access...
                    if (port_lres_scond == 1 && port_scond_ok == 0) {
                        // This is a "store conditional" (SC), which has failed right away...
                        // (a failure may still occur later, even if this condition has not yet been fulfilled here)
                        port_ack = 1;
                    } else {
                        // Normal and (so far) successful SC cache accesses...
                        req_linelock = 1;
                        tagr_req_rd = 1;
                        if (gnt_tagr == 1) {
                            if (gnt_linelock == 1)
                                next_state = s_wp_read_tag;
                            else
                                next_state = s_wp_request_linelock_only;
                        }
                    }
                }
            } else if (port_cache_op.read()[0] == 1 || port_cache_op.read()[1] == 1) {
                // Special operation...
                req_busif = 1;
                req_linelock = 1;
                if (gnt_linelock == 1 && gnt_busif == 0)
                    next_state = s_wp_special_request_busif_only;
                if (gnt_busif == 1 && busif_busy == 0) next_state = s_wp_special;
            }
            break;

            // Additional state for direct memory access...

        case s_wp_direct: // 1
            // Issue "direct write" operation...
            req_busif = 1;
            busif_op = bioDirectWrite;
            port_ac_w = busif_ac_w.read ();
            if (busif_busy == 1) {
                // Now the BUSIF is busy and has captured the data -> can ack and complete
                port_ack = 1;
                next_state = s_wp_init;
            }
            break;

            // Additional states for normal access...

        case s_wp_request_linelock_only: // 2
            // We got a grant for 'tagr', but not the 'linelock'
            // => We must release everything except the 'linelock' to avoid a deadlock
            req_linelock = 1;
            if (gnt_linelock == 1) next_state = s_wp_init;
            break;

        case s_wp_read_tag: // 3
            // Capture the tag and request the bank...
            req_linelock = 1;
            tagr_req_rd = 1;
            tag_out = tag_in.read (); // for bank reading to output the correct cache way
            next_tag_reg = tag_in.read ();
            if (tag_in.read ().valid) {
                // Cache hit...
                if (!tag_in.read ().ac_w
                    || (tag_in.read ().ac_u ? port_trap_u.read () : port_trap_no_u.read ())) {
                    // no write permission or wrong user bit
                    next_state = s_wp_page_fault;
                } else {
                    // writing to the cache bank permitted.
                    if (tag_in.read ().dirty == 1)
                        next_state = s_wp_write_bank;
                    else
                        next_state = s_wp_write_tag1_and_bank;
                }
            } else
                next_state = s_wp_miss;
            break;

        case s_wp_write_tag1_and_bank: // 4
            req_linelock = 1;
            req_tagw = 1;
            tag = tag_reg.read ();
            tag.dirty = 1;
            tag_out = tag;
            bank_req_rd = 1;
            if (bank_gnt == 1) {
                bank_wr = 1;
                next_state = s_wp_write_tag1;
            }
            if (gnt_tagw == 1) {
                tag_wr = 1;
                next_state = s_wp_write_bank;
            }
            if (bank_gnt == 1 && gnt_tagw == 1) {
                port_ack = 1; // can acknowledge to port now (write to bank and tag must be committed!)
                port_ac_w = 1;
                // req_linelock = 0; // avoid that this write port monopolizes the line lock
                bank_wr = 1;
                next_state = s_wp_init;
            }
            break;

        case s_wp_write_tag1: // 5
            req_linelock = 1;
            req_tagw = 1;
            tag = tag_reg.read ();
            tag.dirty = 1;
            tag_out = tag;
            if (gnt_tagw == 1) {
                tag_wr = 1;
                port_ack = 1;
                port_ac_w = 1;
//                req_linelock = 0; // avoid that this write port monopolizes the line lock (code below "Handle new access" can be simplified accordingly)
                next_state = s_wp_init;
            }
            break;

        case s_wp_write_bank: // 6
            req_linelock = 1;
            bank_req_rd = 1;
            bank_wr = 1;
            if (bank_gnt == 1) {
                bank_wr = 1;
                port_ack = 1; // can acknowledge to port now (write to bank must be committed!)
                port_ac_w = 1;
//                req_linelock = 0; // avoid that this write port monopolizes the line lock
                next_state = s_wp_init;
            }
            // Can we accept a new request already in this state?
            // -> No, 'tagr' must not be requested while bank is held (deadlock)
            break;

            // The following states handle a cache miss & replace a cache line.

        case s_wp_miss: // 7
            // Entry state for a cache miss. First, we must request acquire the BusIf and potentially re-acquire the line lock...
            req_busif = 1;
            req_linelock = 1;
            if (gnt_busif == 1 && busif_busy == 0 && gnt_linelock == 1)
                next_state = s_wp_recheck;
            else if (gnt_busif == 0 && gnt_linelock == 1)
                next_state = s_wp_request_busif_only;
            break;

        case s_wp_request_busif_only: // 8
            // Release the line lock and request the BusIf only to avoid deadlocks.
            req_busif = 1;
            if (gnt_busif == 1) next_state = s_wp_miss;
            break;

        case s_wp_recheck: // 9
            // Now we have the BusIf and the line lock, and the BusIf is idle. We must re-check if there is a cache hit now, since
            // some other port may have replaced the cache line in between.
            req_busif = 1;
            req_linelock = 1;
            tagr_req_rd = 1;
            if (gnt_tagr == 1)
                next_state = s_wp_recheck_read_tag;
            break;

        case s_wp_recheck_read_tag: // 10
            // Capture the tag and check it for a cache hit.
            tagr_req_rd = 1;
            req_busif = 1;
            req_linelock = 1;
            next_tag_reg = tag_in.read ();
            if (tag_in.read ().valid == 1) {
                if (!tag_in.read ().ac_w
                    || (tag_in.read ().ac_u ? port_trap_u.read () : port_trap_no_u.read ())) {
                    next_state = s_wp_page_fault;
                } else {
                    if (tag_in.read ().dirty == 1)
                        next_state = s_wp_write_bank;
                    else
                        next_state = s_wp_write_tag1_and_bank;
                }
            } else
                next_state = s_wp_replace;
            break;

        case s_wp_replace: // 11
            // Start the replacement by the BusIf.
            req_busif = 1;
            req_linelock = 1;
            busif_op = bioReplace;
            if (busif_busy == 1) next_state = s_wp_replace_wait_busif;
            break;

        case s_wp_replace_wait_busif: // 12
            // Wait for the BusIf to complete the replacement.
            req_busif = 1;
            req_linelock = 1;
            if (busif_busy == 0) {
                tagr_req_rd = 1;
                if (gnt_tagr == 1){
                    next_state = s_wp_read_tag;
                }else{
                    next_state = s_wp_init;
                }
            }
            break;

            // States for special operations...

        case s_wp_special_request_busif_only: // 13
            req_busif = 1;
            if (gnt_busif == 1) next_state = s_wp_init;
            break;

        case s_wp_special: // 14
            req_busif = 1;
            req_linelock = 1;
            busif_op = port_cache_op.read ();
            if (busif_busy == 1) next_state = s_wp_special_wait_complete;
            break;

        case s_wp_special_wait_complete: // 15
            req_busif = 1;
            req_linelock = 1;
            if (busif_busy == 0) {
                // The BUSIF has completed -> can ack and complete
                port_ack = 1;
                port_ac_w = 1;
                next_state = s_wp_init;
            }
            break;

        case s_wp_page_fault: // 16
            // writing is not permitted.
            port_ack = 1;
            port_ac_w = 0;
            next_state = s_wp_init;
        } // switch (state_reg)

    // Set derived outputs...
    req_tagr = tagr_req_rd;
    tag_rd = tagr_req_rd;
    req_bank[bank] = bank_req_rd;
    bank_rd = bank_req_rd;
}