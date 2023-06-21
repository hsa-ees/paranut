
#include "memu_readport.h"
// **************** MReadPort *******************

void MReadPort::Trace (sc_trace_file *tf, int level) {
    if (!tf || pn_trace_verbose) printf ("\nSignals of Module \"%s\":\n", name ());

    // Ports ...
    PN_TRACE (tf, clk);
    PN_TRACE (tf, reset);

    PN_TRACE (tf, port_rd);
    PN_TRACE (tf, port_direct);
    PN_TRACE (tf, port_ack);
    PN_TRACE (tf, port_adr);
    PN_TRACE (tf, port_data);
    PN_TRACE (tf, port_scond_ok);
    PN_TRACE (tf, port_ac_r);
    PN_TRACE (tf, port_ac_u);
    PN_TRACE (tf, port_ac_x);

    //   Towards BUSIF...
    PN_TRACE (tf, busif_adr);
    PN_TRACE (tf, busif_data);
    PN_TRACE_BUS (tf, busif_data_valid, CFG_MEMU_CACHE_BANKS);
    PN_TRACE (tf, busif_op);
    PN_TRACE (tf, busif_busy);

    //   Towards cache...
    //   - Adress information for tag/banks are routed around this module from 'port_adr'.
    PN_TRACE (tf, tag_rd);
    PN_TRACE (tf, bank_rd);
    PN_TRACE (tf, bank_data_in);
    PN_TRACE (tf, bank_sel);
    PN_TRACE (tf, tag_in);
    PN_TRACE (tf, way_out);

    //   Request & grant lines...
    PN_TRACE (tf, req_tagr);
    PN_TRACE_BUS (tf, req_bank, CFG_MEMU_CACHE_BANKS);
    PN_TRACE (tf, req_busif);
    PN_TRACE (tf, gnt_tagr);
    PN_TRACE_BUS (tf, gnt_bank, CFG_MEMU_CACHE_BANKS);
    PN_TRACE (tf, gnt_busif);

    //   From snoop unit (arbiter)...
    PN_TRACE (tf, snoop_adr);
    PN_TRACE (tf, snoop_stb);

    // Registers...
    PN_TRACE (tf, state_trace);
    PN_TRACE (tf, bank_sel_reg);
    PN_TRACE (tf, link_adr_reg);
    PN_TRACE (tf, link_valid_reg);

    // Internal signals...
    PN_TRACE (tf, busif_hit);
    PN_TRACE (tf, next_link_adr_reg);
    PN_TRACE (tf, next_link_valid_reg);
}

void MReadPort::proc_cmb_hit () {
#pragma HLS array_partition variable=busif_data_valid
    //TODO: Add busif_bsel to hit detection

#if CFG_MEMU_BUSIF_WIDTH == 64
    // Match address for 64bit aligned read
    busif_hit = busif_data_valid[GetBankOfAdr (port_adr.read ())].read () && (port_adr.read () & ~0x4) == (busif_adr.read ());
#else
    busif_hit = busif_data_valid[GetBankOfAdr (port_adr.read ())].read () && port_adr.read () == busif_adr.read ();
#endif
}


void MReadPort::proc_clk_readport () {
    while(1){
        EReadportState next_state_var = next_state.read ();
        state_reg = next_state_var;
        state_trace = (int) next_state_var;

        bank_sel_reg = next_bank_sel.read ();

        link_adr_reg = next_link_adr_reg.read ();
        link_valid_reg = next_link_valid_reg.read ();

        ac_r_reg = next_ac_r_reg.read ();
        ac_x_reg = next_ac_x_reg.read ();
        ac_u_reg = next_ac_u_reg.read ();
        wait();
    }
    
}


void MReadPort::proc_cmb_readport () {
    sc_uint<32>  index, bank;
    bool tagr_req_rd, bank_req_rd; // 'req' and 'rd' signals can be identical
    bool tagr_gnt, bank_gnt;

    // helper variables
    sc_uint<32> port_adr_var = port_adr.read ();
    sc_uint<32> link_adr_reg_var = link_adr_reg.read ();
    sc_uint<32> snoop_adr_var = snoop_adr.read ();
    sc_uint<32> bank_sel_reg_var = bank_sel_reg.read ();
    bool link_valid_reg_var = link_valid_reg.read ();

    // Defaults for outputs...
    port_ack = 0;
    port_scond_ok = link_valid_reg_var && (link_adr_reg_var == port_adr_var);
    port_data = busif_data.read ();
    port_ac_r = ac_r_reg.read ();
    port_ac_x = ac_x_reg.read ();
    port_ac_u = ac_u_reg.read ();

    req_busif = 0;
    for (uint n = 0; n < CFG_MEMU_CACHE_BANKS; n++) req_bank[n] = 0;
    busif_op = bioNothing;

    bank_sel = bank_sel_reg_var;
    way_out = tag_in.read ().way; // TBD: need a register for this?

    next_state = state_reg.read ();
    next_bank_sel = bank_sel_reg_var;

    // Defaults for registers...
    next_link_adr_reg = link_adr_reg_var;
    next_link_valid_reg = link_valid_reg_var;

    // Helper variable...
    index = GetIndexOfAdr (port_adr_var);
    bank = GetBankOfAdr (port_adr_var);
    tagr_req_rd = bank_req_rd = 0;
    tagr_gnt = gnt_tagr;
    bank_gnt = gnt_bank[bank].read ();

     // avoid latches
    next_ac_r_reg = ac_r_reg.read ();
    next_ac_x_reg = ac_x_reg.read ();
    next_ac_u_reg = ac_u_reg.read ();
    req_tagr = 0;
    tag_rd = 0;
    bank_rd = 0;

    if (reset == 1) {
        next_state = s_rp_init;
        next_link_valid_reg = 0;
    } else {
        switch (state_reg) {

            case s_rp_init: // 0
                // Initial state: On new request, initiate all actions for the first cycle
                busif_op = bioDirectRead;
                if (port_rd == 1) {
                    if (port_direct == 1) {
                        // uncached memory access...
                        req_busif = 1;
                        next_state = s_rp_direct_wait_busif;
                    } else {
                        // cached memory access...
                        if (busif_hit == 1) {
                            next_ac_r_reg = busif_ac_r.read ();
                            next_ac_x_reg = busif_ac_x.read ();
                            next_ac_u_reg = busif_ac_u.read ();
                            port_ack = 1; // next state must set "port_data = busif_data"
                        } else {
                            tagr_req_rd = 1;
                            if (gnt_tagr == 1)
                                next_state = s_rp_read_tag;
                        }
                    }
                    if (port_lres_scond == 1) {
                        // Reserve current adress (LR operation)...
                        link_adr_reg_var = port_adr_var;
                        next_link_valid_reg = 1;
                    }
                }
                break;

            case s_rp_direct_wait_busif: // 1
                // Direct access: Wait for response from the BusIf
                busif_op = bioDirectRead;
                req_busif = !busif_hit;
                if (busif_hit == 1) {
                    next_ac_r_reg = busif_ac_r.read ();
                    next_ac_x_reg = busif_ac_x.read ();
                    next_ac_u_reg = busif_ac_u.read ();
                    port_ack = 1; // next state must set "port_data = busif_data"
                    next_state = s_rp_init;
                }
                break;

            case s_rp_read_tag: // 3
                // capture the tag (which was granted last state)
                tagr_req_rd = 1;
                bank_req_rd = 1;
                next_ac_r_reg = tag_in.read ().ac_r;
                next_ac_x_reg = tag_in.read ().ac_x;
                next_ac_u_reg = tag_in.read ().ac_u;
                if (tag_in.read ().valid == 1) {
                    // Cache hit...
                    if (bank_gnt == 1) {
                        port_ack = 1; // next state must set "port_data = bank_data_in"
                        next_bank_sel = bank;
                        next_state = s_rp_read_bank;
                    }
                } else {
                    next_state = s_rp_miss_wait_busif;
                }
                break;

            case s_rp_read_bank: // 4
                // read the bank & complete ...
                //            tagr_req_rd = 1;

                port_data = bank_data_in.read ();
                next_state = s_rp_init;

                // On new request, initiate all actions for the first cycle
                if (port_rd == 1) {
                    if (port_direct == 1) {
                        // uncached memory access...
                        busif_op = bioDirectRead;
                        req_busif = 1;
                        next_state = s_rp_direct_wait_busif;
                    } else {
                        // cached memory access...
                        if (busif_hit == 1) {
                            next_ac_r_reg = busif_ac_r.read ();
                            next_ac_x_reg = busif_ac_x.read ();
                            next_ac_u_reg = busif_ac_u.read ();
                            port_ack = 1; // next state must set "port_data = busif_data"
                        } else {
                            tagr_req_rd = 1;
                            next_state = s_rp_read_tag;
                        }
                    }
                }
                break;

            case s_rp_miss_wait_busif: // 5
                // Cache miss detected: Wait until we get the BusIf and the BusIf becomes idle and catch
                // an incidental BusIf hit if it happens.
                req_busif = 1;
                if (busif_hit == 1) {
                    next_ac_r_reg = busif_ac_r.read ();
                    next_ac_x_reg = busif_ac_x.read ();
                    next_ac_u_reg = busif_ac_u.read ();
                    port_ack = 1; // next state must set "port_data = busif_data"
                    next_state = s_rp_init;
                } else if (gnt_busif == 1 && busif_busy == 0)
                    next_state = s_rp_miss_request_tag;
                break;

            case s_rp_miss_request_tag: // 6
                // Request the tag again to check for a cache hit. The BusIf might already have replaced the
                // cache line during 's_rp_miss_wait_busif' on the request of some other port, and that data may have
                // even been modified by a write port! Hence, replacing that cache line again would lead to wrong data.
                req_busif = 1;
                tagr_req_rd = 1;
                if (gnt_tagr == 1)
                    next_state = s_rp_miss_read_tag;
                break;

            case s_rp_miss_read_tag: // 7
                // read the tag & check for a cache hit
                req_busif = 1;
                tagr_req_rd = 1;
                if (tagr_gnt == 1) {
                    bank_req_rd = 1;
                    next_ac_r_reg = tag_in.read ().ac_r;
                    next_ac_x_reg = tag_in.read ().ac_x;
                    next_ac_u_reg = tag_in.read ().ac_u;
                    if (tag_in.read ().valid == 1) {
                        if (bank_gnt == 1) {
                            next_bank_sel = bank;
                            port_ack = 1; // next state must set "port_data = bank_data_in"
                            next_state = s_rp_read_bank;
                        }
                    } else
                        next_state = s_rp_miss_replace;
                }
                break;

            case s_rp_miss_replace: // 8
                // Run the replacement and wait for the BusIf hit which MUST come some time
                req_busif = 1;
                busif_op = bioReplace;
                if (busif_hit == 1) {
                    next_ac_r_reg = busif_ac_r.read ();
                    next_ac_x_reg = busif_ac_x.read ();
                    next_ac_u_reg = busif_ac_u.read ();
                    port_ack = 1; // next state must set "port_data = busif_data"
                    next_state = s_rp_init;
                }
                break;
            case s_rp_request_tag_only:
                // TODO do something here?
                break;

        } // switch (state_reg)

        // Need to invalidate link_valid_reg?
        if (snoop_stb == 1) {
            if (snoop_adr_var == link_adr_reg_var || snoop_adr_var == link_adr_reg_var) { // need to invalidate?
                next_link_valid_reg = 0;
            }
        }

        // Set derived outputs...
        next_link_adr_reg = link_adr_reg_var;
        req_tagr = tagr_req_rd;
        tag_rd = tagr_req_rd;
        req_bank[bank] = bank_req_rd;
        bank_rd = bank_req_rd;
    }
}