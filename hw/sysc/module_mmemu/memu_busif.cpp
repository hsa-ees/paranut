
#include "memu_busif.h"
// **************** MBusController **********************

void MBusController::Trace (sc_trace_file *tf, int level) {
    if (!tf || pn_trace_verbose) printf ("\nSignals of Module \"%s\":\n", name ());

    PN_TRACE (tf, wb_cyc_o);
    PN_TRACE (tf, wb_stb_o);
    PN_TRACE (tf, wb_we_o);
    PN_TRACE (tf, wb_cti_o);
    PN_TRACE (tf, wb_bte_o);
    PN_TRACE (tf, wb_sel_o);
    PN_TRACE (tf, wb_adr_o);
    PN_TRACE (tf, wb_dat_o);

    PN_TRACE (tf, switch_master);

    PN_TRACE_BUS (tf, master_cyc, MASTER_NO);
    PN_TRACE_BUS (tf, master_stb, MASTER_NO);
    PN_TRACE_BUS (tf, master_we, MASTER_NO);
    PN_TRACE_BUS (tf, master_cti, MASTER_NO);
    PN_TRACE_BUS (tf, master_bte, MASTER_NO);
    PN_TRACE_BUS (tf, master_sel, MASTER_NO);
    PN_TRACE_BUS (tf, master_adr, MASTER_NO);
    PN_TRACE_BUS (tf, master_dat, MASTER_NO);

}

void MBusController::proc_cmb_mbusc () {
    wb_cyc_o = switch_master.read () ? master_cyc[1].read () : master_cyc[0].read ();
    wb_stb_o = switch_master.read () ? master_stb[1].read () : master_stb[0].read ();
    wb_cti_o = switch_master.read () ? master_cti[1].read () : master_cti[0].read ();
    wb_bte_o = switch_master.read () ? master_bte[1].read () : master_bte[0].read ();
    wb_sel_o = switch_master.read () ? master_sel[1].read () : master_sel[0].read ();
    wb_adr_o = switch_master.read () ? master_adr[1].read () : master_adr[0].read ();
    wb_we_o = switch_master.read () ? master_we[1].read () : master_we[0].read ();
    wb_dat_o = switch_master.read () ? master_dat[1].read () : master_dat[0].read ();
}

// **************** MBusIf **********************

void MBusIf::Trace (sc_trace_file *tf, int level) {
    if (!tf || pn_trace_verbose) printf ("\nSignals of Module \"%s\":\n", name ());

    // Ports ...
    PN_TRACE (tf, clk);
    PN_TRACE (tf, reset);

    //   Bus interface (Wishbone)...
    PN_TRACE (tf, wb_cyc_o);
    PN_TRACE (tf, wb_stb_o);
    PN_TRACE (tf, wb_we_o);
    PN_TRACE (tf, wb_cti_o);
    PN_TRACE (tf, wb_bte_o);
    PN_TRACE (tf, wb_sel_o);
    PN_TRACE (tf, wb_ack_i);
    PN_TRACE (tf, wb_adr_o);
    PN_TRACE (tf, wb_dat_i);
    PN_TRACE (tf, wb_dat_o);

    PN_TRACE (tf, ptw_req);
    PN_TRACE (tf, ptw_ack);
    PN_TRACE (tf, ptw_virt_adr);
    PN_TRACE (tf, ptw_phys_adr);

    //   Control inputs/outputs...
    PN_TRACE (tf, busif_op);
    PN_TRACE (tf, busif_nolinelock);
    PN_TRACE (tf, busif_bsel);
    PN_TRACE (tf, busif_busy);

    //   Control lines to Tag & Cache banks...
    PN_TRACE (tf, tag_rd);
    PN_TRACE (tf, tag_rd_way);
    PN_TRACE (tf, tag_wr);
    PN_TRACE_BUS (tf, bank_rd, CFG_MEMU_CACHE_BANKS);
    PN_TRACE_BUS (tf, bank_wr, CFG_MEMU_CACHE_BANKS);

    //   Adress & data busses...
    PN_TRACE (tf, adr_in);
    PN_TRACE (tf, adr_out);
    PN_TRACE_BUS (tf, data_in, CFG_MEMU_CACHE_BANKS);
    PN_TRACE_BUS (tf, data_out, CFG_MEMU_CACHE_BANKS);
    PN_TRACE_BUS (tf, data_out_valid, CFG_MEMU_CACHE_BANKS);
    PN_TRACE (tf, tag_in);
    PN_TRACE (tf, tag_out);

    // Paging related lines
    PN_TRACE (tf, ac_r_out);
    PN_TRACE (tf, ac_w_out);
    PN_TRACE (tf, ac_x_out);
    PN_TRACE (tf, ac_u_out);
    PN_TRACE (tf, paging_mode);

    //   Request & grant lines...
    PN_TRACE (tf, req_linelock);
    PN_TRACE (tf, req_tagw);
    PN_TRACE (tf, req_tagr);
    PN_TRACE_BUS (tf, req_bank, CFG_MEMU_CACHE_BANKS);
    PN_TRACE (tf, gnt_linelock);
    PN_TRACE (tf, gnt_tagw);
    PN_TRACE (tf, gnt_tagr);
    PN_TRACE_BUS (tf, gnt_bank, CFG_MEMU_CACHE_BANKS);

    // Registers...
    PN_TRACE (tf, regs);
    PN_TRACE (tf, next_regs);
    PN_TRACE (tf, state_trace);
}


void MBusIf::proc_clk_busif () {
    SBusIfRegs rst;
    rst.state = BifIdle;
    rst.op = bioNothing;
    rst.linelock = 0;
    rst.virt_adr = 0;
    rst.phys_adr = 0;
    rst.adr_ofs = 0;
    rst.banks_left = -1;
    rst.last_banks_left = -1;
    rst.bsel = 0;
    rst.cnt = 0;
    rst.paging_en = 0;
    rst.ac_r = 1;
    rst.ac_w = 1;
    rst.ac_x = 1;
    rst.ac_u = 1;
    SCacheTag tag;
    rst.setTag(tag);
    for (uint n = 0; n < CFG_MEMU_CACHE_BANKS; n++){
            rst.idata_valid[n] = 0;
    }
    regs = rst;

    wait();
    while(1){
        regs = next_regs.read ();
        
        wait();
    }
    
}


void MBusIf::proc_cmb_busif () {
    const SBusIfRegs regs_var = regs.read (); // Use the const specifier to check at compile time that regs_var is never written
    SBusIfRegs next_regs_var;
    SCacheTag tag_out_var;

    // Read registers...
    next_regs_var = regs_var;
    
    // Defaults for outputs...
    wb_cyc_o = 0;
    wb_stb_o = 0;
    wb_we_o = 0;
    wb_cti_o = 0;
#if CFG_MEMU_BUSIF_WIDTH == 64
    wb_adr_o = regs_var.phys_adr & ~4;
    wb_dat_o =  regs_var.virt_adr[2] ? (regs_var.odata[GetBankOfAdr (regs_var.virt_adr)], sc_uint<32>(0)) : (sc_uint<32>(0), regs_var.odata[GetBankOfAdr (regs_var.virt_adr)]);
    wb_sel_o = regs_var.phys_adr[2] ? (regs_var.bsel, sc_uint<4>(0)) : (sc_uint<4>(0), regs_var.bsel);
#else
    wb_adr_o = regs_var.phys_adr;
    wb_dat_o = regs_var.odata[GetBankOfAdr (regs_var.virt_adr)];
    wb_sel_o = regs_var.bsel;
#endif
    // registered feedback bus cycle only for 4-16 banks
    if (BUSIF_DATA_REG_NUM == 4)
        wb_bte_o = 0b01;
    else if (BUSIF_DATA_REG_NUM == 8)
        wb_bte_o = 0b10;
    else if (BUSIF_DATA_REG_NUM == 16)
        wb_bte_o = 0b11;
    else
        wb_bte_o = 0b00;

    // set access control bits automatically to 1 when paging is not enabled
    next_regs_var.ac_r = regs_var.ac_r || !regs_var.paging_en;
    next_regs_var.ac_w = regs_var.ac_w || !regs_var.paging_en;
    next_regs_var.ac_x = regs_var.ac_x || !regs_var.paging_en;
    next_regs_var.ac_u = regs_var.ac_u || !regs_var.paging_en;

    tag_rd = 0;
    tag_rd_way = 0;
    tag_wr = 0;
    req_tagr = 0;
    req_tagw = 0;
    req_linelock = regs_var.linelock;
    for (uint n = 0; n < CFG_MEMU_CACHE_BANKS; n++) {
        req_bank[n] = 0;
        bank_rd[n] = 0;
        bank_wr[n] = 0;
    }

    busif_busy = 1;

    tag_out_var = regs_var.getTag();
    
    next_regs_var.last_banks_left = regs_var.banks_left;

    ptw_req = 0;
    ptw_virt_adr = regs_var.virt_adr;

    // State Machine...
    switch (regs_var.state) {
        // Idle state...
        case BifIdle: // 0
            busif_busy = 0;
            
            // Latch all inputs since the BUSIF requester may release the request after one clock cycle...
            next_regs_var.transl_done = 0;
            next_regs_var.op = busif_op.read ();
            next_regs_var.bsel = busif_bsel.read ();
            next_regs_var.virt_adr = adr_in.read ();
            next_regs_var.phys_adr = adr_in.read ();
            next_regs_var.adr_ofs = 0; // TBD: The optmization above was deactivated because of the missing wrap mode in the Wishbone/AXI-Bridge hardware
            next_regs_var.cnt = 0;
            next_regs_var.banks_left = -1;
            for (uint n = 0; n < CFG_MEMU_CACHE_BANKS; n++)
                next_regs_var.odata[n] = data_in[n].read (); // only needed for direct writes

            switch (regs_var.op) {
                case bioDirectRead:
                    busif_busy = 1;
                    
                    if (regs_var.paging_en) {
                        ptw_req = 1;
                        next_regs_var.state = BifDirectRead1;
                    } else {
                        wb_cyc_o = 1;
                        wb_stb_o = 1;
                        if (wb_ack_i.read () == 1) {
                            next_regs_var.idata[GetBankOfAdr (regs_var.virt_adr) / (CFG_MEMU_BUSIF_WIDTH/32)] = wb_dat_i.read ();
                            next_regs_var.idata_valid[GetBankOfAdr (regs_var.virt_adr)] = 1;
                            next_regs_var.state = BifDirectRead2;
                        } else
                            next_regs_var.state = BifDirectRead1;
                    }

                    break;

                case bioDirectWrite:
                    if (regs_var.paging_en) {
                        ptw_req = 1;
                    } else {
                        wb_cyc_o = 1;
                        wb_stb_o = 1;
                        wb_we_o = 1;
                    }
                    next_regs_var.state = BifDirectWrite;

                    break;
                case bioNothing:
                    // Do nothing, just wait...
                    next_regs_var.paging_en = paging_mode.read ();
                    break;
                default: // Includes all Cache operations (bioFlush, bioInvalidate, ...)    
                    busif_busy = 1;
                    if (busif_op.read()[2]) { // Cache all operations....
                      next_regs_var.tag_way = 0;
                      next_regs_var.virt_adr = 0;
                      next_regs_var.phys_adr = 0;
                    }
                    next_regs_var.linelock = !busif_nolinelock.read ();
                    next_regs_var.virt_adr(1, 0) = 0; // Remove last two bits
                    next_regs_var.phys_adr(1, 0) = 0; // Remove last two bits

                    if (busif_nolinelock.read ()){
                        next_regs_var.state = BifCacheRequestRTWait;
                    }else{
                        next_regs_var.state = BifCacheRequestLLWait;
                    }    
                    // Start MMU a few cycles before the physical address is actually required
                    if (regs_var.paging_en && (regs_var.op == bioReplace)) {
                        ptw_req = 1;
                    }

                    break;
            } // switch (busif_op.read ())
            break;

        // Perform WB read cycle..
        case BifDirectRead1: // 1
            if (!regs_var.paging_en || (regs_var.paging_en && regs_var.transl_done)) {
                if (regs_var.ac_r || regs_var.ac_x) {
                    wb_cyc_o = 1;
                    wb_stb_o = 1;
                    if (wb_ack_i.read () == 1) {
                        next_regs_var.idata[GetBankOfAdr (regs_var.virt_adr) / (CFG_MEMU_BUSIF_WIDTH/32)] = wb_dat_i.read ();
                        next_regs_var.idata_valid[GetBankOfAdr (regs_var.virt_adr)] = 1;
                        next_regs_var.state = BifDirectRead2;
                    }
                } else {
                    next_regs_var.idata[GetBankOfAdr (regs_var.virt_adr) / (CFG_MEMU_BUSIF_WIDTH/32)] = 0;
                    next_regs_var.idata_valid[GetBankOfAdr (regs_var.virt_adr)] = 1;
                    next_regs_var.state = BifDirectRead2;
                }
            } else if (regs_var.paging_en) {
                next_regs_var.phys_adr = ptw_phys_adr.read ();
                next_regs_var.ac_r = ptw_ac_a && ptw_ac_r;
                next_regs_var.ac_x = ptw_ac_a && ptw_ac_x;
                next_regs_var.ac_u = ptw_ac_u;
                next_regs_var.transl_done = ptw_ack.read ();
            }
        
            break;

        case BifDirectRead2: // 2
            // Reset valid signal to avoid wrong BusIF hits
            next_regs_var.idata_valid[GetBankOfAdr (regs_var.virt_adr)] = 0;
            next_regs_var.state = BifIdle;

            // Latch all inputs since the BUSIF requester may release the request after one clock cycle...
            next_regs_var.paging_en = paging_mode.read ();
            next_regs_var.transl_done = 0;
            next_regs_var.op = busif_op.read ();
            next_regs_var.bsel = busif_bsel.read ();
            next_regs_var.virt_adr = adr_in.read ();
            next_regs_var.phys_adr = adr_in.read ();
            // next_regs_var.adr_ofs = GetBankOfAdr (adr_in.read ());
            next_regs_var.adr_ofs = 0; // TBD: The optmization above was deactivated because of the missing wrap mode in the Wishbone/AXI-Bridge hardware
            next_regs_var.cnt = 0;
            next_regs_var.banks_left = -1;
            for (uint n = 0; n < CFG_MEMU_CACHE_BANKS; n++)
                next_regs_var.odata[n] = data_in[n].read (); // only needed for direct writes
            break;

        // Perform WB write cycle...
        case BifDirectWrite: // 3
            busif_busy = 0;
            
            if (!regs_var.paging_en || (regs_var.paging_en && regs_var.transl_done)) {
                busif_busy = 1;
                if (regs_var.ac_w) {
                    wb_cyc_o = 1;
                    wb_stb_o = 1;
                    wb_we_o = 1;
                    // Set to BIO_NOTHING because the next instruction will be ready at the earliest
                    // in the idle state (make sure we don't execute the same instruction twice)
                    next_regs_var.op = bioNothing;
                    if (wb_ack_i) {
                        next_regs_var.state = BifIdle;
                    }
                } else {
                    next_regs_var.state = BifIdle;
                }
            } else if (regs_var.paging_en) {
                next_regs_var.phys_adr = ptw_phys_adr.read ();
                next_regs_var.ac_w = ptw_ac_a 
                                        && ptw_ac_d 
                                        && ptw_ac_w
                                        && (ptw_ac_u ? !trap_u : !trap_no_u);
                next_regs_var.transl_done = ptw_ack;
            }
            
            break;

        // Perform all cache-related operations...
        case BifCacheRequestLLWait: // 4
            if (gnt_linelock.read ())
                next_regs_var.state = BifCacheRequestRTWait;

            if (ptw_ack) {
                next_regs_var.phys_adr = ptw_phys_adr.read ();
                next_regs_var.ac_r = ptw_ac_a && ptw_ac_r;
                next_regs_var.ac_w = ptw_ac_a && ptw_ac_d && ptw_ac_w;
                next_regs_var.ac_x = ptw_ac_a && ptw_ac_x;
                next_regs_var.ac_u = ptw_ac_u;
                next_regs_var.transl_done = 1;
            }
            break;

        case BifCacheRequestRTWait: // 5
            req_tagr = 1;
            tag_rd = 1;
            tag_rd_way = regs_var.op[2];
            if (gnt_tagr.read ())
                next_regs_var.state = BifCacheReadTag;

            if (ptw_ack && !regs_var.transl_done) {
                next_regs_var.phys_adr = ptw_phys_adr.read ();
                next_regs_var.ac_r = ptw_ac_a && ptw_ac_r;
                next_regs_var.ac_w = ptw_ac_a && ptw_ac_d && ptw_ac_w;
                next_regs_var.ac_x = ptw_ac_a && ptw_ac_x;
                next_regs_var.ac_u = ptw_ac_u;
                next_regs_var.transl_done = 1;
            }

            break;

        case BifCacheReadTag: // 6
            req_tagr = 1;
            tag_rd = 1;
            tag_rd_way = regs_var.op[2];
            // Read tag..
            next_regs_var.setTag(tag_in.read ());
            if (regs_var.op[2]) next_regs_var.tag_way = regs_var.tag_way; // Preserve current way
            switch (regs_var.op) {
                case bioReplace:
                    if (!regs_var.paging_en || (regs_var.paging_en && regs_var.transl_done)) {
                        if (!regs_var.ac_r && !regs_var.ac_w && !regs_var.ac_x) {
                            // Page fault detected, i.e. no memory request required
                            next_regs_var.state = BifCacheFillIdataPageFault;
                        } else {
                            next_regs_var.state = BifCacheReplaceReadIdata;
                        }
                    } else {
                        next_regs_var.phys_adr = ptw_phys_adr.read ();
                        next_regs_var.ac_r = ptw_ac_a && ptw_ac_r;
                        next_regs_var.ac_w = ptw_ac_a && ptw_ac_d && ptw_ac_w;
                        next_regs_var.ac_x = ptw_ac_a && ptw_ac_x;
                        next_regs_var.ac_u = ptw_ac_u;
                        next_regs_var.transl_done = ptw_ack;
                    }

                    break;
                case bioWriteback:
                case bioFlush:
                case bioWritebackAll:
                case bioFlushAll:
                    if (next_regs_var.tag_dirty)
                        next_regs_var.state = BifCacheReadDirtyBanks;
                    else
                        next_regs_var.state = BifCacheWriteTag;
                    break;
                default:
                    next_regs_var.tag_dirty = 0; // Set dirty to 0 for bioInvalidate and bioInvalidateAll command
                    next_regs_var.state = BifCacheWriteTag;
                    break;
            }
            break;

        case BifCacheReplaceReadIdata: // 7
            // Read (new) bus data into 'idata' register...
            // ... allowing to serve a read request as early as possible
            // Always select full words/double words
            wb_cyc_o = 1;
            wb_stb_o = 1;
            wb_sel_o = -1; // Always read full (double) words
            wb_adr_o = (regs_var.phys_adr(31, CFG_MEMU_CACHE_BANKS_LD+2), regs_var.adr_ofs, sc_uint<2>(0));
            if (BUSIF_DATA_REG_NUM >= 4 && BUSIF_DATA_REG_NUM <= 16) {
                // registered feedback bus cycle only for 4-16 banks
                if (regs_var.cnt == BUSIF_DATA_REG_NUM-1)
                    wb_cti_o = 0b111; // End-of-Burst
                else
                    wb_cti_o = 0b010; // Incrementing burst cycle
            }
            if (wb_ack_i) {
                // Select idata register based on adr_ofs_reg (for 64Bit width drop last bit)
                next_regs_var.idata[regs_var.adr_ofs(CFG_MEMU_CACHE_BANKS_LD-1, (CFG_MEMU_BUSIF_WIDTH/32)-1)] = wb_dat_i.read ();
                next_regs_var.idata_valid[regs_var.adr_ofs] = 1;
                #if CFG_MEMU_BUSIF_WIDTH == 64
                    next_regs_var.idata_valid[regs_var.adr_ofs + 1] = 1; // Validate 2 entrys per read
                #endif
                next_regs_var.phys_adr = (regs_var.phys_adr(31, CFG_MEMU_CACHE_BANKS_LD+2), regs_var.adr_ofs, sc_uint<2>(0));

                if (regs_var.cnt != BUSIF_DATA_REG_NUM-1) {
                    // Increment address offset and cnt...
                    // Note: Without the wrap mode cnt and adr_ofs are kind of redundant
                    next_regs_var.adr_ofs = regs_var.adr_ofs + (CFG_MEMU_BUSIF_WIDTH/32U);
                    next_regs_var.cnt = regs_var.cnt + 1;
                } else {
                    // Finished reading...
                    next_regs_var.cnt = 0;
                    if (regs_var.tag_dirty)
                        next_regs_var.state = BifCacheReadDirtyBanks;
                    else
                        next_regs_var.state = BifCacheReplaceInvalidateTag;
                }
            }
            break;

        case BifCacheReadDirtyBanks: // 8
            // Read (old) cache bank into 'odata' register (with maximum parallelism)
            // 'bank_rd' also selects the 'data_in' mux input for banks in
            // the memu so it has to be kept up in the cycle after
            // 'gnt_bank' when data is finally arriving
            for (uint n = 0; n < CFG_MEMU_CACHE_BANKS; n++) {
                req_bank[n] = regs_var.banks_left[n];
                bank_rd[n] = regs_var.last_banks_left[n];
            }

            // Write bank data to odata_reg...
            for (uint n = 0; n < CFG_MEMU_CACHE_BANKS; n++)
                if (regs_var.banks_left[n] == 0 && regs_var.last_banks_left[n] == 1)
                    next_regs_var.odata[n] = data_in[n].read (); // data comes from cache bank

            // Check for bank gnt...
            // bank data will arrive in the next cycle!
            for (uint n = 0; n < CFG_MEMU_CACHE_BANKS; n++)
                if (gnt_bank[n] == 1)
                    next_regs_var.banks_left[n] = 0;

            // Finished?...
            if (regs_var.banks_left == 0) {
                next_regs_var.banks_left = -1;
                if (regs_var.op == bioReplace)
                    next_regs_var.state = BifCacheReplaceInvalidateTag;
                else
                    next_regs_var.state = BifCacheWriteTag;
            }
            break;

        case BifCacheReplaceInvalidateTag: // 9
            // Write invalid tag (to make sure that concurrent readers do not receive invalid data)
            req_tagw = 1;
            tag_out_var.valid = 0;
            if (gnt_tagw) {
                tag_wr = 1;
                next_regs_var.state = BifCacheReplaceWriteBanks;
            }
            break;

        case BifCacheReplaceWriteBanks: // 10
            // Write data to cache banks; operate in parallel as much as possible
            for (uint n = 0; n < CFG_MEMU_CACHE_BANKS; n++) {
                req_bank[n] = regs_var.banks_left[n];
                bank_wr[n] = regs_var.banks_left[n];
                // Check for bank gnt...
                if (gnt_bank[n] == 1)
                    next_regs_var.banks_left[n] = 0;
            }

            // Finished?...
            if (regs_var.banks_left == 0) {
                next_regs_var.banks_left = -1;
                next_regs_var.state = BifCacheWriteTag;
            }
            break;

        case BifCacheWriteTag: // 11
            // Write new tag...
            req_tagw = 1;
                       
            if (/*!op_var[3] &*/ regs_var.op[1]){
                // Bit 1 is only set for bioInvalidate(All) and bioFlush(All) (and bioReplace,
                // but that is handled explicitly below)
                tag_out_var.valid = 0;
            }
            if (regs_var.op == bioReplace) {
                tag_out_var.valid = 1;
                tag_out_var.tadr = GetTagOfAdr(regs_var.virt_adr);
                tag_out_var.ac_r = regs_var.ac_r;
                tag_out_var.ac_w = regs_var.ac_w;
                tag_out_var.ac_x = regs_var.ac_x;
                tag_out_var.ac_u = regs_var.ac_u;
            }
            tag_out_var.dirty = 0;
            if (gnt_tagw) {
                if (!regs_var.op[3] && regs_var.op[1]) {
                    // Bit 3 is set for bioReplace and Bit 1 is only set for bioInvalidate(All) and bioFlush(All)
                    tag_wr = regs_var.tag_valid;
                } else {
                    tag_wr = 1;
                }
                if (regs_var.tag_dirty) {
                    if (!regs_var.paging_en) {
                        next_regs_var.phys_adr = ComposeAdress(regs_var.tag_tadr, GetIndexOfAdr (regs_var.virt_adr), 0, 0);
                        next_regs_var.state = BifCacheWriteBackVictim;
                    } else {
                        ptw_req = 1;
                        sc_uint<32> _ptw_virt_adr = ComposeAdress(regs_var.tag_tadr, GetIndexOfAdr (regs_var.virt_adr), 0, 0);
                        next_regs_var.phys_adr = _ptw_virt_adr;
                        ptw_virt_adr = _ptw_virt_adr;
                        next_regs_var.transl_done = ptw_ack;
                        if (ptw_ack) {
                            next_regs_var.state = BifCacheWriteBackVictim;
                        }
                    }
                } else {
                    // Reset valid signal to avoid wrong BusIF hits
                    for (uint n = 0; n < CFG_MEMU_CACHE_BANKS; ++n)
                        next_regs_var.idata_valid[n] = 0;
                    next_regs_var.state = regs_var.op[2] ? BifCacheAll : BifCacheAck;
                }
            }
            break;

        case BifCacheWriteBackVictim: // 12
            // Write back victim data to bus...
            wb_cyc_o = 1;
            wb_stb_o = 1;
            wb_we_o = 1;
            wb_sel_o = -1; // Always write full (double) words
            if (BUSIF_DATA_REG_NUM >= 4 && BUSIF_DATA_REG_NUM <= 16) {
                // registered feedback bus cycle only for 4-16 banks
                if (regs_var.cnt == BUSIF_DATA_REG_NUM-1)
                    wb_cti_o = 0b111; // End-of-Burst
                else
                    wb_cti_o = 0b010; // Incrementing burst cycle
            }
            wb_adr_o = ComposeAdress (GetTagOfAdr(regs_var.phys_adr), GetIndexOfAdr (regs_var.phys_adr), regs_var.cnt, 0);
            #if CFG_MEMU_BUSIF_WIDTH == 64
                wb_dat_o = (sc_uint<CFG_MEMU_BUSIF_WIDTH>)(regs_var.odata[(regs_var.cnt<<1)+1], regs_var.odata[regs_var.cnt<<1]);
            #else
                wb_dat_o = regs_var.odata[regs_var.cnt];
            #endif
            if (wb_ack_i) {
                if (regs_var.cnt != BUSIF_DATA_REG_NUM-1) {
                    // Increment cnt...
                    next_regs_var.cnt = regs_var.cnt + 1;
                } else {
                    // Finished writing...
                    next_regs_var.cnt = 0;
                    // Reset valid signal to avoid wrong BusIF hits
                    for (uint n = 0; n < CFG_MEMU_CACHE_BANKS; ++n)
                        next_regs_var.idata_valid[n] = 0;
                    next_regs_var.state = regs_var.op[2] ? BifCacheAll : BifCacheAck;
                }
            }
            break;

        case BifCacheAck: // 13
            busif_busy = 0;
            // Latch all inputs since the BUSIF requester may release the request after one clock cycle...
            next_regs_var.paging_en = paging_mode.read ();
            next_regs_var.transl_done = 0;
            next_regs_var.op = busif_op.read ();
            next_regs_var.bsel = busif_bsel.read ();
            next_regs_var.virt_adr = adr_in.read ();
            next_regs_var.phys_adr = adr_in.read ();
            // next_regs_var.adr_ofs = GetBankOfAdr (adr_in.read ());
            next_regs_var.adr_ofs = 0; // TBD: The optmization above was deactivated because of the missing wrap mode in the Wishbone/AXI-Bridge hardware
            next_regs_var.cnt = 0;
            next_regs_var.banks_left = -1;
            for (uint n = 0; n < CFG_MEMU_CACHE_BANKS; n++)
                next_regs_var.odata[n] = data_in[n].read (); // only needed for direct writes
            next_regs_var.linelock = 0;

            next_regs_var.ac_r = 1;
            next_regs_var.ac_w = 1;
            next_regs_var.ac_x = 1;
            next_regs_var.ac_u = 1;
            next_regs_var.state = BifIdle;
            break;

        case BifCacheAll: // 14
            // Loop over all cache sets...
            if (regs_var.tag_way == (1U << CFG_MEMU_CACHE_WAYS_LD) - 1) { 
                next_regs_var.virt_adr(CFG_MEMU_CACHE_SETS_LD + CFG_MEMU_CACHE_BANKS_LD + 2, CFG_MEMU_CACHE_BANKS_LD + 2) = regs_var.virt_adr(CFG_MEMU_CACHE_SETS_LD + CFG_MEMU_CACHE_BANKS_LD + 2, CFG_MEMU_CACHE_BANKS_LD + 2) + 1;
                next_regs_var.phys_adr(CFG_MEMU_CACHE_SETS_LD + CFG_MEMU_CACHE_BANKS_LD + 2, CFG_MEMU_CACHE_BANKS_LD + 2) = regs_var.phys_adr(CFG_MEMU_CACHE_SETS_LD + CFG_MEMU_CACHE_BANKS_LD + 2, CFG_MEMU_CACHE_BANKS_LD + 2) + 1;
                next_regs_var.tag_way = 0;
            // and loop over all cache ways...
            } else {  
                next_regs_var.tag_way = regs_var.tag_way + 1;
            }
            // Finished? ...
            if (regs_var.virt_adr(CFG_MEMU_CACHE_SETS_LD + CFG_MEMU_CACHE_BANKS_LD + 2, CFG_MEMU_CACHE_BANKS_LD + 2) == (1U << CFG_MEMU_CACHE_SETS_LD) -1 && regs_var.tag_way == (1U << CFG_MEMU_CACHE_WAYS_LD) - 1) {
                next_regs_var.state = BifCacheAck;
            } else
                next_regs_var.state = BifCacheRequestRTWait;
            break;

        case BifCacheFillIdataPageFault: // 15
            // Instead of reading (new) bus data into 'idata' register, fill them all with zero
            // ... allowing to serve a read request as early as possible
            for (uint n = 0; n < CFG_MEMU_CACHE_BANKS; ++n) {
                next_regs_var.idata[n] = 0;
                next_regs_var.idata_valid[n] = 1;
            }
            if (regs_var.tag_dirty)
                next_regs_var.state = BifCacheReadDirtyBanks;
            else
                next_regs_var.state = BifCacheReplaceInvalidateTag;
            break;
            
    } // switch (state_var)

    // Write results...
    next_regs = next_regs_var;
    tag_out = tag_out_var;
    state_trace = regs_var.state;

    #if CFG_MEMU_BUSIF_WIDTH == 64
        adr_out = regs_var.virt_adr & ~4;
        for (uint n = 0, j = 0; n < BUSIF_DATA_REG_NUM; n++, j+=2) {
    //        PN_INFOF(("Output: %d=0x%08x, %d=0x%08x", j, (uint32_t)regs_var.idata[n].range(31, 0), j+1, (uint32_t)regs_var.idata[n].range(63, 32) ));
            data_out[j] = (sc_uint<32>)regs_var.idata[n].range(31, 0);
            data_out[j+1] = (sc_uint<32>)regs_var.idata[n].range(63, 32);
    #else
        adr_out = regs_var.virt_adr;
        ac_r_out = regs_var.ac_r;
        ac_w_out = regs_var.ac_w;
        ac_x_out = regs_var.ac_x;
        ac_u_out = regs_var.ac_u;
        for (uint n = 0; n < BUSIF_DATA_REG_NUM; n++) {
            data_out[n] = (sc_uint<32>)regs_var.idata[n];
        }
    #endif
    
    for (uint n = 0; n < CFG_MEMU_CACHE_BANKS; n++) data_out_valid[n] = regs_var.idata_valid[n];

}