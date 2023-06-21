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


#include "memu.h"
#include "lfsr.h"
#include <elab_alloc.h>

// **************** MMemu ***********************
void MMemu::Trace (sc_trace_file *tf, int levels) {
    if (!tf || pn_trace_verbose) printf ("\nSignals of Module \"%s\":\n", name ());

    // Ports ...
    PN_TRACE (tf, clk);
    PN_TRACE (tf, reset);

    //   Bus interface (Wishbone)...
    PN_TRACE (tf, wb_cyc_o);
    PN_TRACE (tf, wb_stb_o);
    PN_TRACE (tf, wb_we_o);
    PN_TRACE (tf, wb_sel_o);
    PN_TRACE (tf, wb_ack_i);
    // PN_TRACE(tf, wb_err_i);
    // PN_TRACE(tf, wb_rty_i);
    PN_TRACE (tf, wb_adr_o);
    PN_TRACE (tf, wb_dat_i);
    PN_TRACE (tf, wb_dat_o);

    //   Read ports...
    PN_TRACE_BUS (tf, rp_rd, CFG_MEMU_RPORTS);
    PN_TRACE_BUS (tf, rp_direct, CFG_MEMU_RPORTS);
    PN_TRACE_BUS (tf, rp_ack, CFG_MEMU_RPORTS);
    PN_TRACE_BUS (tf, rp_bsel, CFG_MEMU_RPORTS);
    PN_TRACE_BUS (tf, rp_adr, CFG_MEMU_RPORTS);
    PN_TRACE_BUS (tf, rp_data, CFG_MEMU_RPORTS);

    //   Write ports...
    PN_TRACE_BUS (tf, wp_wr, CFG_MEMU_WPORTS);
    PN_TRACE_BUS (tf, wp_direct, CFG_MEMU_WPORTS);
    PN_TRACE_BUS (tf, wp_bsel, CFG_MEMU_WPORTS);
    PN_TRACE_BUS (tf, wp_ack, CFG_MEMU_WPORTS);
    PN_TRACE_BUS (tf, wp_lres_scond, CFG_MEMU_WPORTS);
    PN_TRACE_BUS (tf, wp_scond_ok, CFG_MEMU_WPORTS);
    PN_TRACE_BUS (tf, wp_cache_op, CFG_MEMU_WPORTS);
    PN_TRACE_BUS (tf, wp_adr, CFG_MEMU_WPORTS);
    PN_TRACE_BUS (tf, wp_data, CFG_MEMU_WPORTS);

    // Tag RAM...
    PN_TRACE (tf, tagram_ready);
    PN_TRACE_BUS (tf, tagram_rd, TR_PORTS);
    PN_TRACE_BUS (tf, tagram_wr, TR_PORTS);
    PN_TRACE_BUS (tf, tagram_adr, TR_PORTS);
    PN_TRACE_BUS (tf, tagram_tag_in, TR_PORTS);
    PN_TRACE_BUS (tf, tagram_tag_out, TR_PORTS);

    // Bank RAM...
    PN_TRACE_BUS_BUS (tf, bankram_rd, CFG_MEMU_CACHE_BANKS, CFG_MEMU_BANK_RAM_PORTS);
    PN_TRACE_BUS_BUS (tf, bankram_wr, CFG_MEMU_CACHE_BANKS, CFG_MEMU_BANK_RAM_PORTS);
    PN_TRACE_BUS_BUS (tf, bankram_wiadr, CFG_MEMU_CACHE_BANKS, CFG_MEMU_BANK_RAM_PORTS);
    PN_TRACE_BUS_BUS (tf, bankram_wdata, CFG_MEMU_CACHE_BANKS, CFG_MEMU_BANK_RAM_PORTS);
    PN_TRACE_BUS_BUS (tf, bankram_rdata, CFG_MEMU_CACHE_BANKS, CFG_MEMU_BANK_RAM_PORTS);

    // BUSIF...
    PN_TRACE (tf, busif_op);
    PN_TRACE (tf, busif_nolinelock);
    PN_TRACE (tf, busif_busy);
    PN_TRACE (tf, busif_tag_rd);
    PN_TRACE (tf, busif_tag_wr);
    PN_TRACE_BUS (tf, busif_bank_rd, CFG_MEMU_CACHE_BANKS);
    PN_TRACE_BUS (tf, busif_bank_wr, CFG_MEMU_CACHE_BANKS);
    PN_TRACE (tf, busif_adr_in);
    PN_TRACE (tf, busif_adr_out);
    PN_TRACE_BUS (tf, busif_data_in, CFG_MEMU_CACHE_BANKS);
    PN_TRACE_BUS (tf, busif_data_out, CFG_MEMU_CACHE_BANKS);
    PN_TRACE_BUS (tf, busif_data_out_valid, CFG_MEMU_CACHE_BANKS);
    PN_TRACE (tf, busif_tag_in);
    PN_TRACE (tf, busif_tag_out);
    PN_TRACE (tf, busif_bsel);

    // Read ports...
    PN_TRACE_BUS (tf, rp_busif_data_reg, CFG_MEMU_RPORTS);
    PN_TRACE_BUS (tf, rp_busif_data, CFG_MEMU_RPORTS);
    PN_TRACE_BUS (tf, rp_busif_op, CFG_MEMU_RPORTS);
    PN_TRACE_BUS (tf, rp_tag_rd, CFG_MEMU_RPORTS);
    PN_TRACE_BUS (tf, rp_bank_rd, CFG_MEMU_RPORTS);
    PN_TRACE_BUS (tf, rp_tag_in, CFG_MEMU_RPORTS);
    PN_TRACE_BUS (tf, rp_way_out, CFG_MEMU_RPORTS);
    PN_TRACE_BUS (tf, rp_bank_data_in, CFG_MEMU_RPORTS);
    PN_TRACE_BUS (tf, rp_bank_sel, CFG_MEMU_RPORTS);

    // Write ports...
    PN_TRACE_BUS (tf, wp_busif_op, CFG_MEMU_WPORTS);
    PN_TRACE_BUS (tf, wp_busif_nolinelock, CFG_MEMU_WPORTS);
    PN_TRACE_BUS (tf, wp_tag_rd, CFG_MEMU_WPORTS);
    PN_TRACE_BUS (tf, wp_tag_wr, CFG_MEMU_WPORTS);
    PN_TRACE_BUS (tf, wp_bank_rd, CFG_MEMU_WPORTS);
    PN_TRACE_BUS (tf, wp_bank_wr, CFG_MEMU_WPORTS);
    PN_TRACE_BUS (tf, wp_tag_in, CFG_MEMU_WPORTS);
    PN_TRACE_BUS (tf, wp_tag_out, CFG_MEMU_WPORTS);
    PN_TRACE_BUS (tf, wp_bank_data_in, CFG_MEMU_WPORTS);
    PN_TRACE_BUS (tf, wp_bank_data_out, CFG_MEMU_WPORTS);

    // Arbiter: request/grant signals (find comments in 'MArbiter')...
    PN_TRACE (tf, req_busif_linelock);
    PN_TRACE_BUS (tf, req_wp_linelock, CFG_MEMU_WPORTS);
    PN_TRACE (tf, gnt_busif_linelock);
    PN_TRACE_BUS (tf, gnt_wp_linelock, CFG_MEMU_WPORTS);

    PN_TRACE (tf, req_busif_tagw);
    PN_TRACE_BUS (tf, req_wp_tagw, CFG_MEMU_WPORTS);
    PN_TRACE (tf, req_busif_tagr);
    PN_TRACE_BUS (tf, req_wp_tagr, CFG_MEMU_WPORTS);
    PN_TRACE_BUS (tf, req_rp_tagr, CFG_MEMU_RPORTS);
    PN_TRACE (tf, gnt_busif_tagw);
    PN_TRACE_BUS (tf, gnt_wp_tagw, CFG_MEMU_WPORTS);
    PN_TRACE (tf, gnt_busif_tagr);
    PN_TRACE_BUS (tf, gnt_wp_tagr, CFG_MEMU_WPORTS);
    PN_TRACE_BUS (tf, gnt_rp_tagr, CFG_MEMU_RPORTS);

    PN_TRACE_BUS (tf, req_busif_bank, CFG_MEMU_CACHE_BANKS);
    PN_TRACE_BUS_BUS (tf, req_wp_bank, CFG_MEMU_WPORTS, CFG_MEMU_CACHE_BANKS);
    PN_TRACE_BUS_BUS (tf, req_rp_bank, CFG_MEMU_RPORTS, CFG_MEMU_CACHE_BANKS);
    PN_TRACE_BUS (tf, gnt_busif_bank, CFG_MEMU_CACHE_BANKS);
    PN_TRACE_BUS_BUS (tf, gnt_wp_bank, CFG_MEMU_WPORTS, CFG_MEMU_CACHE_BANKS);
    PN_TRACE_BUS_BUS (tf, gnt_rp_bank, CFG_MEMU_RPORTS, CFG_MEMU_CACHE_BANKS);

    PN_TRACE_BUS (tf, req_rp_busif, CFG_MEMU_RPORTS);
    PN_TRACE_BUS (tf, gnt_rp_busif, CFG_MEMU_RPORTS);
    PN_TRACE_BUS (tf, req_wp_busif, CFG_MEMU_WPORTS);
    PN_TRACE_BUS (tf, gnt_wp_busif, CFG_MEMU_WPORTS);

    // Arbiter: other signals ...
    PN_TRACE (tf, wiadr_busif);
    PN_TRACE_BUS (tf, wiadr_rp, CFG_MEMU_RPORTS);
    PN_TRACE_BUS (tf, adr_wp, CFG_MEMU_WPORTS);
    PN_TRACE_BUS (tf, way_wp, CFG_MEMU_WPORTS);
    PN_TRACE (tf, snoop_adr);
    PN_TRACE_BUS (tf, snoop_stb, CFG_MEMU_WPORTS);

    // Sub-Modules...
    //if (levels > 1) {
        levels--;
        bankRam->Trace (tf, levels);
        tagRam->Trace (tf, levels);
        busIf->Trace (tf, levels);
        busController->Trace (tf, levels);
        ptw->Trace (tf, levels);
        tlb->Trace (tf, levels);
        for (uint n = 0; n < CFG_MEMU_RPORTS; n++) readPorts[n].Trace (tf, levels);
        printf("\nTrace write port\n\n");
        for (uint n = 0; n < CFG_MEMU_WPORTS; n++) writePorts[n].Trace (tf, levels);
        arbiter->Trace (tf, levels);
        if (levels > 1){
            levels--;
            for (uint n = 0; n < CFG_MEMU_CACHE_BANKS; ++n) bankRam[n].Trace (tf, levels);
        }
    //}
}

void MMemu::InitSubmodules () {
    // set not connected signals
    NC_bool_in = 0;
    NC_uint32 = 0;

    // Tag RAM...
    tagRam = sc_new<MTagRam>("TagRAM");

    tagRam->clk (clk);
    tagRam->reset (reset);
    tagRam->ready (tagram_ready);
    tagRam->rd_way (busif_tag_rd_way);
    for (uint n = 0; n < TR_PORTS; n++) {
        tagRam->rd[n](tagram_rd[n]);
        tagRam->wr[n](tagram_wr[n]);
        tagRam->adr[n](tagram_adr[n]);
        tagRam->wadr[n] (tagram_wadr[n]);
        tagRam->tag_in[n](tagram_tag_in[n]);
        tagRam->tag_out[n](tagram_tag_out[n]);
    }

    
    // Bank RAM...
    bankRam = sc_new_array<MBankRam>(CFG_MEMU_CACHE_BANKS);
    for (uint n = 0; n < CFG_MEMU_CACHE_BANKS; n++) {
        bankRam[n].clk (clk);
        for (uint k = 0; k < CFG_MEMU_BANK_RAM_PORTS; k++) {
            bankRam[n].rd[k](bankram_rd[n][k]);
            bankRam[n].wr[k](bankram_wr[n][k]);
            bankRam[n].wen[k](bankram_wen[n][k]);
            bankRam[n].wiadr[k](bankram_wiadr[n][k]);
            bankRam[n].wdata[k](bankram_wdata[n][k]);
            bankRam[n].rdata[k](bankram_rdata[n][k]);
        }
    }


    // Bus Controller...
    busController = sc_new<MBusController>("BusController");
    for (unsigned int m = 0; m < MASTER_NO; m++) {
        busController->master_cyc[m] (master_cyc[m]);
        busController->master_stb[m] (master_stb[m]);
        busController->master_we[m] (master_we[m]);
        busController->master_sel[m] (master_sel[m]);
        busController->master_cti[m] (master_cti[m]);
        busController->master_bte[m] (master_bte[m]);
        busController->master_adr[m] (master_adr[m]);
        busController->master_dat[m] (master_dat[m]);
    }
    busController->switch_master (master_stb[1]); // switch to PTW when it requests the Bus

    busController->wb_cyc_o (wb_cyc_o);
    busController->wb_stb_o (wb_stb_o);
    busController->wb_we_o (wb_we_o);
    busController->wb_cti_o (wb_cti_o);
    busController->wb_bte_o (wb_bte_o);
    busController->wb_sel_o (wb_sel_o);
    busController->wb_adr_o (wb_adr_o);
    busController->wb_dat_o (wb_dat_o);

    // BUSIF...
    busIf = sc_new<MBusIf> ("BusIf");

    busIf->clk (clk);
    busIf->reset (reset);

    busIf->wb_cyc_o (master_cyc[0]);
    busIf->wb_stb_o (master_stb[0]);
    busIf->wb_we_o (master_we[0]);
    busIf->wb_sel_o (master_sel[0]);
    busIf->wb_cti_o (master_cti[0]);
    busIf->wb_bte_o (master_bte[0]);
    busIf->wb_adr_o (master_adr[0]);
    busIf->wb_dat_o (master_dat[0]);
    busIf->wb_dat_i (wb_dat_i);
    busIf->wb_ack_i (wb_ack_i);

    busIf->busif_op (busif_op);
    busIf->busif_nolinelock (busif_nolinelock);
    busIf->busif_bsel (busif_bsel);
    busIf->busif_busy (busif_busy);

    busIf->ac_r_out (busif_ac_r);
    busIf->ac_w_out (busif_ac_w);
    busIf->ac_x_out (busif_ac_x);
    busIf->ac_u_out (busif_ac_u);

    busIf->trap_u (busif_trap_u);
    busIf->trap_no_u (busif_trap_no_u);

    busIf->tag_rd (busif_tag_rd);
    busIf->tag_rd_way (busif_tag_rd_way);
    busIf->tag_wr (busif_tag_wr);
    for (uint n = 0; n < CFG_MEMU_CACHE_BANKS; n++) {
        busIf->bank_rd[n](busif_bank_rd[n]);
        busIf->bank_wr[n](busif_bank_wr[n]);
    }
    busIf->adr_in (busif_adr_in);
    busIf->adr_out (busif_adr_out);
    for (uint n = 0; n < CFG_MEMU_CACHE_BANKS; n++) {
        busIf->data_in[n](busif_data_in[n]);
        busIf->data_out[n](busif_data_out[n]);
        busIf->data_out_valid[n](busif_data_out_valid[n]);
    }
    busIf->tag_in (busif_tag_in);
    busIf->tag_out (busif_tag_out);

    busIf->req_linelock (req_busif_linelock);
    busIf->gnt_linelock (gnt_busif_linelock);
    busIf->req_tagw (req_busif_tagw);
    busIf->gnt_tagw (gnt_busif_tagw);
    busIf->req_tagr (req_busif_tagr);
    busIf->gnt_tagr (gnt_busif_tagr);
    for (uint n = 0; n < CFG_MEMU_CACHE_BANKS; n++) {
        busIf->req_bank[n](req_busif_bank[n]);
        busIf->gnt_bank[n](gnt_busif_bank[n]);
    }

    busIf->ptw_virt_adr(ptw_virt_adr);
    busIf->ptw_phys_adr(ptw_phys_adr);
    busIf->ptw_req(ptw_req);
    busIf->ptw_ack(ptw_ack);
    busIf->ptw_ac_r (ptw_ac_r);
    busIf->ptw_ac_w (ptw_ac_w);
    busIf->ptw_ac_x (ptw_ac_x);
    busIf->ptw_ac_u (ptw_ac_u);
    busIf->ptw_ac_d (ptw_ac_d);
    busIf->ptw_ac_a (ptw_ac_a);
    busIf->paging_mode(busif_paging);

    // Read ports...
    readPorts = sc_new_array<MReadPort> (CFG_MEMU_RPORTS);
    for (uint n = 0; n < CFG_MEMU_RPORTS; n++) {
        readPorts[n].clk (clk);
        readPorts[n].reset (reset);

        readPorts[n].port_rd (rp_rd[n]);
        readPorts[n].port_direct (rp_direct[n]);
        readPorts[n].port_ack (rp_ack[n]);
        readPorts[n].port_adr (rp_adr[n]);
        readPorts[n].port_data (rp_data[n]);
        readPorts[n].port_ac_r (rp_ac_r[n]);
        readPorts[n].port_ac_x (rp_ac_x[n]);
        readPorts[n].port_ac_u (rp_ac_u[n]);

        readPorts[n].busif_adr (busif_adr_out);
        readPorts[n].busif_data (rp_busif_data[n]);
        for (uint b = 0; b < CFG_MEMU_CACHE_BANKS; b++) readPorts[n].busif_data_valid[b](busif_data_out_valid[b]);
        readPorts[n].busif_op (rp_busif_op[n]);
        readPorts[n].busif_busy (busif_busy);
        readPorts[n].busif_ac_r (busif_ac_r);
        readPorts[n].busif_ac_x (busif_ac_x);
        readPorts[n].busif_ac_u (busif_ac_u);

        readPorts[n].tag_rd (rp_tag_rd[n]);
        readPorts[n].tag_in (rp_tag_in[n]);
        readPorts[n].way_out (rp_way_out[n]);
        readPorts[n].bank_rd (rp_bank_rd[n]);
        readPorts[n].bank_data_in (rp_bank_data_in[n]);
        readPorts[n].bank_sel (rp_bank_sel[n]);

        readPorts[n].req_tagr (req_rp_tagr[n]);
        readPorts[n].req_busif (req_rp_busif[n]);
        for (uint k = 0; k < CFG_MEMU_CACHE_BANKS; k++) readPorts[n].req_bank[k](req_rp_bank[n][k]);
        readPorts[n].gnt_tagr (gnt_rp_tagr[n]);
        readPorts[n].gnt_busif (gnt_rp_busif[n]);
        for (uint k = 0; k < CFG_MEMU_CACHE_BANKS; k++) readPorts[n].gnt_bank[k](gnt_rp_bank[n][k]);

        if (n < CFG_MEMU_WPORTS) {
            // Route LR/SC signals (only needed for WP acompanied RPs
            readPorts[n].port_scond_ok (wp_scond_ok[n]);
            readPorts[n].snoop_adr (snoop_adr);
            readPorts[n].snoop_stb (snoop_stb[n]);
            readPorts[n].port_lres_scond (wp_lres_scond[n]);
        } else {
            readPorts[n].port_scond_ok (NC_bool_out[n]);
            readPorts[n].snoop_adr (NC_uint32);
            readPorts[n].snoop_stb (NC_bool_in);
            readPorts[n].port_lres_scond (NC_bool_in);
        }
    }

    // Write ports...
    writePorts = sc_new_array<MWritePort> (CFG_MEMU_WPORTS);
    for (uint n = 0; n < CFG_MEMU_WPORTS; n++) {
        

        writePorts[n].clk (clk);
        writePorts[n].reset (reset);

        writePorts[n].port_wr (wp_wr[n]);
        writePorts[n].port_direct (wp_direct[n]);
        writePorts[n].port_bsel (wp_bsel[n]);
        writePorts[n].port_ack (wp_ack[n]);
        writePorts[n].port_lres_scond (wp_lres_scond[n]);
        writePorts[n].port_cache_op (wp_cache_op[n]); 
        writePorts[n].port_adr (wp_adr[n]);
        writePorts[n].port_data (wp_data[n]);
        writePorts[n].port_scond_ok (wp_scond_ok[n]);
        writePorts[n].port_ac_w (wp_ac_w[n]);
        writePorts[n].port_trap_u (wp_trap_u[n]);
        writePorts[n].port_trap_no_u (wp_trap_no_u[n]);

        writePorts[n].busif_adr (busif_adr_out);
        writePorts[n].busif_op (wp_busif_op[n]);
        writePorts[n].busif_nolinelock (wp_busif_nolinelock[n]);
        writePorts[n].busif_busy (busif_busy);
        writePorts[n].busif_ac_w (busif_ac_w);

        writePorts[n].tag_rd (wp_tag_rd[n]);
        writePorts[n].tag_wr (wp_tag_wr[n]);
        writePorts[n].tag_in (wp_tag_in[n]);
        writePorts[n].tag_out (wp_tag_out[n]);
        writePorts[n].bank_rd (wp_bank_rd[n]);
        writePorts[n].bank_wr (wp_bank_wr[n]);
        writePorts[n].bank_data_in (wp_bank_data_in[n]);
        writePorts[n].bank_data_out (wp_bank_data_out[n]);
        writePorts[n].bank_bsel (wp_bank_bsel[n]);

        writePorts[n].req_linelock (req_wp_linelock[n]);
        writePorts[n].req_tagr (req_wp_tagr[n]);
        writePorts[n].req_tagw (req_wp_tagw[n]);
        writePorts[n].req_busif (req_wp_busif[n]);
        for (uint k = 0; k < CFG_MEMU_CACHE_BANKS; k++) writePorts[n].req_bank[k](req_wp_bank[n][k]);
        writePorts[n].gnt_linelock (gnt_wp_linelock[n]);
        writePorts[n].gnt_tagr (gnt_wp_tagr[n]);
        writePorts[n].gnt_tagw (gnt_wp_tagw[n]);
        writePorts[n].gnt_busif (gnt_wp_busif[n]);
        for (uint k = 0; k < CFG_MEMU_CACHE_BANKS; k++) writePorts[n].gnt_bank[k](gnt_wp_bank[n][k]);
    }

    // Arbiter...
    arbiter = sc_new<MArbiter>("Arbiter");

    arbiter->clk (clk);
    arbiter->reset (reset);

    arbiter->tagram_ready (tagram_ready);
    arbiter->wiadr_busif (wiadr_busif);
    arbiter->req_busif_linelock (req_busif_linelock);
    arbiter->gnt_busif_linelock (gnt_busif_linelock);
    arbiter->req_busif_tagw (req_busif_tagw);
    arbiter->req_busif_tagr (req_busif_tagr);
    arbiter->gnt_busif_tagw (gnt_busif_tagw);
    arbiter->gnt_busif_tagr (gnt_busif_tagr);

    arbiter->gnt_busif_tagw_r (gnt_busif_tagw_r);
    arbiter->gnt_busif_tagr_r (gnt_busif_tagr_r);

    arbiter->snoop_adr (snoop_adr);

    for (uint n = 0; n < CFG_MEMU_RPORTS; n++) {
        arbiter->wiadr_rp[n](wiadr_rp[n]);
        arbiter->req_rp_tagr[n](req_rp_tagr[n]);
        arbiter->gnt_rp_tagr[n](gnt_rp_tagr[n]);
        arbiter->req_rp_busif[n](req_rp_busif[n]);
        arbiter->gnt_rp_busif[n](gnt_rp_busif[n]);

        arbiter->gnt_rp_tagr_r[n](gnt_rp_tagr_r[n]);
    }

    for (uint n = 0; n < CFG_MEMU_WPORTS; n++) {
        arbiter->adr_wp[n](adr_wp[n]);
        arbiter->way_wp[n](way_wp[n]);
        arbiter->req_wp_linelock[n](req_wp_linelock[n]);
        arbiter->gnt_wp_linelock[n](gnt_wp_linelock[n]);
        arbiter->req_wp_tagw[n](req_wp_tagw[n]);
        arbiter->req_wp_tagr[n](req_wp_tagr[n]);
        arbiter->gnt_wp_tagw[n](gnt_wp_tagw[n]);
        arbiter->gnt_wp_tagr[n](gnt_wp_tagr[n]);
        arbiter->req_wp_busif[n](req_wp_busif[n]);
        arbiter->gnt_wp_busif[n](gnt_wp_busif[n]);
        arbiter->snoop_stb[n](snoop_stb[n]);

        arbiter->gnt_wp_tagw_r[n](gnt_wp_tagw_r[n]);
        arbiter->gnt_wp_tagr_r[n](gnt_wp_tagr_r[n]);
    }
    for (uint b = 0; b < CFG_MEMU_CACHE_BANKS; b++) {
        arbiter->req_busif_bank[b](req_busif_bank[b]);
        arbiter->gnt_busif_bank[b](gnt_busif_bank[b]);
        for (uint n = 0; n < CFG_MEMU_RPORTS; n++) {
            arbiter->req_rp_bank[n][b](req_rp_bank[n][b]);
            arbiter->gnt_rp_bank[n][b](gnt_rp_bank[n][b]);
        }
        for (uint n = 0; n < CFG_MEMU_WPORTS; n++) {
            arbiter->req_wp_bank[n][b](req_wp_bank[n][b]);
            arbiter->gnt_wp_bank[n][b](gnt_wp_bank[n][b]);
        }
        for (uint n = 0; n < CFG_MEMU_BANK_RAM_PORTS; n++) arbiter->wiadr_bank[b][n](bankram_wiadr[b][n]);
    }

    ptw = sc_new<MPtw>("PTW");

    ptw->clk(clk);
    ptw->reset(reset);

    ptw->busif_virt_adr (ptw_virt_adr);
    ptw->busif_phys_adr (ptw_phys_adr);
    ptw->busif_req (ptw_req);
    ptw->busif_ack (ptw_ack);
    ptw->busif_ac_r (ptw_ac_r);
    ptw->busif_ac_w (ptw_ac_w);
    ptw->busif_ac_x (ptw_ac_x);
    ptw->busif_ac_u (ptw_ac_u);
    ptw->busif_ac_d (ptw_ac_d);
    ptw->busif_ac_a (ptw_ac_a);

    ptw->memu_root_ppn (root_ppn);

    ptw->tlb_req (tlb_req);
    ptw->tlb_wr (tlb_wr);
    ptw->tlb_va_o (tlb_va_o);
    ptw->tlb_pa_o (tlb_pa_o);
    ptw->tlb_superpage_o (tlb_superpage_o);
    ptw->tlb_ac_r_o (tlb_ac_r_o);
    ptw->tlb_ac_w_o (tlb_ac_w_o);
    ptw->tlb_ac_x_o (tlb_ac_x_o);
    ptw->tlb_ac_u_o (tlb_ac_u_o);
    ptw->tlb_ac_d_o (tlb_ac_d_o);
    ptw->tlb_ac_a_o (tlb_ac_a_o);

    ptw->tlb_superpage_i (tlb_superpage_i);
    ptw->tlb_adr_i (tlb_adr_i);
    ptw->tlb_hit (tlb_hit);
    ptw->tlb_miss (tlb_miss);

    ptw->tlb_ac_r_i (tlb_ac_r_i);
    ptw->tlb_ac_w_i (tlb_ac_w_i);
    ptw->tlb_ac_x_i (tlb_ac_x_i);
    ptw->tlb_ac_u_i (tlb_ac_u_i);
    ptw->tlb_ac_d_i (tlb_ac_d_i);
    ptw->tlb_ac_a_i (tlb_ac_a_i);

    ptw->wb_cyc_o (master_cyc[1]);
    ptw->wb_stb_o (master_stb[1]);
    ptw->wb_we_o (master_we[1]);
    ptw->wb_sel_o (master_sel[1]);
    ptw->wb_cti_o (master_cti[1]);
    ptw->wb_bte_o (master_bte[1]);
    ptw->wb_adr_o (master_adr[1]);
    ptw->wb_dat_o (master_dat[1]);
    ptw->wb_dat_i (wb_dat_i);
    ptw->wb_ack_i (wb_ack_i);


    tlb = sc_new<MTlb>("TLB");

    tlb->clk(clk);
    tlb->reset(reset);
    tlb->ptw_req (tlb_req);
    tlb->ptw_wr (tlb_wr);
    tlb->ptw_va_i (tlb_va_o);
    tlb->ptw_pa_i (tlb_pa_o);
    tlb->ptw_superpage_i (tlb_superpage_o);
    tlb->ptw_ac_r_i (tlb_ac_r_o);
    tlb->ptw_ac_w_i (tlb_ac_w_o);
    tlb->ptw_ac_x_i (tlb_ac_x_o);
    tlb->ptw_ac_u_i (tlb_ac_u_o);
    tlb->ptw_ac_d_i (tlb_ac_d_o);
    tlb->ptw_ac_a_i (tlb_ac_a_o);

    tlb->ptw_superpage_o (tlb_superpage_i);
    tlb->ptw_adr_o (tlb_adr_i);
    tlb->ptw_hit_o (tlb_hit);
    tlb->ptw_miss_o (tlb_miss);

    tlb->ptw_ac_r_o (tlb_ac_r_i);
    tlb->ptw_ac_w_o (tlb_ac_w_i);
    tlb->ptw_ac_x_o (tlb_ac_x_i);
    tlb->ptw_ac_u_o (tlb_ac_u_i);
    tlb->ptw_ac_d_o (tlb_ac_d_i);
    tlb->ptw_ac_a_o (tlb_ac_a_i);

    tlb->flush (tlb_flush);

}


void MMemu::FreeSubmodules () {

}

void MMemu::proc_clk_memu () {
    wait();
    while(1){
        for (uint n = 0; n < CFG_MEMU_BUSIF_WIDTH/32; n++)
            rp_busif_data_reg[n] = busif_data_out[GetBankOfAdr (busif_adr_out.read () + n*4)]; 
        wait();
    }
    
}


void MMemu::proc_cmb_interconnect () {
    // To Tag RAM...
    // FELIX: Document this
    for (uint n = 0; n < CFG_NUT_CPU_CORES; n++) {
        bool tagram_rd_var, tagram_wr_var;
        uint32_t tagram_adr_var, tagram_wadr_var;
        SCacheTag tagram_tag_in_var;
        tagram_rd_var = (n == CFG_NUT_CPU_CORES - 1 ? busif_tag_rd.read () : false) || rp_tag_rd[n].read () ||
                       rp_tag_rd[CFG_NUT_CPU_CORES + n].read () || wp_tag_rd[n].read ();
        tagram_wr_var = (n == CFG_NUT_CPU_CORES - 1 ? busif_tag_wr.read () : false) || wp_tag_wr[n].read ();
        tagram_adr_var = n == CFG_NUT_CPU_CORES - 1 && gnt_busif_tagr.read () ?
                        busif_adr_out.read () :
                        gnt_wp_tagr[n].read () ?
                        wp_adr[n].read () :
                        gnt_rp_tagr[n].read () ?
                        rp_adr[n].read () : 
                        gnt_rp_tagr[CFG_NUT_CPU_CORES + n].read () ? rp_adr[CFG_NUT_CPU_CORES + n].read () : (sc_uint<32>) 0xffffffff; // don't care
        tagram_wadr_var = n == CFG_NUT_CPU_CORES - 1 && gnt_busif_tagw.read () ? busif_adr_out.read () : wp_adr[n].read ();
        // tagram_tag_in_var
        if(n == CFG_NUT_CPU_CORES - 1){
            if(gnt_wp_tagw[n].read ()){
                tagram_tag_in_var = wp_tag_out[n].read ();
            }else{
                tagram_tag_in_var = busif_tag_out.read ();
            } 
        }else{
            tagram_tag_in_var = wp_tag_out[n].read ();
        }
    
        tagram_rd[n] = tagram_rd_var;
        tagram_wr[n] = tagram_wr_var;
        tagram_adr[n] = tagram_adr_var;
        tagram_wadr[n] = tagram_wadr_var;
        tagram_tag_in[n] = tagram_tag_in_var;
    }

     // To Bank RAM...
    for (uint b = 0; b < CFG_MEMU_CACHE_BANKS; b++) {
        for (uint p = 0; p < CFG_MEMU_BANK_RAM_PORTS; p++) {
            // defaults...
            bankram_rd[b][p] = 0;
            bankram_wr[b][p] = 0;
            bankram_wen[b][p] = 0xf; // don't care
            // bankram_wiadr[b][p] = 0xffffffff;     // don't care
            bankram_wdata[b][p] = 0xffffffff; // don't care
            // find CPU...
            for (uint cpu = p; cpu < CFG_NUT_CPU_CORES; cpu += CFG_MEMU_BANK_RAM_PORTS) {
                if (gnt_rp_bank[cpu][b] == 1) {
                    if (rp_bank_rd[cpu] == 1) bankram_rd[b][p] = 1;
                }
                if (gnt_rp_bank[CFG_NUT_CPU_CORES + cpu][b] == 1) {
                    if (rp_bank_rd[CFG_NUT_CPU_CORES + cpu] == 1) bankram_rd[b][p] = 1;
                }
                if (gnt_wp_bank[cpu][b] == 1) {
                    if (wp_bank_rd[cpu] == 1) bankram_rd[b][p] = 1;
                    if (wp_bank_wr[cpu] == 1){
                        bankram_wr[b][p] = 1;

                    } 
                    bankram_wen[b][p] = wp_bank_bsel[cpu];
                    bankram_wdata[b][p] = wp_bank_data_out[cpu];
                }
            }
        }
        // eventually link BUSIF to last port...
        if (gnt_busif_bank[b] == 1) {
            if (busif_bank_rd[b] == 1) bankram_rd[b][CFG_MEMU_BANK_RAM_PORTS - 1] = 1;
            if (busif_bank_wr[b] == 1) bankram_wr[b][CFG_MEMU_BANK_RAM_PORTS - 1] = 1;
            bankram_wdata[b][CFG_MEMU_BANK_RAM_PORTS - 1] = busif_data_out[b];
            bankram_wen[b][CFG_MEMU_BANK_RAM_PORTS - 1] = 0b1111;
        }
    }

    // To BUSIF...
    //   defaults ...
    busif_op = bioNothing;
    busif_nolinelock = 0;
    sc_uint<32> busif_adr_in_var = 0;
    busif_bsel = 0;
    busif_trap_u = 0;
    busif_trap_no_u = 0;
    busif_paging = 0;
    //   from tag RAMs...
    busif_tag_in = tagram_tag_out[CFG_NUT_CPU_CORES - 1];
    //   from bank RAMs...
    for (uint b = 0; b < CFG_MEMU_CACHE_BANKS; b++) busif_data_in[b] = bankram_rdata[b][CFG_MEMU_BANK_RAM_PORTS - 1];
    //   from read ports...
    for (uint p = 0; p < CFG_MEMU_RPORTS; p++)
        if (gnt_rp_busif[p]) {
            busif_op = rp_busif_op[p];
            busif_nolinelock = 0;
            busif_adr_in_var = rp_adr[p].read ();
            busif_bsel = rp_bsel[p];
            busif_paging = rp_paging[p];
        }
    //   from write ports...
    for (uint p = 0; p < CFG_MEMU_WPORTS; p++)
        if (gnt_wp_busif[p]) {
            busif_op = wp_busif_op[p];
            busif_nolinelock = wp_busif_nolinelock[p];
            busif_adr_in_var = wp_adr[p].read ();
            for (uint b = 0; b < CFG_MEMU_CACHE_BANKS; b++)
                if (busif_bank_rd[b] == 0) busif_data_in[b] = wp_data[p];
            busif_bsel = wp_bsel[p];
            busif_trap_u = wp_trap_u[p];
            busif_trap_no_u = wp_trap_no_u[p];
            busif_paging = wp_paging[p];
        }
    busif_adr_in = busif_adr_in_var;


    // To read ports...
    for (uint p = 0; p < CFG_MEMU_RPORTS; p++) {
        uint cpu = p % CFG_NUT_CPU_CORES;
        rp_tag_in[p] = tagram_tag_out[cpu];
        rp_bank_data_in[p] = bankram_rdata[rp_bank_sel[p].read ()][cpu % CFG_MEMU_BANK_RAM_PORTS];
#if CFG_MEMU_BUSIF_WIDTH == 64
        // Either top or bottom 32bit
        rp_busif_data[p] = rp_busif_data_reg[(rp_adr[p].read() & 4) == 4]; // busif_data_out[GetBankOfAdr (busif_adr_out)];//rp_busif_data_reg; //
#else
        // All get the same busif_data_reg
        rp_busif_data[p] = rp_busif_data_reg[0];
#endif
    }

    // To write ports...
    for (uint p = 0; p < CFG_MEMU_WPORTS; p++) {
        uint cpu = p;
        wp_tag_in[p] = tagram_tag_out[cpu];
        wp_bank_data_in[p] = bankram_rdata[GetBankOfAdr (wp_adr[p])][cpu % CFG_MEMU_BANK_RAM_PORTS];
    }

    // To arbiter ...
    wiadr_busif = GetWayIndexOfAdr (busif_adr_out, busif_tag_out.read ().way);
    for (uint p = 0; p < CFG_MEMU_RPORTS; p++) wiadr_rp[p] = GetWayIndexOfAdr (rp_adr[p], rp_way_out[p]);
    for (uint p = 0; p < CFG_MEMU_WPORTS; p++) {
        adr_wp[p] = wp_adr[p];
        way_wp[p] = wp_tag_out[p].read ().way;
    }
}

