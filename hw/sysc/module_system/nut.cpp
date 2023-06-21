/*************************************************************************

  This file is part of the ParaNut project.

  Copyright (C) 2010-2022 Alexander Bahle <alexander.bahle@hs-augsburg.de>
                          Gundolf Kiefer <gundolf.kiefer@hs-augsburg.de>
                          Christian H. Meyer <christian.meyer@hs-augsburg.de>
                          Nico Borgsmüller <nico.borgsmueller@hs-augsburg.de>
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


#include "nut.h"
#include "elab_alloc.h"

// **************** Helper Functions *********************************



// **************** Trace *********************************

void MParanut::Trace (sc_trace_file *tf, int levels) {
    if (!tf || pn_trace_verbose) printf ("\nSignals of Module \"%s\":\n", name ());

    // Ports ...
    PN_TRACE (tf, clk_i);
    PN_TRACE (tf, rst_i);

    PN_TRACE (tf, cyc_o);
    PN_TRACE (tf, stb_o);
    PN_TRACE (tf, we_o);
    PN_TRACE (tf, sel_o);
    PN_TRACE (tf, cti_o);
    PN_TRACE (tf, bte_o);
    PN_TRACE (tf, ack_i);
    PN_TRACE (tf, wb_ack);
    PN_TRACE (tf, wb_dat);

    PN_TRACE (tf, adr_o);
    PN_TRACE (tf, dat_i);
    PN_TRACE (tf, dat_o);

    PN_TRACE_BUS (tf, ex_int, CFG_NUT_EX_INT);

    // Connecting signals...
    //   MEMU ...
    PN_TRACE_BUS (tf, rp_rd, 2 * CFG_NUT_CPU_CORES);
    PN_TRACE_BUS (tf, rp_direct, 2 * CFG_NUT_CPU_CORES);
    PN_TRACE_BUS (tf, rp_bsel, 2 * CFG_NUT_CPU_CORES);
    PN_TRACE_BUS (tf, rp_ack, 2 * CFG_NUT_CPU_CORES);
    PN_TRACE_BUS (tf, rp_adr, 2 * CFG_NUT_CPU_CORES);
    PN_TRACE_BUS (tf, rp_data, 2 * CFG_NUT_CPU_CORES);
    PN_TRACE_BUS (tf, wp_wr, CFG_NUT_CPU_CORES);
    PN_TRACE_BUS (tf, wp_direct, CFG_NUT_CPU_CORES);
    PN_TRACE_BUS (tf, wp_bsel, CFG_NUT_CPU_CORES);
    PN_TRACE_BUS (tf, wp_ack, CFG_NUT_CPU_CORES);
    PN_TRACE_BUS (tf, wp_lres_scond, CFG_NUT_CPU_CORES);
    PN_TRACE_BUS (tf, wp_scond_ok, CFG_NUT_CPU_CORES);
    PN_TRACE_BUS (tf, wp_cache_op, CFG_NUT_CPU_CORES);
    PN_TRACE_BUS (tf, wp_adr, CFG_NUT_CPU_CORES);
    PN_TRACE_BUS (tf, wp_data, CFG_NUT_CPU_CORES);
    //   IFU <-> EXU ...
    PN_TRACE_BUS (tf, ifu_next, CFG_NUT_CPU_CAP2_CORES);
    PN_TRACE_BUS (tf, ifu_jump, CFG_NUT_CPU_CAP2_CORES);
    PN_TRACE_BUS (tf, ifu_jump_adr, CFG_NUT_CPU_CAP2_CORES);
    PN_TRACE_BUS (tf, ifu_ir_valid, CFG_NUT_CPU_CAP2_CORES);
    PN_TRACE_BUS (tf, ifu_npc_valid, CFG_NUT_CPU_CAP2_CORES);
    PN_TRACE_BUS (tf, ifu_ir, CFG_NUT_CPU_CAP2_CORES);
    PN_TRACE_BUS (tf, ifu_pc, CFG_NUT_CPU_CAP2_CORES);
    PN_TRACE_BUS (tf, ifu_npc, CFG_NUT_CPU_CAP2_CORES);
    //   LSU <-> EXU ...
    PN_TRACE_BUS (tf, lsu_rd, CFG_NUT_CPU_CORES);
    PN_TRACE_BUS (tf, lsu_wr, CFG_NUT_CPU_CORES);
    PN_TRACE_BUS (tf, lsu_flush, CFG_NUT_CPU_CORES);
    PN_TRACE_BUS (tf, lsu_lres_scond, CFG_NUT_CPU_CORES);
    PN_TRACE_BUS (tf, lsu_cache_op, CFG_NUT_CPU_CORES);
    PN_TRACE_BUS (tf, lsu_ack, CFG_NUT_CPU_CORES);
    PN_TRACE_BUS (tf, lsu_align_err, CFG_NUT_CPU_CORES);
    PN_TRACE_BUS (tf, lsu_scond_ok, CFG_NUT_CPU_CORES);
    PN_TRACE_BUS (tf, lsu_width, CFG_NUT_CPU_CORES);
    PN_TRACE_BUS (tf, lsu_exts, CFG_NUT_CPU_CORES);
    PN_TRACE_BUS (tf, lsu_adr, CFG_NUT_CPU_CORES);
    PN_TRACE_BUS (tf, lsu_rdata, CFG_NUT_CPU_CORES);
    PN_TRACE_BUS (tf, lsu_wdata, CFG_NUT_CPU_CORES);
    //   other signals...
    PN_TRACE (tf, m3_icache_enable);
    PN_TRACE (tf, m3_dcache_enable);

    // Sub-Modules...
    if (levels > 1) {
        levels--;
        memu->Trace (tf, levels);
        dm->Trace (tf, levels);
        dtm->Trace (tf, levels);
        intc->Trace (tf, levels);
        mtimer->Trace (tf, levels);
        for (int n = 0; n < CFG_NUT_CPU_CAP2_CORES; n++) {
            ifu[n].Trace (tf, levels);
        }
        for (int n = 0; n < CFG_NUT_CPU_CORES; n++) {
            lsu[n].Trace (tf, levels);
            exu[n].Trace (tf, levels);
            csr[n].Trace (tf, levels);
        }
    }
}


// **************** Submodules ****************************

void MParanut::InitSubmodules () {
    char name[80];
    int n;

    // DBGU...
    dm = sc_new<MDebugModule>("DM");

    dm->clk_i (clk_i); // clock input
    dm->rst_i (rst_i); // reset

    dm->stb_i (stb_o); // strobe input
    dm->cyc_i (cyc_o); // cycle valid input
    dm->we_i (we_o); // indicates write transfer
    dm->sel_i (sel_o); // byte select inputs
    dm->ack_o (dbg_ack_i); // normal termination
    dm->err_o (vh_open); // termination w/ error
    dm->rty_o (vh_open); // termination w/ retry

    dm->adr_i (adr_o); // address bus inputs
    dm->dat_i (dat_o); // input data bus
    dm->dat_o (dbg_dat_i); // output data bus

    dm->dbg_request (dbg_req);
    dm->dbg_reset (dbg_reset);
    // DMI
    dm->dmi_adr_i (dmi_adr);
    dm->dmi_dat_i (dmi_dat_o);
    dm->dmi_dat_o (dmi_dat_i);
    dm->dmi_rd (dmi_rd);
    dm->dmi_wr (dmi_wr);


    // DTM...
    dtm = sc_new<MDtm>("DTM");

    dtm->reset (rst_i); // reset
    // DMI
    dtm->dmi_adr (dmi_adr);
    dtm->dmi_dat_o (dmi_dat_o);
    dtm->dmi_dat_i (dmi_dat_i);
    dtm->dmi_rd (dmi_rd);
    dtm->dmi_wr (dmi_wr);
    // JTAG
    dtm->tck (tck);
    dtm->tms (tms);
    dtm->tdi (tdi);
    dtm->tdo (tdo);


    // INTC...
    intc = sc_new<MIntC>("IntC");

    intc->clk (clk_i); // clock input
    intc->reset (reset); // reset

    intc->ex_int (intc_ex_int);
    intc->mtimer_int(mtimer_ir_request);

    intc->ir_request (m3_ir_request);   
    intc->ir_id (m3_ir_id);             
    intc->ir_ack (m3_ir_ack);
    intc->ir_enable (m3_ir_enable);
    intc->mip_mtip_out(csr_mip_MTIP);
    intc->mip_meip_out(csr_mip_MEIP);

    // MEMU...
    memu = sc_new<MMemu>("MemU");

    memu->clk (clk_i);
    memu->reset (reset);

    //   bus interface (Wishbone)...
    memu->wb_cyc_o (cyc_o);
    memu->wb_stb_o (stb_o);
    memu->wb_we_o (we_o);
    memu->wb_sel_o (sel_o);
    memu->wb_cti_o (cti_o);
    memu->wb_bte_o (bte_o);
    memu->wb_adr_o (adr_o);
    memu->wb_dat_o (dat_o);
    memu->wb_ack_i (wb_ack);
    memu->wb_dat_i (wb_dat);

    //    read ports...
    for (n = 0; n < CFG_MEMU_RPORTS - CFG_NUT_CPU_CAP1_CORES; n++) {
        memu->rp_rd[n](rp_rd[n]);
        memu->rp_direct[n](rp_direct[n]);
        memu->rp_bsel[n](rp_bsel[n]);
        memu->rp_ack[n](rp_ack[n]);
        memu->rp_adr[n](rp_adr[n]);
        memu->rp_data[n](rp_data[n]);
        memu->rp_ac_r[n](rp_ac_r[n]);
        memu->rp_ac_x[n](rp_ac_x[n]);
        memu->rp_ac_u[n](rp_ac_u[n]);
        memu->rp_paging[n](rp_paging[n]);
    }

    //    unused read ports (Mode 1 only CoPUs)...
    for (; n < CFG_MEMU_RPORTS; n++) {
        memu->rp_rd[n](vh_const<bool> (0));
        memu->rp_direct[n](vh_const<bool> (0));
        memu->rp_bsel[n](vh_const<sc_uint<4>> (0));
        memu->rp_ack[n](vh_open);
        memu->rp_adr[n](vh_const<sc_uint<32> > (n));
        memu->rp_data[n](vh_open);
        memu->rp_ac_r[n](vh_open);
        memu->rp_ac_x[n](vh_open);
        memu->rp_ac_u[n](vh_open);
        memu->rp_paging[n](vh_const<bool> (0));
    }

    //   write ports...
    for (n = 0; n < CFG_MEMU_WPORTS; n++) {
        memu->wp_wr[n](wp_wr[n]);
        memu->wp_direct[n](wp_direct[n]);
        memu->wp_bsel[n](wp_bsel[n]);
        memu->wp_ack[n](wp_ack[n]);
        memu->wp_lres_scond[n](wp_lres_scond[n]);
        memu->wp_scond_ok[n](wp_scond_ok[n]);
        memu->wp_cache_op[n](wp_cache_op[n]);
        memu->wp_adr[n](wp_adr[n]);
        memu->wp_data[n](wp_data[n]);
        memu->wp_ac_w[n](wp_ac_w[n]);
        memu->wp_trap_u[n] (wp_trap_u[n]);
        memu->wp_trap_no_u[n] (wp_trap_no_u[n]);
        memu->wp_paging[n](wp_paging[n]);
    }

    memu->root_ppn (root_ppn);
    memu->tlb_flush (tlb_flush);

    // IFUs...
    ifu = sc_new_array<MIfu>(CFG_NUT_CPU_CAP2_CORES);
    for (n = 0; n < CFG_NUT_CPU_CAP2_CORES; n++) {
        sprintf (name, "IFU%i", n);
        

        ifu[n].clk (clk_i);
        ifu[n].reset (ifu_reset[n]);
        //   to MEMU (read port)...
        ifu[n].rp_rd (rp_rd[CFG_NUT_CPU_CORES + n]);
        ifu[n].rp_ack (rp_ack[CFG_NUT_CPU_CORES + n]);
        ifu[n].rp_adr (rp_adr[CFG_NUT_CPU_CORES + n]);
        ifu[n].rp_data (rp_data[CFG_NUT_CPU_CORES + n]);
        ifu[n].rp_direct (rp_direct[CFG_NUT_CPU_CORES + n]);
        ifu[n].rp_ac_u (rp_ac_u[CFG_NUT_CPU_CORES + n]);
        ifu[n].rp_ac_x (rp_ac_x[CFG_NUT_CPU_CORES + n]);
        ifu[n].rp_paging (rp_paging[CFG_NUT_CPU_CORES + n]);
        //   to EXU ...
        ifu[n].next (ifu_next[n]);
        ifu[n].jump (ifu_jump[n]);
        ifu[n].flush (ifu_flush[n]);
        ifu[n].jump_adr (ifu_jump_adr[n]);
        ifu[n].ir (ifu_ir[n]);
        ifu[n].pc (ifu_pc[n]);
        ifu[n].npc (ifu_npc[n]);
        ifu[n].ir_valid (ifu_ir_valid[n]);
        ifu[n].npc_valid (ifu_npc_valid[n]);
        ifu[n].icache_enable (m3_icache_enable);
        ifu[n].ac_x (ifu_ac_x[n]);
        ifu[n].ac_u (ifu_ac_u[n]);
        ifu[n].paging (ifu_paging_mode);
    }


    // LSUs...
    lsu = sc_new_array<MLsu>(CFG_NUT_CPU_CORES);
    for (n = 0; n < CFG_NUT_CPU_CORES; n++) {
        sprintf (name, "LSU%i", n);

        lsu[n].clk (clk_i);
        lsu[n].reset (reset);


        //   to EXU...
        lsu[n].rd (lsu_rd[n]);
        lsu[n].wr (lsu_wr[n]);
        lsu[n].flush (lsu_flush[n]);
        lsu[n].lres_scond (lsu_lres_scond[n]);
        lsu[n].cache_op (lsu_cache_op[n]);
        lsu[n].ack (lsu_ack[n]);
        lsu[n].align_err (lsu_align_err[n]);
        lsu[n].scond_ok (lsu_scond_ok[n]);
        lsu[n].width (lsu_width[n]);
        lsu[n].exts (lsu_exts[n]);
        lsu[n].adr (lsu_adr[n]);
        lsu[n].rdata (lsu_rdata[n]);
        lsu[n].wdata (lsu_wdata[n]);
        lsu[n].dcache_enable (m3_dcache_enable);
        lsu[n].trap_u (lsu_trap_u[n]);
        lsu[n].trap_no_u (lsu_trap_no_u[n]);
        lsu[n].ac_w (lsu_ac_w[n]);
        lsu[n].ac_r (lsu_ac_r[n]);
        lsu[n].ac_u (lsu_ac_u[n]);
        lsu[n].paging (lsu_paging_mode);
        //   to MEMU/read port...
        lsu[n].rp_rd (rp_rd[n]);
        lsu[n].rp_bsel (rp_bsel[n]);
        lsu[n].rp_ack (rp_ack[n]);
        lsu[n].rp_adr (rp_adr[n]);
        lsu[n].rp_data (rp_data[n]);
        lsu[n].rp_direct (rp_direct[n]);
        lsu[n].rp_ac_r (rp_ac_r[n]);
        lsu[n].rp_ac_u (rp_ac_u[n]);
        lsu[n].rp_paging (rp_paging[n]);
        //   to MEMU/write port...
        lsu[n].wp_wr (wp_wr[n]);
        lsu[n].wp_bsel (wp_bsel[n]);
        lsu[n].wp_ack (wp_ack[n]);
        lsu[n].wp_lres_scond (wp_lres_scond[n]);
        lsu[n].wp_scond_ok (wp_scond_ok[n]);
        lsu[n].wp_cache_op (wp_cache_op[n]);
        lsu[n].wp_adr (wp_adr[n]);
        lsu[n].wp_data (wp_data[n]);
        lsu[n].wp_direct (wp_direct[n]);
        lsu[n].wp_ac_w (wp_ac_w[n]);
        lsu[n].wp_trap_u (wp_trap_u[n]);
        lsu[n].wp_trap_no_u (wp_trap_no_u[n]);
        lsu[n].wp_paging (wp_paging[n]);
    }

    // CePU with hartID 0
    n = 0;
    exu = sc_new_array<MExu>(CFG_NUT_CPU_CORES);

    // Configuration inputs (see exu.h for detailed description)
    exu[n].hartID (vh_const<sc_uint<CFG_NUT_CPU_CORES_LD> > (n));
    exu[n].inCePU (vh_const<bool> (n == 0));
    exu[n].mode2Cap (vh_const<bool> (CFG_EXU_PNM2CAP & (1 << n)));

    exu[n].clk (clk_i);
    exu[n].reset (reset);

    // Exu control signals
    exu[n].haltreq (exu_haltreq[n]);
    exu[n].enable (exu_enable[n]);
    exu[n].linked (exu_linked[n]);
    exu[n].sync_next (vh_const<bool> (0));
    exu[n].ex_o (exu_ex_o[n]);
    exu[n].xsel (exu_xsel[n]);


    //   to IFU ...
    exu[n].ifu_next (ifu_next[n]);
    exu[n].ifu_jump (ifu_jump[n]);
    exu[n].ifu_flush (ifu_flush[n]);
    exu[n].ifu_reset (exu_ifu_reset[n]);
    exu[n].ifu_jump_adr (ifu_jump_adr[n]);
    exu[n].ifu_ir_valid (ifu_ir_valid[n]);
    exu[n].ifu_npc_valid (ifu_npc_valid[n]);
    exu[n].ifu_ir (ifu_ir[n]);
    exu[n].ifu_pc (ifu_pc[n]);
    exu[n].ifu_npc (ifu_npc[n]);
    exu[n].ifu_ac_x (ifu_ac_x[n]);
    exu[n].ifu_ac_u (ifu_ac_u[n]);

    //   to Load/Store Unit (LSU)...
    exu[n].lsu_rd (lsu_rd[n]);
    exu[n].lsu_wr (lsu_wr[n]);
    exu[n].lsu_flush (lsu_flush[n]);
    exu[n].lsu_cache_op (lsu_cache_op[n]);
    exu[n].lsu_ack (lsu_ack[n]);
    exu[n].lsu_align_err (lsu_align_err[n]);
    exu[n].lsu_width (lsu_width[n]);
    exu[n].lsu_exts (lsu_exts[n]);
    exu[n].lsu_adr (lsu_adr[n]);
    exu[n].lsu_rdata (lsu_rdata[n]);
    exu[n].lsu_wdata (lsu_wdata[n]);
    exu[n].lsu_lres_scond (lsu_lres_scond[n]);
    exu[n].lsu_scond_ok (lsu_scond_ok[n]);
    exu[n].lsu_trap_u (lsu_trap_u[n]);
    exu[n].lsu_trap_no_u (lsu_trap_no_u[n]);
    exu[n].lsu_ac_r (lsu_ac_r[n]);
    exu[n].lsu_ac_w (lsu_ac_w[n]);
    exu[n].lsu_ac_u (lsu_ac_u[n]);

    // dedicated for MMU
    exu[n].lsu_paging_mode (lsu_paging_mode);
    exu[n].ifu_paging_mode (ifu_paging_mode);
    exu[n].root_ppn (root_ppn);
    
    // dedicated for TLB
    exu[n].tlb_flush (tlb_flush);

    //   Special CePU signals...
    exu[n].m3_icache_enable (m3_icache_enable);
    exu[n].m3_dcache_enable (m3_dcache_enable);
    exu[n].m3_priv_mode_o (m3_priv_mode);
    exu[n].m3_priv_mode_i (vh_const<sc_uint<2> > (0));
    exu[n].m3_pnce (m3_pnce);
    exu[n].m3_pnlm (m3_pnlm);
    exu[n].m3_pnxsel (m3_pnxsel);
    exu[n].m3_pnhaltreq (m3_pnhaltreq);

    exu[n].m3_pnx (m3_pnx);
    exu[n].m3_ir_request (m3_ir_request);
    exu[n].m3_ir_id (m3_ir_id);

    exu[n].m3_ir_ack (m3_ir_ack);
    exu[n].m3_ir_enable (m3_ir_enable);
    exu[n].ex_i (m3_ex_i);

    //   Special Mode 2 CoPU signals...
    exu[n].m2_ir (vh_const<sc_uint<32> > (0));
    exu[n].m2_pc (vh_const<sc_uint<32> > (0));
    exu[n].m2_ir_valid (vh_const<bool> (0));
    exu[n].m2_ac_u (vh_const<bool> (0));
    exu[n].m2_ac_x (vh_const<bool> (0));

    //    Debug Unit signals...
    exu[n].dbg_req (exu_dbg_req[n]);
    
    // CoPUs
    for (n = 1; n < CFG_NUT_CPU_CAP2_CORES; n++) {

        // Configuration inputs (see exu.h for detailed description)
        exu[n].hartID (vh_const<sc_uint<CFG_NUT_CPU_CORES_LD> > (n));
        exu[n].inCePU (vh_const<bool> (0));
        exu[n].mode2Cap (vh_const<bool> (CFG_EXU_PNM2CAP & (1 << n)));

        exu[n].clk (clk_i);
        exu[n].reset (reset);

        // Exu control signals
        exu[n].haltreq (exu_haltreq[n]);
        exu[n].enable (exu_enable[n]);
        exu[n].linked (exu_linked[n]);
        exu[n].sync_next (exu_sync[CFG_NUT_CPU_CORES-1]);
        exu[n].ex_o (exu_ex_o[n]);
        exu[n].xsel (exu_xsel[n]);

        //   to IFU ...
        exu[n].ifu_next (ifu_next[n]);
        exu[n].ifu_jump (ifu_jump[n]);
        exu[n].ifu_flush (ifu_flush[n]);
        exu[n].ifu_reset (exu_ifu_reset[n]);
        exu[n].ifu_jump_adr (ifu_jump_adr[n]);
        exu[n].ifu_ir_valid (ifu_ir_valid[n]);
        exu[n].ifu_npc_valid (ifu_npc_valid[n]);
        exu[n].ifu_ir (ifu_ir[n]);
        exu[n].ifu_pc (ifu_pc[n]);
        exu[n].ifu_npc (ifu_npc[n]);
        exu[n].ifu_ac_x (ifu_ac_x[n]);
        exu[n].ifu_ac_u (ifu_ac_u[n]);

        //   to Load/Store Unit (LSU)...
        exu[n].lsu_rd (lsu_rd[n]);
        exu[n].lsu_wr (lsu_wr[n]);
        exu[n].lsu_flush (lsu_flush[n]);
        exu[n].lsu_cache_op (lsu_cache_op[n]);
        exu[n].lsu_ack (lsu_ack[n]);
        exu[n].lsu_align_err (lsu_align_err[n]);
        exu[n].lsu_width (lsu_width[n]);
        exu[n].lsu_exts (lsu_exts[n]);
        exu[n].lsu_adr (lsu_adr[n]);
        exu[n].lsu_rdata (lsu_rdata[n]);
        exu[n].lsu_wdata (lsu_wdata[n]);
        exu[n].lsu_lres_scond (lsu_lres_scond[n]);
        exu[n].lsu_scond_ok (lsu_scond_ok[n]);
        exu[n].lsu_trap_u (lsu_trap_u[n]);
        exu[n].lsu_trap_no_u (lsu_trap_no_u[n]);
        exu[n].lsu_ac_r (lsu_ac_r[n]);
        exu[n].lsu_ac_w (lsu_ac_w[n]);
        exu[n].lsu_ac_u (lsu_ac_u[n]);

        //   Special CePU signals left open on CoPUs...
        exu[n].m3_icache_enable (vh_open);
        exu[n].m3_dcache_enable (vh_open);
        exu[n].m3_priv_mode_o (vh_open);
        exu[n].m3_pnce (vh_open);
        exu[n].m3_pnlm (vh_open);
        exu[n].m3_pnxsel (vh_open);
        exu[n].m3_pnhaltreq (vh_const<sc_uint<CFG_NUT_CPU_CORES>> (0));
        exu[n].m3_pnx (vh_const<sc_uint<CFG_NUT_CPU_CORES>> (0));
        exu[n].m3_ir_request (m3_ir_request);
        exu[n].m3_priv_mode_i (m3_priv_mode);
        exu[n].m3_ir_id (vh_const<sc_uint<5>> (0));
        exu[n].m3_ir_ack (vh_open);
        exu[n].m3_ir_enable (vh_open);
        exu[n].ex_i (exu_ex_o[0]); // Special: all CoPUs get CePUs exception output as exception input

        //   Special Mode 2 CoPU signals...
        exu[n].m2_ir (ifu_ir[0]);
        exu[n].m2_pc (ifu_pc[0]);
        exu[n].m2_ir_valid (ifu_ir_valid[0]);
        exu[n].m2_ac_x (ifu_ac_x[0]);
        exu[n].m2_ac_u (ifu_ac_u[0]);

        //    Debug Unit signals...
        exu[n].dbg_req (exu_dbg_req[n]);

        exu[n].ifu_paging_mode (vh_open);
        exu[n].lsu_paging_mode (vh_open);
        exu[n].root_ppn (vh_open);
        exu[n].tlb_flush (vh_open);
    }

    // Mode 1 CoPUs
    for (; n < CFG_NUT_CPU_CORES; n++) {

        // Configuration inputs (see exu.h for detailed description)
        exu[n].hartID (vh_const<sc_uint<CFG_NUT_CPU_CORES_LD> > (n));
        exu[n].inCePU (vh_const<bool> (0));
        exu[n].mode2Cap (vh_const<bool> (0));

        exu[n].clk (clk_i);
        exu[n].reset (reset);

        // Exu control signals
        exu[n].haltreq (exu_haltreq[n]); // todo: Not required for MODE 1 CoPU
        exu[n].enable (exu_enable[n]);
        exu[n].linked (exu_linked[n]); // todo: Could be fixed to 1
        exu[n].sync_next (exu_sync[CFG_NUT_CPU_CORES-1]);
        exu[n].ex_o (exu_ex_o[n]);
        exu[n].xsel (exu_xsel[n]);

        //   to IFU ...
        exu[n].ifu_next (vh_open);
        exu[n].ifu_jump (vh_open);
        exu[n].ifu_flush (vh_open);
        exu[n].ifu_reset (vh_open);
        exu[n].ifu_jump_adr (vh_open);
        exu[n].ifu_ir_valid (ifu_ir_valid[0]); // Linked to CePU
        exu[n].ifu_npc_valid (ifu_npc_valid[0]); // todo: Could probably be omitted? What if we calculate something on saved RA?
        exu[n].ifu_ir (ifu_ir[0]); // Linked to CePU
        exu[n].ifu_pc (ifu_pc[0]); // Linked to CePU
        exu[n].ifu_npc (ifu_npc[0]); // todo: Could probably be omitted? What if we calculate something on saved RA?
        exu[n].ifu_ac_x (ifu_ac_x[0]); // TODO: check if required. Probably fails in CePU when failing in CoPU
        exu[n].ifu_ac_u (ifu_ac_u[0]); // TODO: check if required. Probably fails in CePU when failing in CoPU

        //   to Load/Store Unit (LSU)...
        exu[n].lsu_rd (lsu_rd[n]);
        exu[n].lsu_wr (lsu_wr[n]);
        exu[n].lsu_flush (lsu_flush[n]);
        exu[n].lsu_cache_op (lsu_cache_op[n]); // todo: Is it needed, that each EXU invalidates a different address?
        exu[n].lsu_ack (lsu_ack[n]);
        exu[n].lsu_align_err (lsu_align_err[n]);
        exu[n].lsu_width (lsu_width[n]); // todo: Is the same for all EXUs. Sharing more efficient?
        exu[n].lsu_exts (lsu_exts[n]); // todo: Is the same for all EXUs. Sharing more efficient?
        exu[n].lsu_adr (lsu_adr[n]);
        exu[n].lsu_rdata (lsu_rdata[n]);
        exu[n].lsu_wdata (lsu_wdata[n]);
        exu[n].lsu_lres_scond (lsu_lres_scond[n]);
        exu[n].lsu_scond_ok (lsu_scond_ok[n]);
        exu[n].lsu_trap_u (lsu_trap_u[n]);
        exu[n].lsu_trap_no_u (lsu_trap_no_u[n]);
        exu[n].lsu_ac_r (lsu_ac_r[n]);
        exu[n].lsu_ac_w (lsu_ac_w[n]);
        exu[n].lsu_ac_u (lsu_ac_u[n]);

        //   Special CePU signals left open on CoPUs...
        exu[n].m3_icache_enable (vh_open);
        exu[n].m3_dcache_enable (vh_open);
        exu[n].m3_priv_mode_o (vh_open);
        exu[n].m3_pnce (vh_open);
        exu[n].m3_pnlm (vh_open);
        exu[n].m3_pnxsel (vh_open);
        exu[n].m3_pnhaltreq (vh_const<sc_uint<CFG_NUT_CPU_CORES>> (0));
        exu[n].m3_pnx (vh_const<sc_uint<CFG_NUT_CPU_CORES>> (0));
        exu[n].m3_ir_request (m3_ir_request);
        exu[n].m3_priv_mode_i (m3_priv_mode);
        exu[n].m3_ir_id (vh_const<sc_uint<5>> (0));
        exu[n].m3_ir_ack (vh_open);
        exu[n].m3_ir_enable (vh_open);
        exu[n].ex_i (exu_ex_o[0]); // Special: all CoPUs get CePUs exception output as exception input

        //   Special Mode 2 CoPU signals...
        exu[n].m2_ir (vh_const<sc_uint<32> > (0));
        exu[n].m2_pc (vh_const<sc_uint<32> > (0));
        exu[n].m2_ir_valid (vh_const<bool> (0));
        exu[n].m2_ac_u (vh_const<bool> (0));
        exu[n].m2_ac_x (vh_const<bool> (0));

        exu[n].lsu_paging_mode (vh_open);
        exu[n].ifu_paging_mode (vh_open);
        exu[n].root_ppn (vh_open);
        exu[n].tlb_flush (vh_open);

        //    Debug Unit signals...
        exu[n].dbg_req (exu_dbg_req[n]);
    }


    // Route exception and Mode 1 sync daisy chain...
    if (CFG_NUT_CPU_CORES_LD > 0) {

        // First / CePU:
        exu[0].cause_i (exu_cause[0]);
        exu[0].cause_o (vh_open);

        exu[0].sync_i (exu_sync[0]);
          // CePU sync_o signal gets routed to CoPUs 'sync_next' input
        exu[0].sync_o (exu_sync[CFG_NUT_CPU_CORES-1]);

        // All but the last CoPU(s):
        for (n = 1; n < CFG_NUT_CPU_CORES - 1; n++) {
            exu[n].cause_i (exu_cause[n]);
            exu[n].cause_o (exu_cause[n - 1]);

            exu[n].sync_i (exu_sync[n]);
            exu[n].sync_o (exu_sync[n - 1]);
        }

        // Last CoPU:
        exu[CFG_NUT_CPU_CORES - 1].cause_i (vh_const<sc_uint<5>> (0));
        exu[CFG_NUT_CPU_CORES - 1].cause_o (exu_cause[CFG_NUT_CPU_CORES - 2]);

        exu[CFG_NUT_CPU_CORES - 1].sync_i (vh_const<bool> (1));
        exu[CFG_NUT_CPU_CORES - 1].sync_o (exu_sync[CFG_NUT_CPU_CORES - 2]);
    } else {
        exu[0].cause_i (vh_const<sc_uint<5>> (0));
        exu[0].cause_o (vh_open);

        exu[0].sync_i (vh_const<bool> (1));
        exu[0].sync_o (vh_open);
    }
    csr = sc_new_array<MCsr>(CFG_NUT_CPU_CORES);
    for (n = 0; n < CFG_NUT_CPU_CORES; n++) {
        csr[n].clk (clk_i);
        csr[n].reset (rst_i);
        csr[n].pc (csr_pc[n]);
        csr[n].ir (csr_ir[n]);

        csr[n].hartID (csr_hartID[n]);
        csr[n].inCePU (csr_inCePU[n]);
        csr[n].cpu_enabled (csr_cpu_enabled[n]);
        csr[n].linked (csr_linked[n]);

        // debug
        csr[n].exu_dbg (dbg[n]);
        csr[n].exu_dbg_reg (dbg_reg[n]);
        csr[n].exu_dbg_req (exu[n].dbg_req);
        csr[n].exu_dbg_enter_dreg (dbg_enter_dreg[n]);

        // exception stuff
        csr[n].ex_id_reg (ex_id_reg[n]);
        csr[n].exception (exception[n]);
        csr[n].irq_dreg (irq_dreg[n]);

        csr[n].exu_pop_priv_ir_stack_dreg (pop_priv_ir_stack_dreg[n]);

        // functional signals
        csr[n].csr_tval (csr_tval[n]);
        csr[n].csr_adr_reg (csr_adr_reg[n]);
        csr[n].csr_function_reg (csr_function_reg[n]);
        csr[n].csr_op_a (csr_op_a[n]);
        csr[n].csr_rs1_reg (csr_rs1_reg[n]);
        csr[n].exu_cause (csr_cause[n]);

        csr[n].csr_enable (csr_enable[n]);

        csr[n].m3_pnx (csr_m3_pnx[n]);
        csr[n].m3_pnhaltreq (csr_m3_pnhaltreq[n]);

        csr[n].exu_priv_mode (priv_mode[n]);
        csr[n].exu_load_store_priv_mode (load_store_priv_mode[n]);
        csr[n].exu_csr_mstatus_SIE (csr_mstatus_SIE[n]);
        csr[n].exu_csr_mstatus_TSR (csr_mstatus_TSR[n]);
        csr[n].exu_csr_mstatus_SUM (csr_mstatus_SUM[n]);
        csr[n].exu_csr_mstatus_TVM (csr_mstatus_TVM[n]);
        csr[n].exu_csr_sepc (csr_sepc[n]),
        csr[n].exu_csr_stvec (csr_stvec[n]),
        csr[n].exu_csr_mideleg (csr_mideleg[n]),
        csr[n].exu_csr_mip (csr_mip[n]),

        csr[n].exu_csr_exception (csr_exception[n]);
        csr[n].exu_csr_mstatus_MIE (csr_mstatus_MIE[n]),
        csr[n].exu_csr_dcsr_step (csr_dcsr_step[n]),
        csr[n].exu_csr_dcsr_ebreakm (csr_dcsr_ebreakm[n]);
        csr[n].exu_csr_mepc (csr_mepc[n]),
        csr[n].exu_csr_dpc (csr_dpc[n]),
        csr[n].exu_csr_mtvec (csr_mtvec[n]),
        csr[n].exu_csr_mcause (csr_mcause[n]),
        csr[n].exu_csr_rdata (csr_rdata[n]);

        csr[n].exu_m3_priv_mode (csr_m3_priv_mode[n]),
        csr[n].exu_m3_pnce (csr_m3_pnce[n]),
        csr[n].exu_m3_pnlm (csr_m3_pnlm[n]),
        csr[n].exu_m3_pnxsel (csr_m3_pnxsel[n]);
        csr[n].exu_m3_icache_enable (csr_m3_icache_enable[n]),
        csr[n].exu_m3_dcache_enable (csr_m3_dcache_enable[n]);
        csr[n].exu_isHalted (csr_isHalted[n]);

        csr[n].exu_delegate_dreg (delegate_dreg[n]);
        csr[n].sret_dreg (sret_dreg[n]);

        csr[n].exu_csr_satp_root_ppn(csr_satp_root_ppn[n]);
        csr[n].exu_ifu_paging_mode (csr_ifu_paging_mode[n]);
        csr[n].exu_lsu_paging_mode (csr_lsu_paging_mode[n]);
        
        csr[n].exu_ack (csr_ack[n]);
        csr[n].exu_cache_flush (csr_cache_flush[n]);

#if CFG_EXU_PERFCOUNT_ENABLE == 1
        csr[n].perf_inc (perf_inc[n]);
        csr[n].perf_addr (perf_addr[n]);
#endif

        // csr pass-through
        exu[n].csr_pc (csr_pc[n]);
        exu[n].csr_ir (csr_ir[n]);
        exu[n].csr_hartID (csr_hartID[n]);
        exu[n].csr_inCePU (csr_inCePU[n]);
        exu[n].csr_cpu_enabled (csr_cpu_enabled[n]);
        exu[n].csr_linked (csr_linked[n]);
        exu[n].csr_cause (csr_cause[n]);

        exu[n].csr_mcause (csr_mcause[n]);
        exu[n].csr_mtvec (csr_mtvec[n]);
        exu[n].csr_dpc (csr_dpc[n]);
        exu[n].csr_mepc (csr_mepc[n]);

        // Mode 1 and 2 are not capable of handling interrupts
        // Mode 3 is the only core and we only have 1
        exu[n].csr_mstatus_SIE (csr_mstatus_SIE[0]);
        exu[n].csr_mstatus_MIE (csr_mstatus_MIE[0]);
        exu[n].csr_mstatus_TSR (csr_mstatus_TSR[n]); 
        exu[n].csr_mstatus_SUM (csr_mstatus_SUM[n]);
        exu[n].csr_mstatus_TVM (csr_mstatus_TVM[n]);
        exu[n].csr_ifu_paging_mode (csr_ifu_paging_mode[n]);
        exu[n].csr_lsu_paging_mode (csr_lsu_paging_mode[n]);

        exu[n].csr_dcsr_step (csr_dcsr_step[n]);
        exu[n].csr_dcsr_ebreakm (csr_dcsr_ebreakm[n]);

        exu[n].csr_m3_priv_mode (csr_m3_priv_mode[n]);
        exu[n].csr_m3_pnce (csr_m3_pnce[n]);
        exu[n].csr_m3_pnlm (csr_m3_pnlm[n]);
        exu[n].csr_m3_pnxsel (csr_m3_pnxsel[n]);
        exu[n].csr_m3_icache_enable (csr_m3_icache_enable[n]);
        exu[n].csr_m3_dcache_enable (csr_m3_dcache_enable[n]);

        exu[n].csr_m3_pnx (csr_m3_pnx[n]);
        exu[n].csr_m3_pnhaltreq (csr_m3_pnhaltreq[n]);

        exu[n].csr_isHalted (csr_isHalted[n]);

        exu[n].csr_exception (csr_exception[n]);

        exu[n].csr_rdata (csr_rdata[n]);
        exu[n].csr_priv_mode (priv_mode[n]);
        exu[n].csr_load_store_priv_mode (load_store_priv_mode[n]);
        exu[n].csr_sepc (csr_sepc[n]);
        exu[n].csr_stvec (csr_stvec[n]);
        exu[n].csr_mideleg (csr_mideleg[n]);
        exu[n].csr_mip (csr_mip[n]);
        exu[n].csr_delegate_dreg (delegate_dreg[n]);

        exu[n].csr_ack (csr_ack[n]);
        exu[n].csr_cache_flush (csr_cache_flush[n]);

        exu[n].csr_dbg (dbg[n]);
        exu[n].csr_dbg_reg (dbg_reg[n]);
        exu[n].csr_dbg_enter_dreg (dbg_enter_dreg[n]);
        exu[n].csr_enable (csr_enable[n]);
        exu[n].csr_ex_id_reg (ex_id_reg[n]);
        exu[n].csr_sret_dreg (sret_dreg[n]);
        exu[n].csr_exu_exception (exception[n]);
        exu[n].csr_irq_dreg (irq_dreg[n]);
        exu[n].csr_pop_priv_ir_stack_dreg (pop_priv_ir_stack_dreg[n]);
        exu[n].csr_csr_rs1_reg (csr_rs1_reg[n]);
        exu[n].csr_csr_op_a (csr_op_a[n]);
        exu[n].csr_csr_function_reg (csr_function_reg[n]);
        exu[n].csr_csr_adr_reg (csr_adr_reg[n]);
        exu[n].csr_csr_tval (csr_tval[n]);
        exu[n].csr_satp_root_ppn(csr_satp_root_ppn[n]);

#if CFG_EXU_PERFCOUNT_ENABLE == 1
        exu[n].csr_perf_inc (perf_inc[n]);
        exu[n].csr_perf_addr (perf_addr[n]);
#endif
    }

    // csr to mtimer / IRC (only cpu 0)        
    csr[0].mtie_out (mtimer_irq_enable_in); 
    csr[0].mtip_in (csr_mip_MTIP);
    csr[0].meip_in (csr_mip_MEIP);
    for (n = 1; n < CFG_NUT_CPU_CORES; n++) {
        csr[n].mtie_out (vh_open); 
        csr[n].mtip_in (vh_const<bool> (0));
        csr[n].meip_in (vh_const<bool> (0));
    }

    mtimer = sc_new<Mtimer>("mtimer");
    mtimer->wb_clk_i(clk_i);
    mtimer->wb_rst_i(rst_i);
    
    mtimer->wb_stb_i (stb_o); // strobe input
    mtimer->wb_we_i (we_o); // indicates write transfer
    mtimer->wb_ack_o (mtimer_ack_i); // normal termination

    mtimer->wb_adr_i (adr_o); // address bus inputs
    mtimer->wb_dat_i (dat_o); // input data bus
    mtimer->wb_dat_o (mtimer_dat_i); // output data bus
    
    mtimer->irq_out_enable(mtimer_irq_enable_in);
    mtimer->irq_out(mtimer_ir_request);
}

void MParanut::FreeSubmodules () {
    // delete memu;
    // delete dm;
    // // delete dtm;
    // delete intc;
    // delete mtimer;
    // delete lsu;
    // delete exu;
    // delete csr;
    // delete ifu;
}


// **************** Interconnect method *******************


void MParanut::InterconnectMethod () {
    int n;

    // reset signal...
    reset = dbg_reset | rst_i;

    // CoPU-CePu signals...
    sc_uint<CFG_NUT_CPU_CORES> pnhaltreq;
    sc_uint<CFG_NUT_CPU_CORES> pnx;
    sc_uint<CFG_NUT_CPU_CORES-1> copu_pnx;

    for (n = 0; n < CFG_NUT_CPU_CORES; n++) {
        pnhaltreq[n] = exu_haltreq[n].read ();
        pnx[n] = exu_ex_o[n].read ();
        exu_dbg_req[n] = dbg_req.read ()[n];
    }

    copu_pnx = pnx >> 1;

    m3_pnhaltreq = pnhaltreq;
    m3_ex_i = copu_pnx.or_reduce ();
    m3_pnx = pnx;

    // CePU-CoPU signals...
    for (n = 0; n < CFG_NUT_CPU_CORES; n++) {
        exu_enable[n] = m3_pnce.read ()[n];
        exu_linked[n] = m3_pnlm.read ()[n];
        exu_xsel[n] = m3_pnxsel.read ()[n];
    }

    // Constant IFU-MEMU signals...
    for (n = 0; n < CFG_NUT_CPU_CORES; n++) {
        rp_bsel[CFG_NUT_CPU_CORES + n] = 0xf;

        // IFU-EXU signals ...
        ifu_reset[n] = reset | exu_ifu_reset[n];
    }

    // WB interconnect
    if (dbg_ack_i) {
        wb_dat = dbg_dat_i.read ();
        wb_ack = dbg_ack_i.read ();
    } else if (mtimer_ack_i) {
        wb_dat = mtimer_dat_i.read ();
        wb_ack = mtimer_ack_i.read ();        
    } else {
        wb_dat = dat_i.read ();
        wb_ack = ack_i.read ();
    }

    // External interrupt concat
    sc_uint<CFG_NUT_EX_INT> intc_ex_int_var;
    for (n = 0; n < CFG_NUT_EX_INT; n++) {
        intc_ex_int_var[n] = ex_int[n].read ();
    }
    intc_ex_int = intc_ex_int_var;
}
