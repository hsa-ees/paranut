/*************************************************************************

  This file is part of the ParaNut project.

  Copyright (C) 2010-2019 Alexander Bahle <alexander.bahle@hs-augsburg.de>
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


#include "paranut.h"


// **************** Helper Functions *********************************

// vh_open struct (vhdl open equivalent)
static struct {
    template <typename T> operator sc_core::sc_signal_inout_if<T> & () const {
        return *(new sc_core::sc_signal<T> (sc_core::sc_gen_unique_name ("vh_open")));
    }

} const vh_open = {};

template <typename T>
sc_core::sc_signal_in_if<T> const &vh_const (T const &v) // keep the name consistent with vh_open
{
    // Yes, this is an (elaboration-time) memory leak.  You can avoid it with some extra effort
    sc_core::sc_signal<T> *sig_p =
    new sc_core::sc_signal<T> (sc_core::sc_gen_unique_name ("vh_const"));
    sig_p->write (v);
    return *sig_p;
}


// **************** Trace *********************************

void MParanut::Trace (sc_trace_file *tf, int levels) {
    if (!tf || trace_verbose) printf ("\nSignals of Module \"%s\":\n", name ());

    // Ports ...
    TRACE (tf, clk_i);
    TRACE (tf, rst_i);

    TRACE (tf, cyc_o);
    TRACE (tf, stb_o);
    TRACE (tf, we_o);
    TRACE (tf, sel_o);
    TRACE (tf, ack_i);
    TRACE (tf, wb_ack);
    TRACE (tf, wb_dat);

    TRACE (tf, adr_o);
    TRACE (tf, dat_i);
    TRACE (tf, dat_o);

    TRACE (tf, ex_int);

    // Connecting signals...
    //   MEMU ...
    TRACE_BUS (tf, rp_rd, 2 * CFG_NUT_CPU_CORES);
    TRACE_BUS (tf, rp_direct, 2 * CFG_NUT_CPU_CORES);
    TRACE_BUS (tf, rp_bsel, 2 * CFG_NUT_CPU_CORES);
    TRACE_BUS (tf, rp_ack, 2 * CFG_NUT_CPU_CORES);
    TRACE_BUS (tf, rp_adr, 2 * CFG_NUT_CPU_CORES);
    TRACE_BUS (tf, rp_data, 2 * CFG_NUT_CPU_CORES);
    TRACE_BUS (tf, wp_wr, CFG_NUT_CPU_CORES);
    TRACE_BUS (tf, wp_direct, CFG_NUT_CPU_CORES);
    TRACE_BUS (tf, wp_bsel, CFG_NUT_CPU_CORES);
    TRACE_BUS (tf, wp_ack, CFG_NUT_CPU_CORES);
    TRACE_BUS (tf, wp_lres_scond, CFG_NUT_CPU_CORES);
    TRACE_BUS (tf, wp_scond_ok, CFG_NUT_CPU_CORES);
    TRACE_BUS (tf, wp_writeback, CFG_NUT_CPU_CORES);
    TRACE_BUS (tf, wp_invalidate, CFG_NUT_CPU_CORES);
    TRACE_BUS (tf, wp_adr, CFG_NUT_CPU_CORES);
    TRACE_BUS (tf, wp_data, CFG_NUT_CPU_CORES);
    //   IFU <-> EXU ...
    TRACE_BUS (tf, ifu_next, CFG_NUT_CPU_MODE2_CORES);
    TRACE_BUS (tf, ifu_jump, CFG_NUT_CPU_MODE2_CORES);
    TRACE_BUS (tf, ifu_jump_adr, CFG_NUT_CPU_MODE2_CORES);
    TRACE_BUS (tf, ifu_ir_valid, CFG_NUT_CPU_MODE2_CORES);
    TRACE_BUS (tf, ifu_npc_valid, CFG_NUT_CPU_MODE2_CORES);
    TRACE_BUS (tf, ifu_ir, CFG_NUT_CPU_MODE2_CORES);
    TRACE_BUS (tf, ifu_pc, CFG_NUT_CPU_MODE2_CORES);
    TRACE_BUS (tf, ifu_npc, CFG_NUT_CPU_MODE2_CORES);
    //   LSU <-> EXU ...
    TRACE_BUS (tf, lsu_rd, CFG_NUT_CPU_CORES);
    TRACE_BUS (tf, lsu_wr, CFG_NUT_CPU_CORES);
    TRACE_BUS (tf, lsu_flush, CFG_NUT_CPU_CORES);
    TRACE_BUS (tf, lsu_lres_scond, CFG_NUT_CPU_CORES);
    TRACE_BUS (tf, lsu_cache_writeback, CFG_NUT_CPU_CORES);
    TRACE_BUS (tf, lsu_cache_invalidate, CFG_NUT_CPU_CORES);
    TRACE_BUS (tf, lsu_ack, CFG_NUT_CPU_CORES);
    TRACE_BUS (tf, lsu_align_err, CFG_NUT_CPU_CORES);
    TRACE_BUS (tf, lsu_scond_ok, CFG_NUT_CPU_CORES);
    TRACE_BUS (tf, lsu_width, CFG_NUT_CPU_CORES);
    TRACE_BUS (tf, lsu_exts, CFG_NUT_CPU_CORES);
    TRACE_BUS (tf, lsu_adr, CFG_NUT_CPU_CORES);
    TRACE_BUS (tf, lsu_rdata, CFG_NUT_CPU_CORES);
    TRACE_BUS (tf, lsu_wdata, CFG_NUT_CPU_CORES);
    //   other signals...
    TRACE (tf, m3_icache_enable);
    TRACE (tf, m3_dcache_enable);

    // Sub-Modules...
    if (levels > 1) {
        levels--;
        memu->Trace (tf, levels);
        dm->Trace (tf, levels);
        dtm->Trace (tf, levels);
        intc->Trace (tf, levels);
        for (int n = 0; n < CFG_NUT_CPU_MODE2_CORES; n++) {
            ifu[n]->Trace (tf, levels);
        }
        for (int n = 0; n < CFG_NUT_CPU_CORES; n++) {
            lsu[n]->Trace (tf, levels);
            exu[n]->Trace (tf, levels);
        }
    }
}


// **************** Submodules ****************************

void MParanut::InitSubmodules () {
    char name[80];
    int n;

    // DBGU...
    dm = new MDebugModule ("DM");

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
    dtm = new MDtm ("DTM");

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
    intc = new MIntC ("IntC");

    intc->clk (clk_i); // clock input
    intc->reset (reset); // reset

    intc->ex_int (ex_int);
    intc->ir_request (m3_ir_request);
    intc->ir_id (m3_ir_id);
    intc->ir_ack (m3_ir_ack);
    intc->ir_enable (m3_ir_enable);


    // MEMU...
    memu = new MMemu ("MemU");

    memu->clk (clk_i);
    memu->reset (reset);

    //   bus interface (Wishbone)...
    memu->wb_cyc_o (cyc_o);
    memu->wb_stb_o (stb_o);
    memu->wb_we_o (we_o);
    memu->wb_sel_o (sel_o);
    memu->wb_adr_o (adr_o);
    memu->wb_dat_o (dat_o);
    memu->wb_ack_i (wb_ack);
    memu->wb_dat_i (wb_dat);

    //    read ports...
    for (n = 0; n < RPORTS - CFG_NUT_CPU_MODE1_CORES; n++) {
        memu->rp_rd[n](rp_rd[n]);
        memu->rp_direct[n](rp_direct[n]);
        memu->rp_bsel[n](rp_bsel[n]);
        memu->rp_ack[n](rp_ack[n]);
        memu->rp_adr[n](rp_adr[n]);
        memu->rp_data[n](rp_data[n]);
    }

    //    unused read ports (Mode 1 only CoPUs)...
    for (; n < RPORTS; n++) {
        memu->rp_rd[n](vh_const<bool> (0));
        memu->rp_direct[n](vh_const<bool> (0));
        memu->rp_bsel[n](vh_const<sc_uint<4>> (0));
        memu->rp_ack[n](vh_open);
        memu->rp_adr[n](vh_const<TWord> (n));
        memu->rp_data[n](vh_open);
    }

    //   write ports...
    for (n = 0; n < WPORTS; n++) {
        memu->wp_wr[n](wp_wr[n]);
        memu->wp_direct[n](wp_direct[n]);
        memu->wp_bsel[n](wp_bsel[n]);
        memu->wp_ack[n](wp_ack[n]);
        memu->wp_lres_scond[n](wp_lres_scond[n]);
        memu->wp_scond_ok[n](wp_scond_ok[n]);
        memu->wp_writeback[n](wp_writeback[n]);
        memu->wp_invalidate[n](wp_invalidate[n]);
        memu->wp_adr[n](wp_adr[n]);
        memu->wp_data[n](wp_data[n]);
    }


    // IFUs...
    for (n = 0; n < CFG_NUT_CPU_MODE2_CORES; n++) {
        sprintf (name, "IFU%i", n);
        ifu[n] = new MIfu (name);

        ifu[n]->clk (clk_i);
        ifu[n]->reset (ifu_reset[n]);
        //   to MEMU (read port)...
        ifu[n]->rp_rd (rp_rd[CFG_NUT_CPU_CORES + n]);
        ifu[n]->rp_ack (rp_ack[CFG_NUT_CPU_CORES + n]);
        ifu[n]->rp_adr (rp_adr[CFG_NUT_CPU_CORES + n]);
        ifu[n]->rp_data (rp_data[CFG_NUT_CPU_CORES + n]);
        ifu[n]->rp_direct (rp_direct[CFG_NUT_CPU_CORES + n]);
        //   to EXU ...
        ifu[n]->next (ifu_next[n]);
        ifu[n]->jump (ifu_jump[n]);
        ifu[n]->flush (ifu_flush[n]);
        ifu[n]->jump_adr (ifu_jump_adr[n]);
        ifu[n]->ir (ifu_ir[n]);
        ifu[n]->pc (ifu_pc[n]);
        ifu[n]->npc (ifu_npc[n]);
        ifu[n]->ir_valid (ifu_ir_valid[n]);
        ifu[n]->npc_valid (ifu_npc_valid[n]);
        ifu[n]->icache_enable (m3_icache_enable);
    }


    // LSUs...
    for (n = 0; n < CFG_NUT_CPU_CORES; n++) {
        sprintf (name, "LSU%i", n);
        lsu[n] = new MLsu (name);

        lsu[n]->clk (clk_i);
        lsu[n]->reset (reset);

        //   to EXU...
        lsu[n]->rd (lsu_rd[n]);
        lsu[n]->wr (lsu_wr[n]);
        lsu[n]->flush (lsu_flush[n]);
        lsu[n]->lres_scond (lsu_lres_scond[n]);
        lsu[n]->cache_writeback (lsu_cache_writeback[n]);
        lsu[n]->cache_invalidate (lsu_cache_invalidate[n]);
        lsu[n]->ack (lsu_ack[n]);
        lsu[n]->align_err (lsu_align_err[n]);
        lsu[n]->scond_ok (lsu_scond_ok[n]);
        lsu[n]->width (lsu_width[n]);
        lsu[n]->exts (lsu_exts[n]);
        lsu[n]->adr (lsu_adr[n]);
        lsu[n]->rdata (lsu_rdata[n]);
        lsu[n]->wdata (lsu_wdata[n]);
        lsu[n]->dcache_enable (m3_dcache_enable);
        //   to MEMU/read port...
        lsu[n]->rp_rd (rp_rd[n]);
        lsu[n]->rp_bsel (rp_bsel[n]);
        lsu[n]->rp_ack (rp_ack[n]);
        lsu[n]->rp_adr (rp_adr[n]);
        lsu[n]->rp_data (rp_data[n]);
        lsu[n]->rp_direct (rp_direct[n]);
        //   to MEMU/write port...
        lsu[n]->wp_wr (wp_wr[n]);
        lsu[n]->wp_bsel (wp_bsel[n]);
        lsu[n]->wp_ack (wp_ack[n]);
        lsu[n]->wp_lres_scond (wp_lres_scond[n]);
        lsu[n]->wp_scond_ok (wp_scond_ok[n]);
        lsu[n]->wp_writeback (wp_writeback[n]);
        lsu[n]->wp_invalidate (wp_invalidate[n]);
        lsu[n]->wp_adr (wp_adr[n]);
        lsu[n]->wp_data (wp_data[n]);
        lsu[n]->wp_direct (wp_direct[n]);
    }


    // EXUs...
    // CePU with hartID 0
    n = 0;
    sprintf (name, "EXU%i", n);
    exu[n] = new MExu (name);

    // Configuration inputs (see exu.h for detailed description)
    exu[n]->hartID (vh_const<TWord> (n));
    exu[n]->inCePU (vh_const<bool> (n == 0));
    exu[n]->mode2Cap (vh_const<bool> (CFG_EXU_PNM2CAP & (1 << n)));
    exu[n]->clock_freq_hz (vh_const<TWord> (CFG_NUT_SIM_CLK_SPEED));

    exu[n]->clk (clk_i);
    exu[n]->reset (reset);

    // Exu control signals
    exu[n]->haltreq (exu_haltreq[n]);
    exu[n]->enable (exu_enable[n]);
    exu[n]->linked (exu_linked[n]);
    exu[n]->sync_next (vh_const<bool> (0));
    exu[n]->ex_o (exu_ex_o[n]);
    exu[n]->xsel (exu_xsel[n]);

    //   to IFU ...
    exu[n]->ifu_next (ifu_next[n]);
    exu[n]->ifu_jump (ifu_jump[n]);
    exu[n]->ifu_flush (ifu_flush[n]);
    exu[n]->ifu_reset (exu_ifu_reset[n]);
    exu[n]->ifu_jump_adr (ifu_jump_adr[n]);
    exu[n]->ifu_ir_valid (ifu_ir_valid[n]);
    exu[n]->ifu_npc_valid (ifu_npc_valid[n]);
    exu[n]->ifu_ir (ifu_ir[n]);
    exu[n]->ifu_pc (ifu_pc[n]);
    exu[n]->ifu_npc (ifu_npc[n]);

    //   to Load/Store Unit (LSU)...
    exu[n]->lsu_rd (lsu_rd[n]);
    exu[n]->lsu_wr (lsu_wr[n]);
    exu[n]->lsu_flush (lsu_flush[n]);
    exu[n]->lsu_cache_invalidate (lsu_cache_invalidate[n]);
    exu[n]->lsu_cache_writeback (lsu_cache_writeback[n]);
    exu[n]->lsu_ack (lsu_ack[n]);
    exu[n]->lsu_align_err (lsu_align_err[n]);
    exu[n]->lsu_width (lsu_width[n]);
    exu[n]->lsu_exts (lsu_exts[n]);
    exu[n]->lsu_adr (lsu_adr[n]);
    exu[n]->lsu_rdata (lsu_rdata[n]);
    exu[n]->lsu_wdata (lsu_wdata[n]);
    exu[n]->lsu_lres_scond (lsu_lres_scond[n]);
    exu[n]->lsu_scond_ok (lsu_scond_ok[n]);

    //   Special CePU signals...
    exu[n]->m3_icache_enable (m3_icache_enable);
    exu[n]->m3_dcache_enable (m3_dcache_enable);
    exu[n]->m3_pnhaltreq (m3_pnhaltreq);
    exu[n]->m3_pnce (m3_pnce);
    exu[n]->m3_pnlm (m3_pnlm);
    exu[n]->m3_pnxsel (m3_pnxsel);
    exu[n]->m3_pnx (m3_pnx);
    exu[n]->m3_ir_request (m3_ir_request);
    exu[n]->m3_ir_id (m3_ir_id);
    exu[n]->m3_ir_ack (m3_ir_ack);
    exu[n]->m3_ir_enable (m3_ir_enable);
    exu[n]->ex_i (m3_ex_i);

    //   Special Mode 2 CoPU signals...
    exu[n]->m2_ir (vh_const<TWord> (0));
    exu[n]->m2_pc (vh_const<TWord> (0));
    exu[n]->m2_ir_valid (vh_const<bool> (0));

    //    Debug Unit signals...
    exu[n]->dbg_req (exu_dbg_req[n]);


    // CoPUs
    for (n = 1; n < CFG_NUT_CPU_MODE2_CORES; n++) {
        sprintf (name, "EXU%i", n);
        exu[n] = new MExu (name);

        // Configuration inputs (see exu.h for detailed description)
        exu[n]->hartID (vh_const<TWord> (n));
        exu[n]->inCePU (vh_const<bool> (0));
        exu[n]->mode2Cap (vh_const<bool> (CFG_EXU_PNM2CAP & (1 << n)));
        exu[n]->clock_freq_hz (vh_const<TWord> (CFG_NUT_SIM_CLK_SPEED));

        exu[n]->clk (clk_i);
        exu[n]->reset (reset);

        // Exu control signals
        exu[n]->haltreq (exu_haltreq[n]);
        exu[n]->enable (exu_enable[n]);
        exu[n]->linked (exu_linked[n]);
        exu[n]->sync_next (exu_sync[CFG_NUT_CPU_CORES-1]);
        exu[n]->ex_o (exu_ex_o[n]);
        exu[n]->xsel (exu_xsel[n]);

        //   to IFU ...
        exu[n]->ifu_next (ifu_next[n]);
        exu[n]->ifu_jump (ifu_jump[n]);
        exu[n]->ifu_flush (ifu_flush[n]);
        exu[n]->ifu_reset (exu_ifu_reset[n]);
        exu[n]->ifu_jump_adr (ifu_jump_adr[n]);
        exu[n]->ifu_ir_valid (ifu_ir_valid[n]);
        exu[n]->ifu_npc_valid (ifu_npc_valid[n]);
        exu[n]->ifu_ir (ifu_ir[n]);
        exu[n]->ifu_pc (ifu_pc[n]);
        exu[n]->ifu_npc (ifu_npc[n]);

        //   to Load/Store Unit (LSU)...
        exu[n]->lsu_rd (lsu_rd[n]);
        exu[n]->lsu_wr (lsu_wr[n]);
        exu[n]->lsu_flush (lsu_flush[n]);
        exu[n]->lsu_cache_invalidate (lsu_cache_invalidate[n]);
        exu[n]->lsu_cache_writeback (lsu_cache_writeback[n]);
        exu[n]->lsu_ack (lsu_ack[n]);
        exu[n]->lsu_align_err (lsu_align_err[n]);
        exu[n]->lsu_width (lsu_width[n]);
        exu[n]->lsu_exts (lsu_exts[n]);
        exu[n]->lsu_adr (lsu_adr[n]);
        exu[n]->lsu_rdata (lsu_rdata[n]);
        exu[n]->lsu_wdata (lsu_wdata[n]);
        exu[n]->lsu_lres_scond (lsu_lres_scond[n]);
        exu[n]->lsu_scond_ok (lsu_scond_ok[n]);


        //   Special CePU signals left open on CoPUs...
        exu[n]->m3_icache_enable (vh_open);
        exu[n]->m3_dcache_enable (vh_open);
        exu[n]->m3_pnce (vh_open);
        exu[n]->m3_pnlm (vh_open);
        exu[n]->m3_pnxsel (vh_open);
        exu[n]->m3_pnhaltreq (vh_const<sc_uint<CFG_NUT_CPU_CORES>> (0));
        exu[n]->m3_pnx (vh_const<sc_uint<CFG_NUT_CPU_CORES>> (0));
        exu[n]->m3_ir_request (vh_const<bool> (0));
        exu[n]->m3_ir_id (vh_const<sc_uint<5>> (0));
        exu[n]->m3_ir_ack (vh_open);
        exu[n]->m3_ir_enable (vh_open);
        exu[n]->ex_i (exu_ex_o[0]); // Special: all CoPUs get CePUs exception output as exception input

        //   Special Mode 2 CoPU signals...
        exu[n]->m2_ir (ifu_ir[0]);
        exu[n]->m2_pc (ifu_pc[0]);
        exu[n]->m2_ir_valid (ifu_ir_valid[0]);

        //    Debug Unit signals...
        exu[n]->dbg_req (exu_dbg_req[n]);
    }

    // Mode 1 CoPUs
    for (; n < CFG_NUT_CPU_CORES; n++) {
        sprintf (name, "EXU%i", n);
        exu[n] = new MExu (name);

        // Configuration inputs (see exu.h for detailed description)
        exu[n]->hartID (vh_const<TWord> (n));
        exu[n]->inCePU (vh_const<bool> (0));
        exu[n]->mode2Cap (vh_const<bool> (0));
        exu[n]->clock_freq_hz (vh_const<TWord> (CFG_NUT_SIM_CLK_SPEED));

        exu[n]->clk (clk_i);
        exu[n]->reset (reset);

        // Exu control signals
        exu[n]->haltreq (exu_haltreq[n]); // todo: Not required for MODE 1 CoPU
        exu[n]->enable (exu_enable[n]);
        exu[n]->linked (exu_linked[n]); // todo: Could be fixed to 1
        exu[n]->sync_next (exu_sync[CFG_NUT_CPU_CORES-1]);
        exu[n]->ex_o (exu_ex_o[n]);
        exu[n]->xsel (exu_xsel[n]);

        //   to IFU ...
        exu[n]->ifu_next (vh_open);
        exu[n]->ifu_jump (vh_open);
        exu[n]->ifu_flush (vh_open);
        exu[n]->ifu_reset (vh_open);
        exu[n]->ifu_jump_adr (vh_open);
        exu[n]->ifu_ir_valid (ifu_ir_valid[0]); // Linked to CePU
        exu[n]->ifu_npc_valid (ifu_npc_valid[0]); // todo: Could probably be omitted? What if we calculate something on saved RA?
        exu[n]->ifu_ir (ifu_ir[0]); // Linked to CePU
        exu[n]->ifu_pc (ifu_pc[0]); // Linked to CePU
        exu[n]->ifu_npc (ifu_npc[0]); // todo: Could probably be omitted? What if we calculate something on saved RA?

        //   to Load/Store Unit (LSU)...
        exu[n]->lsu_rd (lsu_rd[n]);
        exu[n]->lsu_wr (lsu_wr[n]);
        exu[n]->lsu_flush (lsu_flush[n]);
        exu[n]->lsu_cache_invalidate (lsu_cache_invalidate[n]); // todo: Is it needed, that each EXU invalidates a different address?
        exu[n]->lsu_cache_writeback (lsu_cache_writeback[n]); // todo: Is it needed, that each EXU writes back a different address?
        exu[n]->lsu_ack (lsu_ack[n]);
        exu[n]->lsu_align_err (lsu_align_err[n]);
        exu[n]->lsu_width (lsu_width[n]); // todo: Is the same for all EXUs. Sharing more efficient?
        exu[n]->lsu_exts (lsu_exts[n]); // todo: Is the same for all EXUs. Sharing more efficient?
        exu[n]->lsu_adr (lsu_adr[n]);
        exu[n]->lsu_rdata (lsu_rdata[n]);
        exu[n]->lsu_wdata (lsu_wdata[n]);
        exu[n]->lsu_lres_scond (lsu_lres_scond[n]);
        exu[n]->lsu_scond_ok (lsu_scond_ok[n]);

        //   Special CePU signals left open on CoPUs...
        exu[n]->m3_icache_enable (vh_open);
        exu[n]->m3_dcache_enable (vh_open);
        exu[n]->m3_pnce (vh_open);
        exu[n]->m3_pnlm (vh_open);
        exu[n]->m3_pnxsel (vh_open);
        exu[n]->m3_pnhaltreq (vh_const<sc_uint<CFG_NUT_CPU_CORES>> (0));
        exu[n]->m3_pnx (vh_const<sc_uint<CFG_NUT_CPU_CORES>> (0));
        exu[n]->m3_ir_request (vh_const<bool> (0));
        exu[n]->m3_ir_id (vh_const<sc_uint<5>> (0));
        exu[n]->m3_ir_ack (vh_open);
        exu[n]->m3_ir_enable (vh_open);
        exu[n]->ex_i (exu_ex_o[0]); // Special: all CoPUs get CePUs exception output as exception input

        //   Special Mode 2 CoPU signals...
        exu[n]->m2_ir (vh_const<TWord> (0));
        exu[n]->m2_pc (vh_const<TWord> (0));
        exu[n]->m2_ir_valid (vh_const<bool> (0));

        //    Debug Unit signals...
        exu[n]->dbg_req (exu_dbg_req[n]);
    }


    // Route exception and Mode 1 sync daisy chain...
    // First CePU:
    if (CFG_NUT_CPU_CORES_LD > 0) {
        exu[0]->cause_i (exu_cause[0]);
        exu[0]->cause_o (vh_open);
        exu[0]->epc_i (exu_epc[0]);
        exu[0]->epc_o (vh_open);

        exu[0]->sync_i (exu_sync[0]);
        // CePU sync_o signal gets routed to CoPUs sync_next input
        exu[0]->sync_o (exu_sync[CFG_NUT_CPU_CORES-1]);

        // All other CoPUs:
        for (n = 1; n < CFG_NUT_CPU_CORES - 1; n++) {
            exu[n]->cause_i (exu_cause[n]);
            exu[n]->cause_o (exu_cause[n - 1]);
            exu[n]->epc_i (exu_epc[n]);
            exu[n]->epc_o (exu_epc[n - 1]);

            exu[n]->sync_i (exu_sync[n]);
            exu[n]->sync_o (exu_sync[n - 1]);
        }

        // Last CoPU:
        exu[CFG_NUT_CPU_CORES - 1]->cause_i (vh_const<sc_uint<5>> (0));
        exu[CFG_NUT_CPU_CORES - 1]->cause_o (exu_cause[CFG_NUT_CPU_CORES - 2]);
        exu[CFG_NUT_CPU_CORES - 1]->epc_i (vh_const<sc_uint<32>> (0));
        exu[CFG_NUT_CPU_CORES - 1]->epc_o (exu_epc[CFG_NUT_CPU_CORES - 2]);

        exu[CFG_NUT_CPU_CORES - 1]->sync_i (vh_const<bool> (1));
        exu[CFG_NUT_CPU_CORES - 1]->sync_o (exu_sync[CFG_NUT_CPU_CORES - 2]);
    } else {
        exu[0]->cause_i (vh_const<sc_uint<5>> (0));
        exu[0]->cause_o (vh_open);
        exu[0]->epc_i (vh_const<sc_uint<32>> (0));
        exu[0]->epc_o (vh_open);

        exu[0]->sync_i (vh_const<bool> (1));
        exu[0]->sync_o (vh_open);
    }
}

void MParanut::FreeSubmodules () {
    delete memu;
    delete dm;
    delete dtm;
    delete intc;
    for (int n = 0; n < CFG_NUT_CPU_CORES; n++) {
        delete lsu[n];
        delete exu[n];
    }
    for (int n = 0; n < CFG_NUT_CPU_MODE2_CORES; n++) delete ifu[n];
}


// **************** Interconnect method *******************

void MParanut::InitInterconnectMethod () {
    SC_METHOD (InterconnectMethod);
        sensitive << dbg_reset << rst_i << reset;
        sensitive << m3_pnce << m3_pnlm << cepu_pnifadr << m3_pnxsel;
        sensitive << dbg_req;
        sensitive << dbg_ack_i << dbg_dat_i << ack_i << dat_i;
        // MEMU port signals...
        for (int n = 0; n < CFG_NUT_CPU_CORES; n++)
            sensitive << exu_ifu_reset[n] << exu_haltreq[n] << exu_ex_o[n];
}


void MParanut::InterconnectMethod () {
    int n;

    // reset signal...
    reset = dbg_reset | rst_i;

    // CoPU-CePu signals...
    sc_uint<CFG_NUT_CPU_CORES> pnhaltreq;
    sc_uint<CFG_NUT_CPU_CORES> pnx;

    for (n = 0; n < CFG_NUT_CPU_CORES; n++) {
        pnhaltreq[n] = exu_haltreq[n].read ();
        pnx[n] = exu_ex_o[n].read ();
        exu_dbg_req[n] = dbg_req.read ()[n];
    }

    m3_pnhaltreq = pnhaltreq;
    m3_ex_i = pnx.or_reduce ();
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
    } else {
        wb_dat = dat_i.read ();
        wb_ack = ack_i.read ();
    }
}
