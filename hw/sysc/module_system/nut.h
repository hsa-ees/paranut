/*************************************************************************

  This file is part of the ParaNut project.

  Copyright (C) 2010-2022 Alexander Bahle <alexander.bahle@hs-augsburg.de>
                          Gundolf Kiefer <gundolf.kiefer@hs-augsburg.de>
                          Christian H. Meyer <christian.meyer@hs-augsburg.de>
                          Nico Borgsmüller <nico.borgsmueller@hs-augsburg.d
      Hochschule Augsburg, University of Applied Sciences

  Description:
    This is the top-level component of the ParaNut.
    It contains the following sub-modules:
    - 1 memory unit (MEMU)
    for each CPU:
    - 1 instruction fetch unit (IFU)
    - 1 execution unit (EXU)
    - 1 load-store unit (LSU)

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


#ifndef _PARANUT_
#define _PARANUT_

#include "dm.h"
#include "exu.h"
#include "csr.h"
#include "ifu.h"
#include "intc.h"
#include "jtag_dtm.h"
#include "lsu.h"
#include "memu.h"
#include "mtimer.h"

#include <systemc.h>


class MParanut : ::sc_core::sc_module {
public:
    // Ports (WISHBONE master)...
    sc_in<bool>         clk_i; // clock input
    sc_in<bool>         rst_i; // reset

    sc_out<bool>        cyc_o; // cycle valid output
    sc_out<bool>        stb_o; // strobe output
    sc_out<bool>        we_o; // indicates write transfer
    sc_out<sc_uint<3> > cti_o; // cycle type identifier
    sc_out<sc_uint<2> > bte_o; // burst type extension
    sc_out<sc_uint<CFG_MEMU_BUSIF_WIDTH/8> > sel_o; // byte select outputs
    sc_in<bool>         ack_i; // normal termination
    sc_in<bool>         err_i; // termination w/ error (presently unsupported)
    sc_in<bool>         rty_i; // termination w/ retry (presently unsupported)

    sc_out<sc_uint<32> >                    adr_o; // address bus outputs
    sc_in<sc_uint<CFG_MEMU_BUSIF_WIDTH> >   dat_i; // input data bus
    sc_out<sc_uint<CFG_MEMU_BUSIF_WIDTH> >  dat_o; // output data bus

    // External interrupt sources
    sc_in<bool> ex_int[CFG_NUT_EX_INT];

    // JTAG TAP
    sc_in<bool>     tck;
    sc_in<bool>     tms;
    sc_in<bool>     tdi;
    sc_out<bool>    tdo;

    // Constructor/Destructor...
    SC_HAS_PROCESS (MParanut);
    MParanut (sc_module_name name = "Mparanut")
        : sc_module (name) {

        InitSubmodules ();

        SC_METHOD (InterconnectMethod);
            sensitive << dbg_reset << rst_i << reset;
            sensitive << m3_pnce << m3_pnlm << cepu_pnifadr << m3_pnxsel;
            sensitive << dbg_req;
            sensitive << dbg_ack_i << dbg_dat_i << ack_i << dat_i;
            sensitive << mtimer_ack_i << mtimer_dat_i;

            // MEMU port signals...
            for (int n = 0; n < CFG_NUT_CPU_CORES; n++)
                sensitive << exu_ifu_reset[n] << exu_haltreq[n] << exu_ex_o[n];
            for (int n = 0; n < CFG_NUT_EX_INT; n++)
                sensitive << ex_int[n];     // ex_int not used -> intc_ext_int would be right ??
    }
    ~MParanut () { FreeSubmodules (); }

    // Functions...
    void Trace (sc_trace_file * tf, int levels = 1);
    void DisplayStatistics (const int num = 0) { exu[num].DisplayStatistics (); }

    bool IsHalted () { return exu[0].IsHalted (); }

    // Processes...
    void InterconnectMethod ();

    // Submodules...
    MMemu *memu;
    MDebugModule *dm;
    MDtm *dtm;
    MIntC *intc;
    MIfu *ifu;
    MCsr *csr;
    MExu *exu;
    MLsu *lsu;
    Mtimer *mtimer; 

    protected:
    // Connecting signals...

    //   MEMU: read ports (rp), write ports (wp)...
    sc_signal<bool> rp_rd[2 * CFG_NUT_CPU_CORES], rp_direct[2 * CFG_NUT_CPU_CORES];
    sc_signal<bool> rp_paging[2 * CFG_NUT_CPU_CORES];
    sc_signal<sc_uint<4> > rp_bsel[2 * CFG_NUT_CPU_CORES];
    sc_signal<bool> rp_ack[2 * CFG_NUT_CPU_CORES];
    sc_signal<sc_uint<32> > rp_adr[2 * CFG_NUT_CPU_CORES];
    sc_signal<sc_uint<32> > rp_data[2 * CFG_NUT_CPU_CORES];
    sc_signal<bool> rp_ac_r[2 * CFG_NUT_CPU_CORES];
    sc_signal<bool> rp_ac_x[2 * CFG_NUT_CPU_CORES];
    sc_signal<bool> rp_ac_u[2 * CFG_NUT_CPU_CORES];
    sc_signal<bool> rp_trap_u[2 * CFG_NUT_CPU_CORES], rp_trap_no_u[2 * CFG_NUT_CPU_CORES];

    sc_signal<bool> wp_wr[CFG_NUT_CPU_CORES], wp_direct[CFG_NUT_CPU_CORES];
    sc_signal<bool> wp_paging[CFG_NUT_CPU_CORES];
    sc_signal<sc_uint<4> > wp_bsel[CFG_NUT_CPU_CORES];
    sc_signal<bool> wp_ack[CFG_NUT_CPU_CORES];
    sc_signal<bool> wp_lres_scond[CFG_NUT_CPU_CORES];
    sc_signal<bool> wp_scond_ok[CFG_NUT_CPU_CORES];
    sc_signal<sc_uint<3> > wp_cache_op[CFG_NUT_CPU_CORES];
    sc_signal<sc_uint<32> > wp_adr[CFG_NUT_CPU_CORES];
    sc_signal<sc_uint<32> > wp_data[CFG_NUT_CPU_CORES];
    sc_signal<bool> wp_ac_w[CFG_NUT_CPU_CORES];
    sc_signal<bool> wp_trap_u[CFG_NUT_CPU_CORES], wp_trap_no_u[CFG_NUT_CPU_CORES];

    sc_signal<bool> wb_ack;
    sc_signal<sc_uint<CFG_MEMU_BUSIF_WIDTH> > wb_dat;

    //   IFU ...
    sc_signal<bool> ifu_next[CFG_NUT_CPU_CAP2_CORES], ifu_jump[CFG_NUT_CPU_CAP2_CORES],
        ifu_flush[CFG_NUT_CPU_CAP2_CORES],
        ifu_reset[CFG_NUT_CPU_CORES]; // todo: remove ifu_reset for mode 1 only CoPUs
    sc_signal<sc_uint<32> > ifu_jump_adr[CFG_NUT_CPU_CAP2_CORES]; // jump adress
    sc_signal<bool> ifu_ir_valid[CFG_NUT_CPU_CAP2_CORES], ifu_npc_valid[CFG_NUT_CPU_CAP2_CORES];
    sc_signal<sc_uint<32> > ifu_ir[CFG_NUT_CPU_CAP2_CORES], ifu_pc[CFG_NUT_CPU_CAP2_CORES],
        ifu_npc[CFG_NUT_CPU_CAP2_CORES]; // expected to be registered (fast) outputs
    sc_signal<bool> ifu_ac_x[CFG_NUT_CPU_CAP2_CORES], ifu_ac_u[CFG_NUT_CPU_CAP2_CORES];

    //   LSU ...
    sc_signal<bool> lsu_rd[CFG_NUT_CPU_CORES], lsu_wr[CFG_NUT_CPU_CORES], lsu_flush[CFG_NUT_CPU_CORES];
    sc_signal<bool> lsu_trap_u[CFG_NUT_CPU_CORES], lsu_trap_no_u[CFG_NUT_CPU_CORES];
    sc_signal<bool> lsu_ac_r[CFG_NUT_CPU_CORES], lsu_ac_w[CFG_NUT_CPU_CORES], lsu_ac_u[CFG_NUT_CPU_CORES];
    sc_signal<bool> lsu_lres_scond[CFG_NUT_CPU_CORES];
    sc_signal<sc_uint<3> > lsu_cache_op[CFG_NUT_CPU_CORES];

    sc_signal<bool> lsu_ack[CFG_NUT_CPU_CORES], lsu_align_err[CFG_NUT_CPU_CORES],
        lsu_scond_ok[CFG_NUT_CPU_CORES];
    sc_signal<sc_uint<2> > lsu_width[CFG_NUT_CPU_CORES]; // "00" = word, "01" = byte, "10" = half word
    sc_signal<bool> lsu_exts[CFG_NUT_CPU_CORES];
    sc_signal<sc_uint<32> > lsu_adr[CFG_NUT_CPU_CORES];
    sc_signal<sc_uint<32> > lsu_rdata[CFG_NUT_CPU_CORES];
    sc_signal<sc_uint<32> > lsu_wdata[CFG_NUT_CPU_CORES];

    // from CePU
    sc_signal<sc_uint<CFG_NUT_CPU_CORES> > m3_pnce;
    sc_signal<sc_uint<2> > m3_priv_mode;
    sc_signal<sc_uint<CFG_NUT_CPU_CORES> > m3_pnlm;
    sc_signal<sc_uint<32> > cepu_pnifadr;
    sc_signal<sc_uint<CFG_NUT_CPU_CORES> > m3_pnifadren;
    sc_signal<sc_uint<CFG_NUT_CPU_CORES> > m3_pnxsel;
    // to CePU
    sc_signal<bool> m3_ir_request;
    sc_signal<sc_uint<5> > m3_ir_id;
    sc_signal<sc_uint<CFG_NUT_CPU_CORES> > m3_pnhaltreq;
    sc_signal<sc_uint<CFG_NUT_CPU_CORES> > m3_pnx;
    sc_signal<bool> m3_ex_i;
    // others...
    sc_signal<bool> m3_icache_enable, m3_dcache_enable;
    sc_signal<bool> m3_ir_ack;
    sc_signal<bool> m3_ir_enable;

    // NEU
    sc_signal<bool> mtimer_ack_i;
    sc_signal<sc_uint<CFG_MEMU_BUSIF_WIDTH> > mtimer_dat_i;
    sc_signal<bool> mtimer_ir_request;
    sc_signal<bool> mtimer_irq_enable_in;
    sc_signal<bool> csr_mtip_in;

    //   EXU ...
    sc_signal<bool> exu_haltreq[CFG_NUT_CPU_CORES];
    sc_signal<bool> exu_enable[CFG_NUT_CPU_CORES];
    sc_signal<bool> exu_ifu_reset[CFG_NUT_CPU_CORES];
    sc_signal<bool> exu_linked[CFG_NUT_CPU_CORES];
    sc_signal<bool> exu_ex_o[CFG_NUT_CPU_CORES];
    sc_signal<bool> exu_xsel[CFG_NUT_CPU_CORES];
    sc_signal<sc_uint<5> > exu_cause[CFG_NUT_CPU_CORES - 1];
    sc_signal<bool> exu_sync[CFG_NUT_CPU_CORES];
    sc_signal<bool> exu_dbg_req[CFG_NUT_CPU_CORES];

    //   DM ...
    sc_signal<sc_uint<CFG_NUT_CPU_CORES> > dbg_req;
    sc_signal<bool> dbg_reset;
    // from DM
    sc_signal<bool> dbg_ack_i;
    sc_signal<sc_uint<CFG_MEMU_BUSIF_WIDTH> > dbg_dat_i;

    //   DTM ...
    sc_signal<sc_uint<DTM_ADDR_WIDTH> > dmi_adr; // address output
    sc_signal<sc_uint<32> > dmi_dat_o; // output data
    sc_signal<sc_uint<32> > dmi_dat_i; // input data
    sc_signal<bool> dmi_rd, dmi_wr;

    // EXU-CSR
    sc_signal<sc_uint<32> > csr_pc[CFG_NUT_CPU_CORES], csr_ir[CFG_NUT_CPU_CORES];
    sc_signal<sc_uint<CFG_NUT_CPU_CORES_LD> > csr_hartID[CFG_NUT_CPU_CORES];
    sc_signal<bool> csr_inCePU[CFG_NUT_CPU_CORES];
    sc_signal<bool> csr_linked[CFG_NUT_CPU_CORES];
    sc_signal<bool> csr_cpu_enabled[CFG_NUT_CPU_CORES];
    sc_signal<bool> dbg[CFG_NUT_CPU_CORES], dbg_reg[CFG_NUT_CPU_CORES], dbg_enter_dreg[CFG_NUT_CPU_CORES];
    sc_signal<bool> csr_enable[CFG_NUT_CPU_CORES];
    sc_signal<sc_uint<5> > ex_id_reg[CFG_NUT_CPU_CORES];
    sc_signal<bool> sret_dreg[CFG_NUT_CPU_CORES];
    sc_signal<bool> exception[CFG_NUT_CPU_CORES];
    sc_signal<bool> irq_dreg[CFG_NUT_CPU_CORES];
    sc_signal<bool> pop_priv_ir_stack_dreg[CFG_NUT_CPU_CORES];
    sc_signal<sc_uint<5> > csr_rs1_reg[CFG_NUT_CPU_CORES];
    sc_signal<sc_uint<32> > csr_op_a[CFG_NUT_CPU_CORES];
    sc_signal<sc_uint<3> > csr_function_reg[CFG_NUT_CPU_CORES];
    sc_signal<sc_uint<12> > csr_adr_reg[CFG_NUT_CPU_CORES];
    sc_signal<sc_uint<32> > csr_tval[CFG_NUT_CPU_CORES];
    sc_signal<sc_uint<CFG_NUT_CPU_CORES> >
        csr_m3_pnce[CFG_NUT_CPU_CORES],
        csr_m3_pnlm[CFG_NUT_CPU_CORES],
        csr_m3_pnxsel[CFG_NUT_CPU_CORES];
    sc_signal<sc_uint<2> > csr_m3_priv_mode[CFG_NUT_CPU_CORES];
    sc_signal<bool>
        csr_m3_icache_enable[CFG_NUT_CPU_CORES],
        csr_m3_dcache_enable[CFG_NUT_CPU_CORES];
    sc_signal<bool> csr_isHalted[CFG_NUT_CPU_CORES];

    sc_signal<sc_uint<2> > priv_mode[CFG_NUT_CPU_CORES], load_store_priv_mode[CFG_NUT_CPU_CORES];
    sc_signal<bool> csr_mstatus_MIE[CFG_NUT_CPU_CORES];
    sc_signal<bool> csr_mstatus_SIE[CFG_NUT_CPU_CORES];
    sc_signal<bool> csr_mstatus_TSR[CFG_NUT_CPU_CORES];
    sc_signal<bool> csr_mstatus_SUM[CFG_NUT_CPU_CORES];
    sc_signal<bool> csr_mstatus_TVM[CFG_NUT_CPU_CORES];
    sc_signal<sc_uint<32> >
            csr_sepc[CFG_NUT_CPU_CORES],
            csr_stvec[CFG_NUT_CPU_CORES],
            csr_mideleg[CFG_NUT_CPU_CORES],
            csr_mip[CFG_NUT_CPU_CORES];
    sc_signal<bool> delegate_dreg[CFG_NUT_CPU_CORES];

    sc_signal<bool> csr_exception[CFG_NUT_CPU_CORES];

    sc_signal<bool>
            csr_dcsr_step[CFG_NUT_CPU_CORES],
            csr_dcsr_ebreakm[CFG_NUT_CPU_CORES];

    sc_signal<sc_uint<32> >
            csr_mepc[CFG_NUT_CPU_CORES],
            csr_dpc[CFG_NUT_CPU_CORES],
            csr_mtvec[CFG_NUT_CPU_CORES],
            csr_mcause[CFG_NUT_CPU_CORES],
            csr_rdata[CFG_NUT_CPU_CORES];
    sc_signal<sc_uint<5> > csr_cause[CFG_NUT_CPU_CORES];

    sc_signal<bool> csr_ack[CFG_NUT_CPU_CORES];
    sc_signal<bool> csr_cache_flush[CFG_NUT_CPU_CORES]; 

    sc_signal<sc_uint<CFG_NUT_CPU_CORES> > csr_m3_pnhaltreq[CFG_NUT_CPU_CORES], csr_m3_pnx[CFG_NUT_CPU_CORES];

    sc_signal<bool> csr_ifu_paging_mode[CFG_NUT_CPU_CORES], csr_lsu_paging_mode[CFG_NUT_CPU_CORES];
    sc_signal<bool> ifu_paging_mode, lsu_paging_mode;
    sc_signal<sc_uint<20> > root_ppn;
    sc_signal<bool> csr_satp_mode[CFG_NUT_CPU_CORES];
    sc_signal<sc_uint<20> > csr_satp_root_ppn[CFG_NUT_CPU_CORES];

    sc_signal<bool> tlb_flush;

    
#if CFG_EXU_PERFCOUNT_ENABLE == 1
    sc_signal<bool> perf_inc[CFG_NUT_CPU_CORES];
    sc_signal<sc_uint<CFG_EXU_PERFCOUNTERS_LD> > perf_addr[CFG_NUT_CPU_CORES];
#endif

    //   INTC ...
    sc_signal<sc_uint<CFG_NUT_EX_INT> > intc_ex_int;
    //   INTC-CSR
    sc_signal<bool> csr_mip_MTIP;
    sc_signal<bool> csr_mip_MEIP;

    //   ParaNut reset ...
    sc_signal<bool> reset;

    // Methods...
    void InitSubmodules ();
    void FreeSubmodules ();

    // Helpers...
};


#endif
