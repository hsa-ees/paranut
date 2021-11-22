/*************************************************************************

  This file is part of the ParaNut project.

  Copyright (C) 2010-2020 Alexander Bahle <alexander.bahle@hs-augsburg.de>
                          Christian H. Meyer <christian.meyer@hs-augsburg.de>
                          Gundolf Kiefer <gundolf.kiefer@hs-augsburg.de>
      Hochschule Augsburg, University of Applied Sciences

  Description:
    This is a SystemC model of the CSR module (CSR) of the ParaNut.
    When accessing an CSR from the EXU the methods of this module
    are called. It is used as a submodule of the EXU.

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

/*
* TODO: create CSR wrapper
* route similar to SystemC wrapper in VHDL
*
*/

#pragma once

#include "base.h"
#include "paranut-config.h"
#include "exu_csr.h"


#if CFG_EXU_PERFCOUNT_ENABLE == 1
#define CYCLEREG_LOW (csr_mcycle.read () (31, 0))
#define CYCLEREG_HIGH (csr_mcycle.read () (63, 32))
#define PERFREGS(COUNTER) csr_mhpmcounter[COUNTER].read ()
#else // CFG_EXU_PERFCOUNT_ENABLE != 1
#define PERFREGS(COUNTER) sc_uint<CFG_EXU_PERFCOUNTER_BITS> (0x0)
#define CYCLEREG_LOW 0x0
#define CYCLEREG_HIGH 0x0
#endif
// **************** CSR Function Codes *************
typedef enum {
    CSRRW = 1,
    CSRRS,
    CSRRC,
} ECSRFunc;

// **************** Special Registers ***********
typedef enum {
    // Machine Information Registers
    mvendorid = 0xF11,
    marchid,
    mimpid,
    mhartid,

    // Machine Trap Setup
    mstatus = 0x300,
    misa,
    medeleg,              // medeleg and mideleg may only exist if S-Mode
    mideleg,              // or N-Extension are available
    mie,
    mtvec,

    // Machine Trap Handling
    mscratch = 0x340,
    mepc,
    mcause,
    mtval,
    mip,

    // Machine Counter/Timers
    mcycle = 0xB00,
    minstret = 0xB02,
    mhpmcounter3,
    mhpmcounter4,
    mhpmcounter5,
    mhpmcounter6,
    mhpmcounter7,
    mhpmcounter8,
    //... todo: implement 9 ... 30
    mhpmcounter31 = 0xB1F,
    mcycleh = 0xB80,
    minstreth = 0xB82,
    mhpmcounter3h,
    mhpmcounter4h,
    mhpmcounter5h,
    mhpmcounter6h,
    mhpmcounter7h,
    mhpmcounter8h,
    //... todo: implement 9 ... 30
    mhpmcounter31h = 0xB9F,

    // Machine Counter Setup
    mhpmevent3 = 0x323,
    //... todo: implement 4 ... 30
    mhpmevent31 = 0x33F,

    // Standard
    mtime = 0xF01, // 64 bit memory mapped timer register
    mtimeh,
    mtimecmp, // 64 bit memory mapped timer compare register
    mtimecmph,

    // Supervisor Trap Setup
    sstatus = 0x100,
    sedeleg = 0x102,
    sideleg,
    sie,
    stvec,
    scounteren,

    // Supervisor Trap Handling
    sscratch = 0x140,
    sepc,
    scause,
    stval,
    sip,

    // Supervisor Protection and Translation
    satp = 0x180,

    // User Trap Setup
    ustatus = 0x000,
    uie = 0x004,
    utvec,

    // User Trap Handling
    uscratch = 0x040,
    uepc,
    ucause,
    utval,
    uip,

    // User Counter/Timers
    ucycle = 0xC00,
    utime,
    uinstret,
    uhpmcounter3,
    uhpmcounter4,
    uhpmcounter5,
    uhpmcounter6,
    uhpmcounter7,
    uhpmcounter8,
    uhpmcounter9,
    uhpmcounter10,
    uhpmcounter11,
    uhpmcounter12,
    uhpmcounter13,
    uhpmcounter14,
    uhpmcounter15,
    uhpmcounter16,
    uhpmcounter17,
    uhpmcounter18,
    uhpmcounter19,
    uhpmcounter20,
    uhpmcounter21,
    uhpmcounter22,
    uhpmcounter23,
    uhpmcounter24,
    uhpmcounter25,
    uhpmcounter26,
    uhpmcounter27,
    uhpmcounter28,
    uhpmcounter29,
    uhpmcounter30,
    uhpmcounter31,
    ucycleh,
    utimeh,
    uinstreth,
    uhpmcounter3h,
    uhpmcounter4h,
    uhpmcounter5h,
    uhpmcounter6h,
    uhpmcounter7h,
    uhpmcounter8h,
    uhpmcounter9h,
    uhpmcounter10h,
    uhpmcounter11h,
    uhpmcounter12h,
    uhpmcounter13h,
    uhpmcounter14h,
    uhpmcounter15h,
    uhpmcounter16h,
    uhpmcounter17h,
    uhpmcounter18h,
    uhpmcounter19h,
    uhpmcounter20h,
    uhpmcounter21h,
    uhpmcounter22h,
    uhpmcounter23h,
    uhpmcounter24h,
    uhpmcounter25h,
    uhpmcounter26h,
    uhpmcounter27h,
    uhpmcounter28h,
    uhpmcounter29h,
    uhpmcounter30h,
    uhpmcounter31h,

    // Debug
    dcsr = 0x7b0,
    dpc,
    dscratch0,
    tselect = 0x7a0,

    // Non-Standard machine read/write paranut:
    pncache = 0x7C0,

    // Non-Standard user read/write paranut:
    pngrpsel = 0x8C0,
    pnce,
    pnlm,
    pnxsel,

    // Non-Standard machine read-only paranut:
    pnm2cp = 0xFC0,
    pnx,
    pncause,
    pnepc,
    pncacheinfo,
    pncachesets,
    pnclockinfo,
    pnmemsize,

    // Non-Standard user read-only paranut:
    pncpus = 0xCD0,
    pncoreid = 0xCD4
} ECSR;

SC_MODULE (MCsr) {
public:
    // Ports ...
    sc_in<bool> clk, reset;
    sc_in<TWord> pc, ir;
    sc_in<bool> exu_pop_priv_ir_stack_dreg; // trigger pushing the Privilege and Global Interrupt-Enable Stack

    sc_in<sc_uint<EX_ID_LENGTH> > ex_id_reg;
    sc_in<bool> exception;  // signal from ExU. Triggers saving of the exception state CSRs (mepc, mcause, ...)
    sc_in<bool> irq_dreg;  // Set if m3_ir_request is set during instruction decode (handle interrupt request)

    // misc (HLS reasons, should be optimized out while synthetization)
    // these could be generics in VHDL, but current tools don't support that
    sc_in<TWord> hartID, clock_freq_hz;
    sc_in<bool> inCePU;
    sc_in<bool> cpu_enabled, linked;
    sc_in<bool> csr_enable;
    sc_in<sc_uint<5> > exu_cause;

    sc_in<bool> exu_dbg, exu_dbg_reg, exu_dbg_req;

    // Decode Registers
    sc_out<bool> exu_delegate_dreg;
    sc_in<bool> mret_dreg;

    sc_out<bool>
            exu_csr_exception, // Writes to read-only CSRs raise an exeption
            exu_csr_rd_exception; // Reads to non-existent CSRs raise an exceptions

    sc_out<bool>
            exu_csr_mstatus_MIE, // Machine Interrupt Enable
            exu_csr_dcsr_step,
            exu_csr_dcsr_ebreakm;
    sc_out<bool> exu_isHalted;

    // Full registers
    sc_out<sc_uint<32> >
            exu_csr_mepc,
            exu_csr_dpc,
            exu_csr_mtvec,
            exu_csr_mcause,
            exu_csr_sepc,
            exu_csr_stvec,
            exu_csr_mideleg;


    sc_out<bool> exu_csr_mstatus_SIE;
    
    sc_out<sc_uint<CFG_NUT_CPU_CORES> > exu_m3_pnce;
    sc_out<sc_uint<CFG_NUT_CPU_CORES> > exu_m3_pnlm;
    sc_out<sc_uint<CFG_NUT_CPU_CORES> > exu_m3_pnxsel;

    // controller outputs...
    sc_out<bool> exu_m3_icache_enable, exu_m3_dcache_enable;

    sc_in<sc_uint<32> > csr_tval;   // Special signal for exception information input
    sc_in<sc_uint<CFG_NUT_CPU_CORES> > m3_pnhaltreq;

    sc_in<sc_uint<3> > csr_function_reg;
    sc_in<sc_uint<5> > csr_rs1_reg;
    sc_in<sc_uint<12> > csr_adr_reg;
    sc_in<sc_uint<32> > csr_op_a;
    sc_out<sc_uint<32> > exu_csr_rdata;
    
    sc_out<sc_uint<2> > exu_priv_mode_reg;

    // Exception signals
    sc_in<sc_uint<32> > exu_epc;

    // from CoPUs
    sc_in<sc_uint<CFG_NUT_CPU_CORES> > m3_pnx;

#if CFG_EXU_PERFCOUNT_ENABLE == 1
    sc_in<bool> perf_inc;
    sc_in<sc_uint<CFG_EXU_PERFCOUNTERS_LD> > perf_addr;
#endif

    // Functions
    void Trace (sc_trace_file * tf, int levels = 1);

    // Processes...
    void CSRHandleMethod ();
    void CSRReadMethod ();
    void CSRWriteMethod ();
    void OutputMethod ();

    // Regular Methods
    void setMstatus (sc_uint<32> wdata);
    sc_uint<32> getMstatus ();

    SC_CTOR (MCsr) {
        SC_METHOD (CSRHandleMethod);
            sensitive << csr_enable << csr_function_reg << csr_op_a << csr_rs1_reg
                      << csr_rdata << csr_rd_exception << csr_adr_reg;
        SC_METHOD (CSRReadMethod);
            sensitive << csr_adr_reg << csr_enable << csr_mepc << csr_mcause << csr_mtval
                      << csr_mtvec << csr_mscratch << csr_mstatus_MIE << csr_mstatus_MPIE
                      << exu_cause << exu_epc << m3_pnx << hartID << inCePU << cpu_enabled
                      << linked << csr_pncache << clock_freq_hz;
            for (int i = 0; i < CFG_NUT_CPU_GROUPS + 1; i++)
                sensitive << csr_pnce[i] << csr_pnlm[i] << csr_pnxsel[i] << csr_pnx[i];
#if CFG_NUT_CPU_GROUPS
            sensitive << csr_pngrpsel;
#endif
        SC_METHOD (CSRWriteMethod);
            sensitive << clk.pos ();
        SC_METHOD (OutputMethod);
            sensitive << csr_exception << csr_rd_exception << csr_mstatus_MIE << csr_dcsr_step
                      << csr_dcsr_ebreakm << csr_mcause << csr_rdata << csr_mepc << csr_dpc
                      << csr_mtvec << priv_mode_reg << csr_mstatus_SIE << csr_satp << csr_sepc
                      << csr_stvec << csr_mideleg << delegate_dreg << csr_pncache;
            for (int i = 0; i < CFG_NUT_CPU_GROUPS + 1; i++) {
                sensitive << csr_pnce[i] << csr_pnlm[i] << csr_pnx[i] << csr_pnxsel[i];
            }

#if CFG_EXU_PERFCOUNT_ENABLE == 1
        // Add PerfCountMethod
        SC_METHOD (PerfcountMethod);
            sensitive << clk.pos ();
#endif
    }

protected:
    sc_signal<sc_uint<32> >
            csr_wdata,
            csr_rdata;

    sc_signal<bool>
            csr_exception, // Writes to read-only CSRs raise an exeption
            csr_rd_exception, // Reads to non-existent CSRs raise an exceptions
            csr_write;

    sc_signal<sc_uint<2> > priv_mode_reg;

    // Decode Registers
    sc_signal<bool> delegate_dreg;      // jump to stvec when set on exception

    // CSR registers
    sc_signal<bool>
            csr_dcsr_step,
            csr_dcsr_ebreakm;
    sc_signal<sc_uint<3> > csr_dcsr_cause;

    sc_signal<bool>
            csr_mip_MEIP, // Machine External Interrupt Pending
//            csr_mip_MTIP,    // Machine Timer Interrupt Pending
//            csr_mie_MTIE,    // Machine Timer Interrupt Enable
            csr_mie_MEIE; // Machine External Interrupt Enable

    sc_signal<bool>
            csr_mstatus_MPIE, // Machine Previous Interrupt Enable
            csr_mstatus_MIE, // Machine Interrupt Enable
            csr_mstatus_SIE, // Supervisor Interrupt Enable
            csr_mstatus_SPIE, // Supervisor PreviousInterrupt Enable
            csr_mstatus_SPP, // Supervisor Previous Privilege
            csr_mstatus_SUM, // permit Supervisor User Memory access
            csr_mstatus_MXR, // Make eXecutable Readable
            csr_mstatus_TVM, // Trap Virtual Memory
            csr_mstatus_TW, // Timeout Wait
            csr_mstatus_TSR, // Trap SRET
            csr_mstatus_SD; // State dirty
    sc_signal<sc_uint<2> >
            csr_mstatus_MPP, // Machine Previous Privilege
            csr_mstatus_XS; // status of additional user-mode extensions

    sc_signal<sc_uint<32> >
            csr_medeleg,
            csr_mideleg,
            csr_stvec,
            csr_scause,
            csr_satp,
            csr_stval,
            csr_sepc,
            csr_sscratch,
            csr_mip,
            csr_mie,
            csr_mcause,
            csr_dpc,
            csr_dscratch0,
            csr_mepc,
            csr_mtval,
            csr_mtvec,
            csr_mscratch;

    // If there are more than 32 CPUs only full 32 bit CSRs are supported/possible at the moment
    sc_signal<sc_uint<MIN (XLEN, CFG_NUT_CPU_CORES)> >
            csr_pnce[CFG_NUT_CPU_GROUPS + 1],
            csr_pnlm[CFG_NUT_CPU_GROUPS + 1],
            csr_pnx[CFG_NUT_CPU_GROUPS + 1],
            csr_pnxsel[CFG_NUT_CPU_GROUPS + 1];

#if CFG_NUT_CPU_GROUPS > 0
    // With more than 32 CPUs create a csr_pngrpsel signal
    sc_signal<sc_uint<CFG_NUT_CPU_GROUPS> > csr_pngrpsel;
#else
    // With less than 32 CPUs don't create a csr_pngrpsel signal, just static const variable
    static const bool csr_pngrpsel = 0x0;
#endif
    sc_signal<sc_uint<2> > csr_pncache;

#if CFG_EXU_PERFCOUNT_ENABLE == 1
    // Perfcount Method ...
    void PerfcountMethod ();
    // Perfcount signals ...
    sc_signal<sc_uint<64> > csr_mcycle;
    sc_signal<sc_uint<CFG_EXU_PERFCOUNTER_BITS> > csr_mhpmcounter[CFG_EXU_PERFCOUNTERS];
#endif
};
