/*************************************************************************

  This file is part of the ParaNut project.

  Copyright (C) 2010-2022 Alexander Bahle <alexander.bahle@hs-augsburg.de>
                          Christian H. Meyer <christian.meyer@hs-augsburg.de>
                          Gundolf Kiefer <gundolf.kiefer@hs-augsburg.de>
                          Nico Borgsmüller <nico.borgsmueller@hs-augsburg.de>
                2022-2023 Lukas Bauer <lukas.bauer@hs-augsburg.de>
      Hochschule Augsburg, University of Applied Sciences

  Description:
    This is a SystemC model of the CSR module (CSR) of the ParaNut.
    When accessing an CSR from the EXU, the methods of this module
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

#pragma once

#include "paranut-peripheral.h"

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
    mconfigptr, // not functional (not implemented)

    // Machine Trap Setup
    mstatus = 0x300,
    misa,
    medeleg,              // medeleg and mideleg may only exist if S-Mode
    mideleg,              // or N-Extension are available
    mie,
    mtvec,
    mcounteren,
    mstatush = 0x310,

    // Machine Trap Handling
    mscratch = 0x340,
    mepc,
    mcause,
    mtval,
    mip,

    // Machine Configuration
    menvcfg = 0x30A,
    menvcfgh = 0x31A,

    // Machine Counter/Timers
    mcycle = 0xB00,
    minstret = 0xB02,
    mhpmcounter3,
    mhpmcounter4,
    mhpmcounter5,
    mhpmcounter6,
    mhpmcounter7,
    mhpmcounter8,

    // without any functionality (not implemented)
    // listed to prevent read errors when using gdb
    mhpmcounter9,
    mhpmcounter10,
    mhpmcounter11,
    mhpmcounter12,
    mhpmcounter13,
    mhpmcounter14,
    mhpmcounter15,
    mhpmcounter16,
    mhpmcounter17,
    mhpmcounter18,
    mhpmcounter19,
    mhpmcounter20,
    mhpmcounter21,
    mhpmcounter22,
    mhpmcounter23,
    mhpmcounter24,
    mhpmcounter25,
    mhpmcounter26,
    mhpmcounter27,
    mhpmcounter28,
    mhpmcounter29,
    mhpmcounter30,
    mhpmcounter31 = 0xB1F,


    mcycleh = 0xB80,
    minstreth = 0xB82,
    mhpmcounter3h,
    mhpmcounter4h,
    mhpmcounter5h,
    mhpmcounter6h,
    mhpmcounter7h,
    mhpmcounter8h,

    // without any functionality (not implemented)
    // listed to prevent read errors when using gdb
    mhpmcounter9h,
    mhpmcounter10h,
    mhpmcounter11h,
    mhpmcounter12h,
    mhpmcounter13h,
    mhpmcounter14h,
    mhpmcounter15h,
    mhpmcounter16h,
    mhpmcounter17h,
    mhpmcounter18h,
    mhpmcounter19h,
    mhpmcounter20h,
    mhpmcounter21h,
    mhpmcounter22h,
    mhpmcounter23h,
    mhpmcounter24h,
    mhpmcounter25h,
    mhpmcounter26h,
    mhpmcounter27h,
    mhpmcounter28h,
    mhpmcounter29h,
    mhpmcounter30h,
    mhpmcounter31h = 0xB9F,

    // Machine Counter Setup
    // without any functionality (not implemented)
    // listed to prevent read errors when using gdb
    mcounterinhibit = 0x320,
    mhpmevent3 = 0x323,
    mhpmevent4,
    mhpmevent5,
    mhpmevent6,
    mhpmevent7,
    mhpmevent8,
    mhpmevent9,
    mhpmevent10,
    mhpmevent11,
    mhpmevent12,
    mhpmevent13,
    mhpmevent14,
    mhpmevent15,
    mhpmevent16,
    mhpmevent17,
    mhpmevent18,
    mhpmevent19,
    mhpmevent20,
    mhpmevent21,
    mhpmevent22,
    mhpmevent23,
    mhpmevent24,
    mhpmevent25,
    mhpmevent26,
    mhpmevent27,
    mhpmevent28,
    mhpmevent29,
    mhpmevent30,
    mhpmevent31 = 0x33F,

    // Supervisor Trap Setup
    sstatus = 0x100,
    sedeleg = 0x102,
    sideleg,
    sie,
    stvec,
    scounteren,

    // Supervisor Configuration
    senvcfg = 0x10A, //not functional (not implemented)

    // Supervisor Trap Handling
    sscratch = 0x140,
    sepc,
    scause,
    stval,
    sip,

    // Supervisor Protection and Translation
    satp = 0x180,

    // Debug/Trace Registers
    scontext = 0x5A8,

    // registers with the prefix "u" are the csr registers defined for the
    // user/unprivleged Mode. In the specification there prefix "u". This means
    // that e.g. "ucycle" in the ParaNut is "cycle" in the ISA Specification.
    // This prefix is only present in the SystemC Code. But is not used when
    // accessing a csr register with an assembler command. This is done to prevent
    // collisions with the C Function "time" with the csr "time". And to keep a common
    // naming convention for all user/unprivleged mode csr registers.

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
    ucycleh = 0xC80, // not tested if ok
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
    pncacheinfo = 0xFC4,
    pncachesets,
    pnclockinfo,
    pnmemsize,
    pnece,
    pntimebase,

    // Non-Standard user read-only paranut:
    pncpus = 0xCD0,
    pncoreid = 0xCD4,

    // Non-Standard User read/write paranut:
    pnepc = 0x841,
    pncause = 0x842,
} ECSR;


SC_MODULE (MCsr) {
public:
    // Ports ...
    sc_in<bool> clk, reset;
    sc_in<sc_uint<32> > pc, ir;
    sc_in<bool> exu_pop_priv_ir_stack_dreg; // trigger pushing the Privilege and Global Interrupt-Enable Stack

    sc_in<sc_uint<EX_ID_LENGTH> > ex_id_reg;
    sc_in<bool> exception;  // signal from ExU. Triggers saving of the exception state CSRs (mepc, mcause, ...)
    sc_in<bool> irq_dreg;  // Set if m3_ir_request is set during instruction decode (handle interrupt request)

    sc_out<bool> mtie_out;  // mtie bit of mie register is set
    sc_in<bool> mtip_in;    // mtip bit of mip register
    sc_in<bool> meip_in;    // meip bit of mip register

    // misc (HLS reasons, should be optimized out while synthetization)
    // these could be generics in VHDL, but current tools don't support that
    sc_in<sc_uint<CFG_NUT_CPU_CORES_LD> > hartID;
    sc_in<bool> inCePU;
    sc_in<bool> cpu_enabled, linked;
    sc_in<bool> csr_enable;
    sc_in<sc_uint<5> > exu_cause;

    sc_in<bool> exu_dbg, exu_dbg_reg, exu_dbg_req, exu_dbg_enter_dreg;

    // Decode Registers
    sc_out<bool> exu_delegate_dreg;
    sc_in<bool> sret_dreg;

    sc_out<bool> exu_csr_exception; // Writes to read-only CSRs raise an exeption

    sc_out<bool>
            exu_csr_mstatus_MIE, // Machine Interrupt Enable
            exu_csr_mstatus_SIE,
            exu_csr_mstatus_TSR, // Trap SRET
            exu_csr_mstatus_TVM, // Trap Virtual Memory
            exu_csr_mstatus_SUM,
            exu_csr_dcsr_step,
            exu_csr_dcsr_ebreakm;
    sc_out<bool> exu_isHalted;

    sc_out<sc_uint<2> > exu_load_store_priv_mode;

    // Full registers
    sc_out<sc_uint<32> >
            exu_csr_mepc,
            exu_csr_dpc,
            exu_csr_mtvec,
            exu_csr_mcause,
            exu_csr_sepc,
            exu_csr_stvec,
            exu_csr_mideleg,
            exu_csr_mip;


    sc_out<bool> exu_ifu_paging_mode; // for instructions
    sc_out<bool> exu_lsu_paging_mode; // for load-store OPs
    sc_out<sc_uint<20> > exu_csr_satp_root_ppn;

    sc_out<sc_uint<CFG_NUT_CPU_CORES> > exu_m3_pnce;
    sc_out<sc_uint<CFG_NUT_CPU_CORES> > exu_m3_pnlm;
    sc_out<sc_uint<CFG_NUT_CPU_CORES> > exu_m3_pnxsel;
    sc_in<sc_uint<2> > exu_m3_priv_mode;

    // EXU attention signals
    sc_out<bool> exu_cache_flush;
    sc_in<bool> exu_ack;

    // controller outputs...
    sc_out<bool> exu_m3_icache_enable, exu_m3_dcache_enable;

    sc_in<sc_uint<32> > csr_tval;   // Special signal for exception information input
    sc_in<sc_uint<CFG_NUT_CPU_CORES> > m3_pnhaltreq;

    sc_in<sc_uint<3> > csr_function_reg;
    sc_in<sc_uint<5> > csr_rs1_reg;
    sc_in<sc_uint<12> > csr_adr_reg;
    sc_in<sc_uint<32> > csr_op_a;
    sc_out<sc_uint<32> > exu_csr_rdata;

    sc_out<sc_uint<2> > exu_priv_mode;

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
    void setSstatus (sc_uint<32> wdata);
    sc_uint<32> getSstatus ();

    SC_HAS_PROCESS (MCsr);
    MCsr (sc_module_name name = "MCsr" ){
        SC_METHOD (CSRHandleMethod);
            sensitive << csr_enable << csr_function_reg << csr_op_a << csr_rs1_reg
                      << csr_rdata << csr_rd_exception << csr_adr_reg
                      << mtip_in;
        SC_METHOD (CSRReadMethod);
            sensitive << csr_adr_reg << csr_enable << csr_mepc << csr_mcause << csr_mtval
                      << csr_mtvec << csr_mscratch << csr_mstatus_MIE << csr_mstatus_MPIE
                      << exu_cause << m3_pnx << hartID << inCePU << cpu_enabled
                      << linked << csr_pncache << csr_mstatus_MPRV << csr_mstatus_MPP;
            for (int i = 0; i < CFG_NUT_CPU_GROUPS + 1; i++)
                sensitive << csr_pnce[i] << csr_pnece[i] << csr_pnlm[i] << csr_pnxsel[i] << csr_pnx[i];
#if CFG_NUT_CPU_GROUPS
            sensitive << csr_pngrpsel;
#endif
        SC_METHOD (CSRWriteMethod);
            sensitive << clk.pos ();
        SC_METHOD (OutputMethod);
            sensitive << csr_exception << csr_rd_exception << exu_cache_flush_reg
                      << csr_mstatus_MIE << exu_csr_mstatus_SUM << csr_mstatus_TSR << csr_mstatus_TVM << csr_mstatus_SIE
                      << csr_mtvec << priv_mode_reg << csr_satp_root_ppn << csr_satp_mode
                      << csr_sepc << csr_stvec << csr_mideleg << delegate_dreg << csr_pncache
                      << csr_dcsr_step << csr_dcsr_ebreakm << csr_mcause << csr_rdata << csr_mepc << csr_dpc
                      << csr_mie_MTIE << csr_mstatus_MPP << csr_mstatus_MPRV;
            for (int i = 0; i < CFG_NUT_CPU_GROUPS + 1; i++) {
                sensitive << csr_pnce[i] << csr_pnece[i] << csr_pnlm[i] << csr_pnx[i] << csr_pnxsel[i];
            }

#if CFG_EXU_PERFCOUNT_ENABLE == 1
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

    sc_signal<sc_uint<2> > priv_mode_reg, next_priv_mode_reg;

    // Decode Registers
    sc_signal<bool> delegate_dreg;      // jump to stvec when set on exception
    sc_signal<bool> exu_cache_flush_reg;      // jump to stvec when set on exception

    // CSR registers
    sc_signal<bool>
            csr_dcsr_step,
            csr_dcsr_ebreakm;
    sc_signal<sc_uint<3> > csr_dcsr_cause;
    sc_signal<sc_uint<2> > csr_dcsr_prv;

    sc_signal<bool>
            csr_mip_MEIP, // Machine External Interrupt Pending
            csr_mip_MTIP,    // Machine Timer Interrupt Pending
            csr_mie_MTIE,    // Machine Timer Interrupt Enable
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
            csr_mstatus_SD, // State dirty
            csr_mstatus_MPRV; // Modify Privilege
    sc_signal<sc_uint<2> >
            csr_mstatus_MPP, // Machine Previous Privilege
            csr_mstatus_XS; // status of additional user-mode extensions

    sc_signal<bool> csr_satp_mode;
    sc_signal<sc_uint<20> > csr_satp_root_ppn;

    sc_signal<sc_uint<32> >
            csr_medeleg,
            csr_mideleg,
            csr_stvec,
            csr_scause,
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
            csr_pnece[CFG_NUT_CPU_GROUPS + 1],
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
