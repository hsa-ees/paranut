/*************************************************************************

  This file is part of the ParaNut project.

  Copyright (C) 2010-2022 Alexander Bahle <alexander.bahle@hs-augsburg.de>
                          Christian H. Meyer <christian.meyer@hs-augsburg.de>
                          Gundolf Kiefer <gundolf.kiefer@hs-augsburg.de>
                          Nico Borgsmüller <nico.borgsmueller@hs-augsburg.de>
      Hochschule Augsburg, University of Applied Sciences

  Description:
    This is a SystemC model of the execution unit (EXU) of the ParaNut.
    The EXU contains the ALU, the register file, the capability to
    decode instructions. It interfaces with the IFU and the LSU.

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

#include "base.h"
#include "paranut-config.h"
#include "mextension.h"
#include "exu_csr.h"

#include <systemc.h>

#define REGISTERS 32

// **************** RISC-V ISA Encoding *************
typedef enum { RType, IType, SType, BType, UType, JType } EAluRISCVType;

typedef enum {
    LOAD = 0x03,
    PARA = 0x0B, // custom-0: ParaNut Extension
    MISC_MEM = 0x0F,
    OP_IMM = 0x13,
    AUIPC = 0x17,
    STORE = 0x23,
    OP = 0x33,
    LUI = 0x37,
    BRANCH = 0x63,
    JALR = 0x67,
    JAL = 0x6F,
    SYSTEM = 0x73,
    AMO = 0x2F,
} EOpRISCV;

typedef enum {
    // group OP and OP_IMM:
    afAdd = 0x0,
    afSub = 0x0, // AF of afAdd
    afSll = 0x1,
    afSlt = 0x2,
    afSltu = 0x3,
    afXor = 0x4,
    afSrl = 0x5,
    afSra = 0x5, // AF of afSrl
    afOr = 0x6,
    afAnd = 0x7,
    afEq = 0x8,
    afBltu = 0x9,
    afBlt = 0xA,
} EAluFuncRISCV;

// **************** EXU States *************
typedef enum {
    ExuExecuteInsn,
    ExuMem,
    ExuMemWB,
    ExuJump,
    ExuBranch,
    ExuCSR,
    ExuXRET,
    ExuXRETCsrWait,
    ExuXRETFinish,
    // ParaNut extension states:
    ExuHalt,
    ExuLSUFlush,
    ExuCACHECONT,
    ExuCACHECONT2,
    // Exception states:
    ExuExOrIrq,
    ExuExWaitForCoPUs,
    ExuExJumpTvec,
    // Special states (for extensions etc.)
#if CFG_EXU_M_EXTENSION == 1
    ExuDiv,
    ExuMulDivWB,
#endif
    // Debug states:
    ExuDBGEnter,
    ExuDBG,
} EExuState;

typedef enum {
    OpNOP,
    OpAlu,
    OpJump,
    OpBranch,
    OpMem,
    OpCSR,
    OpXRet,
    OpHalt,
    OpParaNut,
    OpException,
    OpDebug,
    OpSFenceVMA,

    // Special Operation (for extensions etc.)
#if CFG_EXU_M_EXTENSION == 1
    OpMulDiv,
#endif
} EExuOp;



// **************** MExu ************************

class MExu : public ::sc_core::sc_module {
public:

    // Ports ...
    sc_in<bool> clk, reset;

    //   to IFU ...
    sc_out<bool> ifu_next, ifu_jump, ifu_flush, ifu_reset;
    sc_out<sc_uint<32> > ifu_jump_adr; // jump address
    sc_in<bool> ifu_ir_valid, ifu_npc_valid;
    sc_in<sc_uint<32> > ifu_ir, ifu_pc, ifu_npc; // expected to be registered (fast) outputs
    sc_in<bool> ifu_ac_x, ifu_ac_u;

    //   to Load/Store Unit (LSU)...
    sc_out<bool> lsu_rd, lsu_wr, lsu_flush;
    sc_out<sc_uint<3> > lsu_cache_op;
    sc_in<bool> lsu_ack;
    sc_in<bool> lsu_align_err;
    sc_out<sc_uint<2> > lsu_width; // "00" = word, "01" = byte, "10" = half word
    sc_out<bool> lsu_exts;
    sc_out<sc_uint<32> > lsu_adr;
    sc_in<sc_uint<32> > lsu_rdata;
    sc_out<sc_uint<32> > lsu_wdata;
    sc_out<bool> lsu_lres_scond;
    sc_in<bool> lsu_scond_ok;
    sc_out<bool> lsu_trap_u, lsu_trap_no_u;
    sc_in<bool> lsu_ac_r, lsu_ac_w, lsu_ac_u;
    // TODO: omit checking the ac_u bit in the EXU. Instead, check trap_u and trap_no_u bits in MemU


    // TBD:
    //   1. Rename the following ports to properly relect their (usual) peer component.
    //      (see ports up to here)
    //   2. Complete comments accordingly

    //   to [[ TBD: specify the peer component ]]: Exception signals
    sc_in<bool> xsel;
    sc_in<bool> ex_i;
    sc_out<bool> ex_o;
    sc_in<sc_uint<5> > cause_i;
    sc_out<sc_uint<5> > cause_o;

    //   to other EXUs: Mode 1 syncronization signals
    sc_in<bool> sync_i;     // Sync daisy chain input: Connected to 'sync_o' of the next CoPU or constant '1' for the last CoPU
    sc_out<bool> sync_o;    // Sync daisy chain output: Drives 0 if this core is busy or 'sync_i' otherwise
    sc_in<bool> sync_next;  // Global signal from the CePU to indicate that the next instruction is to be executed
                            // (connected to CePU 'sync_o')

    //   to [[ TBD: specify the peer component ]]: Mode 2 linked mode signals
    sc_in<bool> m2_ir_valid;
    sc_in<sc_uint<32> > m2_ir, m2_pc;
    sc_in<bool> m2_ac_x, m2_ac_u;

    //   to CePU [[ TBD: specify the peer component; "CePU" is ambiguous ]]
    sc_in<bool> enable, linked;
    sc_out<bool> haltreq;

    //   to CoPUs [[ TBD: specify the peer component; "CePU" is ambiguous ]]
    sc_out<sc_uint<CFG_NUT_CPU_CORES> > m3_pnce;
    sc_out<sc_uint<CFG_NUT_CPU_CORES> > m3_pnlm;
    sc_out<sc_uint<CFG_NUT_CPU_CORES> > m3_pnxsel;
    sc_in<sc_uint<CFG_NUT_CPU_CORES> > m3_pnhaltreq, m3_pnx;
    sc_in<sc_uint<2> > m3_priv_mode_i;
    sc_out<sc_uint<2> > m3_priv_mode_o;

    //   to interrupt controller ...
    sc_in<bool> m3_ir_request;
    sc_in<sc_uint<5> > m3_ir_id;
    sc_out<bool> m3_ir_ack;
    sc_out<bool> m3_ir_enable;

    //   special outputs ...
    sc_out<bool> m3_icache_enable, m3_dcache_enable;
      // cache enable control signals (CePU only)

    //   to Debug Unit ...
    sc_in<bool> dbg_req;

    // to CSR Module
    sc_out<bool> csr_dbg, csr_dbg_reg, csr_dbg_enter_dreg;
    sc_out<bool> csr_cpu_enabled;
    sc_out<sc_uint<5> > csr_ex_id_reg;
    sc_out<bool> csr_sret_dreg;
    sc_out<bool> csr_exu_exception;
    sc_out<bool> csr_irq_dreg;
    sc_out<bool> csr_pop_priv_ir_stack_dreg;
    sc_out<sc_uint<5> > csr_csr_rs1_reg;
    sc_out<sc_uint<32> > csr_csr_op_a;
    sc_out<sc_uint<3> > csr_csr_function_reg;
    sc_out<sc_uint<12> > csr_csr_adr_reg;
    sc_out<sc_uint<32> > csr_csr_tval;

    sc_out<sc_uint<32> > csr_pc, csr_ir;

    sc_out<sc_uint<CFG_NUT_CPU_CORES_LD> > csr_hartID;
    sc_out<bool> csr_inCePU;
    sc_out<bool> csr_linked;
    sc_out<bool> csr_enable;
    sc_out<sc_uint<5> > csr_cause;
    sc_out<sc_uint<CFG_NUT_CPU_CORES> > csr_m3_pnhaltreq, csr_m3_pnx;
    sc_out<sc_uint<2> > csr_m3_priv_mode;

#if CFG_EXU_PERFCOUNT_ENABLE == 1
    //     Perfcount signals ...
    sc_out<bool> csr_perf_inc;
    sc_out<sc_uint<CFG_EXU_PERFCOUNTERS_LD> > csr_perf_addr;
#endif

    // to/from CSR Module
    sc_in<bool> csr_exception; // Writes to read-only CSRs raise an exeption
    sc_in<sc_uint<32> > csr_rdata;
    sc_in<bool> csr_delegate_dreg;      // jump to stvec when set on exception
    sc_in<bool> csr_isHalted;   
    sc_in<bool> csr_cache_flush; // CSR module requests cache flushing
    sc_out<bool> csr_ack;
    sc_in<sc_uint<2> > csr_priv_mode, csr_load_store_priv_mode;

    sc_in<bool> csr_mstatus_SIE,      // jump to stvec when set on exception
            csr_mstatus_SUM,
            csr_mstatus_MIE,
            csr_mstatus_TSR,
            csr_mstatus_TVM,
            csr_dcsr_ebreakm,
            csr_dcsr_step;
    sc_in<sc_uint<32> >
            csr_mideleg,
            csr_sepc,
            csr_stvec,
            csr_mcause,
            csr_mtvec,
            csr_dpc,
            csr_mepc,
            csr_mip;

    sc_in<sc_uint<CFG_NUT_CPU_CORES> >
            csr_m3_pnce,
            csr_m3_pnlm,
            csr_m3_pnxsel;

    sc_in<bool> csr_m3_icache_enable, csr_m3_dcache_enable;
    
    sc_in<bool> csr_lsu_paging_mode, csr_ifu_paging_mode;
    sc_in<sc_uint<20> > csr_satp_root_ppn;

    // to MMU
    sc_out<bool> tlb_flush;

    sc_out<bool> ifu_paging_mode, lsu_paging_mode;
    sc_out<sc_uint<20> > root_ppn;

    // Misc. ports ...
    //   TBD: Are they still necessary? -> Eliminate or document
    // (HLS reasons, should be optimized out while synthetization)
    // these could be generics in VHDL, but current tools don't support that
    sc_in<sc_uint<CFG_NUT_CPU_CORES_LD> > hartID;
    // 'inCePU' indicates whether the surrounding CPU is the CePU.
    // 'mode2Cap' indicates the maximum mode supported.
    // The (only) possible combinations of ('inCePU', 'mode2Cap') are:
    //    (true, true)    // CePU, all capabilities
    //    (false, true)   // CoPU supporting modes 0-2
    //    (false, false)  // CoPU supporting modes 0-1 (no unlinked mode)
    sc_in<bool> inCePU, mode2Cap;



    // Constructor...
    SC_HAS_PROCESS (MExu);
    MExu (sc_module_name name = "MExu")
    : sc_module (name)
#if CFG_EXU_M_EXTENSION == 1
      // Instantiate M-Extension submodule
      , mext ("MExtension")
#endif
    {
        // EXU methods
        PN_CLOCK_TRIGGERED (MainMethod);
        SC_METHOD (MainCombMethod);
            sensitive << reset << enable << alu_result << ir_valid << lsu_ack << lsu_width_dreg << lsu_align_err
                      << lsu_rdata << lsu_scond_ok << ifu_npc << state_reg << csr_mepc << csr_mtvec << csr_stvec
                      << csr_mcause << csr_exception << csr_rdata << inCePU << lsu_ac_r << lsu_ac_w << lsu_ac_u
                      << csr_ifu_paging_mode << csr_lsu_paging_mode << csr_priv_mode << csr_load_store_priv_mode
                      << mode2Cap << ex_i << m3_pnx << ex_handle_reg << sync_reg << sync_i << m3_priv_mode_i
                      << sync_next << linked << alu_branch_reg << alu_result_reg << internal_next
                      << csr_delegate_dreg << sret_dreg << csr_sepc << csr_satp_root_ppn << csr_mstatus_SUM
                      << exu_op_dreg << op_a_dreg << op_b_dreg << lsu_rd_dreg << cache_op_dreg
                      << flush_dreg << lres_scond_dreg << illegal_insn_dreg << xret_dreg
                      << csr_cache_flush << ebreak_dreg << ecall_dreg << insn_page_fault_dreg << dbg_enter_dreg
                      << csr_m3_pnce << m3_ir_request << ignore_ir_reg << csr_mstatus_MIE << csr_mstatus_SIE
                      << dbg << irq_dreg << step_dreg << m3_ir_id << cause_i << csr_mideleg
                      << ex_CoPU_dreg << csr_dcsr_step << xsel << csr_dpc << pc << dbg_reg;
#if CFG_EXU_M_EXTENSION == 1
            // Add M-Extension signals to MainCombMethods sensitivity list
            sensitive << alu_d_result << alu_m_result << alu_md_dreg << alu_m_valid << alu_d_valid;

        // Route M-Extension submodule signals
        mext.clk (clk);
        mext.reset (reset);
        mext.d_result (alu_d_result);
        mext.d_valid (alu_d_valid);
        mext.d_enable (alu_d_enable);
        mext.md_func (alu_md_function_dreg);
        mext.op_a (op_a_dreg);
        mext.op_b (op_b_dreg);
        mext.m_enable (alu_m_enable);
        mext.m_valid (alu_m_valid);
        mext.m_result (alu_m_result);
#endif

        PN_CLOCK_TRIGGERED (DecodeMethod);
        SC_METHOD (Mode2Method);
            sensitive << linked << ifu_ir << ifu_ir_valid << ifu_pc << m2_ir_valid << m2_ir << m2_pc
                      << ifu_ac_x << ifu_ac_u << m2_ac_x << m2_ac_u << inCePU << mode2Cap;

        // Alu methods
        SC_METHOD (AluShiftMethod);
            sensitive << alu_s_enable << alu_s_function << alu_s_amount << op_a_dreg;
        SC_METHOD (AluCombMethod);
            sensitive << op_a_dreg << op_b_dreg << branch_a_dreg << branch_b_dreg << alu_s_ready << alu_s_result
                      << alu_function_dreg << alu_af_dreg << alu_branch_dreg;

        SC_METHOD (CSRMethod);
            sensitive << pop_priv_ir_stack_dreg << csr_instr << exception << irq_dreg << dbg
                      << dbg_reg << csr_rs1_reg << csr_op_a << csr_function_reg << csr_adr_reg << csr_priv_mode
                      << ex_id_reg << sret_dreg << csr_tval << csr_m3_pnxsel << csr_m3_pnce << csr_m3_pnlm
                      << csr_m3_icache_enable << csr_m3_dcache_enable << ifu_pc << ifu_ir << m3_priv_mode_i
                      << hartID << inCePU << linked << m3_pnhaltreq << m3_pnx << cause_i << enable << csr_satp_root_ppn
                      << dbg_enter_dreg;
#if CFG_EXU_PERFCOUNT_ENABLE == 1
            sensitive << perf_inc << perf_addr;
#endif
    }

    // Functions...
    void Trace (sc_trace_file * tf, int levels = 1);

#ifndef __SYNTHESIS__
    void DisplayStatistics () { return perf_mon_.Display (this->name ()); }
    bool IsHalted () { return csr_isHalted.read (); }
#endif

    // Processes...
    void AluCombMethod ();
    void AluShiftMethod ();

    void MainMethod ();
    void MainCombMethod ();
    void DecodeMethod ();

    void Mode2Method ();
    void CSRMethod ();

protected:
    // Debug state ...
    sc_signal<bool> dbg, dbg_reg;

    // Exception handling ...
    sc_signal<sc_uint<5> > ex_id, ex_id_reg;
    sc_signal<bool> ex_handle_reg, ex_handle; // ex_handle_reg is 1 during exception handling (gets reset by executing mret instruction)
    sc_signal<bool> exception;  // internal signal, active for one cycle (ExuExOrIrq state) to
                                // trigger saving of the exception state CSRs (mepc, mcause, ...)
    sc_signal<bool> ignore_ir_reg, ignore_ir;
    sc_signal<bool> m3_ir_request_reg; // save ir_request so we can prioritize CoPU excecptions

    // CPU general purpose integer registers ...
    sc_signal<sc_uint<32> > gp_regs[REGISTERS];
    sc_signal<sc_uint<32> > gpr_in;    // Input value for GPR writes
    sc_signal<bool> gpr_write;  // Write gpr_in value to GPR specified by gpr_sel_dreg

    // Exu signals/registers ...
    sc_signal<sc_uint<EX_ID_LENGTH> > state_reg, state;
    sc_signal<bool> stall_decode;
    sc_signal<bool> internal_next;

    // Exu Mode 1 signals/registers ...
    sc_signal<bool> sync_reg, sync;
    sc_signal<bool> ir_valid;
    sc_signal<sc_uint<32> > ir, pc;
    sc_signal<bool> ac_x, ac_u;

    // Helper methods...
    void InstructionTrace ();
    void DumpRegisterInfo ();
    void DumpRegisterChange ();

    // CSR method signals/registers ...
    sc_signal<sc_uint<3> > csr_function_reg;
    sc_signal<sc_uint<12> > csr_adr_reg;
    sc_signal<sc_uint<5> > csr_rs1_reg;
    sc_signal<bool> csr_instr;
    sc_signal<sc_uint<32> > csr_op_a;
    sc_signal<sc_uint<32> > csr_tval;

    // ALU signals ...
    sc_signal<bool> alu_finished, alu_branch;
    sc_signal<sc_uint<32> > alu_result;

    sc_signal<sc_uint<32> > alu_result_reg; // Register for saving alu return value
    sc_signal<bool> alu_branch_reg;         // Register for saving alu branch decision value

    sc_signal<sc_uint<2> > alu_s_function;
    sc_signal<sc_uint<5> > alu_s_amount;
    sc_signal<bool> alu_s_ready, alu_s_enable;
    sc_signal<sc_uint<32> > alu_s_result;

#if CFG_EXU_M_EXTENSION == 1
    // M-Extension module ...
    MMExtension mext;
    // M-Extension signals/registers ...
    sc_signal<bool> alu_m_enable, alu_m_valid;
    sc_signal<sc_uint<32> > alu_m_result;
    sc_signal<bool> alu_d_enable, alu_d_valid;
    sc_signal<sc_uint<32> > alu_d_result;
    sc_signal<bool> alu_md_dreg; // Decode Register: 1 for div operations, 0 for mul operations
    sc_signal<sc_uint<2> > alu_md_function_dreg; // Decode Register: See EMExtFunc enum for value description
#endif

#if CFG_EXU_PERFCOUNT_ENABLE == 1
    // Perfcount signals ...
    sc_signal<bool> perf_inc;
    sc_signal<sc_uint<CFG_EXU_PERFCOUNTERS_LD> > perf_addr;
#endif

    // Decode registers ...
    sc_signal<sc_uint<32> >
            op_a_dreg,      // Operand A for ALU/... operations (depends on instruction format)
            op_b_dreg,      // Operand B for ALU/... operations (depends on instruction format)
            branch_a_dreg,  // Operand A for Branch decision (B-Type instruction format: PC)
            branch_b_dreg;  // Operand B for Branch decision (B-Type instruction format: sign extended 12 bit immediat)
    sc_signal<sc_uint<5> > gpr_sel_dreg; // Destination register select
    sc_signal<bool>
            sret_dreg,          // Set to trigger usage of mepc - sepc otherwise
            xret_dreg,          // Used when executing xret insructions
            pop_priv_ir_stack_dreg, // trigger popping the Privilege and Global Interrupt-Enable Stack
            alu_af_dreg,        // Set to use alternate function (Add->Sub, Srl->Sra, see EAluFuncRISCV)
            alu_branch_dreg,    // Set to calculate branch result
            illegal_insn_dreg,  // Set if illegal instruction was decoded
            insn_page_fault_dreg, // Set on unprivileged instruction accesses
            ex_CoPU_dreg,       // Set if ex_i was set during instruction decode (handle CoPU exceptions)
            irq_dreg,           // Set if m3_ir_request is set during instruction decode (handle interrupt request)
            step_dreg,          // Set when csr_dcsr_step is set, enter debug mode automatically after one instruction
            ecall_dreg,         // Set to trigger ecall exception
            ebreak_dreg,        // Set to trigger ebreak exception
            flush_dreg,         // Set for fence and PN_CFLUSH, PN_CINVALIDATE, PN_CWRITEBACK instructions (flush IFU and LSU write buffer)
            lsu_rd_dreg,        // 1 for load/lres operations, 0 for store/scond operations
            lres_scond_dreg,    // Set for lres and scond operations
            dbg_enter_dreg;     // Set while entering debug mode
    sc_signal<sc_uint<2> > lsu_width_dreg; // set to indicate L/S width or cache operation
    sc_signal<sc_uint<3> > cache_op_dreg; // Set for the cache operations (flush, invalidate, writeback,...)
    sc_signal<sc_uint<4> >
            exu_op_dreg,        // EXU Execute (Stage) operation (see EExuOp)
            alu_function_dreg;  // ALU operation (see EAluFuncRISCV)


    // Performance Monitor ...
    CPerfMonCPU perf_mon_;
};
