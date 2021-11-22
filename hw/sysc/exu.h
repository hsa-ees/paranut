/*************************************************************************

  This file is part of the ParaNut project.

  Copyright (C) 2010-2020 Alexander Bahle <alexander.bahle@hs-augsburg.de>
                          Christian H. Meyer <christian.meyer@hs-augsburg.de>
                          Gundolf Kiefer <gundolf.kiefer@hs-augsburg.de>
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
    // ParaNut extension states:
    ExuHalt,
    ExuLSUFlush,
    ExuCACHECONT,
    ExuCACHECONT2,
    // Exception states:
    ExuExOrIrq,
    ExuExCoPUHalt,
    ExuExCoPUWaitHandle,
    ExuExCoPUWaitCePU,
    ExuExWaitCoPUs,
    ExuExJumpMtvec,
    // Special states (for extensions etc.)
#if CFG_EXU_M_EXTENSION == 1
    ExuDiv,
    ExuMulDivWB,
#endif
    // Debug states:
    ExuDBG,
} EExuState;

typedef enum {
    OpNOP,
    OpAlu,
    OpJump,
    OpBranch,
    OpMem,
    OpSystem,
    OpXRet,
    OpHalt,
    OpParaNut,
    OpException,
    OpDebug,

    // Special Operation (for extensions etc.)
#if CFG_EXU_M_EXTENSION == 1
    OpMulDiv,
#endif
} EExuOp;

// **************** Exception Codes *************
typedef enum {
    /*** Exceptions: ***/
    InstructionAddressMisaligned = 0,
    // InstructionAccessFault,                 // no access faults from IFU
    IllegalInstruction = 2,
    Breakpoint = 3,
    LoadAddressMisaligned = 4,
    // LoadAccessFault,                        // no access faults from LSU
    StoreAddressMisaligned = 6,
    // StoreAccessFault,                       // no access faults from LSU
    ECallU = 8,                                // no User Mode
    ECallS = 9,                                // no Supervisor Mode
    // 10 Reserved
    ECallM = 11,
    // InstructionPageFault,
    // LoadPageFault,
    // 14 Reserved
    // StorePageFault = 15,
    // >= 16 Reserved
    CoPUException = 16
} EExceptions;


// **************** MExu ************************

class MExu : ::sc_core::sc_module {
public:
    // Ports ...
    sc_in<bool> clk, reset;

    //   to IFU ...
    sc_out<bool> ifu_next, ifu_jump, ifu_flush, ifu_reset;
    sc_out<TWord> ifu_jump_adr; // jump address
    sc_in<bool> ifu_ir_valid, ifu_npc_valid;
    sc_in<TWord> ifu_ir, ifu_pc, ifu_npc; // expected to be registered (fast) outputs

    //   to Load/Store Unit (LSU)...
    sc_out<bool> lsu_rd, lsu_wr, lsu_flush;
    sc_out<sc_uint<3> > lsu_cache_op;
    sc_in<bool> lsu_ack;
    sc_in<bool> lsu_align_err;
    sc_out<sc_uint<2> > lsu_width; // "00" = word, "01" = byte, "10" = half word
    sc_out<bool> lsu_exts;
    sc_out<TWord> lsu_adr;
    sc_in<TWord> lsu_rdata;
    sc_out<TWord> lsu_wdata;
    sc_out<bool> lsu_lres_scond;
    sc_in<bool> lsu_scond_ok;

    // Exception signals
    sc_in<bool> xsel;
    sc_in<bool> ex_i;
    sc_out<bool> ex_o;
    sc_in<sc_uint<32> > epc_i;
    sc_out<sc_uint<32> > epc_o;
    sc_in<sc_uint<5> > cause_i;
    sc_out<sc_uint<5> > cause_o;

    // Mode 1 syncronization signal
    sc_in<bool> sync_i;
    sc_out<bool> sync_o;
    sc_in<bool> sync_next;

    // Mode 2 linked mode signals
    sc_in<bool> m2_ir_valid;
    sc_in<TWord> m2_ir, m2_pc;

    // from CePU
    sc_in<bool> enable, linked;
    // to CePU
    sc_out<bool> haltreq;

    // to CoPUs
    sc_out<sc_uint<CFG_NUT_CPU_CORES> > m3_pnce;
    sc_out<sc_uint<CFG_NUT_CPU_CORES> > m3_pnlm;
    sc_out<sc_uint<CFG_NUT_CPU_CORES> > m3_pnxsel;
    // from CoPUs
    sc_in<sc_uint<CFG_NUT_CPU_CORES> > m3_pnhaltreq, m3_pnx;
    // from IntC
    sc_in<bool> m3_ir_request;
    sc_in<sc_uint<5> > m3_ir_id;
    // to IntC
    sc_out<bool> m3_ir_ack;
    sc_out<bool> m3_ir_enable;

    // controller outputs...
    sc_out<bool> m3_icache_enable, m3_dcache_enable;

    // from Debug Unit
    sc_in<bool> dbg_req;

    // to CSR Module
    sc_out<bool> csr_dbg, csr_dbg_reg;
    sc_out<bool> csr_cpu_enabled;
    sc_out<sc_uint<5> > csr_ex_id_reg;
    sc_out<bool> csr_mret_dreg;
    sc_out<bool> csr_exu_exception;
    sc_out<bool> csr_irq_dreg;
    sc_out<bool> csr_pop_priv_ir_stack_dreg;
    sc_out<sc_uint<5> > csr_csr_rs1_reg;
    sc_out<sc_uint<32> > csr_csr_op_a;
    sc_out<sc_uint<3> > csr_csr_function_reg;
    sc_out<sc_uint<12> > csr_csr_adr_reg;
    sc_out<sc_uint<32> > csr_csr_tval;

    sc_out<TWord> csr_pc, csr_ir;

    sc_out<TWord> csr_hartID;
    sc_out<TWord> csr_clock_freq_hz;
    sc_out<bool> csr_inCePU;
    sc_out<bool> csr_linked;
    sc_out<bool> csr_enable;
    sc_out<sc_uint<5> > csr_cause;
    sc_out<sc_uint<32> > csr_epc;
    sc_out<sc_uint<CFG_NUT_CPU_CORES> > csr_m3_pnhaltreq, csr_m3_pnx;

    
#if CFG_EXU_PERFCOUNT_ENABLE == 1
    // Perfcount signals ...
    sc_out<bool> csr_perf_inc;
    sc_out<sc_uint<CFG_EXU_PERFCOUNTERS_LD> > csr_perf_addr;
#endif

    // from CSR Module
    sc_in<bool>
            csr_exception, // Writes to read-only CSRs raise an exeption
            csr_rd_exception; // Reads to non-existent CSRs raise an exceptions
    sc_in<sc_uint<32> > csr_rdata;
    sc_in<bool> csr_delegate_dreg;      // jump to stvec when set on exception
    sc_in<bool> csr_isHalted;

    sc_in<sc_uint<2> > csr_priv_mode_reg;

    sc_in<bool> csr_mstatus_SIE,      // jump to stvec when set on exception
            csr_mstatus_MIE,
            csr_dcsr_ebreakm,
            csr_dcsr_step;
    sc_in<sc_uint<32> >
            csr_mideleg,
            csr_sepc,
            csr_stvec,
            csr_mcause,
            csr_mtvec,
            csr_dpc,
            csr_mepc;

    sc_in<sc_uint<CFG_NUT_CPU_CORES> > 
            csr_m3_pnce,
            csr_m3_pnlm,
            csr_m3_pnxsel;

    sc_in<bool> csr_m3_icache_enable, csr_m3_dcache_enable;
    
    // misc (HLS reasons, should be optimized out while synthetization)
    // these could be generics in VHDL, but current tools don't support that
    sc_in<TWord> hartID;
    // 'inCePU' indicates whether the surrounding CPU is the CePU.
    // 'mode2Cap' indicates the maximum mode supported.
    // The (only) possible combinations of ('inCePU', 'mode2Cap') are:
    //    (true, true)    // CePU, all capabilities
    //    (false, true)   // CoPU supporting modes 0-2
    //    (false, false)  // CoPU supporting modes 0-1 (no unlinked mode)
    sc_in<bool> inCePU, mode2Cap;
    // Clock frequency in Hz (can be read throughg pnclockinfo CSR)
    sc_in<TWord> clock_freq_hz;

    //   TBD: timer, interrupt controller ...

    // Constructor...
    SC_HAS_PROCESS (MExu);
    MExu (sc_module_name name)
    : sc_module (name)
#if CFG_EXU_M_EXTENSION == 1
      // Instantiate M-Extension submodule
      , mext ("MExtension")
#endif
    {
        // EXU methods
        SC_METHOD (MainMethod);
            sensitive << clk.pos ();
        SC_METHOD (MainCombMethod);
            sensitive << reset << enable << alu_result << ir_valid << lsu_ack << lsu_align_err
                      << lsu_rdata << lsu_scond_ok << ifu_npc << state_reg << csr_mepc << csr_mtvec
                      << csr_mcause << csr_exception << csr_rd_exception << csr_rdata << inCePU
                      << mode2Cap << ex_i << m3_pnx << ex_handle_reg << sync_reg << sync_i
                      << sync_next << linked << alu_branch_reg << alu_result_reg << internal_next
                      << csr_delegate_dreg << irq_ign_dreg << mret_dreg << csr_sepc << csr_priv_mode_reg
                      << exu_op_dreg << op_a_dreg << op_b_dreg << lsu_rd_dreg << cache_op_dreg 
                      << lres_scond_dreg << illegal_insn_dreg << ebreak_dreg << ecall_dreg;
#if CFG_EXU_M_EXTENSION == 1
            // Add M-Extension signals to MainCombMethods sensitivity list
            sensitive << alu_d_result << alu_m_result << alu_m_valid << alu_d_valid;

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

        SC_METHOD (DecodeMethod);
            sensitive << clk.pos ();
        SC_METHOD (Mode2Method);
            sensitive << linked << ifu_ir_valid << ifu_pc << m2_ir_valid << m2_ir << m2_pc << inCePU << mode2Cap;

        // Alu methods
        SC_METHOD (AluShiftMethod);
            sensitive << alu_s_enable << alu_s_function << alu_s_amount << op_a_dreg;
        SC_METHOD (AluCombMethod);
            sensitive << op_a_dreg << op_b_dreg << branch_a_dreg << branch_b_dreg << alu_s_ready << alu_s_result
                      << alu_function_dreg << alu_af_dreg << alu_branch_dreg;

        SC_METHOD (CSRMethod);
            sensitive << pop_priv_ir_stack_dreg << csr_instr << exception << irq_dreg << dbg
                      << dbg_reg << csr_rs1_reg << csr_op_a << csr_function_reg << csr_adr_reg
                      << ex_id_reg << mret_dreg << csr_tval << csr_m3_pnxsel << csr_m3_pnce << csr_m3_pnlm
                      << csr_m3_icache_enable << csr_m3_dcache_enable << ifu_pc << ifu_ir
                      << hartID << clock_freq_hz << inCePU << linked << m3_pnhaltreq << m3_pnx
                      << epc_i << cause_i << enable;
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

    // CPU general purpose integer registers ...
    sc_signal<TWord> gp_regs[REGISTERS];
    sc_signal<TWord> gpr_in;    // Input value for GPR writes
    sc_signal<bool> gpr_write;  // Write gpr_in value to GPR specified by gpr_sel_dreg

    // Exu signals/registers ...
    sc_signal<sc_uint<EX_ID_LENGTH> > state_reg, state;
    sc_signal<bool> stall_decode;
    sc_signal<bool> internal_next;

    // Exu Mode 1 signals/registers ...
    sc_signal<bool> sync_reg, sync;
    sc_signal<bool> ir_valid;
    sc_signal<TWord> ir, pc;

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
            irq_ign_dreg,          // Set to trigger usage of mepc - sepc otherwise
            mret_dreg,          // Set to trigger usage of mepc - sepc otherwise
            pop_priv_ir_stack_dreg, // trigger popping the Privilege and Global Interrupt-Enable Stack
            alu_af_dreg,        // Set to use alternate function (Add->Sub, Srl->Sra, see EAluFuncRISCV)
            alu_branch_dreg,    // Set to calculate branch result
            illegal_insn_dreg,  // Set if illegal instruction was decoded
            ex_CoPU_dreg,       // Set if ex_i was set during instruction decode (handle CoPU exceptions)
            irq_dreg,           // Set if m3_ir_request is set during instruction decode (handle interrupt request)
            step_dreg,          // Set when csr_dcsr_step is set, enter debug mode automatically after one instruction
            ecall_dreg,         // Set to trigger ecall exception
            ebreak_dreg,        // Set to trigger ebreak exception
            flush_dreg,         // Set for fence and PN_CFLUSH, PN_CINVALIDATE, PN_CWRITEBACK instructions (flush IFU and LSU write buffer)
            lsu_rd_dreg,        // 1 for load/lres operations, 0 for store/scond operations
            lres_scond_dreg;    // Set for lres and scond operations
    sc_signal<sc_uint<3> > cache_op_dreg; // Set for the cache operations (flush, invalidate, writeback,...)
    sc_signal<sc_uint<4> >
            exu_op_dreg,        // EXU Execute (Stage) operation (see EExuOp)
            alu_function_dreg;  // ALU operation (see EAluFuncRISCV)


    // Performance Monitor ...
    CPerfMonCPU perf_mon_;
};
