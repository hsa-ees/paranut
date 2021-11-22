/*************************************************************************

  This file is part of the ParaNut project.

  Copyright (C) 2010-2020 Alexander Bahle <alexander.bahle@hs-augsburg.de>
                          Christian H. Meyer <christian.meyer@hs-augsburg.de>
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


#include "exu.h"
#include "memory.h" // only for debugging 'mainMemory'

#include <assert.h>

// ************ Performance Counters ************

#if CFG_EXU_PERFCOUNT_ENABLE == 1
// PERFORMANCE COUNTER MACROS:
#define PERFSTD()      \
    {                  \
        perf_inc = 0;  \
        perf_addr = 0; \
    }
#define PERFINC(COUNTER)     \
    {                        \
        perf_addr = COUNTER; \
        perf_inc = 1;        \
    }
#else // CFG_EXU_PERFCOUNT_ENABLE != 1
#define PERFSTD()   \
    { /* nothing */ \
    }
#define PERFINC(COUNTER) \
    { /* nothing */      \
    }
#endif

// **************** Tracing *********************

#ifndef __SYNTHESIS__
void MExu::Trace (sc_trace_file *tf, int level) {
    if (!tf || pn_trace_verbose) printf ("\nSignals of Module \"%s\":\n", name ());

    // Ports ...
    PN_TRACE (tf, clk);
    PN_TRACE (tf, reset);
    //   to IFU ...
    PN_TRACE (tf, ifu_next);
    PN_TRACE (tf, ifu_jump);
    PN_TRACE (tf, ifu_reset);
    PN_TRACE (tf, ifu_jump_adr);
    PN_TRACE (tf, ifu_ir_valid);
    PN_TRACE (tf, ifu_npc_valid);
    PN_TRACE (tf, ifu_ir);
    PN_TRACE (tf, ifu_pc);
    PN_TRACE (tf, ifu_npc);
    //   to Load/Store Unit (LSU)...
    PN_TRACE (tf, lsu_rd);
    PN_TRACE (tf, lsu_wr);
    PN_TRACE (tf, lsu_flush);
    PN_TRACE (tf, lsu_cache_op);
    PN_TRACE (tf, lsu_ack);
    PN_TRACE (tf, lsu_align_err);
    PN_TRACE (tf, lsu_width);
    PN_TRACE (tf, lsu_exts);
    PN_TRACE (tf, lsu_adr);
    PN_TRACE (tf, lsu_rdata);
    PN_TRACE (tf, lsu_wdata);

    PN_TRACE (tf, m3_icache_enable);
    PN_TRACE (tf, m3_dcache_enable);
    PN_TRACE (tf, m3_ir_request);


    PN_TRACE (tf, m2_pc);
    PN_TRACE (tf, m2_ir);
    PN_TRACE (tf, m2_ir_valid);

    PN_TRACE (tf, sync_reg);
    PN_TRACE (tf, sync);
    PN_TRACE (tf, sync_i);
    PN_TRACE (tf, sync_o);
    PN_TRACE (tf, sync_next);
    PN_TRACE (tf, m3_pnce);
    PN_TRACE (tf, m3_pnlm);

    PN_TRACE (tf, ir);
    PN_TRACE (tf, ir_valid);
    PN_TRACE (tf, pc);

    // to CePU
    PN_TRACE (tf, ex_i);
    PN_TRACE (tf, haltreq);
    PN_TRACE (tf, ex_o);
    PN_TRACE (tf, m3_pnx);
    PN_TRACE (tf, epc_i);
    PN_TRACE (tf, epc_o);
    PN_TRACE (tf, cause_i);
    PN_TRACE (tf, cause_o);
    PN_TRACE (tf, m3_pnxsel);
    PN_TRACE (tf, xsel);

    PN_TRACE (tf, enable);
    PN_TRACE (tf, linked);
    PN_TRACE (tf, m3_pnhaltreq);

    PN_TRACE (tf, ex_handle_reg);
    PN_TRACE (tf, ex_handle);
    PN_TRACE (tf, exception);
    PN_TRACE (tf, ex_id);

    PN_TRACE (tf, dbg_req);
    PN_TRACE (tf, dbg);
    PN_TRACE (tf, dbg_reg);
    PN_TRACE (tf, step_dreg);

    // Registers...
    PN_TRACE_BUS (tf, gp_regs, REGISTERS);
    PN_TRACE (tf, gpr_write);
    PN_TRACE (tf, gpr_in);
    PN_TRACE (tf, gpr_sel_dreg);

    PN_TRACE (tf, irq_dreg);
    PN_TRACE (tf, exu_op_dreg);
    PN_TRACE (tf, stall_decode);
    PN_TRACE (tf, state_reg);
    PN_TRACE (tf, state);


    PN_TRACE (tf, pop_priv_ir_stack_dreg);
#if CFG_PRIV_LEVELS == 3
    PN_TRACE (tf, mret_dreg);
#endif

    PN_TRACE (tf, exu_op_dreg);
    PN_TRACE (tf, stall_decode);
    PN_TRACE (tf, state_reg);
    PN_TRACE (tf, state);

    PN_TRACE (tf, csr_adr_reg);
    PN_TRACE (tf, csr_enable);
    PN_TRACE (tf, csr_function_reg);
    PN_TRACE (tf, csr_op_a);
    PN_TRACE (tf, csr_rdata);
    PN_TRACE (tf, csr_rs1_reg);
    PN_TRACE (tf, csr_exception);
    PN_TRACE (tf, csr_rd_exception);

    // Alu signals
    PN_TRACE (tf, alu_finished);
    PN_TRACE (tf, alu_function_dreg);
    PN_TRACE (tf, alu_result);
    PN_TRACE (tf, alu_af_dreg);
    PN_TRACE (tf, alu_s_ready);
    PN_TRACE (tf, op_a_dreg);
    PN_TRACE (tf, op_b_dreg);
    PN_TRACE (tf, alu_branch_dreg);
    PN_TRACE (tf, alu_branch);
    PN_TRACE (tf, alu_s_amount);
    PN_TRACE (tf, alu_s_result);
    PN_TRACE (tf, alu_s_function);
    PN_TRACE (tf, alu_s_enable);
#if CFG_EXU_M_EXTENSION == 1
    PN_TRACE (tf, alu_d_enable);
    PN_TRACE (tf, alu_m_valid);
    PN_TRACE (tf, alu_m_result);
    PN_TRACE (tf, alu_md_dreg);
    PN_TRACE (tf, alu_d_valid);
    PN_TRACE (tf, alu_d_result);
    // MExtension Sub-Module
    if (level > 1) {
        level--;
        mext.Trace (tf, level);
    }
#endif // CFG_EXU_M_EXTENSION == 1
}
#endif // __SYNTHESIS__


// ************** Simulation Helpers ******************

void MExu::DumpRegisterInfo () {
    PN_INFOF (("   (%s)  R1/ra=%08x   R2/sp=%08x   R3/gp=%08x    R4/tp=%08x   R5/t0=%08x   R6/t1=%08x "
            "  R7/t2=%08x",
            strrchr (name (), '.') + 1, gp_regs[1].read (), gp_regs[2].read (), gp_regs[3].read (),
            gp_regs[4].read (), gp_regs[5].read (), gp_regs[6].read (), gp_regs[7].read ()));
    PN_INFOF (("   (%s)  R8/s0=%08x   R9/s1=%08x  R10/a0=%08x   R11/a1=%08x  R12/a2=%08x  R13/a3=%08x "
            " R14/a4=%08x  R15/a5=%08x",
            strrchr (name (), '.') + 1, gp_regs[8].read (), gp_regs[9].read (), gp_regs[10].read (),
            gp_regs[11].read (), gp_regs[12].read (), gp_regs[13].read (), gp_regs[14].read (),
            gp_regs[15].read ()));
    PN_INFOF (("   (%s) R16/a6=%08x  R17/a7=%08x  R18/s2=%08x   R19/s3=%08x  R20/s4=%08x  R21/s5=%08x "
            " R22/s6=%08x  R23/s7=%08x",
            strrchr (name (), '.') + 1, gp_regs[16].read (), gp_regs[17].read (),
            gp_regs[18].read (), gp_regs[19].read (), gp_regs[20].read (), gp_regs[21].read (),
            gp_regs[22].read (), gp_regs[23].read ()));
    PN_INFOF (("   (%s) R24/s8=%08x  R25/s9=%08x  R26/s10=%08x R27/s11=%08x  R28/t3=%08x  R29/t4=%08x "
            " R30/t5=%08x  R31/t6=%08x",
            strrchr (name (), '.') + 1, gp_regs[24].read (), gp_regs[25].read (),
            gp_regs[26].read (), gp_regs[27].read (), gp_regs[28].read (), gp_regs[29].read (),
            gp_regs[30].read (), gp_regs[31].read ()));
}

#ifndef __SYNTHESIS__
void MExu::DumpRegisterChange () {
    if (pn_cfg_insn_trace > 0) {
        PN_INFOF (("   (%s) R%-2d = %08x -> %08x    %d -> %d", strrchr (name (), '.') + 1, gpr_sel_dreg.read().value() ,
                gp_regs[gpr_sel_dreg.read()].read(),  gpr_in.read(), gp_regs[gpr_sel_dreg.read()].read(), gpr_in.read()));
    }
}

// Helper function to display the instruction trace during simulation
void MExu::InstructionTrace () {
    if (pn_cfg_insn_trace > 1)
        DumpRegisterInfo ();
    if (pn_cfg_insn_trace > 0)
        PN_INFOF (("   (%s) %s", strrchr (name (), '.') + 1, mainMemory->GetDumpStr (pc.read ())));
}
#else
void MExu::InstructionTrace () {}
void MExu::DumpRegisterChange () {}
#endif


// **************** ALU *************************

void MExu::AluShiftMethod () {
    sc_uint<2> function;
    sc_uint<32> op_a;
    sc_uint<32> upper, lower;
    sc_uint<64> result;
    sc_uint<5> amount;
    bool ready;

    // Read inputs and set default values
    op_a = op_a_dreg.read ();
    function = alu_s_function.read ();
    ready = 1;

    amount = function.or_reduce () ? alu_s_amount.read () : (sc_uint<5>)~alu_s_amount.read ();
    if (function == 0) {
        // SLL:
        upper = (sc_uint<1> (0), op_a (31, 1));
        lower = (op_a[0], sc_uint<31> (0));
    } else {
        // SRL/SRA:
        upper = function[1] & op_a[31] ? -1 : 0; // sign only relevant for SRA (select[1] == 1) is set
        lower = op_a;
    }

    result = (upper, lower);

    // Barrel shifter:
    if (amount[4] == 1) result (47, 0) = result (63, 16);
    if (amount[3] == 1) result (39, 0) = result (47, 8);
    if (amount[2] == 1) result (35, 0) = result (39, 4);
    if (amount[1] == 1) result (33, 0) = result (35, 2);
    if (amount[0] == 1) result (31, 0) = result (32, 1);

    /* Generic shift with 1 clk per shift (not working currently)
     * 	// Read registers:
        count = aluShiftCount.read();
        shiftReturn = alu_s_result.read();

        if(alu_s_enable == 0) {
            // Init state
            count = alu_s_amount.read();
            shiftReturn = aluA.read();;
        } else {
            // Check for end of shift
            if(count == 0) {
                finished = 1;
            } else {
                switch (select) {
                    case 1: // SLL
                        shiftReturn = (sc_uint<32>) ( shiftReturn(30, 0), sc_uint<1>(0) ) ;
                        break;
                    case 2: // SRL
                        shiftReturn = (sc_uint<32>) ( sc_uint<1>(0), shiftReturn(31, 1) );
                        break;
                    case 3: // SRA
                        shiftReturn = (sc_uint<32>) ( shiftReturn[31], shiftReturn(31, 1) );
                        break;
                    default:
                        // shiftSel signal is wrong
                        PN_ASSERTF (false, ("Impossible/implausible shift select signal: %d (Must be
     between 1 and 3)", select.value())); break;
                }
                count--;
            }
        }
        aluShiftCount = count;
    */

    alu_s_ready = ready;
    alu_s_result = result (31, 0);
}

void MExu::AluCombMethod () {
    sc_uint<32> op_a, op_b, result;
    sc_uint<33> add_a, add_b;
    sc_uint<4> function;

    // Read input signals
    function = alu_function_dreg.read ();
    op_a = op_a_dreg.read ();
    op_b = op_b_dreg.read ();

    add_a = alu_branch_dreg ? (branch_a_dreg.read (), sc_uint<1> (1)) : (op_a, sc_uint<1> (1));
    add_b = alu_branch_dreg ? (branch_b_dreg.read (), sc_uint<1> (0)) : (op_b, sc_uint<1> (0));

    // Preset control signals ...
    alu_s_enable = 0;


    // ADDER
    if (alu_af_dreg & !alu_branch_dreg)
        add_b = add_b ^ sc_uint<33> (~0);
    sc_uint<33> add_result = add_a + add_b;


    // COMPARE
    bool equal_result = op_a == op_b;
    bool less_result = op_a < op_b;

    if ((op_a[31] ^ op_b[31]) & !function[0]) // function[1]: 0 for SLT/BLT and 1 for SLTU/BLTU
        less_result = !less_result; // less is now signed less

    bool compare_result = alu_af_dreg ? !less_result | equal_result : // BGE/BGEU
                         less_result; // SLT/SLTU/BLT/BLTU


    // SHIFT
    // Handled in AluShiftMethod()


    // OUTPUT MULTIPLEXER
    switch (function) {
    // Logical...
    case afSlt:
    case afSltu:
        result = (sc_uint<31> (0), compare_result);
        break;
    case afAnd:
        result = op_a & op_b;
        break;
    case afOr:
        result = op_a | op_b;
        break;
    case afXor:
        result = op_a ^ op_b;
        break;
    // Shift ...
    case afSll:
    case afSrl: // afSra implicit
        result = alu_s_result.read ();
        // finished when shift is ready again
        break;
    // Add/sub...
    default:
        result = add_result (32, 1);
        break;
    }

    // Set return values
    alu_result = result;
    alu_branch = function == afEq ? equal_result ^ alu_af_dreg : compare_result;
    alu_finished = 1;

    // Hardwired shift signals
    alu_s_enable = function == afSll | function == afSrl;
    alu_s_function = (alu_af_dreg.read (), function[2]);
    alu_s_amount = op_b (4, 0);
}


// **************** Main ************************

void MExu::MainMethod() {
#pragma HLS ARRAY_PARTITION variable=gp_regs complete dim=1

    if(reset){
        ex_handle_reg = 0;
        state_reg = 0;
        sync_reg = 0;
        alu_result_reg = 0;
        ex_id_reg = 0;
        dbg_reg = 0;
    }else{
         // Exception registers
        ex_handle_reg = ex_handle.read();

        // State transition
        state_reg = state.read();

        // ParaNut Mode 1 sync register
        sync_reg = sync ? !sync_reg.read() : sync_reg.read();

        // ALU return register
        alu_result_reg = alu_result.read();
        alu_branch_reg = alu_branch.read();

        ex_id_reg = ex_id.read();

        // Debug mode register
        dbg_reg = dbg ? !dbg_reg.read() : dbg_reg.read();
    }

    // Write register
    if(gpr_write) {
        DumpRegisterChange (); // Display register change if instruction trace is enabled
        gp_regs[gpr_sel_dreg.read()] = gpr_in.read();
    }
    gp_regs[0] = 0;   // R0 is always zero
}

void MExu::DecodeMethod() {
    // **************** Instruction Decode Stage ***********************
    // *****************************************************************
    sc_uint<32> insn;
    sc_uint<32> op_a, op_b, gpr_b_val;
    sc_uint<3> alu_type;
    sc_uint<2> lsu_width_var;
    EOpRISCV opcode;
    bool insn_ebreak, insn_ecall, lres_scond;
    static bool last_ir_valid = 1;
    sc_uint<2> priv_mode;

    // Read input signals
    insn = ir.read ();
    opcode = (EOpRISCV) (__uint8_t)insn (6, 0);
    gpr_b_val = gp_regs[insn (24, 20)].read ();
    insn_ebreak = insn_ecall = 0;
    priv_mode = csr_priv_mode_reg.read ();
    lres_scond = 0;
    lsu_width_var = (insn[12], sc_uint<1> (!(insn[12] ^ insn[13])));

    if (!stall_decode | reset) {
        // Preset decode register signals (implicit reset?)
        // Execution stage
        exu_op_dreg = OpNOP;

        // Alu
        alu_type = RType;
        alu_af_dreg = 0;
        alu_branch_dreg = 0;
        alu_function_dreg = afAdd;
#if CFG_EXU_M_EXTENSION == 1
        alu_md_dreg = 0;
        alu_md_function_dreg = insn (13, 12);
#endif

        // CSR
        csr_adr_reg = insn (31, 20);
        csr_function_reg = insn (14, 12);
        csr_rs1_reg = insn (19, 15);

        // GP
        gpr_sel_dreg = insn (11, 7);

        // LSU/Memory
        lsu_rd_dreg = 0;

        // Special control signals
        illegal_insn_dreg = 0;
        ex_CoPU_dreg = 0;
        irq_dreg = 0;
        ecall_dreg = 0;
        ebreak_dreg = 0;
        flush_dreg = 0;
        cache_op_dreg = 0;
        mret_dreg = 0;
        irq_ign_dreg = 0;

        if (ir_valid && enable && !sync && !sync_reg && !internal_next && !sync_next) {
            // Register for single stepping
            step_dreg = csr_dcsr_step & !dbg_reg;

            // Instruction trace for simulation:
            InstructionTrace ();

            //            PN_ASSERTF(insn.value() == mainMemory->ReadWord((TWord) pc.read()),("Instruction
            //            does not match the memory at PC: Mem: (0x%08x) 0x%08x != Insn: 0x%08x", (TWord) pc.read(), (TWord)mainMemory->ReadWord((TWord) pc.read()), insn.value()));
            switch (opcode) {
            case OP:
                if (insn[25]) { // Bit 25 is set in M-Extension instructions
#if CFG_EXU_M_EXTENSION == 1
                    // (ALU, MUL/DIV) M-Extension  ...
                    alu_md_dreg = insn[14];
                    exu_op_dreg = OpMulDiv;
#else
                    PN_WARNINGF (("   (%s)  Illegal Instruction (M-Extension not activated): 0x%08x",
                               strrchr (name (), '.') + 1, insn.value ()));
                    illegal_insn_dreg = 1;
                    exu_op_dreg = OpException;
#endif
                } else {
                    // (ALU) ALU without immediate...
                    alu_function_dreg = insn (14, 12);
                    alu_af_dreg = (bool)insn[30];
                    exu_op_dreg = OpAlu;
                }
                perf_mon_.Count (EV_ALU);
                break;
            case OP_IMM:
                // (ALU) ALU with immediate...
                alu_type = IType;
                alu_function_dreg = insn (14, 12);
                // AF only applicable for SRLI -> SRAI else it MUST be 0
                alu_af_dreg = ((__uint8_t)insn (14, 12) == afSrl) ?
                              (bool)insn[30] :
                              0;
                exu_op_dreg = OpAlu;
                perf_mon_.Count (EV_ALU);
                break;
            case LUI:
            case AUIPC:
                // (ALU) ALU add 20 Bit upper immediate to register or PC...
                alu_type = UType;
                exu_op_dreg = OpAlu;
                perf_mon_.Count (EV_ALU);
                break;
            case LOAD:
                // (LS) Load...
                alu_type = IType;
                lsu_rd_dreg = 1;
                exu_op_dreg = OpMem;
                perf_mon_.Count (EV_LOAD);
                break;
            case STORE:
                // (LS) Store...
                alu_type = SType;
                exu_op_dreg = OpMem;
                perf_mon_.Count (EV_STORE);
                break;
            case JAL:
            case JALR:
                // (JAL) Jump and Link...
                alu_type = insn[3] ? JType : IType;
                exu_op_dreg = OpJump;
                perf_mon_.Count (EV_JUMP);
                break;
            case BRANCH:
                // (BRANCH) Conditional Branch...
                // bit 14:    BLT/BGE/BLTU/BGEU or BEQ/BNEQ
                // -> bit 13: BLTU/BGEU or BLT/BGE;
                alu_function_dreg = insn[14] ? (insn[13] ? afBltu : afBlt) : afEq;

                alu_af_dreg = (insn[12]); // AF needed for BGE/BGEU and BEQ;
                alu_branch_dreg = 1;
                exu_op_dreg = OpBranch;
                perf_mon_.Count (EV_JUMP);
                break;
            case SYSTEM:
                // (OTHER) SYS / CSR / TRAP ...
                if ((insn (14, 12) == 0)) {
                    // MRET, ECALL, EBREAK instruction
                    bool is_trap_return = insn[21];

                    if (!is_trap_return) {
                        insn_ebreak =  insn[20] & ~insn[22] &  // EBREAK instruction
                                  (!linked | inCePU);       // only CePU executes ebreak in linked mode
                        insn_ecall = ~insn[20];
                        ecall_dreg = insn_ecall;
                        exu_op_dreg = insn_ebreak | insn_ecall ? OpException : OpAlu; // otherwise WFI
#if CFG_PRIV_LEVELS == 1
                    } else if (insn[28] & insn[29]) {
                        exu_op_dreg = OpXRet; // MRET
#elif CFG_PRIV_LEVELS == 2
                    } else if (insn[28] & insn[29] & (priv_mode == Machine)) {
                        exu_op_dreg = OpXRet; // MRET only allowed in M-Mode
#elif CFG_PRIV_LEVELS == 3
                    } else if (insn[28] & priv_mode[0]) { // at least SRET
                        exu_op_dreg = OpXRet;
                        if (insn[29] & (priv_mode[1])) { // is MRET
                            mret_dreg = 1;
                        } else if (insn[29] & ~(priv_mode[1])) {
                            exu_op_dreg = OpException;
                            illegal_insn_dreg = 1;
                        }
#endif
                    } else { // raise Illegal Instruction Exception when unknown or unprivileged Instruction
                        exu_op_dreg = OpException;
                        illegal_insn_dreg = 1;
                    }
                } else {
                    // CSR instruction
                    exu_op_dreg = OpSystem;
                }
                perf_mon_.Count (EV_OTHER);
                break;
            case MISC_MEM:
                // (OTHER) FENCE / FENCE.I instruction ...
                alu_type = JType;
                flush_dreg = 1;
                exu_op_dreg = OpParaNut;
                perf_mon_.Count (EV_OTHER);
                break;
            case PARA:
                // (LS, ParaNut extension) ...
                // PN_CFLUSH, PN_CINVALIDATE, PN_CWRITEBACK
                lsu_width_var = 3; // Set lsu_width to 0b11 (non valid value)
                cache_op_dreg = insn(14, 12);
                exu_op_dreg = OpParaNut;
                alu_type = IType;
                perf_mon_.Count (EV_OTHER);
                break;
#if CFG_EXU_A_EXTENSION == 1
            case AMO:
                // (LR/SC, AMO) A-Extension ...
                exu_op_dreg = OpMem;
                lsu_rd_dreg = !insn[27];
                flush_dreg = 1;
                lres_scond = 1;
                perf_mon_.Count (EV_OTHER);
                break;
#endif
            default:
                // Illegal instruction:
                PN_WARNINGF (("   (%s)  Illegal Instruction: (0x%08x): 0x%08x",
                           strrchr (name (), '.') + 1, pc.read (), ir.read ()));
                illegal_insn_dreg = 1;
                exu_op_dreg = OpException;
                perf_mon_.Count (EV_OTHER);
                break;
            } // switch (opcode)
        } // if(ir_valid && enable)
        else if (last_ir_valid == 1){
            perf_mon_.Count (EV_IFETCH);
        }

        // Enter debug mode if necessary
        if ((dbg_req & !dbg & !dbg_reg)         // Debug request from DM
            | (step_dreg & !ex_handle_reg)      // Single stepping active
            | (insn_ebreak & (csr_dcsr_ebreakm.read () | dbg_reg.read ()))) // Ebreak instruction with dcsr_ebreakm set
        {
            step_dreg = 0;
            exu_op_dreg = OpDebug;
        }

        // Handle interrupts or CoPU exceptions (architectural like interrupts)
        if (!ex_handle_reg & !csr_dcsr_step & !dbg_reg) { // Interrupts are disabled during single stepping
            ex_CoPU_dreg = ex_i.read (); // CoPU exceptions have priority over interrupts
#if CFG_PRIV_LEVELS <= 2
            irq_dreg = m3_ir_request.read () & !ex_i.read ();
            if (ex_i | m3_ir_request) {
#elif CFG_PRIV_LEVELS == 3
            bool ignore_interrupt;

            if (priv_mode == User) {
              ignore_interrupt = 0;
            } else if (priv_mode == Supervisor) {
              bool deleg_irq = (1 << csr_mideleg.read ()) == m3_ir_id.read ();
              ignore_interrupt = deleg_irq ? csr_mstatus_SIE : 0;
            } else {
              ignore_interrupt = csr_mstatus_MIE;
            }

            irq_ign_dreg = ignore_interrupt;
            irq_dreg = ignore_interrupt ? 0 : m3_ir_request.read () & !ex_i.read ();
            if ((ex_i | m3_ir_request) & ~irq_ign_dreg.read ()) {
#endif
                exu_op_dreg = OpException;
            }
        }

        // Determine Alu inputs ...
        switch (alu_type) {
        case IType: // ADDI/SUBI...: 12 Bit sign extended Immediate
            op_a = gp_regs[insn (19, 15)].read ();
            op_b = ((insn >> 20) ^ 0x800) - 0x800;
            break;
        case SType: // Store: 12 Bit sign extended Immediate (misaligned)
            op_a = gp_regs[insn (19, 15)].read ();
            op_b = ((insn (31, 25), insn (11, 7)) ^ 0x800) - 0x800;
            break;
        case UType: // LUI: 20 Bit Immediate
            // LUI and AUIPC use Type U:
            if (insn[5] == 1)
                op_a = 0; // Zero
            else
                op_a = pc.read ();
            op_b = insn (31, 12) << 12;
            break;
        case JType: // JAL: PC and 20 Bit sign-extended Immediate
            op_a = pc.read ();
            op_b = (sc_int<32>)sc_int<21> ((insn[31], insn (19, 12), insn[20],
                                            insn (30, 25), insn (24, 21), sc_uint<1> (0)));
            break;
        default: // default R-type
            op_a = gp_regs[insn (19, 15)].read ();
            op_b = lres_scond ? sc_uint<32>(0) : gpr_b_val; // special case for LR/SC instructions
            break;
        }
        op_a_dreg = op_a;
        op_b_dreg = op_b;
        // B-type encoding for branch operands A and B
        branch_a_dreg = pc.read ();
        branch_b_dreg = (sc_uint<32>)((sc_int<32>)sc_int<13> (
        (insn[31], insn[7], insn (30, 25), insn (11, 8), sc_uint<1> (0))));

        // MLsu
        lsu_width = lsu_width_var;
        lsu_exts = ~insn[14];
        lsu_wdata = gpr_b_val;
        lres_scond_dreg = lres_scond;

    } // if(!stall_decode)


    last_ir_valid = ir_valid.read();

    if (reset) {
        exu_op_dreg = OpNOP;

        // Alu
        alu_type = RType;
        alu_af_dreg = 0;
        alu_branch_dreg = 0;
        alu_function_dreg = afAdd;
#if CFG_EXU_M_EXTENSION == 1
        alu_md_dreg = 0;
        alu_md_function_dreg = insn (13, 12);
#endif

        // CSR
        csr_adr_reg = insn (31, 20);
        csr_function_reg = insn (14, 12);
        csr_rs1_reg = insn (19, 15);

        // GP
        gpr_sel_dreg = insn (11, 7);

        // LSU/Memory
        lsu_rd_dreg = 0;
        lres_scond_dreg = 0;

        // Special control signals
        illegal_insn_dreg = 0;
        ex_CoPU_dreg = 0;
        irq_dreg = 0;
        ecall_dreg = 0;
        ebreak_dreg = 0;
        flush_dreg = 0;
        cache_op_dreg = 0;
    }
}

void MExu::MainCombMethod () {
    EExceptions ex_id_var;
    bool ex, ex_o_var, ifu_next_var, ifu_jump_var, jmp_adr_mis;
    EExuState state_next;
    sc_uint<32> jmp_adr, alu_result_var;
    EExuOp opcode;
    sc_uint<2> priv_mode;

    // Read input signals ...
    opcode = (EExuOp) (__uint16_t)exu_op_dreg.read ();
    state_next = (EExuState) (__uint16_t)state_reg.read ();
    alu_result_var = alu_result.read ();
    priv_mode = csr_priv_mode_reg.read ();

    // Preset control signals ...
    jmp_adr (31, 1) = (alu_result_reg.read () (31, 1));
    jmp_adr[0] = alu_result_reg.read ()[0] &
                 exu_op_dreg.read ()[2]; // remove last bit (todo: specification only requires this for JALR)
    jmp_adr_mis = jmp_adr (1, 0) & 3; // jump address missaligned signal

    ex_id_var = InstructionAddressMisaligned;
    ex = 0;


    // MIfu
    ifu_next_var = 0;
    ifu_jump_var = 0;
    ifu_flush = 0;
    ifu_jump_adr = jmp_adr;
    internal_next = 0;
    // MLsu
    lsu_adr = alu_result_reg.read ();
    lsu_lres_scond = 0;
    lsu_cache_op = 0;
    lsu_flush = 0;
    lsu_rd = 0;
    lsu_wr = 0;
    // CSR
    csr_instr = 0;
    csr_op_a = op_a_dreg.read (); // Connect op_a_dreg to csr_op_a
    // GP register input multiplexer
    switch (opcode) {
    case OpMem: // For SC we need to save !lsu_scond_ok
        gpr_in = lres_scond_dreg & !lsu_rd_dreg ? (TWord)!lsu_scond_ok.read () : lsu_rdata.read ();
        break;
    case OpJump:
        gpr_in = pc.read () + 4; // ifu_npc.read (); TODO: Which solution needs less logic: 32 Bit bus from IFU0 or Adder?
        break;
    case OpSystem:
        gpr_in = csr_rdata.read ();
        break;
#if CFG_EXU_M_EXTENSION == 1
    case OpMulDiv:
        gpr_in = alu_md_dreg ? alu_d_result.read () : alu_m_result.read ();
        break;
#endif
    default:
        gpr_in = alu_result_var;
        break;
    }
    gpr_write = 0;
    // Debug signals
    dbg = 0;
    // Exception signals
    ex_handle = ex_handle_reg.read ();
    exception = 0;
    pop_priv_ir_stack_dreg = 0;
    m3_ir_ack = 0;
#if CFG_PRIV_LEVELS == 3 
    m3_ir_ack = irq_ign_dreg.read ();
#endif

    // Exception daisy chain:
    cause_o = xsel ? csr_mcause.read () (4, 0) : cause_i.read ();
    epc_o = xsel ? csr_mepc.read () : epc_i.read ();

    // Differen ParaNut control signals for CePU and CoPUs:
    if (inCePU) {
        // Set and reset ex_o as soon as possible
        ex_o_var = ((opcode == OpException) | ex_handle_reg) &  // Set on exception entry and hold
                   (opcode != OpXRet);                        // Reset on xret operation
        csr_tval = jmp_adr; // Exception information
        m3_ir_enable = csr_mstatus_MIE.read ();
#if CFG_PRIV_LEVELS == 2
        m3_ir_enable = priv_mode == User ? 1 : csr_mstatus_MIE.read ();
#elif CFG_PRIV_LEVELS == 3
        /* Interrupts are always enabled.
         * However, in some cases they are ignored by simply acking theme in the DecodeMethod */
        m3_ir_enable = 1;
#endif
    } else {
        ex_o_var = 0;
        csr_tval = 0;
        m3_ir_enable = 0;
    }
    haltreq = 0;

#if CFG_EXU_M_EXTENSION == 1
    alu_m_enable = 1; // Multiplication only takes one clock anyways
    alu_d_enable = 0;
#endif

    // Set perf_inc to 0
    PERFSTD ();

    // *********************** Execute Stage ***************************
    // *****************************************************************
    switch (state_next) {
    case ExuExecuteInsn: // Handle single cycle instructions
//        PN_ASSERTF (insn.value () == mainMemory->ReadWord ((TWord)pc.read ()),
//                 ("Instruction does not match the memory at PC: Mem: (0x%08x) 0x%08x != Insn: "
//                  "0x%08x",
//                  (TWord)pc.read (), (TWord)mainMemory->ReadWord ((TWord)pc.read ()), insn.value ()));
        switch (opcode) {
        case OpAlu: // todo: MEALY
            // (ALU) ALU operation (Completes in 1 Cycle)
            gpr_write = 1;
            ifu_next_var = 1;
            PERFINC (perfALU);
            break;
        case OpMem:
            // (LS) Load/Store...
            state_next = lres_scond_dreg ? ExuLSUFlush : ExuMem;
            break;
        case OpJump:
            // (JAL) Jump and Link...
            state_next = ExuJump;
            break;
        case OpBranch:
            state_next = ExuBranch;
            break;
        case OpXRet:
            state_next = ExuXRET;
            break;
        case OpSystem:
            // (OTHER) CSR
            state_next = ExuCSR;
            break;
        case OpParaNut:
            if (flush_dreg)
                state_next = ExuLSUFlush;
            else if (cache_op_dreg.read () != 0)
                state_next = ExuCACHECONT2;
            else
                state_next = ExuHalt;
            break;
        case OpException:
            state_next = ExuExOrIrq;
            break;
        case OpDebug:
            state_next = ExuDBG;
            break;
#if CFG_EXU_M_EXTENSION == 1
        case OpMulDiv:
            if (alu_md_dreg) // Currently multiplication finishes in one cycle
                state_next = ExuDiv;
            else
                state_next = ExuMulDivWB;
            break;
#endif
        default: // OpNOP
            // Nothing
            break;
        } // switch (opcode)
        break;
    // ALU execution states:
    // -----------------------------------------
//    case ExuAluWB:
//        perf_mon_.Count(evALU);
//        PERFINC(perfALU);
//        INSTRET_SET_TRUE;
//        break;
    // LOAD/STORE execution states:
    // -----------------------------------------
    case ExuMem: // (LS) Load/Store...
        lsu_rd = lsu_rd_dreg.read (); // Could be set without the state
        lsu_wr = !lsu_rd_dreg.read (); // Could be set without the state
        lsu_lres_scond = lres_scond_dreg.read ();

        ex_id_var = lsu_rd_dreg ? LoadAddressMisaligned : StoreAddressMisaligned;

        if (lsu_align_err)
            state_next = ExuExOrIrq;
        else {
            if (lsu_ack) { // Wait for acknowledge
                state_next = ExuMemWB;
                // One could already assert 'ifu_next_var' here but doing so yields
                // a long critical path from MEMU -> LSU -> EXU -> IFU
                // so no mealy here based on 'lsu_ack'!
            }
        }
        // Write GPR on scond (needs to happen before we go to ExuMemWB state)
        gpr_write = !lsu_rd_dreg.read () & lres_scond_dreg.read ();

        PERFINC (perfLoadStoreWait);
//        if (lsu_align_err) // alignment error
//            PN_WARNINGF (("Load address misaligned: 0x%08x, Instruction: (0x%08x) = 0x%08x",
//                       (__uint32_t)alu_result.read (), pc.read (), (__uint32_t)ir.read ()));
        break;
    case ExuMemWB: // (LS) Load Write Back (we are delayed one clock) ...
        // Write GPR on load/lres
        gpr_write = lsu_rd_dreg.read ();

        if (lsu_rd_dreg) {
            PERFINC (perfLoad);
        } else {
            PERFINC (perfStore);
        }
        ifu_next_var = 1;
        state_next = ExuExecuteInsn;
        break;
    // JUMP/BRANCH execution states:
    // -----------------------------------------
    case ExuJump: // (JAL) Jump and Link...
        // alignment check
        ex_id_var = InstructionAddressMisaligned;
//        if (jmp_adr_mis)
//            PN_WARNINGF (("   (%s)  Jump: Instruction address misaligned: 0x%08x, Instruction: "
//                       "(0x%08x) = 0x%08x",
//                       strrchr (name (), '.') + 1, (__uint32_t)alu_result.read (),
//                       (__uint32_t)pc.read (), (__uint32_t)ir.read ()));
//         else
//             PN_INFOF (("Instruction: (0x%08x) = 0x%08x Jumping to 0x%x  ", (__uint32_t)pc.read (),
//                     insn.value (), adr));
        // Save return address (npc)
        gpr_write = !jmp_adr_mis;

        // jump
        ifu_jump_var = !jmp_adr_mis;
        ifu_next_var = !jmp_adr_mis;
        if (jmp_adr_mis)
            state_next = ExuExOrIrq;
        else
            state_next = ExuExecuteInsn;
        PERFINC (perfJump);
        //			}
        break;
    case ExuBranch: // (BRANCH) Conditional Branch Decision ...
        // alignment check
        ex_id_var = InstructionAddressMisaligned;
//        if(jmp_adr_mis) // todo: needs to change for 16 bit support
//          PN_WARNINGF (("Branch: Instruction address misaligned: 0x%08x, Instruction: "
//                     "(0x%08x) = 0x%08x",
//                     alu_result.read (), pc.read (), insn.value ()));
//        else
//            PN_INFOF (("Instruction: (0x%08x) = 0x%08x Branching to 0x%x  ", (__uint32_t)pc.read (),
//                    insn.value (), adr));

        ifu_jump_var = !jmp_adr_mis & alu_branch_reg;
        ifu_next_var = !(jmp_adr_mis & alu_branch_reg);
        PERFINC (perfJump);
        if (jmp_adr_mis & alu_branch_reg)
            state_next = ExuExOrIrq;
        else
            state_next = ExuExecuteInsn;
        break;
    // SYSTEM execution states:
    // -----------------------------------------
    case ExuCSR: // (OTHER) CSR
        csr_instr = 1;

        // Write GPR
        gpr_write = 1;

        // Only continue if csr_exception wasn't set
        ifu_next_var = !csr_exception;

        if (csr_exception.read ())
            state_next = ExuExOrIrq;
        else
            state_next = ExuExecuteInsn;
        PERFINC (perfSystem);
        break;
    case ExuXRET: // (OTHER) xRET to pc saved in mepc or dpc
        ex_handle = 0;
        dbg = dbg_reg.read ();
#if CFG_PRIV_LEVELS <= 2
        ifu_jump_adr = dbg_reg ? csr_dpc.read () : csr_mepc.read ();
#elif CFG_PRIV_LEVELS == 3
        ifu_jump_adr = dbg_reg ? csr_dpc.read () : mret_dreg.read () ? csr_mepc.read () : csr_sepc.read ();
#endif

        ifu_jump_var = 1;
        ifu_next_var = 1;
        pop_priv_ir_stack_dreg = 1;
        
        state_next = ExuExecuteInsn;

        PERFINC (perfSystem);
        break;
    // PARANUT execution states:
    // -----------------------------------------
    case ExuHalt: //  (LS, ParaNut extension) ...
        // Send haltreq to CePU
        haltreq = 1;

        // Wait for CePU to disable this CoPU
        if (!enable) {
            state_next = ExuExecuteInsn;
        }

        PERFINC (perfSystem);
        break;
    case ExuLSUFlush: // (LS, ParaNut extension) LSU Flush ...
        // Send flush signals
        ifu_flush = flush_dreg.read ();
        lsu_flush = flush_dreg.read ();

        //  Wait for LSU to finish flush
        if (lsu_ack)
            state_next = lres_scond_dreg ? ExuMem : ExuCACHECONT;
        break;
    case ExuCACHECONT: // (LS, ParaNut extension) LSU Cache Control Delay ...
        // we have to delay cache control signals one clock
        state_next = ExuCACHECONT2;
        PERFINC (perfSystem); // CacheControl gets counted in this state
        break;
    case ExuCACHECONT2: // (LS, ParaNut extension) LSU Cache Control ...
        //   set cache control signals...
        lsu_wr = cache_op_dreg.read () != 0;
        lsu_cache_op = cache_op_dreg.read ();
        
        //  Wait for LSU ack or return if neither wb. or inv. are set
        if (lsu_ack | (cache_op_dreg.read () == 0)) {
            state_next = ExuMemWB;
            // One could already assert 'ifu_next_var' here but doing so yields
            // a long critical path from MEMU -> LSU -> EXU -> IFU
            // so no mealy here based on 'lsu_ack'!
        }
        break;
    // EXCEPTION execution states:
    // -----------------------------------------
    case ExuExOrIrq:
        ex_handle = 1;
        m3_ir_ack = irq_dreg.read (); // Send ack to INTC
        exception = 1; // Trigger saving of the exception state CSRs (mepc, mcause, ...)
        if (inCePU) {
            state_next = ExuExWaitCoPUs;
        } else {
            // CoPU exceptions..
            if (illegal_insn_dreg | csr_exception | ebreak_dreg | ecall_dreg | !ex_i)
                state_next = ExuExCoPUHalt; // This CoPU encountered an exception
            else
                state_next = ExuExCoPUWaitCePU; // Other CPU encountered an exception
        }
        break;
    case ExuExCoPUHalt:
        ex_o_var = 1;
        haltreq = 1; // set haltreq as long as we are enabled and ex_i s not set
        if (!enable) {
            state_next = ExuExCoPUWaitHandle;
            ifu_next_var = 1; // CoPUs with handled exception just fetch next instruction
        }
        break;
    case ExuExCoPUWaitHandle:
        ex_o_var = 1;
        // Wait till the exception gets handled by the CePU
        // (CoPU gets enabled during exception handler)
        if (ex_i & enable) {
            state_next = ExuExCoPUWaitCePU;
            ifu_next_var = 1; // CoPUs with handled exception just fetch next instruction
        }
        break;
    case ExuExCoPUWaitCePU:
        // Hold ex_o as long as the CePU
        ex_handle = 0;
        ex_o_var = 1;
        if (ex_i == 0) {
            state_next = ExuExecuteInsn;
        }
        break;
    case ExuExWaitCoPUs:
        // Wait for all CoPUs to get into the Exception State
        if (m3_pnx.read ().and_reduce ())
            // Enter debug mode if we encounterd an exception during debug or stepping
            state_next = csr_dcsr_step | dbg_reg ? ExuDBG : ExuExJumpMtvec;
        break;
    case ExuExJumpMtvec:
        // Jump to exception routine...
#if CFG_PRIV_LEVELS == 3
        ifu_jump_adr = csr_delegate_dreg ? csr_stvec.read () : csr_mtvec.read ();
#else
        ifu_jump_adr = csr_mtvec.read ();
#endif
        ifu_jump_var = 1;
        ifu_next_var = 1;
        state_next = ExuExecuteInsn;
        break;
        // Special states (for extenstions etc.):
        // -----------------------------------------
#if CFG_EXU_M_EXTENSION == 1
    case ExuDiv: // (ALU, DIV) M-Extension  ...
        alu_d_enable = 1;
        if (alu_d_valid) // Wait for valid signal
            state_next = ExuMulDivWB;
        break;
    case ExuMulDivWB: // (ALU, MUL/DIV) M-Extension  ...
        // Regfile
        gpr_write = 1;

        ifu_next_var = 1;
        state_next = ExuExecuteInsn;
        PERFINC (perfALU);
        break;
#endif
    // Special debug states:
    // -----------------------------------------
    case ExuDBG:
        dbg = !dbg_reg.read ();
        ex_handle = 0; // Reset regHandleEx before entering debug mode
        // Jump to debug rom ...
        ifu_jump_adr = ex_handle_reg ? 0x408 : 0x400;
        ifu_jump_var = 1;
        ifu_next_var = 1;
        state_next = ExuExecuteInsn;
        break;
    default:
        PN_ERRORF (("EXU is in unknown state: %d Instruction: 0x%08x PC: 0x%08x",
                 state_reg.read ().value (), ir.read (), pc.read ()));
        break;
    } // switch(newExuState)

    // *********************** Exception Stage *************************
    // *****************************************************************

    // Handle Exception...
//    if (ex) {
//        state_next = ExuExOrIrq;
//    } // if (ex)

    // exceptionId multiplexer
    // TODO: prioritize exceptions
    if (illegal_insn_dreg | csr_exception | csr_rd_exception)
        ex_id_var = IllegalInstruction;
    else if (ecall_dreg)
        ex_id_var = (EExceptions)(int)(ECallU | priv_mode);
    else if (ebreak_dreg)
        ex_id_var = Breakpoint;
    else if (ex_CoPU_dreg)
        ex_id_var = CoPUException;
    else if (irq_dreg)
        ex_id_var = (EExceptions)(int)m3_ir_id.read ();

    ex_id = ex_id_var;
    ex_o = ex_o_var;

    // ************************* Output Stage **************************
    // *****************************************************************

    // Set state signal
    state = state_next;

    // Stall decode for slow instructions
    stall_decode = enable & // Only enabled CPUs
                   (state_reg.read () != ExuExCoPUWaitCePU) & // CoPUs must not stall while in exception handling state
                   !((state_reg.read () == ExuExecuteInsn) & (opcode == OpAlu | opcode == OpNOP)) & // CPUs must not stall in the wait state and for single-cycle operations
                   !ifu_next_var; // Quick disable of stall_decode when the current instruction is finished


    // IFU reset if CPU is not enabled, linked or temporarly disabled because of an exception
    if (mode2Cap)
        ifu_reset = (!enable & !linked & !ex_o_var);
    else // no IFU in Mode 1 only CoPUs
        ifu_reset = 0;

    if (inCePU) {
        // Set ifu_next & ifu_jump signal (just jump in CePU)
        ifu_next = internal_next.read();
        ifu_jump = ifu_jump_var;
        internal_next = linked & !ex_handle_reg & !dbg & !dbg_reg ? ((sync_reg ^ ifu_next_var)) & sync_i.read () : ifu_next_var;

        // External sync signal from CePU to sync_next inputs of all CoPUs ...
        sync_o = (linked & enable) & sync_i & (dbg_reg == dbg) & !step_dreg? ifu_next_var | sync_reg : 0;

        // Internal sync control signal for Mode 1 control
        sync = (linked & !ex_handle_reg & ifu_next_var & !sync_i)  // Set when we finished our own instruction (
               | (sync_i & sync_reg);                              // Reset when all linked CPUs finished

    } else {
        // Set ifu_next & ifu_jump signal (block IFU signals if running in Mode 1)
        ifu_next = internal_next.read();
        ifu_jump = linked ? 0 : ifu_jump_var;
        internal_next = linked ? 0 : ifu_next_var;


        // External sync signal daisy chain ....
        sync_o = (linked & enable) & !(sync_reg | ifu_next_var) ? 0 : sync_i.read ();

        // Internal sync control signal for Mode 1 control
        sync = (linked & !ex_handle_reg & ifu_next_var & !sync_next) // Set when we finished our own instruction
               | (sync_next & sync_reg); // Reset when all linked CPUs finished
    }
}

void MExu::Mode2Method () {
    if (inCePU | !mode2Cap) {
        // CePU and Mode 1 CoPUs
        ir = ifu_ir.read ();
        pc = ifu_pc.read ();
        ir_valid = ifu_ir_valid.read ();
    } else {
        // Mode 2 capable CoPUs have to switch to special input ports when linked
        ir = linked ? m2_ir.read () : ifu_ir.read ();
        pc = linked ? m2_pc.read () : ifu_pc.read ();
        ir_valid = linked ? m2_ir_valid.read () : ifu_ir_valid.read ();
    }
}

void MExu::CSRMethod () {
    csr_pop_priv_ir_stack_dreg = pop_priv_ir_stack_dreg.read ();
    csr_enable = csr_instr.read ();
    csr_cpu_enabled = enable.read ();
    csr_exu_exception = exception.read ();
    csr_irq_dreg = irq_dreg.read ();
    csr_dbg = dbg.read ();
    csr_dbg_reg = dbg_reg.read ();
    csr_csr_rs1_reg = csr_rs1_reg.read ();
    csr_csr_op_a = csr_op_a.read ();
    csr_csr_function_reg = csr_function_reg.read ();
    csr_csr_adr_reg = csr_adr_reg.read ();
    csr_ex_id_reg = ex_id_reg.read ();
    csr_mret_dreg = mret_dreg.read ();
    csr_csr_tval = csr_tval.read ();
    csr_cause = cause_i.read ();
    csr_epc = epc_i.read ();

    csr_pc = ifu_pc.read ();
    csr_ir = ifu_ir.read ();

    csr_hartID = hartID.read ();
    csr_clock_freq_hz = clock_freq_hz.read ();
    csr_inCePU = inCePU.read ();
    csr_linked = linked.read ();
#if CFG_EXU_PERFCOUNT_ENABLE == 1
    // Perfcount signals ...
    csr_perf_inc = perf_inc.read ();
    csr_perf_addr = perf_addr.read ();
#endif

    m3_pnxsel = csr_m3_pnxsel.read ();
    m3_pnce = csr_m3_pnce.read ();
    m3_pnlm = csr_m3_pnlm.read ();

    csr_m3_pnhaltreq = m3_pnhaltreq.read ();
    csr_m3_pnx = m3_pnx.read ();

    m3_icache_enable = csr_m3_icache_enable.read ();
    m3_dcache_enable = csr_m3_dcache_enable.read ();
}
