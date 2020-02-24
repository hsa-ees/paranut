/*************************************************************************

  This file is part of the ParaNut project.

  Copyright (C) 2010-2020 Alexander Bahle <alexander.bahle@hs-augsburg.de>
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


// **************** Tracing *********************

#ifndef __SYNTHESIS__
void MExu::Trace (sc_trace_file *tf, int level) {
    if (!tf || trace_verbose) printf ("\nSignals of Module \"%s\":\n", name ());

    // Ports ...
    TRACE (tf, clk);
    TRACE (tf, reset);
    //   to IFU ...
    TRACE (tf, ifu_next);
    TRACE (tf, ifu_jump);
    TRACE (tf, ifu_reset);
    TRACE (tf, ifu_jump_adr);
    TRACE (tf, ifu_ir_valid);
    TRACE (tf, ifu_npc_valid);
    TRACE (tf, ifu_ir);
    TRACE (tf, ifu_pc);
    TRACE (tf, ifu_npc);
    //   to Load/Store Unit (LSU)...
    TRACE (tf, lsu_rd);
    TRACE (tf, lsu_wr);
    TRACE (tf, lsu_flush);
    TRACE (tf, lsu_cache_invalidate);
    TRACE (tf, lsu_cache_writeback);
    TRACE (tf, lsu_ack);
    TRACE (tf, lsu_align_err);
    TRACE (tf, lsu_width);
    TRACE (tf, lsu_exts);
    TRACE (tf, lsu_adr);
    TRACE (tf, lsu_rdata);
    TRACE (tf, lsu_wdata);

    TRACE (tf, m3_icache_enable);
    TRACE (tf, m3_dcache_enable);
    TRACE (tf, m3_ir_request);


    TRACE (tf, m2_pc);
    TRACE (tf, m2_ir);
    TRACE (tf, m2_ir_valid);

    TRACE (tf, sync_reg);
    TRACE (tf, sync);
    TRACE (tf, sync_i);
    TRACE (tf, sync_o);
    TRACE (tf, sync_next);
    TRACE (tf, m3_pnce);
    TRACE (tf, m3_pnlm);

    TRACE (tf, ir);
    TRACE (tf, ir_valid);
    TRACE (tf, pc);

    // to CePU
    TRACE (tf, ex_i);
    TRACE (tf, haltreq);
    TRACE (tf, ex_o);
    TRACE (tf, m3_pnx);
    TRACE (tf, epc_i);
    TRACE (tf, epc_o);
    TRACE (tf, cause_i);
    TRACE (tf, cause_o);
    TRACE (tf, m3_pnxsel);
    TRACE (tf, xsel);

    TRACE (tf, enable);
    TRACE (tf, linked);
    TRACE (tf, m3_pnhaltreq);

    TRACE (tf, ex_handle_reg);
    TRACE (tf, ex_handle);
    TRACE (tf, exception);
    TRACE (tf, ex_id);

    TRACE (tf, dbg_req);
    TRACE (tf, dbg);
    TRACE (tf, dbg_reg);
    TRACE (tf, step_dreg);
    // Registers...
    TRACE_BUS (tf, gp_regs, REGISTERS);
    TRACE (tf, gpr_write);
    TRACE (tf, gpr_in);
    TRACE (tf, gpr_sel_dreg);

    TRACE (tf, exu_op_dreg);
    TRACE (tf, stall_decode);
    TRACE (tf, state_reg);
    TRACE (tf, state);

    // CSRs
    TRACE (tf, csr_mcause);
    TRACE (tf, csr_mepc);
    TRACE (tf, csr_mtval);
    TRACE (tf, csr_mtvec);
    TRACE (tf, csr_mstatus_MIE);
    TRACE (tf, csr_mstatus_MPIE);
    TRACE (tf, csr_dcsr_cause);
    TRACE (tf, csr_dcsr_step);
    TRACE (tf, csr_dcsr_ebreakm);
    TRACE (tf, csr_dpc);
    TRACE (tf, csr_dscratch0);
    TRACE (tf, csr_mscratch);
    TRACE (tf, csr_pncache);
    TRACE_BUS (tf, csr_pnxsel, CFG_NUT_CPU_GROUPS + 1);
    TRACE_BUS (tf, csr_pnx, CFG_NUT_CPU_GROUPS + 1);
    TRACE_BUS (tf, csr_pnce, CFG_NUT_CPU_GROUPS + 1);
    TRACE_BUS (tf, csr_pnlm, CFG_NUT_CPU_GROUPS + 1);

    TRACE (tf, csr_adr_reg);
    TRACE (tf, csr_enable);
    TRACE (tf, csr_function_reg);
    TRACE (tf, csr_op_a);
    TRACE (tf, csr_rdata);
    TRACE (tf, csr_rs1_reg);
    TRACE (tf, csr_exception);
    TRACE (tf, csr_rd_exception);
    TRACE (tf, csr_wdata);
    TRACE (tf, csr_rdata);
    TRACE (tf, csr_write);

    // Alu signals
    TRACE (tf, alu_finished);
    TRACE (tf, alu_function_dreg);
    TRACE (tf, alu_result);
    TRACE (tf, alu_af_dreg);
    TRACE (tf, alu_s_ready);
    TRACE (tf, op_a_dreg);
    TRACE (tf, op_b_dreg);
    TRACE (tf, alu_branch_dreg);
    TRACE (tf, alu_branch);
    TRACE (tf, alu_s_amount);
    TRACE (tf, alu_s_result);
    TRACE (tf, alu_s_function);
    TRACE (tf, alu_s_enable);
#if CFG_EXU_M_EXTENSION == 1
    TRACE (tf, alu_d_enable);
    TRACE (tf, alu_m_valid);
    TRACE (tf, alu_m_result);
    TRACE (tf, alu_md_dreg);
    TRACE (tf, alu_d_valid);
    TRACE (tf, alu_d_result);
    // MExtension Sub-Module
    if (level > 1) {
        level--;
        mext.Trace (tf, level);
    }
#endif // CFG_EXU_M_EXTENSION == 1

	TRACE (tf, csr_mcycle);
    TRACE_BUS (tf, csr_mhpmcounter, CFG_EXU_PERFCOUNTERS);
}
#endif // __SYNTHESIS__


// ************** Simulation Helpers ******************

void MExu::DumpRegisterInfo () {
    INFOF (("   (%s)  R1/ra=%08x   R2/sp=%08x   R3/gp=%08x    R4/tp=%08x   R5/t0=%08x   R6/t1=%08x "
            "  R7/t2=%08x",
            strrchr (name (), '.') + 1, gp_regs[1].read (), gp_regs[2].read (), gp_regs[3].read (),
            gp_regs[4].read (), gp_regs[5].read (), gp_regs[6].read (), gp_regs[7].read ()));
    INFOF (("   (%s)  R8/s0=%08x   R9/s1=%08x  R10/a0=%08x   R11/a1=%08x  R12/a2=%08x  R13/a3=%08x "
            " R14/a4=%08x  R15/a5=%08x",
            strrchr (name (), '.') + 1, gp_regs[8].read (), gp_regs[9].read (), gp_regs[10].read (),
            gp_regs[11].read (), gp_regs[12].read (), gp_regs[13].read (), gp_regs[14].read (),
            gp_regs[15].read ()));
    INFOF (("   (%s) R16/a6=%08x  R17/a7=%08x  R18/s2=%08x   R19/s3=%08x  R20/s4=%08x  R21/s5=%08x "
            " R22/s6=%08x  R23/s7=%08x",
            strrchr (name (), '.') + 1, gp_regs[16].read (), gp_regs[17].read (),
            gp_regs[18].read (), gp_regs[19].read (), gp_regs[20].read (), gp_regs[21].read (),
            gp_regs[22].read (), gp_regs[23].read ()));
    INFOF (("   (%s) R24/s8=%08x  R25/s9=%08x  R26/s10=%08x R27/s11=%08x  R28/t3=%08x  R29/t4=%08x "
            " R30/t5=%08x  R31/t6=%08x",
            strrchr (name (), '.') + 1, gp_regs[24].read (), gp_regs[25].read (),
            gp_regs[26].read (), gp_regs[27].read (), gp_regs[28].read (), gp_regs[29].read (),
            gp_regs[30].read (), gp_regs[31].read ()));
}

void MExu::DumpRegisterChange () {
    if (cfg_insn_trace) {
        INFOF (("   (%s) R%-2d = %08x -> %08x    %d -> %d", strrchr (name (), '.') + 1, gpr_sel_dreg.read().value() ,
                gp_regs[gpr_sel_dreg.read()].read(),  gpr_in.read(), gp_regs[gpr_sel_dreg.read()].read(), gpr_in.read()));
    }
}

// Helper function to display the instruction trace during simulation
#ifndef __SYNTHESIS
void MExu::InstructionTrace () {
    if (cfg_insn_trace) {
        DumpRegisterInfo ();
        INFOF (("   (%s) %s", strrchr (name (), '.') + 1, mainMemory->GetDumpStr (pc.read ())));
    }
}
#else
void MExu::IntructionTrace () {
    // Nothing
}
#endif


// ************ Performance Counters ************

typedef enum {
    perfInstret,
    perfALU,
    perfLoad,
    perfStore,
    perfJump,
    perfSystem,
    perfLoadStoreWait
} EPerfCount;

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
#define CYCLEREG_LOW (csr_mcycle.read () (31, 0))
#define CYCLEREG_HIGH (csr_mcycle.read () (63, 32))
#define PERFREGS(COUNTER) csr_mhpmcounter[COUNTER].read ()

void MExu::PerfcountMethod () {
#pragma HLS ARRAY_PARTITION variable = csr_mhpmcounter complete dim = 1
    if (inCePU) {
        __uint64_t mcycle_val = csr_mcycle.read ();
        sc_uint<CFG_EXU_PERFCOUNTERS_LD> addr = perf_addr.read ();
        sc_uint<CFG_EXU_PERFCOUNTER_BITS> value = csr_mhpmcounter[addr].read ();
        sc_uint<CFG_EXU_PERFCOUNTER_BITS> instret = csr_mhpmcounter[perfInstret].read ();

        if (reset) {
            csr_mcycle = 0;
            for (int i = 0; i < CFG_EXU_PERFCOUNTERS; i++) csr_mhpmcounter[i] = 0;
        } else {
            // mcycle/h:
            csr_mcycle = ++mcycle_val;

            // others:
            if (perf_inc) {
                value++;
                instret++; // if any other instruction is incremented also increment instret
//               INFOF(("PerfCount: %u = %lu, %lu", addr.value(), csr_mhpmcounter[addr].read().value(), value.value()));
//               INFOF(("PerfCount: Cycles = %lu", value_cycle));
            }
            csr_mhpmcounter[addr] = value;
            csr_mhpmcounter[perfInstret] = value;
        }
    }
}
#else // CFG_EXU_PERFCOUNT_ENABLE != 1
#define PERFSTD()   \
    { /* nothing */ \
    }
#define PERFINC(COUNTER) \
    { /* nothing */      \
    }
#define PERFREGS(COUNTER) sc_uint<CFG_EXU_PERFCOUNTER_BITS> (0x0)
#define CYCLEREG_LOW 0x0
#define CYCLEREG_HIGH 0x0
#endif

// ************ Control and Special Registers ************
void MExu::CSRHandleMethod() {
    sc_uint<5> rs1;
    sc_uint<3> function;
    sc_uint<32> rs1_val, old_val, new_val;
    bool wr, rs1_nzero;

    // Read input signals
    old_val = csr_rdata.read ();
    rs1 = csr_rs1_reg.read ();
    function = csr_function_reg.read ();

    // Set default values
    rs1_val = function[2] ? (sc_uint<32>)(sc_uint<28> (0), rs1) :
                            csr_op_a.read (); // either Immediate or Register input
    new_val = rs1_val;
    wr = 0;
    rs1_nzero = rs1.or_reduce ();

    if (csr_enable) {
        // Handle operation
        switch (function (1, 0)) {
        case CSRRW:
            // CSRRW/I - WRITE
            wr = 1; // writes even on exception
            break;
        case CSRRS:
            // CSRRS/I - SET
            // Apply bit-mask and set write
            new_val = old_val | rs1_val;
            wr = rs1_nzero; // write only if rs1 is != x0
            break;
        case CSRRC:
            // CSRRC/I - CLEAR
            // Apply clear bit-mask and set write
            new_val = old_val & (~rs1_val);
            wr = rs1_nzero; // write only if rs1 is != x0
            break;
        default:
//            WARNINGF(("   (%s)  CSRHandleMethod: Invalid subcode in instruction: 0x%x", strrchr (name (), '.') + 1, function.value()));
            break;
        }
    }

    // Set output signals
    csr_wdata = new_val;
    csr_write = wr;

    // Exception (Currently only writes to read-only CSRs raise an exeption
    csr_exception = wr ? csr_adr_reg.read ()[10] & csr_adr_reg.read ()[11] : 0;
//    csr_exception = ex | csrWriteException;
}

void MExu::CSRReadMethod (){
    ECSR csradr;
    sc_uint<32> rdata;
    bool read_exception;

    // Read input signals
    csradr = (ECSR) ((__uint16_t)csr_adr_reg.read ());
    read_exception = 0;
    rdata = 0;

    if (inCePU) {
        // Read CSR
        if (csr_enable) {
            switch (csradr) {
            case misa: // Machine ISA Register
                // MXL[31:30]: 1 - XLEN 32 is 32 Bit
                rdata = 0x40800100; // RV32I + non-standard extensions present (= Paranut)
#if CFG_EXU_M_EXTENSION == 1
                // Extensions[25:0]: 0x1000 - M-Extension supported
                rdata |= 0x1000;
#endif
#if CFG_EXU_A_EXTENSION == 1
                // Extensions[25:0]: 0x1 - A-Extension supported
                rdata |= 0x1;
#endif
                break;
            case mvendorid: // Machine Vendor ID Register (RO)
                // 0 - No JEDEC Vendor ID, non-commercial implementation
                rdata = 0x0;
                break;
            case marchid: // Machine Architecture ID Register (RO)
                // 0 - Architecture ID not assigned from RISC-V
                rdata = 0x0;
                break;
            case mimpid: // Machine Implementation ID Register (RO)
                // 0 - Implementation ID not implemented
                rdata = 0x0;
                break;
            case mhartid: // Machine Hart ID Register (RO)
                // Hart ID is set on creation (see constructor of MExu)
                rdata = hartID.read ();
                break;
            case mstatus: // Machine Status Register
                rdata = (sc_uint<1> (0),    // SD: hardwired 0
                         sc_uint<8> (0),    // RESERVED
                         sc_uint<5> (0),    // TSR, TW, TVM, MXR, SUM: No S-mode supported - hardwired 0
                         sc_uint<1> (0),    // MPRV: No U-mode supported - hardwired 0
                         sc_uint<2> (0),    // XS: No user-mode extensions - hardwired 0
                         sc_uint<2> (0),    // FS: No floating point unit - hardwired 0
                         sc_uint<2> (0),    // MPP: No S- or U-mode - MPP only needs 00
                         sc_uint<2> (0),    // RESERVED
                         sc_uint<1> (0),    // SPP: No S-mode - hardwired 0
                         csr_mstatus_MPIE,  // MPIE
                         sc_uint<1> (0),    // RESERVED
                         sc_uint<1> (0),    // SPIE: No S-mode - hardwired 0
                         sc_uint<1> (0),    // UPIE: No U-mode - hardwired 0
                         csr_mstatus_MIE,   // MIE
                         sc_uint<1> (0),    // RESERVED
                         sc_uint<1> (0),    // SIE: No S-mode - hardwired 0
                         sc_uint<1> (0)     // UIE: No U-mode - hardwired 0
                        );
                break;
            case mtvec: // Machine Trap-Vector Base-Address Register
                rdata = csr_mtvec.read ();
                break;
            case mip: // Machine Interrupt-Pending Register
                // 0 - Not implemented
                // todo: Add bits for External and Timer Interrupt
                rdata = 0x0;
                break;
            case mie: // Machine Interrupt-Enable Register
                // 0 - Not implemented
                // todo: Add bits for External and Timer Interrupt
                rdata = 0x0;
                break;
            case mtime: // Machine Timer Register 31-0
                // 0 - Not implemented
                rdata = 0x0;
                break;
            case mtimeh: // Machine Timer Register 63-32
                // 0 - Not implemented
                rdata = 0x0;
                break;
            case mtimecmp: // Machine Timer Compare Register 31-0
                // 0 - Not implemented
                rdata = 0x0;
                break;
            case mtimecmph: // Machine Timer Compare Register 63-32
                // 0 - Not implemented
                rdata = 0x0;
                break;
            case mcycle: // Machine Cycle Register 31-0
                rdata = CYCLEREG_LOW;
                break;
            case mcycleh: // Machine Cycle Register 63-32
                rdata = CYCLEREG_HIGH;
                break;
            case minstret: // Machine Instruction Retired Register 31-0
                rdata = PERFREGS (perfInstret) (31, 0);
                break;
            case minstreth: // Machine Instruction Retired Register 63-32
                rdata = sc_uint<32> (PERFREGS (perfInstret) (CFG_EXU_PERFCOUNTER_BITS - 1, 32));
                break;
            case mhpmcounter3: // perfALU
                rdata = PERFREGS (perfALU) (31, 0);
                break;
            case mhpmcounter4: // perfLoad
                rdata = PERFREGS (perfLoad) (31, 0);
                break;
            case mhpmcounter5: // perfStore
                rdata = PERFREGS (perfStore) (31, 0);
                break;
            case mhpmcounter6: // perfJump
                rdata = PERFREGS (perfJump) (31, 0);
                break;
            case mhpmcounter7: // perfSystem
                rdata = PERFREGS (perfSystem) (31, 0);
                break;
            case mhpmcounter8: // perfLoadStoreWait
                rdata = PERFREGS (perfLoadStoreWait) (31, 0);
                break;
            case mhpmcounter3h: // perfALU
                rdata = sc_uint<32> (PERFREGS (perfALU) (CFG_EXU_PERFCOUNTER_BITS - 1, 32));
            case mhpmcounter4h: // perfLoad
                rdata = sc_uint<32> (PERFREGS (perfLoad) (CFG_EXU_PERFCOUNTER_BITS - 1, 32));
            case mhpmcounter5h: // perfStore
                rdata = sc_uint<32> (PERFREGS (perfStore) (CFG_EXU_PERFCOUNTER_BITS - 1, 32));
            case mhpmcounter6h: // perfJump
                rdata = sc_uint<32> (PERFREGS (perfJump) (CFG_EXU_PERFCOUNTER_BITS - 1, 32));
            case mhpmcounter7h: // perfSystem
                rdata = sc_uint<32> (PERFREGS (perfSystem) (CFG_EXU_PERFCOUNTER_BITS - 1, 32));
            case mhpmcounter8h: // perfLoadStoreWait
                rdata = sc_uint<32> (PERFREGS (perfLoadStoreWait) (CFG_EXU_PERFCOUNTER_BITS - 1, 32));
                break;
            case mscratch: // Machine Scratch Register
                rdata = csr_mscratch.read ();
                break;
            case mepc: // Machine Exception Program Counter Register
                rdata = csr_mepc.read ();
                break;
            case mcause: // Machine Cause Register
                rdata = csr_mcause.read ();
                break;
            case mtval: // Machine Trap Value Register
                rdata = csr_mtval.read ();
                break;
            case dcsr: // Debug Control and Status
                rdata = (sc_uint<4> (4),            // External debug support
                         sc_uint<12> (0),           // See specification for detailed information
                         csr_dcsr_ebreakm.read (),  // Behaviour of ebreak instruction 0 - Exception 1 - Jump to Debug
                         sc_uint<6> (0),
                         csr_dcsr_cause.read (),    // Cause of entering debug mode
                         sc_uint<3> (0),            // See specification for detailed information
                         csr_dcsr_step.read (),     // Enter debug mode after single instruction
                         sc_uint<2> (3)             // prv: Fixed to 3 - Machine
                );
                break;
            case dpc: // Debug Program Counter
                rdata = csr_dpc.read ();
                break;
            case dscratch0: // Debug Scratch Register 0
                rdata = csr_dscratch0.read ();
                break;
            case tselect: // Trace Select Register
                rdata = 0xFFFFFFFF; // Fixed to -1, else GDB assumes there is a Trace register
                break;
            case pncpus: // ParaNut Number of CPUs Register
                // Fixed configured value
                rdata = CFG_NUT_CPU_CORES;
                break;
            case pnm2cp: // ParaNut CPU Capabilities Register
                // Fixed configured value
                rdata = CFG_EXU_PNM2CAP;
                break;
            case pnce: // ParaNut CPU Enable Register
                // CePU:
                rdata = csr_pnce[csr_pngrpsel].read ();
                break;
            case pnlm: // ParaNut CPU Linked Mode Register
                // CePU:
                rdata = csr_pnlm[csr_pngrpsel].read ();
                break;
            case pncache: // Cache Control Register
                // CePU:
                rdata = (sc_uint<30> (0),       // RESERVED
                         csr_pncache.read ());  // DCACHE - 1 enabled, 0 disabled
                                                // ICACHE - 1 enabled, 0 disabled
                break;
            case pnx: // ParaNut CoPU Exception Register
                rdata = csr_pnx[csr_pngrpsel].read ();
                break;
            case pnxsel: // ParaNut CoPU Exception Select Register
                rdata = csr_pnxsel[csr_pngrpsel].read ();
                break;
            case pncause: // ParaNut CoPU Cause ID
                rdata = cause_i.read ();
                break;
            case pnepc: // ParaNut CoPU Exception Program Counter
                rdata = epc_i.read ();
                break;
            case pngrpsel: // ParaNut CoPU CPU Group Select Register
#if (CFG_NUT_CPU_CORES - 1) / 32 // Special: More than 32 CPUs
                rdata = csr_pngrpsel.read ();
#else
                rdata = csr_pngrpsel;
#endif
                break;
            case pncacheinfo: // ParaNut Cache Information Register
                rdata = (sc_uint<24>(CFG_MEMU_CACHE_BANKS),       // Number of Cache Banks
                         sc_int<5>(CFG_MEMU_ARBITER_METHOD),      // Used Arbiter Method (signed value!)
                         sc_uint<2>(CFG_MEMU_CACHE_WAYS_LD),      // Cache Associativity (0 = 1-way, 1 = 2-way, or 2 = 4-way)
                         sc_uint<1>(CFG_MEMU_CACHE_REPLACE_LRU)   // Used Replacement Method (0 - random, 1 - LRU)
                         );
                break;
            case pncachesets: // ParaNut Number of Cache Sets Register
                rdata = CFG_MEMU_CACHE_SETS;
                break;
            case pnclockinfo:
                rdata = clock_freq_hz.read();
                break;
            case pnmemsize:
                rdata = CFG_NUT_MEM_SIZE;
                break;
            default:
                WARNINGF (("   (%s)  GetCSR: Read access to unknown CSR 0x%04x, Instruction: "
                           "(0x%08x) = 0x%08x",
                           strrchr (name (), '.') + 1, csradr, pc.read (), ir.read ()));
                //                 readException = 1;
                rdata = 0x0;
            }
        }
    } else { // if(!inCePU)
        // Read CSR
        if (csr_enable) {
            switch (csradr) {
            case misa: // Machine ISA Register
                       // MXL[31:30]: 1 - XLEN 32 is 32 Bit
#if CFG_EXU_M_EXTENSION == 1
                       // Extensions[25:0]: 0x1100 - RV32I and M-Extension supported
                rdata = 0x40001100;
#else
                       // Extensions[25:0]: 0x100 - Only RV32I supported
                rdata = 0x40000100;
#endif
                break;
            case mhartid: // Machine Hart ID Register (RO)find_
                // Hart ID is set on creation (see constructor of MExu)
                rdata = hartID.read ();
                break;
            case pncpus: // Machine ParaNut Number of CPUs Register
                // Fixed configured value
                rdata = CFG_NUT_CPU_CORES;
                break;
            case pnce: // Machine ParaNut CPU Enable Register
                // CoPU:
                rdata = enable.read ();
                break;
            case pnlm: // Machine ParaNut CPU Linked Mode Register
                // CoPU:
                rdata = linked.read ();
                break;
            default:
                WARNINGF (("   (%s)  GetCSR: Read access to unknown CSR 0x%04x, Instruction: "
                           "(0x%08x) = 0x%08x",
                           strrchr (name (), '.') + 1, csradr, pc.read (), ir.read ()));
                rdata = 0;
            }
        }
    }


    // Output signals
    csr_rd_exception = read_exception;
    csr_rdata = rdata;
}

void MExu::CSRWriteMethod (){
    ECSR csradr;
    sc_uint<32> wdata;

	// Read input signals
    csradr = (ECSR) ((__uint16_t)csr_adr_reg.read());
    wdata = csr_wdata.read();

	if(reset){ // Dominating reset
        if(inCePU){
            // Reset CSRs
            csr_mstatus_MIE = 0;
            csr_mstatus_MPIE = 0;
            csr_mip_MEIP = 0;
            csr_mie_MEIE = 0;
            csr_mepc = 0;
            csr_mcause = 0;
            csr_mtval = 0;
            csr_mtvec = CFG_NUT_SIM_MEM_ADDR;
            csr_mscratch = 0;
            csr_dcsr_cause = 0;
            csr_dcsr_step = 0;
            csr_dcsr_ebreakm = 0;
            csr_dpc = 0;
            csr_dscratch0 = 0;

            // Reset ParaNut CSRs
            csr_pncache = 0;

            // Grouped ParaNut CSRs
            for(int i = 0; i < CFG_NUT_CPU_GROUPS+1; i++){
                # pragma HLS UNROLL
                csr_pnce[i] = (CFG_EXU_PNM2CAP & (0xffffffff << 32*i)) >> 32*i; // activate all mode 2 exus;
                csr_pnlm[i] = 0;
                csr_pnxsel[i] = 0;
                csr_pnx[i] = 0;
            }
            //~ csr_pnce[0] = 1;
        }
	} else {
		// Set CSR
        if(inCePU){
            if(csr_write && !csr_exception){
                switch (csradr) {
                case misa: // Machine ISA Register
                    INFOF (("   (%s)  SetCSR: Writing 0x%x to CSR 0x%x ignored because of implementation (WARL)", strrchr (name (), '.') + 1, wdata.value(), csradr));
                    break;
                case mstatus: // Machine Status Register
                    csr_mstatus_MIE = wdata[3];
                    csr_mstatus_MPIE = wdata[7];
                    break;
                case mtvec: // Machine Trap-Vector Base-Address Register
                    csr_mtvec = wdata;
                    break;
                case mip: // Machine Interrupt Pending Register
                    INFOF (("   (%s)  SetCSR: Writing 0x%x to CSR 0x%x ignored because of implementation (WARL)", strrchr (name (), '.') + 1, wdata.value(), csradr));
                    break;
                case mie: // Machine Interrupt Enable Register
                    INFOF (("   (%s)  SetCSR: Writing 0x%x to CSR 0x%x ignored because of implementation (WARL)", strrchr (name (), '.') + 1, wdata.value(), csradr));
                    break;
                case mscratch:  // Machine Scratch Register
                    csr_mscratch = wdata;
                    break;
                case mepc: // Machine Exception Program Counter Register
                    // mepc[1:0] needs to be always zero, so it can never trigger an instruction-address-misaligned exception
                    csr_mepc = (wdata(31,2), sc_uint<2>(0) ); // todo: For 16 bit ISA support only mepc[0] needs to be 0
                    break;
                case mcause: // Machine Cause Register
                    csr_mcause = wdata; // 	todo: It should be only possible to write legal values (WLRL) (see privileged ISA)
                    break;
                case mtval: // Machine Trap Value Register
                    csr_mtval = wdata;
                    break;
                case mtimecmp: // Machine Timer Compare Register 31-0
                    INFOF (("   (%s)  SetCSR: Writing 0x%x to CSR 0x%x ignored because it is not yet implemented", strrchr (name (), '.') + 1, wdata.value(), csradr));
                    break;
                case mtimecmph: // Machine Timer Compare Register 63-32
                    INFOF (("   (%s)  SetCSR: Writing 0x%x to CSR 0x%x ignored because it is not yet implemented", strrchr (name (), '.') + 1, wdata.value(), csradr));
                    break;
                case dcsr: // Debug Control and Status
                    // csr_dcsr_cause = Read only
                    csr_dcsr_step = wdata[2];
                    csr_dcsr_ebreakm = wdata[15];
                    break;
                case dpc: // Debug Program Counter
                    csr_dpc = wdata;
                    break;
                case dscratch0: // Debug Scratch Register 0
                    csr_dscratch0 = wdata;
                    break;
                case pnce: // Machine ParaNut CPU Enable Register
                    csr_pnce[csr_pngrpsel] = wdata;//(CFG_NUT_CPU_CORES-1, 0);
                    break;
                case pncache:// ParaNut Cache Control Register
                    csr_pncache = wdata(1,0);
                    break;
                case pnlm: // Machine ParaNut CPU Linked Mode Register
                    csr_pnlm[csr_pngrpsel] = wdata(CFG_NUT_CPU_CORES-1, 0);
                    break;
                case pnx: // Machine ParaNut CoPU Exception Register
                    csr_pnx[csr_pngrpsel] = wdata(CFG_NUT_CPU_CORES-1, 0);
                    break;
                case pnxsel: // Machine ParaNut CoPU Exception Select Register
                    csr_pnxsel[csr_pngrpsel] = wdata(MIN(XLEN-1,CFG_NUT_CPU_CORES), 0);
                    break;
                case pngrpsel: // Machine ParaNut CPU Group Select Register
#if (CFG_NUT_CPU_CORES-1)/32  // Special: More than 32 CPUs
                    csrMpngrpsel = newCSRValue;
#else
                    INFOF (("   (%s)  SetCSR: Writing 0x%x to CSR 0x%x (mpngrpsel) ignored because of implementation", strrchr (name (), '.') + 1, wdata.value(), csradr));
#endif
                    break;
                default:
                    // Special case: Machine Performance Monitoring Event Registers
                    if(csradr >= mhpmevent3 && csradr <= mhpmevent31){
                        INFOF (("   (%s)  SetCSR: Writing 0x%x to CSR 0x%x ignored because of implementation (WARL)", strrchr (name (), '.') + 1, wdata.value(), csradr));
                    }else{
                        WARNINGF(("   (%s)  SetCSR: CSR wfrite to unknown register 0x%04x, Instruction: (0x%08x) = 0x%08x", strrchr (name (), '.') + 1, csradr, pc.read(), ir.read()));
                    }
                    break;
                }
            } else {
                // When no CSR write operation is requested handle halt requests from CoPUs
                for(int i = 0; i < CFG_NUT_CPU_GROUPS+1; i++){
                    # pragma HLS UNROLL
                    csr_pnce[i] = csr_pnce[i].read() & ~m3_pnhaltreq.read()(MIN((XLEN*(i+1))-1, CFG_NUT_CPU_CORES-1),32*i);
                }
                // and/or handle writing of exception values
                if(exception){
                    csr_mcause =  (irq_dreg.read(), sc_uint<26>(0), ex_id_reg.read());
                    csr_mstatus_MPIE = csr_mstatus_MIE.read();
                    csr_mstatus_MIE = 0;
                    csr_mepc = pc.read();
                    csr_mtval = csr_mtval_in.read();
                    for(int i = 0; i < CFG_NUT_CPU_GROUPS+1; i++){
                        # pragma HLS UNROLL
                        csr_pnx[i] = m3_pnx.read()(MIN((XLEN*(i+1))-1, CFG_NUT_CPU_CORES-1),32*i);
                    }
                }

                // Set debug program counter and debug cause on debug entry
                if(dbg & !dbg_reg){
                    csr_dpc = pc.read();
                    csr_dcsr_cause = csr_dcsr_step ? 4 : dbg_req ? 3 : 1; // Cause is either 4: Step or 3: Halt request or 1: ebreak
                }

                // Enable CePU when we get a debug request
                if(dbg_req){
                  csr_pnce[0] = csr_pnce[0].read() | 0x1;
                }

            }
        } else { // if(inCePU)
            if(csr_write && (csradr != pnce && linked)){  // Don't warn on write to pnce in linked mode (happens during shutdown of linked mode)
                WARNINGF(("   (%s)  SetCSR: CSR write in CoPU not permitted 0x%04x, Instruction: (0x%08x) = 0x%08x", strrchr (name (), '.') + 1, (__uint16_t)csr_adr_reg.read(), pc.read(), ir.read()));
            }

            // handle writing of exception values
            if(exception) {
                csr_mcause =  (sc_uint<26>(0), ex_id.read());
                csr_mepc = pc.read();
            }

            // Set debug program counter and debug cause on debug entry
            if(dbg & !dbg_reg){
                csr_dpc = pc.read();
                csr_dcsr_cause = csr_dcsr_step ? 4 : dbg_req ? 3 : 1; // Cause is either 4: Step or 3: Halt request or 1: ebreak
            }
        }
	}
}


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
                        ASSERTF (false, ("Impossible/implausible shift select signal: %d (Must be
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
#pragma HLS ARRAY_PARTITION variable=csr_pnce complete dim=1
#pragma HLS ARRAY_PARTITION variable=csr_pnlm complete dim=1
#pragma HLS ARRAY_PARTITION variable=csr_pnx complete dim=1
#pragma HLS ARRAY_PARTITION variable=csr_pnxsel complete dim=1

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
    EOpRISCV opcode;
    bool insn_ebreak, insn_ecall;

    // Read input signals
    insn = ir.read ();
    opcode = (EOpRISCV) (__uint8_t)insn (6, 0);
    gpr_b_val = gp_regs[insn (24, 20)].read ();
    insn_ebreak = insn_ecall = 0;

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
        lres_scond_dreg = 0;

        // Special control signals
        illegal_insn_dreg = 0;
        ex_CoPU_dreg = 0;
        irq_dreg = 0;
        ecall_dreg = 0;
        ebreak_dreg = 0;
        flush_dreg = 0;
        cinvalidate_dreg = 0;
        cwriteback_dreg = 0;

        if (ir_valid && enable && !sync && !sync_reg && !internal_next && !sync_next) {
            // Register for single stepping
            step_dreg = csr_dcsr_step & !dbg_reg;

            // Instruction trace for simulation:
            InstructionTrace ();

            //            ASSERTF(insn.value() == mainMemory->ReadWord((TWord) pc.read()),("Instruction
            //            does not match the memory at PC: Mem: (0x%08x) 0x%08x != Insn: 0x%08x", (TWord) pc.read(), (TWord)mainMemory->ReadWord((TWord) pc.read()), insn.value()));
            switch (opcode) {
            case OP:
                if (insn[25]) { // Bit 25 is set in M-Extension instructions
#if CFG_EXU_M_EXTENSION == 1
                    // (ALU, MUL/DIV) M-Extension  ...
                    alu_md_dreg = insn[14];
                    exu_op_dreg = OpMulDiv;
#else
                    WARNINGF (("   (%s)  Illegal Instruction (M-Extension not activated): 0x%08x",
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
                break;
            case LUI:
            case AUIPC:
                // (ALU) ALU add 20 Bit upper immediate to register or PC...
                alu_type = UType;
                exu_op_dreg = OpAlu;
                break;
            case LOAD:
                // (LS) Load...
                alu_type = IType;
                lsu_rd_dreg = 1;
                exu_op_dreg = OpMem;
                break;
            case STORE:
                // (LS) Store...
                alu_type = SType;
                exu_op_dreg = OpMem;
                break;
            case JAL:
            case JALR:
                // (JAL) Jump and Link...
                alu_type = insn[3] ? JType : IType;
                exu_op_dreg = OpJump;
                break;
            case BRANCH:
                // (BRANCH) Conditional Branch...
                // bit 14:    BLT/BGE/BLTU/BGEU or BEQ/BNEQ
                // -> bit 13: BLTU/BGEU or BLT/BGE;
                alu_function_dreg = insn[14] ? (insn[13] ? afBltu : afBlt) : afEq;

                alu_af_dreg = (insn[12]); // AF needed for BGE/BGEU and BEQ;
                alu_branch_dreg = 1;
                exu_op_dreg = OpBranch;
                break;
            case SYSTEM:
                // (OTHER) SYS / CSR / TRAP ...
                if (insn (14, 12) == 0) {
                    // MRET, ECALL, EBREAK instruction
                    insn_ebreak = ~insn[21] & insn[20] &    // EBREAK instruction
                                  (!linked | inCePU);       // only CePU executes ebreak in linked mode
                    insn_ecall = ~insn[21] & ~insn[20];
                    ecall_dreg = insn_ecall;
                    ebreak_dreg = insn_ebreak;
                    exu_op_dreg = insn[21] ? OpXRet : insn_ebreak | insn_ecall ? OpException : OpAlu;
                } else {
                    // CSR instruction
                    exu_op_dreg = OpSystem;
                }
                break;
            case MISC_MEM:
                // (OTHER) FENCE / FENCE.I instruction ...
                alu_type = JType;
                flush_dreg = 1;
                exu_op_dreg = OpParaNut;
                break;
            case PARA:
                // (LS, ParaNut extension) ...
                // PN_CFLUSH, PN_CINVALIDATE, PN_CWRITEBACK
                // todo: Illegal Instructions (insn (14,12) > 4) will also be executed
                flush_dreg = (insn (14, 12) != 0);
                cinvalidate_dreg = insn[12];
                cwriteback_dreg = insn[13];
                exu_op_dreg = OpParaNut;
                alu_type = IType;
                break;
#if CFG_EXU_A_EXTENSION == 1
            case AMO:
                // (LR/SC, AMO) A-Extension ...
                exu_op_dreg = OpMem;
                lsu_rd_dreg = !insn[27];
                flush_dreg = 1;
                lres_scond_dreg = 1;
                break;
#endif
            default:
                // Illegal instruction:
                WARNINGF (("   (%s)  Illegal Instruction: (0x%08x): 0x%08x",
                           strrchr (name (), '.') + 1, pc.read (), ir.read ()));
                illegal_insn_dreg = 1;
                exu_op_dreg = OpException;
                break;
            } // switch (opcode)
        } // if(ir_valid && enable)

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
            ex_CoPU_dreg = ex_i.read (); // CoPU excepitons have priority over interrupts
            irq_dreg = m3_ir_request.read () & !ex_i.read ();
            if (ex_i | m3_ir_request) {
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
            op_b = gpr_b_val;
            break;
        }
        op_a_dreg = op_a;
        op_b_dreg = op_b;
        // B-type encoding for branch operands A and B
        branch_a_dreg = pc.read ();
        branch_b_dreg = (sc_uint<32>)((sc_int<32>)sc_int<13> (
        (insn[31], insn[7], insn (30, 25), insn (11, 8), sc_uint<1> (0))));

        // MLsu
        lsu_width = (insn[12], sc_uint<1> (!(insn[12] ^ insn[13])));
        lsu_exts = ~insn[14];
        lsu_wdata = gpr_b_val;

    } // if(!stall_decode)


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
        cinvalidate_dreg = 0;
        cwriteback_dreg = 0;
    }
}

void MExu::MainCombMethod () {
    EExceptions ex_id_var;
    bool ex, ex_o_var, ifu_next_var, ifu_jump_var, jmp_adr_mis;
    EExuState state_next;
    sc_uint<32> jmp_adr, alu_result_var;
    EExuOp opcode;


    // Read input signals ...
    opcode = (EExuOp) (__uint16_t)exu_op_dreg.read ();
    state_next = (EExuState) (__uint16_t)state_reg.read ();
    alu_result_var = alu_result.read ();


    // Preset control signals ...
    jmp_adr (31, 1) = (alu_result_reg.read () (31, 1));
    jmp_adr[0] = alu_result_reg.read ()[0] &
                 exu_op_dreg.read ()[2]; // remove last bit (todo: specification only requires this for JALR)
    jmp_adr_mis = jmp_adr (1, 0) & 3; // jump address missaligned signal

    ex_id_var = InststructionAddressMisaligned;
    ex = 0;

    // MIfu
    ifu_next_var = 0;
    ifu_jump_var = 0;
    ifu_flush = 0;
    ifu_jump_adr = jmp_adr;
    internal_next = 0;
    // MLsu
    lsu_adr = lres_scond_dreg ? op_a_dreg.read () : alu_result_reg.read ();
    lsu_lres_scond = 0;
    lsu_cache_invalidate = 0;
    lsu_cache_writeback = 0;
    lsu_flush = 0;
    lsu_rd = 0;
    lsu_wr = 0;
    // CSR
    csr_enable = 0;
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
    m3_ir_ack = 0;
    exception = 0;

    // Exception daisy chain:
    cause_o = xsel ? csr_mcause.read () (4, 0) : cause_i.read ();
    epc_o = xsel ? csr_mepc.read () : epc_i.read ();

    // Differen ParaNut control signals for CePU and CoPUs:
    if (inCePU) {
        // Set and reset ex_o as soon as possible
        ex_o_var = ((opcode == OpException) | ex_handle_reg) &  // Set on exception entry and hold
                   (opcode != OpXRet);                        // Reset on xret operation
        csr_mtval_in = jmp_adr; // Exception information
        m3_ir_enable = csr_mstatus_MIE.read ();
    } else {
        ex_o_var = 0;
        csr_mtval_in = 0;
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
//        ASSERTF (insn.value () == mainMemory->ReadWord ((TWord)pc.read ()),
//                 ("Instruction does not match the memory at PC: Mem: (0x%08x) 0x%08x != Insn: "
//                  "0x%08x",
//                  (TWord)pc.read (), (TWord)mainMemory->ReadWord ((TWord)pc.read ()), insn.value ()));
        switch (opcode) {
        case OpAlu: // todo: MEALY
            // (ALU) ALU operation (Completes in 1 Cycle)
            gpr_write = 1;
            ifu_next_var = 1;
            perf_mon_.Count (EV_ALU);
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
        lsu_rd = lsu_rd_dreg.read ();
        lsu_wr = !lsu_rd_dreg.read ();
        lsu_lres_scond = lres_scond_dreg.read ();

        ex_id_var = lsu_rd_dreg ? LoadAddressMisaligned : StoreAddressMisaligned;
        ex = lsu_align_err.read ();

        if (lsu_ack) { // Wait for acknowledge
            state_next = ExuMemWB;
            // One could already assert 'ifu_next_var' here but doing so yields
            // a long critical path from MEMU -> LSU -> EXU -> IFU
            // so no mealy here based on 'lsu_ack'!
        }
        // Write GPR on scond (needs to happen before we go to ExuMemWB state)
        gpr_write = !lsu_rd_dreg.read () & lres_scond_dreg.read ();

        PERFINC (perfLoadStoreWait);
//        if (lsu_align_err) // alignment error
//            WARNINGF (("Load address misaligned: 0x%08x, Instruction: (0x%08x) = 0x%08x",
//                       (__uint32_t)alu_result.read (), pc.read (), (__uint32_t)ir.read ()));
        break;
    case ExuMemWB: // (LS) Load Write Back (we are delayed one clock) ...
        // Write GPR on load/lres
        gpr_write = lsu_rd_dreg.read ();

        if (lsu_rd_dreg) {
            perf_mon_.Count (EV_LOAD);
            PERFINC (perfLoad);
        } else {
            perf_mon_.Count (EV_STORE);
            PERFINC (perfStore);
        }
        ifu_next_var = 1;
        state_next = ExuExecuteInsn;
        break;
    // JUMP/BRANCH execution states:
    // -----------------------------------------
    case ExuJump: // (JAL) Jump and Link...
        // alignment check
        ex_id_var = InststructionAddressMisaligned;
        ex = jmp_adr_mis;
//        if (jmp_adr_mis)
//            WARNINGF (("   (%s)  Jump: Instruction address misaligned: 0x%08x, Instruction: "
//                       "(0x%08x) = 0x%08x",
//                       strrchr (name (), '.') + 1, (__uint32_t)alu_result.read (),
//                       (__uint32_t)pc.read (), (__uint32_t)ir.read ()));
//         else
//             INFOF (("Instruction: (0x%08x) = 0x%08x Jumping to 0x%x  ", (__uint32_t)pc.read (),
//                     insn.value (), adr));
        // Save return address (npc)
        gpr_write = !jmp_adr_mis;

        // jump
        ifu_jump_var = !jmp_adr_mis;
        ifu_next_var = !jmp_adr_mis;
        state_next = ExuExecuteInsn;
        perf_mon_.Count (EV_JUMP);
        PERFINC (perfJump);
        //			}
        break;
    case ExuBranch: // (BRANCH) Conditional Branch Decision ...
        // alignment check
        ex_id_var = InststructionAddressMisaligned;
        ex = jmp_adr_mis & alu_branch_reg;
//        if(jmp_adr_mis) // todo: needs to change for 16 bit support
//          WARNINGF (("Branch: Instruction address misaligned: 0x%08x, Instruction: "
//                     "(0x%08x) = 0x%08x",
//                     alu_result.read (), pc.read (), insn.value ()));
//        else
//            INFOF (("Instruction: (0x%08x) = 0x%08x Branching to 0x%x  ", (__uint32_t)pc.read (),
//                    insn.value (), adr));

        ifu_jump_var = !jmp_adr_mis & alu_branch_reg;
        ifu_next_var = !(jmp_adr_mis & alu_branch_reg);
        perf_mon_.Count (EV_JUMP);
        PERFINC (perfJump);
        state_next = ExuExecuteInsn;
        break;
    // SYSTEM execution states:
    // -----------------------------------------
    case ExuCSR: // (OTHER) CSR
        csr_enable = 1;

        // Write GPR
        gpr_write = 1;

        // Only continue if csr_exception wasn't set
        ex = csr_exception.read ();
        ifu_next_var = !csr_exception;

        perf_mon_.Count (EV_OTHER);
        state_next = ExuExecuteInsn;
        PERFINC (perfSystem);
        break;
    case ExuXRET: // (OTHER) xRET to pc saved in mepc or dpc
        ex_handle = 0;
        dbg = dbg_reg.read ();
        ifu_jump_adr = dbg_reg ? csr_dpc.read () : csr_mepc.read ();

        ifu_jump_var = 1;
        ifu_next_var = 1;
        state_next = ExuExecuteInsn;

        perf_mon_.Count (EV_OTHER);
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

        perf_mon_.Count (EV_OTHER);
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
        lsu_cache_invalidate = cinvalidate_dreg.read ();
        lsu_cache_writeback = cwriteback_dreg.read ();

        //  Wait for LSU ack or return if neither wb. or inv. are set
        if (lsu_ack | !(cinvalidate_dreg | cwriteback_dreg)) {
            state_next = ExuMemWB;
            // One could already assert 'ifu_next_var' here but doing so yields
            // a long critical path from MEMU -> LSU -> EXU -> IFU
            // so no mealy here based on 'lsu_ack'!
            perf_mon_.Count (EV_OTHER);
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
        ifu_jump_adr = csr_mtvec.read ();
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
        perf_mon_.Count (EV_ALU);
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
        ERRORF (("EXU is in unknown state: %d Instruction: 0x%08x PC: 0x%08x",
                 state_reg.read ().value (), ir.read (), pc.read ()));
        break;
    } // switch(newExuState)

    // *********************** Exception Stage *************************
    // *****************************************************************

    // Handle Exception...
    if (ex) {
        state_next = ExuExOrIrq;
    } // if (ex)

    // exceptionId multiplexer
    if (illegal_insn_dreg | csr_exception)
        ex_id = IllegalInstruction;
    else if (ebreak_dreg)
        ex_id = Breakpoint;
    else if (ecall_dreg)
        ex_id = ECallM;
    else if (ex_CoPU_dreg)
        ex_id = CoPUException;
    else if (irq_dreg)
        ex_id = m3_ir_id.read ();
    else
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

        // Signals with csrMpngrpsel
        sc_uint<CFG_NUT_CPU_CORES> m3_pnxsel_var, m3_pnce_var, m3_pnlm_var;
        for (int i = 0; i < CFG_NUT_CPU_CORES; i++) {
#pragma HLS unroll
            m3_pnxsel_var[i] = csr_pnxsel[i / 32].read ().bit (i % 32);
            m3_pnlm_var[i] = csr_pnlm[i / 32].read ().bit (i % 32);
            m3_pnce_var[i] = csr_pnce[i / 32].read ().bit (i % 32);
        }
        m3_pnxsel = m3_pnxsel_var;
        m3_pnce = m3_pnce_var;
        m3_pnlm = m3_pnlm_var;

#ifndef __SYNTHESIS__
        m3_icache_enable->write (csr_pncache.read ()[0] & !cfg_disable_cache);
        m3_dcache_enable->write (csr_pncache.read ()[1] & !cfg_disable_cache);
#else
        m3_icache_enable->write (csr_pncache.read ()[0]);
        m3_dcache_enable->write (csr_pncache.read ()[1]);
#endif

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
