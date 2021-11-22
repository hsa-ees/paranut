/*************************************************************************

  This file is part of the ParaNut project.

  Copyright (C) 2010-2020 Alexander Bahle <alexander.bahle@hs-augsburg.de>
                          Christian H. Meyer <christian.meyer@hs-augsburg.de>
                          Gundolf Kiefer <gundolf.kiefer@hs-augsburg.de>
      Hochschule Augsburg, University of Applied Sciences

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.

 *************************************************************************/


#include "csr.h"


#ifndef __SYNTHESIS__
void MCsr::Trace (sc_trace_file *tf, int level) {
    if (!tf || pn_trace_verbose) printf ("\nSignals of Module \"%s\":\n", name ());

    PN_TRACE (tf, clk);
    PN_TRACE (tf, reset);

    PN_TRACE (tf, csr_mcycle);
    PN_TRACE_BUS (tf, csr_mhpmcounter, CFG_EXU_PERFCOUNTERS);

    PN_TRACE (tf, csr_wdata);
    PN_TRACE (tf, csr_rdata);
    PN_TRACE (tf, csr_exception);
    PN_TRACE (tf, csr_rd_exception);
    PN_TRACE (tf, csr_write);

    PN_TRACE (tf, priv_mode_reg);
    PN_TRACE (tf, delegate_dreg);
    PN_TRACE (tf, csr_dcsr_step);
    PN_TRACE (tf, csr_dcsr_ebreakm);
    PN_TRACE (tf, csr_dcsr_cause);
    PN_TRACE (tf, csr_mip_MEIP);
    PN_TRACE (tf, csr_mie_MEIE);

    PN_TRACE (tf, csr_mstatus_MPIE);
    PN_TRACE (tf, csr_mstatus_MIE);
    PN_TRACE (tf, csr_mstatus_SIE);
    PN_TRACE (tf, csr_mstatus_SPIE);
    PN_TRACE (tf, csr_mstatus_SPP);
    PN_TRACE (tf, csr_mstatus_SUM);
    PN_TRACE (tf, csr_mstatus_MXR);
    PN_TRACE (tf, csr_mstatus_TVM);
    PN_TRACE (tf, csr_mstatus_TW);
    PN_TRACE (tf, csr_mstatus_TSR);
    PN_TRACE (tf, csr_mstatus_SD);
    PN_TRACE (tf, csr_mstatus_MPP);
    PN_TRACE (tf, csr_mstatus_XS);

    PN_TRACE (tf, csr_medeleg);
    PN_TRACE (tf, csr_mideleg);
    PN_TRACE (tf, csr_stvec);
    PN_TRACE (tf, csr_scause);
    PN_TRACE (tf, csr_satp);
    PN_TRACE (tf, csr_stval);
    PN_TRACE (tf, csr_sepc);
    PN_TRACE (tf, csr_sscratch);
    PN_TRACE (tf, csr_mip);
    PN_TRACE (tf, csr_mie);
    PN_TRACE (tf, csr_mcause);
    PN_TRACE (tf, csr_dpc);
    PN_TRACE (tf, csr_dscratch0);
    PN_TRACE (tf, csr_mepc);
    PN_TRACE (tf, csr_mtval);
    PN_TRACE (tf, csr_mtvec);

    PN_TRACE_BUS (tf, csr_pnce, CFG_NUT_CPU_GROUPS);
    PN_TRACE_BUS (tf, csr_pnlm, CFG_NUT_CPU_GROUPS);
    PN_TRACE_BUS (tf, csr_pnx, CFG_NUT_CPU_GROUPS);
    PN_TRACE_BUS (tf, csr_pnxsel, CFG_NUT_CPU_GROUPS);
    
    PN_TRACE (tf, csr_pncache);
}
#endif // __SYNTHESIS__

#if CFG_EXU_PERFCOUNT_ENABLE == 1
void MCsr::PerfcountMethod () {
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
//               PN_INFOF(("PerfCount: %u = %lu, %lu", addr.value(), csr_mhpmcounter[addr].read().value(), value.value()));
//               PN_INFOF(("PerfCount: Cycles = %lu", value_cycle));
            }
            csr_mhpmcounter[addr] = value;
            csr_mhpmcounter[perfInstret] = value;
        }
    }
}
#else
void MCsr::PerfcountMethod () {}
#endif

void MCsr::CSRHandleMethod () {
    sc_uint<5> rs1;
    sc_uint<3> function;
    sc_uint<32> rs1_val, old_val, new_val;
    sc_uint<32> csr_adr = csr_adr_reg.read();
    bool wr, rs1_nzero;
    bool is_readonly_csr;
    bool is_unprivileged;

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
//            PN_WARNINGF(("   (%s)  CSRHandleMethod: Invalid subcode in instruction: 0x%x", strrchr (name (), '.') + 1, function.value()));
            break;
        }
    }

    if (inCePU) {
        // writes to read-only-CSRs raise an IllegalInstruction exception
        is_readonly_csr = csr_adr[10].to_bool () & csr_adr[11].to_bool ();
        is_unprivileged = csr_adr.range (9, 8) > priv_mode_reg.read ();
        csr_exception = ((wr & is_readonly_csr) | (csr_enable & is_unprivileged));
    } else {
        csr_exception = 0;
    }

    // Set output signals
    csr_wdata = new_val;
    csr_write = wr;
}

void MCsr::OutputMethod () {
    exu_csr_exception = csr_exception.read ();
    exu_csr_rd_exception = csr_rd_exception.read ();
    exu_csr_mstatus_MIE = csr_mstatus_MIE.read ();
    exu_csr_dcsr_step = csr_dcsr_step.read ();
    exu_csr_dcsr_ebreakm = csr_dcsr_ebreakm.read ();
    exu_csr_mcause = csr_mcause.read ();
    exu_csr_rdata = csr_rdata.read ();
    exu_csr_mepc = csr_mepc.read ();
    exu_csr_dpc = csr_dpc.read ();
    exu_csr_mtvec = csr_mtvec.read ();

    exu_priv_mode_reg = priv_mode_reg.read ();
    exu_csr_mstatus_SIE = csr_mstatus_SIE.read ();
    exu_csr_sepc = csr_sepc.read ();
    exu_csr_stvec = csr_stvec.read ();
    exu_csr_mideleg = csr_mideleg.read ();
    exu_delegate_dreg = delegate_dreg.read ();

    exu_isHalted = csr_pnce->read () == 0;

    sc_uint<CFG_NUT_CPU_CORES> m3_pnxsel_var, m3_pnce_var, m3_pnlm_var;
    for (int i = 0; i < CFG_NUT_CPU_CORES; i++) {
#pragma HLS unroll
        m3_pnxsel_var[i] = csr_pnxsel[i / 32].read ().bit (i % 32);
        m3_pnlm_var[i] = csr_pnlm[i / 32].read ().bit (i % 32);
        m3_pnce_var[i] = csr_pnce[i / 32].read ().bit (i % 32);
    }
    exu_m3_pnxsel = m3_pnxsel_var;
    exu_m3_pnce = m3_pnce_var;
    exu_m3_pnlm = m3_pnlm_var;

#ifndef __SYNTHESIS__
    exu_m3_icache_enable->write (csr_pncache.read ()[0] & !pn_cfg_disable_cache);
    exu_m3_dcache_enable->write (csr_pncache.read ()[1] & !pn_cfg_disable_cache);
#else
    exu_m3_icache_enable->write (csr_pncache.read ()[0]);
    exu_m3_dcache_enable->write (csr_pncache.read ()[1]);
#endif
}

void MCsr::CSRReadMethod (){
    ECSR csradr;
    sc_uint<32> rdata;
    bool read_exception;

    // Read input signals
    csradr = (ECSR) ((__uint16_t)csr_adr_reg.read ());
    rdata = 0;
    read_exception = 0;

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
#if CFG_PRIV_LEVELS >= 2
                // Extensions[25:0]: Bit 20: User-Mode supported 
                rdata |= 0x100000;
#endif
#if CFG_PRIV_LEVELS == 3
                // Extensions[25:0]: Bit 18: Supervisor-Mode supported 
                rdata |= 0x040000;
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
                rdata = CFG_NUT_MIMPID;
                break;
            case mhartid: // Machine Hart ID Register (RO)
            case pncoreid:
                // Hart ID is set on creation (see constructor of MExu)
                rdata = hartID.read ();
                break;
            case mstatus: // Machine Status Register
                rdata = getMstatus();
                break;
            case mtvec: // Machine Trap-Vector Base-Address Register
                rdata = csr_mtvec.read ();
                break;
            case mip: // Machine Interrupt-Pending Register
                rdata = csr_mip;
                // TODO: Add bits for External and Timer Interrupt
                // TODO: remove all those bits and use just a single 32 bit register
                break;
            case mie: // Machine Interrupt-Enable Register
                rdata = csr_mie;
                // 0 - Not implemented
                // todo: Add bits for External and Timer Interrupt
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
                rdata = exu_cause.read ();
                break;
            case pnepc: // ParaNut CoPU Exception Program Counter
                rdata = exu_epc.read ();
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
#if CFG_PRIV_LEVELS == 3
            case medeleg: // only required whith S-MODE and/or N-Extension available
                rdata = csr_medeleg;
                break;
            case mideleg:  // only required whith S-MODE and/or N-Extension available
                rdata = csr_mideleg;
                break;
            case scause:
                rdata = csr_scause;
                break;
            case sscratch:
                rdata = csr_sscratch;
                break;
            case sepc:
                rdata = csr_sepc;
                break;
            case stval:
                rdata = csr_mtval;
                break;
            case sip:
                // 0 - Not implemented
                // TODO: Add bits for External and Timer Interrupt
                rdata = 0x0;
                break;
            case sie:
                // 0 - Not implemented
                // TODO: Add bits for External and Timer Interrupt
                rdata = 0x0;
                break;
            case stvec:
                rdata = csr_stvec;
                break;
            case satp:
                // TODO: hardwire unsupported bits
                rdata = csr_satp;
                break;
            case sstatus:
                rdata = (csr_mstatus_SD,    // State dirty
                         sc_uint<11> (0),   // RESERVED
                         csr_mstatus_MXR,   // Make eXecutable Readable
                         csr_mstatus_SUM,   // permit Supervisor User Memory access
                         sc_uint<1> (0),    // MPRV: No PMP available - hardwired 0
                         csr_mstatus_XS,
                         sc_uint<2> (0),    // XS: No user-mode extensions - hardwired 0
                         sc_uint<2> (0),    // RESERVED
                         csr_mstatus_SPP,   // Supervisor Previous Privilege
                         sc_uint<2> (0),    // RESERVED
                         csr_mstatus_SPIE, // Supervisor PreviousInterrupt Enable
                         sc_uint<1> (0),    // UPIE: No N-extension - hardwired 0
                         sc_uint<2> (0),    // RESERVED
                         csr_mstatus_SIE, // Supervisor Interrupt Enable
                         sc_uint<1> (0)     // UIE: No N-extension - hardwired 0
                );
                break;
#endif
            default:
                PN_WARNINGF (("   (%s)  GetCSR: Read access to unknown CSR 0x%04x, Instruction: "
                           "(0x%08x) = 0x%08x",
                           strrchr (name (), '.') + 1, csradr, pc.read (), ir.read ()));
                rdata = 0x0;
                read_exception = 1;
            }
        }
#if CFG_PRIV_LEVELS >= 2
        // do not return a value if there is no appropriate permission
        if (csr_exception) {
            rdata = 0;
            PN_WARNINGF (("   (%s)  GetCSR: Read access to CSR 0x%04x without appropriate permissions, Instruction: "
                       "(0x%08x) = 0x%08x",
                       strrchr (name (), '.') + 1, csradr, pc.read (), ir.read ()));
        }
#endif
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
            case pncoreid:
                // Hart ID is set on creation (see constructor of MExu)
                rdata = hartID.read ();
                break;
            case pncpus: // Machine ParaNut Number of CPUs Register
                // Fixed configured value
                rdata = CFG_NUT_CPU_CORES;
                break;
            case pnce: // Machine ParaNut CPU Enable Register
                // CoPU:
                rdata = cpu_enabled.read ();
                break;
            case pnlm: // Machine ParaNut CPU Linked Mode Register
                // CoPU:
                rdata = linked.read ();
                break;
            default:
                PN_WARNINGF (("   (%s)  GetCSR: Read access to unknown CSR 0x%04x, Instruction: "
                           "(0x%08x) = 0x%08x",
                           strrchr (name (), '.') + 1, csradr, pc.read (), ir.read ()));
                read_exception = 1;
                rdata = 0;
            }
        }
    }

    // Output signals
    csr_rd_exception = read_exception;
    csr_rdata = rdata;
}

void MCsr::CSRWriteMethod (){
#pragma HLS ARRAY_PARTITION variable=csr_pnce complete dim=1
#pragma HLS ARRAY_PARTITION variable=csr_pnlm complete dim=1
#pragma HLS ARRAY_PARTITION variable=csr_pnx complete dim=1
#pragma HLS ARRAY_PARTITION variable=csr_pnxsel complete dim=1

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
            csr_mtvec = CFG_NUT_RESET_ADDR;
            csr_mscratch = 0;
            csr_dcsr_cause = 0;
            csr_dcsr_step = 0;
            csr_dcsr_ebreakm = 0;
            csr_dpc = 0;
            csr_dscratch0 = 0;
            csr_mip = 0;
            csr_mie = 0;

            // required when M/U mode available
            priv_mode_reg = Machine;
            csr_mstatus_MPP = Machine;

            // required when M/S/U mode available
            csr_medeleg = 0;
            csr_mideleg = 0;
            csr_stvec = 0;
            csr_scause = 0;
            csr_satp = 0;
            csr_mstatus_SIE = 0;
            csr_mstatus_SPIE = 0;
            csr_mstatus_SPP = 0;
            csr_mstatus_SUM = 0;
            csr_mstatus_MXR = 0;
            csr_mstatus_TVM = 0;
            csr_mstatus_TW = 0;
            csr_mstatus_TSR = 0;
            csr_mstatus_SD = 0;
            csr_mstatus_XS = 0;

            delegate_dreg = 0;

            // Reset ParaNut CSRs
            csr_pncache = 0;

            // Grouped ParaNut CSRs
            for(int i = 0; i < CFG_NUT_CPU_GROUPS+1; i++){
                # pragma HLS UNROLL
                csr_pnce[i] = (CFG_EXU_PNM2CAP & (0xffffffff << 32*i)) >> 32*i; // activa     te all mode 2 exus;
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
                    PN_INFOF (("   (%s)  SetCSR: Writing 0x%x to CSR 0x%x ignored because of implementation (WARL)", strrchr (name (), '.') + 1, wdata.value(), csradr));
                    break;
                case mstatus: // Machine Status Register
                    setMstatus (wdata);
                    break;
                case mtvec: // Machine Trap-Vector Base-Address Register
                    csr_mtvec = wdata;
                    break;
                case mip: // Machine Interrupt Pending Register
                    PN_INFOF (("   (%s)  SetCSR: Writing 0x%x to CSR 0x%x ignored because of implementation (WARL)", strrchr (name (), '.') + 1, wdata.value(), csradr));
                    break;
                case mie: // Machine Interrupt Enable Register
                    PN_INFOF (("   (%s)  SetCSR: Writing 0x%x to CSR 0x%x ignored because of implementation (WARL)", strrchr (name (), '.') + 1, wdata.value(), csradr));
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
                    PN_INFOF (("   (%s)  SetCSR: Writing 0x%x to CSR 0x%x ignored because it is not yet implemented", strrchr (name (), '.') + 1, wdata.value(), csradr));
                    break;
                case mtimecmph: // Machine Timer Compare Register 63-32
                    PN_INFOF (("   (%s)  SetCSR: Writing 0x%x to CSR 0x%x ignored because it is not yet implemented", strrchr (name (), '.') + 1, wdata.value(), csradr));
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
                    PN_INFOF (("   (%s)  SetCSR: Writing 0x%x to CSR 0x%x (mpngrpsel) ignored because of implementation", strrchr (name (), '.') + 1, wdata.value(), csradr));
#endif
                    break;
#if CFG_PRIV_LEVELS == 3
                case medeleg: // only required whith S-MODE and/or N-Extension available
                    csr_medeleg = (wdata.range (31, 12), sc_uint<1>(0), wdata.range(10, 0));
                    break;
                case mideleg:  // only required whith S-MODE and/or N-Extension available
                    csr_mideleg = wdata;
                    break;
                case scause:
                    csr_scause = wdata;
                    break;
                case stval:
                    csr_stval = wdata;
                    break;
                case sscratch:
                    // TODO
                    csr_sscratch = wdata;
                    break;
                case sepc:
                    csr_sepc = (wdata(31,2), sc_uint<2>(0) ); // TODO: For 16 bit ISA support only sepc[0] needs to be 0
                    break;
                case sip:
                    PN_INFOF (("   (%s)  SetCSR: Writing 0x%x to CSR 0x%x ignored because of implementation (WARL)", strrchr (name (), '.') + 1, wdata.value(), csradr));
                    // TODO: Add bits for External and Timer Interrupt
                    break;
                case sie:
                    PN_INFOF (("   (%s)  SetCSR: Writing 0x%x to CSR 0x%x ignored because of implementation (WARL)", strrchr (name (), '.') + 1, wdata.value(), csradr));
                    // TODO: Add bits for External and Timer Interrupt
                    break;
                case scounteren:
                    PN_INFOF (("   (%s)  SetCSR: Writing 0x%x to CSR 0x%x ignored because of implementation (WARL)", strrchr (name (), '.') + 1, wdata.value(), csradr));
                    // TODO
                    break;
                case stvec:
                    csr_stvec = wdata;
                    break;
                case satp:
                    // TODO: hardwire unsupported bits
                    csr_satp = wdata;
                    break;
                case sstatus:
                    csr_mstatus_SD = wdata[31];    // State dirty
                    csr_mstatus_MXR = wdata[19];   // Make eXecutable Readable
                    csr_mstatus_SUM = wdata[18];   // permit Supervisor User Memory access
                    csr_mstatus_XS = wdata.range (16, 15);
                    csr_mstatus_SPP = wdata[8];   // Supervisor Previous Privilege
                    csr_mstatus_SPIE = wdata[5]; // Supervisor PreviousInterrupt Enable
                    csr_mstatus_SIE = wdata[1]; // Supervisor Interrupt Enable
                    break;
#endif
                default:
                    // Special case: Machine Performance Monitoring Event Registers
                    if(csradr >= mhpmevent3 && csradr <= mhpmevent31){
                        PN_INFOF (("   (%s)  SetCSR: Writing 0x%x to CSR 0x%x ignored because of implementation (WARL)", strrchr (name (), '.') + 1, wdata.value(), csradr));
                    }else{
                        PN_WARNINGF(("   (%s)  SetCSR: CSR wfrite to unknown register 0x%04x, Instruction: (0x%08x) = 0x%08x", strrchr (name (), '.') + 1, csradr, pc.read(), ir.read()));
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
#if CFG_PRIV_LEVELS == 3
                    // TODO: this might be an extraordinary long path and may need optimizations
                    sc_uint<32> mdeleg = irq_dreg.read () ? csr_mideleg.read () : csr_medeleg.read ();
                    bool delegate = (1 << ex_id_reg.read ()) & mdeleg;
                    delegate_dreg = delegate;

                    if (delegate & (priv_mode_reg.read () != Machine)) {
                        csr_scause =  (irq_dreg.read (), sc_uint<26> (0), ex_id_reg.read ());
                        csr_mstatus_SPIE = csr_mstatus_SIE.read ();
                        csr_mstatus_SIE = 0;

                        csr_mstatus_SPP = priv_mode_reg.read ()[0];
                        priv_mode_reg = Supervisor;

                        csr_sepc = pc.read();
                        csr_stval = csr_tval.read ();
                    } else {
                        csr_mcause = (irq_dreg.read (), sc_uint<26> (0), ex_id_reg.read ());
                        csr_mstatus_MPIE = csr_mstatus_MIE.read ();
                        csr_mstatus_MIE = 0;

                        csr_mstatus_MPP = priv_mode_reg.read ();
                        priv_mode_reg = Machine;

                        csr_mepc = pc.read();
                        csr_mtval = csr_tval.read ();
                    }
#elif CFG_PRIV_LEVELS <= 2
                    csr_mcause = (irq_dreg.read (), sc_uint<26> (0), ex_id_reg.read ());
                    csr_mstatus_MPIE = csr_mstatus_MIE.read ();
                    csr_mstatus_MIE = 0;

                    csr_mepc = pc.read();
                    csr_mtval = csr_tval.read ();
#endif
#if CFG_PRIV_LEVELS == 2
                    csr_mstatus_MPP = priv_mode_reg.read ();
                    priv_mode_reg = Machine;
#endif
                    for(int i = 0; i < CFG_NUT_CPU_GROUPS+1; i++){
                        # pragma HLS UNROLL
                        csr_pnx[i] = m3_pnx.read()(MIN((XLEN*(i+1))-1, CFG_NUT_CPU_CORES-1),32*i);
                    }
                }

                // pop privilege and interrupt stack on xRET
                if(exu_pop_priv_ir_stack_dreg.read ()) {
#if CFG_PRIV_LEVELS == 3
                    if (mret_dreg) { // executing MRET
                        priv_mode_reg = csr_mstatus_MPP.read ();
                        csr_mstatus_MPP = User; 

                        csr_mstatus_MIE = csr_mstatus_MPIE.read ();
                        csr_mstatus_MPIE = 1;
                    } else { // SRET executed (since no N-Extensions supported)
                        priv_mode_reg = (sc_uint<1> (0), csr_mstatus_SPP.read ());
                        csr_mstatus_SPP = User; 

                        csr_mstatus_SIE = csr_mstatus_SPIE.read ();
                        csr_mstatus_SPIE = 1;
                    }
#elif CFG_PRIV_LEVELS == 2
                    priv_mode_reg = csr_mstatus_MPP.read();
                    csr_mstatus_MPP = User; 
#endif
#if CFG_PRIV_LEVELS <= 2
                    csr_mstatus_MIE = csr_mstatus_MPIE.read();
                    csr_mstatus_MPIE = 1;
#endif
                }

                // Set debug program counter and debug cause on debug entry
                if(exu_dbg & !exu_dbg_reg){
                    csr_dpc = pc.read();
                    csr_dcsr_cause = csr_dcsr_step ? 4 : exu_dbg_req ? 3 : 1; // Cause is either 4: Step or 3: Halt request or 1: ebreak
                }

                // Enable CePU when we get a debug request
                if(exu_dbg_req){
                  csr_pnce[0] = csr_pnce[0].read() | 0x1;
                }

            }
        } else { // if(inCePU)
            if(csr_write && (csradr != pnce && linked)){  // Don't warn on write to pnce in linked mode (happens during shutdown of linked mode)
                PN_WARNINGF(("   (%s)  SetCSR: CSR write in CoPU not permitted 0x%04x, Instruction: (0x%08x) = 0x%08x", strrchr (name (), '.') + 1, (__uint16_t)csr_adr_reg.read(), pc.read(), ir.read()));
            }

            // handle writing of exception values
            if(exception) {
                csr_mcause =  (sc_uint<26>(0), ex_id_reg.read());
                csr_mepc = pc.read();
            }

            // Set debug program counter and debug cause on debug entry
            if(exu_dbg & !exu_dbg_reg){
                csr_dpc = pc.read();
                csr_dcsr_cause = csr_dcsr_step ? 4 : exu_dbg_req ? 3 : 1; // Cause is either 4: Step or 3: Halt request or 1: ebreak
            }
        }
    }
}

sc_uint<32> MCsr::getMstatus () {
    return (
        csr_mstatus_SD, // State dirty
        sc_uint<1> (0),    // SD: hardwired 0
        sc_uint<8> (0),    // RESERVED
        csr_mstatus_TSR, // Trap SRET
        csr_mstatus_TW, // Timeout Wait
        csr_mstatus_TVM, // Trap Virtual Memory
        csr_mstatus_MXR, // Make eXecutable Readable
        csr_mstatus_SUM, // permit Supervisor User Memory access
        sc_uint<1> (0),    // MPRV: No PMP available - hardwired 0
        csr_mstatus_XS.read (),
        sc_uint<2> (0),    // FS: No floating point unit - hardwired 0
        // TODO: M/U setup only requires one bit. Elaborate if 
        // HLS terminates the other bit
        csr_mstatus_MPP.read (),
        sc_uint<2> (0),    // RESERVED
        csr_mstatus_SPP, // Supervisor Previous Privilege
        csr_mstatus_MPIE,  // MPIE
        sc_uint<1> (0),    // RESERVED
        csr_mstatus_SPIE, // Supervisor PreviousInterrupt Enable
        sc_uint<1> (0),    // UPIE: No N-extension - hardwired 0
        csr_mstatus_MIE,   // MIE
        sc_uint<1> (0),    // RESERVED
        csr_mstatus_SIE, // Supervisor Interrupt Enable
        sc_uint<1> (0)     // UIE: No N-extension - hardwired 0
    );
}

void MCsr::setMstatus (sc_uint<32> wdata) {
    csr_mstatus_MIE = wdata[3];
    csr_mstatus_MPIE = wdata[7];
#if CFG_PRIV_LEVELS == 2
    csr_mstatus_MPP = (wdata[12], wdata[12]);
#elif CFG_PRIV_LEVELS == 3
    csr_mstatus_MPP = (wdata[12], wdata[12] | wdata[11]); // avoid hypervisor
    csr_mstatus_SIE = wdata[1],
    csr_mstatus_SPIE = wdata[5],
    csr_mstatus_SPP = wdata[8],
    csr_mstatus_XS = wdata.range(14, 13);
    csr_mstatus_SUM = wdata[18],
    csr_mstatus_MXR = wdata[19],
    csr_mstatus_TVM = wdata[20],
    csr_mstatus_TW = wdata[21],
    csr_mstatus_TSR = wdata[22];
#endif
}
