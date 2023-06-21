/*************************************************************************
  
  This file was copied and modified from the Spike ISA Simulator project:
  https://github.com/riscv/riscv-isa-sim
  
  Copyright (c) 2012-2015, The Regents of the University of California (Regents).
  All Rights Reserved.
     This application requires the libparanut.
  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:
  1. Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.
  2. Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in the
     documentation and/or other materials provided with the distribution.
  3. Neither the name of the Regents nor the
     names of its contributors may be used to endorse or promote products
     derived from this software without specific prior written permission.

  IN NO EVENT SHALL REGENTS BE LIABLE TO ANY PARTY FOR DIRECT, INDIRECT,
  SPECIAL, INCIDENTAL, OR CONSEQUENTIAL DAMAGES, INCLUDING LOST PROFITS, ARISING
  OUT OF THE USE OF THIS SOFTWARE AND ITS DOCUMENTATION, EVEN IF REGENTS HAS
  BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

  REGENTS SPECIFICALLY DISCLAIMS ANY WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
  THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
  PURPOSE. THE SOFTWARE AND ACCOMPANYING DOCUMENTATION, IF ANY, PROVIDED
  HEREUNDER IS PROVIDED "AS IS". REGENTS HAS NO OBLIGATION TO PROVIDE
  MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.
  
*************************************************************************/

#![allow(dead_code)]
#![no_std]

#[allow(unused_imports)]
use core::arch::asm;

pub const MSTATUS_UIE: u32 = 0x0000_0001;
pub const MSTATUS_SIE: u32 = 0x0000_0002;
pub const MSTATUS_HIE: u32 = 0x0000_0004;
pub const MSTATUS_MIE: u32 = 0x0000_0008;
pub const MSTATUS_UPIE: u32 = 0x0000_0010;
pub const MSTATUS_SPIE: u32 = 0x0000_0020;
pub const MSTATUS_HPIE: u32 = 0x0000_0040;
pub const MSTATUS_MPIE: u32 = 0x0000_0080;
pub const MSTATUS_SPP: u32 = 0x0000_0100;
pub const MSTATUS_HPP: u32 = 0x0000_0600;
pub const MSTATUS_MPP: u32 = 0x0000_1800;
pub const MSTATUS_FS: u32 = 0x0000_6000;
pub const MSTATUS_XS: u32 = 0x0001_8000;
pub const MSTATUS_MPRV: u32 = 0x0002_0000;
pub const MSTATUS_SUM: u32 = 0x0004_0000;
pub const MSTATUS_MXR: u32 = 0x0008_0000;
pub const MSTATUS_TVM: u32 = 0x0010_0000;
pub const MSTATUS_TW: u32 = 0x0020_0000;
pub const MSTATUS_TSR: u32 = 0x0040_0000;
pub const MSTATUS32_SD: u32 = 0x8000_0000;
pub const MSTATUS_UXL: u64 = 0x0000_0003_0000_0000;
pub const MSTATUS_SXL: u64 = 0x0000_000C_0000_0000;
pub const MSTATUS64_SD: u64 = 0x8000_0000_0000_0000;

pub const SSTATUS_UIE: u32 = 0x0000_0001;
pub const SSTATUS_SIE: u32 = 0x0000_0002;
pub const SSTATUS_UPIE: u32 = 0x0000_0010;
pub const SSTATUS_SPIE: u32 = 0x0000_0020;
pub const SSTATUS_SPP: u32 = 0x0000_0100;
pub const SSTATUS_FS: u32 = 0x0000_6000;
pub const SSTATUS_XS: u32 = 0x0001_8000;
pub const SSTATUS_SUM: u32 = 0x0004_0000;
pub const SSTATUS_MXR: u32 = 0x0008_0000;
pub const SSTATUS32_SD: u32 = 0x8000_0000;
pub const SSTATUS_UXL: u64 = 0x0000_0003_0000_0000;
pub const SSTATUS64_SD: u64 = 0x8000_0000_0000_0000;

pub const DCSR_XDEBUGVER: u32 = 3<<30;
pub const DCSR_NDRESET: u32 = 1<<29;
pub const DCSR_FULLRESET: u32 = 1<<28;
pub const DCSR_EBREAKM: u32 = 1<<15;
pub const DCSR_EBREAKH: u32 = 1<<14;
pub const DCSR_EBREAKS: u32 = 1<<13;
pub const DCSR_EBREAKU: u32 = 1<<12;
pub const DCSR_STOPCYCLE: u32 = 1<<10;
pub const DCSR_STOPTIME: u32 = 1<<9;
pub const DCSR_CAUSE: u32 = 7<<6;
pub const DCSR_DEBUGINT: u32 = 1<<5;
pub const DCSR_HALT: u32 = 1<<3;
pub const DCSR_STEP: u32 = 1<<2;
pub const DCSR_PRV: u32 = 3<<0;

pub const DCSR_CAUSE_NONE: u32 = 0;
pub const DCSR_CAUSE_SWBP: u32 = 1;
pub const DCSR_CAUSE_HWBP: u32 = 2;
pub const DCSR_CAUSE_DEBUGINT: u32 = 3;
pub const DCSR_CAUSE_STEP: u32 = 4;
pub const DCSR_CAUSE_HALT: u32 = 5;

#[macro_export]
macro_rules! MCONTROL_TYPE{
    ($xlen:expr) => {

        {
            0xf_u64 << ($xlen - 4)
        }

    }
}

#[macro_export]
macro_rules! MCONTROL_DMODE{
    ($xlen:expr) => {

        {
            1_u64 << ($xlen - 5)
        }

    }
}

#[macro_export]
macro_rules! MCONTROL_MASKMAX{
    ($xlen:expr) => {

        {
            0x3_u64 << ($xlen -11)
        }

    }
}

pub const MCONTROL_SELECT: u32 = 1<<19;
pub const MCONTROL_TIMING: u32 = 1<<18;
pub const MCONTROL_ACTION: u32 = 0x3f<<12;
pub const MCONTROL_CHAIN: u32 = 1<<11;
pub const MCONTROL_MATCH: u32 = 0xf<<7;
pub const MCONTROL_M: u32 = 1<<6;
pub const MCONTROL_H: u32 = 1<<5;
pub const MCONTROL_S: u32 = 1<<4;
pub const MCONTROL_U: u32 = 1<<3;
pub const MCONTROL_EXECUTE: u32 = 1<<2;
pub const MCONTROL_STORE: u32 = 1<<1;
pub const MCONTROL_LOAD: u32 = 1<<0;

pub const MCONTROL_TYPE_NONE: u32 = 0;
pub const MCONTROL_TYPE_MATCH: u32 = 2;

pub const MCONTROL_ACTION_DEBUG_EXCEPTION: u32 = 0;
pub const MCONTROL_ACTION_DEBUG_MODE: u32 = 1;
pub const MCONTROL_ACTION_TRACE_START: u32 = 2;
pub const MCONTROL_ACTION_TRACE_STOP: u32 = 3;
pub const MCONTROL_ACTION_TRACE_EMIT: u32 = 4;

pub const MCONTROL_MATCH_EQUAL: u32 = 0;
pub const MCONTROL_MATCH_NAPOT: u32 = 1;
pub const MCONTROL_MATCH_GE: u32 = 2;
pub const MCONTROL_MATCH_LT: u32 = 3;
pub const MCONTROL_MATCH_MASK_LOW: u32 = 4;
pub const MCONTROL_MATCH_MASK_HIGH: u32 = 5;

pub const MIP_SSIP: u32 = 1 << IRQ_S_SOFT;
pub const MIP_HSIP: u32 = 1 << IRQ_H_SOFT;
pub const MIP_MSIP: u32 = 1 << IRQ_M_SOFT;
pub const MIP_STIP: u32 = 1 << IRQ_S_TIMER;
pub const MIP_HTIP: u32 = 1 << IRQ_H_TIMER;
pub const MIP_MTIP: u32 = 1 << IRQ_M_TIMER;
pub const MIP_SEIP: u32 = 1 << IRQ_S_EXT;
pub const MIP_HEIP: u32 = 1 << IRQ_H_EXT;
pub const MIP_MEIP: u32 = 1 << IRQ_M_EXT;

pub const MIE_SSIE: u32 = 1 << IRQ_S_SOFT;
pub const MIE_HSIE: u32 = 1 << IRQ_H_SOFT;
pub const MIE_MSIE: u32 = 1 << IRQ_M_SOFT;
pub const MIE_STIE: u32 = 1 << IRQ_S_TIMER;
pub const MIE_HTIE: u32 = 1 << IRQ_H_TIMER;
pub const MIE_MTIE: u32 = 1 << IRQ_M_TIMER;
pub const MIE_SEIE: u32 = 1 << IRQ_S_EXT;
pub const MIE_HEIE: u32 = 1 << IRQ_H_EXT;
pub const MIE_MEIE: u32 = 1 << IRQ_M_EXT;

pub const SIP_SSIP: u32 = 1 << IRQ_S_SOFT;
pub const SIP_STIP: u32 = 1 << IRQ_S_TIMER;

pub const PRV_U: u32 = 0;
pub const PRV_S: u32 = 1;
pub const PRV_H: u32 = 2;
pub const PRV_M: u32 = 3;

pub const SATP32_MODE: u32 = 0x8000_0000;
pub const SATP32_ASID: u32 = 0x7FC0_0000;
pub const SATP32_PPN: u32 = 0x003F_FFFF;
pub const SATP64_MODE: u64 = 0xF000_0000_0000_0000;
pub const SATP64_ASID: u64 = 0x0FFF_F000_0000_0000;
pub const SATP64_PPN: u64 = 0x0000_0FFF_FFFF_FFFF;

pub const SATP_MODE_OFF: u32 = 0;
pub const SATP_MODE_SV32: u32 = 1;
pub const SATP_MODE_SV39: u32 = 8;
pub const SATP_MODE_SV48: u32 = 9;
pub const SATP_MODE_SV57: u32 = 10;
pub const SATP_MODE_SV64: u32 = 11;

pub const PMP_R: u32 = 0x01;
pub const PMP_W: u32 = 0x02;
pub const PMP_X: u32 = 0x04;
pub const PMP_A: u32 = 0x18;
pub const PMP_L: u32 = 0x80;
pub const PMP_SHIFT: u32 = 2;

pub const PMP_TOR: u32 = 0x08;
pub const PMP_NA4: u32 = 0x10;
pub const PMP_NAPOT: u32 = 0x18;

pub const IRQ_S_SOFT: u32 = 1;
pub const IRQ_H_SOFT: u32 = 2;
pub const IRQ_M_SOFT: u32 = 3;
pub const IRQ_S_TIMER: u32 = 5;
pub const IRQ_H_TIMER: u32 = 6;
pub const IRQ_M_TIMER: u32 = 7;
pub const IRQ_S_EXT: u32 = 9;
pub const IRQ_H_EXT: u32 = 10;
pub const IRQ_M_EXT: u32 = 11;
pub const IRQ_COP: u32 = 12;
pub const IRQ_HOST: u32 = 13;

pub const DEFAULT_RSTVEC: u32 = 0x0000_1000;
pub const CLINT_BASE: u32 = 0x0200_0000;
pub const CLINT_SIZE: u32 = 0x000c_0000;
pub const EXT_IO_BASE: u32 = 0x4000_0000;
pub const DRAM_BASE: u32 = 0x8000_0000;

// page table entry (PTE) fields
pub const PTE_V: u32 = 0x001; // Valid
pub const PTE_R: u32 = 0x002; // Read
pub const PTE_W: u32 = 0x004; // Write
pub const PTE_X: u32 = 0x008; // Execute
pub const PTE_U: u32 = 0x010; // User
pub const PTE_G: u32 = 0x020; // Global
pub const PTE_A: u32 = 0x040; // Accessed
pub const PTE_D: u32 = 0x080; // Dirty
pub const PTE_SOFT: u32 = 0x300; // Reserved for Software

pub const PTE_PPN_SHIFT: u32 = 10;

#[macro_export]
macro_rules! PTE_TABLE{
    ($pte:expr) => {

        {
            ((($pte) & (PTE_V | PTE_R | PTE_W | PTE_X)) == PTE_V)
        }

    }
}

pub const MSTATUS_SD: u32 = 0x80000000;
pub const SSTATUS_SD: u32 = 0x80000000;
pub const RISCV_PGLEVEL_BITS: u32  = 10;
pub const SATP_MODE: u32 = 0x80000000;

pub const RISCV_PGSHIFT: u32 = 12;
pub const RISCV_PGSIZE: u32 = 1 << 12;

#[macro_export]
macro_rules! read_csr{
    ($register:expr) => {
        {
            let return_value: usize;
            asm!(concat!("csrr {val}, ", $register),
             val = out(reg) return_value,
            );

            return_value
        }
    }
}

#[macro_export]
macro_rules! write_csr{
    ($register:expr, $input_value:expr) => {
        {
            asm!(concat!("csrw ", $register,", {val}"),
             val = in(reg) $input_value,
            );
        }
    }
}

#[macro_export]
macro_rules! swap_csr{
    ($register:expr, $input_value:expr) => {
        {
            let return_value: usize;
            asm!(concat!("csrrw {r_val}, ", $register, ", {i_val}"),
             r_val = out(reg) return_value,
             i_val = in(reg) $input_value,
            );

            return_value
        }
    }
}

#[macro_export]
macro_rules! set_csr{
    ($register:expr, $input_bit:expr) => {
        {
            let return_value: usize;
            asm!(concat!("csrrs {r_val}, ", $register, ", {i_bit}"),
             r_val = out(reg) return_value,
             i_bit = in(reg) $input_bit,
            );

            return_value
        }
    }
}

#[macro_export]
macro_rules! clear_csr{
    ($register:expr, $input_bit:expr) => {
        {
            let return_value: usize;
            asm!(concat!("csrrc {r_val}, ", $register, ", {i_bit}"),
             r_val = out(reg) return_value,
             i_bit = in(reg) $input_bit,
            );

            return_value
        }
    }
}

#[macro_export]
macro_rules! rdtime{
    () => {
        {
            read_csr(CSR_TIME);
        }
    }
}

#[macro_export]
macro_rules! rdcycle{
    () => {
        {
            read_csr(CSR_CYCLE);
        }
    }
}

#[macro_export]
macro_rules! rdinstret{
    () => {
        {
            read_csr(CSR_INSTRET);
        }
    }
}


#[macro_export]
macro_rules! read_pncsr{
    ($register:expr) => {
        {
            let return_value: usize;
            asm!(concat!("csrrs {r_val}, ", $register, ", {i_bit}"),
             r_val = out(reg) return_value,
             i_bit = in(reg) $input_bit,
            );

            return_value
        }
    }
}


pub const MATCH_BEQ: u32 = 0x63;
pub const MASK_BEQ: u32 = 0x707f;
pub const MATCH_BNE: u32 = 0x1063;
pub const MASK_BNE: u32 = 0x707f;
pub const MATCH_BLT: u32 = 0x4063;
pub const MASK_BLT: u32 = 0x707f;
pub const MATCH_BGE: u32 = 0x5063;
pub const MASK_BGE: u32 = 0x707f;
pub const MATCH_BLTU: u32 = 0x6063;
pub const MASK_BLTU: u32 = 0x707f;
pub const MATCH_BGEU: u32 = 0x7063;
pub const MASK_BGEU: u32 = 0x707f;
pub const MATCH_JALR: u32 = 0x67;
pub const MASK_JALR: u32 = 0x707f;
pub const MATCH_JAL: u32 = 0x6f;
pub const MASK_JAL: u32 = 0x7f;
pub const MATCH_LUI: u32 = 0x37;
pub const MASK_LUI: u32 = 0x7f;
pub const MATCH_AUIPC: u32 = 0x17;
pub const MASK_AUIPC: u32 = 0x7f;
pub const MATCH_ADDI: u32 = 0x13;
pub const MASK_ADDI: u32 = 0x707f;
pub const MATCH_SLLI: u32 = 0x1013;
pub const MASK_SLLI: u32 = 0xfc00_707f;
pub const MATCH_SLTI: u32 = 0x2013;
pub const MASK_SLTI: u32 = 0x707f;
pub const MATCH_SLTIU: u32 = 0x3013;
pub const MASK_SLTIU: u32 = 0x707f;
pub const MATCH_XORI: u32 = 0x4013;
pub const MASK_XORI: u32 = 0x707f;
pub const MATCH_SRLI: u32 = 0x5013;
pub const MASK_SRLI: u32 = 0xfc00_707f;
pub const MATCH_SRAI: u32 = 0x4000_5013;
pub const MASK_SRAI: u32 = 0xfc00_707f;
pub const MATCH_ORI: u32 = 0x6013;
pub const MASK_ORI: u32 = 0x707f;
pub const MATCH_ANDI: u32 = 0x7013;
pub const MASK_ANDI: u32 = 0x707f;
pub const MATCH_ADD: u32 = 0x33;
pub const MASK_ADD: u32 = 0xfe00_707f;
pub const MATCH_SUB: u32 = 0x4000_0033;
pub const MASK_SUB: u32 = 0xfe00_707f;
pub const MATCH_SLL: u32 = 0x1033;
pub const MASK_SLL: u32 = 0xfe00_707f;
pub const MATCH_SLT: u32 = 0x2033;
pub const MASK_SLT: u32 = 0xfe00_707f;
pub const MATCH_SLTU: u32 = 0x3033;
pub const MASK_SLTU: u32 = 0xfe00_707f;
pub const MATCH_XOR: u32 = 0x4033;
pub const MASK_XOR: u32 = 0xfe00_707f;
pub const MATCH_SRL: u32 = 0x5033;
pub const MASK_SRL: u32 = 0xfe00_707f;
pub const MATCH_SRA: u32 = 0x4000_5033;
pub const MASK_SRA: u32 = 0xfe00_707f;
pub const MATCH_OR: u32 = 0x6033;
pub const MASK_OR: u32 = 0xfe00_707f;
pub const MATCH_AND: u32 = 0x7033;
pub const MASK_AND: u32 = 0xfe00_707f;
pub const MATCH_ADDIW: u32 = 0x1b;
pub const MASK_ADDIW: u32 = 0x707f;
pub const MATCH_SLLIW: u32 = 0x101b;
pub const MASK_SLLIW: u32 = 0xfe00_707f;
pub const MATCH_SRLIW: u32 = 0x501b;
pub const MASK_SRLIW: u32 = 0xfe00_707f;
pub const MATCH_SRAIW: u32 = 0x4000_501b;
pub const MASK_SRAIW: u32 = 0xfe00_707f;
pub const MATCH_ADDW: u32 = 0x3b;
pub const MASK_ADDW: u32 = 0xfe00_707f;
pub const MATCH_SUBW: u32 = 0x4000_003b;
pub const MASK_SUBW: u32 = 0xfe00_707f;
pub const MATCH_SLLW: u32 = 0x103b;
pub const MASK_SLLW: u32 = 0xfe00_707f;
pub const MATCH_SRLW: u32 = 0x503b;
pub const MASK_SRLW: u32 = 0xfe00_707f;
pub const MATCH_SRAW: u32 = 0x4000_503b;
pub const MASK_SRAW: u32 = 0xfe00_707f;
pub const MATCH_LB: u32 = 0x3;
pub const MASK_LB: u32 = 0x707f;
pub const MATCH_LH: u32 = 0x1003;
pub const MASK_LH: u32 = 0x707f;
pub const MATCH_LW: u32 = 0x2003;
pub const MASK_LW: u32 = 0x707f;
pub const MATCH_LD: u32 = 0x3003;
pub const MASK_LD: u32 = 0x707f;
pub const MATCH_LBU: u32 = 0x4003;
pub const MASK_LBU: u32 = 0x707f;
pub const MATCH_LHU: u32 = 0x5003;
pub const MASK_LHU: u32 = 0x707f;
pub const MATCH_LWU: u32 = 0x6003;
pub const MASK_LWU: u32 = 0x707f;
pub const MATCH_SB: u32 = 0x23;
pub const MASK_SB: u32 = 0x707f;
pub const MATCH_SH: u32 = 0x1023;
pub const MASK_SH: u32 = 0x707f;
pub const MATCH_SW: u32 = 0x2023;
pub const MASK_SW: u32 = 0x707f;
pub const MATCH_SD: u32 = 0x3023;
pub const MASK_SD: u32 = 0x707f;
pub const MATCH_FENCE: u32 = 0xf;
pub const MASK_FENCE: u32 = 0x707f;
pub const MATCH_FENCE_I: u32 = 0x100f;
pub const MASK_FENCE_I: u32 = 0x707f;
pub const MATCH_MUL: u32 = 0x200_0033;
pub const MASK_MUL: u32 = 0xfe00_707f;
pub const MATCH_MULH: u32 = 0x200_1033;
pub const MASK_MULH: u32 = 0xfe00_707f;
pub const MATCH_MULHSU: u32 = 0x200_2033;
pub const MASK_MULHSU: u32 = 0xfe00_707f;
pub const MATCH_MULHU: u32 = 0x200_3033;
pub const MASK_MULHU: u32 = 0xfe00_707f;
pub const MATCH_DIV: u32 = 0xfe00_707f;
pub const MASK_DIV: u32 = 0xfe00_707f;
pub const MATCH_DIVU: u32 = 0x200_5033;
pub const MASK_DIVU: u32 = 0xfe00_707f;
pub const MATCH_REM: u32 = 0x200_6033;
pub const MASK_REM: u32 = 0xfe00_707f;
pub const MATCH_REMU: u32 = 0x200_7033;
pub const MASK_REMU: u32 = 0xfe00_707f;
pub const MATCH_MULW: u32 = 0x200_003b;
pub const MASK_MULW: u32 = 0xfe00_707f;
pub const MATCH_DIVW: u32 = 0x200_403b;
pub const MASK_DIVW: u32 = 0xfe00_707f;
pub const MATCH_DIVUW: u32 = 0x200_503b;
pub const MASK_DIVUW: u32 = 0xfe00_707f;
pub const MATCH_REMW: u32 = 0x200_603b;
pub const MASK_REMW: u32 = 0xfe00_707f;
pub const MATCH_REMUW: u32 = 0x200_703b;
pub const MASK_REMUW: u32 = 0xfe00_707f;
pub const MATCH_AMOADD_W: u32 = 0x202f;
pub const MASK_AMOADD_W: u32 = 0x202f;
pub const MATCH_AMOXOR_W: u32 = 0x2000_202f;
pub const MASK_AMOXOR_W: u32 = 0xf800707f;
pub const MATCH_AMOOR_W: u32 = 0x4000_202f;
pub const MASK_AMOOR_W: u32 = 0xf800_707f;
pub const MATCH_AMOAND_W: u32 = 0x6000_202f;
pub const MASK_AMOAND_W: u32 = 0xf800_707f;
pub const MATCH_AMOMIN_W: u32 = 0x8000_202f;
pub const MASK_AMOMIN_W: u32 = 0xf800_707f;
pub const MATCH_AMOMAX_W: u32 = 0xa000_202f;
pub const MASK_AMOMAX_W: u32 = 0xf800_707f;
pub const MATCH_AMOMINU_W: u32 = 0xc000_202f;
pub const MASK_AMOMINU_W: u32 = 0xf800_707f;
pub const MATCH_AMOMAXU_W: u32 = 0xe00_0202f;
pub const MASK_AMOMAXU_W: u32 = 0xf800_707f;
pub const MATCH_AMOSWAP_W: u32 = 0x800_202f;
pub const MASK_AMOSWAP_W: u32 = 0xf800_707f;
pub const MATCH_LR_W: u32 = 0x1000_202f;
pub const MASK_LR_W: u32 = 0xf9f0_707f;
pub const MATCH_SC_W: u32 = 0x1800_202f;
pub const MASK_SC_W: u32 = 0xf800_707f;
pub const MATCH_AMOADD_D: u32 = 0x302f;
pub const MASK_AMOADD_D: u32 = 0xf800_707f;
pub const MATCH_AMOXOR_D: u32 = 0x2000_302f;
pub const MASK_AMOXOR_D: u32 = 0xf800_707f;
pub const MATCH_AMOOR_D: u32 = 0x4000_302f;
pub const MASK_AMOOR_D: u32 = 0xf800_707f;
pub const MATCH_AMOAND_D: u32 = 0x6000_302f;
pub const MASK_AMOAND_D: u32 = 0xf800_707f;
pub const MATCH_AMOMIN_D: u32 = 0x8000_302f;
pub const MASK_AMOMIN_D: u32 = 0xf800_707f;
pub const MATCH_AMOMAX_D: u32 = 0xa000_302f;
pub const MASK_AMOMAX_D: u32 = 0xf800_707f;
pub const MATCH_AMOMINU_D: u32 = 0xc000_302f;
pub const MASK_AMOMINU_D: u32 = 0xf800_707f;
pub const MATCH_AMOMAXU_D: u32 = 0xe000_302f;
pub const MASK_AMOMAXU_D: u32 = 0xf800_707f;
pub const MATCH_AMOSWAP_D: u32 = 0x800_302f;
pub const MASK_AMOSWAP_D: u32 = 0xf800_707f;
pub const MATCH_LR_D: u32 = 0x1000_302f;
pub const MASK_LR_D: u32 = 0xf9f0_707f;
pub const MATCH_SC_D: u32 = 0x1800_302f;
pub const MASK_SC_D: u32 = 0xf800_707f;
pub const MATCH_ECALL: u32 = 0x73;
pub const MASK_ECALL: u32 = 0xffff_ffff;
pub const MATCH_EBREAK: u32 = 0x10_0073;
pub const MASK_EBREAK: u32 = 0xffff_ffff;
pub const MATCH_URET: u32 = 0x20_0073;
pub const MASK_URET: u32 = 0xffff_ffff;
pub const MATCH_SRET: u32 = 0x1020_0073;
pub const MASK_SRET: u32 = 0xffff_ffff;
pub const MATCH_MRET: u32 = 0x3020_0073;
pub const MASK_MRET: u32 = 0xffff_ffff;
pub const MATCH_DRET: u32 = 0x7b20_0073;
pub const MASK_DRET: u32 = 0xffff_ffff;
pub const MATCH_SFENCE_VMA: u32 = 0x1200_0073;
pub const MASK_SFENCE_VMA: u32 = 0xfe00_7fff;
pub const MATCH_WFI: u32 = 0x1050_0073;
pub const MASK_WFI: u32 = 0xffff_ffff;
pub const MATCH_CSRRW: u32 = 0x1073;
pub const MASK_CSRRW: u32 = 0x707f;
pub const MATCH_CSRRS: u32 = 0x2073;
pub const MASK_CSRRS: u32 = 0x707f;
pub const MATCH_CSRRC: u32 = 0x3073;
pub const MASK_CSRRC: u32 = 0x707f;
pub const MATCH_CSRRWI: u32 = 0x5073;
pub const MASK_CSRRWI: u32 = 0x707f;
pub const MATCH_CSRRSI: u32 = 0x6073;
pub const MASK_CSRRSI: u32 = 0x707f;
pub const MATCH_CSRRCI: u32 = 0x7073;
pub const MASK_CSRRCI: u32 = 0x707f;
pub const MATCH_FADD_S: u32 = 0x53;
pub const MASK_FADD_S: u32 = 0xfe00_007f;
pub const MATCH_FSUB_S: u32 = 0x800_0053;
pub const MASK_FSUB_S: u32 = 0xfe00_007f;
pub const MATCH_FMUL_S: u32 = 0x1000_0053;
pub const MASK_FMUL_S: u32 = 0xfe00_007f;
pub const MATCH_FDIV_S: u32 = 0x1800_0053;
pub const MASK_FDIV_S: u32 = 0xfe00_007f;
pub const MATCH_FSGNJ_S: u32 = 0x2000_0053;
pub const MASK_FSGNJ_S: u32 = 0xfe00_707f;
pub const MATCH_FSGNJN_S: u32 = 0x2000_1053;
pub const MASK_FSGNJN_S: u32 = 0xfe00_707f;
pub const MATCH_FSGNJX_S: u32 = 0x2000_2053;
pub const MASK_FSGNJX_S: u32 = 0xfe00_707f;
pub const MATCH_FMIN_S: u32 = 0x2800_0053;
pub const MASK_FMIN_S: u32 = 0xfe00_707f;
pub const MATCH_FMAX_S: u32 = 0x2800_1053;
pub const MASK_FMAX_S: u32 = 0xfe00_707f;
pub const MATCH_FSQRT_S: u32 = 0x5800_0053;
pub const MASK_FSQRT_S: u32 = 0xfff0_007f;
pub const MATCH_FADD_D: u32 = 0x200_0053;
pub const MASK_FADD_D: u32 = 0xfe00_007f;
pub const MATCH_FSUB_D: u32 = 0xa00_0053;
pub const MASK_FSUB_D: u32 = 0xfe00_007f;
pub const MATCH_FMUL_D: u32 = 0x1200_0053;
pub const MASK_FMUL_D: u32 = 0xfe00_007f;
pub const MATCH_FDIV_D: u32 = 0x1a00_0053;
pub const MASK_FDIV_D: u32 = 0xfe00_007f;
pub const MATCH_FSGNJ_D: u32 = 0x2200_0053;
pub const MASK_FSGNJ_D: u32 = 0xfe00_707f;
pub const MATCH_FSGNJN_D: u32 = 0x2200_1053;
pub const MASK_FSGNJN_D: u32 = 0xfe00_707f;
pub const MATCH_FSGNJX_D: u32 = 0x2200_2053;
pub const MASK_FSGNJX_D: u32 = 0xfe00_707f;
pub const MATCH_FMIN_D: u32 = 0x2a00_0053;
pub const MASK_FMIN_D: u32 = 0xfe00_707f;
pub const MATCH_FMAX_D: u32 = 0x2a00_1053;
pub const MASK_FMAX_D: u32 = 0xfe00_707f;
pub const MATCH_FCVT_S_D: u32 = 0x4010_0053;
pub const MASK_FCVT_S_D: u32 = 0xfff0_007f;
pub const MATCH_FCVT_D_S: u32 = 0x4200_0053;
pub const MASK_FCVT_D_S: u32 = 0xfff0_007f;
pub const MATCH_FSQRT_D: u32 = 0x5a00_0053;
pub const MASK_FSQRT_D: u32 = 0xfff0_007f;
pub const MATCH_FADD_Q: u32 = 0x600_0053;
pub const MASK_FADD_Q: u32 = 0xfe00_007f;
pub const MATCH_FSUB_Q: u32 = 0xe00_0053;
pub const MASK_FSUB_Q: u32 = 0xfe00_007f;
pub const MATCH_FMUL_Q: u32 = 0x1600_0053;
pub const MASK_FMUL_Q: u32 = 0xfe00_007f;
pub const MATCH_FDIV_Q: u32 = 0x1e00_0053;
pub const MASK_FDIV_Q: u32 = 0xfe00_007f;
pub const MATCH_FSGNJ_Q: u32 = 0x2600_0053;
pub const MASK_FSGNJ_Q: u32 = 0xfe00_707f;
pub const MATCH_FSGNJN_Q: u32 = 0x2600_1053;
pub const MASK_FSGNJN_Q: u32 = 0xfe00_707f;
pub const MATCH_FSGNJX_Q: u32 = 0x2600_2053;
pub const MASK_FSGNJX_Q: u32 = 0xfe00_707f;
pub const MATCH_FMIN_Q: u32 = 0x2e00_0053;
pub const MASK_FMIN_Q: u32 = 0xfe00_707f;
pub const MATCH_FMAX_Q: u32 = 0x2e00_1053;
pub const MASK_FMAX_Q: u32 = 0xfe00_707f;
pub const MATCH_FCVT_S_Q: u32 = 0x4030_0053;
pub const MASK_FCVT_S_Q: u32 = 0xfff0_007f;
pub const MATCH_FCVT_Q_S: u32 = 0x4600_0053;
pub const MASK_FCVT_Q_S: u32 = 0xfff0_007f;
pub const MATCH_FCVT_D_Q: u32 = 0x4230_0053;
pub const MASK_FCVT_D_Q: u32 = 0xfff0_007f;
pub const MATCH_FCVT_Q_D: u32 = 0x4610_0053;
pub const MASK_FCVT_Q_D: u32 = 0xfff0_007f;
pub const MATCH_FSQRT_Q: u32 = 0x5e00_0053;
pub const MASK_FSQRT_Q: u32 = 0xfff0_007f;
pub const MATCH_FLE_S: u32 = 0xa000_0053;
pub const MASK_FLE_S: u32 = 0xfe00_707f;
pub const MATCH_FLT_S: u32 = 0xa000_1053;
pub const MASK_FLT_S: u32 = 0xfe00_707f;
pub const MATCH_FEQ_S: u32 = 0xa000_2053;
pub const MASK_FEQ_S: u32 = 0xfe00_707f;
pub const MATCH_FLE_D: u32 = 0xa200_0053;
pub const MASK_FLE_D: u32 = 0xfe00_707f;
pub const MATCH_FLT_D: u32 = 0xa200_1053;
pub const MASK_FLT_D: u32 = 0xfe00_707f;
pub const MATCH_FEQ_D: u32 = 0xa200_2053;
pub const MASK_FEQ_D: u32 = 0xfe00_707f;
pub const MATCH_FLE_Q: u32 = 0xa600_0053;
pub const MASK_FLE_Q: u32 = 0xfe00_707f;
pub const MATCH_FLT_Q: u32 = 0xa600_1053;
pub const MASK_FLT_Q: u32 = 0xfe00_707f;
pub const MATCH_FEQ_Q: u32 = 0xa600_2053;
pub const MASK_FEQ_Q: u32 = 0xfe00_707f;
pub const MATCH_FCVT_W_S: u32 = 0xc000_0053;
pub const MASK_FCVT_W_S: u32 = 0xfff0_007f;
pub const MATCH_FCVT_WU_S: u32 = 0xc010_0053;
pub const MASK_FCVT_WU_S: u32 = 0xfff0_007f;
pub const MATCH_FCVT_L_S: u32 = 0xc020_0053;
pub const MASK_FCVT_L_S: u32 = 0xfff0_007f;
pub const MATCH_FCVT_LU_S: u32 = 0xc030_0053;
pub const MASK_FCVT_LU_S: u32 = 0xfff0_007f;
pub const MATCH_FMV_X_W: u32 = 0xe000_0053;
pub const MASK_FMV_X_W: u32 = 0xfff0_707f;
pub const MATCH_FCLASS_S: u32 = 0xe000_1053;
pub const MASK_FCLASS_S: u32 = 0xfff0_707f;
pub const MATCH_FCVT_W_D: u32 = 0xc200_0053;
pub const MASK_FCVT_W_D: u32 = 0xfff0_007f;
pub const MATCH_FCVT_WU_D: u32 = 0xc210_0053;
pub const MASK_FCVT_WU_D: u32 = 0xfff0_007f;
pub const MATCH_FCVT_L_D: u32 = 0xc220_0053;
pub const MASK_FCVT_L_D: u32 = 0xfff0_007f;
pub const MATCH_FCVT_LU_D: u32 = 0xc230_0053;
pub const MASK_FCVT_LU_D: u32 = 0xfff0_007f;
pub const MATCH_FMV_X_D: u32 = 0xe200_0053;
pub const MASK_FMV_X_D: u32 = 0xfff0_707f;
pub const MATCH_FCLASS_D: u32 = 0xe200_1053;
pub const MASK_FCLASS_D: u32 = 0xfff0_707f;
pub const MATCH_FCVT_W_Q: u32 = 0xc600_0053;
pub const MASK_FCVT_W_Q: u32 = 0xfff0_007f;
pub const MATCH_FCVT_WU_Q: u32 = 0xc610_0053;
pub const MASK_FCVT_WU_Q: u32 = 0xfff0_007f;
pub const MATCH_FCVT_L_Q: u32 = 0xc620_0053;
pub const MASK_FCVT_L_Q: u32 = 0xfff0_007f;
pub const MATCH_FCVT_LU_Q: u32 = 0xc630_0053;
pub const MASK_FCVT_LU_Q: u32 = 0xfff0_007f;
pub const MATCH_FMV_X_Q: u32 = 0xe600_0053;
pub const MASK_FMV_X_Q: u32 = 0xfff0_707f;
pub const MATCH_FCLASS_Q: u32 = 0xe600_1053;
pub const MASK_FCLASS_Q: u32 = 0xfff0_707f;
pub const MATCH_FCVT_S_W: u32 = 0xd000_0053;
pub const MASK_FCVT_S_W: u32 = 0xfff0_007f;
pub const MATCH_FCVT_S_WU: u32 = 0xd010_0053;
pub const MASK_FCVT_S_WU: u32 = 0xfff0_007f;
pub const MATCH_FCVT_S_L: u32 = 0xd020_0053;
pub const MASK_FCVT_S_L: u32 = 0xfff0_007f;
pub const MATCH_FCVT_S_LU: u32 = 0xd030_0053;
pub const MASK_FCVT_S_LU: u32 = 0xfff0_007f;
pub const MATCH_FMV_W_X: u32 = 0xf000_0053;
pub const MASK_FMV_W_X: u32 = 0xfff0_707f;
pub const MATCH_FCVT_D_W: u32 = 0xd200_0053;
pub const MASK_FCVT_D_W: u32 = 0xfff0_007f;
pub const MATCH_FCVT_D_WU: u32 = 0xd210_0053;
pub const MASK_FCVT_D_WU: u32 = 0xfff0_007f;
pub const MATCH_FCVT_D_L: u32 = 0xd220_0053;
pub const MASK_FCVT_D_L: u32 = 0xfff0_007f;
pub const MATCH_FCVT_D_LU: u32 = 0xd230_0053;
pub const MASK_FCVT_D_LU: u32 = 0xfff0_007f;
pub const MATCH_FMV_D_X: u32 = 0xf200_0053;
pub const MASK_FMV_D_X: u32 = 0xfff0_707f;
pub const MATCH_FCVT_Q_W: u32 = 0xd600_0053;
pub const MASK_FCVT_Q_W: u32 = 0xfff0_007f;
pub const MATCH_FCVT_Q_WU: u32 = 0xd610_0053;
pub const MASK_FCVT_Q_WU: u32 = 0xfff0_007f;
pub const MATCH_FCVT_Q_L: u32 = 0xd620_0053;
pub const MASK_FCVT_Q_L: u32 = 0xfff0_007f;
pub const MATCH_FCVT_Q_LU: u32 = 0xd630_0053;
pub const MASK_FCVT_Q_LU: u32 = 0xfff0_007f;
pub const MATCH_FMV_Q_X: u32 = 0xf600_0053;
pub const MASK_FMV_Q_X: u32 = 0xfff0_707f;
pub const MATCH_FLW: u32 = 0x2007;
pub const MASK_FLW: u32 = 0x707f;
pub const MATCH_FLD: u32 = 0x3007;
pub const MASK_FLD: u32 = 0x707f;
pub const MATCH_FLQ: u32 = 0x4007;
pub const MASK_FLQ: u32 = 0x707f;
pub const MATCH_FSW: u32 = 0x2027;
pub const MASK_FSW: u32 = 0x707f;
pub const MATCH_FSD: u32 = 0x3027;
pub const MASK_FSD: u32 = 0x707f;
pub const MATCH_FSQ: u32 = 0x4027;
pub const MASK_FSQ: u32 = 0x707f;
pub const MATCH_FMADD_S: u32 = 0x43;
pub const MASK_FMADD_S: u32 = 0x600_007f;
pub const MATCH_FMSUB_S: u32 = 0x47;
pub const MASK_FMSUB_S: u32 = 0x600_007f;
pub const MATCH_FNMSUB_S: u32 = 0x4b;
pub const MASK_FNMSUB_S: u32 = 0x600_007f;
pub const MATCH_FNMADD_S: u32 = 0x4f;
pub const MASK_FNMADD_S: u32 = 0x600_007f;
pub const MATCH_FMADD_D: u32 = 0x200_0043;
pub const MASK_FMADD_D: u32 = 0x600_007f;
pub const MATCH_FMSUB_D: u32 = 0x200_0047;
pub const MASK_FMSUB_D: u32 = 0x600_007f;
pub const MATCH_FNMSUB_D: u32 = 0x200_004b;
pub const MASK_FNMSUB_D: u32 = 0x600_007f;
pub const MATCH_FNMADD_D: u32 = 0x200_004f;
pub const MASK_FNMADD_D: u32 = 0x600_007f;
pub const MATCH_FMADD_Q: u32 = 0x600_0043;
pub const MASK_FMADD_Q: u32 = 0x600_007f;
pub const MATCH_FMSUB_Q: u32 = 0x600_0047;
pub const MASK_FMSUB_Q: u32 = 0x600_007f;
pub const MATCH_FNMSUB_Q: u32 = 0x600_004b;
pub const MASK_FNMSUB_Q: u32 = 0x600_007f;
pub const MATCH_FNMADD_Q: u32 = 0x600_004f;
pub const MASK_FNMADD_Q: u32 = 0x600_007f;
pub const MATCH_C_NOP: u32 = 0x1;
pub const MASK_C_NOP: u32 = 0xffff;
pub const MATCH_C_ADDI16SP: u32 = 0x6101;
pub const MASK_C_ADDI16SP: u32 = 0xef83;
pub const MATCH_C_JR: u32 = 0x8002;
pub const MASK_C_JR: u32 = 0xf07f;
pub const MATCH_C_JALR: u32 = 0x9002;
pub const MASK_C_JALR: u32 = 0xf07f;
pub const MATCH_C_EBREAK: u32 = 0x9002;
pub const MASK_C_EBREAK: u32 = 0xffff;
pub const MATCH_C_LD: u32 = 0x6000;
pub const MASK_C_LD: u32 = 0xe003;
pub const MATCH_C_SD: u32 = 0xe000;
pub const MASK_C_SD: u32 = 0xe003;
pub const MATCH_C_ADDIW: u32 = 0x2001;
pub const MASK_C_ADDIW: u32 = 0xe003;
pub const MATCH_C_LDSP: u32 = 0x6002;
pub const MASK_C_LDSP: u32 = 0xe003;
pub const MATCH_C_SDSP: u32 = 0xe002;
pub const MASK_C_SDSP: u32 = 0xe003;
pub const MATCH_C_ADDI4SPN: u32 = 0x0;
pub const MASK_C_ADDI4SPN: u32 = 0xe003;
pub const MATCH_C_FLD: u32 = 0x2000;
pub const MASK_C_FLD: u32 = 0xe003;
pub const MATCH_C_LW: u32 = 0x4000;
pub const MASK_C_LW: u32 = 0xe003;
pub const MATCH_C_FLW: u32 = 0x6000;
pub const MASK_C_FLW: u32 = 0xe003;
pub const MATCH_C_FSD: u32 = 0xa000;
pub const MASK_C_FSD: u32 = 0xe003;
pub const MATCH_C_SW: u32 = 0xc000;
pub const MASK_C_SW: u32 = 0xe003;
pub const MATCH_C_FSW: u32 = 0xe000;
pub const MASK_C_FSW: u32 = 0xe003;
pub const MATCH_C_ADDI: u32 = 0x1;
pub const MASK_C_ADDI: u32 = 0xe003;
pub const MATCH_C_JAL: u32 = 0x2001;
pub const MASK_C_JAL: u32 = 0xe003;
pub const MATCH_C_LI: u32 = 0x4001;
pub const MASK_C_LI: u32 = 0xe003;
pub const MATCH_C_LUI: u32 = 0x6001;
pub const MASK_C_LUI: u32 = 0xe003;
pub const MATCH_C_SRLI: u32 = 0x8001;
pub const MASK_C_SRLI: u32 = 0xec03;
pub const MATCH_C_SRAI: u32 = 0x8401;
pub const MASK_C_SRAI: u32 = 0xec03;
pub const MATCH_C_ANDI: u32 = 0x8801;
pub const MASK_C_ANDI: u32 = 0xec03;
pub const MATCH_C_SUB: u32 = 0x8c01;
pub const MASK_C_SUB: u32 = 0xfc63;
pub const MATCH_C_XOR: u32 = 0x8c21;
pub const MASK_C_XOR: u32 = 0xfc63;
pub const MATCH_C_OR: u32 = 0x8c41;
pub const MASK_C_OR: u32 = 0xfc63;
pub const MATCH_C_AND: u32 = 0x8c61;
pub const MASK_C_AND: u32 = 0xfc63;
pub const MATCH_C_SUBW: u32 = 0x9c01;
pub const MASK_C_SUBW: u32 = 0xfc63;
pub const MATCH_C_ADDW: u32 = 0x9c21;
pub const MASK_C_ADDW: u32 = 0xfc63;
pub const MATCH_C_J: u32 = 0xa001;
pub const MASK_C_J: u32 = 0xe003;
pub const MATCH_C_BEQZ: u32 = 0xc001;
pub const MASK_C_BEQZ: u32 = 0xe003;
pub const MATCH_C_BNEZ: u32 = 0xe001;
pub const MASK_C_BNEZ: u32 = 0xe003;
pub const MATCH_C_SLLI: u32 = 0x2;
pub const MASK_C_SLLI: u32 = 0xe003;
pub const MATCH_C_FLDSP: u32 = 0x2002;
pub const MASK_C_FLDSP: u32 = 0xe003;
pub const MATCH_C_LWSP: u32 = 0x4002;
pub const MASK_C_LWSP: u32 = 0xe003;
pub const MATCH_C_FLWSP: u32 = 0x6002;
pub const MASK_C_FLWSP: u32 = 0xe003;
pub const MATCH_C_MV: u32 = 0x8002;
pub const MASK_C_MV: u32 = 0xf003;
pub const MATCH_C_ADD: u32 = 0x9002;
pub const MASK_C_ADD: u32 = 0xf003;
pub const MATCH_C_FSDSP: u32 = 0xa002;
pub const MASK_C_FSDSP: u32 = 0xe003;
pub const MATCH_C_SWSP: u32 = 0xc002;
pub const MASK_C_SWSP: u32 = 0xe003;
pub const MATCH_C_FSWSP: u32 = 0xe002;
pub const MASK_C_FSWSP: u32 = 0xe003;
pub const MATCH_CUSTOM0: u32 = 0xb;
pub const MASK_CUSTOM0: u32 = 0x707f;
pub const MATCH_CUSTOM0_RS1: u32 = 0x200b;
pub const MASK_CUSTOM0_RS1: u32 = 0x707f;
pub const MATCH_CUSTOM0_RS1_RS2: u32 = 0x300b;
pub const MASK_CUSTOM0_RS1_RS2: u32 = 0x707f;
pub const MATCH_CUSTOM0_RD: u32 = 0x400b;
pub const MASK_CUSTOM0_RD: u32 = 0x707f;
pub const MATCH_CUSTOM0_RD_RS1: u32 = 0x600b;
pub const MASK_CUSTOM0_RD_RS1: u32 = 0x707f;
pub const MATCH_CUSTOM0_RD_RS1_RS2: u32 = 0x700b;
pub const MASK_CUSTOM0_RD_RS1_RS2: u32 = 0x707f;
pub const MATCH_CUSTOM1: u32 = 0x2b;
pub const MASK_CUSTOM1: u32 = 0x707f;
pub const MATCH_CUSTOM1_RS1: u32 = 0x202b;
pub const MASK_CUSTOM1_RS1: u32 = 0x707f;
pub const MATCH_CUSTOM1_RS1_RS2: u32 = 0x302b;
pub const MASK_CUSTOM1_RS1_RS2: u32 = 0x707f;
pub const MATCH_CUSTOM1_RD: u32 = 0x402b;
pub const MASK_CUSTOM1_RD: u32 = 0x707f;
pub const MATCH_CUSTOM1_RD_RS1: u32 = 0x602b;
pub const MASK_CUSTOM1_RD_RS1: u32 = 0x707f;
pub const MATCH_CUSTOM1_RD_RS1_RS2: u32 = 0x702b;
pub const MASK_CUSTOM1_RD_RS1_RS2: u32 = 0x707f;
pub const MATCH_CUSTOM2: u32 = 0x5b;
pub const MASK_CUSTOM2: u32 = 0x707f;
pub const MATCH_CUSTOM2_RS1: u32 = 0x205b;
pub const MASK_CUSTOM2_RS1: u32 = 0x707f;
pub const MATCH_CUSTOM2_RS1_RS2: u32 = 0x305b;
pub const MASK_CUSTOM2_RS1_RS2: u32 = 0x707f;
pub const MATCH_CUSTOM2_RD: u32 = 0x405b;
pub const MASK_CUSTOM2_RD: u32 = 0x707f;
pub const MATCH_CUSTOM2_RD_RS1: u32 = 0x605b;
pub const MASK_CUSTOM2_RD_RS1: u32 = 0x707f;
pub const MATCH_CUSTOM2_RD_RS1_RS2: u32 = 0x705b;
pub const MASK_CUSTOM2_RD_RS1_RS2: u32 = 0x707f;
pub const MATCH_CUSTOM3: u32 = 0x7b;
pub const MASK_CUSTOM3: u32 = 0x707f;
pub const MATCH_CUSTOM3_RS1: u32 = 0x207b;
pub const MASK_CUSTOM3_RS1: u32 = 0x707f;
pub const MATCH_CUSTOM3_RS1_RS2: u32 = 0x307b;
pub const MASK_CUSTOM3_RS1_RS2: u32 = 0x707f;
pub const MATCH_CUSTOM3_RD: u32 = 0x407b;
pub const MASK_CUSTOM3_RD: u32 = 0x707f;
pub const MATCH_CUSTOM3_RD_RS1: u32 = 0x607b;
pub const MASK_CUSTOM3_RD_RS1: u32 = 0x707f;
pub const MATCH_CUSTOM3_RD_RS1_RS2: u32 = 0x707b;
pub const MASK_CUSTOM3_RD_RS1_RS2: u32 = 0x707f;
pub const CSR_FFLAGS: u32 = 0x1;
pub const CSR_FRM: u32 = 0x2;
pub const CSR_FCSR: u32 = 0x3;
pub const CSR_CYCLE: u32 = 0xc00;
pub const CSR_TIME: u32 = 0xc01;
pub const CSR_INSTRET: u32 = 0xc02;
pub const CSR_HPMCOUNTER3: u32 = 0xc03;
pub const CSR_HPMCOUNTER4: u32 = 0xc04;
pub const CSR_HPMCOUNTER5: u32 = 0xc05;
pub const CSR_HPMCOUNTER6: u32 = 0xc06;
pub const CSR_HPMCOUNTER7: u32 = 0xc07;
pub const CSR_HPMCOUNTER8: u32 = 0xc08;
pub const CSR_HPMCOUNTER9: u32 = 0xc09;
pub const CSR_HPMCOUNTER10: u32 = 0xc0a;
pub const CSR_HPMCOUNTER11: u32 = 0xc0b;
pub const CSR_HPMCOUNTER12: u32 = 0xc0c;
pub const CSR_HPMCOUNTER13: u32 = 0xc0d;
pub const CSR_HPMCOUNTER14: u32 = 0xc0e;
pub const CSR_HPMCOUNTER15: u32 = 0xc0f;
pub const CSR_HPMCOUNTER16: u32 = 0xc10;
pub const CSR_HPMCOUNTER17: u32 = 0xc11;
pub const CSR_HPMCOUNTER18: u32 = 0xc12;
pub const CSR_HPMCOUNTER19: u32 = 0xc13;
pub const CSR_HPMCOUNTER20: u32 = 0xc14;
pub const CSR_HPMCOUNTER21: u32 = 0xc15;
pub const CSR_HPMCOUNTER22: u32 = 0xc16;
pub const CSR_HPMCOUNTER23: u32 = 0xc17;
pub const CSR_HPMCOUNTER24: u32 = 0xc18;
pub const CSR_HPMCOUNTER25: u32 = 0xc19;
pub const CSR_HPMCOUNTER26: u32 = 0xc1a;
pub const CSR_HPMCOUNTER27: u32 = 0xc1b;
pub const CSR_HPMCOUNTER28: u32 = 0xc1c;
pub const CSR_HPMCOUNTER29: u32 = 0xc1d;
pub const CSR_HPMCOUNTER30: u32 = 0xc1e;
pub const CSR_HPMCOUNTER31: u32 = 0xc1f;
pub const CSR_SSTATUS: u32 = 0x100;
pub const CSR_SIE: u32 = 0x104;
pub const CSR_STVEC: u32 = 0x105;
pub const CSR_SCOUNTEREN: u32 = 0x106;
pub const CSR_SSCRATCH: u32 = 0x140;
pub const CSR_SEPC: u32 = 0x141;
pub const CSR_SCAUSE: u32 = 0x142;
pub const CSR_STVAL: u32 = 0x143;
pub const CSR_SIP: u32 = 0x144;
pub const CSR_SATP: u32 = 0x180;
pub const CSR_MSTATUS: u32 = 0x300;
pub const CSR_MISA: u32 = 0x301;
pub const CSR_MEDELEG: u32 = 0x302;
pub const CSR_MIDELEG: u32 = 0x303;
pub const CSR_MIE: u32 = 0x304;
pub const CSR_MTVEC: u32 = 0x305;
pub const CSR_MCOUNTEREN: u32 = 0x306;
pub const CSR_MSCRATCH: u32 = 0x340;
pub const CSR_MEPC: u32 = 0x341;
pub const CSR_MCAUSE: u32 = 0x342;
pub const CSR_MTVAL: u32 = 0x343;
pub const CSR_MIP: u32 = 0x344;
pub const CSR_PMPCFG0: u32 = 0x3a0;
pub const CSR_PMPCFG1: u32 = 0x3a1;
pub const CSR_PMPCFG2: u32 = 0x3a2;
pub const CSR_PMPCFG3: u32 = 0x3a3;
pub const CSR_PMPADDR0: u32 = 0x3b0;
pub const CSR_PMPADDR1: u32 = 0x3b1;
pub const CSR_PMPADDR2: u32 = 0x3b2;
pub const CSR_PMPADDR3: u32 = 0x3b3;
pub const CSR_PMPADDR4: u32 = 0x3b4;
pub const CSR_PMPADDR5: u32 = 0x3b5;
pub const CSR_PMPADDR6: u32 = 0x3b6;
pub const CSR_PMPADDR7: u32 = 0x3b7;
pub const CSR_PMPADDR8: u32 = 0x3b8;
pub const CSR_PMPADDR9: u32 = 0x3b9;
pub const CSR_PMPADDR10: u32 = 0x3ba;
pub const CSR_PMPADDR11: u32 = 0x3bb;
pub const CSR_PMPADDR12: u32 = 0x3bc;
pub const CSR_PMPADDR13: u32 = 0x3bd;
pub const CSR_PMPADDR14: u32 = 0x3be;
pub const CSR_PMPADDR15: u32 = 0x3bf;
pub const CSR_TSELECT: u32 = 0x7a0;
pub const CSR_TDATA1: u32 = 0x7a1;
pub const CSR_TDATA2: u32 = 0x7a2;
pub const CSR_TDATA3: u32 = 0x7a3;
pub const CSR_DCSR: u32 = 0x7b0;
pub const CSR_DPC: u32 = 0x7b1;
pub const CSR_DSCRATCH: u32 = 0x7b2;
pub const CSR_MCYCLE: u32 = 0xb00;
pub const CSR_MINSTRET: u32 = 0xb02;
pub const CSR_MHPMCOUNTER3: u32 = 0xb03;
pub const CSR_MHPMCOUNTER4: u32 = 0xb04;
pub const CSR_MHPMCOUNTER5: u32 = 0xb05;
pub const CSR_MHPMCOUNTER6: u32 = 0xb06;
pub const CSR_MHPMCOUNTER7: u32 = 0xb07;
pub const CSR_MHPMCOUNTER8: u32 = 0xb08;
pub const CSR_MHPMCOUNTER9: u32 = 0xb09;
pub const CSR_MHPMCOUNTER10: u32 = 0xb0a;
pub const CSR_MHPMCOUNTER11: u32 = 0xb0b;
pub const CSR_MHPMCOUNTER12: u32 = 0xb0c;
pub const CSR_MHPMCOUNTER13: u32 = 0xb0d;
pub const CSR_MHPMCOUNTER14: u32 = 0xb0e;
pub const CSR_MHPMCOUNTER15: u32 = 0xb0f;
pub const CSR_MHPMCOUNTER16: u32 = 0xb10;
pub const CSR_MHPMCOUNTER17: u32 = 0xb11;
pub const CSR_MHPMCOUNTER18: u32 = 0xb12;
pub const CSR_MHPMCOUNTER19: u32 = 0xb13;
pub const CSR_MHPMCOUNTER20: u32 = 0xb14;
pub const CSR_MHPMCOUNTER21: u32 = 0xb15;
pub const CSR_MHPMCOUNTER22: u32 = 0xb16;
pub const CSR_MHPMCOUNTER23: u32 = 0xb17;
pub const CSR_MHPMCOUNTER24: u32 = 0xb18;
pub const CSR_MHPMCOUNTER25: u32 = 0xb19;
pub const CSR_MHPMCOUNTER26: u32 = 0xb1a;
pub const CSR_MHPMCOUNTER27: u32 = 0xb1b;
pub const CSR_MHPMCOUNTER28: u32 = 0xb1c;
pub const CSR_MHPMCOUNTER29: u32 = 0xb1d;
pub const CSR_MHPMCOUNTER30: u32 = 0xb1e;
pub const CSR_MHPMCOUNTER31: u32 = 0xb1f;
pub const CSR_MHPMEVENT3: u32 = 0x323;
pub const CSR_MHPMEVENT4: u32 = 0x324;
pub const CSR_MHPMEVENT5: u32 = 0x325;
pub const CSR_MHPMEVENT6: u32 = 0x326;
pub const CSR_MHPMEVENT7: u32 = 0x327;
pub const CSR_MHPMEVENT8: u32 = 0x328;
pub const CSR_MHPMEVENT9: u32 = 0x329;
pub const CSR_MHPMEVENT10: u32 = 0x32a;
pub const CSR_MHPMEVENT11: u32 = 0x32b;
pub const CSR_MHPMEVENT12: u32 = 0x32c;
pub const CSR_MHPMEVENT13: u32 = 0x32d;
pub const CSR_MHPMEVENT14: u32 = 0x32e;
pub const CSR_MHPMEVENT15: u32 = 0x32f;
pub const CSR_MHPMEVENT16: u32 = 0x330;
pub const CSR_MHPMEVENT17: u32 = 0x331;
pub const CSR_MHPMEVENT18: u32 = 0x332;
pub const CSR_MHPMEVENT19: u32 = 0x333;
pub const CSR_MHPMEVENT20: u32 = 0x334;
pub const CSR_MHPMEVENT21: u32 = 0x335;
pub const CSR_MHPMEVENT22: u32 = 0x336;
pub const CSR_MHPMEVENT23: u32 = 0x337;
pub const CSR_MHPMEVENT24: u32 = 0x338;
pub const CSR_MHPMEVENT25: u32 = 0x339;
pub const CSR_MHPMEVENT26: u32 = 0x33a;
pub const CSR_MHPMEVENT27: u32 = 0x33b;
pub const CSR_MHPMEVENT28: u32 = 0x33c;
pub const CSR_MHPMEVENT29: u32 = 0x33d;
pub const CSR_MHPMEVENT30: u32 = 0x33e;
pub const CSR_MHPMEVENT31: u32 = 0x33f;
pub const CSR_MVENDORID: u32 = 0xf11;
pub const CSR_MARCHID: u32 = 0xf12;
pub const CSR_MIMPID: u32 = 0xf13;
pub const CSR_MHARTID: u32 = 0xf14;
pub const CSR_CYCLEH: u32 = 0xc80;
pub const CSR_TIMEH: u32 = 0xc81;
pub const CSR_INSTRETH: u32 = 0xc82;
pub const CSR_HPMCOUNTER3H: u32 = 0xc83;
pub const CSR_HPMCOUNTER4H: u32 = 0xc84;
pub const CSR_HPMCOUNTER5H: u32 = 0xc85;
pub const CSR_HPMCOUNTER6H: u32 = 0xc86;
pub const CSR_HPMCOUNTER7H: u32 = 0xc87;
pub const CSR_HPMCOUNTER8H: u32 = 0xc88;
pub const CSR_HPMCOUNTER9H: u32 = 0xc89;
pub const CSR_HPMCOUNTER10H: u32 = 0xc8a;
pub const CSR_HPMCOUNTER11H: u32 = 0xc8b;
pub const CSR_HPMCOUNTER12H: u32 = 0xc8c;
pub const CSR_HPMCOUNTER13H: u32 = 0xc8d;
pub const CSR_HPMCOUNTER14H: u32 = 0xc8e;
pub const CSR_HPMCOUNTER15H: u32 = 0xc8f;
pub const CSR_HPMCOUNTER16H: u32 = 0xc90;
pub const CSR_HPMCOUNTER17H: u32 = 0xc91;
pub const CSR_HPMCOUNTER18H: u32 = 0xc92;
pub const CSR_HPMCOUNTER19H: u32 = 0xc93;
pub const CSR_HPMCOUNTER20H: u32 = 0xc94;
pub const CSR_HPMCOUNTER21H: u32 = 0xc95;
pub const CSR_HPMCOUNTER22H: u32 = 0xc96;
pub const CSR_HPMCOUNTER23H: u32 = 0xc97;
pub const CSR_HPMCOUNTER24H: u32 = 0xc98;
pub const CSR_HPMCOUNTER25H: u32 = 0xc99;
pub const CSR_HPMCOUNTER26H: u32 = 0xc9a;
pub const CSR_HPMCOUNTER27H: u32 = 0xc9b;
pub const CSR_HPMCOUNTER28H: u32 = 0xc9c;
pub const CSR_HPMCOUNTER29H: u32 = 0xc9d;
pub const CSR_HPMCOUNTER30H: u32 = 0xc9e;
pub const CSR_HPMCOUNTER31H: u32 = 0xc9f;
pub const CSR_MCYCLEH: u32 = 0xb80;
pub const CSR_MINSTRETH: u32 = 0xb82;
pub const CSR_MHPMCOUNTER3H: u32 = 0xb83;
pub const CSR_MHPMCOUNTER4H: u32 = 0xb84;
pub const CSR_MHPMCOUNTER5H: u32 = 0xb85;
pub const CSR_MHPMCOUNTER6H: u32 = 0xb86;
pub const CSR_MHPMCOUNTER7H: u32 = 0xb87;
pub const CSR_MHPMCOUNTER8H: u32 = 0xb88;
pub const CSR_MHPMCOUNTER9H: u32 = 0xb89;
pub const CSR_MHPMCOUNTER10H: u32 = 0xb8a;
pub const CSR_MHPMCOUNTER11H: u32 = 0xb8b;
pub const CSR_MHPMCOUNTER12H: u32 = 0xb8c;
pub const CSR_MHPMCOUNTER13H: u32 = 0xb8d;
pub const CSR_MHPMCOUNTER14H: u32 = 0xb8e;
pub const CSR_MHPMCOUNTER15H: u32 = 0xb8f;
pub const CSR_MHPMCOUNTER16H: u32 = 0xb90;
pub const CSR_MHPMCOUNTER17H: u32 = 0xb91;
pub const CSR_MHPMCOUNTER18H: u32 = 0xb92;
pub const CSR_MHPMCOUNTER19H: u32 = 0xb93;
pub const CSR_MHPMCOUNTER20H: u32 = 0xb94;
pub const CSR_MHPMCOUNTER21H: u32 = 0xb95;
pub const CSR_MHPMCOUNTER22H: u32 = 0xb96;
pub const CSR_MHPMCOUNTER23H: u32 = 0xb97;
pub const CSR_MHPMCOUNTER24H: u32 = 0xb98;
pub const CSR_MHPMCOUNTER25H: u32 = 0xb99;
pub const CSR_MHPMCOUNTER26H: u32 = 0xb9a;
pub const CSR_MHPMCOUNTER27H: u32 = 0xb9b;
pub const CSR_MHPMCOUNTER28H: u32 = 0xb9c;
pub const CSR_MHPMCOUNTER29H: u32 = 0xb9d;
pub const CSR_MHPMCOUNTER30H: u32 = 0xb9e;
pub const CSR_MHPMCOUNTER31H: u32 = 0xb9f;
pub const CAUSE_MISALIGNED_FETCH: u32 = 0x0;
pub const CAUSE_FETCH_ACCESS: u32 = 0x1;
pub const CAUSE_ILLEGAL_INSTRUCTION: u32 = 0x2;
pub const CAUSE_BREAKPOINT: u32 = 0x3;
pub const CAUSE_MISALIGNED_LOAD: u32 = 0x4;
pub const CAUSE_LOAD_ACCESS: u32 = 0x5;
pub const CAUSE_MISALIGNED_STORE: u32 = 0x6;
pub const CAUSE_STORE_ACCESS: u32 = 0x7;
pub const CAUSE_USER_ECALL: u32 = 0x8;
pub const CAUSE_SUPERVISOR_ECALL: u32 = 0x9;
pub const CAUSE_HYPERVISOR_ECALL: u32 = 0xa;
pub const CAUSE_MACHINE_ECALL: u32 = 0xb;
pub const CAUSE_FETCH_PAGE_FAULT: u32 = 0xc;
pub const CAUSE_LOAD_PAGE_FAULT: u32 = 0xd;
pub const CAUSE_STORE_PAGE_FAULT: u32 = 0xf;
// Special ParaNut cause
pub const CAUSE_COPU_EXCEPTION: u32 = 0x10;
// Special ParaNut CSRs
pub const CSR_PNCACHE: u32 = 0x7C0;
pub const CSR_PNGRPSEL: u32 = 0x8C0;
pub const CSR_PNCE: u32 = 0x8C1;
pub const CSR_PNLM: u32 = 0x8C2;
pub const CSR_PNXSEL: u32 = 0x8C3;
pub const CSR_PNM2CP: u32 = 0xFC0;
pub const CSR_PNX: u32 = 0xFC1;
pub const CSR_PNCAUSE: u32 = 0xFC2;
pub const CSR_PNEPC: u32 = 0xFC3;
pub const CSR_PNCACHEINFO: u32 = 0xFC4;
pub const CSR_PNCACHESETS: u32 = 0xFC5;
pub const CSR_PNCLOCKINFO: u32 = 0xFC6;
pub const CSR_PNMEMSIZE: u32 = 0xFC7;
pub const CSR_PNCPUS: u32 = 0xCD0;
pub const CSR_PNCOREID: u32 = 0xCD4;
