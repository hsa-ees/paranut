/*************************************************************************

  This file is part of the ParaNut project.

  Copyright (C) 2019-2020 Alexander Bahle <alexander.bahle@hs-augsburg.de>
      Hochschule Augsburg, University of Applied Sciences

  Description:
    This file contains the configuration options for the ParaNut.

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

#ifndef _CONFIG_
#define _CONFIG_

// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
// Warning: 	Don't change this file manually! 
// 				Use the "../config" file located 
//				at the root of the project
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

// **************** SystemC Simulation options ***********************
// Simulation Clock Speed in Hz
#define CFG_NUT_SIM_CLK_SPEED {CFG_NUT_SIM_CLK_SPEED} 	

// Simulation Memory Address
#define CFG_NUT_SIM_MEM_ADDR {CFG_NUT_SIM_MEM_ADDR} 	

// **************** General options ***********************

// Overall number of cores
#define CFG_NUT_CPU_CORES_LD {CFG_NUT_CPU_CORES_LD}
#define CFG_NUT_CPU_CORES (1 << CFG_NUT_CPU_CORES_LD)
#define CFG_NUT_CPU_GROUPS ((CFG_NUT_CPU_CORES-1)/32)

// Number of cores with mode capability 1
#define CFG_NUT_CPU_MODE1_CORES {CFG_NUT_CPU_MODE1_CORES}

// Number of cores with mode capability >= 2
#define CFG_NUT_CPU_MODE2_CORES (CFG_NUT_CPU_CORES - CFG_NUT_CPU_MODE1_CORES)

// ParaNut Options: (see handbook for description)
#define CFG_EXU_PNM2CAP ~(0xffffffff << CFG_NUT_CPU_MODE2_CORES)

// Memory size (needs (8*MB) recommended for simulation, otherwise needs to match the memory size of the used board):
#define CFG_NUT_MEM_SIZE {CFG_NUT_MEM_SIZE}

// Number of external interrupt lines (needs to be >= 1) 
#define CFG_NUT_EX_INT {CFG_NUT_EX_INT}

static inline bool AdrIsCached (TWord adr) { return ( adr ^ CFG_NUT_SIM_MEM_ADDR) < CFG_NUT_MEM_SIZE;  }
static inline bool AdrIsSpecial (TWord adr) { return (adr & 0xf0000000) == 0x90000000; }    // These are I/O adresses

// **************** EXU ***********************

// RISC-V ISA Extensions
// 0 - extension disabled
// 1 - extension enabled
#define CFG_EXU_M_EXTENSION {CFG_EXU_M_EXTENSION}
#define CFG_EXU_A_EXTENSION {CFG_EXU_A_EXTENSION}


// Performance counter options:
// CFG_EXU_PERFCOUNT_ENABLE  -  0 no performance counters are used (reads to counter CSRs are fixed to 0)
//                          1 64bit cycle counter is added
// CFG_EXU_PERFCOUNTER_BITS  -  33 up to 64 bit supported
// CFG_EXU_PERFCOUNTERS_LD   -  3 is the only supported value for now (see exu.cpp for implemented counters)
#define CFG_EXU_PERFCOUNT_ENABLE {CFG_EXU_PERFCOUNT_ENABLE}
#define CFG_EXU_PERFCOUNTER_BITS {CFG_EXU_PERFCOUNTER_BITS}
#define CFG_EXU_PERFCOUNTERS_LD {CFG_EXU_PERFCOUNTERS_LD}
#define CFG_EXU_PERFCOUNTERS (1 << CFG_EXU_PERFCOUNTERS_LD)


// ********************* MemU *****************************
// Number of cache banks
// a cache line has a size of CFG_MEMU_CACHE_BANKS words
#define CFG_MEMU_CACHE_BANKS_LD {CFG_MEMU_CACHE_BANKS_LD}
#define CFG_MEMU_CACHE_BANKS (1 << CFG_MEMU_CACHE_BANKS_LD)

// Number of cache sets
#define CFG_MEMU_CACHE_SETS_LD {CFG_MEMU_CACHE_SETS_LD}   
#define CFG_MEMU_CACHE_SETS (1 << CFG_MEMU_CACHE_SETS_LD)

// Number of cache ways
// associativity; supported values are 0..2, corresponding to 1/2/4-way set-associativity
#define CFG_MEMU_CACHE_WAYS_LD {CFG_MEMU_CACHE_WAYS_LD}      
#define CFG_MEMU_CACHE_WAYS (1 << CFG_MEMU_CACHE_WAYS_LD)

// Cache replacement method
// 0 - random replacement
// 1 - LRU replacement
#define CFG_MEMU_CACHE_REPLACE_LRU {CFG_MEMU_CACHE_REPLACE_LRU}   

// Arbiter Method
// > 0: round-robin arbitration, switches every (1 << CFG_MEMU_ARBITER_METHOD) clocks
// < 0: pseudo-random arbitration (LFSR-based)
#define CFG_MEMU_ARBITER_METHOD {CFG_MEMU_ARBITER_METHOD}   

// Busif Data Width
// 64 - 64 Bit data width
// 32 - 32 Bit data width
#define CFG_MEMU_BUSIF_WIDTH {CFG_MEMU_BUSIF_WIDTH}   


// ********************* Ifu *****************************
// IFU buffer size 
#define CFG_IFU_IBUF_SIZE_LD	{CFG_IFU_IBUF_SIZE_LD}	// number of Ifu buffer elements
#define CFG_IFU_IBUF_SIZE (1 << CFG_IFU_IBUF_SIZE_LD)


// ********************* Lsu *****************************
// LSU write buffer size
#define CFG_LSU_WBUF_SIZE_LD {CFG_LSU_WBUF_SIZE_LD}
#define CFG_LSU_WBUF_SIZE (1 << CFG_LSU_WBUF_SIZE_LD)


// ***** auto-generated *****


#define WPORTS CFG_NUT_CPU_CORES
#define RPORTS (2 * CFG_NUT_CPU_CORES)

#define CACHE_SIZE (CFG_MEMU_CACHE_SETS * CFG_MEMU_CACHE_WAYS * CFG_MEMU_CACHE_BANKS * 4)



#endif

