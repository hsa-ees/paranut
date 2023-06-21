#  -----------------------------------------------------------------------------
#
#  This file is part of the ParaNut project.
#
#  Copyright (C) 2019-2021 Alexander Bahle <alexander.bahle@hs-augsburg.de>
#                          Michael Schaeferling <michael.schaeferling@hs-augsburg.de>
#      Hochschule Augsburg, University of Applied Sciences
#
#  Description:
#    This file contains the global configuration options for the ParaNut.
#
#  --------------------- LICENSE -----------------------------------------------
#  Redistribution and use in source and binary forms, with or without modification,
#  are permitted provided that the following conditions are met:
#
#  1. Redistributions of source code must retain the above copyright notice, this
#     list of conditions and the following disclaimer.
#
#  2. Redistributions in binary form must reproduce the above copyright notice,
#     this list of conditions and the following disclaimer in the documentation and/or
#     other materials provided with the distribution.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
#  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
#  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
#  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
#  ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
#  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
#  ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
#  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
#  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#  -----------------------------------------------------------------------------


## Do not edit
KB = 1024
MB = 1024*1024


# **************** SystemC Simulation options ***********************
# Simulation clock speed in Hz.
# Defines the simulation clock speed in Hz. The configured value can be read from the
# pnclockinfo CSR.
CFG_NUT_SIM_CLK_SPEED ?= 25000000

# timebase in us.
# Defines the mtime timebase in us
CFG_NUT_MTIMER_TIMEBASE_US 	?=  1000

# mtimer base address.
# Defines the address at which the mtimer will be added to the system interconnect
CFG_NUT_MTIMER_ADDR 		?=  0x80000000

# Simulation memory address.
# Defines the start address (reset address) of the ParaNut during simulation and
# the address at which the main memory will be added to the system interconnect.
# Also used to determine the cacheable memory addresses.
CFG_NUT_RESET_ADDR ?= 0x10000000

# Simulation maximum peripherals number.
# Defines the maximum number of peripherals that can be connected to the systems Wishbone
# interconnect.
CFG_NUT_SIM_MAX_PERIPHERY ?= 5


# **************** General options ***********************
# Number of cores overall as log2.
# Defines the log2 of the overall number of ParaNut cores (ExUs).
CFG_NUT_CPU_CORES_LD ?= 1

# Number of cores (ExUs) with mode capability = 1 (linked).
# Defines the number of ParaNut cores (ExUs) that have a mode capability of 1 and thus can only
# operate in mode 1 (linked mode).
CFG_NUT_CPU_CAP1_CORES ?= 0

# System memory size.
# Defines the system memory size in Bytes. 8MB is recommended for simulation, otherwise
# needs to match the memory size of the used board.
# Also used to determine the cacheable memory addresses.
CFG_NUT_MEM_SIZE ?= (256 * MB)

# Number of external interrupt lines.
# Defines the number of external interrupt lines. Needs to be at least equal to 1.
CFG_NUT_EX_INT ?= 2


# **************** EXU ***********************
# RISC-V ISA Extensions
# Defines if the RISC-V Extension hardware is enabled/disabled.
# 0 - extension disabled
# 1 - extension enabled
CFG_EXU_M_EXTENSION ?= 1
CFG_EXU_A_EXTENSION ?= 1

# Privilege Mode options.
# Defines the RISC-V privilege mode capability of the ParaNut. Do NOT use any other value!
# 1 - only M-Mode is available
# 2 - M- and U-Mode are available
# 3 - M-, S- and U-Mode available, enable the Memory Management Unit (MMU)
CFG_PRIV_LEVELS ?= 3

# Performance counter enable.
# Defines if the hardware performance counters are enabled/disabled.
# 0 - no performance counters
# 1 - performance counter and 64bit cycle counter is added
CFG_EXU_PERFCOUNT_ENABLE ?= 1

# Performance counter register width.
# Defines the number of bits for the hardware performance counters.
# Warning: Has to be between 33 and 64.
CFG_EXU_PERFCOUNTER_BITS ?= 40

# Performance counter number of registers as log2.
# Defines the log2 of the number of hardware performance counters.
# Warning: 3 is the only supported value for now
CFG_EXU_PERFCOUNTERS_LD ?= 3


# ********************* MemU *****************************
# Number of cache banks as log2.
# Defines the log2 of the number of cache banks.
# A cache line has a size of 2^CFG_MEMU_CACHE_BANKS_LD words. A good starting point is 2 (4 banks).
CFG_MEMU_CACHE_BANKS_LD ?= 2

# Number of cache sets as log2.
# Defines the log2 of the number of cache sets.
# A bank has 2^CFG_MEMU_CACHE_SETS_LD * 2^CFG_MEMU_CACHE_WAYS_LD words.
CFG_MEMU_CACHE_SETS_LD ?= 9

# Number of cache ways as log2.
# Defines the log2 of the number of cache ways (cache associativity).
# A bank has 2^CFG_MEMU_CACHE_SETS_LD * 2^CFG_MEMU_CACHE_WAYS_LD words.
# 0 - 1-way set-associativity
# 1 - 2-way set-associativity
# 2 - 4-way set-associativity
CFG_MEMU_CACHE_WAYS_LD ?= 2

# Cache replacement method.
# Defines the cache replacement method/strategy, either pseudo-random or least recently used (LRU).
# 0 - random replacement
# 1 - LRU replacement
CFG_MEMU_CACHE_REPLACE_LRU ?= 1

# Arbiter Method.
# Defines the MemU arbitration method/strategy, either a round-robin or pseudo-random arbitration.
# >0 - round-robin arbitration, switches every (1 << CFG_MEMU_ARBITER_METHOD) clocks
# <0 - pseudo-random arbitration (LFSR-based)
CFG_MEMU_ARBITER_METHOD ?= 7

# Busif Data Width.
# Defines the width of the master Wishbone data bus.
# 32 - 32 Bit data width
# 64 - 64 Bit data width
CFG_MEMU_BUSIF_WIDTH ?= 32


# ********************* MMU *****************************

# Enable or disable the Translation Lookaside Buffer (TLB)
# 1 - TLB enabled
CFG_MMU_TLB_ENABLE ?= 0

# Number of TLB entries as log2. 
# Defines the log2 of the number of TLB entries.
# Must be at least 1
CFG_MMU_TLB_ENTRIES_LD ?= 2


# ********************* Ifu *****************************
# Instruction buffer size as log2.
# Defines the log2 of the number of instruction buffer elements (max. number of prefetched
# instructions).
CFG_IFU_IBUF_SIZE_LD ?= 2


# ********************* Lsu *****************************
# LSU write buffer size as log2.
# Defines the log2 of the number of write buffer elements in the LSU.
CFG_LSU_WBUF_SIZE_LD ?= 2
