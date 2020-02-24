/*Header***********************************************************************/
/*************************************************************************

  This file is part of the ParaNut project.

  Copyright (C) 2019 Anna Pfuetzner <anna.pfuetzner@hs-augsburg.de>
                     Alexander Bahle <alexander.bahle@hs-augsburg.de>
      Hochschule Augsburg, University of Applied Sciences

  Description:
    This file contains the startup code (bootrom) for a ParaNut 
    processor. It is usable with software compiled with or without
    the libparanut (see example applications). 
    Gets the ParaNut processor into a known state, invalidates
    the cache, sets up a default trap handler and afterwards jumps into 
    _init function.

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
 * Assign some labels to memory areas to make them more easily accessible.
 */

.section ".tdata.begin"
.align   2                             /* align _tdata_begin to 2^2 bytes     */
_tdata_begin:

.section ".tdata.end"
.align   2                             /* align _tdata_end to 2^2 bytes       */
_tdata_end:

.section ".tbss.end"
.align   2                             /* align _tbss_end to 2^2 bytes        */
_tbss_end:
  
.section ".tohost","aw",@progbits
.align   4                             /* align tohost to 2^4 bytes           */
tohost:
   .word 0
fromhost:
   .word 0

.section ".pntdata","aw",@progbits
.align   4                             /* align _thread_data to 2^4 bytes     */
_thread_data:
   .zero 32*4*2                        /* thread space for 32 CPUs (256 Byte) */
   
shared_mem_start:
   .zero 4                             /* adress of shared memory start       */
shared_mem_size:
   .zero 4                             /* shared memory size in byte          */

error_string:                          /* prepare error message               */
   .string "There was an error in the startup code. Contact maintainers!\n\n"
   
   
/* declare labels in here to be global */
.globl   _tdata_begin                  /* label for area .tdata.begin         */
.globl   _tdata_end                    /* label for area .tdata.end           */
.globl   _tbss_end                     /* label for area .tbss.end            */
.globl   tohost                        /* label for area .tohost              */
.globl   fromhost                      /* label for area .tohost + 4 byte     */
.globl   _thread_data                  /* label for area .tdata.begin         */
.globl   shared_mem_start              /* adress of shared memory start       */
.globl   shared_mem_size               /* shared memory size in byte          */


/*
 * Put the startup code in the correct section
 */

.section ".text.init"                  /* enter .text.init section            */
.align   2                             /* align code to 2^2 bytes             */

/* declare the startup code as global */
.globl   _start                        /* startup routine                     */

/*Functions********************************************************************/

_start:
   
  /*
   * Set all registers to 0 (except x0, of course, which is hardwired 0).
   */
   
   li    x1,      0
   li    x2,      0
   li    x3,      0
   li    x4,      0  
   li    x5,      0
   li    x6,      0
   li    x7,      0  
   li    x8,      0
   li    x9,      0
   li    x10,     0
   li    x11,     0
   li    x12,     0
   li    x13,     0
   li    x14,     0
   li    x15,     0
   li    x16,     0
   li    x17,     0 
   li    x18,     0
   li    x19,     0
   li    x20,     0
   li    x21,     0
   li    x22,     0
   li    x23,     0
   li    x24,     0
   li    x25,     0
   li    x26,     0
   li    x27,     0
   li    x28,     0
   li    x29,     0
   li    x30,     0
   li    x31,     0

  /*
   * Only CePU needs to initialize exception module.
   * If you are not using libparanut exception module, it is assumed that you
   * are using trap_entry instead, which calls into the sycalls.c in
   * sw/riscv_common provides. If that is not the case, hang in your own stuff
   * here. It was mainly created for compatibility of this startup code with the
   * syscalls.c implementation.
   */
   
   csrr  a0,      mhartid              /* read core ID                        */
   bnez  a0,      init_gp              /* other CPUs can immediately load gp  */
   
   li    t0,      0                    /* set t0 to 0                         */
   csrw  mstatus, t0                   /* disable interrupts                  */
   la    t0,      trap_entry           /* use trap_entry (see below)          */
   csrw  mtvec,   t0                   /* set trap handler base adress        */
   
   
init_gp:

  /*
   * The global pointer is supposed to be an absolute location for all cores
   * to orient themselves on. We cannot let the linker throw it around at will
   * for the sake of optimizing away an auipc+jalr!
   * This means we are using the option .norelax whenever we get the gp from
   * __global_pointer$.
   */
   
   .option push
   .option norelax
   la gp, __global_pointer$            /* set global pointer                  */
   .option pop
   
   
init_tp:
  
  /*
   * Iniialize thread pointer to after end of uninitialized segment.
   * Using the AND with -64 (0xFFFFFFE0) aligns the tp.
   * Afterwards, give each core 16MB of stack/TLS and shuffle the stack pointers
   * around so they point at the end of the stack for each core (stack grows
   * downwards).
   */ 
   
   la    tp,      _end + 63
   and   tp,      tp,      -64
   mv    sp,      tp                   /* set sp to tp                        */
   
   mv    t0,      a0                   /* copy core id to t0                  */
   li    t1,      0x1000000            /* put 16MB offset into t1             */
   add   sp,      sp,      t1          /* add 16MB to sp                      */
   
  /*
   * The Thread module demands a shared memory space for the cores. This shall
   * be right after the end and it must be as big as the other memory areas.
   */
   
   la    t5,      shared_mem_start     /* load adress of shared_mem_start     */
   sw    tp,      0(t5)                /* set shared_mem_start                */
   la    t5,      shared_mem_size      /* load adress of shared_mem_size      */
   sw    t1,      0(t5)                /* set shared_mem_size                 */
   add   tp,      tp,      t1          /* add 16MB to tp                      */
   add   sp,      sp,      t1          /* add 16MB to sp                      */
   
   
tp_loop:
   beqz  t0,      init_bss             /* if t0 is zero, leave loop           */
   add   tp,      tp,      t1          /* add 16MB to tp                      */
   add   sp,      sp,      t1          /* add 16MB to sp                      */
   addi  t0,      t0,      -1          /* substract 1 from t0                 */
   j     tp_loop                       /* go to start of loop                 */


init_bss:

  /*
   * Initialize the bss (CePU only).
   * Necessary for C libraries to work properly.
   */

   bnez  a0,      pn_thread_entry      /* if core id not zero, skip this and  */
                                       /*  go into pn_thread_entry            */
   
   la    a0,      __bss_start          /* load start adress into a0           */
   la    a2,      __bss_end            /* load end address into a2            */
   sub   a2,      a2,      a0          /* load size of bss into a2            */
   li    a1,      0                    /* load 0 into a1                      */
   call  memset                        /* memset(__bss_start, 0, size_of_bss) */


setup_cache:
   
  /*
   * Setup cache (CePU only).
   * pn_cache_init() invalidates and enables the cache.
   * If pn_cache_init() is not available, we will just do that here, but this
   * takes more ressources.
   */
   
   la    t0,      _thread_data         /* check if we are in a simulation     */
   lb    t1,      0(t0)                /* load first byte from _thread_data   */
   xori  t1,      t1,      'S'         /* check if first byte is an S         */
   beqz  t1,      cache_enable         /* if we are in simulation, skip cache */
                                       /*  invalidation                       */
                                       
   la    t0,      _start               /* load start address of memory        */
   csrr  t1,      0xFC8                /* read pnmemsize register             */
   add   t1,      t0,      t1          /* set t1 to first illegal address     */
   addi  t1,      t1,      -4          /* set t1 to last word                 */

   csrr  t2,      0xFC5                /* read pncacheinfo register           */
   srli  t2,      t2,      8           /* shift right to get cache banks      */
   slli  t2,      t2,      2           /* shift left to multiply with 4       */
   
   /* t2 contains cache line size in byte now */

invalidation_loop:
   .word (0x100B | ((6) << 15) | (((0) & 0xfff) << 20))
                                       /* invalidate line which contains      */
                                       /*  address in register 6 (t1)         */
   sub   t1,      t1,      t2	         /* substract line size from end        */
   bge   t1,      t0,      invalidation_loop
                                       /* proceed if t1 is smaller than start */
   
cache_enable:
   li    t0,      3                    /* load 0b101 into t0                  */
   csrs  0x7C4,   t0                   /* set pncache register                */
   
  /*
   * Jump into C code.
   */
   
   j     _init                         /* CePU jump into _init (syscalls.c)   */
   
   
startup_error:
   
   la    a0,      error_string         /* load address of error_string        */
   call  printf                        /* print error_string                  */   
   .word 0x0000000B                    /* halt execution                      */

/*----------------------------------------------------------------------------*/

.weak pn_thread_entry
pn_thread_entry:

  /* 
   * If CoPUs were started in Mode 2 and libparanut is not compiled in, halt 
   * execution.
   */

   .word 0x0000000B                    /* halt execution                      */
   
/*----------------------------------------------------------------------------*/

trap_entry:

  /*
   * Halt execution on exception, except if it was a breakpoint (cause 3).
   */
  
   csrr  t0,      mcause               /* write machine trap cause to t0      */
   li    t1,      3                    /* load 3 into t1                      */
   beq   t0,      t1,      is_3        /* if cause is 3, go to is_3           */
   .word 0x0000000B                    /* halt execution                      */
   
is_3:
   csrr  t0,      mepc                 /* read machine trap pc to t0          */
   addi  t0,      t0,      4           /* increment to next instruction       */
   csrw  mepc,    t0                   /* write back                          */
   mret                                /* machine mode exception return       */

/*EOF**************************************************************************/
