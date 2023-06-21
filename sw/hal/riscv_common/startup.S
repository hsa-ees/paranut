/*Header***********************************************************************/
/*************************************************************************

  This file is part of the ParaNut project.

  Copyright (C) 2019-2022 Anna Pfuetzner <anna.pfuetzner@hs-augsburg.de>
                          Alexander Bahle <alexander.bahle@hs-augsburg.de>
                          Nico Borgsmüller <nico.borgsmueller@hs-augsburg.de>
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
  
.section ".pn_tohost","aw",@progbits
.align   4                             /* align pn_tohost to 2^4 bytes           */
pn_tohost:
   .word 0
pn_fromhost:
   .word 0

.section ".pntdata","aw",@nobits
.align   4                             /* align _thread_data to 2^4 bytes     */
_thread_data:
   .zero 32*4*2                        /* thread space for 32 CPUs (256 Byte) */
   
shared_mem_start:
   .zero 4                             /* adress of shared memory start       */
shared_mem_size:
   .zero 4                             /* shared memory size in byte          */
  
   
/* declare labels in here to be global */
.globl   _tdata_begin                  /* label for area .tdata.begin         */
.globl   _tdata_end                    /* label for area .tdata.end           */
.globl   _tbss_end                     /* label for area .tbss.end            */
.globl   pn_tohost                        /* label for area .pn_tohost              */
.globl   pn_fromhost                      /* label for area .pn_tohost + 4 byte     */
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
   
   csrr  a0,      0xCD4              /* read core ID from pncoreid CSR      */
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
   * Afterwards, give each core __usr_stack_heap_size (defined in paranut.ld) 
   * stack/TLS and shuffle the stack pointers around so they point at the end 
   * of the stack for each core (stack grows downwards).
   */ 
   
   la    tp,      _end + 63
   and   tp,      tp,      -64
   mv    sp,      tp                   /* set sp to tp                        */
   
   mv    t0,      a0                   /* copy core id to t0                  */
   la    t1,      __usr_stack_heap_size/* load the offset/size into t1        */
   add   sp,      sp,      t1          /* add offset to sp                    */
   
  /*
   * The Thread module demands a shared memory space for the cores. This shall
   * be right after the end and it must be as big as the other memory areas.
   */
   
   la    t5,      shared_mem_start     /* load adress of shared_mem_start     */
   sw    tp,      0(t5)                /* set shared_mem_start                */
   la    t5,      shared_mem_size      /* load adress of shared_mem_size      */
   sw    t1,      0(t5)                /* set shared_mem_size                 */
   add   tp,      tp,      t1          /* add offset to tp                    */
   add   sp,      sp,      t1          /* add offset to sp                    */
   
   
tp_loop:
   beqz  t0,      setup_cache          /* if t0 is zero, leave loop           */
   add   tp,      tp,      t1          /* add offset to tp                    */
   add   sp,      sp,      t1          /* add offset to sp                    */
   addi  t0,      t0,      -1          /* substract 1 from t0                 */
   j     tp_loop                       /* go to start of loop                 */
       
    
setup_cache:
   
  /*
   * Setup cache (CePU only).
   * pn_cache_init() invalidates and enables the cache.
   * If pn_cache_init() is not available, we will just do that here, but this
   * takes more ressources.
   */
   
   bnez  a0,      pn_thread_entry      /* if core id not zero, skip this and  */
                                       /*  go into pn_thread_entry            */
invalidate_all:
   .word 0x0000600B                    /* invalidate all special ParaNut inst */
   fence                               /* wait for completion                 */
   
cache_enable:
   li    t0,      3                    /* load 0b11 into t0                   */
   csrs  0x7C0,   t0                   /* set pncache register                */
  


reloc_data:
  /*
   * Relocate the .data section from permanent memory (flash, rom, ...) to RAM
   * (CePU only).
   * Writes full words for improved performance (.data section is word aligned)
   */
   
   la    t0,      __src_data_start     /* load address of .data to copy from */
   la    t1,      __data_size          /* load the size of the section       */
   add   t1,      t1,     t0           /* calculate end address              */
   la    t2,      __data_start         /* load target address                */
reloc_loop:   
   lw    t3,      0(t0)                /* read value from source to t3       */ 
   sw    t3,      0(t2)                /* write value to target              */
   addi  t0,      t0,     4            /* increment source address           */
   addi  t2,      t2,     4            /* increment target address           */
   bne   t0,      t1,     reloc_loop   /* check if source addr /= end addr   */


init_bss:

  /*
   * Initialize the bss (CePU only).
   * Necessary for C libraries to work properly.
   */
   
   la    a0,      __bss_start          /* load start adress into a0           */
   la    a2,      __bss_size           /* load size of bss into a2            */
   li    a1,      0                    /* load 0 into a1                      */
   call  memset                        /* memset(__bss_start, 0, size_of_bss) */

   
  /*
   * Jump into C code.
   */
   
   call __libc_init_array              /* Call __libc_init_array              */
   j     _init                         /* CePU jump into _init (syscalls.c)   */
   call __libc_fini_array              /* Call __libc_fini_array - never reached
                                          but should be executed by exit()    */
   
   
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


error_string:                          /* prepare error message               */
   .string "There was an error in the startup code. Contact maintainers!\n\n"

/*EOF**************************************************************************/
