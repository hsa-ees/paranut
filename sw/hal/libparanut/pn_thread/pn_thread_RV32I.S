/*
 * Copyright 2019-2020 Anna Pfuetzner (<annakerstin.pfuetzner@gmail.com>)
 *
 * Redistribution and use in source and binary forms, with or without 
 * modification, are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, 
 * this list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form must reproduce the above copyright notice, 
 * this list of conditions and the following disclaimer in the documentation 
 * and/or other materials provided with the distribution.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE 
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
 * POSSIBILITY OF SUCH DAMAGE.
 */

/** @file */

/**
 * \internal
 * \file          pn_thread/pn_thread_RV32I.S
 * \brief         Contains RV32I assembly implementations of assembly functions
 *                called in \ref pn_thread.c.
 *
 * \includelineno pn_thread/pn_thread_RV32I.S
 */

/* 
 * Put in here so Doxygen will know that it is implemented in this file.
 * Sadly, Doxygen has no built in assembly interpreter, so we are stuck with
 * this.
 */

#ifdef DOXYGEN

  /**
   * \addtogroup as
   * @{
   */
    
  /**
   * @{
   */
   
  /**
   * \internal
   * \fn         void set_threaded_as(PN_CMSK coremask)
   * \brief      Takes the linked flag away from all cores and enables the cores
   *             in the coremask.
   * 
   * \todo       Needs changes in case it will ever be allowed to run some cores
   *             in linked mode and some in threaded mode.
   * \todo       Correct this after rewrite of thread module
   */
   void set_threaded_as(PN_CMSK coremask) {}

  /**
   * \internal
   * \fn         void enter_threaded_mode_as(void)
   * \brief      Prepares CoPU for threaded mode by taking the CePUs stack.
   * 
   * \todo       Needs changes in case it will ever be allowed to run some cores
   *             in linked mode and some in threaded mode.
   */   
   void enter_threaded_mode_as(void) {}

  /**
   * @}
   */
 
  /**
   * @}
   */
   
#endif /* DOXYGEN */

/*Header***********************************************************************/

#ifndef DOXYGEN

.text                                  /* enter text section                  */
.align 2                               /* align Code to 2^2 Bytes             */

/* define sp_loc and tp_loc as external (as defined in common.h) */
.extern sp_loc
.extern tp_loc

/* declare labels in here to be global */
.globl   set_threaded_as
.globl   enter_threaded_mode_as
.globl   set_thread_run_as
.globl   enter_thread_run_as

/* ParaNut Custom Registers and Instructions */
#include "custom_RV32I.S"

/*Functions********************************************************************/

set_threaded_as:

  /* 
   * Save away the CePUs state so the CoPUs can recover it later.
   */
   
   addi  sp,   sp,   -52               /* get space on stack                  */

                                       /* zero does not need saving           */
   sw    ra,   0*4(sp)                 /* save return address                 */
                                       /* stack pointer stored elsewhere      */
                                       /* global pointer is read later        */
                                       /* thread pointer is not changed       */
                                       /* temporary regs are caller saved     */
   sw    s0,   1*4(sp)                 /* save callee saved register 0        */
   sw    s1,   2*4(sp)                 /* save callee saved register 1        */
                                       /* argument regs are caller saved      */
   sw    s2,   3*4(sp)                 /* save callee saved register 2        */
   sw    s3,   4*4(sp)                 /* save callee saved register 3        */
   sw    s4,   5*4(sp)                 /* save callee saved register 4        */
   sw    s5,   6*4(sp)                 /* save callee saved register 5        */
   sw    s6,   7*4(sp)                 /* save callee saved register 6        */
   sw    s7,   8*4(sp)                 /* save callee saved register 7        */
   sw    s8,   9*4(sp)                 /* save callee saved register 8        */
   sw    s9,  10*4(sp)                 /* save callee saved register 9        */
   sw    s10, 11*4(sp)                 /* save callee saved register 10       */
   sw    s11, 12*4(sp)                 /* save callee saved register 11       */
                                       /* temporary regs are caller saved     */   
   
   
  /*
   * Put the stack pointer and thread pointer somewhere safe so that the CoPUs
   * can calculate their own stack pointer later.
   */
   
   la    t0,   sp_loc                  /* load adress of location             */
   sw    sp,   0(t0)                   /* put in stack pointer                */
   la    t0,   tp_loc                  /* load adress of location             */
   sw    tp,   0(t0)                   /* put in thread pointer               */

  /*
   * Save a0 away since it will get overridden soon.
   */
   
   mv    s0,   a0                      /* copy a0 to s0                       */
   
  /*
   * Copy the whole thread stack to shared memory location.
   * Look at the following ASCII art to understand what I am doing.
   * To the left, you can see the thread space of the CePU. The stack is the
   * bottom part. It grows "downwards" which means it goes against normal adress
   * growth direction (so upwards in this picture).
   * On the right, you can see the shared memory. This is where we want to copy
   * the stack.
   * 
   *    tp--------                  |           shared_mem_start(t0)--     |
   *    |        |                  |                       |        |     |
   *    |        |                  |                       |        |     |
   *    |        |                  |                       |        |     |
   *    |        |                  |                       |        |  adress
   *    |        |                  |                       |        |  growth
   *    |        |           shared_mem_size(t1)            |        | direction
   *    |        |                  |                       |        |     |
   *    |        |                  |                       |        |     |
   *    |        |                  |                       |        |     |
   *    |        |                  |                       |        |     |
   *    |        |                  |                       |        |     |
   *    sp--memcpy source(a1)     | |         memcpy destination(a0)--     |
   *    | This   |                | |                       | Copy it|     |
   *    | needs  |  memcpy size(a2) |                       | over   |     |
   *    | to be  |                | |                       | here!  |     |
   *    | copied!|                | |                       |        |     |
   *    t2--------                v v     a0 (after first operation)--     v
   */
   
   lw    t0,   shared_mem_start        /* load adress of shared memory        */
   lw    t1,   shared_mem_size         /* load size of shared memory          */
   
   add   t2,   tp,   t1                /* calculate the start adress of stack */
   sub   a2,   t2,   sp                /* calculate the stack size            */
   add   a0,   t0,   t1                /* calculate where the end of the      */
   sub   a0,   a0,   a2                /*  stack would be in shared memory    */
   mv    a1,   sp                      /* use stack pointer as source         */
   fence                               /* synchronize memory                  */

   call  memcpy                        /* copy the memory                     */
   fence                               /* synchronize memory                  */
   
  /* 
   * Activate the threaded mode.
   */
   
   csrw  pnlm, x0                      /* set linked mask to all 0            */
   csrw  pnce, s0                      /* enable the cores in the coremask    */
   
  /* 
   * The rest is done by the CoPUs themselves (see startup code). We only need
   * to return at this point. The CoPUs will return to the same address!
   */
   
   lw    s0,   1*4(sp)                 /* get s0 back                         */
   lw    ra,   0*4(sp)                 /* get ra back                         */
   addi  sp,   sp,   52                /* put stack back                      */   
   ret                                 /* return                              */
      
/*----------------------------------------------------------------------------*/

enter_threaded_mode_as:

  /* 
   * So, what do we need to do so the CoPU is can operate in threaded mode?
   * Basically, we just need to copy the stack from the CePU in the state it 
   * was during set_threaded_as(). Afterwards, we are retrieving the CePUs state
   * from the copied stack. Since we also recovered the return adress, to CoPU
   * that executes this will afterwards jump to the end of the function that
   * started the threaded mode.
   *
   * Let us get the stack from the shared memory area. The CePU was hopefully
   * nice enough to copy its stack there earlier.
   * Look at the following ASCII art to understand what I am doing.
   * To the left, you can see the thread space of the CoPU. The stack is the
   * bottom part. It grows "downwards" which means it goes against normal adress
   * growth direction (so upwards in this picture). We need to set our own
   * stack pointer correctly from the data we got.
   * On the right, you can see the shared memory. This is where we want to get
   * the stack from.
   * So what do we even know in the current position?
   * We know: shared_mem_start, shared_mem_size, and the thread and stack
   * pointer of the CePU. From that, we can calculate everything we need.
   * 
   *    tp--------                   |          shared_mem_start(t0)--     |
   *    |        |                   |                      |        |     |
   *    |        |                   |                      |        |     |
   *    |        |                   |                      |        |     |
   *    |        |                   |                      |        |  adress
   *    |        |                   |                      |        |  growth
   *    |        |            shared_mem_size(t1)           |        | direction
   *    |        |                   |                      |        |     |
   *    |        |                   |                      |        |     |
   *    |        |                   |                      |        |     |
   *    |        |                   |                      |        |     |
   *    |        |                   |                      |        |     |
   *    sp--memcpy destination(a0) | |             memcpy source(a1)--     |
   *    | Copy it|                 | |                      | This   |     |
   *    | over   |   memcpy size(a2) |                      | needs  |     |
   *    | here!  |                 | |                      | to be  |     |
   *    |        |                 | |                      | copied!|     |
   *    ----------                 v v                      ----------     v
   */
   
   lw    t0,   shared_mem_start        /* load adress of shared memory        */
   lw    t1,   shared_mem_size         /* load size of shared memory          */
   lw    t4,   sp_loc                  /* get CePUs stack pointer             */
   lw    t5,   tp_loc                  /* get CePUs thread pointer            */
   
   add   a2,   t5,   t1                /* memcpy size = (tp of CePU +         */
   sub   a2,   a2,   t4                /*  shared_mem_size) - sp of CePU      */
   
   add   a1,   t0,   t1                /* memcpy source = (shared_mem_start + */
   sub   a1,   a1,   a2                /*  shared_mem_size) - memcpy size     */
   
   add   a0,   tp,   t1                /* memcpy destination = (tp +          */
   sub   a0,   a0,   a2                /* shared_mem_size) - memcpy size      */
   
   mv    sp,   a0                      /* sp = memcpy destination             */
   
   call  memcpy                        /* copy the memory                     */
   
  /* 
   * Put in a fence to make sure everything is properly stored away.
   */
   
   fence                               /* synchronize memory                  */   

  /*
   * Recover the CePUs state.
   */
   
   lw    ra,   0*4(sp)                 /* load return address                 */
   lw    s0,   1*4(sp)                 /* load callee saved register 0        */
   lw    s1,   2*4(sp)                 /* load callee saved register 1        */
   lw    s2,   3*4(sp)                 /* load callee saved register 2        */
   lw    s3,   4*4(sp)                 /* load callee saved register 3        */
   lw    s4,   5*4(sp)                 /* load callee saved register 4        */
   lw    s5,   6*4(sp)                 /* load callee saved register 5        */
   lw    s6,   7*4(sp)                 /* load callee saved register 6        */
   lw    s7,   8*4(sp)                 /* load callee saved register 7        */
   lw    s8,   9*4(sp)                 /* load callee saved register 8        */
   lw    s9,  10*4(sp)                 /* load callee saved register 9        */
   lw    s10, 11*4(sp)                 /* load callee saved register 10       */
   lw    s11, 12*4(sp)                 /* load callee saved register 11       */ 
   
   addi  sp,   sp,   52                /* set back stack pointer              */   

  /*
   * Returning. Note that we have changed ra to somewhere in 
   * pn_begin_threaded[_m|_gm].
   */
   
   ret                                 /* return                              */


set_thread_run_as:
  /* 
   * Save return address, s0, a1 (function to be executed by CoPU(s)) and a2 (args)
   * to stack 
   */
  addi  sp,   sp,   -16               /* get space on stack                  */
  sw    a1,   0*4(sp)                 /* save function address               */
  sw    a2,   1*4(sp)                 /* save args address                   */
  sw    ra,   2*4(sp)                 /* save CePU return address            */
  sw    s0,   3*4(sp)                 /* save callee saved register 0        */
  /*
   * Put the stack pointer and thread pointer somewhere safe so that the CoPUs
   * can calculate their own stack pointer later.
   */
   
   la    t0,   sp_loc                  /* load adress of location             */
   sw    sp,   0(t0)                   /* put in stack pointer                */
   la    t0,   tp_loc                  /* load adress of location             */
   sw    tp,   0(t0)                   /* put in thread pointer               */

  /*
   * Save a0 away since it will get overridden soon.
   */
   
   mv    s0,   a0                      /* copy a0 to s0                       */
   
  /*
   * Copy the whole the fucntion pointer to shared memory location.
   * Look at the following ASCII art to understand how this is done.
   * To the left, you can see the thread space of the CePU. The stack is the
   * bottom part. It grows "downwards" which means it goes against normal adress
   * growth direction (so upwards in this picture).
   * On the right, you can see the shared memory. This is where we want to copy
   * the pointer.
   * 
   *    tp--------                  |           shared_mem_start(t0)--     |
   *    |        |                  |                       |        |     |
   *    |        |                  |                       |        |     |
   *    |        |                  |                       |        |     |
   *    |        |                  |                       |        |  adress
   *    |        |                  |                       |        |  growth
   *    |        |           shared_mem_size(t1)            |        | direction
   *    |        |                  |                       |        |     |
   *    |        |                  |                       |        |     |
   *    |        |                  |                       |        |     |
   *    |        |                  |                       |        |     |
   *    |        |                  |                       |        |     |
   *    sp--memcpy source(a1)     | |         memcpy destination(a0)--     |
   *    | This   |                | |                       | Copy it|     |
   *    | needs  |  memcpy size(a2) |                       | over   |     |
   *    | to be  |                | |                       | here!  |     |
   *    | copied!|                | |                       |        |     |
   *    t2--------                v v     a0 (after first operation)--     v
   */
   
   lw    t0,   shared_mem_start        /* load adress of shared memory        */
   lw    t1,   shared_mem_size         /* load size of shared memory          */
   
   add   t2,   tp,   t1                /* calculate the start adress of stack */
   li    a2,   8                       /* memcopy size is 8 byte              */
   add   a0,   t0,   t1                /* calculate where the end of the      */
   sub   a0,   a0,   a2                /*  stack would be in shared memory    */
   mv    a1,   sp                      /* use stack pointer as source         */
   fence                               /* synchronize memory                  */

   call  memcpy                        /* copy the memory                     */
   fence                               /* synchronize memory                  */
   
  /* 
   * Activate the threaded mode.
   */
   
   csrw  pnlm, x0                      /* set linked mask to all 0            */
   csrr  a0,   pnce                    /* put pnce a0          */
   or    a0,   a0,  s0                 /* or a0 with s0 so we do not 
                                                       halt any running CoPUs */
   csrw  pnce, a0                      /* enable the cores in the coremask    */
   
  /* 
   * The rest is done by the CoPUs themselves (see startup code). We only need
   * to return at this point. The CoPUs never return, as they are halted after 
   * executing the instructions at a1!
   */
   
   lw    ra,   2*4(sp)                 /* get ra back                         */
   lw    s0,   3*4(sp)                 /* get s0 back                         */
   addi  sp,   sp,   16                /* put stack back                      */   
   ret                                 /* return                              */
      
/*----------------------------------------------------------------------------*/

enter_thread_run_as:

  /* 
   * So, what do we need to do so the CoPU is can execute a thread?
   * All that has to be done is retrieve the entrypoint stored in shared memory
   * and execute the instructions located at that adress.
   * Finaly the core is halted, as the function has been executed.
   *
   * To retreive the instruction location, the shared memory must be accessed
   * Previously it has been written there by the CePU.
   * Look at the following ASCII art to understand how that is done.
   * To the left, you can see the thread space of the CoPU. The stack is the
   * bottom part. It grows "downwards" which means it goes against normal adress
   * growth direction (so upwards in this picture). We need to set our own
   * stack pointer correctly from the data we got.
   * On the right, you can see the shared memory. This is where we want to get
   * the data from.
   * So what do we even know in the current position?
   * We know: shared_mem_start, shared_mem_size, and the thread and stack
   * pointer of the CePU. From that, we can calculate everything we need.
   * 
   *    tp--------                   |          shared_mem_start(t0)--     |
   *    |        |                   |                      |        |     |
   *    |        |                   |                      |        |     |
   *    |        |                   |                      |        |     |
   *    |        |                   |                      |        |  adress
   *    |        |                   |                      |        |  growth
   *    |        |            shared_mem_size(t1)           |        | direction
   *    |        |                   |                      |        |     |
   *    |        |                   |                      |        |     |
   *    |        |                   |                      |        |     |
   *    |        |                   |                      |        |     |
   *    |        |                   |                      |        |     |
   *    sp--memcpy destination(a0) | |             memcpy source(a1)--     |
   *    | Copy it|                 | |                      | This   |     |
   *    | over   |   memcpy size(a2) |                      | needs  |     |
   *    | here!  |                 | |                      | to be  |     |
   *    |        |                 | |                      | copied!|     |
   *    ----------                 v v                      ----------     v
   */
   
   lw    t0,   shared_mem_start        /* load adress of shared memory        */
   lw    t1,   shared_mem_size         /* load size of shared memory          */
   lw    t4,   sp_loc                  /* get CePUs stack pointer             */
   lw    t5,   tp_loc                  /* get CePUs thread pointer            */
   
   li    a2,   8                         /* memcpy size = (tp of CePU +         */
   
   add   a1,   t0,   t1                /* memcpy source = (shared_mem_start + */
   sub   a1,   a1,   a2                /*  shared_mem_size) - memcpy size     */
   
   add   a0,   tp,   t1                /* memcpy destination = (tp +          */
   sub   a0,   a0,   a2                /* shared_mem_size) - memcpy size      */
   
   mv    sp,   a0                      /* sp = memcpy destination             */
   
   call  memcpy                        /* copy the memory                     */
   
  /* 
   * Put in a fence to make sure everything is properly stored away.
   */
   
   fence                               /* synchronize memory                  */   

  /*
   * Recover the CePUs state.
   */
   
   lw    s0,   0*4(sp)                 /* load return address                 */
   lw    a0,   1*4(sp)                 /* load args into parameter 1          */
   csrr    a1,   pncoreid              /* load core id into parameter 2       */
   
   addi  sp,   sp,   8                /* set back stack pointer              */   
   
   jalr   s0, 0
  /*
   * Returning. Note that we have changed ra to somewhere in 
   * pn_begin_threaded[_m|_gm].
   */
   call halt_as
   ret                                 /* return sould not be reached        */


#endif /* !DOXYGEN */
   
/*EOF**************************************************************************/
