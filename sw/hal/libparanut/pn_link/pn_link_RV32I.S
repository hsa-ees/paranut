/*
 * Copyright 2019-2020 Anna Pfuetzner (<annakerstin.pfuetzner@gmail.com>)
 *                     Alexander Bahle (<alexander.bahle@hs-augsburg.de>)
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
 * \file
 * \brief         Contains RV32I assembly implementations of assembly functions
 *                called in \ref pn_link.c.
 * 
 *  \includelineno pn_link/pn_link_RV32I.S
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
   * \fn         PN_CID set_linked_as(PN_CMSK coremask, void *frame_adr)
   * \brief      Puts the cores marked in the bitmask to linked mode and enables
   *             them.
   * 
   * Saves the CePUs registers which are preserved aross call, sets CPUs to
   * linked, enables the CoPUs, writes the saved values into everyones 
   * registers, and returns the core ID.
   * 
   * \returns    Core ID.
   */
   PN_CID set_linked_as(PN_CMSK coremask, void *frame_adr, char flags) {}

  /**
   * \internal
   * \fn         PN_CMSK read_PNLM_as(void)
   * \brief      Reads PNLM register.
   * 
   * \returns    Bitmask representing cores in linked mode.
   */
   PN_CMSK read_PNLM_as(void) {}
   
  /**
   * \internal
   * \fn         void write_PNLM_as(PN_CMSK coremask)
   * \brief      Writes PNLM register.
   * 
   * \param[in]  Bitmask representing cores in linked mode.
   */
   void write_PNLM_as(PN_CMSK coremask) {}
 
   /**
   * \internal
   * \fn         void* stack_ptr_as(void)
   * \brief      Returns the stack pointer.
   * 
   */
   void* stack_ptr_as(void) {}
 
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
.align   2                             /* align code to 2^2 bytes             */

/* define sp_loc and tp_loc as external (as defined in common.h) */
.extern sp_loc
.extern tp_loc

/* declare labels in here to be global */
.globl   set_linked_as
.globl   read_PNLM_as
.globl   write_PNLM_as
.globl   stack_ptr_as

/* ParaNut Custom Registers and Instructions */
#include "custom_RV32I.S"

/*Functions********************************************************************/

set_linked_as:

  /* 
   * Save away the CePUs state so the CoPUs can recover it later.
   */
   
   addi  sp,   sp,   -60               /* get space on stack                  */

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
   * Save a0 and a1 away since they will get overridden soon.
   */
   
   mv    s0,   a0                      /* copy a0 to s0 (numcores)            */
   mv    s1,   a1                      /* copy a1 to s1 (frame_adr)           */
   sw    a1,  13*4(sp)                 /* save frame_adrr from arguments to
                                          load it later on all CoPUs          */
   sw    a2,  14*4(sp)                 /* save flags from arguments to
                                          load it later on all CoPUs          */
 
  /* 
   * Put in a fence to make sure everything is properly stored away.
   */

   fence                               /* synchronize memory                  */
   
  /*
   * Activate the linked mode.
   */
   
   csrw  pnlm, a0                      /* sets pnlm to passed bitmask         */
   csrw  pnce, a0                      /* sets pnce to passed bitmask         */
   
  /*
   * Get back the registers so all CoPUs have the same values to work with.
   * We are now linked!
   * Now, to further proceed, we have to do the following:
   * - Initialize gp of CoPUs.
   * - Initialize tp of CoPUs.
   * - Initialize sp of CoPUs.
   * - Copy the stack of the CePU to all cores.
   *
   * The following code only works out because the memory layout is like this:
   * 
   *    tp0-------     ---                                     Small Adress
   *    |        |      |
   *    |        |      |
   *    sp0-------      |
   *    | This   |      |  Thread Space of CePU
   *    | needs  |      |
   *    | to be  |      |
   *    | copied!|      |
   *    tp1------|     ---
   *    |        |      |
   *    |        |      |
   *    sp1-------      |
   *    | Copy   |      |  Thread Space of CoPU 1
   *    | it     |      |
   *    | here!  |      |
   *    |        |      |
   *    tp2------|     ---
   *    |        |      |
   *    |        |      |
   *    sp2-------      |
   *    | Copy   |      |  Thread Space of CoPU 2
   *    | it     |      |
   *    | here!  |      |
   *    |        |      |
   *    tp3------|     ---
   *    |        |      |                                      Big Adress
   *             ...
   *
   * All thread spaces are equally sized to shared_mem_size. The stack grows
   * downwards (so upwards in this picture since the adress growth direction is
   * downwards.
   *
   * All of this means that we can calculate tp1, tp2, tp3 etc. with just tp0
   * and an offset of shared_mem_size * Core ID.
   * Same goes for sp1, sp2, sp3 etc.
   * The size of the stack can be calculated with just this data, too.
   */
   
   /* initialize global pointer */
   .option push
   .option norelax
   la gp, __global_pointer$
   .option pop
   
   /* get the thread pointer */
   csrr  t0,   pncoreid                 /* read core ID                        */
   lw    t1,   shared_mem_size         /* load size of shared memory          */
   mul   t2,   t1,   t0                /* calculate individual offset         */
   lw    a2,   tp_loc                  /* load thread pointer of CePU         */
   add   tp,   a2,   t2                /* add individual offset               */
   
   /* getting the stack pointer */
   lw    a1,   sp_loc                  /* get stack pointer of CePU           */
   lw    s1,   13*4(a1)                /* load saved frame_address            */
   lw    s2,   14*4(a1)                /* load saved flags                    */
   lw    s0,   1*4(a1)                 /* load callee saved register 0        */
   add   sp,   a1,   t2                /* add individual offset               */
   beqz  s2,   skipfp                  /* if flags != 0 then s0 must contain 
                                          the adjusted frame pointer after 
                                          exiting this funciton */   
   add   s0,   s0,   t2                /* add individual offset to fp         */
skipfp:

  /* 
   * Copy stack - memcpy(destination, source, size)
   * Destination is own stack.
   * Source is stack pointer of CePU -> Already written into a1!
   * Size is calculated as follows:
   *    (tp of CePU + shared memory size) - sp of CePU 
   */
   
   mv    a0,   sp                      /* destination is own stack            */
   add   a2,   a2,   t1                /* tp of CePU + shared memory size     */

   beqz  s1,   1f                      /* if s1 is zero copy the whole stack  */
   mv    a2,   s1                      /* else use frame_adr address          */
1: sub   a2,   a2,   a1                /* substract sp of CePU                */
   fence                               /* synchronize memory                  */   
   call  memcpy                        /* copy the stack                      */
   fence                               /* synchronize memory                  */    
   
  /*
   * Recover the CePUs state.
   */
   
   lw    ra,   0*4(sp)                 /* load return address                 */
                                       /* skip s0/fp it has already been done */
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
   
   addi  sp,   sp,   60                /* set back stack pointer              */  
   
  /*
   * Get the core ID, store in return value, return
   */
   
   csrr  a0,   pncoreid                 /* put pncoreid into a0                 */
   ret                                 /* return                              */
   
/*----------------------------------------------------------------------------*/

read_PNLM_as:

   csrr  a0, pnlm                      /* put pnlm in return value            */
   ret                                 /* return                              */
   
/*----------------------------------------------------------------------------*/

write_PNLM_as:

   csrw  pnlm, a0                      /* put pnlm in return value            */
   ret                                 /* return                              */

/*----------------------------------------------------------------------------*/

stack_ptr_as:

   mv   a0, sp                         /* put stack pointer in return value   */
   ret                                 /* return                              */
   
#endif /* !DOXYGEN */
   
/*EOF**************************************************************************/
