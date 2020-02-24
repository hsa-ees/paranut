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
 * \file
 * \brief         Contains RV32I assembly implementations of assembly functions
 *                called in \ref pn_exception.c.
 *
 * \includelineno pn_exception/pn_exception_RV32I.S
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
   * \fn         void exception_init_as(void)
   * \brief      Sets Machine Trap-Vector Base-Address Register correctly and
   *             disables interrupts.
   */
   void exception_init_as(void) {}
   
  /**
   * \internal
   * \fn         void exception_entry_as(void)
   * \brief      Entry point for exceptions (includes interrupts).
   */
   void exception_entry_as(void) {}
   
  /**
   * \internal
   * \fn         PN_CMSK read_PNX_as(void)
   * \brief      Reads ParaNut CoPU exception pending register.
   * 
   * \return     Mask that represents CPUs.
   */
   PN_CMSK read_PNX_as(void) {}
   
  /**
   * \internal
   * \fn         PN_CMSK read_PNCAUSE_as(void)
   * \brief      Reads ParaNut CoPU trap cause ID register.
   * 
   * \return     Mask that represents CPUs.
   */
   PN_CMSK read_PNCAUSE_as(void) {}
   
  /**
   * \internal
   * \fn         PN_CMSK read_PNEPC_as(void)
   * \brief      Reads ParaNut CoPU exception program counter register.
   * 
   * \return     Mask that represents CPUs.
   */
   PN_CMSK read_PNEPC_as(void) {}
   
  /**
   * \internal
   * \fn         unsigned int read_MTVAL_as(void)
   * \brief      Reads Machine Trap Value register.
   * 
   * \return     Machine Trap Value register.
   */
   unsigned int read_MTVAL_as(void) {}
   
  /**
   * \internal
   * \fn         void write_PNXSEL_as(PN_CMSK coremask)
   * \brief      Writes ParaNut CoPU exception select register.
   *
   * \param[in]  coremask is a mask that represents CPUs. Only 1 bit shall be 
   *             set.
   */
   void write_PNXSEL_as(PN_CMSK coremask) {}
   
  /**
   * \internal
   * \fn         void write_MSTATUS_as(unsigned int register)
   * \brief      Writes Machine Status register.
   *
   * \param[in]  register is the new value of the register.
   */
   void write_MSTATUS_as(unsigned int register) {}
   
  /**
   * \internal
   * \fn         void ecall_as(void)
   * \brief      Causes environment call exception.
   */
   void ecall_as(void) {}
   
  /**
   * \internal
   * \fn         void progress_mepc_as(void)
   * \brief      Sets MEPC to next instruction.
   */
   void progress_mepc_as(void) {}
 
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

/* declare labels in here to be global */
.globl   exception_init_as
.globl   exception_entry_as
.globl   read_PNX_as
.globl   read_PNCAUSE_as
.globl   read_PNEPC_as
.globl   read_MTVAL_as
.globl   write_PNXSEL_as
.globl   write_MSTATUS_as
.globl   ecall_as
.globl   progress_mepc_as

/* ParaNut Custom Registers and Instructions */
#include "custom_RV32I.S"

/*Functions********************************************************************/

exception_init_as:
   
   li    t0,      0                    /* set t0 to 0                         */
   csrw  mstatus, t0                   /* disable interrupts                  */
   la    t0,      exception_entry_as   /* load address of exception_entry_as  */
   csrw  mtvec,   t0                   /* set trap-handler base address to    */
                                       /*  exception_entry_as                 */
   ret                                 /* return                              */
               
   

/*----------------------------------------------------------------------------*/

exception_entry_as:

  /*
   * Save away registers on stack.
   */
   
   addi  sp,       sp,      -124
   sw    x1,       0*4(sp)
   sw    x2,       1*4(sp)
   sw    x3,       2*4(sp)
   sw    x4,       3*4(sp)
   sw    x5,       4*4(sp)
   sw    x6,       5*4(sp)
   sw    x7,       6*4(sp)
   sw    x8,       7*4(sp)
   sw    x9,       8*4(sp)
   sw    x10,      9*4(sp)
   sw    x11,     10*4(sp)
   sw    x12,     11*4(sp)
   sw    x13,     12*4(sp)
   sw    x14,     13*4(sp)
   sw    x15,     14*4(sp)
   sw    x16,     15*4(sp)
   sw    x17,     16*4(sp)
   sw    x18,     17*4(sp)
   sw    x19,     18*4(sp)
   sw    x20,     19*4(sp)
   sw    x21,     20*4(sp)
   sw    x22,     21*4(sp)
   sw    x23,     22*4(sp)
   sw    x24,     23*4(sp)
   sw    x25,     24*4(sp)
   sw    x26,     25*4(sp)
   sw    x27,     26*4(sp)
   sw    x28,     27*4(sp)
   sw    x29,     28*4(sp)
   sw    x30,     29*4(sp)
   sw    x31,     30*4(sp)

  /*
   * Check if we got an interrupt (asynchronous exception) or a synchronous 
   * exception.
   * If mcause has the first bit set (which makes it a negative number), we got
   * an actual interrupt!
   */
   
   csrr  a0,      mcause               /* write machine trap cause to a0      */
   csrr  a1,      mepc                 /* write exception program count to a1 */
   mv    a2,      sp                   /* copy stack pointer to a2            */
   bltz  a0,      interrupt            /* check if first bit set              */


exception:

  /*
   * If we got here, it means we have an exception, not an interrupt.
   * Call exception handler. Note that MEPC points to the instruction that 
   * caused the exception.
   */
   
   jal   handle_exc                    /* call exception handler              */
   j     return                        /* return from trap                    */


interrupt: 

  /*
   * If we got here, it means we have an interrupt, not an exception.
   * First, remove the interrupt flag, then call actual interrupt handler.
   * Afterwards, check if interrupts were previously enabled (mstatus.MPIE) and
   * enable interrupts again if they were.
   * For interrupts, MEPC points to the instruction where execution should 
   * resume after interrupt is handled.
   */

   slli  a0,      a0,      1           /* shift a0 to the left by one bit     */
   srli  a0,      a0,      1	         /* shift a0 to the right by one bit    */ 
   
   jal   handle_int                    /* call interrupt handler              */
   
   csrr  a0,      mstatus              /* load machine status into a0         */
   andi  a0,      a0,      0x80        /* check on MPIE bit                   */
   srli  a0,      a0,      4           /* if MPIE was set before, set MIE     */
   csrs  mstatus, a0                   /* activate all that is set in a0      */

return:
  
  /*
   * Get back the registers from stack.
   */

   lw    x1,       0*4(sp)
   lw    x2,       1*4(sp)
   lw    x3,       2*4(sp)
   lw    x4,       3*4(sp)
   lw    x5,       4*4(sp)
   lw    x6,       5*4(sp)
   lw    x7,       6*4(sp)
   lw    x8,       7*4(sp)
   lw    x9,       8*4(sp)
   lw    x10,      9*4(sp)
   lw    x11,     10*4(sp)
   lw    x12,     11*4(sp)
   lw    x13,     12*4(sp)
   lw    x14,     13*4(sp)
   lw    x15,     14*4(sp)
   lw    x16,     15*4(sp)
   lw    x17,     16*4(sp)
   lw    x18,     17*4(sp)
   lw    x19,     18*4(sp)
   lw    x20,     19*4(sp)
   lw    x21,     20*4(sp)
   lw    x22,     21*4(sp)
   lw    x23,     22*4(sp)
   lw    x24,     23*4(sp)
   lw    x25,     24*4(sp)
   lw    x26,     25*4(sp)
   lw    x27,     26*4(sp)
   lw    x28,     27*4(sp)
   lw    x29,     28*4(sp)
   lw    x30,     29*4(sp)
   lw    x31,     30*4(sp)
   addi  sp,      sp,      124
   
   mret                                /* machine mode exception return       */
      
/*----------------------------------------------------------------------------*/

read_PNX_as:

   csrr  a0,      pnx                  /* read pnx                            */
   ret                                 /* return                              */

/*----------------------------------------------------------------------------*/
   
read_PNCAUSE_as:

   csrr  a0,      pncause              /* read pncause                        */
   ret                                 /* return                              */

/*----------------------------------------------------------------------------*/
   
read_PNEPC_as:

   csrr  a0,      pnepc                /* read pnepc                          */
   ret                                 /* return                              */
   
/*----------------------------------------------------------------------------*/
   
read_MTVAL_as:

   csrr  a0,      mtval                /* read mtval                          */
   ret                                 /* return                              */

/*----------------------------------------------------------------------------*/
   
write_PNXSEL_as:

   csrw  pnxsel,  a0                   /* write pnxsel                        */
   ret                                 /* return                              */

/*----------------------------------------------------------------------------*/

write_MSTATUS_as:

   csrw  mstatus, a0                   /* write mstatus                       */
   ret                                 /* return                              */
   
/*----------------------------------------------------------------------------*/

ecall_as:

   ecall                               /* cause environment exception         */
   ret                                 /* return                              */
   
/*----------------------------------------------------------------------------*/

progress_mepc_as:

   csrr  t0,      mepc                 /* read MEPC into t0                   */
   addi  t0,      t0,      4           /* set to next instruction             */
   csrw  mepc,    t0                   /* write back                          */
   ret                                 /* return                              */

#endif /* !DOXYGEN */
   
/*EOF**************************************************************************/
