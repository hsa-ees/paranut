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
 *                called in \ref pn_spinlock.c.
 *
 * When changing anything in here, make sure that these changes reflect on the
 * internal lock of the \ref th. The internal lock there needs to be created as
 * an initialized lock!
 *
 * \includelineno pn_spinlock/pn_spinlock_RV32I.S
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
   * \fn         int init_as(_pn_spinlock *spinlock)
   * \brief      Sets lock to free, no conditions checked.
   */
   int init_as(_pn_spinlock *spinlock) {}

  /**
   * \internal
   * \fn         int trylock_as(_pn_spinlock *spinlock, PN_CID coreid)
   * \brief      Tries locking the lock. Fails if not free.
   */
   int trylock_as(_pn_spinlock *spinlock, PN_CID coreid) {}

  /**
   * \internal
   * \fn         int unlock_as(_pn_spinlock *spinlock, PN_CID coreid)
   * \brief      Tries unlocking. Fails if not owned by current hart.
   */
   int unlock_as(_pn_spinlock *spinlock, PN_CID coreid) {}

  /**
   * \internal
   * \fn         int destroy_as(_pn_spinlock *spinlock, PN_CID coreid)
   * \brief      Sets lock to dead if it's free or it's owned by current hart.
   */
   int destroy_as(_pn_spinlock *spinlock, PN_CID coreid) {}
 
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
.globl   init_as
.globl   trylock_as
.globl   unlock_as
.globl   destroy_as

/* local defines */
.equ     DEAD, -2
.equ     FREE, -1
   
/* ParaNut Custom Registers and Instructions */
#include "custom_RV32I.S"
 
/*
 * Naming conventions in this file:
 *
 * a0 contains address of owner_ID or error value on return. Only the Helpers at
 * the bottom are allowed to meddle with it.
 *
 * a1 contains ID of executing core.
 *
 * t0 is copy of owner_ID. It takes values DEAD, FREE, or ID of the CPU owning
 * the lock.
 *
 * t1 indicates if original value has been touched when trying to write the copy 
 * back to memory. This will lead to occ_error.
 *
 * Unexpected values of t0 will lead to param_error.
 *
 * t2 is DEAD, t3 is FREE.
 */

/*Functions********************************************************************/

init_as:
   
   lr.w  t0,         (a0)           /* load owner_ID into t0                  */
   li    t0,         FREE           /* bring me to live (wake me up inside)   */
   j                 store          /* store back and evaluate success        */
   
/*----------------------------------------------------------------------------*/

trylock_as:

   li    t3,         FREE           /* load FREE                              */
   lr.w  t0,         (a0)           /* load owner_ID into t0                  */
   bne   t0,   t3,   occ_error      /* if lock is not free, return occerr     */
   mv    t0,         a1             /* set owner_ID to core ID                */
   j                 store          /* store back and evaluate success        */  
   
/*----------------------------------------------------------------------------*/

unlock_as:

   lr.w  t0,         (a0)           /* load owner_ID into t0                  */
   bne   t0,   a1,   param_error    /* if lock not CPU owned, param_error     */
   li    t0,         FREE           /* set owner_ID to FREE                   */
   j                 store          /* store back and evaluate success        */
   
/*----------------------------------------------------------------------------*/
   
destroy_as:
   
   li    t3,         FREE           /* load FREE                              */
   lr.w  t0,         (a0)           /* load owner_ID into t0                  */
   beq   t0,   t3,   free_or_mine   /* if lock is free, skip next instruction */
   bne   t0,   a1,   param_error    /* if lock not CPU owned, return paramerr */
free_or_mine:                       /* reached this if lock is free or mine   */
   li    t0,         DEAD           /* kill lock                              */
   j                 store          /* store back and evaluate success        */   

/*Helpers**********************************************************************/

store:
   sc.w  t1,   t0,   (a0)           /* write back owner ID                    */
   bnez  t1,   occ_error            /* jump if we did not get reservation     */          
   j     success_return             /* successful execution                   */

/*----------------------------------------------------------------------------*/

success_return:
   li    a0,         0              /* load return value for success          */
   ret

/*----------------------------------------------------------------------------*/

param_error:
   li    a0,         1              /* load return value for parameter error  */
   ret

/*----------------------------------------------------------------------------*/

occ_error:
   li    a0,         2              /* load return value for occupation error */
   ret
   
#endif /* !DOXYGEN */

/*EOF**************************************************************************/
