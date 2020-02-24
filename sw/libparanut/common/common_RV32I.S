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
 *                called in all modules.
 *
 * \includelineno common/common_RV32I.S
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
   * \fn         void halt_as(void)
   * \brief      Halts the core this is executed on.
   */
   void halt_as(void) {}

  /**
   * \internal
   * \fn         PN_NUMC numcores_as(void)
   * \brief      Get the number of cores in your system.
   *
   * \return     The number of cores in your ParaNut implementation.
   */
   PN_NUMC numcores_as(void) {}

  /**
   * \internal
   * \fn         PN_CID coreid_as(void)
   * \brief      Get the core ID.
   *
   * \return     The core ID.
   */
   PN_CID coreid_as(void) {}

  /**
   * \internal
   * \fn         PN_CMSK read_PNCE_as(void)
   * \brief      Reads ParaNut CPU enable register.
   * 
   * \return     Mask that represents enabled CPUs.
   */
   PN_CMSK read_PNCE_as(void) {}

  /**
   * \internal
   * \fn         void enable_CPU_as(PN_CMSK coremask)
   * \brief      Enables the cores marked in the bitmask, disables the other ones.
   * 
   * Has to check stability, too.
   */
   void enable_CPU_as(PN_CMSK coremask) {}

  /**
   * \internal
   * \fn         PN_CMSK m2cap_as(void)
   * \brief      Get the register where Mode 2 capability cores are marked.
   *
   * \return     The core mask.
   */
   PN_CMSK m2cap_as(void) {}
   
  /**
   * \internal
   * \fn         int simulation_as(void)
   * \brief      Checks if we run in simulation instead of real hardware.
   *
   * \return     Zero if we run on hardware, non-zero if we run in simulation.
   */
   int simulation_as(void) {}

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
.globl   halt_as
.globl   numcores_as
.globl   coreid_as
.globl   read_PNCE_as
.globl   enable_CPU_as
.globl   m2cap_as
.globl   simulation_as

/* ParaNut Custom Registers and Instructions */
#include "custom_RV32I.S"

/*Functions********************************************************************/

halt_as:

   fence                               /* make sure memory is synchronized    */
   HALT                                /* halt executing core                 */
   ret                                 /* return                              */
   
/*----------------------------------------------------------------------------*/

numcores_as:

   csrr  a0,   pncpus                  /* put ncpus in return value           */
   ret                                 /* return                              */
   
/*----------------------------------------------------------------------------*/

coreid_as:

   csrr  a0,   mhartid                 /* put mhartid in return value         */
   ret                                 /* return                              */
   
/*----------------------------------------------------------------------------*/

read_PNCE_as:

   csrr  a0,   pnce                    /* read pnce, store in return value    */
   ret                                 /* return                              */

/*----------------------------------------------------------------------------*/

enable_CPU_as:

   fence                               /* synchronize memory before enabling  */
   csrw  pnce, a0                      /* sets pnce to passed bitmask         */
   ret                                 /* return                              */
 
/*----------------------------------------------------------------------------*/
   
m2cap_as:

   csrr  a0,   pnm2cap                 /* put pnm2cap in return value         */
   ret                                 /* return                              */
   
/*----------------------------------------------------------------------------*/
   
simulation_as:

   la    t0,   _thread_data            /* get address of thread data          */
   li    a0,   0                       /* clear a0                            */
   lb    a0,   0(t0)                   /* load first byte from thread data    */
   xori  a0,   a0,      'S'            /* check if it is an 'S'               */
   beqz  a0,   is_sim                  /* if a0 is 0, it is a simulation      */
   
is_not_sim:                            /* we got here if we run on hardware   */
   li    a0,   0                       /* put zero as return value            */
   ret                                 /* return                              */
   
is_sim:                                /* we got here if we run in simulation */
   li    a0,   1                       /* put non-zero as return value        */
   ret                                 /* return                              */
   
#endif /* !DOXYGEN */
   
/*EOF**************************************************************************/
