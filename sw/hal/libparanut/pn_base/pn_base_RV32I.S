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
 *                called in \ref pn_base.c.
 *
 * \includelineno pn_base/pn_base_RV32I.S
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
   * \fn         uint64_t ticks_as(void)
   * \brief      Reads machine cycle counter.
   *
   * \return     Machine cycle counter.
   */
   uint64_t ticks_as(void) {}
   
  /**
   * \internal
   * \fn         int freq_as(void)
   * \brief      Reads pnclockinfo register which contains clock speed.
   *
   * \return     pnclockinfo register.
   */
   int freq_as(void) {}

  /**
   * \internal
   * \fn         int timebase_as(void)
   * \brief      Reads pntimebase register which contains machine timer timebase.
   *
   * \return     pntimebase register.
   */
   int timebase_as(void) {}
   
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
.globl   ticks_as
.globl   freq_as
.global  timebase_as

/* ParaNut Custom Registers and Instructions */
#include "custom_RV32I.S"

/*Functions********************************************************************/

ticks_as:

   csrr  a0,   mcycle                  /* put mcycle in return value          */
   csrr  a1,   mcycleh                 /* put mcycleh in second return value  */
   ret                                 /* return                              */
   
/*----------------------------------------------------------------------------*/
   
freq_as:

   csrr  a0,   pnclockinfo             /* put pnclockinfo in return value     */
   ret                                 /* return                              */
   
/*----------------------------------------------------------------------------*/
   
timebase_as:

   csrr  a0,   pntimebase              /* put pntimebase in return value      */
   ret                                 /* return                              */
   
#endif /* !DOXYGEN */
   
/*EOF**************************************************************************/
