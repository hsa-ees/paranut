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
 * \dir        pn_link
 * \brief      Contains \ref li.
 */


/**
 * \file
 * \brief      Contains architecture independent implementations of the \ref li
 *             functions.
 * 
 * \internal
 * Functions with suffix \_as are architecture specific and therefore 
 * implemented in the pn_link_\$(PN_ISA).S file in the same directory.
 * \endinternal
 */

/*Includes*********************************************************************/

#include "common.h"

/*Assembly Functions***********************************************************/

extern PN_CID  set_linked_as(PN_CMSK coremask);
extern PN_CMSK read_PNLM_as(void);
extern void    write_PNLM_as(PN_CMSK coremask);

/*Local Defines****************************************************************/

/*Static Functions*************************************************************/

/*Module Functions*************************************************************/

PN_CID pn_begin_linked(PN_NUMC numcores)
{
  /*
   * locals
   */
   int      i;                         /* loop counter and error value        */
   PN_CMSK  coremask = 0b11;           /* transforming numcores to mask       */
   
   #ifndef PN_COMPILE_RAW
   
      BEGIN_THREADED_LINKED_SEC_CHECK
         
   #endif /* PN_COMPILE_RAW */
   
   CONVERT_NUMC_TO_MASK
   
   /* start linked execution */ 
   return set_linked_as(coremask);
}

PN_CID pn_begin_linked_m(PN_CMSK coremask)
{
   #ifndef PN_COMPILE_RAW
   
      BEGIN_THREADED_LINKED_SEC_CHECK_M
      
   #endif /* PN_COMPILE_RAW */
   
   /* start linked execution */
   return set_linked_as(coremask);
}

PN_CID pn_begin_linked_gm(PN_CMSK *coremask_array, PN_NUMG array_size)
{
  /** 
   * \internal
   * \todo Group function implementation.
   */
   
   return PN_ERR_NOIMP;
}

int pn_end_linked(void)
{
  /*
   * locals
   */
   #ifndef PN_COMPILE_RAW
      PN_CMSK  enabled;                   /* the enabled cores (as mask)      */
   
      enabled = read_PNCE_as();
   
      if (enabled <= 1) 
         return PN_ERR_MATCH;
         
      if (enabled != read_PNLM_as())
         return PN_ERR_MATCH;
      
   #endif /* PN_COMPILE_RAW */
   
   /* disable all cores except CePU */
   enable_CPU_as((PN_CMSK) 0b1);
   
   /* set linked register to all 0's */
   write_PNLM_as((PN_CMSK) 0b0);
   
   return PN_SUCCESS;
}

/*EOF**************************************************************************/
