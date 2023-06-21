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
 * \dir        pn_thread
 * \brief      Contains \ref th.
 */

/**
 * \file
 * \brief      Contains architecture independent implementations of the \ref th
 *             functions.
 * 
 * \internal
 * Functions with suffix \_as are architecture specific and therefore 
 * implemented in the pn_thread_\$(PN_ISA).S file in the same directory.
 * \endinternal
 */

/*Includes*********************************************************************/

#include "common.h"
/* Read CSR - Assembly Functions***********************************************/
#define read_csr(reg) ({ unsigned long __tmp; \
  __asm__ volatile ("csrr %0, " #reg : "=r"(__tmp)); \
  __tmp; })

/*Assembly Functions***********************************************************/
 
extern void set_threaded_as(PN_CMSK coremask);
extern void enter_threaded_mode_as(void);

extern void set_thread_run_as(PN_CMSK coremask, void *function, void *args);
extern void enter_thread_run_as();

/*Local Defines****************************************************************/

#define        NONE              0
#define        THREADED_MODE     1
#define        THREADED_RUN     2

/*Static Variables*************************************************************/

/*
 * Internal status variable. This module is basically a state machine.
 */
static int           s_status = NONE;

/*Static Functions*************************************************************/

/*Module Functions*************************************************************/

void pn_thread_entry()
{
   /* CePU is not supposed to execute the following part */
   if (coreid_as() == 0)
      return;
   
   switch (s_status)
   {
      case NONE:
      
         /* if there's nothing to be done, halt */
         halt_as();
         
         break;
         
      case THREADED_MODE:
         
         /* enter the threaded mode - jumps into pn_begin_threaded[_m | _gm] */
         enter_threaded_mode_as();
         
         break;
         
      case THREADED_RUN:
         enter_thread_run_as();
         
         /* not implemented yet */
         break;
         
      default:
         break;
   }         
    
   /* in case we escaped the switch case, halt CoPU here - should not happen */
   halt_as();
   return;
}

PN_CID pn_begin_threaded(PN_NUMC numcores)
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
   
   /* set status to threaded mode */
   s_status = THREADED_MODE;
   
   set_threaded_as(coremask);
   
  /* 
   * The CoPUs all got this as a return address for after they wake up. This
   * means we are practically done, just return the core ID now.
   */
   
   return coreid_as();
}

PN_CID pn_begin_threaded_m(PN_CMSK coremask)
{
   #ifndef PN_COMPILE_RAW
   
      BEGIN_THREADED_LINKED_SEC_CHECK_M
      
   #endif /* PN_COMPILE_RAW */
   
   /* set status to threaded mode */
   s_status = THREADED_MODE;
   
   set_threaded_as(coremask);
   
  /* 
   * The CoPUs all got this as a return address for after they wake up. This
   * means we are practically done, just return the core ID now.
   */
   
   return coreid_as();
}

PN_CID pn_begin_threaded_gm(PN_CMSK *coremask_array, PN_NUMG array_size)
{
  /** 
   * \internal
   * \todo Group function implementation.
   */
   
   return PN_ERR_NOIMP;
}

int pn_end_threaded(void)
{
  /* 
   * Disable all cores except CePU.
   */
   
   if (coreid_as() != 0)
   {
      /* halt this core if it is not the CePU */
      halt_as();
   }
   else
   {
      /* wait for the other cores to be disabled */
      while (read_PNCE_as() != 1) {}
      
      #ifndef PN_COMPILE_RAW
   
         if ((s_status != THREADED_MODE) || (read_PNLM_as() != 0))
         {            
            /* return matching error */
            return PN_ERR_MATCH;
         }         
      
      #endif /* PN_COMPILE_RAW */
      
      /* set status away from threaded mode */
      s_status = NONE;
   }
      
   return 0;
}

PN_CMSK pn_run_threaded(PN_NUMC numcores, void (*function)(void *args, PN_CID cid), void *args)
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
   
   /* set status to threaded mode */
   s_status = THREADED_RUN;
   
   set_thread_run_as(coremask, function, args);
   
  /* 
   * The CoPUs all got this as a return address for after they wake up. This
   * means we are practically done, just return the core ID now.
   */
   
   return coremask;
}

PN_CMSK pn_run_threaded_m(PN_CMSK coremask, void (*function)(void *args, PN_CID cid), void *args)
{
   #ifndef PN_COMPILE_RAW
   
      CREATE_THREAD_SEC_CHECK_M
      
   #endif /* PN_COMPILE_RAW */
   
   /* set status to threaded mode */
   s_status = THREADED_RUN;
   
   set_thread_run_as(coremask, function, args);
   
  /* 
   * The CoPUs all got this as a return address for after they wake up. This
   * means we are practically done, just return the core ID now.
   */
   
   return coremask;
}

PN_CMSK pn_join_thread_m(PN_CMSK coremask, uint32_t timeout){
   uint32_t time_threashold = read_csr(0xb00) + timeout;
   
   while( ( read_PNCE_as() & (coremask ^ 1))){
      if ((timeout != 0) && (read_csr(0xb00) > time_threashold)) {
         return PN_ERR_COPU;
      }
   }

   return coremask;

}

/*EOF**************************************************************************/
