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
 * \dir        pn_spinlock
 * \brief      Contains \ref sp.
 */


/**
 * \file
 * \brief      Contains architecture independent implementations of the \ref sp
 *             functions.
 * 
 * \internal
 * Functions with suffix \_as are architecture specific and therefore 
 * implemented in the pn_spinlock_\$(PN_ISA).S file in the same directory.
 * \endinternal
 */

/*Includes*********************************************************************/

#include "common.h"
#include <stddef.h>

/*Assembly Functions***********************************************************/

extern int  init_as(_pn_spinlock *spinlock);
extern int  trylock_as(_pn_spinlock *spinlock, PN_CID coreid);
extern int  unlock_as(_pn_spinlock *spinlock, PN_CID coreid);
extern int  destroy_as(_pn_spinlock *spinlock, PN_CID coreid);

/*Local Defines****************************************************************/

/*Static Functions*************************************************************/

/*Module Functions*************************************************************/

int pn_spinlock_init(_pn_spinlock *spinlock)
{
   #ifndef PN_COMPILE_RAW
      if (spinlock == NULL)
         return PN_ERR_PARAM;
   #endif /* PN_COMPILE_RAW */
      
   switch (init_as(spinlock))
   {
      case 0:  return PN_SUCCESS;
      case 2:  return PN_ERR_LOCKOCC;
      default: return PN_ERR_NOIMP;
   }
}

int pn_spinlock_lock(_pn_spinlock *spinlock)
{
  /*
   * locals
   */
   PN_CID coreid;                   /* saved core ID                          */
   
   #ifndef PN_COMPILE_RAW
      if (spinlock == NULL)
         return PN_ERR_PARAM;
   #endif /* PN_COMPILE_RAW */
   
   coreid = coreid_as();
   
   while (trylock_as(spinlock, coreid) != 0) {}
   
   return PN_SUCCESS;
}

int pn_spinlock_trylock(_pn_spinlock *spinlock)
{   
   #ifndef PN_COMPILE_RAW
      if (spinlock == NULL)
         return PN_ERR_PARAM;
   #endif /* PN_COMPILE_RAW */
   
   switch (trylock_as(spinlock, coreid_as()))
   {
      case 0:  return PN_SUCCESS;
      case 1:  return PN_ERR_PARAM;
      case 2:  return PN_ERR_LOCKOCC;
      default: return PN_ERR_NOIMP;
   }
}

int pn_spinlock_unlock(_pn_spinlock *spinlock)
{   
   #ifndef PN_COMPILE_RAW
      if (spinlock == NULL)
         return PN_ERR_PARAM;
   #endif /* PN_COMPILE_RAW */
 
   switch (unlock_as(spinlock, coreid_as()))
   {
      case 0:  return PN_SUCCESS;
      case 1:  return PN_ERR_PARAM;
      case 2:  
      
        /* 
         * If we got here, the lock is owned by us, but someone tried locking
         * it at the same time -> Just try again until it works.
         */
         
         while (unlock_as(spinlock, coreid_as()) != PN_SUCCESS) {}
         return PN_SUCCESS;
         
      default: return PN_ERR_NOIMP;
   }
}

int pn_spinlock_destroy(_pn_spinlock *spinlock)
{
   #ifndef PN_COMPILE_RAW
      if (spinlock == NULL)
         return PN_ERR_PARAM;
   #endif /* PN_COMPILE_RAW */
      
   switch (destroy_as(spinlock, coreid_as()))
   {
      case 0:  return PN_SUCCESS;
      case 1:  return PN_ERR_PARAM;
      case 2:  return PN_ERR_LOCKOCC;
      default: return PN_ERR_NOIMP;
   }
}

/*EOF**************************************************************************/
