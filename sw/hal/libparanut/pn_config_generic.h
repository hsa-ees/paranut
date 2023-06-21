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
 * \file
 * \brief      See Documentaion of \ref co.
 * 
 * \internal
 * Please note that the file that is documented here is not the same file as the
 * pn_config.h that is created by the \ref Makefile.
 * This file is here for throwing errors when libparanut is compiled wrong. The
 * pn_config.h that is created by the \ref Makefile contains the actual defines
 * that are then used by libparanut and by application.
 * \endinternal
 */

/*Compile Time Parameters******************************************************/

/**
 * \defgroup   co libparanut Compile Time Parameters
 * \brief      Group contains defines that inform the application writer how the
 *             libparanut was compiled. 
 *
 * The libparanut is a very flexible piece of software. Some modules may have 
 * been compiled in, others may not. The cache line size could have a fixed
 * value to improve speed, or it could be set on auto which is more compatible.
 * Find out by including pn_config.h in your application and checking the
 * defines listed in here!
 */
 
/**
 * \addtogroup co
 * @{
 */
 
/* 
 * The weird #if DOXYGEN is done because Doxygen won't document it otherwise. 
 * Sorry about that.
 * If you find a more elegant solution, do not hesitate to put it in :)
 */
 
/**
 * \def        PN_CACHE_LINESIZE
 * \brief      Size of a cache line in bit.
 * 
 * This decides which assembly file was included during compilation of the
 * \ref ca. If "auto" was chosen, the file that contains functions for all 
 * possible cache line sizes is included. This means great binary compatibility,
 * but terribly big code size. When your application is deployed, you should
 * definitely compile and link a version of libparanut with this parameter set
 * to the cache line size you want to use eventually.
 * 
 * Also check documentation of \ref pn_cache_RV32I_buildscript.py.
 */
 
#if DOXYGEN

   #define PN_CACHE_LINESIZE
   
#endif

#ifndef PN_CACHE_LINESIZE

   #error PN_CACHE_LINESIZE undefined! Check "System Parameters" Documentation!
   
#endif

/**
 * \def        PN_RWIDTH
 * \brief      Register width in bit.
 * 
 * Was set to 32 bit in this documentation to enable Doxygen to properly write
 * down the typedefs in the Typedefs section of paranut.h. This should not 
 * be of interest at the moment since there is only a 32 bit version of the 
 * ParaNut, but it may become relevant in the future.
 */
 
#if DOXYGEN

   #define PN_RWIDTH 32
   
#endif

#ifndef PN_RWIDTH

   #error PN_RWIDTH undefined! Check "System Parameters" Documentation!

#endif

/**
 * \def        PN_COMPILE_RAW 
 * \brief      All security checks in libparanut are dropped if this is set to 
 *             1.
 * 
 * Since functions in this library may be timing critical, you can compile the
 * libparanut with this parameter and disable all the security checks.
 *  
 * While you are developing, it is recommended you don't do this, as the
 * security checks will tell you when you are giving input that does not make 
 * sense. You can enable it when you properly tested your system to get optimal 
 * performance. Or don't. I mean, it's not like I'm the code police.
 */
 
#if DOXYGEN

   #define PN_COMPILE_RAW

#endif

/**
 * \def        PN_WITH_BASE
 * \brief      libparanut was compiled with \ref ba. Also check 
 *             \ref modules for more information.
 */
 
#if DOXYGEN

   #define PN_WITH_BASE

#endif

/**
 * \def        PN_WITH_CACHE
 * \brief      libparanut was compiled with \ref ca. Also check 
 *             \ref modules for more information.
 */
 
#if DOXYGEN

   #define PN_WITH_CACHE

#endif

/**
 * \def        PN_WITH_LINK
 * \brief      libparanut was compiled with \ref li. Also check 
 *             \ref modules for more information.
 */
 
#if DOXYGEN

   #define PN_WITH_LINK

#endif

/**
 * \def        PN_WITH_THREAD
 * \brief      libparanut was compiled with \ref th. Also check 
 *             \ref modules for more information.
 */
 
#if DOXYGEN

   #define PN_WITH_THREAD

#endif

/**
 * \def        PN_WITH_EXCEPTION
 * \brief      libparanut was compiled with \ref ex. Also check 
 *             \ref modules for more information.
 */
 
#if DOXYGEN

   #define PN_WITH_EXCEPTION

#endif

/**
 * \def        PN_WITH_SPINLOCK
 * \brief      libparanut was compiled with \ref sp. Also check 
 *             \ref modules for more information.
 */
 
#if DOXYGEN

   #define PN_WITH_SPINLOCK

#endif

/**
 * @}
 */

/*EOF**************************************************************************/
