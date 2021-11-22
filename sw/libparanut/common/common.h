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
 * \file
 * \brief      Contains architecture independent internal prototypes and defines needed in \ref mo.
 * 
 * Is included by all modules and includes the \ref paranut.h itself, thereby
 * is a "common layer" for all modules.
 *
 * \internal
 * \includelineno common/common.h
 */

/*Includes*********************************************************************/

#include "paranut.h"

/*Commonly Used Defines********************************************************/

/**
 * \internal
 * \defgroup   comm_def Commonly Used Defines
 * \brief      Architecture independent defines for internal usage in \ref mo.
 */
 
/**
 * \addtogroup comm_def
 * @{
 */
 
/**
 * @{
 */

/**
 * \internal
 * \def        COPU_CHECK
 * \brief      Checks if we are running on CoPU.
 * 
 * \todo       May need a change if there's ever a ParaNut implementation with
 *             more than 1 Mode 3 capable CPU.
 */
#define COPU_CHECK   if (coreid_as() != 0) return PN_ERR_COPU;

/**
 * \internal
 * \def        ALL_HALTED
 * \brief      Checks if all CoPUs are halted.
 * 
 * \todo       May need a change if there's ever a ParaNut implementation with
 *             more than 1 Mode 3 capable CPU.
 */
#define ALL_HALTED   if (read_PNCE_as() > 1) return PN_ERR_MATCH;

/**
 * \internal
 * \def        BEGIN_THREADED_LINKED_SEC_CHECK
 * \brief      Performs security checks in \ref pn_begin_linked() and \ref
 *             pn_begin_threaded().
 * 
 * \todo       Take out PN_ERR_NOIMP when group selection is implemented.
 */
#define BEGIN_THREADED_LINKED_SEC_CHECK                                        \
                     COPU_CHECK                                                \
                     ALL_HALTED                                                \
                     if ((numcores > numcores_as()) || (numcores <= 1))        \
                        return PN_ERR_PARAM;                                   \
                     if (numcores >= PN_RWIDTH)                                \
                        return PN_ERR_NOIMP;
                        
/**
 * \internal
 * \def        BEGIN_THREADED_LINKED_SEC_CHECK_M
 * \brief      Performs security checks in \ref pn_begin_linked_m() and \ref
 *             pn_begin_threaded_m().
 */
#define BEGIN_THREADED_LINKED_SEC_CHECK_M                                      \
                     COPU_CHECK                                                \
                     ALL_HALTED                                                \
                     if (!(coremask & 0b1))                                    \
                        return PN_ERR_PARAM;                                   \

/**
 * \internal
 * \def        BEGIN_LINKED_STACK_FRAME_CHECK
 * \brief      Performs stack_frame checks in \ref pn_begin_linked().
 * 
 */
 extern int shared_mem_size;
#define BEGIN_LINKED_STACK_FRAME_CHECK                                        \
                      if (frame_adr != NULL &&                                \
                          (frame_adr < stack_ptr_as() ||                      \
                           frame_adr > (stack_ptr_as() + shared_mem_size)))   \
                        return PN_ERR_PARAM;                                  \
               
/**
 * \internal
 * \def        CONVERT_NUMC_TO_MASK
 * \brief      Converts a given number of cores to a minimal bitmask.
 */
#define CONVERT_NUMC_TO_MASK                                                   \
                     for (i = 2; i < numcores; i++)                            \
                        coremask |= (1 << i);
              
/**
 * \internal
 * \def        TERMNL
 * \brief      Terminal newline.
 */
#define TERMNL       "\n\r"

/**
 * @}
 */
 
/**
 * @}
 */

/*Global Variables*************************************************************/

/**
 * \internal
 * \defgroup   comm_glo Global Variables
 * \brief      Global Variables for internal usage in \ref mo.
 */
 
/**
 * \addtogroup comm_glo
 * @{
 */
 
/**
 * @{
 */

/**
 * \internal
 * \var        sp_loc
 * \brief      Stack Pointer location, used by \ref set_linked_as() and 
 *             \ref set_threaded_as().
 */
int                  sp_loc;

/**
 * \internal
 * \var        tp_loc
 * \brief      Thread Pointer location, used in \ref th.
 */
int                  tp_loc;

/**
 * @}
 */
 
/**
 * @}
 */

/*Commonly Used Assembly Functions*********************************************/

extern void          halt_as(void);
extern PN_NUMC       numcores_as(void);
extern PN_CID        coreid_as(void);
extern PN_CMSK       read_PNCE_as(void);
extern PN_CMSK       read_PNLM_as(void);
extern void          enable_CPU_as(PN_CMSK coremask);
extern PN_CMSK       m2cap_as(void);
extern int           simulation_as(void);

/*EOF**************************************************************************/
