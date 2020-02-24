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
 * \dir        pn_exception
 * \brief      Contains \ref ex.
 */

/**
 * \file
 * \brief      Contains (somewhat) architecture independent implementations of 
 *             the \ref ex functions.
 * 
 * The exception module is not exactly architecture independent since it
 * implements a RISC-V exception table.
 * 
 * \todo Layer this better in later versions of libparanut.
 * 
 * \internal
 * Functions with suffix \_as are architecture specific and therefore 
 * implemented in the pn_exception_\$(PN_ISA).S file in the same directory.
 * \endinternal
 */

/*Includes*********************************************************************/

#include "common.h"
#include <stdio.h>                     /* printf() in default handler         */

/*Assembly Functions***********************************************************/

extern void          exception_init_as(void);
extern void          exception_entry_as(void);
extern PN_CMSK       read_PNX_as(void);
extern PN_CMSK       read_PNCAUSE_as(void);
extern PN_CMSK       read_PNEPC_as(void);
extern unsigned int  read_MTVAL_as(void);
extern void          write_PNXSEL_as(PN_CMSK coremask);
extern void          write_MSTATUS_as(unsigned int register);
extern void          ecall_as(void);
extern void          progress_mepc_as(void);

/*Static Functions*************************************************************/

static void default_exc(unsigned int cause, 
                        unsigned int program_counter, 
                        unsigned int mtval)
{   
  /*
   * locals
   */ 
   unsigned int   pnx;                 /* representation of pnx register      */
   int            i;                   /* loop counter                        */
   
   if(cause == 3)
   {
      
     /*
      * If breakpoint exception landed in trap handler, we are not being
      * debugged. This should not really happen.
      * -> Just returning.
      */
		
      return;
	}
   
	if(cause == 16)
   {	
      
     /*
      * If CoPU exception, gather data from all CPUs that threw an exception.
      */
      
      pnx = read_PNX_as();
      			 
		printf("CoPU Exception!"                             TERMNL
             "   Cause ID                      : %u      " TERMNL
             "   Address of current instruction: 0x%08X  " TERMNL
             "   Exception for cores           : %u      " TERMNL
             "   Currently enabled cores:      : %u      " TERMNL TERMNL,
             cause, program_counter, pnx, (unsigned int)read_PNCE_as());
		
		/* print exception information of all failing CoPUs */
		for (i = 1; i < numcores_as(); i++)
      {
			if (pnx & (1 << i)) 
         {
				write_PNXSEL_as(1 << i);
				printf("Information for CoPU %d:                   " TERMNL
                   "   Cause ID                       : %u     " TERMNL
                   "   Address of current instruction : 0x%08X " TERMNL
                   "   Machine Trap Value register    : 0x%08X " TERMNL TERMNL,
                   i, 
                   (unsigned int)read_PNCAUSE_as(), 
                   (unsigned int)read_PNEPC_as(), 
                   mtval);
			}		
		}
	}
   else
   {
      
     /*
      * For all other exceptions, just print out information.
      */
      
		printf("Exception!"                                  TERMNL
             "   Cause ID                       : %u     " TERMNL
             "   Address of current instruction : 0x%08X " TERMNL
             "   Machine Trap Value register    : 0x%08X " TERMNL TERMNL,
             cause, program_counter, mtval);
		
	}
   
  /*
   * Halt all CPUs.
   */
   
	enable_CPU_as(0);	
   return;
}

/*----------------------------------------------------------------------------*/

static void default_int(unsigned int cause, 
                        unsigned int program_counter, 
                        unsigned int mtval)
{
	printf("Interrupt!                               " TERMNL
          "   Cause ID                    : %u      " TERMNL
          "   Address of next instruction : 0x%08X  " TERMNL
          "   Machine Trap Value register : 0x%08X  " TERMNL TERMNL, 
          cause, program_counter, mtval);
}

/*Local Defines****************************************************************/

/*
 * List of all function pointers used in this module.
 */

static void (*handle_exc_0)(unsigned int, unsigned int, unsigned int)   = &default_exc;
static void (*handle_exc_1)(unsigned int, unsigned int, unsigned int)   = &default_exc;
static void (*handle_exc_2)(unsigned int, unsigned int, unsigned int)   = &default_exc;
static void (*handle_exc_3)(unsigned int, unsigned int, unsigned int)   = &default_exc;
static void (*handle_exc_4)(unsigned int, unsigned int, unsigned int)   = &default_exc;
static void (*handle_exc_5)(unsigned int, unsigned int, unsigned int)   = &default_exc;
static void (*handle_exc_6)(unsigned int, unsigned int, unsigned int)   = &default_exc;
static void (*handle_exc_7)(unsigned int, unsigned int, unsigned int)   = &default_exc;
static void (*handle_exc_8)(unsigned int, unsigned int, unsigned int)   = &default_exc;
static void (*handle_exc_9)(unsigned int, unsigned int, unsigned int)   = &default_exc;
static void (*handle_exc_10)(unsigned int, unsigned int, unsigned int)  = &default_exc;
static void (*handle_exc_11)(unsigned int, unsigned int, unsigned int)  = &default_exc;
static void (*handle_exc_12)(unsigned int, unsigned int, unsigned int)  = &default_exc;
static void (*handle_exc_13)(unsigned int, unsigned int, unsigned int)  = &default_exc;
static void (*handle_exc_14)(unsigned int, unsigned int, unsigned int)  = &default_exc;
static void (*handle_exc_15)(unsigned int, unsigned int, unsigned int)  = &default_exc;
static void (*handle_exc_16)(unsigned int, unsigned int, unsigned int)  = &default_exc;
static void (*handle_exc_def)(unsigned int, unsigned int, unsigned int) = &default_exc;

static void (*handle_int_0)(unsigned int, unsigned int, unsigned int)   = &default_int;
static void (*handle_int_1)(unsigned int, unsigned int, unsigned int)   = &default_int;
static void (*handle_int_2)(unsigned int, unsigned int, unsigned int)   = &default_int;
static void (*handle_int_3)(unsigned int, unsigned int, unsigned int)   = &default_int;
static void (*handle_int_4)(unsigned int, unsigned int, unsigned int)   = &default_int;
static void (*handle_int_5)(unsigned int, unsigned int, unsigned int)   = &default_int;
static void (*handle_int_6)(unsigned int, unsigned int, unsigned int)   = &default_int;
static void (*handle_int_7)(unsigned int, unsigned int, unsigned int)   = &default_int;
static void (*handle_int_8)(unsigned int, unsigned int, unsigned int)   = &default_int;
static void (*handle_int_9)(unsigned int, unsigned int, unsigned int)   = &default_int;
static void (*handle_int_10)(unsigned int, unsigned int, unsigned int)  = &default_int;
static void (*handle_int_11)(unsigned int, unsigned int, unsigned int)  = &default_int;
static void (*handle_int_def)(unsigned int, unsigned int, unsigned int) = &default_int;

/*Entry Point Functions********************************************************/

void handle_exc(unsigned int cause, unsigned int program_counter)
{
  /*
   * locals
   */
   unsigned int mtval = read_MTVAL_as();
   
   switch (cause)
   {
      case 0:  /* Instruction address misaligned                              */
         (*handle_exc_0)(cause, program_counter, mtval);
         break;
         
      case 1:  /* Instruction access fault                                    */
         (*handle_exc_1)(cause, program_counter, mtval);
         break;
      
      case 2:  /* Illegal instruction                                         */
         (*handle_exc_2)(cause, program_counter, mtval);
         break;
      
      case 3:  /* Breakpoint                                                  */
         (*handle_exc_3)(cause, program_counter, mtval);
         break;
      
      case 4:  /* Load address misaligned                                     */
         (*handle_exc_4)(cause, program_counter, mtval);
         break;
      
      case 5:  /* Load access fault                                           */
         (*handle_exc_5)(cause, program_counter, mtval);
         break;
      
      case 6:  /* Store/AMO address misaligned                                */
         (*handle_exc_6)(cause, program_counter, mtval);
         break;
      
      case 7:  /* Store/AMO access fault                                      */
         (*handle_exc_7)(cause, program_counter, mtval);
         break;
      
      case 8:  /* Environment call from U-mode                                */
         (*handle_exc_8)(cause, program_counter, mtval);
         break;
      
      case 9:  /* Environment call from S-mode                                */
         (*handle_exc_9)(cause, program_counter, mtval);
         break;
      
      case 10: /* Reserved - Not yet included in RISC-V                       */
         (*handle_exc_10)(cause, program_counter, mtval);
         break;
      
      case 11: /* Environment call from M-mode                                */
         (*handle_exc_11)(cause, program_counter, mtval);
         break;
      
      case 12: /* Instruction page fault                                      */
         (*handle_exc_12)(cause, program_counter, mtval);
         break;
      
      case 13: /* Load page fault                                             */
         (*handle_exc_13)(cause, program_counter, mtval);
         break;
      
      case 14: /* Reserved - Not yet included in RISC-V                       */
         (*handle_exc_14)(cause, program_counter, mtval);
         break;
         
      case 15: /* Store/AMO page fault                                        */
         (*handle_exc_15)(cause, program_counter, mtval);
         break;
      
      case 16: /* ParaNut CoPU exception (ususally reserved in RISC-V)        */
         (*handle_exc_16)(cause, program_counter, mtval);
         break;
      
      default:
         (*handle_exc_def)(cause, program_counter, mtval);
         break;
   }			
   
   return;
}

/*----------------------------------------------------------------------------*/

void handle_int(unsigned int cause, unsigned int program_counter)
{  
  /*
   * locals
   */
   unsigned int mtval = read_MTVAL_as();
   
   switch (cause)
   {
      case 0:  /* User software interrupt                                     */
         (*handle_int_0)(cause, program_counter, mtval);
         break;
         
      case 1:  /* Supervisor software interrupt                               */
         (*handle_int_1)(cause, program_counter, mtval);
         break;
      
      case 2:  /* Reserved - Not yet included in RISC-V                       */
         (*handle_int_2)(cause, program_counter, mtval);
         break;
      
      case 3:  /* Machine software interrupt                                  */
         (*handle_int_3)(cause, program_counter, mtval);
         break;
      
      case 4:  /* User timer interrupt                                        */
         (*handle_int_4)(cause, program_counter, mtval);
         break;
      
      case 5:  /* Supervisor timer interrupt                                  */
         (*handle_int_5)(cause, program_counter, mtval);
         break;
      
      case 6:  /* Reserved - Not yet included in RISC-V                       */
         (*handle_int_6)(cause, program_counter, mtval);
         break;
      
      case 7:  /* Machine timer interrupt                                     */
         (*handle_int_7)(cause, program_counter, mtval);
         break;
      
      case 8:  /* User external interrupt                                     */
         (*handle_int_8)(cause, program_counter, mtval);
         break;
      
      case 9:  /* Supervisor external interrupt                               */
         (*handle_int_9)(cause, program_counter, mtval);
         break;
      
      case 10: /* Reserved - Not yet included in RISC-V                       */
         (*handle_int_10)(cause, program_counter, mtval);
         break;
      
      case 11: /* Machine external interrupt                                  */
         (*handle_int_11)(cause, program_counter, mtval);
         break;
      
      default:
         (*handle_int_def)(cause, program_counter, mtval);
         break;
   }			
   
   return;
}

/*Module Functions*************************************************************/

void pn_exception_init(void)
{
   exception_init_as();
   return;   
}

/*----------------------------------------------------------------------------*/

int pn_exception_set_handler(
                              void (*handler)(
                                 unsigned int cause, 
                                 unsigned int program_counter, 
                                 unsigned int mtval),
                              unsigned int exception_code
                            )
{
   #ifndef PN_COMPILE_RAW
   
      if (handler == NULL)
         return PN_ERR_PARAM;

   #endif /* !PN_COMPILE_RAW */
   
   if (exception_code > 0)
   {
      switch (exception_code)
      {
         case 0:  /* Instruction address misaligned                           */
            handle_exc_0 = handler;
            break;
         
         case 1:  /* Instruction access fault                                 */
            handle_exc_1 = handler;
            break;
      
         case 2:  /* Illegal instruction                                      */
            handle_exc_2 = handler;
            break;
      
         case 3:  /* Breakpoint                                               */
            handle_exc_3 = handler;
            break;
      
         case 4:  /* Load address misaligned                                  */
            handle_exc_4 = handler;
            break;
      
         case 5:  /* Load access fault                                        */
            handle_exc_5 = handler;
            break;
      
         case 6:  /* Store/AMO address misaligned                             */
            handle_exc_6 = handler;
            break;
      
         case 7:  /* Store/AMO access fault                                   */
            handle_exc_7 = handler;
            break;
      
         case 8:  /* Environment call from U-mode                             */
            handle_exc_8 = handler;
            break;
      
         case 9:  /* Environment call from S-mode                             */
            handle_exc_9 = handler;
            break;
      
         case 10: /* Reserved - Not yet included in RISC-V                    */
            handle_exc_10 = handler;
            break;
      
         case 11: /* Environment call from M-mode                             */
            handle_exc_11 = handler;
            break;
      
         case 12: /* Instruction page fault                                   */
            handle_exc_12 = handler;
            break;
      
         case 13: /* Load page fault                                          */
            handle_exc_13 = handler;
            break;
      
         case 14: /* Reserved - Not yet included in RISC-V                    */
            handle_exc_14 = handler;
            break;
         
         case 15: /* Store/AMO page fault                                     */
            handle_exc_15 = handler;
            break;
      
         case 16: /* ParaNut CoPU exception (ususally reserved in RISC-V)     */
            handle_exc_16 = handler;
            break;
      
         default:
            return PN_ERR_EXC;
      }
   }
   
   else
   {
      exception_code = exception_code & (0xF);
      switch (exception_code)
      {
         case 0:  /* User software interrupt                                  */
            handle_int_0 = handler;
            break;
            
         case 1:  /* Supervisor software interrupt                            */
            handle_int_1 = handler;
            break;
         
         case 2:  /* Reserved - Not yet included in RISC-V                    */
            handle_int_2 = handler;
            break;
         
         case 3:  /* Machine software interrupt                               */
            handle_int_3 = handler;
            break;
         
         case 4:  /* User timer interrupt                                     */
            handle_int_4 = handler;
            break;
         
         case 5:  /* Supervisor timer interrupt                               */
            handle_int_5 = handler;
            break;
         
         case 6:  /* Reserved - Not yet included in RISC-V                    */
            handle_int_6 = handler;
            break;
         
         case 7:  /* Machine timer interrupt                                  */
            handle_int_7 = handler;
            break;
         
         case 8:  /* User external interrupt                                  */
            handle_int_8 = handler;
            break;
         
         case 9:  /* Supervisor external interrupt                            */
            handle_int_9 = handler;
            break;
         
         case 10: /* Reserved - Not yet included in RISC-V                    */
            handle_int_10 = handler;
            break;
         
         case 11: /* Machine external interrupt                               */
            handle_int_11 = handler;
            break;
         
         default:
            return PN_ERR_EXC;
      }	
   }
      
   return PN_SUCCESS;   
}

/*----------------------------------------------------------------------------*/

void pn_ecall(void)
{
   ecall_as();
   return;
}

/*----------------------------------------------------------------------------*/

void pn_interrupt_enable(void)
{
   /* set Bit 3 to 1 - Machine Interrupt Enable */
   write_MSTATUS_as(0x00000008);
   return;   
}

/*----------------------------------------------------------------------------*/

void pn_interrupt_disable(void)
{
   /* set Bit 3 to 0 - Machine Interrupt Enable */
   write_MSTATUS_as(0x00000000);
   return;   
}

/*----------------------------------------------------------------------------*/

void pn_progress_mepc(void)
{
   progress_mepc_as();
   return;
}

/*EOF**************************************************************************/
