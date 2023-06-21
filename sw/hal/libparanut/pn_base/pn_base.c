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
 * \dir        pn_base
 * \brief      Contains \ref ba.
 */

/**
 * \file
 * \brief      Contains architecture independent implementations of the \ref ba
 *             functions.
 * 
 * \internal
 * Functions with suffix \_as are architecture specific and therefore 
 * implemented in the pn_base_\$(PN_ISA).S file in the same directory.
 * \endinternal
 */

/*Includes*********************************************************************/

#include "common.h"

/*Assembly Functions***********************************************************/

extern uint64_t ticks_as(void);
extern int freq_as(void);
extern int timebase_as(void);

/*Local Defines****************************************************************/

/*Static Functions*************************************************************/

/*Module Functions*************************************************************/

PN_NUMC pn_numcores(void)
{
   #ifndef PN_COMPILE_RAW
      COPU_CHECK
   #endif /* PN_COMPILE_RAW */
   
   return numcores_as();
}

PN_CMSK pn_m2cap(void)
{
   #ifndef PN_COMPILE_RAW
      COPU_CHECK
   #endif /* PN_COMPILE_RAW */
   
   return m2cap_as();
}

PN_CMSK pn_m2cap_g(PN_NUMG groupnum)
{
   #ifndef PN_COMPILE_RAW
      COPU_CHECK
   #endif /* PN_COMPILE_RAW */
   
  /**
   * \todo Group function implementation.
   */
   
   return (PN_CMSK) PN_ERR_NOIMP;
}

PN_CMSK pn_m3cap(void)
{
   #ifndef PN_COMPILE_RAW
      COPU_CHECK
   #endif /* PN_COMPILE_RAW */
   
  /**
   * \todo If other cores are ever capable of Mode 3 (and if there ever is a
   * register to get the information from), implement this properly.
   */
   
   return 1;
}

PN_CMSK pn_m3cap_g(PN_NUMG groupnum)
{
   #ifndef PN_COMPILE_RAW
      COPU_CHECK
   #endif /* PN_COMPILE_RAW */
   
  /**
   * \todo Group function implementation.
   */
   
   return (PN_CMSK) PN_ERR_NOIMP;
}

PN_CID pn_coreid(void)
{
   return coreid_as();
}

PN_CID pn_coreid_g(PN_NUMG *groupnum)
{
  /** 
   * \todo Group function implementation.
   */
   
   return PN_ERR_NOIMP;
}

void pn_halt(void)
{
   halt_as();
   return;
}

int pn_halt_CoPU(PN_CID coreid)
{
  /*
   * locals
   */
   int      i;                         /* loop counter                        */
   PN_CMSK  coremask = 2;              /* transforming ID into halt mask      */
   PN_CMSK  enabled;                   /* the already enabled cores (as mask) */
   
   #ifndef PN_COMPILE_RAW

      COPU_CHECK
   
      /* check if core ID is legal */
      if ((coreid == 0) | (coreid >= numcores_as()))
         return PN_ERR_PARAM;
   
     /** 
      * \internal
      * \todo Group selection.
      */
      if (coreid >= PN_RWIDTH)
         return PN_ERR_NOIMP;
   
   #endif /* PN_COMPILE_RAW */
   
   /* convert ID to mask */
   for (i = 1; i < coreid; i++)
      coremask = coremask << 2;
      
   /* get enabled cores from system */
   enabled = read_PNCE_as();
      
   #ifndef PN_COMPILE_RAW
   
      /* check if the core is already halted */
      if ((~enabled) & coremask)
         return PN_ERR_PARAM;
   
   #endif /* PN_COMPILE_RAW */
   
   /* disable the wanted CPU by unsetting its enable bit */   
   enable_CPU_as(enabled & ~coremask);
   
   return PN_SUCCESS;
}

int pn_halt_CoPU_m(PN_CMSK coremask)
{
  /*
   * locals
   */
   PN_CMSK  enabled;                   /* the already enabled cores (as mask) */
   
   #ifndef PN_COMPILE_RAW
   
      COPU_CHECK
   
      /* check if core mask is legal */
      if ((coremask == 0) | (coremask & 0x00000001))
         return PN_ERR_PARAM;
   
   #endif /* PN_COMPILE_RAW */
   
   /* get enabled cores from system */
   enabled = read_PNCE_as();
      
   #ifndef PN_COMPILE_RAW
   
      /* check if the cores are already halted */
      if ((~enabled) & coremask)
         return PN_ERR_PARAM;
   
   #endif /* PN_COMPILE_RAW */
   
   /* disable the wanted CPUs by unsetting their enable bit */   
   enable_CPU_as(enabled & ~coremask);
   
   return PN_SUCCESS;
}

int pn_halt_CoPU_gm(PN_CMSK *coremask_array, PN_NUMG array_size)
{
   #ifndef PN_COMPILE_RAW
      COPU_CHECK
   #endif /* PN_COMPILE_RAW */
   
  /** 
   * \internal
   * \todo Group function implementation.
   */
   
   return PN_ERR_NOIMP;
}

unsigned int pn_clock_freq(void)
{
  return freq_as();
}

unsigned int pn_timebase_us(void)
{
    return timebase_as();
}

int64_t pn_time_ns(void)
{
  /*
   * locals
   */
   int64_t ticks;                   /* number of ticks                  */
   static int64_t s_freq = 0;       /* frequency in Hz                  */   
   static int64_t s_factor = 0;     /* factor to multiply ticks with    */
   
  /**
   * \internal
   * \todo On RISCV C calling convention, long long type fits for both RV32 and
   * RV64. Does this hold true on other architectures?
   */
   
   #ifndef PN_COMPILE_RAW
      COPU_CHECK
   #endif /* PN_COMPILE_RAW */
   
  /**
   * \internal
   * time in nanoseconds = time in seconds  * 1000000000
   * frequency = ticks / second  =>  time in seconds = ticks / frequency
   * 
   * From those two, it follows that
   * time in nanoseconds = (ticks / frequency) * 1000000000
   * 
   * Which is equivalent to
   * time in nanoseconds = ticks * (1000000000 / frequency)
   * 
   * With the frequency values that the ParaNut currently has, this is accurate
   * enough.
   * 
   * \todo This might change in the future.
   */
   
   if (!s_freq)
   {
      s_freq = (int64_t)freq_as();
      s_factor = 1000000000LL / s_freq;
   }
   
   ticks = ticks_as();
   return ticks * s_factor;
}

void pn_usleep(int64_t useconds){
   int64_t soll = pn_time_ns() + useconds * 1000;
   while(pn_time_ns() < soll);
}

int pn_simulation(void)
{
   return simulation_as();
}

/*EOF**************************************************************************/
