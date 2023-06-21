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
 * \dir        pn_cache
 * \brief      Contains \ref ca.
 */


/**
 * \file
 * \brief      Contains architecture independent implementations of the \ref ca
 *             functions.
 * 
 * \internal
 * Functions with suffix \_as are architecture specific and therefore 
 * implemented in the pn_cache_\$(PN_ISA).S file in the same directory.
 * 
 * For RV32I architecture, it was not practical to implement the assembly code
 * by hand. A buildscript called \ref pn_cache_RV32I_buildscript.py was created
 * instead which takes the \ref PN_CACHE_LINESIZE parameter and creates a
 * correct version of the pn_cache_RV32I_nnn.S (example: \ref 
 * pn_cache_RV32I_auto.S) file. For reasons on why this
 * was done, see the documentation of \ref pn_cache_RV32I_buildscript.py itself.
 * \endinternal
 */

/*Includes*********************************************************************/

#include "common.h"

/*Assembly Functions***********************************************************/

extern void enable_cache_as(void);
extern void disable_cache_as(void);
extern unsigned int cache_banks_as(void);
extern unsigned int cache_sets_as(void);
extern unsigned int mem_size_as(void);

extern void invalidate_32_as(unsigned int addr, unsigned int endaddr);
extern void invalidate_bulk_32_as(unsigned int addr, unsigned int endaddr);
extern void writeback_32_as(unsigned int addr, unsigned int endaddr);
extern void writeback_bulk_32_as(unsigned int addr, unsigned int endaddr);
extern void flush_32_as(unsigned int addr, unsigned int endaddr);
extern void flush_bulk_32_as(unsigned int addr, unsigned int endaddr);

extern void invalidate_64_as(unsigned int addr, unsigned int endaddr);
extern void invalidate_bulk_64_as(unsigned int addr, unsigned int endaddr);
extern void writeback_64_as(unsigned int addr, unsigned int endaddr);
extern void writeback_bulk_64_as(unsigned int addr, unsigned int endaddr);
extern void flush_64_as(unsigned int addr, unsigned int endaddr);
extern void flush_bulk_64_as(unsigned int addr, unsigned int endaddr);

extern void invalidate_128_as(unsigned int addr, unsigned int endaddr);
extern void invalidate_bulk_128_as(unsigned int addr, unsigned int endaddr);
extern void writeback_128_as(unsigned int addr, unsigned int endaddr);
extern void writeback_bulk_128_as(unsigned int addr, unsigned int endaddr);
extern void flush_128_as(unsigned int addr, unsigned int endaddr);
extern void flush_bulk_128_as(unsigned int addr, unsigned int endaddr);

extern void invalidate_256_as(unsigned int addr, unsigned int endaddr);
extern void invalidate_bulk_256_as(unsigned int addr, unsigned int endaddr);
extern void writeback_256_as(unsigned int addr, unsigned int endaddr);
extern void writeback_bulk_256_as(unsigned int addr, unsigned int endaddr);
extern void flush_256_as(unsigned int addr, unsigned int endaddr);
extern void flush_bulk_256_as(unsigned int addr, unsigned int endaddr);

extern void invalidate_512_as(unsigned int addr, unsigned int endaddr);
extern void invalidate_bulk_512_as(unsigned int addr, unsigned int endaddr);
extern void writeback_512_as(unsigned int addr, unsigned int endaddr);
extern void writeback_bulk_512_as(unsigned int addr, unsigned int endaddr);
extern void flush_512_as(unsigned int addr, unsigned int endaddr);
extern void flush_bulk_512_as(unsigned int addr, unsigned int endaddr);

extern void invalidate_1024_as(unsigned int addr, unsigned int endaddr);
extern void invalidate_bulk_1024_as(unsigned int addr, unsigned int endaddr);
extern void writeback_1024_as(unsigned int addr, unsigned int endaddr);
extern void writeback_bulk_1024_as(unsigned int addr, unsigned int endaddr);
extern void flush_1024_as(unsigned int addr, unsigned int endaddr);
extern void flush_bulk_1024_as(unsigned int addr, unsigned int endaddr);

extern void invalidate_2048_as(unsigned int addr, unsigned int endaddr);
extern void invalidate_bulk_2048_as(unsigned int addr, unsigned int endaddr);
extern void writeback_2048_as(unsigned int addr, unsigned int endaddr);
extern void writeback_bulk_2048_as(unsigned int addr, unsigned int endaddr);
extern void flush_2048_as(unsigned int addr, unsigned int endaddr);
extern void flush_bulk_2048_as(unsigned int addr, unsigned int endaddr);

/*Local Defines****************************************************************/

/**
 * \internal
 * \def        BULK_LINES
 * \brief      Number of lines that will at least be affected when calling the
 *             bulk functions.
 */
#define BULK_LINES  32

/**
 * \internal
 * \def        sec_check
 * \brief      Check if size is larger than actual memory.
 *
 * \todo Can these functions be used on CoPU?
 */
#define sec_check    if (size > s_mem_size)                                    \
                        return PN_ERR_PARAM;  

/*Static Variables*************************************************************/

/* overall memory size in byte */
static unsigned int  s_mem_size;

/* overall cache size in byte */
static unsigned long s_cache_size;

/* memory start address */
extern int           _start;
static void          *s_addr = &_start;

/* cache line size in bit */
static unsigned int  s_cache_linesize;

/* cache line size in byte */
static unsigned int  s_cache_linesize_byte;

/* pointer to the invalidate, writeback and flush functions */
static void          (*invalidate_as)(unsigned int addr, unsigned int endaddr);
static void          (*invalidate_bulk_as)(unsigned int addr, unsigned int endaddr);
static void          (*writeback_as)(unsigned int addr, unsigned int endaddr);
static void          (*writeback_bulk_as)(unsigned int addr, unsigned int endaddr);
static void          (*flush_as)(unsigned int addr, unsigned int endaddr);
static void          (*flush_bulk_as)(unsigned int addr, unsigned int endaddr);

/*Static Functions*************************************************************/

static int cache_action(
                  void *addr, 
                  unsigned long size, 
                  void (*singlefunc)(unsigned int addr, unsigned int endaddr), 
                  void (*bulkfunc)(unsigned int addr, unsigned int endaddr))
{
  /*
   * locals
   */
   unsigned int lines;                 /* number of affected cache lines      */
   unsigned int single_lines = 0;      /* number of affected cache lines that */
                                       /* cannot be handled in bulk           */
   unsigned int calc_addr;             /* non "smart pointer" adress          */

   #ifndef PN_COMPILE_RAW
      sec_check
   #endif
   
   /* find out number of affected cache lines */
   if ((size % s_cache_linesize_byte) != 0)
      lines = (unsigned int)(size / s_cache_linesize_byte) + 1;
   else
      lines = (unsigned int)(size / s_cache_linesize_byte);
   
   /* if passed size was 0, still pass one line */
   if (lines == 0)
      lines = 1;
   
   /* cast pointer to calculatable address */
   calc_addr = (unsigned int)addr;
   
   /* find out how many lines cannot be done in bulk */
   if ((single_lines = (lines % BULK_LINES)) != 0)
   {
      singlefunc(calc_addr, calc_addr + (single_lines * s_cache_linesize_byte));
   }
   
   /* check if there even is a bunch of lines that can be done in bulk */
   if (!(lines < BULK_LINES))
   {
      bulkfunc(calc_addr + single_lines * s_cache_linesize_byte,
               calc_addr + size); 
   }
      
   return PN_SUCCESS;
}

/*Module Functions*************************************************************/

int pn_cache_init(void)
{   
   #ifndef PN_COMPILE_RAW
   
      COPU_CHECK
      
   #endif /* PN_COMPILE_RAW */
   
  /* 
   * Initialize static variables of this module.
   */
   
   /* fill in s_cache_size and s_cache_linesize */
   s_cache_linesize_byte   = cache_banks_as() * 4;
   s_cache_linesize        = s_cache_linesize_byte * 8;
   s_cache_size            = cache_sets_as() * s_cache_linesize_byte;
   s_mem_size              = mem_size_as();
    
   /* set the function pointers correctly */
   switch (s_cache_linesize)
   {
      #if PN_CACHE_LINESIZE == 32 || PN_CACHE_LINESIZE == auto
      case 32:
      {
         invalidate_as        = &invalidate_32_as;
         invalidate_bulk_as   = &invalidate_bulk_32_as;
         writeback_as         = &writeback_32_as;
         writeback_bulk_as    = &writeback_bulk_32_as;
         flush_as             = &flush_32_as;
         flush_bulk_as        = &flush_bulk_32_as;
         break;
      }
      #endif /* PN_CACHE_LINESIZE == 32 || PN_CACHE_LINESIZE == auto */
      #if PN_CACHE_LINESIZE == 64 || PN_CACHE_LINESIZE == auto
      case 64:
      {
         invalidate_as        = &invalidate_64_as;
         invalidate_bulk_as   = &invalidate_bulk_64_as;
         writeback_as         = &writeback_64_as;
         writeback_bulk_as    = &writeback_bulk_64_as;
         flush_as             = &flush_64_as;
         flush_bulk_as        = &flush_bulk_64_as;
         break;
      }
      #endif /* PN_CACHE_LINESIZE == 64 || PN_CACHE_LINESIZE == auto */
      #if PN_CACHE_LINESIZE == 128 || PN_CACHE_LINESIZE == auto
      case 128:
      {
         invalidate_as        = &invalidate_128_as;
         invalidate_bulk_as   = &invalidate_bulk_128_as;
         writeback_as         = &writeback_128_as;
         writeback_bulk_as    = &writeback_bulk_128_as;
         flush_as             = &flush_128_as;
         flush_bulk_as        = &flush_bulk_128_as;
         break;
      }
      #endif /* PN_CACHE_LINESIZE == 128 || PN_CACHE_LINESIZE == auto */
      #if PN_CACHE_LINESIZE == 256 || PN_CACHE_LINESIZE == auto
      case 256:
      {
         invalidate_as        = &invalidate_256_as;
         invalidate_bulk_as   = &invalidate_bulk_256_as;
         writeback_as         = &writeback_256_as;
         writeback_bulk_as    = &writeback_bulk_256_as;
         flush_as             = &flush_256_as;
         flush_bulk_as        = &flush_bulk_256_as;
         break;
      }
      #endif /* PN_CACHE_LINESIZE == 256 || PN_CACHE_LINESIZE == auto */
      #if PN_CACHE_LINESIZE == 512 || PN_CACHE_LINESIZE == auto
      case 512:
      {
         invalidate_as        = &invalidate_512_as;
         invalidate_bulk_as   = &invalidate_bulk_512_as;
         writeback_as         = &writeback_512_as;
         writeback_bulk_as    = &writeback_bulk_512_as;
         flush_as             = &flush_512_as;
         flush_bulk_as        = &flush_bulk_512_as;
         break;
      }
      #endif /* PN_CACHE_LINESIZE == 512 || PN_CACHE_LINESIZE == auto */
      #if PN_CACHE_LINESIZE == 1024 || PN_CACHE_LINESIZE == auto
      case 1024:
      {
         invalidate_as        = &invalidate_1024_as;
         invalidate_bulk_as   = &invalidate_bulk_1024_as;
         writeback_as         = &writeback_1024_as;
         writeback_bulk_as    = &writeback_bulk_1024_as;
         flush_as             = &flush_1024_as;
         flush_bulk_as        = &flush_bulk_1024_as;
         break;
      }
      #endif /* PN_CACHE_LINESIZE == 1024 || PN_CACHE_LINESIZE == auto */
      #if PN_CACHE_LINESIZE == 2048 || PN_CACHE_LINESIZE == auto
      case 2048:
      {
         invalidate_as        = &invalidate_2048_as;
         invalidate_bulk_as   = &invalidate_bulk_2048_as;
         writeback_as         = &writeback_2048_as;
         writeback_bulk_as    = &writeback_bulk_2048_as;
         flush_as             = &flush_2048_as;
         flush_bulk_as        = &flush_bulk_2048_as;
         break;
      }
      #endif /* PN_CACHE_LINESIZE == 2048 || PN_CACHE_LINESIZE == auto */
      default:
      {
         return PN_ERR_CACHE_LINESIZE;
      }
   }
      
  /* 
   * Enable the cache.
   * No need for error check, since we already made a CoPU check and that's the
   * only possibility for an error.
   */
   
   pn_cache_enable();
   
   return PN_SUCCESS;   
}

int pn_cache_enable(void)
{
  /* 
   * When changing this, make sure that the pn_cache_init() function is still
   * correct!
   */
   
   #ifndef PN_COMPILE_RAW
   
      COPU_CHECK
      
   #endif /* PN_COMPILE_RAW */
   
   enable_cache_as();
   
   return PN_SUCCESS;   
}

int pn_cache_disable(void)
{
   #ifndef PN_COMPILE_RAW
   
      COPU_CHECK
      
   #endif /* PN_COMPILE_RAW */
   
   /* flush all */
   pn_cache_flush_all();
   
   /* disable cache */
   disable_cache_as();
   
   return PN_SUCCESS;   
}

unsigned long pn_cache_linesize(void)
{
   return s_cache_linesize;   
}

unsigned long pn_cache_size(void)
{
   return s_cache_size;   
}

int pn_cache_invalidate(void *addr, unsigned long size)
{
   return cache_action(addr, size, invalidate_as, invalidate_bulk_as);
}

int pn_cache_invalidate_all(void)
{
  /* 
   * When changing this, make sure that the pn_cache_init() function is still
   * correct!
   */
   
   return cache_action(s_addr, s_mem_size, invalidate_as, invalidate_bulk_as);   
}

int pn_cache_writeback(void *addr, unsigned long size)
{
   return cache_action(addr, size, writeback_as, writeback_bulk_as);
}

int pn_cache_writeback_all(void)
{
   return cache_action(s_addr, s_mem_size, writeback_as, writeback_bulk_as);  
}

int pn_cache_flush(void *addr, unsigned long size)
{
   return cache_action(addr, size, flush_as, flush_bulk_as);
}

int pn_cache_flush_all(void)
{
   return cache_action(s_addr, s_mem_size, flush_as, flush_bulk_as);    
}

/*EOF**************************************************************************/
