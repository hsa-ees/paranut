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
 * \brief      Contains main function which calls all the testcases.
 * 
 * Execution is ended automatically when a testcase fails. This is because some
 * testcases need other functionality to work perfectly before testing
 * the actual function.
 * 
 * \includelineno ./libparanut_unittest_main.c
 */

/*Includes*********************************************************************/

#include "libparanut_unittest.h"

/*Helpers**********************************************************************/

/**
 * \def        TEST(x)
 * \brief      Helper for ending the test when execution failed.
 */
#define TEST(x)      printf("###STARTING %s###" TERMNL, #x);                \
                     if ((ret = x()) == TEST_FAIL)                          \
                     {                                                      \
                        printf("###TESTCASE FAILED###" TERMNL TERMNL);      \
                        printf("###Unsuccessful End of Test :(###"          \
                                                            TERMNL TERMNL); \
                        return -1;                                          \
                     }                                                      \
                     else if (ret == TEST_SKIPPED)                          \
                     {                                                      \
                        printf("###TESTCASE SKIPPED###" TERMNL TERMNL);     \
                     }                                                      \
                     else                                                   \
                     {                                                      \
                        printf("###TESTCASE SUCCESS###" TERMNL TERMNL);     \
                     }

/*Main Function****************************************************************/

/**
 * \fn         int main()
 * \brief      Main Function which starts the test cases.
 */
int main()
{
   
#if !(defined PN_WITH_BASE)            \
      && !(defined PN_WITH_CACHE)      \
      && !(defined PN_WITH_LINK)       \
      && !(defined PN_WITH_THREAD)     \
      && !(defined PN_WITH_EXCEPTION)  \
      && !(defined PN_WITH_SPINLOCK)

   printf("###No Modules were compiled in libparanut, cannot start test :(###" 
                                                                        TERMNL);
   return TEST_FAIL;
      
#else

  /*
   * locals
   */
   TEST_RET       ret;              /* saves return value - see helper TEST() */
   
#endif
   
#ifdef PN_WITH_BASE
   long long int  start, end;       /* start and end time of test             */
#endif /* PN_WITH_BASE */
   
   printf("###Welcome to libparanut Unittest###" TERMNL TERMNL);
   
  /*
   * Initialize all of libparanut Modules that need initializing. This is sorta
   * untestable by itself. Things will go wrong in the unit test itself if 
   * something's wrong here, though.
   */

#ifdef PN_WITH_EXCEPTION
   printf("###Initializing exception module ...###" TERMNL TERMNL);
   pn_exception_init();
#endif /* PN_WITH_EXCEPTION */   

#ifdef PN_WITH_CACHE
   printf("###Initializing cache module ...###" TERMNL TERMNL);
   if (pn_cache_init() != PN_SUCCESS)
   {
      printf("Error in pn_cache_init(). We can not proceed with this test."
                                                                 TERMNL TERMNL);
      printf("###Unsuccessful End of Test :(###" TERMNL TERMNL);
      return -1;
   }
#endif /* PN_WITH_CACHE */   
   
#ifdef PN_WITH_BASE
  
  /**
   * \fn       TEST_RET test_time(void)
   * \brief    Tests function pn_time_ns().
   * 
   * Tested first so we can measure time for the rest of the test.
   */
   TEST(test_time)
   
   start = pn_time_ns();
   
  /**
   * \fn       TEST_RET test_clock_freq(void)
   * \brief    Tests function pn_clock_freq().
   * 
   * Tested next so we can make sure time for the rest of the test is correct.
   */
   TEST(test_clock_freq)
   
  /**
   * \fn       TEST_RET test_numcores(void)
   * \brief    Tests function pn_numcores().
   */
   TEST(test_numcores)

  /**
   * \fn       TEST_RET test_cap(void)
   * \brief    Tests functions pn_m2cap() and pn_m3cap().
   */
   TEST(test_cap)
   
#endif /* PN_WITH_BASE */

#if defined PN_WITH_LINK && defined PN_WITH_BASE

 /**
   * \fn       TEST_RET test_link(void)
   * \brief    Tests all functions in link module.
   * 
   * Uses pn_numcores().
   */
   TEST(test_link)
   
#endif /* defined PN_WITH_LINK && defined PN_WITH_BASE */

#if defined PN_WITH_THREAD && defined PN_WITH_BASE
 
 /**
   * \fn       TEST_RET test_thread(void)
   * \brief    Tests all functions in thread module.
   * 
   * Assumes that entry point for CoPUs is set correctly in the startup code.
   * 
   * Uses pn_m2cap().
   */
   TEST(test_thread)
   
#endif /* defined PN_WITH_THREAD && defined PN_WITH_BASE */

#if defined PN_WITH_BASE && defined PN_WITH_THREAD

  /**
   * \fn       TEST_RET test_halt_CoPU(void)
   * \brief    Tests function pn_halt_CoPU().
   *
   * This test was being put in here because threaded mode has to work properly 
   * before this can be tested.
   */
   TEST(test_halt_CoPU)
   
#endif /* defined PN_WITH_BASE && defined PN_WITH_THREAD */

#ifdef PN_WITH_CACHE

  /**
   * \fn       TEST_RET test_cache(void)
   * \brief    Tests all functions in cache module. Also implicitely tests
   *           pn_simulation().
   * 
   * Assumes cache module to have been initialized before.
   * 
   * Testing the cache is skipped in ParaNut simulation since it is 
   * excruciatingly slow. Also tests pn_simulation(). This means, if you're not
   * in a simulation and this testcase is skipped, something is wrong with
   * pn_simulation().
   */
   TEST(test_cache)

#endif /* PN_WITH_CACHE */
   
#ifdef PN_WITH_EXCEPTION

 /**
   * \fn       TEST_RET test_exception(void)
   * \brief    Tests all functions in exception module.
   * 
   * Assumes exception module to have been initialized before.
   * 
   * \todo I have no idea how I am supposed to test pn_interrupt_enable() and
   * pn_interrupt_disable() at the current ParaNut implementation, since we do
   * not have a working mtimecmp and mtime register yet. This may change in the
   * future, though.
   */
   TEST(test_exception)
   
#endif /* PN_WITH_EXCEPTION */

#if defined PN_WITH_SPINLOCK && defined PN_WITH_THREAD

 /**
   * \fn       TEST_RET test_spinlock(void)
   * \brief    Tests all functions in spinlock module.
   * 
   * Implicitely tests pn_begin_threaded() and pn_end_threaded().
   * 
   * \todo This needs changes in case there's more than one group of CPUs.
   */
   TEST(test_spinlock)
   
#endif /* defined PN_WITH_SPINLOCK && defined PN_WITH_THREAD */

#ifdef PN_WITH_BASE
   end = pn_time_ns();
   printf("###Execution time of test: %lld ms###" TERMNL TERMNL, 
                                                       (end - start) / 1000000);
#endif /* PN_WITH_BASE */

   printf("###Successfull End of Test :)###" TERMNL TERMNL);

   return TEST_SUCCESS;
}

/*EOF**************************************************************************/
