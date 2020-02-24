/** @file */

/**
 * \file
 * \brief      Contains helpers and function prototypes of testcases.
 * 
 * \includelineno ./libparanut_unittest.h
 */

/** 
 * \mainpage libparanut Unittest Documentation
 * 
 * \section Description
 * 
 * This is a Unittest for the libparanut, a hardware abstraction layer for
 * ParaNut architectures.
 * 
 * \section Copyright
 * 
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
 * 
 * \section HOWTO
 * 
 * First, check this Unittests Makefile. In there, you will see a section
 * called "System Configuration":
 * \code 
 * #System Configuration########################################################
 * # Call clean when changing these!
 *
 * # System Parameters
 * NUMCORES  = 4
 * M2CAP_MSK = 0x0000000F
 * M3CAP_MSK = 0x00000001
 * \endcode
 * 
 * It is necessary to chose the right parameters for your ParaNut configuration
 * here, else the Unittest cannot check if the ParaNut gives correct data about
 * itself. NUMCORES is about how many cores your system has. M2CAP_MSK is a
 * bitmask with the bits turned on that represent the cores which are able of
 * running in Threaded Mode. M3CAP_MSK is a bit mask with the bits turned on
 * that represent the cores which are able of handling their own exceptions.
 * 
 * The parameters in these section should already be the same as the default
 * configuration of the ParaNut, so if you didn't change anything in the
 * config File of the ParaNut, you don't need to worry about this.
 * 
 * To run in SystemC simulation, execute:
 * \code
 * make sim
 * \endcode
 * for just the execution. To produce more debug information, execute:
 * \code
 * make sim_dbg
 * \endcode
 * 
 * This produces additional information, like a very full binary dump, a 
 * reduced dump, and a Waveform which you can open with GTKWave. All of that is
 * available in the directory Debugging_Aid (it aids debugging). For viewing the
 * waveform, I have already prepared a standard view which proved to be very
 * useful for debugging the libparanut. It can be found under 
 * Debugging_Aid/waveview.gtkw. The file paranut.cfg in the same directory can
 * be used for connecting GDB to simulation (see ParaNut Manual Apendix for
 * instructions on how to do that).
 * 
 * For running on Zybo Z7020, execute:
 * \code
 * make flash-z20-bit
 * \endcode
 * 
 * Further explainations on this
 * can be found in the documentation of module \ref ar.
 * 
 * \section also Also see ...
 * 
 * For further information on what exactly is being tested here, check
 * the documentation of the libparanut itself and the ParaNut Manual.
 *
 * \todo Test _g functions too when they are actually implemented in libparanut.
 */

/*Includes*********************************************************************/

#include <stdio.h>
#include <limits.h>
#include <string.h>
#include "libparanut.h"

/*Architecture Defines*********************************************************/

/**
 * \defgroup   ar Architecture Defines
 * \brief      Defines that give information about your ParaNut architecture.
 *
 * Since this Unittest is designed to be run on many different ParaNut
 * implementations, it needs some information on your exact architecture. You 
 * have to set these things explicitely while compiling the test. If you don't, 
 * errors are thrown.
 * 
 * For learning how to set the defines during compilation, check the manual of 
 * your preprocessor/compiler.
 */
 
/**
 * \addtogroup ar
 * @{
 */

/**
 * @{
 */
 
/* 
 * The weird #if DOXYGEN is done because Doxygen won't document it otherwise. 
 * Sorry about that.
 * If you find a more elegant solution, do not hesitate to put it in :)
 */

/**
 * \def        NUMCORES
 * \brief      Number of cores on your system (includes CePU).
 */
#if DOXYGEN

   #define NUMCORES
   
#endif

#ifndef NUMCORES

   #define NUMCORES
   #error NUMCORES undefined! Check "Architecture Defines" Documentation!
   
#endif

/**
 * \def        M2CAP_MSK
 * \brief      Mask representing which cores are capable of Mode 2.
 * 
 * Make this as wide as your native register width. Only represent the first
 * group (group number 0).
 */
#if DOXYGEN

   #define M2CAP_MSK
   
#endif

#ifndef M2CAP_MSK

   #define M2CAP_MSK
   #error M2CAP_MSK undefined! Check "Architecture Defines" Documentation!
   
#endif

/**
 * \def        M3CAP_MSK
 * \brief      Mask representing which cores are capable of Mode 3.
 * 
 * Make this as wide as your native register width. Only represent the first
 * group (group number 0).
 */
#if DOXYGEN

   #define M3CAP_MSK
   
#endif

#ifndef M3CAP_MSK

   #define M3CAP_MSK
   #error M3CAP_MSK undefined! Check "Architecture Defines" Documentation!
   
#endif

/**
 * @}
 */

/**
 * @}
 */

/*Helpers**********************************************************************/

/**
 * \def        TERMNL
 * \brief      Terminal newline, works on several platforms.
 */
#define TERMNL             "\n\r"

/**
 * \defgroup   ret Test Case Return Values
 * \brief      Defines and Typedef for Test Case Return Values
 */
 
/**
 * \addtogroup ret
 * @{
 */
 
/**
 * @{
 */

/**
 * \typedef    TEST_RET
 * \brief      Renaming of int8_t to mark clearly where a test return value is
 *             expected.
 */
typedef int8_t TEST_RET;

/**
 * \def        TEST_SUCCESS
 * \brief      Return value if test succeded.
 */
#define TEST_SUCCESS       ( 0)

/**
 * \def        TEST_FAIL
 * \brief      Return value if test failed.
 */
#define TEST_FAIL          (-1)

/**
 * \def        TEST_SKIPPED
 * \brief      Return value if test was not executed.
 */
#define TEST_SKIPPED       (-2)

/**
 * @}
 */
 
/**
 * @}
 */

/*Test Case Prototypes*********************************************************/

TEST_RET test_time(void);
TEST_RET test_numcores(void);
TEST_RET test_cap(void);
TEST_RET test_link(void);
TEST_RET test_thread(void);
TEST_RET test_halt_CoPU(void);
TEST_RET test_cache(void);
TEST_RET test_exception(void);
TEST_RET test_spinlock(void);

/*EOF**************************************************************************/

