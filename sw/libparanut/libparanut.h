/** @file */

/**
 * \file
 * \brief      API of the libparanut.
 */

/** 
 * \mainpage libparanut Documentation
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
 * \section modules Modules of the libparanut
 *
 * Welcome to the libparanut documentation! This is a hardware abstraction 
 * library you can use to program your ParaNut without having to stare at 
 * Assembly Code.
 *
 * To do that, the libparanut serves several functions to you on a silver
 * platter. These are grouped into \ref mo for your convenience.
 *
 *    1. The \ref ba contains functions for getting general information
 *       about the ParaNut, like the number of cores and such. There's also some
 *       functions for halting cores.
 *    2. The \ref li contains functions for stopping and starting Linked
 *       Mode execution. Don't know what that is? Check \ref ms or the fine 
 *       ParaNut Manual.
 *    3. The \ref th contains functions for stopping and starting Threaded
 *       Mode execution. Same advice goes.
 *    4. The \ref ca contains functions for controlling the ParaNut cache and 
 *       getting information about it.
 *    5. The \ref ex contains functions for controlling Interrupts and
 *       Exceptions.
 *    6. The \ref sp is an implementation of a spinlock so you can properly
 *       protect and synchronize your stuff in Threaded Mode.
 *
 * \section System Overview
 *
 * The following picture shows a system overview. It might be a little confusing
 * at first. Don't worry about that, just read some more documentation and it
 * will probably all fall into place.
 *
 * \todo Add English Version and fix format
 *
 * \image html aufbau.png "Overview of the libparanut" width=800px
 * \image latex aufbau.eps "Overview of the libparanut" width=0.8\textwidth
 * 
 * \section HOWTO
 * 
 * \todo Add documentation here in case other compilers/ISAs are used someday.
 * 
 * \todo The \ref Makefile might not be compliant with any other tool than GNU 
 *       make.
 * 
 * How to make libparanut run with our in-house ParaNut implementation:
 * You will need GNU make and the RISC-V GNU Toolchain for that (see Appendix A 
 * in ParaNut Manual). Also, you may need to have Python on your system (so far,
 * no restrictions on the version are known). This assumes you already got the
 * ParaNut up and running!
 * 
 *    1. Open up the wonderful \ref Makefile in the libparanut directory. This 
 *       is the place where all your wishes come true. Right at the start, 
 *       there's the section "System Configuration" where you can set a few 
 *       things to tweek your libparanut. Check out the \ref co to see what 
 *       those values mean. The default should be fine at first.
 *    2. Onto the section "Compiler Configuration". 
 *       See the "PN_ISA" in the \ref Makefile? This is the Instruction Set
 *       Architecture. The very nice \ref Makefile tree will chose all the right 
 *       assembly source code files for the libparanut if this variable is set 
 *       correctly. Currently, our ParaNut implements RISCV ISA, which is why
 *       this value says RV32I. This means you don't have to touch it. 
 *    3. The \ref Makefile also lets you chose the compiler. If you want another
 *       compiler than GCC, you will also need to add some if-else logic in the
 *       section "Compiler and Assembler Flags".
 *    4. To reduce the code size of the libparanut, you could set "PN_DEBUG" to
 *       0. This means that the libparanut will not be compiled with debug 
 *       symbols. It should be fine to compile it with debug symbols at first,
 *       though.
 *    7. Want everything compiled now? Say \code gmake all \endcode and the 
 *       magic is done. All the modules are compiled and statically linked into 
 *       "libparanut.a". You can find it in the newly generated directory named 
 *       "INSTALL". Do not forget to call clean beforehand if you changed 
 *       something in the Makefile.
 *    8. Include libparanut.h in your application. Set your include-path to 
 *       the INSTALL directory while compiling: 
 *       \code -I/path/to/libparanut/INSTALL \endcode
 *    9. Link libparanut.a into your application. To do that in GCC,
 *       you can just put \code -L/path/to/libparanut/INSTALL -lparanut \endcode 
 *       at the end of your linker/compilation command.
 * 
 * I hope the following example can explain what I mean:
 * 
 * \todo Prettier example, explain Link Module and Spinlock Module.
 * 
 * \code 
 * cd /path/to/libparanut
 * gmake all
 * cd /path/to/my_application.c
 * my_gcc -c my_compiler_options -I/path/to/libparanut/INSTALL my_application.c
 * my_gcc my_link_options my_object.o -L/path/to/libparanut/INSTALL -lparanut
 * \endcode
 * 
 * Do not forget to also compile in the startup code. To see a full example of
 * what you need to do, check the Makefiles in the other sw/ subdirectories in 
 * the GIT.
 * 
 * \section expectations Expectations to the application
 * 
 * Here's a list of things that the libparanut expects you to have been done 
 * before usage:
 * 
 *    1. The \ref ca expects you to have called \ref pn_cache_init() before
 *       using any of its other functions. Behaviour of the other functions is
 *       undefined if not.
 *    2. The \ref ex expects you to have called \ref pn_exception_init() before
 *       calling \ref pn_exception_set_handler(). I mean, you can set your own
 *       exception handler beforehand, but it will not be called if the
 *       exception occurs.
 *       The \ref pn_interrupt_enable(), \ref pn_interrupt_disable(), and \ref
 *       pn_progress_mepc() functions will work with no problem, though.
 *    3. The \ref th is the complicated one. It's a little tricky because of its
 *       workflow. The CePU internally sets a few needed variables which 
 *       indicate a status to the CoPU. After a CoPU is woken up, they start 
 *       executing code at the reset address. This means that the end of the 
 *       startup code for the CoPUs also needs to be their entrance point to the
 *       \ref th, where they can read the internal variables and figure out what
 *       they are supposed to do with them. This entrance point is 
 *       \ref pn_thread_entry(), and it needs to be called in the startup code 
 *       on all CoPUs. Our in-house startup code does this for you, but I'm 
 *       leaving that here just in case you want to know.
 *    5. The \ref th has one more expectation. The state of the CePU and its
 *       memory need to be shared with all cores. That means that there has 
 *       to be a shared memory section that has to be as big as the memory for
 *       the individual cores. The start adress of this area has to be put into 
 *       a globally known location called shared_mem_start, and the size has
 *       to be put into shared_mem_size. This also happens in our build-in
 *       startup code, so you do not need to worry about it.
 * 
 * Also, here's some general advice: If you want your startup code to be 
 * compatible with many versions of the libparanut, you can check if a certain
 * module is available. The file \ref pn_config.h, which is created during 
 * compilation time, has some defines set for every module that is enabled.
 * Check out the \ref pn_config.h documentation to find out more!
 *
 * \internal
 * \section future Future Ideas for libparanut
 * 
 *    1. In the \ref ca, split \ref pn_cache_enable() and 
 *       \ref pn_cache_disable() into seperate functions for data and
 *       instruction cache.
 *    2. In the \ref th, build in POSIX style thread functions.
 *    3. Build a state machine for every core that tracks wether a core is used
 *       in linked mode, threaded mode, or doing POSIX threads. Enable the user
 *       of the libparanut to use threaded and linked mode in a mixed way.
 *    4. In \ref ex, provide a function to hang in a "raw" exception handler
 *       (basically change mtvec).
 *    5. Implement a possibility to check wether or not a module has been
 *       initialized already. Also implement a pn_init() function which
 *       checks for all modules if the module is already initialized, and if it 
 *       is not, initializes them (provided the module was compiled in).
 *    6. Concerning the init() functions: If a function in a module is used
 *       despite of the module not being initialized, throw an exception
 *       (use \ref pn_ecall()).
 *    7. Implement a pn_strerror() function which takes an error value and
 *       prints what the error means.
 *    8. Implement a global checkable error variable.
 *    9. For threaded and linked mode, implement the possibility to pass a stack
 *       size that shall be copied. It should also be able to tell these
 *       functions that only the stack from the top function shall be copied.
 *    10. Implement a pn_atomic_increment() and pn_atomic_decrement() function.
 *    11. Implement pn_printf() which can be used in linked mode.
 *    12. In the Makefile, copy the libparanut to the place where the RISCV
 *        toolchain is installed. This would make it easier for applications to
 *        compile.
 */
 
/**
 * \internal
 * \defgroup   as Internal Assembly Calls
 *
 * \brief      Calls from the architecture independent implementation to the
 *             architecture specific assembly code.
 * 
 * \todo       If there ever is a new ParaNut with a different ISA,
 *             documentation for the new .S files has to be added in the 
 *             respective directories.
 */
 
/**
 * \file          Makefile
 * \brief         Makefile of the libparanut.
 * 
 * This Makefile is supposed to make (ha!) the compilation of libparanut more
 * handy. To check out exactly how this works, see the section \ref HOWTO in the
 * mainpage!
 * 
 * \internal
 * \todo The Makefile might not be compliant with any other tool than GNU make.
 *       Compatibility with other tools should be tested and listed (maybe in
 *       the mainpage). The \ref HOWTO section needs to be changed, then.
 * 
 * \attention Why am I calling a Python Skript when I'm building 
 * pn_cache_RV32I_nnn.S (example: \ref pn_cache_RV32I_auto.S) instead of just 
 * writing the file myself? And why does that cache target exist?
 * Click \ref pn_cache_RV32I_buildscript.py to find out!
 * \endinternal
 * 
 * \includelineno Makefile
 */

/*Includes*********************************************************************/

#include <stdint.h>
#include "pn_config.h"
 
/*Typedefs*********************************************************************/

/**
 * \defgroup   he libparanut Helpers
 * \brief      Typedefs and defines of libparanut which can also be used in your
 *             application.
 */
 
/**
 * \addtogroup he
 * @{
 */

/**
 * \defgroup   ty Typedefs
 * \brief      Needed typedefs.
 * 
 * Please note that there are several typedefs for differing register widths of
 * the ParaNut. Which ones are used depend on how you set \ref PN_RWIDTH while
 * compiling (pn_config.h). Doxygen can only document one of them, so here you
 * are, stuck with the 32-bit version. Check source code of libparanut.h, 
 * section Typedefs, if you want to see the others.
 */
 
/**
 * \addtogroup ty
 * @{
 */
 
/**
 * \typedef    _pn_spinlock
 * \brief      Renaming of struct \ref __pn_spinlock for your convenience.
 * 
 * Check documentation of \ref __pn_spinlock to get more information.
 */

/**
 * @{
 */

/**
 * \typedef    PN_CID
 * \brief      Signed type that can be used to address any core in this
 *             architecture.
 *
 * See documentation of \ref PN_NUMC to understand why we use only the actual
 * register width and not more.
 */

/**
 * \typedef    PN_NUMC
 * \brief      Signed type that can hold the maximum number of cores in this
 *             architecture.
 *
 * Let's say your ParaNut has a group register width of 32 bits. This means that 
 * there are 4.294.967.296 potential groups. Every group has 32 bits to 
 * represent different cores. That means there are 137.438.953.472 cores that
 * can be adressed.
 * 
 * This does, in theory, mean that we need 64 bit to represent the possible
 * number of cores. However, it is deemed to be pretty unrealistic that there
 * will be a ParaNut version with more than 4.294.967.296 cores anytime soon.
 * So, for optimization purposes, we just use 32 bit here. Even half of that is
 * probably not possible in my lifetime, which is why we are not even using 
 * unsigned (also because some compilers could throw errors when mixing signed 
 * and unsigned types, e.g. in a for loop). One more plus is that we can use
 * these values to signal errors when they are returned by a function.
 * 
 * Same explaination goes in all other register widths. If you really need more, 
 * feel free to double the bits.
 */
 
/**
 * \typedef    PN_CMSK
 * \brief      Unsigned type that can act as a core mask.
 *
 * This is, of course, the same as the register width of the ParaNut. Unsigned
 * because it direcetly represents a register.
 */
 
/**
 * \typedef    PN_NUMG
 * \brief      Signed type that can be used to address any group in this
 *             architecture.
 *
 * This is, of course, the same as the register width of the ParaNut.
 */
 

#if PN_RWIDTH == 8

   typedef int8_t    PN_CID;
   typedef int8_t    PN_NUMC;
   typedef uint8_t   PN_CMSK;
   typedef int8_t    PN_NUMG;
   
#elif PN_RWIDTH == 16

   typedef int16_t   PN_CID;
   typedef int16_t   PN_NUMC;
   typedef uint16_t  PN_CMSK;
   typedef int16_t   PN_NUMG;
   
#elif PN_RWIDTH == 32

   typedef int32_t   PN_CID;
   typedef int32_t   PN_NUMC;
   typedef uint32_t  PN_CMSK;
   typedef int32_t   PN_NUMG;

#elif PN_RWIDTH == 64

   typedef int64_t   PN_CID;
   typedef int64_t   PN_NUMC;
   typedef uint64_t  PN_CMSK;
   typedef int64_t   PN_NUMG;
   
#elif PN_RWIDTH == 128

   typedef int128_t  PN_CID;
   typedef int128_t  PN_NUMC;
   typedef uint128_t PN_CMSK;
   typedef int128_t  PN_NUMG;

#else
   
   #error Your ParaNut has a very exotic register width that was not considered\
    when libparanut was written. Check libparanut.h, section Typedefs, and add \
    your funky register width there.
   
#endif

/**
 * @}
 */
 
/**
 * @}
 */

/*Error Codes******************************************************************/

/**
 * \defgroup   er Error Codes
 * \brief      Error codes returned by the functions in this library.
 */
 
/**
 * \addtogroup er
 * @{
 */
 
/**
 * @{
 */

/**
 * \def        PN_SUCCESS
 * \brief      Successful execution.
 * 
 * Implies that a function finished successfully.
 */
#define PN_SUCCESS            0

/**
 * \def        PN_ERR_PARAM
 * \brief      Parameter error.
 *
 * The parameters given to a function were wrong (i.e. out of range).
 */
#define PN_ERR_PARAM          (-1)

/**
 * \def        PN_ERR_NOIMP
 * \brief      Function not implemented.
 *
 * The libparanut function is not yet implemented.
 */
#define PN_ERR_NOIMP          (-2)

/**
 * \def        PN_ERR_COPU
 * \brief      CoPU error.
 *
 * Function that isn't allowed on CoPU was being executed on CoPU.
 */
#define PN_ERR_COPU           (-3)

/**
 * \def        PN_ERR_MATCH
 * \brief      Mode begin and end matching error.
 *
 * Functions for beginning and ending linked and threaded mode have to be
 * matched. Linked and threaded mode shall not be mixed.
 */
#define PN_ERR_MATCH          (-4)

/**
 * \def        PN_ERR_LOCKOCC
 * \brief      Lock occupied error.
 *
 * Can occur if you tried destroying an occupied lock or used the trylock
 * function on an occupied lock.
 */
#define PN_ERR_LOCKOCC        (-5)

/**
 * \def        PN_ERR_CACHE_LINESIZE
 * \brief      Weird cache line size error.
 *
 * Can occur if libparanut is supposed to run on an architecture that has a
 * cache line size which is not either 32, 64, 128, 256, 512, 1024 or 2048 bit.
 * Should be more of a development error instead of a normal usage error.
 * 
 * In other words, it should not occur if you are just developing middle end
 * stuff while using the libparanut on a deployed ParaNut. Contact
 * the maintainers about this if it still does.
 * 
 * \internal
 * 
 * The following steps would be necessary to introduce a new cache line size on
 * RV32I libparanut:
 *    - Go to the top of \ref pn_cache_RV32I_buildscript.py and add the new
 *      cache line size in bits to "cache_line_size_list".
 *    - Go to the \ref Makefile and take a look at the target "cache". Add a new
 *      call to the buildscript for your size.
 *    - Go into \ref pn_cache.c and look at "Assembly Functions". Add your
 *      references to the others.
 *    - Also look at the function \ref pn_cache_init(). You will see that there
 *      is a switch that decides what functions to use. Add a new 
 *      case there (before the default, of course!) and put in your newly
 *      added functions.
 *    - After a \code make clean all \endcode your new cache line size should be
 *      ready to use.
 *    - Change the places in the documentation where the linesizes are listed 
 *      (should only be here and at the top of 
 *      \ref pn_cache_RV32I_buildscript.py). Add your
 *      new assembly file to the GIT repo and push that stuff.
 * 
 * \todo This description should change if other ParaNut architectures or cache
 * line sizes are introduced.
 * 
 * \endinternal
 */
#define PN_ERR_CACHE_LINESIZE (-6)

/**
 * \def        PN_ERR_EXC
 * \brief      Exception code not implemented error.
 *
 * You tried callin \ref pn_exception_set_handler() with an invalid exception
 * code.
 */
#define PN_ERR_EXC            (-8)

/**
 * @}
 */
 
/**
 * @}
 */
 
/*Modes************************************************************************/

/**
 * \defgroup   ms Modes
 *
 * \brief      Modes of the ParaNut Cores.
 *
 * The CePU can only ever operate in Mode 3 (autonomous). It is still shown as 
 * capable of Mode 2 (threaded Mode) because Mode 3 is an extension in 
 * functionality in comparison to Mode 2. Mode 2 cores do not handle their own
 * interrupts/exceptions, which a Mode 3 core does.
 *
 * It can also be set into Mode 0, which does not break hardware debugging
 * support.
 *
 * The CePU is the only core capable of changing other cores Modes.
 *
 * The CoPUs are never capable of Mode 3. They may be capable of Mode 2, which
 * means they are able to fetch their own instructions and are therefore able
 * to do different work in parallel to the CePU. They are, at minimum, capable
 * of Mode 1 (linked Mode), which means it will execute the same instructions as
 * the CePU on different data. This does not start until the CePU is also told
 * to now start executing in linked mode, though.
 * 
 * \internal
 * \todo The information above may change some day.
 * \endinternal
 *
 * Which Mode the CoPUs are in after system reset is an implementation detail of
 * the ParaNut itself and the startup code.
 *
 * For further information, check ParaNut Manual.
 */
 
/**
 * \addtogroup ms
 * @{
 */
 
/**
 * @{
 */

/**
 * \def        PN_M0
 * \brief      Mode 0 (halted Mode).
 */
#define PN_M0              0x0U

/**
 * \def        PN_M1
 * \brief      Mode 1 (linked Mode).
 */
#define PN_M1              0x1U

/**
 * \def        PN_M2
 * \brief      Mode 2 (unlinked or threaded Mode).
 */
#define PN_M2              0x2U

/**
 * \def        PN_M3
 * \brief      Mode 3 (autonomous Mode).
 */
#define PN_M3              0x3U

/**
 * @}
 */
 
/**
 * @}
 */
 
/**
 * @}
 */

/*Base Module******************************************************************/

/**
 * \defgroup   mo libparanut Modules
 *
 * \brief      Modules of libparanut.
 */
 
/**
 * \addtogroup mo
 * @{
 */

/**
 * \defgroup   ba Base Module
 *
 * \brief      Functions for getting the status of your ParaNut and halting 
 *             cores.
 *
 * Concerning the _g functions: If your ParaNut implementation has more cores 
 * than what is the standard register size on your system (i.e. register size is
 * 32, but you got more than 32 cores including the CePU), you have to chose the
 * group of cores that you want to talk to. For that, most functions in this
 * section have a group version (suffix _g) which has an additional group 
 * parameter.
 *
 * This works the following way: If your register size is 32, than the CePU is
 * in group 0 (0x00000000) and is always represented by the first bit in the 
 * coremasks (0x00000001). The first CoPU is also in group 0 and represented by
 * the second bit (0x00000002). After that comes the second CoPU, represented by
 * 0x00000004. And so on until CoPU 30. The 31st CoPU is then represented by
 * group 1 (0x00000001) and the first bit (0x00000001). The 63rd CoPU is
 * represented by group 2 (0x00000002) and the first bit (0x00000001). The 95th
 * CoPU is represented by group 3 (0x00000003) and the first bit (0x00000001).
 * This information may become untrue if a big-endian version of the ParaNut is
 * ever released.
 *
 * For further information on this topic, you should check the ParaNut Manual
 * itself. Also, the documentation on the ParaNut \ref ms that is included in 
 * here could clear some things up.
 * 
 * \internal
 * The base module also contains functions that are used in other modules as
 * well.
 * \todo Currently (Oct. 2019), there is no ParaNut implementation that actually 
 * has to use more than one group, which is why the group functions are only
 * implemented as stubs right now.
 * \endinternal
 */
 
/**
 * \addtogroup ba
 * @{
 */
 
/**
 * @{
 */

/**
 * \fn         PN_NUMC pn_numcores(void)
 * \brief      Get the number of cores in your system.
 * 
 * Cannot be called from CoPU.
 * 
 * \return     The number of cores in your ParaNut implementation or \ref
 *             PN_ERR_COPU.
 */
PN_NUMC pn_numcores(void);

/**
 * \fn         PN_CMSK pn_m2cap(void)
 * \brief      Check which cores are capable of Mode 2 operation.
 * 
 * See documentation of \ref ms for more information.
 * 
 * Cannot be called from CoPU.
 *
 * \return     A bitmask representing your ParaNut cores or \ref PN_ERR_COPU. 
 *             If a bit is set to 1, it means that the core is capable of 
 *             operating in Mode 2.
 */
PN_CMSK pn_m2cap(void);

/**
 * \fn         PN_CMSK pn_m2cap_g(PN_NUMG groupnum)
 * \brief      Check which cores are capable of Mode 2 operation.
 * 
 * \todo Currently only a stub. Will therefore always return either 
 *       \ref PN_ERR_COPU or \ref PN_ERR_NOIMP if executed on CePU.
 * 
 * See documentation of \ref ms for more information.
 * 
 * Cannot be called from CoPU.
 *
 * \param[in]  groupnum is the group of cores you want to know about.
 * \return     A bitmask representing your ParaNut cores or \ref PN_ERR_COPU. If
 *             a bit is set to 1, it means that the core is capable of operating
 *             in Mode 2.
 */
PN_CMSK pn_m2cap_g(PN_NUMG groupnum);

/**
 * \fn         PN_CMSK pn_m3cap(void)
 * \brief      Check which cores are capable of Mode 3 operation.
 *
 * \attention This function will, in the current ParaNut implementation, return 
 * a hard coded 1 or \ref PN_ERR_COPU if executed on CoPU. The reason for this 
 * is that only the CePU is capable of Mode 3.
 * 
 * See documentation of \ref ms for more information.
 * 
 * Cannot be called from CoPU.
 *
 * \return     A bitmask representing your ParaNut cores or \ref PN_ERR_COPU. If
 *             a bit is set to 1, it means that the core is capable of operating
 *             in Mode 3.
 */
PN_CMSK pn_m3cap(void);

/**
 * \fn         PN_CMSK pn_m3cap_g(PN_NUMG groupnum)
 * \brief      Check which cores are capable of Mode 3 operation.
 *
 * \todo Currently only a stub. Will therefore always return either 
 *       \ref PN_ERR_COPU or \ref PN_ERR_NOIMP if executed on CePU.
 * 
 * See documentation of \ref ms for more information.
 * 
 * Cannot be called from CoPU.
 *
 * \param[in]  groupnum is the group of cores you want to know about.
 * \return     A bitmask representing your ParaNut cores or \ref PN_ERR_COPU. If
 *             a bit is set to 1, it means that the core is capable of operating
 *             in Mode 3.
 */
PN_CMSK pn_m3cap_g(PN_NUMG groupnum);

/**
 * \fn         PN_CID pn_coreid(void)
 * \brief      Get the ID of the core that this function is executed on.
 *
 * \return     The core ID. Starts with 0 for CePU. Can not return an error.
 */
PN_CID pn_coreid(void);

/**
 * \fn         PN_CID pn_coreid_g(PN_NUMG *groupnum)
 * \brief      Get the ID and group number of the core that this code is running
 *             on.
 * 
 * \todo Currently only a stub. Will therefore always return \ref PN_ERR_NOIMP.
 *
 * \param[out] groupnum is a reference to be filled with the group number of 
 *             the core.
 * \return     The core ID. Starts with 0 for CePU. Does not start again when in
 *             another group than group 0. Can not return an error.
 */
PN_CID pn_coreid_g(PN_NUMG *groupnum);

/**
 * \fn         void pn_halt(void)
 * \brief      Halt whatever core the function is executed on.
 *
 * If executed on a core, it will be set to Mode 0 and stop operation. Causes
 * reset of program counter to reset address on a Mode 2 capable CPU.
 * 
 * If executed on CePU, also halts all other CoPUs on system. 
 * 
 * See documentation of \ref ms for more information.
 *
 * This function returns nothing because it should not be possible for any core
 * to leave this state on its own.
 */
void pn_halt(void);

/**
 * \fn         int pn_halt_CoPU(PN_CID coreid)
 * \brief      Halts a CoPU.
 *
 * Sets the CoPU with the given ID into a halted state (Mode 0).
 * 
 * See documentation of \ref ms for more information.
 *
 * Cannot be executed on CoPU.
 *
 * Halting an already halted core results in failure (\ref PN_ERR_PARAM) to aid 
 * debugging.
 * 
 * \todo Not yet implemented for given core IDs outside of group 0. Will return
 * \ref PN_ERR_NOIMP in this case.
 *
 * \param[in]  coreid is the ID of the CoPUs you want to halt. Since ID 0
 *             represents the CePU, this function will throw an error when given
 *             0 to aid debugging. If you want the CePU to halt, use function
 *             \ref pn_halt().
 * \return     Either \ref PN_SUCCESS, \ref PN_ERR_PARAM or \ref PN_ERR_COPU.
 */
int pn_halt_CoPU(PN_CID coreid);

/**
 * \fn         int pn_halt_CoPU_m(PN_CMSK coremask)
 * \brief      Halts one or more CoPUs.
 *
 * Sets the CoPUs represented in the bitmask into a halted state (Mode 0).
 * 
 * See documentation of \ref ms for more information.
 *
 * Cannot be executed on CoPU.
 *
 * Halting an already halted core results in failure (\ref PN_ERR_PARAM) to aid 
 * debugging.
 *
 * \param[in]  coremask is the bitmask representing the CoPUs you want to halt.
 *             When all bits are set to 0, this function will return an error
 *             (\ref PN_ERR_PARAM) to aid debugging. Also, setting the first bit
 *             to 1 will return an error (\ref PN_ERR_PARAM) since it represents
 *             the CePU, which should be halted with \ref pn_halt().
 * \return     Either \ref PN_SUCCESS, \ref PN_ERR_PARAM or \ref PN_ERR_COPU.
 */
int pn_halt_CoPU_m(PN_CMSK coremask);

/**
 * \fn         int pn_halt_CoPU_gm(PN_CMSK *coremask_array, PN_NUMG array_size)
 * \brief      Halts the CoPUs specified in the coremask_array.
 * 
 * \todo Currently only a stub. Will therefore always return either 
 *       \ref PN_ERR_COPU or \ref PN_ERR_NOIMP if executed on CePU.
 *
 * Sets the CoPUs represented by bitmask and their position in the array (=
 * their group number) into a halted state (Mode 0).
 * 
 * See documentation of \ref ms for more information.
 *
 * Cannot be executed on CoPU.
 *
 * Halting an already halted core results in failure (\ref PN_ERR_PARAM) to aid 
 * debugging.
 *
 * \param[in]  coremask_array is a pointer to the start of a coremask array. The
 *             position of the mask in the array represents the group number of
 *             the cores. When all bits are set to 0, this function will return 
 *             an error (\ref PN_ERR_PARAM) to aid debugging. Also, setting the 
 *             first bit to 1 will return an error (\ref PN_ERR_PARAM) since it 
 *             represents the CePU, which should be halted with \ref pn_halt().
 * \param[in]  array_size is the number of entries in the coremask_array.
 * \return     Either \ref PN_SUCCESS, \ref PN_ERR_PARAM or \ref PN_ERR_COPU.
 */
int pn_halt_CoPU_gm(PN_CMSK *coremask_array, PN_NUMG array_size);

/**
 * \fn         long long int pn_time_ns(void)
 * \brief      Returns system time in ns. Does not care for overflow.
 *
 * Cannot be executed on CoPU.
 * 
 * The first time executing this function takes the longest time since it has
 * to initialize the frequency and an internal conversion factor. So if you want
 * to use it for time measurement, you can call the function once before 
 * actual measurement to make the values more comparable.
 * 
 * When testing on my ParaNut, this made a difference of around 2000 ticks.
 * 
 * \return     System time in ns or \ref PN_ERR_COPU.
 */
long long int pn_time_ns(void);

/**
 * \fn         int pn_simulation(void)
 * \brief      Checks if we run in simulation instead of real hardware.
 * 
 * \warning This function is a thing that only works with our specific
 * ParaNut simulation. If the simulation changes, this needs to be changed, too. 
 *
 * \return     Zero if we run on hardware, non-zero if we run in simulation.
 */
int pn_simulation(void);

/**
 * @}
 */
 
/**
 * @}
 */

/*Linked Module****************************************************************/

/**
 * \defgroup   li Link Module
 *
 * \brief      Functions for using the linked mode.
 * 
 * Also see \ref ms.
 * 
 * \warning
 * If you want to use the Linked Module on RISC-V ParaNut, your ParaNut has to
 * support the M Extension and you have to compile your application with the 
 * flag mabi=rv32im. The libparanut \ref Makefile sets this flag automatically 
 * when you chose the \ref li or one of the Modules that has \ref li as a 
 * dependency.
 */
 
/**
 * \addtogroup li
 * @{
 */
 
/**
 * @{
 */

/**
 * \fn         PN_CID pn_begin_linked(PN_NUMC numcores)
 * \brief      Links a given number of CoPUs to the CePU.
 *
 * \internal
 * \todo This functions specification depends a lot on ParaNut hardware design.
 * In later implementations, using linked and threaded mode in a mixed way might
 * be possible. The documentation and implementation will have to be changed,
 * then.
 * \endinternal
 * 
 * \todo Not yet implemented for more cores than what is available in one group.
 * Will return \ref PN_ERR_NOIMP when called with more cores.
 * 
 * Sets numcores-1 CoPUs to Mode 1 (linked Mode) so they start executing the 
 * instruction stream fetched by the CePU. This function will return an error
 * (\ref PN_ERR_MATCH) if the CoPUs are not all halted.
 * 
 * See documentation of \ref ms for more information.
 *
 * Cannot be executed on CoPU. Cannot be mixed with threaded mode or other 
 * linked mode functions, until \ref pn_end_linked() is called.
 * 
 * \attention All data that you want to preserve after \ref pn_end_linked()
 * was called can not be stored on stack. You can make it static or global.
 * 
 * \warning There is no guarantee that the execution of code actually happens at
 * the same time on CePUs and CoPUs.
 *
 * \param[in]  numcores is the number of cores that shall be linked together.
 *             A value of 0 or 1 will return an error (\ref PN_ERR_PARAM) to aid
 *             debugging.
 * \return     The ID of the core, or \ref PN_ERR_MATCH, \ref PN_ERR_PARAM, or
 *             \ref PN_ERR_COPU.
 */
PN_CID pn_begin_linked(PN_NUMC numcores);

/**
 * \fn         PN_CID pn_begin_linked_m(PN_CMSK coremask)
 * \brief      Links the CPUs specified in the coremask.
 * 
 * \internal
 * \todo This functions specification depends a lot on ParaNut hardware design.
 * In later implementations, using linked and threaded mode in a mixed way might
 * be possible. The documentation and implementation will have to be changed,
 * then.
 * \endinternal
 *
 * Sets the CoPUs represented by the bitmask to Mode 1 (linked Mode) so they 
 * start executing the instruction stream fetched by the CePU. This function 
 * will return an error (\ref PN_ERR_MATCH) if the CoPUs are not all halted.
 * 
 * See documentation of \ref ms for more information.
 *
 * Cannot be executed on CoPU. Cannot be mixed with threaded mode or other 
 * linked mode functions, until \ref pn_end_linked() is called.
 * 
 * \attention All data that you want to preserve after \ref pn_end_linked()
 * was called can not be stored on stack. You can make it static or global.
 * 
 * \warning There is no guarantee that the execution of code actually happens at
 * the same time on CePUs and CoPUs.
 *
 * \param[in]  coremask is the bitmask representing the CoPUs you want to link.
 *             When all bits are set to 0, this function will return an error 
 *             (\ref PN_ERR_PARAM) to aid debugging. Also, not setting the first
 *             bit to 1 will return an error (\ref PN_ERR_PARAM) since it 
 *             represents the CePU (which needs to be linked, too).
 * \return     The ID of the core, or \ref PN_ERR_MATCH, \ref PN_ERR_PARAM, or
 *             \ref PN_ERR_COPU.
 */
PN_CID pn_begin_linked_m(PN_CMSK coremask);

/**
 * \fn         PN_CID pn_begin_linked_gm(PN_CMSK *coremask_array, 
 *                                                           PN_NUMG array_size)
 * \brief      Links the CPUs specified in the coremask_array.
 *
 * \internal
 * \todo This functions specification depends a lot on ParaNut hardware design.
 * In later implementations, using linked and threaded mode in a mixed way might
 * be possible. The documentation and implementation will have to be changed,
 * then.
 * \endinternal
 * 
 * \todo Currently only a stub. Will therefore always return \ref PN_ERR_NOIMP.
 * 
 * Sets the CoPUs represented by bitmask and their position in the array (=
 * their group number) to Mode 1 (linked Mode) so they start executing the 
 * instruction stream fetched by the CePU. This function will return an error 
 * (\ref PN_ERR_MATCH) if the CoPUs are not all halted.
 * 
 * See documentation of \ref ms for more information.
 *
 * Cannot be executed on CoPU. Cannot be mixed with threaded mode or other 
 * linked mode functions, until \ref pn_end_linked() is called.
 * 
 * \attention All data that you want to preserve after \ref pn_end_linked()
 * was called can not be stored on stack. You can make it static or global.
 * 
 * \warning There is no guarantee that the execution of code actually happens at
 * the same time on CePUs and CoPUs.
 *
 * \param[in]  coremask_array is a pointer to the start of a coremask array. The
 *             position of the mask in the array represents the group number of
 *             the cores. When all bits are set to 0, this function will return 
 *             an error (\ref PN_ERR_PARAM) to aid debugging. Also, not setting 
 *             the first bit to 1 will return an error (\ref PN_ERR_PARAM) since
 *             it represents the CePU (which needs to be linked, too).
 * \param[in]  array_size is the number of entries in the coremask_array.
 * \return     The ID of the core, or \ref PN_ERR_MATCH, \ref PN_ERR_PARAM, or
 *             \ref PN_ERR_COPU.
 */
PN_CID pn_begin_linked_gm(PN_CMSK *coremask_array, PN_NUMG array_size);

/**
 * \fn         int pn_end_linked(void)
 * \brief      Ends linked execution.
 *
 * Halts all CoPUs that are currently linked together, effectively ending the
 * linked execution. Will fail if there are no cores linked together.
 *
 * See documentation of \ref ms for more information.
 *
 * Can be executed on CoPU, but will do nothing then.
 * 
 * \return     Either \ref PN_SUCCESS or \ref PN_ERR_MATCH.
 */
int pn_end_linked(void);

/**
 * @}
 */
 
/**
 * @}
 */
 
/*Threaded Module**************************************************************/

/**
 * \defgroup   th Thread Module
 *
 * \brief      Functions for using the threaded mode.
 * 
 * Also see \ref ms.
 */
 
/**
 * \addtogroup th
 * @{
 */
 
/**
 * @{
 */
 
/**
 * \fn         void pn_thread_entry(void)
 * \brief      Function that has to be called for CoPUs at the end of the 
 *             startup code.
 *
 * Marks the entry point of CoPUs into the \ref th. Necessary for threaded Mode.
 * 
 * The CoPUs are set up to work correctly in threaded Mode.
 * 
 * Execution is only effective on CoPU. Can be executed on CePU, will do nothing
 * then.
 * 
 * Should not be called in a normal application at all. Left in here for 
 * startup code writers convenience.
 * 
 * \attention What comes after this part in the startup code is irrelevant, 
 * since the CoPUs that landed there are either put into threaded mode or 
 * halted. The function can therefore not return any errors.
 * 
 */
void pn_thread_entry(void);
 
/**
 * \fn         PN_CID pn_begin_threaded(PN_NUMC numcores)
 * \brief      Puts numcores CPUs in threaded mode.
 *
 * \internal
 * \todo This functions specification depends a lot on ParaNut hardware design.
 * In later implementations, using linked and threaded mode in a mixed way might
 * be possible. The documentation and implementation will have to be changed,
 * then.
 * \endinternal
 * 
 * \todo Not yet implemented for more cores than what is available in one group.
 * Will return \ref PN_ERR_NOIMP when called with more cores.
 * 
 * Sets numcores-1 CoPUs to Mode 2 (unlinked Mode) so they start executing the 
 * following code in parallel. This function will return an error
 * (\ref PN_ERR_MATCH) if the CoPUs are not all halted.
 * 
 * See documentation of \ref ms for more information.
 *
 * Cannot be executed on CoPU. Cannot be mixed with linked mode or other 
 * begin threaded functions, until \ref pn_end_threaded() is called.
 * 
 * \attention All data that you want to preserve after \ref pn_end_threaded()
 * was called can not be stored on stack. You can make it static or global.
 * 
 * \warning There is no guarantee that the execution of code actually happens at
 * the same time on CePUs and CoPUs.
 *
 * \param[in]  numcores is the number of cores that shall run threaded.
 *             A value of 0 or 1 will return an error (\ref PN_ERR_PARAM) to aid
 *             debugging.
 * \return     The ID of the core, or \ref PN_ERR_MATCH, \ref PN_ERR_PARAM, or
 *             \ref PN_ERR_COPU.
 */
PN_CID pn_begin_threaded(PN_NUMC numcores);

/**
 * \fn         PN_CID pn_begin_threaded_m(PN_CMSK coremask)
 * \brief      Puts the CPUs specified in the coremask in threaded mode.
 * 
 * \internal
 * \todo This functions specification depends a lot on ParaNut hardware design.
 * In later implementations, using linked and threaded mode in a mixed way might
 * be possible. The documentation and implementation will have to be changed,
 * then.
 * \endinternal
 * 
 * Sets the CoPUs represented by the bitmask to Mode 2 (unlinked Mode) so they 
 * start executing the following code in parallel. This function 
 * will return an error (\ref PN_ERR_MATCH) if the CoPUs are not all halted.
 * 
 * See documentation of \ref ms for more information.
 *
 * Cannot be executed on CoPU. Cannot be mixed with linked mode or other 
 * begin threaded functions, until \ref pn_end_threaded() is called.
 * 
 * \attention All data that you want to preserve after \ref pn_end_threaded()
 * was called can not be stored on stack. You can make it static or global.
 * 
 * \warning There is no guarantee that the execution of code actually happens at
 * the same time on CePUs and CoPUs.
 *
 * \param[in]  coremask is the bitmask representing the CoPUs you want to run
 *             threaded. When all bits are set to 0, this function will return 
 *             an error (\ref PN_ERR_PARAM) to aid debugging. Also, not setting
 *             the first bit to 1 will return an error (\ref PN_ERR_PARAM) since
 *             it represents the CePU (which needs to run threaded, too).
 * \return     The ID of the core, or \ref PN_ERR_MATCH, \ref PN_ERR_PARAM, or
 *             \ref PN_ERR_COPU.
 */
PN_CID pn_begin_threaded_m(PN_CMSK coremask);

/**
 * \fn         PN_CID pn_begin_threaded_gm(PN_CMSK *coremask_array, 
 *                                                           PN_NUMG array_size)
 * \brief      Puts the CPUs specified in the coremask_array in threaded mode.
 *
 * \internal
 * \todo This functions specification depends a lot on ParaNut hardware design.
 * In later implementations, using linked and threaded mode in a mixed way might
 * be possible. The documentation and implementation will have to be changed,
 * then.
 * \endinternal
 * 
 * \todo Currently only a stub. Will therefore always return \ref PN_ERR_NOIMP.
 * 
 * Sets the CoPUs represented by bitmask and their position in the array (=
 * their group number) to Mode 2 (unlinked Mode) so they start executing the 
 * following code in parallel. This function will return an error 
 * (\ref PN_ERR_MATCH) if the CoPUs are not all halted.
 * 
 * See documentation of \ref ms for more information.
 *
 * Cannot be executed on CoPU. Cannot be mixed with linked mode or other 
 * begin threaded functions, until \ref pn_end_threaded() is called.
 * 
 * \attention All data that you want to preserve after \ref pn_end_threaded()
 * was called can not be stored on stack. You can make it static or global.
 * 
 * \warning There is no guarantee that the execution of code actually happens at
 * the same time on CePUs and CoPUs.
 *
 * \param[in]  coremask_array is a pointer to the start of a coremask array. The
 *             position of the mask in the array represents the group number of
 *             the cores. When all bits are set to 0, this function will return 
 *             an error (\ref PN_ERR_PARAM) to aid debugging. Also, not setting 
 *             the first bit to 1 will return an error (\ref PN_ERR_PARAM) since
 *             it represents the CePU (which needs to run threaded, too).
 * \param[in]  array_size is the number of entries in the coremask_array.
 * \return     The ID of the core, or \ref PN_ERR_MATCH, \ref PN_ERR_PARAM, or
 *             \ref PN_ERR_COPU.
 */
PN_CID pn_begin_threaded_gm(PN_CMSK *coremask_array, PN_NUMG array_size);

/**
 * \fn         int pn_end_threaded(void)
 * \brief      Ends threaded execution.
 *
 * On CoPU, halts the core. On CePU, waits until all other cores are halted. 
 * This ends the threaded mode.
 * 
 * See documentation of \ref ms for more information.
 * 
 * \return     Either \ref PN_SUCCESS or \ref PN_ERR_MATCH.
 */
int pn_end_threaded(void);

/**
 * @}
 */
 
/**
 * @}
 */

/*Cache Module*****************************************************************/

/**
 * \defgroup   ca Cache Module
 *
 * \brief      Special functions for controlling the shared ParaNut cache.
 * 
 * \internal
 * 
 * If you want to see how to add a new cache line size, check 
 * \ref PN_ERR_CACHE_LINESIZE.
 * 
 * \todo May need some changes in the future when ParaNut cores get some cache
 * on their own or a proper cache controller is implemented.
 * \todo Virtual addressing is not implemented in the current ParaNut
 * implementation, but may be featured later on. When the day comes, put in a 
 * physical address calculation in the implementation of these functions.
 * \endinternal
 */
 
/**
 * \addtogroup ca
 * @{
 */
 
/**
 * @{
 */
 
/**
 * \fn         int pn_cache_init(void)
 * \brief      Function that has to be called in the main function before any
 *             of the functions in the \ref ca can be called. Initializes some
 *             internal data and enables the cache.
 * 
 * Can only be used on CePU.
 * 
 * \return     Either \ref PN_SUCCESS, \ref PN_ERR_COPU, or 
 *             \ref PN_CACHE_LINESIZE.
 */
int pn_cache_init(void);

/**
 * \fn         int pn_cache_enable(void)
 * \brief      Enables instruction and data cache. When changing this, make sure
 *             that the \ref pn_cache_init() function is still correct!
 * 
 * \attention Careful here: ParaNut Cache is flushed completely before
 * disabling.
 * 
 * Can only be used on CePU.
 * 
 * \return     Either \ref PN_SUCCESS or \ref PN_ERR_COPU.
 */
int pn_cache_enable(void);

/**
 * \fn         int pn_cache_disable(void)
 * \brief      Disables instruction and data cache.
 * 
 * \attention Careful here: ParaNut Cache is flushed completely before
 * disabling.
 * 
 * \warning Atomic memory operations (\ref sp) are not possible on a disabled 
 * cache.
 * 
 * Can only be used on CePU.
 * 
 * \return     Either \ref PN_SUCCESS or \ref PN_ERR_COPU.
 */
int pn_cache_disable(void);
 
/**
 * \fn         unsigned long pn_cache_linesize(void)
 * \brief      Returns the cache line size in bit.
 * 
 * \return     The cache line size in bit. Can not return an error.
 */
unsigned long pn_cache_linesize(void);

/**
 * \fn         unsigned long pn_cache_size(void)
 * \brief      Returns the cache size in Byte.
 * 
 * \return     The cache size in byte. Can not return an error.
 */
unsigned long pn_cache_size(void);

/**
 * \fn         int pn_cache_invalidate(void *addr, unsigned long size)
 * \brief      Invalidates the cache entries containing the given address range.
 *
 * \param[in]  addr is the (virtual) start address of the memory you want to 
 *             invalidate.
 * \param[in]  size is the size of the address range you want to invalidate in 
 *             byte. The size will always be aligned to the cache line size. 
 * \return     Either \ref PN_SUCCESS or \ref PN_ERR_PARAM if given size is
 *             bigger than memory size.
 */
int pn_cache_invalidate(void *addr, unsigned long size);

/**
 * \fn         int pn_cache_invalidate_all(void)
 * \brief      Invalidates the whole cache. When changing this, make sure that 
 *             the \ref pn_cache_init() function is still correct!
 * \return     Can only return \ref PN_SUCCESS, is not made void for the sake of
 *             making the internal implementation more similar for cache
 *             functions.
 */
int pn_cache_invalidate_all(void);

/**
 * \fn         int pn_cache_writeback(void *addr, unsigned long size)
 * \brief      Writes back the cache lines that cached the given address range.
 *
 * \param[in]  addr is the (virtual) start address of the memory you want 
 *             written back.
 * \param[in]  size is the size of the address range you want written back in 
 *             byte. The size will always be aligned to the cache line size.
 * \return     Either \ref PN_SUCCESS or \ref PN_ERR_PARAM if given size is
 *             bigger than memory size.
 */
int pn_cache_writeback(void *addr, unsigned long size);

/**
 * \fn         int pn_cache_writeback_all(void)
 * \brief      Writes whole cache back.
 * \return     Can only return \ref PN_SUCCESS, is not made void for the sake of
 *             making the internal implementation more similar for cache
 *             functions.
 */
int pn_cache_writeback_all(void);

/**
 * \fn         int pn_cache_flush(void *addr, unsigned long size)
 * \brief      Combination of \ref pn_cache_invalidate() and \ref 
 *             pn_cache_writeback().
 *
 * \param[in]  addr is the (virtual)start address of the memory you want 
 *             flushed.
 * \param[in]  size is the size of the address range you want flushed in 
 *             byte. The size will always be aligned to the cache line size. 
 * \return     Either \ref PN_SUCCESS or \ref PN_ERR_PARAM if given size is
 *             bigger than memory size.
 */
int pn_cache_flush(void *addr, unsigned long size);

/**
 * \fn         int pn_cache_flush_all(void)
 * \brief      Flushes the whole cache.
 * \return     Can only return \ref PN_SUCCESS, is not made void for the sake of
 *             making the internal implementation more similar for cache
 *             functions.
 */
int pn_cache_flush_all(void);

/**
 * @}
 */
 
/**
 * @}
 */

/*Exception Module*************************************************************/

/**
 * \defgroup   ex Exception Module
 *
 * \brief      Functions for controlling the handling of interrupts/exceptions.
 * 
 * Why are we calling this exceptions, even though most people would call this
 * an interrupt? Historic reasons. The libparanut was first written for the 
 * RISCV implementation of the ParaNut, and the RISCV specification refers to
 * both interrupts and exceptions as exceptions.
 * 
 * \internal
 * \todo All of these functions may be too RISCV specific to ever use them on
 * another architecture. I have not done more research on this topic yet. If
 * that's the case, we might even need to add architecture specific .c files
 * for this module. I would name them pn_cache_RV32I.c, pn_cache_RV64I.c, and so
 * on. The Makefile and Documentation will probably need some changes then.
 * 
 * Also, when the ParaNut has a more advanced mstatus register, the
 * API implementation of the \ref pn_interrupt_enable() and 
 * \ref pn_interrupt_disable() functions has to change.
 * \endinternal
 */
 
/**
 * \addtogroup ex
 * @{
 */
 
/**
 * @{
 */
 
/**
 * \fn         void pn_exception_init(void)
 * \brief      Initializes libparanut internal exception handling. Interrupts
 *             (not exceptions in general!) are disabled after. Should be 
 *             called before using \ref pn_exception_set_handler().
 */
void pn_exception_init(void);

/**
 * \fn         int pn_exception_set_handler(
 *                                           void (*handler)(
 *                                              unsigned int cause, 
 *                                              unsigned int program_counter, 
 *                                              unsigned int mtval),
 *                                           unsigned int exception_code
 *                                         )
 * \brief      Set your own exception handler.
 * 
 * Can be called without using \ref pn_exception_init() first, will not work
 * though.
 *
 * Already does the work of saving away registers, setting program counter etc.
 * for you. You can just hang in what you want to do.
 * 
 * \attention For exceptions, the register that contains the adress where
 * execution resumes is set to the faulty instruction that threw the exception.
 * For interrupts, it already points to the instruction where execution should
 * resume. Consider this in your handler. If you need to, use 
 * \ref pn_progress_mepc.
 *
 * \param[in]  handler is a function pointer to your exception handler. Will
 *             return an error (\ref PN_ERR_PARAM) if NULL is given.
 * \param[in]  exception_code is the number that your exception has. You can 
 *             look up the exception codes in the ParaNut Manual. For 
 *             interrupts, the value of the most significant bit of the 
 *             exception code has to be 1. For synchronous exceptions, it has
 *             to be 0. This function will return an error (\ref PN_ERR_EXC) if 
 *             a non implemented value for cause is given.
 * \return     Either \ref PN_SUCCESS, \ref PN_ERR_EXC, or \ref PN_ERR_PARAM.
 */
int pn_exception_set_handler(
                              void (*handler)(
                                 unsigned int cause, 
                                 unsigned int program_counter, 
                                 unsigned int mtval),
                              unsigned int exception_code
                            );
    
/**
 * \fn         void pn_ecall(void)
 * \brief      Raises an environment call exception. 
 * 
 * Can be called without using \ref pn_exception_init() first.
 */
void pn_ecall(void);                        
                        
/**
 * \fn         void pn_interrupt_enable(void)
 * \brief      Enables interrupts only. 
 * 
 * Can be called without using \ref pn_exception_init() first.
 */
void pn_interrupt_enable(void);
                        
/**
 * \fn         void pn_interrupt_disable(void)
 * \brief      Disables interrupts only.
 * 
 * Can be called without using \ref pn_exception_init() first.
 */
void pn_interrupt_disable(void);

/**
 * \fn         void pn_progress_mepc(void)
 * \brief      Sets program counter of the register which keeps the exception
 *             return adress to next instruction.
 * 
 * Can be called without using \ref pn_exception_init() first.
 */
void pn_progress_mepc(void);

/**
 * @}
 */
 
/**
 * @}
 */

/*Spinlock Module**************************************************************/

/**
 * \defgroup   sp Spinlock Module
 * \brief      Functions and structure used for synchronizing memory access.
 *
 * \warning
 * The functions in here are really kinda performance critical, since it is
 * always important to do as little as possible when you have reserved a
 * memory area. This means that the functions will do extremly little security
 * checks, which means you have to use them the way they are described.
 * Read the detailed descriptions of the functions carefully. Or don't. I'm not
 * the coding police.
 * 
 * \warning
 * Using the spinlock functions in linked Mode (see \ref li and \ref ms) results
 * in undefined behaviour.
 * 
 * \warning
 * If you want to use the Spinlock Module on RISC-V ParaNut, your ParaNut has to
 * support the A Extension and you have to compile your application with the 
 * flag mabi=rv32ia. The libparanut \ref Makefile sets this flag automatically 
 * when you chose the \ref sp or one of the Modules that has \ref sp as a 
 * dependency.
 * 
 * Return value of functions is always error code, except when stated otherwise 
 * in the description of the function.
 */
 
/**
 * \addtogroup sp
 * @{
 */

/**
 * \struct     __pn_spinlock
 * \brief      A synchronization primitive. Use \ref _pn_spinlock instead of
 *             this.
 *
 * A simple implementation of a synchronization primitive. Might be used for
 * implementing POSIX Threads later on.
 * 
 * \warning
 * You are not supposed to touch anything in this struct, which is why you can't
 * see anything about the members in this documentation.
 * 
 * \attention I highly recommend to not put locks on stack. The reason is that
 * the stack (or at least a part of it) will be copied when putting the ParaNut
 * into threaded Mode (see \ref pn_begin_threaded()). In other words, if you
 * put a lock on stack, it will be copied, and the new instances of the lock
 * will be available per core. You should make it static, global or get 
 * the memory by allocation.
 * 
 * \warning Atomic memory operations are not possible on a disabled cache.
 * 
 * \internal Note for other architectures: Check your ABI on how structs are
 * aligned and what data size integer type has. On RV32I GCC, I have no problems
 * with this, since int size is 4 bytes, PN_CID is just int32 (also 4 bytes),
 * and ABI convention says that int is aligned at 4 bytes. I assume that GCC
 * uses the convention, but switching to an obscure compiler could destroy that.
 * So the stuff in here for GCC can just be treated as lying right behind each
 * other in 4 bytes of memory, which is convenient for me. It should work on 
 * all other compilers using the ABI convention, but I just didn't want this to
 * go unsaid.
 */
typedef struct __pn_spinlock 
{
   PN_CID owner_ID;  /**< \internal ID of the CPU that owns me. Can also be
                      *             negative to represent statuses free (not
                      *             locked by anyone) or dead (not initialized).
                      */
} _pn_spinlock;

/**
 * @{
 */

/**
 * \fn         int pn_spinlock_init(_pn_spinlock *spinlock)
 * \brief      Creates a lock.
 *
 * You allocate the space for the spinlock. Really don't care where you get it 
 * from (check \ref __pn_spinlock for a recommendation). You pass a reference to
 * this function, and the function initializes it for you. And you shall never 
 * touch what's in it.
 *
 * Afterwards, you can use the other functions in this module on the same lock.
 * Behaviour will always be undefined if you don't call this function first.
 * 
 * The function does not care if your lock was already initialized. It will fail
 * if the CPU did not get the memory reservation (\ref PN_ERR_LOCKOCC), which 
 * should never happen. This is a very good indicator that something is very 
 * wrong with your program.
 *
 * After initialization, the lock is free. It will not be automatically owned by
 * the hart that initialized it.
 *
 * \param      spinlock is a pointer to the lock. The function will return
 *             \ref PN_ERR_PARAM if NULL is passed.
 * \return     Either \ref PN_SUCCESS, \ref PN_ERR_PARAM, or 
 *             \ref PN_ERR_LOCKOCC. If something internally went very wrong, the
 *             function is also theoretically able to return \ref PN_ERR_NOIMP.
 */
int pn_spinlock_init(_pn_spinlock *spinlock);

/**
 * \fn         int pn_spinlock_lock(_pn_spinlock *spinlock)
 * \brief      Waits for a lock. Forever, if it must. Use with caution.
 * 
 * Behaviour of this function is undefined if the lock wasn't initialized by 
 * \ref pn_spinlock_init().
 *
 * \warning
 * The function will be stuck in eternity if the lock is already in the current 
 * harts posession, or if someone else owns the lock and forgot to unlock it, or 
 * if the lock was destroyed.
 *
 * \param      spinlock is a pointer to a lock that we want to aquire. The 
 *             function will return \ref PN_ERR_PARAM if NULL is passed.
 * \return     Either \ref PN_SUCCESS or \ref PN_ERR_PARAM.
 */
int pn_spinlock_lock(_pn_spinlock *spinlock);

/**
 * \fn         int pn_spinlock_trylock(_pn_spinlock *spinlock);
 * \brief      Tries to acquire a lock. Nonblocking.
 *
 * Behaviour of this function is undefined if the lock wasn't initialized by 
 * \ref pn_spinlock_init().
 *
 * Will fail if lock is already owned (no matter by whom), another CPU got the
 * memory reservation, or if lock was destroyed.
 * 
 * \param      spinlock is a pointer to a lock that we want to aquire. The 
 *             function will return \ref PN_ERR_PARAM if NULL is passed.
 * \return     Either \ref PN_SUCCESS, \ref PN_ERR_LOCKOCC or \ref PN_ERR_PARAM.
 *             If something internally went very wrong, the
 *             function is also theoretically able to return \ref PN_ERR_NOIMP.
 */
int pn_spinlock_trylock(_pn_spinlock *spinlock);

/**
 * \fn         int pn_spinlock_unlock(_pn_spinlock *spinlock)
 * \brief      Unlocks a lock.
 * 
 * Behaviour of this function is undefined if the lock wasn't initialized by 
 * \ref pn_spinlock_init().
 *
 * Will fail is lock is not owned by current hart (\ref PN_ERR_PARAM).
 *
 * \param      spinlock is a pointer to a lock that we want to unlock. The 
 *             function will return \ref PN_ERR_PARAM if NULL is passed.
 * \return     Either \ref PN_SUCCESS or \ref PN_ERR_PARAM.
 *             If something internally went very wrong, the
 *             function is also theoretically able to return \ref PN_ERR_NOIMP.
 */
int pn_spinlock_unlock(_pn_spinlock *spinlock);

/**
 * \fn         int pn_spinlock_destroy(_pn_spinlock *spinlock)
 * \brief      Destroys a lock.
 *
 * Behaviour of this function is undefined if the lock wasn't initialized by 
 * \ref pn_spinlock_init().
 *
 * A destroyed lock can be re-initialized by using \ref pn_spinlock_init().
 *
 * The lock can either be owned by current hart or unlocked, else the function
 * will fail.
 *
 * \param      spinlock is a pointer to a lock that we want to destroy. The 
 *             function will return \ref PN_ERR_PARAM if NULL is passed.
 * \return     Either \ref PN_SUCCESS, \ref PN_ERR_LOCKOCC or \ref PN_ERR_PARAM.
 *             If something internally went very wrong, the
 *             function is also theoretically able to return \ref PN_ERR_NOIMP.
 */
int pn_spinlock_destroy(_pn_spinlock *spinlock);

/**
 * @}
 */
 
/**
 * @}
 */
 
/**
 * @}
 */
 
/*EOF**************************************************************************/
