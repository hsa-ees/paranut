# Copyright 2019-2020 Anna Pfuetzner (<annakerstin.pfuetzner@gmail.com>)
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
# this list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation
# and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

## @file

##
#  \file
#  \brief Builds the architecture dependent assembly files of the \ref ca since
#  they are performance critical, very dependent on cache line size, and should
#  not take up to much space.
#
#  This means we can neither make the cache line size a variable, since
#  that would mean a performance trade off, nor can we put an implementation for
#  every possible cache line size, since that would take a giant amount of space
#  in the binary.
#  This Python Script seems the perfect solution for that, except for the fact
#  that it reduces binary compatibility of the libparanut to ParaNuts with
#  different cache line sizes. Having to re-compile the libparanut is, of
#  of course, a minor inconvenience, but the only other solution, which would be
#  code that modifies itself on startup, is extremely complex and could be a
#  possible subject for another Bachelor Thesis.
#
#  For all those who hate having to recompile stuff, it is also possible to
#  start this script with the option "auto", which generates a file that does
#  contain all of the cache line size functions from 32 bit to 2048 in powers of
#  two. Since every other linesize is deemed to be unrealistic, this version has
#  the most binary compatibility ... but also the biggest code size of them all.
#  Make of that what you want.
#
#  Also check documentation of \ref PN_CACHE_LINESIZE.
#
#  \internal
#  You can take a look at an example (\ref pn_cache_RV32I_auto.S) of the files
#  built with this script.
#
#  Also, if you want to know how to introduce a new cache line size into the
#  libparanut, take a look at \ref PN_ERR_CACHE_LINESIZE.
#  \includelineno pn_cache/pn_cache_RV32I_buildscript.py
#  \endinternal
#

#Imports########################################################################

# enable command line argument parsing
import sys

# enable ceil function
import math

#Global Variables###############################################################

cache_line_size_list = "32", "64", "128", "256", "512", "1024", "2048"
function_list        = "invalidate", "writeback", "flush"

#Subfunctions###################################################################

def create_warning(filep):

  filep.write(
    "/* \n"                                                                  +
    " * I was automatically generated by pn_cache_RV32I_buildscript.py,\n"   +
    " * invoked by libparanut Makefile -> No edits!\n"                       +
    " */\n"                                                                  +
    "\n"
  )

################################################################################

def create_doxy(filep, specific_size):

  filep.write(
    "/** @file */\n"                                                         +
    "\n"                                                                     +
    "/**\n"                                                                  +
    " * \\internal\n"                                                        +
    " * \\file\n"                                                            +
    " * \\brief        Contains RV32I assembly implementations of assembly\n"
  )

  if specific_size:

    filep.write(
    " *               functions called in \\ref pn_cache.c for cache line\n" +
    " *               size " + sys.argv[1] + ".\n"                           +
    " *\n"
    )

  else:

    filep.write(
    " *               functions called in \\ref pn_cache.c for all cache \n" +
    " *               line sizes.\n"                                         +
    " *\n"
    )

  filep.write(
    " * This file (for this specific architecture, others may do it\n"       +
    " * differently) was created by \\ref pn_cache_RV32I_buildscript.py\n"   +
    " * while compiling (automatically called by \\ref Makefile), as\n"      +
    " * are the other files with the pattern pn_cache_RV32I_*.S for name.\n" +
    " * The reasons for this are described in the \\ref\n"                   +
    " * pn_cache_RV32I_buildscript.py documentation itself.\n"               +
    " *\n"
  )

  if specific_size:

    filep.write(
    " * The code shown below was created with a cache line size of "         +
        sys.argv[1] + " Bit.\n"                                              +
    " *\n"
    )

  else:

    filep.write(
    " * The code shown below was created for all cache line sizes.\n"        +
    " *\n"
    )

  filep.write(
    " * So far, the following cache line sizes are available (in bits):\n"   +
    " * " + " ".join(cache_line_size_list) + "\n"                            +
    " *\n"                                                                   +
    " * Contains implementations of:\n"                                      +
    " *    - \\ref enable_cache_as()\n"                                      +
    " *    - \\ref disable_cache_as()\n"                                     +
    " *    - \\ref cache_banks_as()\n"                                       +
    " *    - \\ref cache_sets_as()\n"                                        +
    " *    - \\ref mem_size_as()\n"
  )

  if specific_size:

    filep.write(
    " *    - \\ref invalidate_" + sys.argv[1] + "_as()\n"                    +
    " *    - \\ref invalidate_bulk_" + sys.argv[1] + "_as()\n"               +
    " *    - \\ref writeback_" + sys.argv[1] + "_as()\n"                     +
    " *    - \\ref writeback_bulk_" + sys.argv[1] + "_as()\n"                +
    " *    - \\ref flush_" + sys.argv[1] + "_as()\n"                         +
    " *    - \\ref flush_bulk_" + sys.argv[1] + "_as()\n"                    +
    " *\n"
    )

  else:

    for cache_line_size in cache_line_size_list:
      filep.write(
    " *    - \\ref invalidate_" + cache_line_size + "_as()\n"                +
    " *    - \\ref invalidate_bulk_" + cache_line_size + "_as()\n"           +
    " *    - \\ref writeback_" + cache_line_size + "_as()\n"                 +
    " *    - \\ref writeback_bulk_" + cache_line_size + "_as()\n"            +
    " *    - \\ref flush_" + cache_line_size + "_as()\n"                     +
    " *    - \\ref flush_bulk_" + cache_line_size + "_as()\n"
    )

    filep.write(" *\n")

  filep.write(
    " * \\includelineno pn_cache/pn_cache_RV32I_" + sys.argv[1] + ".S\n"     +
    " */\n\n"
  )

  filep.write(
    "/*\n"                                                                   +
    " * Put in here so Doxygen will know that it is implemented in this\n"   +
    " * file. Sadly, Doxygen has no built in assembly interpreter, so we\n"  +
    " * are stuck with this.\n"                                              +
    " */\n"                                                                  +
    "\n"                                                                     +
    "#ifdef DOXYGEN\n"                                                       +
    "\n"                                                                     +
    "  /**\n"                                                                +
    "    * \\addtogroup as\n"                                                +
    "    * @{\n"                                                             +
    "    */\n"                                                               +
    "\n"                                                                     +
    "  /**\n"                                                                +
    "    * @{\n"                                                             +
    "    */\n"                                                               +
    "\n"
  )

  filep.write(
    "  /**\n"                                                                +
    "   * \\internal\n"                                                      +
    "   * \\fn         void enable_cache_as(void)\n"                         +
    "   * \\brief      Enables cache.\n"                                     +
    "   */\n"                                                                +
    "   void enable_cache_as(void) {}\n"                                     +
    "\n"                                                                     +
    "  /**\n"                                                                +
    "   * \\internal\n"                                                      +
    "   * \\fn         void disable_cache_as(void)\n"                        +
    "   * \\brief      Disables cache.\n"                                    +
    "   */\n"                                                                +
    "   void disable_cache_as(void) {}\n"                                    +
    "\n"                                                                     +
    "  /**\n"                                                                +
    "   * \\internal\n"                                                      +
    "   * \\fn         unsigned int cache_banks_as(void)\n"                  +
    "   * \\brief      Returns number of cache banks.\n"                     +
    "   *\n"                                                                 +
    "   * \\return     Number of cache banks.\n"                             +
    "   */\n"                                                                +
    "   unsigned int cache_banks_as(void) {}\n"                              +
    "\n"                                                                     +
    "  /**\n"                                                                +
    "   * \\internal\n"                                                      +
    "   * \\fn         unsigned int cache_sets_as(void)\n"                   +
    "   * \\brief      Returns number of cache sets.\n"                      +
    "   *\n"                                                                 +
    "   * \\return     Number of cache sets.\n"                              +
    "   */\n"                                                                +
    "   unsigned int cache_sets_as(void) {}\n"                               +
    "\n"                                                                     +
    "  /**\n"                                                                +
    "   * \\internal\n"                                                      +
    "   * \\fn         unsigned int mem_size_as(void)\n"                     +
    "   * \\brief      Reads memory size.\n"                                 +
    "   *\n"                                                                 +
    "   * \\return     Content of pnmemsize register.\n"                     +
    "   */\n"                                                                +
    "   unsigned int mem_size_as(void) {}\n"                                 +
    "\n"
  )

  if specific_size:
    for function in function_list:

      action = ""
      if function == "invalidate":
        action = "Invalidates "
      if function == "writeback":
        action = "Writes back "
      if function == "flush":
        action = "Flushes "

      filep.write(
    "  /**\n"                                                                +
    "   * \\internal\n"                                                      +
    "   * \\fn         void " + function + "_" + sys.argv[1]                 +
                            "_as(unsigned int addr, unsigned int endaddr)\n" +
    "   * \\brief      " + action + "single cache lines until end address\n" +
    "   *             is reached.\n"                                         +
    "   *\n"                                                                 +
    "   * Will at least " + function + " one line.\n"                        +
    "   */\n"                                                                +
    "   void " + function + "_" + sys.argv[1]                                +
                       "_as(unsigned int addr, unsigned int endaddr) {}\n"   +
    "\n"                                                                     +
    "  /**\n"                                                                +
    "   * \\internal\n"                                                      +
    "   * \\fn         void " + function + "_bulk_" + sys.argv[1]            +
                            "_as(unsigned int addr, unsigned int endaddr)\n" +
    "   * \\brief      " + action + "32 cache lines at once until end \n"    +
    "   *             address is reached.\n"                                 +
    "   *\n"                                                                 +
    "   * Will at least " + function + " 32 lines.\n"                        +
    "   */\n"                                                                +
    "   void " + function + "_bulk_" + sys.argv[1]                           +
                       "_as(unsigned int addr, unsigned int endaddr) {}\n"   +
    "\n"
      )


  else:
    for cache_line_size in cache_line_size_list:
      for function in function_list:

        action = ""
        if function == "invalidate":
          action = "Invalidates "
        if function == "writeback":
          action = "Writes back "
        if function == "flush":
          action = "Flushes "

        filep.write(
    "  /**\n"                                                                +
    "   * \\internal\n"                                                      +
    "   * \\fn         void " + function + "_" + cache_line_size             +
                            "_as(unsigned int addr, unsigned int endaddr)\n" +
    "   * \\brief      " + action + "single cache lines until end address\n" +
    "   *             is reached.\n"                                         +
    "   *\n"                                                                 +
    "   * Will at least " + function + " one line.\n"                        +
    "   */\n"                                                                +
    "   void " + function + "_" + cache_line_size                            +
                       "_as(unsigned int addr, unsigned int endaddr) {}\n"   +
    "\n"                                                                     +
    "  /**\n"                                                                +
    "   * \\internal\n"                                                      +
    "   * \\fn         void " + function + "_bulk_" + cache_line_size        +
                            "_as(unsigned int addr, unsigned int endaddr)\n" +
    "   * \\brief      " + action + "32 cache lines at once until end \n"    +
    "   *             address is reached.\n"                                 +
    "   *\n"                                                                 +
    "   * Will at least " + function + " 32 lines.\n"                        +
    "   */\n"                                                                +
    "   void " + function + "_bulk_" + cache_line_size                       +
                       "_as(unsigned int addr, unsigned int endaddr) {}\n"   +
    "\n"
        )

  filep.write(
    "  /**\n"                                                                +
    "    * @}\n"                                                             +
    "    */\n"                                                               +
    "\n"                                                                     +
    "  /**\n"                                                                +
    "    * @}\n"                                                             +
    "    */\n"                                                               +
    "\n"                                                                     +
    "#endif /* DOXYGEN */\n"                                                 +
    "\n"
  )

################################################################################

def linker_instructions(filep, specific_size):

  filep.write(
    "/*Header**************************************************************" +
                                                              "*********/\n" +
    "\n"                                                                     +
    "#ifndef DOXYGEN\n"                                                      +
    "\n"                                                                     +
    ".text                                  /* enter text section          " +
                                                              "        */\n" +
    ".align 2                               /* align Code to 2^2 Bytes     " +
                                                              "        */\n" +
    "\n"                                                                     +
    "/* declare labels in here to be global */\n"                            +
    ".globl enable_cache_as\n"                                               +
    ".globl disable_cache_as\n"                                              +
    ".globl cache_banks_as\n"                                                +
    ".globl cache_sets_as\n"                                                 +
    ".globl mem_size_as\n"
  )

  if specific_size:

    filep.write(
    ".globl invalidate_" + sys.argv[1] + "_as\n"                             +
    ".globl invalidate_bulk_" + sys.argv[1] + "_as\n"                        +
    ".globl writeback_" + sys.argv[1] + "_as\n"                              +
    ".globl writeback_bulk_" + sys.argv[1] + "_as\n"                         +
    ".globl flush_" + sys.argv[1] + "_as\n"                                  +
    ".globl flush_bulk_" + sys.argv[1] + "_as\n"                             +
    "\n"
    )

  else:

    for cache_line_size in cache_line_size_list:
      filep.write(
    ".globl invalidate_" + cache_line_size + "_as\n"                         +
    ".globl invalidate_bulk_" + cache_line_size + "_as\n"                    +
    ".globl writeback_" + cache_line_size + "_as\n"                          +
    ".globl writeback_bulk_" + cache_line_size + "_as\n"                     +
    ".globl flush_" + cache_line_size + "_as\n"                              +
    ".globl flush_bulk_" + cache_line_size + "_as\n"
      )

    filep.write("\n")

  filep.write(
    "/* ParaNut Custom Registers and Instructions */\n"                      +
    "#include \"custom_RV32I.S\"\n"                                          +
    "\n"
  )

################################################################################

def common_functions(filep):

  filep.write(
    "enable_cache_as:\n"                                                     +
    "   fence\n"                                                             +
    "   li   t0,      3\n"                                                   +
    "   csrs pncache, t0\n"                                                  +
    "   ret\n"                                                               +
    "\n"                                                                     +
    "/*--------------------------------------------------------------------" +
                                                            "--------*/\n\n" +
    "disable_cache_as:\n"                                                    +
    "   fence\n"                                                             +
    "   li   t0,      0\n"                                                   +
    "   csrw pncache, t0\n"                                                  +
    "   ret\n"                                                               +
    "\n"                                                                     +
    "/*--------------------------------------------------------------------" +
                                                            "--------*/\n\n" +
    "cache_banks_as:\n"                                                      +
    "   csrr a0,      pncacheinfo\n"                                         +
    "   srli a0,      a0,      8\n"                                          +
    "   ret\n"                                                               +
    "\n"                                                                     +
    "/*--------------------------------------------------------------------" +
                                                            "--------*/\n\n" +
    "cache_sets_as:\n"                                                       +
    "  csrr a0,       pncachesets\n"                                         +
    "  ret\n"                                                                +
    "\n"                                                                     +
    "/*--------------------------------------------------------------------" +
                                                            "--------*/\n\n" +
    "mem_size_as:\n"                                                         +
    "  csrr a0,       pnmemsize\n"                                           +
    "  ret\n"                                                                +
    "\n"                                                                     +
    "/*--------------------------------------------------------------------" +
                                                              "--------*/\n\n"
  )

################################################################################

def macrocheck(funcname):

  macroname = ""
  if funcname == "invalidate":
    macroname = "   CINV(10, "
  elif funcname == "writeback":
    macroname = "   CWB(10, "
  elif funcname == "flush":
    macroname = "   CFLUSH(10, "
  else:
    raise ValueError('This function name has no matching macro!')

  return macroname

################################################################################

def create_function(filep, funcname, linesize):

  macro = macrocheck(funcname)

  #
  # Single line function
  #

  # put the function name
  filep.write(funcname + "_" + linesize + "_as:\n")

  # put in a fence
  filep.write("   fence\n")

  # if linesize in byte is bigger than 2048, we cannot use addi later
  if (int(linesize) / 8) >= 2048:
    filep.write("   li t0, " + str( int( int(linesize) / 8 ) ) + "\n")

  # put label on loop
  looplabel = funcname + "_" + linesize + "_as_loop"
  filep.write(looplabel + ":\n")

  # actual operation
  filep.write(macro + "0x000)\n")

  # add linesize in byte
  if (int(linesize) / 8) >= 2048:
    filep.write("   add a0, a0, t0\n")
  else:
    filep.write("   addi a0, a0, " + str( int( int(linesize) / 8) ) + "\n")

  # loop wrap
  filep.write(
  "   blt a0, a1, %s\n" % looplabel                                           +
  "   ret\n"                                                                  +
  "\n"                                                                        +
  "/*-----------------------------------------------------------------------" +
  "-----*/\n"                                                                 +
  "\n"
  )

  #
  # Bulk function
  #

  # put the function name
  filep.write(funcname + "_bulk_" + linesize + "_as:\n")

  # put in a fence
  filep.write("   fence\n")

  # get the loop label
  looplabel = funcname + "_bulk_" + linesize + "_as_loop"

  # if cache line in byte size does not fit into 2048 more than once, we can
  # only use the single line action.
  if (int(linesize) / 8) >= 2048:
    filep.write("   j " + funcname + "_" + linesize + "_as\n\n")
    return

  # if the linesize and factors fit with no problem, we can roll them all out
  if ((int(linesize) / 8) * 32) < 2048:

    # put label on loop
    filep.write(looplabel + ":\n")

    # unroll loop 32 times
    for i in range(32):
      filep.write(macro + "%d)\n" %  ( int( int(linesize) / 8) * i) )

    # update start value
    filep.write("   addi a0, a0, %d\n" % ( int( int(linesize) / 8 ) * 32) )

  else:

    # check how many lines can be done at once
    max_lines = int( math.ceil( 2048 / float( int(linesize) / 8) ) )

    # do not use addi later so we can possible fit one more line
    filep.write("li t0, %i\n" % ( int( int(linesize) / 8 ) * max_lines) )

    # put label on loop
    filep.write(looplabel + ":\n")

    # unroll 32 times without branching
    for i in range(0, 32, max_lines):

      # unroll loop as many times as possible without addition
      for j in range(max_lines):

        filep.write(macro + "%d)\n" % ( int( int(linesize) / 8) * j) )

      # update start value
      filep.write("   add a0, a0, t0\n")

    if not ((32 % max_lines) == 0):

      # unroll loop for the missing lines
      for i in range(0, 32 % max_lines, 1):
        filep.write(macro + "%d)\n" % ( int( int(linesize) / 8) * i) )

      # update start value (guaranteed to fit into 2048)
      filep.write(
      "   addi a0, a0, %d\n" % ( int( int(linesize) / 8) * (32 % max_lines))
      )

  # loop wrap
  filep.write(
  "   blt  a0, a1, %s \n" % looplabel                                         +
  "   ret\n"                                                                  +
  "\n"
  )

#Main###########################################################################

#
# Check if we are generating a specific cache line size or the auto file.
#

specific_size = sys.argv[1].isdigit()

#
# Check if cache line size is divisible by 32 or keyword auto.
#

if specific_size:
  if (int(sys.argv[1]) % 32) != 0:
    raise ValueError('Line size is not a multiple of word size')
elif sys.argv[1] != "auto":
  raise ValueError('Given keyword was not \"auto\"!')

#
# Create/open the assembly source code file.
#

filename = "pn_cache/pn_cache_RV32I_" + sys.argv[1] + ".S"
filep = open(filename,"w+")

#
# Put in some pre-code stuff
#

# create warning
create_warning(filep)

# create Doxygen documentation
create_doxy(filep, specific_size)

# put in linker instructions
linker_instructions(filep, specific_size)

#
# Actual function code generation.
#
# Explainations:
#
# a0 (register 10) always contains start address.
# a1 (register 11) always contains end address.
#
# 0x100B is custom opcode for cache invalidate.
# 0x200B is custom opcode for cache writeback.
# 0x300B is custom opcode for cache flush.
#

filep.write(
"/*Functions*****************************************************************" +
                                                                      "***/\n" +
"\n"
)

#
# Put in the functions that every file needs and that are not dependent on line
# size.
#

common_functions(filep)

#
# all other functions for the specific cache line size files
#

if specific_size:

  create_function(filep, "invalidate", sys.argv[1])

  filep.write("/*-----------------------------------------------------------" +
                                                     "-----------------*/\n\n")

  create_function(filep, "writeback", sys.argv[1])

  filep.write("/*-----------------------------------------------------------" +
                                                     "-----------------*/\n\n")

  create_function(filep, "flush", sys.argv[1])

#
# functions for auto file
#

else:

  for cache_line_size in cache_line_size_list:

    for function in function_list:

      create_function(filep, function, cache_line_size)

      if (cache_line_size != cache_line_size_list[-1]):
        if (function != "flush"):
          filep.write("/*-----------------------------------------------" +
                                     "-----------------------------*/\n\n")


filep.write(
"#endif /* !DOXYGEN */\n"                                                      +
"\n"                                                                           +
"/*EOF***********************************************************************" +
                                                                       "***/\n")

#
# close file
#

filep.close()

#EOF############################################################################
