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
 * \internal
 * \file
 * \brief         Contains defines used in assembly functions of all modules.
 *
 * \includelineno common/custom_RV32I.S
 */

/*ParaNut Control and Status Registers*****************************************/
 
/**
 * \internal
 * \defgroup   pn_reg ParaNut Control and Status Registers
 * \brief      Architecture dependent defines for using the adresses of ParaNut
 *             Control and Status Registers.
 *
 * Even though it's a convention to write defines in all capital letters, I'm
 * writing all of these in small letters to use them more like I would use a
 * normal RISC-V register name.
 * 
 * Check out the ParaNut Manual for more information.
 */
 
/**
 * \addtogroup pn_reg
 * @{
 */
 
/**
 * @{
 */

/**
 * \internal
 * \def        pncache
 * \brief      ParaNut Cache control register.
 */
#define pncache      0x7C0

/**
 * \internal
 * \def        pngrpsel
 * \brief      ParaNut CPU group select register.
 */
#define pngrpsel     0x8C0

/**
 * \internal
 * \def        pnce
 * \brief      ParaNut CPU enable register.
 */
#define pnce         0x8C1

/**
 * \internal
 * \def        pnlm
 * \brief      ParaNut CPU linked mode register.
 */
#define pnlm         0x8C2

/**
 * \internal
 * \def        pnxsel
 * \brief      ParaNut CoPU exception select register.
 */
#define pnxsel       0x8C3

/**
 * \internal
 * \def        pnm2cap
 * \brief      ParaNut CPU capabilities register.
 */
#define pnm2cap      0xFC0

/**
 * \internal
 * \def        pnx
 * \brief      ParaNut CoPU exception pending register.
 */
#define pnx          0xFC1

/**
 * \internal
 * \def        pncause
 * \brief      ParaNut CoPU trap cause ID register.
 */
#define pncause      0xFC2

/**
 * \internal
 * \def        pnepc
 * \brief      ParaNut CoPU exception program counter register.
 */
#define pnepc        0xFC3

/**
 * \internal
 * \def        pncacheinfo
 * \brief      ParaNut cache information register.
 */
#define pncacheinfo  0xFC4

/**
 * \internal
 * \def        pncachesets
 * \brief      ParaNut number of cache sets register.
 */
#define pncachesets  0xFC5

/**
 * \internal
 * \def        pnclockinfo
 * \brief      ParaNut clock speed information register.
 */
#define pnclockinfo  0xFC6

/**
 * \internal
 * \def        pnmemsize
 * \brief      ParaNut memory size register.
 */
#define pnmemsize    0xFC7

/**
 * \internal
 * \def        pncpus
 * \brief      ParaNut number of CPUs register.
 */
#define pncpus       0xCD0

/**
 * \internal
 * \def        pncoreid
 * \brief      ParaNut core ID register.
 */
#define pncoreid       0xCD4

/**
 * \internal
 * \def        mtval
 * \brief      Machine Trap Value Register.
 *
 * Compiler does not recognize this one for some reason, even though it's 
 * RISC-V and not ParaNut specific. 
 */
#define mtval        0x343

/**
 * @}
 */
 
/**
 * @}
 */

/*ParaNut Custom Instructions**************************************************/

/**
 * \internal
 * \defgroup   pn_cus ParaNut Custom Instructions
 * \brief      Architecture dependent defines for ParaNut Custom Instructions
 *             which have to be added as binary machine code.
 *
 * Check out the ParaNut Manual for more information. The translation to
 * bytecode can be taken from RISCV specification.
 */
 
/**
 * \addtogroup pn_cus
 * @{
 */
 
/**
 * @{
 */

/**
 * \internal
 * \def        HALT
 * \brief      Halts the core which this instruction is executed on.
 */
#define HALT      .word 0x0000000B

/**
 * \internal
 * \def        CINV(regnum, offset)
 * \brief      Cache invalidate.
 *
 * \param[in]  regnum is the number of the hardware register that contains the
 *             base adress.
 * \param[in]  offset is the offset that is added to the base adress. The
 *             resulting adress is the memory which is cached and you want to
 *             invalidate.
 */
#define CINV(regnum, offset)                                                   \
                  .word (0x100B | ((regnum) << 15) | (((offset) & 0xfff) << 20))
                  
/**
 * \internal
 * \def        CWB(regnum, offset)
 * \brief      Cache writeback.
 *
 * \param[in]  regnum is the number of the hardware register that contains the
 *             base adress.
 * \param[in]  offset is the offset that is added to the base adress. The
 *             resulting adress is the memory which is cached and you want to
 *             write back.
 */
#define CWB(regnum, offset)                                                    \
                  .word (0x200B | ((regnum) << 15) | (((offset) & 0xfff) << 20))
                  
/**
 * \internal
 * \def        CINV(regnum, offset)
 * \brief      Cache flush.
 *
 * \param[in]  regnum is the number of the hardware register that contains the
 *             base adress.
 * \param[in]  offset is the offset that is added to the base adress. The
 *             resulting adress is the memory which is cached and you want to
 *             flush.
 */
#define CFLUSH(regnum, offset)                                                 \
                  .word (0x300B | ((regnum) << 15) | (((offset) & 0xfff) << 20))

/**
 * @}
 */
 
/**
 * @}
 */
                  
/*EOF**************************************************************************/

