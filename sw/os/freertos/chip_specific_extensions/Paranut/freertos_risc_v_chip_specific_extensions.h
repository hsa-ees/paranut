/*
 * FreeRTOS Kernel V10.4.6
 * Copyright (C) 2021 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * SPDX-License-Identifier: MIT
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * https://www.FreeRTOS.org
 * https://github.com/FreeRTOS
 *
 */

/*
 * The FreeRTOS kernel's RISC-V port is split between the the code that is
 * common across all currently supported RISC-V chips (implementations of the
 * RISC-V ISA), and code that tailors the port to a specific RISC-V chip:
 *
 * + FreeRTOS\Source\portable\GCC\RISC-V-RV32\portASM.S contains the code that
 *   is common to all currently supported RISC-V chips.  There is only one
 *   portASM.S file because the same file is built for all RISC-V target chips.
 *
 * + Header files called freertos_risc_v_chip_specific_extensions.h contain the
 *   code that tailors the FreeRTOS kernel's RISC-V port to a specific RISC-V
 *   chip.  There are multiple freertos_risc_v_chip_specific_extensions.h files
 *   as there are multiple RISC-V chip implementations.
 *
 * !!!NOTE!!!
 * TAKE CARE TO INCLUDE THE CORRECT freertos_risc_v_chip_specific_extensions.h
 * HEADER FILE FOR THE CHIP IN USE.  This is done using the assembler's (not the
 * compiler's!) include path.  For example, if the chip in use includes a core
 * local interrupter (CLINT) and does not include any chip specific register
 * extensions then add the path below to the assembler's include path:
 * FreeRTOS\Source\portable\GCC\RISC-V-RV32\chip_specific_extensions\RV32I_CLINT_no_extensions
 *
 */


#ifndef __FREERTOS_RISC_V_EXTENSIONS_H__
#define __FREERTOS_RISC_V_EXTENSIONS_H__

#define portasmHAS_SIFIVE_CLINT         1
#define portasmHAS_MTIME                1
#define portasmADDITIONAL_CONTEXT_SIZE  ( 3 )
#define portasmHAS_PARANUT_SUPPORT   1
#include "paranut-config.h"

#define CPU_CORE_MASK ((1 << CFG_NUT_CPU_CORES) - 1)

.macro portasmSAVE_ADDITIONAL_REGISTERS
    addi sp, sp, -(portasmADDITIONAL_CONTEXT_SIZE * portWORD_SIZE)
	.endm

.macro portasmRESTORE_ADDITIONAL_REGISTERS
    addi sp, sp, (portasmADDITIONAL_CONTEXT_SIZE * portWORD_SIZE)
	.endm

.macro portasmSAVE_MULTI_CORE_REGISTERS
	addi sp, sp, -(portasmADDITIONAL_CONTEXT_SIZE * portWORD_SIZE) -portCONTEXT_SIZE

    /* Backup a0 first, save seperately, this will be the actual value after a task switch */
    store_x a0, 1 * portWORD_SIZE ( sp )
    //store_x x10, 7 * portWORD_SIZE +portasmADDITIONAL_CONTEXT_SIZE ( sp )	/* a0 */
    /* pnece */
    csrr    a0, 0xFC8 /* pnce 0x8c1 */
    store_x a0, 2 * portWORD_SIZE ( sp )
    /* pnlm */
    csrr    a0, 0x8c2
    store_x a0, 3 * portWORD_SIZE ( sp )

	addi sp, sp, (portasmADDITIONAL_CONTEXT_SIZE * portWORD_SIZE) + portCONTEXT_SIZE
    .endm

.macro portasmRESTORE_MULTI_CORE_REGISTERS
	/* pnce */
    load_x  a0, 2 * portWORD_SIZE -((portasmADDITIONAL_CONTEXT_SIZE * portWORD_SIZE) + portCONTEXT_SIZE) ( sp )
    csrw    0x8c1, a0
	/* pnlm (pnce or pnlm should enable and link all cores prior to this,
		therefore it's safe to potentially turn them off prior to this, and potentially unlink them here) */
	load_x  a0, 3 * portWORD_SIZE -((portasmADDITIONAL_CONTEXT_SIZE * portWORD_SIZE) + portCONTEXT_SIZE) ( sp )
    csrw    0x8c2, a0

	/* All cores that are activated in linked-mode load the content to a0
		If it's CePU (Core 0) however, a0 gets loaded from another source */
	/* Fence is placed here as during testing it has been observed that some cores might not react yet.
		The next instruction however is important. Fence usually synces cores. */
	fence
	csrr	a0,0xcd4
	seqz	a0,a0							/* table[CORE_COUNT=4] = 1,0,0,0 */

	load_x  x11, 7 * portWORD_SIZE -portCONTEXT_SIZE ( sp )	/* a0 to a1 for CoPUs */
	mul     a2,a0,a1	/* temp = table * original; a2 = temp TODO: Consider using AND here after extending a0 with neg */
	/*a2 = 3,0,0,0; This is the component we want to get rid of */
	/*a2 = 3,22,33,44 - 3,0,0,0 = 0,22,33,44; These are the components that are ok for CoPUs */
	sub		a2,a1,a2	/* temp = original - temp */
	load_x  a3, 1 * portWORD_SIZE -((portasmADDITIONAL_CONTEXT_SIZE * portWORD_SIZE) + portCONTEXT_SIZE) ( sp ) /* load a0 for CePU as a3 (= original) */
	mul     a3,a0,a3	/* a3 = 11,0,0,0 */
	add		a0,a2,a3	/* a0 = 11,22,33,44 */	

	load_x  x11, 8 * portWORD_SIZE -portCONTEXT_SIZE ( sp )	/* a1 */
	load_x  x12, 9 * portWORD_SIZE -portCONTEXT_SIZE ( sp )	/* a2 */
	load_x  x13, 10 * portWORD_SIZE -portCONTEXT_SIZE ( sp )	/* a3 */
    .endm

/* Disables all cores except for CePU and restores a0 on that one */
.macro portasmDISABLE_ALL_COPUS
	/* Revert to CePU only */
	li a0, 0x1
	/* Disable cores */
	csrw	0x8c1,a0
	/* Disable linked-mode for cores */
	csrw	0x8c2,a0
    .endm

.macro portasmENABLE_ALL_COPUS
	/* Activate all cores */
	li a0, CPU_CORE_MASK
	/* Enable linked-mode for cores */
	csrw	0x8c2,a0
	/* Enable cores */
	csrw	0x8c1,a0

	fence
    .endm

/* t0 contains base pointer to the array of stack pointers of each core 
    Effectively calculates pointer for the current core by increasing the pointer with the appropriate amount
*/
.macro portasmADD_MULTI_CORE_STACK_OFFSET
	/* Read core id */
	csrr	a0,0xcd4
    #if( __riscv_xlen == 64 )
	/* Core ID * 8 */
	slli	a0,a0,0x3
	add     t0, t0, a0    
    #elif( __riscv_xlen == 32 )
	/* Core ID * 4 */
	slli	a0,a0,0x2
	add     t0, t0, a0
    #endif
	.endm
#endif /* __FREERTOS_RISC_V_EXTENSIONS_H__ */
