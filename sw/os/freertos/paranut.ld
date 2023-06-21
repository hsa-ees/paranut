/*************************************************************************

  This file is part of the ParaNut project.

  Copyright (C) 2019 Anna Pfuetzner <anna.pfuetzner@hs-augsburg.de>
                     Alexander Bahle <alexander.bahle@hs-augsburg.de>
      Hochschule Augsburg, University of Applied Sciences

  Description:
     This is the linker script used when building software for the ParaNut.

  Redistribution and use in source and binary forms, with or without modification,
  are permitted provided that the following conditions are met:

  1. Redistributions of source code must retain the above copyright notice, this 
     list of conditions and the following disclaimer.

  2. Redistributions in binary form must reproduce the above copyright notice,
     this list of conditions and the following disclaimer in the documentation and/or
     other materials provided with the distribution.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND 
  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE 
  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
  ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES 
  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; 
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON 
  ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS 
  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 *************************************************************************/

/*======================================================================*/
/* ParaNut linker script                                           */
/*======================================================================*/

/*----------------------------------------------------------------------*/
/* Setup                                                                */
/*----------------------------------------------------------------------*/

/* The OUTPUT_ARCH command specifies the machine architecture where the
   argument is one of the names used in the BFD library. More
   specifically one of the entires in bfd/cpu-mips.c */
OUTPUT_ARCH( "riscv" )

/* The ENTRY command specifies the entry function in the ELF header.
   Does not necessarily mean that this is the first code to be executed
   upon reset (depends on the SECTIONS configured below). */
ENTRY(_start)

/* Memory Spaces Definitions */
/* On the ZYBO boards and the SystemC simulation the memory for code, data, stack
   and heap is all the same, but they are defined seperately here to enable some 
   checks (e.g. read only) and the relocation of the data to the ram memory */
MEMORY
{
    code (rx)  : ORIGIN = 0x10000000, LENGTH = 0x01000000 /* DDR3, 16MB */
    ram  (rwx) : ORIGIN = 0x11000000, LENGTH = 0x0F000000 /* DDR3, 240B */
}

/* This constant is used to check that at least STACK_SIZE of RAM
  is available for stack. If not, linker will issue an error. */
STACK_SIZE = 0x00800000; /* 8MB */

/* This constant is used to check that at least HEAP_SIZE of RAM
   is available for heap. If not, linker will issue an error. */
HEAP_SIZE = 0x00800000; /* 8MB */


/*----------------------------------------------------------------------*/
/* Sections                                                             */
/*----------------------------------------------------------------------*/

SECTIONS
{

  /* text section - Program code and read only data section */
  .text : 
  {
    /* Startup and initialization code */
    . = ALIGN(0x4);
    *(.text.init)
    /* libc/newlib stuff - aligned to 0x100 */
    . = ALIGN(0x100);
    PROVIDE_HIDDEN (__preinit_array_start = .);
    KEEP (*(.preinit_array*))
    PROVIDE_HIDDEN (__preinit_array_end = .);
    PROVIDE_HIDDEN (__init_array_start = .);
    KEEP (*(SORT(.init_array.*)))
    KEEP (*(.init_array ))
    PROVIDE_HIDDEN (__init_array_end = .);
    PROVIDE_HIDDEN (__fini_array_start = .);
    KEEP (*(SORT(.fini_array.*)))
    KEEP (*(.fini_array*))
    PROVIDE_HIDDEN (__fini_array_end = .);
    /* Special tohost/fromhost section for UART communication 
       should land on 0x10000300. This may change in the future. */
    . = ALIGN(0x100);
    *(.tohost)
    /* The rest of the program code */
    . = ALIGN(0x100);
    *(.text)
    . = ALIGN(0x4);
    *(.text.*)
    *(.gnu.linkonce.t.*)
    /* Read only data */
    *(.rodata*)
    *(.srodata*) 
    /* C++ exception stuff */
    *(.eh_frame_hdr) 
    *(.eh_frame)
  } >code
  
  /* data section - initialized data */
  /*  Starts of in the code section at address __src_data_start and needs to be 
      relocated to the start of the ram (__data_start) by the startup code */
  __src_data_start = .;
 .data : 
  {
      . = ALIGN(4);
      /* Start address of .data label in the ram memory (VMA) */
      __data_start = .; 
      /* global pointer points 2KB into the data section to maximize its usage 
         (negative and positive offset in lw/sw instructions) */
      __global_pointer$ = . + 0x800; 
      *(.data*)
      *(.sdata*)
      *(.tdata.begin)
      *(.tdata)
      *(.tdata.end)
      . = ALIGN(4);
      /* End address of .data label in the ram memory (VMA) */
      __data_end = .;
  } >ram AT >code
  
  /* .bss section - uninitialized data */
  /*  Uninitialized data segtion that gets zeroed out by the startup code. 
      NOLOAD means that it does not actually take any space in the ELF/Binary 
      and will not be transfered to the hardware during load. */ 
  .bss (NOLOAD):
  {
    . = ALIGN(4);
    /* Start of .bss label in the ram memory (for startup) */
    __bss_start = .;                 
    *(.sbss .sbss.* .gnu.linkonce.sb.*)
    *(.scommon)
    *(.bss)
    *(.bss.*)
    *(COMMON)
    . = ALIGN(4);
    /* End of .bss label in the ram memory (for startup) */
    __bss_end = .;                 /* end of .bss label (for startup) */
  } >ram

  /* .pntdata section - ParaNut thread data (used for starting threads) */
  /*  Could be part of .bss but we left it seperate for now. 
  .pntdata (NOLOAD): {
    . = ALIGN(0x4);
    *(.pntdata)
  } >ram
 
  /* End of initialized/uninitalized data segement in the ram memory.
     This marks the start of the heap of the first core in a ParaNut */
  . = ALIGN(0x4);
  _end = .;
  
   /* ._usr_stack_heap section - User stack and heap */
   /* This is just to check that there is enough ram left for the user stack
      and heap of ONE core. It should generate an error if it's full. */
   /* _stack_start is used in syscalls.c to ensure no more heap is used than expected */
  ._usr_stack_heap (NOLOAD): {
    . = ALIGN(4);
    . = . + HEAP_SIZE ;
    _stack_start = .;
    . = . + STACK_SIZE ;
    /* Emulate the behaviour of the startup script */
    . = ALIGN(16);
    __freertos_irq_stack_top = .;
  } >ram
  
}

/* size helpers */ 
__text_size = SIZEOF(.text);
__data_size = SIZEOF(.data);
__bss_size = SIZEOF(.bss);
__pntdata_size = SIZEOF(.pntdata);
__usr_stack_heap_size = SIZEOF(._usr_stack_heap);
