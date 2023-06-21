/*************************************************************************

  This file is part of the ParaNut project.

  Copyright (C) 2010-2020 Christian H. Meyer <christian.meyer@hs-augsburg.de>
      Hochschule Augsburg, University of Applied Sciences

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


void return_mmode();
void set_umode();
int linked_mode_demo();
int thread_mode_demo();

typedef enum {
  User = 0,
  Supervisor = 1,
  Machine = 3,
} EPrivMode;

#define mret() asm volatile ("mret")
#define sret() asm volatile ("sret")
#define uret() asm volatile ("uret")
#define ecall() asm volatile ("ecall")

#define MSTATUS_MPP_OFFSET 11
#define MSTATUS_SPP_OFFSET 9

#define MAX_PRIV_MODES 4
#define TEST_CASES 25
/*

Paging testing
At first, paging tables are created and afterwards the following tests take place:

    0. A huge address space translates .data, .text and some more to itself
    1. a virtual memory area is filled with data
    2. from another virtual memory, the same physical memory area as above is accessed

    

Privilege mode testing

    0. Read non-existent CSR
    1. Write non-existent CSR

    2. Read M-Mode RO CSR
    3. Write M-Mode RO CSR
    4. Read M-Mode RW CSR
    5. Write M-Mode RW CSR

    6. Read S-Mode RW CSR
    7. Write S-Mode RW CSR

    8.Read U-Mode RO CSR
    9. Write U-Mode RO CSR
    10. Read U-Mode RW CSR
    11. Write U-Mode RW CSR

    12. MRET
    13. SRET (TSR=0)
    14. SRET (TSR=1)
    15. URET

    16. Interrupt (MIE=0, SIE=0)
    17. Interrupt (MIE=0, SIE=1)
    18. Interrupt (MIE=1, SIE=0)
    19. Interrupt (MIE=1, SIE=1)

    20. Delegate interrupt to S-Mode (MIE=0, SIE=0)
    21. Delegate interrupt to S-Mode (MIE=0, SIE=1)
    22. Delegate interrupt to S-Mode (MIE=1, SIE=0)
    23. Delegate interrupt to S-Mode (MIE=1, SIE=1)

    24. ECALL from current Mode



    TODOs:
    - ParaNut Exception

 The raised exceptions are written into the array testcase_results. Afterwards,
 they are compared with the excepted exceptions stored in expected_results.
 Note: A value of zero means there was no exception for this test case. This 
 is used because this test should  never raise an Instruction address misaligned 
 exception and simplifies automatic checks.

 Note: To distinguish in which mode interrupts are handled, S-Mode sets the MSB to 1 in
       the result.

 Also note: Results for M-Mode SRET and S-mode CSR access tests are changed dynamically,
 because they differ depending on the M/S/U configuration, 
 */
int expected_results[MAX_PRIV_MODES][TEST_CASES] = {
    {
        // User mode
        CAUSE_ILLEGAL_INSTRUCTION,
        CAUSE_ILLEGAL_INSTRUCTION,

        CAUSE_ILLEGAL_INSTRUCTION,
        CAUSE_ILLEGAL_INSTRUCTION,
        CAUSE_ILLEGAL_INSTRUCTION,
        CAUSE_ILLEGAL_INSTRUCTION,

        CAUSE_ILLEGAL_INSTRUCTION,
        CAUSE_ILLEGAL_INSTRUCTION,

        0,
        CAUSE_ILLEGAL_INSTRUCTION,
        0,
        0,

        CAUSE_ILLEGAL_INSTRUCTION,
        CAUSE_ILLEGAL_INSTRUCTION,
        CAUSE_ILLEGAL_INSTRUCTION,
        CAUSE_ILLEGAL_INSTRUCTION,

        7,
        7,
        7,
        7,

        (1 << 31) | 7,
        (1 << 31) | 7,
        (1 << 31) | 7,
        (1 << 31) | 7,

        CAUSE_USER_ECALL,
    },
    {
        // Supervisor mode
        CAUSE_ILLEGAL_INSTRUCTION,
        CAUSE_ILLEGAL_INSTRUCTION,

        CAUSE_ILLEGAL_INSTRUCTION,
        CAUSE_ILLEGAL_INSTRUCTION,
        CAUSE_ILLEGAL_INSTRUCTION,
        CAUSE_ILLEGAL_INSTRUCTION,

        0,
        0,

        0,
        CAUSE_ILLEGAL_INSTRUCTION,
        0,
        0,

        CAUSE_ILLEGAL_INSTRUCTION,
        0,
        CAUSE_ILLEGAL_INSTRUCTION,
        CAUSE_ILLEGAL_INSTRUCTION,

        7,
        7,
        7,
        7,

        0,
        (1 << 31) | 7,
        0,
        (1 << 31) | 7,

        CAUSE_SUPERVISOR_ECALL,
    },
    {
        // Hypervisor mode — currently not implemented
        CAUSE_ILLEGAL_INSTRUCTION,
        CAUSE_ILLEGAL_INSTRUCTION,

        CAUSE_ILLEGAL_INSTRUCTION,
        CAUSE_ILLEGAL_INSTRUCTION,
        CAUSE_ILLEGAL_INSTRUCTION,
        CAUSE_ILLEGAL_INSTRUCTION,

        0,
        0,

        0,
        CAUSE_ILLEGAL_INSTRUCTION,
        0,
        0,

        CAUSE_ILLEGAL_INSTRUCTION,
        0,
        0,
        CAUSE_ILLEGAL_INSTRUCTION,

        7,
        7,
        7,
        7,

        7,
        7,
        7,
        7,

        CAUSE_HYPERVISOR_ECALL,
    },
    {
        // Machine mode
        CAUSE_ILLEGAL_INSTRUCTION,
        CAUSE_ILLEGAL_INSTRUCTION,

        0,
        CAUSE_ILLEGAL_INSTRUCTION,
        0,
        0,

        0,
        0,

        0,
        CAUSE_ILLEGAL_INSTRUCTION,
        0,
        0,

        0,
        0,
        0,
        CAUSE_ILLEGAL_INSTRUCTION,

        0,
        0,
        7,
        7,

        0,
        0,
        7,
        7,

        CAUSE_MACHINE_ECALL,
    }};



typedef uint32_t pte_t;

#define PPN1_LENGTH 12
#define PPN0_LENGTH 10
#define RSW_LENGTH 2

#define PPN1_OFFSET 20
#define PPN0_OFFSET 10
#define RSW_OFFSET 8
#define PTE_D_OFFSET 7
#define PTE_A_OFFSET 6
#define PTE_G_OFFSET 5
#define PTE_U_OFFSET 4
#define PTE_X_OFFSET 3
#define PTE_W_OFFSET 2
#define PTE_R_OFFSET 1
#define PTE_V_OFFSET 0

#define VA_VPN1_OFFSET 22
#define VA_VPN0_OFFSET 12

#define VA_VPN_LENGTH 10
#define VA_PAGE_OFFSET_LENGTH 12

#define VA_VPN(va) ((va) >> VA_VPN0_OFFSET)
#define VA_VPN1(va) ((va) >> VA_VPN1_OFFSET)
#define VA_VPN0(va) (((va) >> VA_VPN0_OFFSET) & ((1 << VA_VPN_LENGTH) - 1))
#define VA_PAGE_OFFSET(va) ((va) & ((1 << VA_PAGE_OFFSET_LENGTH) - 1))

#define PTE_PPN1(pte) ((pte) >> PPN1_OFFSET)
#define PTE_PPN0(pte) (((pte) >> PPN0_OFFSET) & ((0x1 << PPN0_LENGTH) - 1))
#define PTE_RSW(pte) (((pte) >> RSW_OFFSET) & ((0x1 << RSW_LENGTH) - 1))
