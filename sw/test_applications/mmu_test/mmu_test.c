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

/*
 * help on
 *
 * - linker script: https://home.cs.colorado.edu/~main/cs1300/doc/gnu/ld_3.html#SEC40
 *
 * - special gcc attributes: https://gcc.gnu.org/onlinedocs/gcc-3.2/gcc/Function-Attributes.html#Function%20Attributes
 *
 * -
 *
 */

/*
 * Todos:
 *
 * - new satp CSR was introduced which requires testing
 *
 * - satp points to page table pointer and contains enable bit
 *
 *   - create a page table with two memory regions and make satp point to it
 *
 *  Tasks:
 *
 *  - Reserve memory region for page table (linker script)
 *
 *  - Reserve memory region for second level page table
 *
 *  - Reserve memory region for another second level page table
 *
 *  - Create page frames (normal pages and superpages) and 
 *
 *   - do calculations in M-mode with paging disabled
 *   - do calculations in M-mode with paging enabled
 *
 *   - do calculations in S-mode with paging enabled
 *   - do calculations in S-mode with paging disabled
 *
 *   - do calculations in U-mode with paging enabled
 *   - do calculations in U-mode with paging disabled
 *
 *    - test permission bits: r,w,x,u
 *
 *    - test u-bit in M-,S- and U-mode with SUM disabled and enabled
 *   - should work 
 *
 * - access to .data must work as before, i.e. translates any address to itself
 * 
 *  this little tool swaps two functions and two memory areas by translating 
 *  using the physical address of the first as the virtual address of the other
 * */


// enable/disable debugging information by setting to 0/1
#define DEBUG_LEVEL 1

#define DEBUG(...)           \
    if (DEBUG_LEVEL)         \
    {                        \
        printf(__VA_ARGS__); \
    }

#define INC_TEST_CNTR() test_counter++; DEBUG("Test #%d\n", test_counter);

#include <stdio.h>
#include <stdint.h>
#include <time.h>
#include <encoding.h>
#include <stdlib.h>
#include <stdbool.h>

#include <paranut.h>
#include "mmu_test.h"

typedef struct {
    uint32_t mtime;
    uint32_t mtimeh;
    uint32_t mtimecmp;
    uint32_t mtimecmph;
} mtimer_t;

volatile bool enabled_modes[MAX_PRIV_MODES] = {0};
volatile int current_mode = 0;
volatile bool completed_test_series = false;

volatile unsigned int test_counter = 0;
volatile mtimer_t * const mtimer = (mtimer_t *) 0x20000000;
volatile int interrupt_enable = 0;
volatile bool testing_interrupts = 0;
volatile bool set_mideleg = 0;
volatile bool set_mie = 0;
volatile bool clear_mie = 0;
volatile bool is_TSR_set = 0;
volatile bool is_paging = 0;
volatile bool received_interrupt = 0;
volatile bool paging_success = true;

volatile pte_t root_page_table[1024] __attribute__ ((aligned (0x1000)));
volatile pte_t leaf_page_table0[1024] __attribute__ ((aligned (0x1000)));
volatile pte_t leaf_page_table1[1024] __attribute__ ((aligned (0x1000)));

volatile uint32_t page_frame0[1024] __attribute__ ((aligned (0x1000)));

#define VIRTUAL_ADDRESS0 0x42344000
#define VIRTUAL_ADDRESS1 0x87354000
#define STACK_ADDRESS 0x12000000

int testcase_results[MAX_PRIV_MODES][TEST_CASES] = {0};

void prepare_page_tables() {
    DEBUG("\nPreparing page tables\n\n");

    // invalidate any entry first; writing 0 is propably as fast as any other option
    for (int i = 0; i < 1024; i++) {
        root_page_table[i] = 0;
        leaf_page_table0[i] = 0;
        leaf_page_table1[i] = 0;
    }

    /* .data and .text must be translated to itself.
     * Using a megapage (size 4^22) should do the trick.
     */
    unsigned int index = VA_VPN1((uint32_t) &prepare_page_tables);

    root_page_table[index] = 1; // set V bit; everything else is set to 0 first
    root_page_table[index] |= 1 << PTE_R_OFFSET;
    root_page_table[index] |= 1 << PTE_W_OFFSET;
    root_page_table[index] |= 1 << PTE_X_OFFSET;
    root_page_table[index] |= 1 << PTE_U_OFFSET;
    root_page_table[index] |= 1 << PTE_A_OFFSET;
    root_page_table[index] |= 1 << PTE_D_OFFSET;
    root_page_table[index] |= VA_VPN1((uint32_t) &prepare_page_tables) << PPN1_OFFSET;

    index = VA_VPN1((uint32_t) &paging_success);

    root_page_table[index] = 1; // set V bit; everything else is set to 0 first
    root_page_table[index] |= 1 << PTE_R_OFFSET;
    root_page_table[index] |= 1 << PTE_W_OFFSET;
    root_page_table[index] |= 1 << PTE_X_OFFSET;
    root_page_table[index] |= 1 << PTE_U_OFFSET;
    root_page_table[index] |= 1 << PTE_A_OFFSET;
    root_page_table[index] |= 1 << PTE_D_OFFSET;
    root_page_table[index] |= VA_VPN1((uint32_t) &paging_success) << PPN1_OFFSET;

    /* Stack section must be translated to itself
     * Using a megapage (size 4^22) should do the trick.
     */
    index = VA_VPN1((uint32_t) &index);
    root_page_table[index] = 1; // set V bit; everything else is set to 0 first
    root_page_table[index] |= 1 << PTE_R_OFFSET;
    root_page_table[index] |= 1 << PTE_W_OFFSET;
    root_page_table[index] |= 1 << PTE_X_OFFSET;
    root_page_table[index] |= 1 << PTE_U_OFFSET;
    root_page_table[index] |= 1 << PTE_A_OFFSET;
    root_page_table[index] |= 1 << PTE_D_OFFSET;
    root_page_table[index] |= VA_VPN1((uint32_t) &index) << PPN1_OFFSET;
    DEBUG("address: 0x%x\n", &root_page_table[index]);

    // point to a leaf page table
    index = VA_VPN1(VIRTUAL_ADDRESS0);
    root_page_table[index] = 1; // set V bit; everything else is set to 0 first
    root_page_table[index] |= 1 << PTE_U_OFFSET;
    root_page_table[index] |= 1 << PTE_A_OFFSET;
    root_page_table[index] |= 1 << PTE_D_OFFSET;
    root_page_table[index] |= VA_VPN((uint32_t) leaf_page_table0) << PPN0_OFFSET;
    DEBUG("At address 0x%x: 0x%x\n", &root_page_table[index], root_page_table[index]);

    // fill leaf page table
    index = VA_VPN0(VIRTUAL_ADDRESS0);
    leaf_page_table0[index] = 1;
    leaf_page_table0[index] |= 1 << PTE_R_OFFSET;
    leaf_page_table0[index] |= 1 << PTE_W_OFFSET;
    leaf_page_table0[index] |= 1 << PTE_X_OFFSET;
    leaf_page_table0[index] |= 1 << PTE_U_OFFSET;
    leaf_page_table0[index] |= 1 << PTE_A_OFFSET;
    leaf_page_table0[index] |= 1 << PTE_D_OFFSET;
    leaf_page_table0[index] |= VA_VPN((uint32_t) page_frame0) << PPN0_OFFSET;
    DEBUG("At address 0x%x: 0x%x\n", &leaf_page_table0[index], leaf_page_table0[index]);
    // point to a leaf page table
    index = VA_VPN1(VIRTUAL_ADDRESS1);
    root_page_table[index] = 1; // set V bit; everything else is set to 0 first
    root_page_table[index] |= 1 << PTE_U_OFFSET;
    root_page_table[index] |= 1 << PTE_A_OFFSET;
    root_page_table[index] |= 1 << PTE_D_OFFSET;
    root_page_table[index] |= VA_VPN((uint32_t) leaf_page_table1) << PPN0_OFFSET
    ;
    DEBUG("At address 0x%x: 0x%x\n", &root_page_table[index], root_page_table[index]);

    // fill leaf page table
    index = VA_VPN0(VIRTUAL_ADDRESS1);
    leaf_page_table1[index] = 1;
    leaf_page_table1[index] |= 1 << PTE_R_OFFSET;
    leaf_page_table1[index] |= 1 << PTE_W_OFFSET;
    leaf_page_table1[index] |= 1 << PTE_X_OFFSET;
    leaf_page_table1[index] |= 1 << PTE_U_OFFSET;
    leaf_page_table1[index] |= 1 << PTE_A_OFFSET;
    leaf_page_table1[index] |= 1 << PTE_D_OFFSET;
    leaf_page_table1[index] |= VA_VPN((uint32_t) page_frame0) << PPN0_OFFSET;
    DEBUG("At address 0x%x: 0x%x\n", &leaf_page_table1[index], leaf_page_table1[index]);
}

volatile void paging_example_code() {
    uint32_t *pt0 = (uint32_t *) VIRTUAL_ADDRESS0;
    uint32_t *pt1 = (uint32_t *) VIRTUAL_ADDRESS1;

    for (int i = 0; i < 1024; i++) {
        *pt0 = i;
        
        paging_success &= *pt0 == *pt1;

        pt0++;
        pt1++;
    }
}

void test_paging() {
    uint32_t tmp;

    is_paging = true;

    // enable paging
    write_csr(satp, (1 << 31) | (((uint32_t) root_page_table) >> 12));
    clear_csr(mstatus, MSTATUS_MPP);

    
    DEBUG("\nTesting paging\n\n");

    asm volatile(
    "auipc %0, 0\n"
    "addi %0, %0, 20\n"
    "csrw mepc, %0\n"
    : "=&r"(tmp));
    // pn_cache_flush_all();
    asm volatile("mret");
    
    paging_example_code();
    asm volatile("ecall");

    if (paging_success) {
        printf("\nLooking good! Everything is similar.\n");
    } else {
        printf("\nOh no! It's different!\n");
    }

    DEBUG("\nTesting done\n\n");

    is_paging = false;
    write_csr(satp, 0);
}

void handle_illegal_instruction(unsigned int cause, unsigned int epc, unsigned int mtval)
{
    testcase_results[current_mode][test_counter] = cause;

    DEBUG("Illegal instruction: %d\n", cause);
    write_csr(mepc, epc + 4);
}


void handle_page_fault(unsigned int cause, unsigned int epc, unsigned int mtval)
{
    testcase_results[current_mode][test_counter] = cause;

    DEBUG("Page fault cause: 0x%x, PC: 0x%x, mtval: 0x%x\n\n", cause, epc, mtval);
}

void volatile handle_ecall(unsigned int cause, unsigned int epc, unsigned int mtval)
{
    uint32_t mstatus_reg;

    DEBUG("Ecall!\n");

    if (is_paging) {
        DEBUG("Disable paging...\n");

        uint32_t mstatus_reg = read_csr(mstatus);
        mstatus_reg &= 0x11 << MSTATUS_MPP_OFFSET;
        write_csr(mstatus, mstatus_reg);

        write_csr(satp, 0);
        
        mstatus_reg = read_csr(mstatus);
        DEBUG("Reset current mode...\n");
        mstatus_reg |= MSTATUS_MPP;
        write_csr(mstatus, mstatus_reg);

        write_csr(mepc, epc + 4);
    } else if (completed_test_series) {
        DEBUG("Return to M-Mode...\n");
        // return to M-Mode
        testcase_results[current_mode][test_counter] = cause;
        mstatus_reg = read_csr(mstatus);
        mstatus_reg |= MSTATUS_MPP;
        write_csr(mstatus, mstatus_reg);
        write_csr(mepc, epc + 4);
    } else if (set_mie) {
        set_csr(mie, MIE_MTIE);
        write_csr(mepc, epc + 4);
    } else if (clear_mie) {
        clear_csr(mie, MIE_MTIE);
        write_csr(mepc, epc + 4);
    } else if (set_mideleg) {
        if (enabled_modes[Supervisor])
        {
            DEBUG("Setting mdeleg...\n");
            unsigned int mdeleg = read_csr(mideleg);
            mdeleg ^= 1 << 7;
            write_csr(mideleg, mdeleg);
        }
        write_csr(mepc, epc + 4);
    } else if (testing_interrupts) {
        mstatus_reg = read_csr(mstatus);
        DEBUG("Resetting current mode...\n");
        mstatus_reg &= ~MSTATUS_TSR;
        mstatus_reg &= ~MSTATUS_MPP;
        mstatus_reg |= current_mode << MSTATUS_MPP_OFFSET;
        DEBUG("Setting MIE and SIE...\n");
        mstatus_reg &= ~MSTATUS_MPIE & ~MSTATUS_SIE;
        mstatus_reg |= (interrupt_enable & 0x1) << 1; // set SIE
        mstatus_reg |= (interrupt_enable & 0x2) << 6; // set MPIE, which sets MIE on MRET
        write_csr(mstatus, mstatus_reg);
        DEBUG("Jumping to 0x%x...\n", epc + 4);
        write_csr(mepc, epc + 4);
    }
    else {
        int tsr;

        DEBUG("Resetting previous mode...\n");
        // return into mode that we are currently testing
        mstatus_reg = read_csr(mstatus);

        mstatus_reg &= ~MSTATUS_MPP;
        mstatus_reg |= current_mode << MSTATUS_MPP_OFFSET;

        if (mstatus_reg & MSTATUS_TSR) {
            is_TSR_set = false;
            mstatus_reg &= ~MSTATUS_TSR;
        } else {
            is_TSR_set = true;
            mstatus_reg |= MSTATUS_TSR;
        }

        write_csr(mstatus, mstatus_reg);
        // continue after the previous test case
        DEBUG("Jumping back to 0x%x...\n", epc + 4);

        write_csr(mepc, epc + 4);
    }
}

void handle_interrupt(unsigned int cause, unsigned int epc, unsigned int mtval) {
    DEBUG("Handling interrupt...\n");
    mtimer->mtimecmp = 0xffffffff;
    received_interrupt = 1;
    testcase_results[current_mode][test_counter] = cause;
    DEBUG("Writing cause 0x%x\n", cause);
}

__attribute__((interrupt("supervisor"))) void handle_supervisor_interrupt()
{
    DEBUG("Delegating interrupt...\n");
    unsigned int epc = read_csr(0x141);   // sepc
    unsigned int cause = read_csr(0x142); // scause
    unsigned int tval = read_csr(0x143);  // stval
    handle_interrupt(cause, epc, tval);
}

volatile void determine_priv_modes()
{
    uint32_t mstatus_reg;

    DEBUG("\nEnabled privilege modes are");

    for (current_mode = 0; current_mode < MAX_PRIV_MODES; current_mode++)
    {
        clear_csr(mstatus, MSTATUS_MPP);
        set_csr(mstatus, current_mode << MSTATUS_MPP_OFFSET);

        mstatus_reg = read_csr(mstatus);
        if ((mstatus_reg & MSTATUS_MPP) == (current_mode << MSTATUS_MPP_OFFSET))
        {
            enabled_modes[current_mode] = true;
            DEBUG(" %i", current_mode);
        }
    }

    DEBUG(". If this configuration is wrong: TEST FAILED!\n");
}

void adjust_expected_results()
{
    if (!enabled_modes[Supervisor])
    {
        // Only when supervisor is not implemented, SRET and S-mode CSRs are not implemented
        // and therefore raise an Illegal Instruction Exception
        expected_results[Machine][13] = CAUSE_ILLEGAL_INSTRUCTION;
        expected_results[Machine][14] = CAUSE_ILLEGAL_INSTRUCTION;
        expected_results[Machine][6] = CAUSE_ILLEGAL_INSTRUCTION;
        expected_results[Machine][7] = CAUSE_ILLEGAL_INSTRUCTION;

        expected_results[User][20] = 16;
        expected_results[User][21] = 16;
        expected_results[User][22] = 16;
        expected_results[User][23] = 16;
    }
    else
    {
        write_csr(0x105, handle_supervisor_interrupt); // Enable Interrupt handler for supervisor
    }
}

void access_CSRs()
{
    DEBUG("Reading non-existent CSR...\n");
    read_csr(0xccc);

    INC_TEST_CNTR();
    DEBUG("Writing non-existent CSR...\n");
    write_csr(0xccc, 1);

    INC_TEST_CNTR();
    DEBUG("Reading M-Mode RO CSR...\n");
    read_csr(0xF14);

    INC_TEST_CNTR();
    DEBUG("Writing M-Mode RO CSR...\n");
    write_csr(0xF14, 1);

    INC_TEST_CNTR();
    DEBUG("Reading M-Mode RW CSR...\n");
    read_csr(0x340);

    INC_TEST_CNTR();
    DEBUG("Writing M-Mode RW CSR...\n");
    write_csr(0x340, 1);

    INC_TEST_CNTR();
    DEBUG("Reading S-Mode RW CSR...\n");
    read_csr(0x140);

    INC_TEST_CNTR();
    DEBUG("Writing S-Mode RW CSR...\n");
    write_csr(0x140, 1);

    INC_TEST_CNTR();
    DEBUG("Reading U-Mode RO CSR...\n");
    read_csr(0xCD4);

    INC_TEST_CNTR();
    DEBUG("Writing U-Mode RO CSR...\n");
    write_csr(0xCD4, 1);

    INC_TEST_CNTR();
    DEBUG("Reading U-Mode RW CSR...\n");
    read_csr(0x8C3);

    INC_TEST_CNTR();
    DEBUG("Writing U-Mode RW CSR...\n");
    write_csr(0x8C3, 1);
}

void execute_xRET()
{
    int tmp;

    INC_TEST_CNTR();
    DEBUG("Executing mret...\n");
    if (current_mode == Machine)
    {
        // only preparing in M-Mode, since it will fail otherwise
        set_csr(mstatus, MSTATUS_MPP);
        asm volatile(
            "auipc %0, 0\n"
            "addi %0, %0, 20\n"
            "csrw mepc, %0\n"
            : "=&r"(tmp));
    }
    mret();

    for (volatile int i = 0; i < 2; i++)
    {
        // run code two times: One time without TSR bit set, the other one with.
        // The TSR bit is set in the Ecall handler
        INC_TEST_CNTR();
        DEBUG("Executing SRET (TSR=%i)...\n", is_TSR_set);

        if (current_mode >= Supervisor && enabled_modes[Supervisor])
        {
            // only preparing in S-Mode, since it will fail otherwise
            set_csr(sstatus, SSTATUS_SPP);

            asm volatile(
                "auipc %0, 0\n"
                "addi %0, %0, 20\n"
                "csrw sepc, %0\n"
                : "=&r"(tmp));
        }
        sret();

        // Run ecall to switch back to mode that was set before SRET execution
        // also, set TSR bit
        ecall();
    }

    INC_TEST_CNTR();
    DEBUG("Executing URET...\n");
    // not setting up anything here, 'cause it should fail anyway
    uret();
}

void set_privilege_mode(int mode)
{
    int tmp;

    clear_csr(mstatus, MSTATUS_MPP | MSTATUS_TSR);
    set_csr(mstatus, mode << MSTATUS_MPP_OFFSET);
    asm volatile(
        "auipc %0, 0\n"
        "addi %0, %0, 20\n"
        "csrw mepc, %0\n"
        : "=&r"(tmp));
    mret();
}

void validate_interrupt_delegation()
{
    DEBUG("Testing MIE, SIE and interrupt delegation ...\n");
    testing_interrupts = true;

    for (int i = 0; i < 2; i++)
    {
        for (interrupt_enable = 0; interrupt_enable < 4; interrupt_enable++)
        {
            volatile unsigned int dummy_value = 0;
            
            received_interrupt = 0;
            
            INC_TEST_CNTR();
            // Run ecall to set each option of MIE/SIE bits as they are represented in interrupt_enable
            // Afterwards, switch back to curent mode
            ecall();

            DEBUG("Set mtimer\n");
            mtimer->mtime = 0;
            mtimer->mtimeh = 0;
            mtimer->mtimecmp = 2;

            // enable mtimer interrupt, which is only allowed in M-Mode
            set_mie = true;
            ecall();
            set_mie = false;
            
            // do busy waiting 
            while (!received_interrupt && ((mtimer->mtime < mtimer->mtimecmp) && !(mtimer->mtimeh))) {asm volatile ("nop\n");}
            
            // disable mtimer interrupt, which is only allowed in M-Mode
            clear_mie = true;
            ecall();
            clear_mie = false;
        }
        set_mideleg = true;
        ecall();
        set_mideleg = false;
    }
    testing_interrupts = false;
}

void evaluate_results()
{
    bool test_failed = false;

    for (current_mode = MAX_PRIV_MODES - 1; current_mode >= 0; current_mode--)
    {
        if (enabled_modes[current_mode])
        {
            for (int i = 0; i < TEST_CASES; i++)
            {
                if (testcase_results[current_mode][i] != expected_results[current_mode][i])
                {
                    printf("\n\nResults for testcase %i in mode %i differ: Expected 0x%x, but got 0x%x\n", i, current_mode, expected_results[current_mode][i], testcase_results[current_mode][i]);
                    test_failed = true;
                }
            }
        }
    }

    if (test_failed)
    {
        printf("\n\nTest FAILED!\n\n");
    }
    else
    {
        printf("\n\nTest SUCCEEDED!\n\n");
    }
}

int main()
{
    int tmp;
    int n, id;
    uint32_t cpus = read_csr(0xCD0);

    pn_cache_init();
    pn_cache_enable();
    pn_exception_init();
    pn_exception_set_handler(&handle_illegal_instruction, CAUSE_ILLEGAL_INSTRUCTION);
    pn_exception_set_handler(&handle_ecall, CAUSE_USER_ECALL);
    pn_exception_set_handler(&handle_ecall, CAUSE_SUPERVISOR_ECALL);
    pn_exception_set_handler(&handle_ecall, CAUSE_MACHINE_ECALL);
    pn_exception_set_handler(&handle_page_fault, CAUSE_FETCH_PAGE_FAULT);
    pn_exception_set_handler(&handle_page_fault, CAUSE_LOAD_PAGE_FAULT);
    pn_exception_set_handler(&handle_page_fault, CAUSE_STORE_PAGE_FAULT);
    pn_exception_set_handler(&handle_interrupt, (1 << 31) | IRQ_M_TIMER);

    printf("\n--------------- Welcome to the ParaNut MMU test ---------------\n\n");


    determine_priv_modes();
    adjust_expected_results();


    // run tests
    for (current_mode = MAX_PRIV_MODES - 1; current_mode >= 0; current_mode--) {
        is_TSR_set = false;
        testing_interrupts = false;
        set_mideleg = false;

        if (enabled_modes[Supervisor]) {
            // reset interrupt delegation register
            write_csr(mideleg, 0);
        }

        
        if (enabled_modes[current_mode]) {
            printf("\nTesting mode %i\n", current_mode);

            while (true) {
                if (enabled_modes[Supervisor] && current_mode == Machine ) {
                    prepare_page_tables();
                    test_paging();
                }
            }
            
            test_counter = 0;
            completed_test_series = 0;

            // Set priv mode
            set_privilege_mode(current_mode);

            // test different CSR access cases
            access_CSRs();
            execute_xRET();
            validate_interrupt_delegation();

            INC_TEST_CNTR();
            completed_test_series = true;
            DEBUG("Test series done.\n");
            ecall();
        }
        completed_test_series = false;
    }

    evaluate_results();
}
