/*************************************************************************

  This file is part of the ParaNut project.

  Copyright (C) 2021 Mark Endres <mark.endres@hs-augsburg.de>
      Hochschule Augsburg, University of Applied Sciences

  Description:
     Demo application showcasing the CoPU Linked Mode (Mode 1).
     This application requires the libparanut.  

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

#include <stdio.h> 
#include <stdint.h>
#include <time.h>
#include <encoding.h>
#include <string.h>
#include <stdlib.h>
#include "paranut.h"

#define MTIE (1UL << 7)
#define MTIP (1UL << 7) 

#define MTIME_BASE_ADDR     0x80000000
#define MTIME_ADDR          MTIME_BASE_ADDR + 0
#define MTIMEH_ADDR         MTIME_BASE_ADDR + 4
#define MTIMECMP_ADDR       MTIME_BASE_ADDR + 8
#define MTIMECMPH_ADDR      MTIME_BASE_ADDR + 12

#define MTIMER_IR_INTERVAL_MS    5

static uint8_t bExit = 0;
static uint32_t ecall_count = 0;
static uint32_t timer_ir_count = 0;

// helpers --------------------------------------------------------------------
void write_address(uint32_t address, uint32_t val) {
    uint32_t volatile * const addr = (uint32_t *) address; 
    *addr = val;   
}

uint32_t read_address(uint32_t address) {
    uint32_t volatile *addr = (uint32_t*)address;
    return *addr; 
}

/*void print_mcause() {
    uint32_t mcause_val = read_csr(mcause);
    printf("register mcause value: %x is ", mcause_val);
    if ((mcause_val & (1ULL << 31)) && (mcause_val & 0x7)) {
        printf("correct\n");
    }
    else {
        printf("not correct\n");
        bExit = 1;
    }
}*/

void do_ecall() {
    asm volatile ("ecall\n");
}

uint8_t get_mip_MTIP() {
    uint32_t mip_val = read_csr(mip);   
    return (mip_val & MTIP);
}


__attribute__ ((interrupt ("machine"))) void trap_handler() {
    uint32_t mcause_val = read_csr(mcause);
    printf("mcause value: %x\n", mcause_val);
    
    switch (mcause_val) {
    case 11:
        // environment call
        printf("Entered environment exception handler:\n");
        /*if (ecall_count > 0) {
            printf("ERROR: multiple calls of environment exception handler:\n\n");
            bExit = 1;
            break;
        }*/
        //ecall_count++;
        
        printf("Waiting for asynchronous machine timer interrupt ...\n");
        while (!get_mip_MTIP());
        printf("Now machine timer interrupt is pending!\n");
        
        uint32_t mepc_val = read_csr(mepc);
        mepc_val += 4;
        write_csr(mepc, mepc_val);
        printf("Leaving environment exception handler!\n\n");
        break;
    case 0x80000007:
        // machine timer interrupt
        printf("Entered machine timer interrupt handler:\n");
        timer_ir_count++; // @TODO: error bringt fehlverhalten

        // MTIP is not guaranteed to update immediately, so give it enough time to get set
        uint8_t MTIP_state = 0;
        for (int i = 0; i < 10; i++) {
            MTIP_state |= get_mip_MTIP();
        }

        if (MTIP_state) {
            printf("MTIP Bit in mip register set!\n");

            uint32_t mtime_val = read_address(MTIME_ADDR); 
            uint32_t mtimecmp_val = read_address(MTIMECMP_ADDR);
            printf("mtime value: %d\n", mtime_val);
            printf("mtimecmp value: %d\n", mtimecmp_val);
            
            printf("reloading machine timer\n\n");
            uint32_t new_mtimecmp_val = mtime_val+ 300;
            write_address(MTIMECMP_ADDR, new_mtimecmp_val);
        }
        else {
            printf("ERROR: MTIP Bit in mip register not set!\n\n");  
            uint32_t mtime_val = read_address(MTIME_ADDR); 
            uint32_t mtimecmp_val = read_address(MTIMECMP_ADDR);
            printf("mtime value: %d\n", mtime_val);
            printf("mtimecmp value: %d\n", mtimecmp_val);
            bExit = 1;
        }
        break;
    default:
        // something went wrong
        printf("ERROR: unexpected value of mcause: %x\n\n", mcause_val);
        bExit = 1;
        break;
    }     

    //asm volatile ("mret \n");
}

//-----------------------------------------------------------------------------
int main() {
    printf("\n\nstarting ParaNut mtimer test ...\n\n");
    // !!! mtime timebase is 1ms !!!

    // set mtimecmp to 2ms
    write_address(MTIMECMP_ADDR, 2);

    // set interrupt handler
    write_csr(mtvec, trap_handler);

    // enable mtime interrupts
    write_csr(mie, MTIE);

    // enable global interrupts
    pn_interrupt_enable();      // mstatus bit 3 - MIE 

    if (get_mip_MTIP()) {
        printf("ERROR: MTIP Bit in mip register already set!\n\n");
        return 0;
    }

    do_ecall();

    printf("Returned from environment call!\n\n");

    printf("Monitoring mtimer...\n\n");

    //while (read_address(MTIME_ADDR) < 12); do_ecall(); // causes problem because mcause is 0 then

    while (!bExit) {
        uint32_t mtime_val = read_address(MTIME_ADDR);
        uint32_t mtimecmp_val = read_address(MTIMECMP_ADDR);

        // wait for timer interrupts and monitor...
        if (mtime_val > mtimecmp_val) {
            printf("ERROR: mtime exceeded mtimecmp!\n");    // @TODO: sometimes mtime_val is invalid!
            printf("mtime value: %d\n", mtime_val);
            printf("mtimecmp value: %d\n\n", mtimecmp_val);
            
            //bExit = 1;
        }
        // periodically test the exception, which will wait for async timer IR
        else if (mtime_val % 10 == 0) {
            do_ecall();
        }
    }

    printf("End of ParaNut mtime test\n----------------------------------\n\n");
    return 0;
}
