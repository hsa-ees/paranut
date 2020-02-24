/******************************************************************************
*
* Copyright (C) 2009 - 2014 Xilinx, Inc.  All rights reserved.
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in
* all copies or substantial portions of the Software.
*
* Use of the Software is limited solely to applications:
* (a) running on a Xilinx device, or
* (b) that interact with a Xilinx device through a bus or interconnect.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
* XILINX  BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
* WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF
* OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
* SOFTWARE.
*
* Except as contained in this notice, the name of the Xilinx shall not be used
* in advertising or otherwise to promote the sale, use or other dealings in
* this Software without prior written authorization from Xilinx.
*
******************************************************************************/
/**************************************************************************
 *
 *  This file is part of the ParaNut project.
 *
 *  Copyright (C) 2018-2020 Alexander Bahle <alexander.bahle@hs-augsburg.de>
 *                          Gundolf Kiefer <gundolf.kiefer@hs-augsburg.de>
 *      Efficient Embedded Systems Group
 *      Hochschule Augsburg, University of Applied Sciences
 *
 *  Description:
 *   	Firmware main program for interacting with a ParaNut 
 *      through the paranut_flash python tool
 *
 **************************************************************************/

#include <stdio.h>
#include "platform.h"
#include "xil_printf.h"
#include "xil_io.h"
#include "xil_cache.h"
#include "sleep.h"
#include "xparameters.h"


/*  Host communication
 *  ==================
 *
 *  Host communication is done by stdin/stdout, which is typically directed to
 *  the PS7 UART (Zynq), configured by bootrom/bsp.
 *
 *  UART parameters: 115200, 8 bits, no parity
 *
 *  Connect manually:   > picocom -c -b 115200 /dev/ttyZYBO
 *
 *
 *
 *  Command protocol
 *  ================
 *
 *  Unless specified otherwise, all numbers are transferred as hexadecimal numbers
 *  without any prefix (like "0x"). Trailing '0's can (in CMDs) and may be (in ANSs)
 *  omitted.
 *
 *
 *  General answers
 *  ---------------
 *
 *  The following answers can occur as a response to any command:
 *
 *    ANS: "\x06 ?e <type> <message>"
 *
 *      An error has occured. <message> is a clear-text string starting after the second
 *      whitespace and continuing until the end of the message. It may be presented to
 *      the user. <type> may represent the error class / necessary consequences.
 *      It is presently unused (always 0).
 *
 *    ANS: "<...>"
 *
 *      Lines not starting with '\x16' (SYN) are stdout/sterr of the ParaNut processor.
 *
 *
 *  Command: Hello
 *  --------------
 *
 *    CMD: "\x16 ?h <version host>"
 *    ANS: "\x06 ?h <version fw>"
 *
 *  Mutual greeting, to be initiated by the host, in order to check the presence
 *  and exchange version information.
 *
 *      <version host>, <version fw> are 32 bit numbers:
 *          31..16: major version
 *          15.. 8: minor version
 *           7.. 0: revision
 *
 *
 *  Command: Exit
 *  -------------
 *
 *    CMD: "\x16 ?x"
 *    ANS: No answer
 *
 *  Shutdown the firmware (optional) - Deactivated for now.
 *
 *
 *  Command: Write block memory
 *  ------------------------
 *
 *    CMD:  "\0x16 ?b <adr> <size> <block_size>"
 *    ANS1: "\0x06 ?b <adr> <size> <block_size>"
 *    ... RAW DATA TRANSFER UNTILL SIZE ...
 *    ANS2: "\0x06 ?b <adr+size> <size> <block_size>"
 *
 *  Write large amount of data to memory (e.g. application code).
 *
 *  The first answer is the acknowledgement and start signal for the host application to start sending
 *  the raw data. If enough bytes were received (= <size>) the firmware sends the second answer to 
 *  complete the block data transfer. 
 *  
 *  NOTE: During the raw data transfer no other command can be received or handled!
 *
 *
 *  Command: Read from memory or I/O
 *  --------------------------------
 *
 *    CMD: "\x16 ?r <adr>"
 *    ANS: "\x06 ?r <adr> <data>"
 *
 *  Read a memory word.
 *
 *
 *  Command: Write to memory or I/O
 *  -------------------------------
 *
 *    CMD: "\x16 ?w <adr> <data>"
 *    ANS: "\x06 ?w <adr> <data>"
 *
 *  Write a memory word.
 *
 *
 *  Command: Set tohost/fromhost adresses
 *  --------------
 *
 *    CMD: "\x16 ?! <tohost-adr> <fromhost-adr>"
 *    ANS: "\x06 ?! <tohost-adr> <fromhost-adr>"    
 *
 *  Set tohost and fromhost adresses if they are different from the default value.
 *
 *
 */


#include <xparameters.h>

#include "platform.h"

#include <stdio.h>


#define VERSION_MAJOR 1
#define VERSION_MINOR 0
#define VERSION_REVISION 0


#define SYN (0x16)
#define ACK (0x06)


/***************************** Helpers ***************************************/


#define BOOL int

#define PARANUT_MEM_ADDR  0x10000000
#define MAX_MEM_SIZE      (256*1024*1024) // 256 MB 

#define UART_BASE		0xE0001000
volatile uint8_t *UART_SR = (uint8_t*)UART_BASE + 0x0000002C;
volatile uint8_t* UART_FIFO  = (uint8_t*)(UART_BASE + 0x00000030);
#define UART_TXFULL 	0x00000010
#define UART_RXEMPTY 	0x00000002

uint32_t TOHOST_ADR  = 0x10000300;
uint32_t FROMHOST_ADR = 0x10000304;



/***************************** Hardware access *******************************/


static inline void mem_write (uint32_t adr, uint32_t data) {
  Xil_Out32(adr, data);
  Xil_DCacheFlushRange(adr, 4*sizeof(char));
}


static inline uint32_t mem_read (uint32_t adr) {
  Xil_DCacheInvalidateRange(adr, 4*sizeof(char));
  uint32_t data = Xil_In32(adr);
  //~ printf ("### Reading reg #0x%02x: r(0x%08x) = 0x%08x\n", reg, XPAR_AXI2ParaNut_0_S00_AXI_BASEADDR + reg, data);
  return data;
}

static inline uint32_t uart_getc () {
  uint32_t data = 0;
  if((*UART_SR & UART_RXEMPTY) != UART_RXEMPTY){
	 data = *UART_FIFO;
  }
  return data;
}

static inline BOOL paranut_fromhost_empty () {
    Xil_DCacheInvalidateRange(FROMHOST_ADR, 4);
    if (Xil_In32(FROMHOST_ADR) == 0) 
        return 1;
    return 0;
}

static inline void paranut_putc (const char c) {
    /* Write char from Host/Proxy to ParaNut (don't check if FROMHOST is empty) */
    Xil_Out8(FROMHOST_ADR, c);
    Xil_Out8(FROMHOST_ADR+3, 1);
    Xil_DCacheFlushRange(FROMHOST_ADR, 4);
}

static inline void paranut_clear_tohost () {
    /* Clear TOHOST so ParaNut can write again */
    Xil_Out8(TOHOST_ADR, 0);
    Xil_DCacheFlushRange(TOHOST_ADR, 1);
}
    

static inline char paranut_getc () {
    /* Read char from ParaNut */
    //~ printf("(FIRMWARE) Read from 0x%08x\n", TOHOST_ADR);
    Xil_DCacheInvalidateRange(TOHOST_ADR, 1);
    return Xil_In8(TOHOST_ADR);
}


/***************************** Helpers ***************************************/


void command_error (const char *msg, const char *cmd) {
  if (cmd) printf ("\x06 ?e 0 %s (CMD='%s')\n", msg, cmd);
  else printf ("\x06 ?e 0 %s\n", msg);
}

// Clear FIFO of UART
void clear_receiver() {
	char c;
	while (((*UART_SR) & UART_RXEMPTY) != UART_RXEMPTY) {
			c = *UART_FIFO;
	}
	Xil_Out32(FROMHOST_ADR, 0);
	Xil_DCacheFlushRange(FROMHOST_ADR, 4*sizeof(char));
	Xil_Out32(TOHOST_ADR, 0);
	Xil_DCacheFlushRange(TOHOST_ADR, 4*sizeof(char));
}


BOOL check_hardware () {
  static int hw_ok = -1;
  uint32_t hw_ver;

  /* Identify hardware if not yet happened ... */
//  if (hw_ok < 0) {
//    hw_ok = 0;
//    if (paranut_read (A2V_IDENTIFICATION_REG) == A2V_IDENTIFICATION_VALUE) {
//      hw_ver = paranut_read (A2V_VERSION_REG);
//      printf ("# Detected hardware: ParaNut_DEBUG v%i.%i.%i.\n",
//              A2V_VERSION_MAJOR(hw_ver), A2V_VERSION_MINOR(hw_ver), A2V_VERSION_REVISION(hw_ver));
//      if (hw_ver >= 0x10000)       // require at least version 1.0.0
        hw_ok = 1;
//    }
//  }

  /* Done ... */
  return hw_ok;   /* this is either 0 or 1 */
}


BOOL hardware_ok () {
  int hw_ok = check_hardware ();
  if (!hw_ok) command_error ("No valid ParaNut hardware found", NULL);
  return hw_ok;
}


/* Queue struct */
#define SIZE 256
struct QUEUE {
    int tail;
    char elems[SIZE];
} q; 

/* Initialize queue */
void qinit () {
    q.tail = 0;
}
 
/* Return 1 if queue is full, otherwise return 0 */
int qfull () {
    return q.tail == SIZE;
}
 
/* Return 1 if the queue is empty, otherwise return 0 */
int qempty () {
    return q.tail == 0;
}

/* Enqueue an element */
void qenqueue (char element) {
    /* Drop inputs if we are full */
    if(!qfull()) { 
        q.elems[q.tail++] = element;
    }
}   
 
/* Dequeue an element */
char qdequeue () {
    char c = -1;
    
    /* Check if queue is empty */
    if (!qempty()) {
        c = q.elems[0];
        q.tail--;
        /* Shift all up one position */
        for(int i = 0; i < q.tail; i++){
            q.elems[i] = q.elems[i+1];
        }
    }
    
    return c;
} 
 
/* Print queue content */
//~ void display()
//~ {
    //~ for(int i = 0; i < q.tail; i++) 
        //~ printf("%c ",q.elems[i]);
    //~ printf("\n");
//~ }

/***************************** Commands **************************************/


void cmd_hello (const char *cmd, const char *args) {
  /* Checks for the host's version and compatibility may be added here later. */
  printf ("\x06 ?h %08x %08x %08x\n", VERSION_MAJOR, VERSION_MINOR, VERSION_REVISION);
}


void cmd_mem_read (const char *cmd, const char *args) {
	uint32_t adr, data;

	/* Sanity... */
	if (!hardware_ok ()) return;

	/* Scan command... */
	if (sscanf (args, "%lx", &adr) != 1) {
        command_error ("Syntax error", cmd);
        return;
	}
    
    /* Check arguments */
	if (adr < PARANUT_MEM_ADDR || adr > PARANUT_MEM_ADDR+MAX_MEM_SIZE) {
        command_error ("Adress error", cmd);
        return;
	}

	data = mem_read(adr);

	/* Send result (UART)... */
    printf ("\x06 ?r %08lx %08lx\n", adr, data);
}


void cmd_mem_write (const char *cmd, const char *args) {
	uint32_t adr, data;

	/* Sanity ... */
	if (!hardware_ok ()) return;

	/* Scan command ... */
	if (sscanf (args, "%lx %lx", &adr, &data) != 2) {
        command_error ("Syntax error", cmd);
        return;
	}
    
    /* Check arguments */
	if (adr < PARANUT_MEM_ADDR || adr > PARANUT_MEM_ADDR+MAX_MEM_SIZE) {
        command_error ("Adress error", cmd);
        return;
	}

	mem_write(adr, data);

    /* Send result (UART)... */
    printf ("\x06 ?w %08lx %08lx\n", adr, data);
}

void cmd_block_write (const char *cmd, const char *args) {
	uint32_t adr, size, chunk_size, cnt = 0;
	char c;

	/* Sanity ... */
	if (!hardware_ok ()) return;

	/* Scan command ... */
	if (sscanf (args, "%lx %lx %lx", &adr, &size, &chunk_size) != 3) {
        command_error ("Syntax error", cmd);
        return;
	}
    
    /* Check arguments */
    //~ printf("(FIRMWARE) Adr: 0x%08x, Size: 0x%08x, End: 0x%08x\n", adr, size, adr+size);
	if (adr < PARANUT_MEM_ADDR || adr > PARANUT_MEM_ADDR+MAX_MEM_SIZE
        || adr+size > PARANUT_MEM_ADDR+MAX_MEM_SIZE) {
        command_error ("Address or range error", cmd);
        return;
	}
    
    /* Send acknowledge for program size first */
	printf("\x06 ?b %08lx %08lx %08lx\n", adr, size, chunk_size);

	/* Start program receiving */    
    for (uint32_t end_adr = adr + size; adr < end_adr; adr++){
        Xil_Out8(adr, getchar());
    }
    
	/* Write from cache to DDR */
	Xil_DCacheFlush();

	/* Print confirmation ... */
	printf("\x06 ?b %08lx %08lx %08lx\n", adr, size, chunk_size);
}

void cmd_set_adr (const char *cmd, const char *args) {
    uint32_t tohost, fromhost;

	/* Sanity ... */
	if (!hardware_ok ()) return;

	/* Scan command ... */
	if (sscanf (args, "%lx %lx", &tohost, &fromhost) != 2) {
        command_error ("Syntax error", cmd);
        return;
	}
    
    /* Check arguments */
	if (tohost < PARANUT_MEM_ADDR || tohost > PARANUT_MEM_ADDR+MAX_MEM_SIZE ||
        fromhost < PARANUT_MEM_ADDR || fromhost > PARANUT_MEM_ADDR+MAX_MEM_SIZE) {
        command_error ("Adress error", cmd);
        return;
	}

	/* Set */
    TOHOST_ADR = tohost;
    FROMHOST_ADR = fromhost;
    
    /* Send result (UART)... */
    printf ("\x06 ?! %08lx %08lx\n", TOHOST_ADR, FROMHOST_ADR);
}


void RunCommand (char *cmd) {
      
    /* Execute the firmware to host app commands... */
    //~ printf("(FIRMWARE) cmd[0]=%c, cmd[1]= %c, cmd[2]=%c\n", cmd[0], cmd[1], cmd[2]);
    if(cmd[0] == ' ' && cmd[1] == '?') {
        switch (cmd[2]) {
          case 'x':
            /* don't quit the software 
             * or ParaNut will hang in UART */
            break;
          case 'h':
            cmd_hello (cmd, cmd+4);
            break;
          case '!':
            cmd_set_adr (cmd, cmd+4);
            break;
          case 'r':
            cmd_mem_read (cmd, cmd+4);
            break;
          case 'w':
            cmd_mem_write (cmd, cmd+4);
            break;
          case 'b':
            cmd_block_write (cmd, cmd+4);
            break;
          default:
            command_error ("Unknown command", cmd);
        }
    }
}


/***************************** Main ******************************************/

int main() {
  char command[256];
  char c;
  BOOL quit;
   int cmd_idx = -1;

  /* Initialize ... */
  init_platform();
  Xil_ICacheEnable();
  Xil_DCacheEnable();
  qinit();

  printf ("(FIRMWARE) ParaNut Proxy firmware v%i.%i.%i ready.\n",
          VERSION_MAJOR, VERSION_MINOR, VERSION_REVISION);
  check_hardware ();

  /* Clear UART and TOHOST/FROMHOST ... */
  clear_receiver();

  /* Main loop ... */
  quit = 0;
  while (!quit) {

    /* Read input... */
    /* Check for UART input (From Host-PC to ARM/ParaNut)*/
    c =  uart_getc();
    if (c != 0) {
        if (c == SYN) /* Command starts? */
            cmd_idx = 0; 
        else {
            if (cmd_idx != -1){
                if (c == '\n'){ /* Command end? */
                    command[cmd_idx] = '\0';
                    //~ printf("(FIRMWARE) Command: %s\n", command+1);
                    RunCommand(command); // TBD
                    cmd_idx = -1;
                } else {
                    command[cmd_idx] = c;
                    cmd_idx ++;
                }
            } else { /* Else just write to ParaNut */
                // TBD: Add check to only write to FROMHOST_ADR/TOHOST_ADR if we know the address is valid
                // TBD: Do we need this?
                qenqueue(c);
            }       
        }
    }
    
    /* Check for ParaNut output (From ParaNut to ARM/Host-PC)*/
    c = paranut_getc ();
    
    if(c != 0) {
      /* Write ParaNut output to host application */
      putchar(c);
      /* Clear TOHOST (Functions as acknowledgment) */
      paranut_clear_tohost ();
    }
    
    /* Check queue for input we have to write to FROMHOST (From ARM/Host-PC to ParaNut) */
    if(!qempty() && paranut_fromhost_empty()) {
        paranut_putc(qdequeue());
    }
    
  }

  /* Shutdown ... */
  printf ("# ParaNut Proxy firmware says: Goodbye!\nx\n");
  cleanup_platform();
  return 0;
}
