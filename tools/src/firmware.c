/**************************************************************************
 *
 *  This file is part of the ParaNut project.
 *
 *  Copyright (C) 2018-2022 Alexander Bahle <alexander.bahle@hs-augsburg.de>
 *                          Gundolf Kiefer <gundolf.kiefer@hs-augsburg.de>
 *                          Michael Schaeferling <michael.schaeferling@hs-augsburg.de>
 *                          Nico Borgsmueller <nico.borgsmueller@hs-augsburg.de>
 *      Efficient Embedded Systems Group
 *      Hochschule Augsburg, University of Applied Sciences
 *
 *  Description:
 *    Firmware main program for interacting with a ParaNut through the
 *    pn-flash python tool.
 *
 *  --------------------- LICENSE ----------------------------------------------
 *  Redistribution and use in source and binary forms, with or without modification,
 *  are permitted provided that the following conditions are met:
 *
 *  1. Redistributions of source code must retain the above copyright notice, this
 *     list of conditions and the following disclaimer.
 *
 *  2. Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation and/or
 *     other materials provided with the distribution.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 *  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 *  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 *  ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 *  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 *  ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 *  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 **************************************************************************/

#include <stdio.h>
#include "platform.h"
#include "xil_printf.h"
#include "xil_io.h"
#include "xil_cache.h"
#include "sleep.h"
#include "xparameters.h"
#include "xgpiops.h"
#include "xtime_l.h"
#include "uzlib.h"

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
 *    CMD:  "\0x16 ?b <adr> <size> <block_size> <compressed>"
 *    ANS1: "\0x06 ?b <adr> <size> <block_size> <compressed>"
 *    ... RAW DATA TRANSFER, expects following ACK every "block_size" bytes:
 *    ACK: "\0x06 ?k"
 *    ... UNTIL SIZE ...
 *    ANS2: "\0x06 ?b <adr+size> <size> <block_size> <compressed>"
 *
 *  Write large amount of data to memory (e.g. application code).
 *  If the compressed flag (1 or 0) is set, the content should be uncompressed with gzip to the given location.
 *
 *  The first answer is the acknowledgement and start signal for the host application to start sending
 *  the raw data. To control the transmission speed and prevent the receive buffer from overflowing,
 *  data is sent in chunks of "block_size" bytes, which have to be acknowledged by the firmware.
 *  If enough bytes were received (= <size>) the firmware sends the second answer to
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

static XGpioPs Gpio; /* The instance of the GPIO driver */

#define RESET_PIN 54 /* The pin number of the EMIO reset pin */

#define RESET_BTN_PIN 50 /* The pin number of BTN4 on ZYBO boards used as reset pin */

#define LED_PIN 7 /* The pin number of the MIO LD4 LED */

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

static inline void pl_reset_high () {
  /* Set reset pin to high (reset not active) */
  XGpioPs_WritePin(&Gpio, RESET_PIN, 1);
  XGpioPs_WritePin(&Gpio, LED_PIN, 0);
}

static inline void pl_reset_low () {
  /* Set reset pin to low (reset active) */
  XGpioPs_WritePin(&Gpio, RESET_PIN, 0);
  XGpioPs_WritePin(&Gpio, LED_PIN, 1);
}

static inline void pl_reset () {
  /* Set reset pin to low, wait a bit and set reset pin high */
  pl_reset_low ();
  usleep(10);
  pl_reset_high ();
}


/***************************** Helpers ***************************************/


int init_xgpiops () {
  XGpioPs_Config *ConfigPtr;
  int Status;

  /* Initialize the Gpio driver. */
  ConfigPtr = XGpioPs_LookupConfig(XPAR_XGPIOPS_0_DEVICE_ID);
  if (ConfigPtr == NULL) {
    return XST_FAILURE;
  }

  XGpioPs_CfgInitialize(&Gpio, ConfigPtr, ConfigPtr->BaseAddr);

  /* Run a self-test on the GPIO device. */
  Status = XGpioPs_SelfTest(&Gpio);
  if (Status != XST_SUCCESS) {
    return XST_FAILURE;
  }

  /* Set the direction for the reset pin to be output. */
  XGpioPs_SetDirectionPin(&Gpio, RESET_PIN, 1);
  XGpioPs_SetOutputEnablePin(&Gpio, RESET_PIN, 1);
  XGpioPs_WritePin(&Gpio, RESET_PIN, 0x0);

  /* Set the direction for the reset button pin to be input. */
  XGpioPs_SetDirectionPin(&Gpio, RESET_BTN_PIN, 0);

  /* Set the direction for the LED pin to be output. */
  XGpioPs_SetDirectionPin(&Gpio, LED_PIN, 1);
  XGpioPs_SetOutputEnablePin(&Gpio, LED_PIN, 1);
  XGpioPs_WritePin(&Gpio, LED_PIN, 0x0);
}

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

static int read_until_ack;
static uint32_t ack_chunk_size;
int getchar_with_ack() {
  int new_char = getchar();
  read_until_ack += 1;

  if (read_until_ack == ack_chunk_size) {
    read_until_ack = 0;
    getchar(); // read final \n from FIFO
    printf ("\x06 ?k\n");
  }
  return new_char;
}

// This is called by the decompression library once new data is required
static uint32_t block_size;
int uncompress_callback(struct uzlib_uncomp *uncomp) {
  if (block_size > 0) {
    block_size -= 1;
    return getchar_with_ack();
  }
  return -1;
}

uint32_t decompress (const char *cmd, uint32_t adr, uint32_t size) {
  TINF_DATA tinf;
  uint8_t dict[32768];
  int tinf_res;
  uint8_t output;
  uint32_t i = 0;

  // Define size for decompression callback
  block_size = size;

  // Init data structure
  uzlib_uncompress_init(&tinf, dict, 32768);

  // Use callback for streaming decompression
  tinf.source = NULL;
  tinf.source_limit = NULL;
  tinf.source_read_cb = &uncompress_callback;

  tinf_res = uzlib_gzip_parse_header(&tinf);
  if (tinf_res != TINF_OK) {
    command_error ("Uncompress parse header failed", cmd);
    return 0;
  }

  // Decompress byte by byte, write it directly to target addr
  while (1) {
    // Always reset output buffer
    tinf.dest_start = tinf.dest = &output;
    // Only read one byte per step
    tinf.dest_limit = tinf.dest + 1;
    // Uncompress with checksum checking
    tinf_res = uzlib_uncompress_chksum(&tinf);
    if (tinf_res == TINF_DONE) {
      break;
    }
    if (tinf_res != TINF_OK) {
      char buf[100];
      sprintf(buf, "Uncompress failed: %i; bytes read %u; bytes produced: %i", tinf_res, block_size, i);
      command_error (buf, cmd);
      return 0;
    }
    // Write data out
    Xil_Out8(adr + i, output);
    i++;
  }
  return size - block_size;
}

void cmd_block_write (const char *cmd, const char *args) {
  uint32_t adr, size, chunk_size, compressed, cnt = 0;

  /* Sanity ... */
  if (!hardware_ok ()) return;

  /* Scan command ... */
  if (sscanf (args, "%lx %lx %lx %lx", &adr, &size, &chunk_size, &compressed) != 4) {
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

  // Reset ACK counter
  read_until_ack = 0;
  ack_chunk_size = chunk_size;
  /* Send acknowledge for program size first */
  printf("\x06 ?b %08lx %08lx %08lx %1lx\n", adr, size, chunk_size, compressed);

  /* Start program receiving */
  if (compressed) {
    adr = adr + decompress(cmd, adr, size);
  } else {
    for (uint32_t end_adr = adr + size; adr < end_adr; adr++){
        Xil_Out8(adr, getchar_with_ack());
    }
  }

  /* Write from cache to DDR */
  Xil_DCacheFlush();

  /* Print confirmation ... */
  printf("\x06 ?b %08lx %08lx %08lx %1lx\n", adr, size, chunk_size, compressed);
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

  clear_receiver();
}

void cmd_reset_pl (const char *cmd, const char *args) {
  /* Just reset the PL ... */
  uint32_t value;

  /* Scan command ... */
  if (sscanf (args, "%x", &value) != 1) {
      command_error ("Syntax error", cmd);
      return;
  }

  //printf ("(FIRMWARE) Set Reset PL to %d ...\n", value);
  if (value == 0)
    /* 0 = reset not active */
    pl_reset_high ();
  else
    /* != 0 = reset active */
    pl_reset_low ();

  /* Send acknowledge ... */
  printf ("\x06 ?n %02x\n", value);
}

void cmd_peripherals_init (const char *cmd, const char *args) {
    /* No peripherals to init by default, just continue */

    /* Send ack to signal the operation being finished */
    printf ("\x06 ?a\n");
}


void run_command (char *cmd) {

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
          case 'n':
            cmd_reset_pl (cmd, cmd+4);
            break;
          case 'a':
            cmd_peripherals_init (cmd, cmd+4);
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
  BOOL reset_btn = 0, last_reset_btn = 0;
  int cmd_idx = -1;
  XTime debounceTime, currentTime;

  /* Initialize ... */
  init_platform();
  Xil_ICacheEnable();
  Xil_DCacheEnable();
  init_xgpiops ();
  qinit();

  printf ("(FIRMWARE) ParaNut Proxy firmware v%i.%i.%i ready.\n",
          VERSION_MAJOR, VERSION_MINOR, VERSION_REVISION);
  check_hardware ();

  /* Clear UART and TOHOST/FROMHOST ... */
  clear_receiver();

  uzlib_init();

  /* Main loop ... */
  quit = 0;
  while (!quit) {
    /* Check reset button ... */
    int cur_val = XGpioPs_ReadPin(&Gpio, RESET_BTN_PIN) ;
    if (cur_val != reset_btn) {
      /* Set debounce time on a change of the value */
      XTime_GetTime(&debounceTime);
    }

    /* Wait till the change stays at least 1ms befor resetting the ParaNut */
    XTime_GetTime(&currentTime);
    if ((currentTime - debounceTime) > (COUNTS_PER_SECOND/1000)) {
      /* Reset PL low on 0 to 1 transition */
      if (last_reset_btn == 0 && cur_val == 1) {
        printf ("(FIRMWARE) Set Reset\n");
        pl_reset_low ();
      }
      /* Reset PL high on 1 to 0 transition */
      if (last_reset_btn == 1 && cur_val == 0) {
        printf ("(FIRMWARE) Unset Reset\n");
        pl_reset_high ();
      }
      last_reset_btn = cur_val;
    }
    reset_btn = cur_val;

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
                    run_command(command); // TBD
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
