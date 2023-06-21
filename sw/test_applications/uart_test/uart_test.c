/*************************************************************************

  This file is part of the ParaNut project.

  Copyright (C) 2023 Elias Schuler <elias.schuler@hs-augsburg.de>
        					   Lukas Bauer <lukas.bauer@hs-augsburg.de>
        Augsburg, University of Applied Sciences

  Description:
     Demo application showcasing the functionality of the UART-Module

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
#include <unistd.h>
#include <stdint.h>

#define UART_BASE_ADDRESS       0x60000000
#define UART_DATA_REGISTER      (UART_BASE_ADDRESS + 0x00)
#define UART_DLL_REGISTER       (UART_BASE_ADDRESS + 0x00)
#define UART_DLM_REGISTER       (UART_BASE_ADDRESS + 0x04)
#define UART_FCR_REGISTER       (UART_BASE_ADDRESS + 0x08)
#define UART_LCR_REGISTER       (UART_BASE_ADDRESS + 0x0C)
#define UART_MCR_REGISTER       (UART_BASE_ADDRESS + 0x0F)
#define UART_LSR_REGISTER       (UART_BASE_ADDRESS + 0x14)


int main () {

  volatile uint32_t* uart_data = (uint32_t*) UART_DATA_REGISTER;
  volatile uint32_t* uart_dll = (uint32_t*) UART_DLL_REGISTER;
  volatile uint32_t* uart_dlm = (uint32_t*) UART_DLM_REGISTER;
  volatile uint32_t* uart_fcr = (uint32_t*) UART_FCR_REGISTER;
  volatile uint32_t* uart_lcr = (uint32_t*) UART_LCR_REGISTER;
  volatile uint32_t* uart_mcr = (uint32_t*) UART_MCR_REGISTER;
  volatile uint32_t* uart_lsr = (uint32_t*) UART_LSR_REGISTER;

  // for(size_t i = 0; i <100; i++);

  // Set DLAB to configure the UART module
  *uart_lcr = 0x80;


  // for(size_t i = 0; i <100; i++);
  // Set the upper byte of the divisor latch (for a Baudrate of 115200)
  *uart_dlm = 0x00;

  // for(size_t i = 0; i <100; i++);
  // Set the lower byte of the divisor latch (for a Baudrate of 115200)
  *uart_dll = 0x01;

  // for(size_t i = 0; i <100; i++);
  // Enable the RX/TX FIFOs in 64 byte mode
  *uart_fcr = 0x21;

  // for(size_t i = 0; i<100; i++);
  // Disable DLAB and set the word length to 8 byte
  *uart_lcr = 0x03;

  // Write the whole alphabet to the TX FIFO to be sent
  for(size_t i = 0; i <26; i++){
    *uart_data = 0x41+i;
  //   for(size_t j = 0; j < 100; j++);
  }

  char rcv_data;

  while (1){
    // check if a word was received
    if ((*(uart_lsr) & 0x01) == 0x1){

    // get the received data from the UART
    rcv_data = *(uart_data);

    // send the received data to the UART
    *uart_data = rcv_data;

    printf("%c\n", rcv_data);

    }

  }

  return 0;
}