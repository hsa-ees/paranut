/*************************************************************************

  This file is part of the ParaNut project.

  Copyright (C) 2023 Elias Schuler <elias.schuler@hs-augsburg.de>
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

#include <libuart.h>


int main () {
  struct uart_setup uart_setup;
  uart_setup.baudrate = BAUDRATE_115200;
  uart_setup.wordLength = 8;
  uart_setup.fifo64ByteMode = 1;
  uart_init(uart_setup);

  // Write the whole alphabet to the TX FIFO to be sent
  for(size_t i = 0; i <26; i++){
    uart_send_char((0x41+i));
  }
  char rcv_data;

  while (1){
    // check if a word was received
    if(uart_receive_char(1, &rcv_data, 0) == UART_SUCCESS){

      // send the received data to the UART
      uart_send_char(rcv_data);

      printf("%c\n", rcv_data);
    }else{
      printf("AN ERROR OCCURED WHILE READING!\n");
    }

  }

  return 0;
}