/*************************************************************************

  This file is part of the ParaNut project.

  Copyright (C) 2023 Elias Schuler <elias.schuler@hs-augsburg.de>
                            
      Hochschule Augsburg, University of Applied Sciences

  Description:
     This File is the API for the UART Module for easier development

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

#include "libuart.h"

// Initialisation of the UART Module right now setup so it runs in 115200 baudrate with 1 Stop Bit and 8 Data Bits
// No flow Control enabled, FIFO in 64 byte Mode and enabled as needed for the transmission to a bluetooth module
void uart_init(struct uart_setup initSetup){

  volatile uint32_t* uart_dll = (uint32_t*) UART_DLL_REGISTER;
  volatile uint32_t* uart_dlm = (uint32_t*) UART_DLM_REGISTER;
  volatile uint32_t* uart_lcr = (uint32_t*) UART_LCR_REGISTER;
  volatile uint32_t* uart_fcr = (uint32_t*) UART_FCR_REGISTER;
  // Set DLAB to configure the UART module
  *uart_lcr = 0x80;
  // Set the upper byte of the divisor latch
  *uart_dlm = 0x00;
  // Set the lower byte of the divisor latch
  *uart_dll = initSetup.baudrate;
  // Enable the RX/TX FIFOs in 64 byte mode or 16 byte mode
  *uart_fcr = (initSetup.fifo64ByteMode != 0) ?0x21:0x01;
  // Disable DLAB and set the word length to wordLength defaults to 8
  if(initSetup.wordLength >= 5 && initSetup.wordLength <= 8){
    *uart_lcr = initSetup.wordLength-5;
  } else {
    *uart_lcr = 0x3;
  }

}

// Send one character over the uart
void uart_send_char(char toSent){
  volatile uint32_t* uart_data_out = (uint32_t*) UART_DATA_OUT_REGISTER;
  *uart_data_out = toSent;
}

// Checks the Line Status Register to see, if there is data available to receive from the UART Fifo
int8_t uart_data_ready(){
  volatile uint32_t* uart_lsr = (uint32_t*) UART_LSR_REGISTER;
  if(!((*(uart_lsr) & 0x01) == 0x1)){
    return UART_FAILED;
  }else{
    return UART_SUCCESS;
  }
}

// Receive one character from the Uart Fifo
// bool flag enableBlocking to decide if the function is run blocking or non blocking
// if fifo is empty on execution return 0
int8_t uart_receive_char(int8_t enableBlocking, char* out, uint32_t timeOutAttempts){
  volatile uint32_t* uart_data_in = (uint32_t*) UART_DATA_IN_REGISTER;
  char rcv_data;
  if(enableBlocking){
      long attempts = 0;
      // wait for a data ready bit of the line status register or for a timeOut
      while(uart_data_ready() == UART_FAILED && (attempts < timeOutAttempts || timeOutAttempts == 0)){
        attempts++;
      }
      // receive the available data on the register or get time outed
      if(attempts == timeOutAttempts && timeOutAttempts != 0){
        return UART_FAILED;
      } else {
        rcv_data = *(uart_data_in);
        *out = rcv_data;
        return UART_SUCCESS;
      }
  } else {
      // check if data is available on the Line Status Register 
      if(uart_data_ready() == UART_FAILED){
          // if no data is available return UART_FAILED
          return UART_FAILED;
      } else {
          // if data is available return the newest fifo read
          rcv_data = *(uart_data_in);
          *out = rcv_data;
          return UART_SUCCESS; 
      }
  }
}

// Function to write to an register of the Uart 
// the writeable registers can be determint by the documentation of the UART TL16C750
void uart_write_register(uint32_t address, uint32_t data){
    volatile uint32_t* registerToWrite = (uint32_t*) address;
    *(registerToWrite) = data;
}

// Function to read an register of the Uart 
// the readable registers can be determint by the documentation of the UART TL16C750
uint32_t uart_read_register(uint32_t address){
    volatile uint32_t* registerToRead = (uint32_t*) address;
    uint32_t data;
    data = *(registerToRead);
    return data;
}

void uart_clear_rx_fifo(){
  // Sets bit 2 of the FCR register to clear the RX FIFO
  volatile uint32_t* fcr = (uint32_t*)UART_FCR_REGISTER;
  *fcr |= 0x2;
}

void uart_clear_tx_fifo(){
  // Sets bit 3 of the FCR register to clear the TX FIFO
  volatile uint32_t* fcr = (uint32_t*)UART_FCR_REGISTER;
  *fcr |= 0x4;
}

// This function is needed to cleanup the uart 
// No functionality yet
void uart_done(){
    // If later on the UART needs cleaning you can add the cleanup code here
}