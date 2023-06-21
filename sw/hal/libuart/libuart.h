/*************************************************************************

  This file is part of the ParaNut project.

  Copyright 2023 Elias Schuler <elias.schuler@hs-augsburg.de>
                            
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
/**
 * \file
 * \brief      UART API.
 */


#include <stdio.h>
#include <unistd.h>
#include <paranut-config.h>
#include <stdint.h>

/** 
 * \mainpage UART Documentation
 * 
 * \section ParaNut UART Library Documentation
 * This is a library intended to provide an easier way to use when developing a UART compatible application on the Paranunt. 
 * All the Functions are defined in the libuart.h
*/

/**
 * \defgroup   libuart libuart
 *
 * \brief      Functions for accessing ParaNut UART 
 *
 * This Module contains functions for accessing the UART Modul of the ParaNut for further information how to use the UART Modul  
 * consider the README in the /<PARANUT directory>/sw/uart_test and the official Texas Instruments Documentation of the TL16C750 Single UART with 64-Byte Fifos.
 */

/**
 * \addtogroup libuart
 * @{
 */

/**
 * \def        UART_DATA_IN_REGISTER 
 * \brief      Defines the address of the Receiver Buffer Register
 * 
 * the Receiver Buffer Register is based at the address CFG_UART_BASE_ADDRESS it is read only 
 */
#define UART_DATA_IN_REGISTER      (CFG_UART_BASE_ADDRESS + 0x00)

/**
 * \def        UART_DATA_OUT_REGISTER 
 * \brief      Defines the address of the Transmitter Holding Register 
 * 
 * The Transmitter Holding Register is based at the address CFG_UART_BASE_ADDRESS  it is write only
 */
#define UART_DATA_OUT_REGISTER      (CFG_UART_BASE_ADDRESS + 0x00)

/**
 * \def        UART_DLL_REGISTER 
 * \brief      Defines the address of divisor latch (lsb)
 * 
 * The divisor latch (lsb) is based at the address CFG_UART_BASE_ADDRESS it can only be accessed if the Divisor Latch Access Bit(DLAB) is set
 * It is the lower 8 bits of the divisor for the baudrate
 */
#define UART_DLL_REGISTER       (CFG_UART_BASE_ADDRESS + 0x00)


/**
 * \def        UART_DLM_REGISTER 
 * \brief      Defines the address of divisor latch (msb)
 * 
 * The divisor latch (msb) is based at the address CFG_UART_BASE_ADDRESS + 0x4 it can only be accessed if the Divisor Latch Access Bit(DLAB) is set
 * It is the most signifigcant 8 bits of the divisor for the baudrate
 */
#define UART_DLM_REGISTER       (CFG_UART_BASE_ADDRESS + 0x04)

/**
 * \def        UART_IIR_REGISTER 
 * \brief      Defines the address of the interrupt identifier register
 * 
 * The interrupt identifier register is based at the address CFG_UART_BASE_ADDRESS + 0x8 it is a read only register
 */
#define UART_IIR_REGISTER       (CFG_UART_BASE_ADDRESS + 0x08)

/**
 * \def        UART_FCR_REGISTER 
 * \brief      Defines the address of the fifo control register
 * 
 * The fifo control register is based at the address CFG_UART_BASE_ADDRESS + 0x8 it is a write only register
 */
#define UART_FCR_REGISTER       (CFG_UART_BASE_ADDRESS + 0x08)

/**
 * \def        UART_LCR_REGISTER 
 * \brief      Defines the address of the line control register
 * 
 * The line control register is based at the address CFG_UART_BASE_ADDRESS + 0xC
 */
#define UART_LCR_REGISTER       (CFG_UART_BASE_ADDRESS + 0x0C)

/**
 * \def        UART_MCR_REGISTER 
 * \brief      Defines the address of the modem control register
 * 
 * The modem control register is based at the address CFG_UART_BASE_ADDRESS + 0xF
 */
#define UART_MCR_REGISTER       (CFG_UART_BASE_ADDRESS + 0x0F)

/**
 * \def        UART_LSR_REGISTER 
 * \brief      Defines the address of the line status register
 * 
 * The line status register is based at the address CFG_UART_BASE_ADDRESS + 0x14
 */
#define UART_LSR_REGISTER       (CFG_UART_BASE_ADDRESS + 0x14)

/**
 * \def        UART_MSR_REGISTER 
 * \brief      Defines the address of the modem status register
 * 
 * The modem status register is based at the address CFG_UART_BASE_ADDRESS + 0x18
 */
#define UART_MSR_REGISTER       (CFG_UART_BASE_ADDRESS + 0x18)

/**
 * \def        BAUDRATE_115200
 * \brief      Define for the function parameter of uart_init
 * 
 * Predetermined value for a Baudrate of 115200
 */
#define BAUDRATE_115200 1

/**
 * \def        BAUDRATE_52600
 * \brief      Define for the function parameter of uart_init
 * 
 * Predetermined value for a Baudrate of 52600
 */
#define BAUDRATE_52600 2


/**
 * \def        BAUDRATE_9600
 * \brief      Define for the function parameter of uart_init
 * 
 * Predetermined value for a Baudrate of 9600
 */
#define BAUDRATE_9600 12


/**
 * \def        UART_SUCCESS
 * \brief      Return Value of Functions
 * 
 * UART_SUCCESS gets returned when one of the Functions runs sucessfully
 */
#define UART_SUCCESS 1

/**
 * \def        UART_FAILED
 * \brief      Return Value of Functions
 * 
 * UART_FAILED gets returned when one of the Functions fails
 */
#define UART_FAILED -1 


//Struct for initialisation with Parameters Baudrate, Wordlength and Fifo64ByteEnable

/**
 * \struct     uart_setup
 * \brief      A struct holding the setup settings for the uart module
 * 
 *  This holds the 3 setup options the libuart api is currently supporting in the uart_init
 *  These options are the int baudrate which are defined in defines BAUDRATE_9600, BAUDRATE_52600 and BAUDRATE_115200
 *  the int wordLength with valid values between 5 and 8 including 5 and 8. 
 *  and the int fifo64ByteMode
 */
struct uart_setup{
   int baudrate;
   int wordLength;
   int fifo64ByteMode;
};

/**
 * \fn         void uart_init(struct uart_setup)
 * \brief     Initializes the UART Module with the specific setup needed for the bluetooth modul
 * 
 * Initialisation of the UART Module with a struct of baudrate, wordLength and fifo64byteMode
 * The baudrate can be chosen from predetermined defines, more defines can be added from the Table 9. of the Data Sheet 
 * for the Texas Instruments Uart 16750
 * Word lengths can be 5, 6, 7 or 8 as specified in the data sheet
 * fifo64ByteMode enables the Fifo in 64 Byte else it will be 16 Bytes
 * 
 * \todo: add more setup options like even or odd parity and Stob bit selection
 * 
 * \return     void
 */
void uart_init(struct uart_setup initSetup);


/**
 * \fn         int8_t uart_data_ready()
 * \brief     Returns a UART_SUCCESS that is defined when data is available
 * 
 * Checks the Line Status Register to see, if there is data available to receive from the UART Fifo
 * Returns UART_SUCCESS if there is data available else returns UART_FAILED
 * 
 * \return     UART_SUCCESS or UART_FAILED with UART_SUCCESS beeing equal to 1 and UART_FAILED eqaul to -1
 */
int8_t uart_data_ready();

/**
 * \fn         void uart_send_char(char toSent)
 * \brief     Sends one character over the UART
 *
 * \return     void
 */
void uart_send_char(char toSent);


/**
 * \fn         int8_t uart_receive_char(int8_t enableBlocking, char* out, uint32_t timeOutAttempts)
 * \brief     Receive one character from the Receive Fifo of the UART 
 * Receive one character from the Uart Fifo
 * This function returns a boolean value if it succeded and the out pointer with the read value
 * bool flag enableBlocking to decide if the function is run blocking or non blocking
 * if fifo is empty on execution return UART_FAILED
 * timeOutAttempts specifices how many tries the Blocking variant will take before leaving the function a value of 0 correlates to never
 * 
 * \return     UART_SUCCESS or UART_FAILED with UART_SUCCESS beeing equal to 1 and UART_FAILED eqaul to -1
 */
int8_t uart_receive_char(int8_t enableBlocking, char* out, uint32_t timeOutAttempts);


/**
 * \fn         void uart_write_register(uint32_t address, uint32_t data)
 * \brief     Function to write to an register of the Uart
 * Function to write to an register of the Uart 
 * the writeable registers can be determint by the documentation of the UART TL16C750.
 * The addresses for each of the registers are defined in the libuart.h and can be used to access each
 * of the registers.
 * 
 * \return     void
 */
void uart_write_register(uint32_t address, uint32_t data);

/**
 * \fn         uint32_t uart_read_register(uint32_t address, uint32_t data)
 * \brief     Function to write to an register of the Uart
 * Function to write to an register of the Uart 
 * the readable registers can be determint by the documentation of the UART TL16C750.
 * The addresses for each of the registers are defined in the libuart.h and can be used to access each
 * of the registers.
 * 
 * \return     uint32_t with the data which is read from the register
 */
uint32_t uart_read_register(uint32_t address);

/**
 * \fn         void uart_read_register()
 * \brief     Function to clear the Receiver Fifo of the UART Module
 * Function to clear the Receiver Fifo of the UART Module 
 * Sets the Bit 2 of the FCR Register which clears the FIFO of the RX_Fifo
 * 
 * \return     void
 */
void uart_clear_rx_fifo();

/**
 * \fn         void uart_read_register()
 * \brief     Function to clear the Transmitter Fifo of the UART Module
 * Function to clear the Transmitter Fifo of the UART Module 
 * Sets the Bit 2 of the FCR Register which clears the FIFO of the RX_Fifo
 * 
 * \return     void
 */
void uart_clear_tx_fifo();

/**
 * \fn        void uart_done()
 * \brief     This function is used to cleanup the uart
 * This function is used to cleanup the uart 
 * No functionality yet
 * can be added if needed
 * 
 * \return     void
 */
void uart_done();



