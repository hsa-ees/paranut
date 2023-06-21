/*************************************************************************

  This file is part of the ParaNut project.

  Copyright (C) 2023 Elias Schuler <elias.schuler@hs-augsburg.de>
                            
      Hochschule Augsburg, University of Applied Sciences

  Description:
     This File is the API to use with the BM 70 Bluetooth Module from Microchip

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
 * \brief      Bluetooth API.
 */
#include <stdio.h>
#include <unistd.h>
#include <stdint.h>
#include <paranut.h>
#include <libgpio.h>
#include <libuart.h>
#include <string.h>

/** 
 * \mainpage Bluetooth Documentation
 * 
 * \section ParaNut Bluetooth Library Documentation
 * This is a library intended to provide an easier way to use when developing a Bluetooth compatible application on the Paranunt. 
 * All the Functions are defined in the libbluetooth.h
*/

/**
 * \defgroup   libbluetooth libbluetooth
 *
 * \brief      Functions for accessing ParaNut Bluetooth
 *
 * This Module contains functions for accessing the Bluetooth Module of the ParaNut for further information how to use the Bluetooth Module  
 * consider the README in the /<PARANUT directory>/sw/libbluetooth.
 */

/**
 * \addtogroup libbluetooth
 * @{
 */



/**
 * \def        BLUETOOTH_SUCCESS
 * \brief      Return Value of Functions
 * 
 * BLUETOOTH_SUCCESS gets returned when one of the Functions runs sucessfully
 */
#define BLUETOOTH_SUCCESS 1


/**
 * \def        BLUETOOTH_GPIO_ERROR
 * \brief      Return Value of Functions
 * 
 * BLUETOOTH_GPIO_ERROR gets returned when one of the Functions runs into an GPIO_ERROR this is mostly due to using pins out of range for the Setup
 * 
 */
#define BLUETOOTH_GPIO_ERROR -1 


/**
 * \def        BLUETOOTH_STATUS_ERROR
 * \brief      Return Value of Functions
 * 
 * BLUETOOTH_GPIO_ERROR gets returned when one of the Functions runs into an STATUS_ERROR
 * This gets returned when the Bluetooth Function either reaches an illogical Status or when you try to send data when not in 
 * BLUETOOTH_TRANSPARTENT_UART_READY State
 */
#define BLUETOOTH_STATUS_ERROR -2

/**
 * \def        BLUETOOTH_NO_DATA
 * \brief      Return Value of Functions
 * 
 * BLUETOOTH_NO_DATA gets returned when there is no Data in the UART Receiver Fifo
 */
#define BLUETOOTH_NO_DATA -3

/**
 * \def        BLUETOOTH_TIME_OUT
 * \brief      Return Value of Functions
 * 
 * BLUETOOTH_TIME_OUT gets returned when the amount of timeOutAttempts is reached and the function timesOut
 */
#define BLUETOOTH_TIME_OUT -4

/**
 * \def        BLUETOOTH_DATA
 * \brief      Return Value of Functions
 * 
 * BLUETOOTH_DATA gets returned when there is Data in the UART Receiver Fifo
 */
#define BLUETOOTH_DATA 2

/**
 * \def        BLUETOOTH_SHUTDOWN_MODE
 * \brief      Return Value of Functions
 * 
 * BLUETOOTH_SHUTDOWN_MODE gets returned from the bluetooth_check_connectionstatus when the Module is in Shutdown Mode
 */
#define BLUETOOTH_SHUTDOWN_MODE 10


/**
 * \def        BLUETOOTH_STANDBYE_MODE
 * \brief      Return Value of Functions
 * 
 * BLUETOOTH_STANDBYE_MODE gets returned from the bluetooth_check_connectionstatus when the Module is in Standbye Mode
 */
#define BLUETOOTH_STANDBYE_MODE 11

/**
 * \def        BLUETOOTH_CONNECTED_MODE
 * \brief      Return Value of Functions
 * 
 * BLUETOOTH_CONNECTED_MODE gets returned from the bluetooth_check_connectionstatus when the Module connected, 
 * but not yet ready to transmit over the GATT Service
 */
#define BLUETOOTH_CONNECTED_MODE 12

/**
 * \def        BLUETOOTH_TRANSPARENT_UART_READY
 * \brief      Return Value of Functions
 * 
 * BLUETOOTH_TRANSPARENT_UART_READY gets returned from the bluetooth_check_connectionstatus when the Module connected, 
 * and ready to receive and send data over the Transparent UART GATT Service
 */
#define BLUETOOTH_TRANSPARENT_UART_READY 13

/**
 * \def        BLUETOOTH_POLL_REACHED
 * \brief      Return Value of Functions
 * 
 * BLUETOOTH_POLL_REACHED gets returned from the bluetooth_poll_status function once the assigned status is reached.
 */
#define BLUETOOTH_POLL_REACHED 42


/**
 * \fn         int8_t bluetooth_init(uint8_t statusPin1, uint8_t statusPin2, uint8_t wakeUpPin)
 * \brief      Function to initialise the bluetooth Module
 *
 * 
 * Initialises the Bluetooth Module, for this the GPIO gpio_init() function and the UART uart_init() function are beeing called
 * The Parameter statusPin1 and statusPin2 are the Input Pin number on which the status pin of the BM71 board are beeing connected
 * The Parameter wakeUpPin is the Output Pin number on which the wakeup Pin of the BM71 board is beeing connected, this one is LOW active so we pull it HIGH 
 * These get saved in global variables to be used in the other functions of the Bluetooth Module
 * 
 * 
 * \return     BLUETOOTH_SUCCESS when the initialisation was successful  BLUETOOTH_GPIO_ERROR when wrong pin numbers were given
 */
int8_t bluetooth_init(uint8_t statusPin1, uint8_t statusPin2, uint8_t wakeUpPin);


/**
 * \fn         int8_t bluetooth_get_inputPinNumberStatusPin1()
 * \brief      Getter Function
 *
 * 
 * Gets the inputPinNumberStatusPin1 set in the uart_init()
 * 
 * 
 * \return     int8_t inputPinNumberStatusPin1
 */
int8_t bluetooth_get_inputPinNumberStatusPin1();

/**
 * \fn         int8_t bluetooth_get_inputPinNumberStatusPin2()
 * \brief      Getter Function
 *
 * 
 * Gets the inputPinNumberStatusPin2 set in the uart_init()
 * 
 * 
 * \return     int8_t inputPinNumberStatusPin2
 */
int8_t bluetooth_get_inputPinNumberStatusPin2();

/**
 * \fn         int8_t bluetooth_get_outputPinNumberWakeUpPin()
 * \brief      Getter Function
 *
 * 
 * Gets the outputPinNumberWakeUpPin set in the uart_init()
 * 
 * 
 * \return     int8_t inputPinNumberStatusPin1
 */
int8_t bluetooth_get_outputPinNumberWakeUpPin();

/**
 * \fn         int8_t bluetooth_check_connectionstatus()
 * \brief      Gets in what Status the Bluetooth Module is right now
 *
 * 
 * Gets inputPinNumberStatusPin1 and inputPinNumberStatusPin2 and returns the correlating Status
 * 
 * 
 * \return     BLUETOOTH_SHUTDOWN_MODE, BLUETOOTH_STANDBYE_MODE, BLUETOOTH_CONNECTED_MODE and BLUETOOTH_TRANSPARENT_UART_READY
 */
int8_t bluetooth_get_status();


/**
 * \fn         int8_t bluetooth_poll_status(int8_t statusToReach, int64_t uSecIntervall, uint32_t timeOutAttempts)
 * \brief      Active waiting function activly waiting until the statusToReach gets reached 
 *
 * 
 * Actively waits for the Bluetooth Status to reach the set statusToReach in the function parameters,
 * each attempt the funtion checks if the Module fell into BLUETOOTH_SHUTDOWN_MODE and wakes it up in that case 
 * or if a BLUETOOTH_STATUS_ERROR occured in which case it returns with an ERROR
 * The wait time between each check can be set with the uSecIntervall Parameter.
 * When the function reaches the determined Status it returns a BLUETOOTH_POLL_REACHED
 * timeOutAttempts is the maximum amount of times the function gets the new status so the maximum amount of wait time can be calculated by
 * maxWaitTime = uSecIntercall*timeOutAttempts
 * if a value of 0 is set for the timeOutAttempts the function never times out
 * 
 * 
 * \return     BLUETOOTH_POLL_REACHED when the statusToReach gets reached else BLUETOOTH_GPIO_ERROR when the Pin selected isn't valid or BLUETOOTH_STATUS_ERROR when a invalid Status is reached
 */
int8_t bluetooth_wait_status(int8_t statusToReach, int64_t uSecIntervall, uint32_t timeOutAttempts);


/**
 * \fn         int8_t bluetooth_send_data(int dataLength, char* transmitString)
 * \brief      Sends Multiple Charakters over the Bluetooth Module
 *
 * 
 * Checks if the Status is BLUETOOTH_TRANSPARENT_UART_READY,
 * if thats the case it sends the Information over Bluetooth.
 * For this a char* transmitString has to be passed to the function aswell as the dataLength of the char*.
 * DATA IS LOST WHEN FIFO RUNS OVER
 * 
 * 
 * \return     BLUETOOTH_SUCCESS when the transmission was succesful on the part of the ParaNut else BLUETOOTH_STATUS_ERROR when not in the BLUETOOTH_TRANSPARENT_UART_READY State
 */
int8_t bluetooth_send_data(int dataLength, char* transmitString);


/**
 * \fn         int8_t bluetooth_data_ready()
 * \brief      Checks the Uart Receiver Fifo if atleast one byte is set
 *
 * 
 * Checks if their is Data in the UART Receiver Fifo if atleast one byte is there returns a BLUETOOTH_DATA
 * otherwise returns BLUETOOTH_NO_DATA
 * 
 * 
 * 
 * \return     returns a BLUETOOTH_DATA otherwise returns BLUETOOTH_NO_DATA
 */
int8_t bluetooth_data_ready();
/**
 * \fn         int8_t bluetooth_receive_data(int dataLength,char* receivedString, uint8_t blockingEnabled, uint32_t timeOutAttempts)
 * \brief      Receives multiple charakters over the bluetooth module
 *
 * 
 * Checks if the Status is BLUETOOTH_TRANSPARENT_UART_READY,
 * if thats the case it runs the uart_receive_char() with blockingEnabled and timeOutAttempts both can be decided by the parameter timeOutAttempts and blockingEnabled
 * If Data is received it gets appended to the receivedString until either the dataLength is reached or a Line Break is received
 * or it timeOutAttempts get reached
 * It also gets terminated by a Incorrect Status.  
 * In nonBlockingMode there isn't a guarantee you receive the whole message, it is sleeping for 20 useconds between each read, but doesn't 
 * guarantee you will get a whole command.
 * It returns the amount of characters it succesfully received from the uart module the rest of the Fifo gets cleared after a read, 
 * so that only one command can be send at a time 
 * 
 * 
 * \return     Returns the dataLength which has been received successfully
 */
int8_t bluetooth_receive_data(int dataLength,char* receivedString, uint8_t blockingEnabled, uint32_t timeOutAttempts);



/**
 * \fn         int8_t bluetooth_wake_up()
 * \brief      Wakes up the Bluetooth Module
 *
 * 
 * Wakes up the Bluetooth Module for this the outputPinNumberWakeUpPin is driven low for 10 millioseconds
 * After this the Pin is driven high again which eqauls a non actived state and we wait for 33 milliseconds so the BM70 Module reaches an active state
 * This wakes the Bluetooth Module up and its back in the BLUETOOTH_STANDBYE_MODE
 * 
 * 
 * \return     BLUETOOTH_SUCCESS when outputPinNumberWakeUpPin was driven low for 10 milliseconds else BLUETOOTH_GPIO_ERROR when the Pin selected isn't valid
 */
int8_t bluetooth_wake_up();



/**
 * \fn         int8_t bluetooth_done();
 * \brief      This function cleans up the Bluetooth Module
 *
 * 
 * This Function cleans up the Bluetooth Module by calling the uart_done() and the gpio_done() functions 
 * aswell as driving the outputPinNumberWakeUpPin to low a again like its by default
 * 
 * 
 * \return     BLUETOOTH_SUCCESS when function gets run correctly else BLUETOOTH_GPIO_ERROR when the Pin selected isn't valid
 */
int8_t bluetooth_done();



