/*************************************************************************

  This file is part of the ParaNut project.

  Copyright (C) 2023 Elias Schuler <elias.schuler@hs-augsburg.de>
                            
      Hochschule Augsburg, University of Applied Sciences

  Description:
     This File is the API for the GPIO Module

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
 * \brief      GPIO API.
 */
#include <stdio.h>
#include <unistd.h>
#include <stdint.h>
#include <paranut-config.h>

/** 
 * \mainpage GPIO Documentation
 * 
 * \section ParaNut GPIO Library Documentation
 * This is a library intended to provide an easier way to use when developing a GPIO compatible application on the Paranunt. 
 * All the Functions are defined in the libgpio.h
*/

/**
 * \defgroup   libgpio libgpio
 *
 * \brief      Functions for accessing ParaNut GPIO
 *
 * This Module contains functions for accessing the GPIO Modul of the ParaNut for further information how to use the GPIO Modul  
 * consider the README in the /<PARANUT directory>/sw/libgpio.
 */

/**
 * \addtogroup libgpio
 * @{
 */


/**
 * \def        GPIO_PINS 
 * \brief      Defines the address for accessing the register of the GPIO Module
 * 
 * the register of the GPIO Module is based at the address CFG_GPIO_BASE_ADDRESS
 */
#define GPIO_PINS      (CFG_GPIO_BASE_ADDRESS + 0x00)

/**
 * \def        GPIO_AMOUNT 
 * \brief      Defines the amount of Gpio Pins specified by the developer in the config.mk for ease of use
 * 
 */
#define GPIO_AMOUNT (CFG_GPIO_AMOUNT)

/**
 * \def        GPIO_INAMOUNT 
 * \brief      Defines the amount of GPIO Input pins specified by the developer for ease of use
 *   CFG_GPIO_INAMOUNT is calculated by subtracting CFG_GPIO_AMOUNT - CFG_GPIO_OUTAMOUNT
 */
#define GPIO_INAMOUNT (CFG_GPIO_INAMOUNT)
// Defines the amount of GPIO Output pins specified by the developer in the config.mk for ease of use

/**
 * \def        GPIO_OUTAMOUNT 
 * \brief      Defines the amount of GPIO Output pins specified by the developer in the config.mk for ease of use
 */
#define GPIO_OUTAMOUNT (CFG_GPIO_OUTAMOUNT)


/**
 * \def        GPIO_SUCCESS 
 * \brief      Define for the return of a successful gpio_read
 */
#define GPIO_SUCCESS 1

/**
 * \def        GPIO_FAILED 
 * \brief      Define for the return of a failed gpio_read
 */
#define GPIO_FAILED -1 

/**
 * \def        GPIO_HIGH 
 * \brief      Define for turning a GPIO OUTPUT pin to HIGH
 */
#define GPIO_HIGH 1
/**
 * \def        GPIO_LOW 
 * \brief      Define for turning a GPIO OUTPUT pin to LOW
 */
#define GPIO_LOW 0


/**
 * \fn        void gpio_init()
 * \brief     Initializes the GPIO Module
 * 
 * Initialisation of the GPIO Module
 * No functionality yet, can be added later if a setup for the GPIO Module is needed
 * 
 * \return     void
 */
void gpio_init();

/**
 * \fn        int8_t gpio_toogle_output(uint8_t outputpin)
 * \brief     Toogles the Output Pin
 * 
 * Toogles the given Output Pin specified in the function parameters. 
 * For help outputpin enumeration check the README in the <ParaNut directory>/sw/libgpio.h
 * 
 * \return    the Values GPIO_SUCCESS or GPIO_FAILED. GPIO_FAILED gets returned when the specified Output Pin is exceding the GPIO_OUTAMOUNT specified by the config.mk
 */
int8_t gpio_toogle_output(uint8_t outputpin);

/**
 * \fn        int8_t gpio_set_output(uint8_t outputpin, uint8_t value)
 * \brief     Sets the Output Pin
 * 
 * Sets the given Output Pin specified in the function parameter outputpin to the value specified by value, Compatible values are GPIO_HIGH and GPIO_LOW. 
 * For help on outputpin enumeration check the README in the <ParaNut directory>/sw/libgpio.h
 * 
 * \return    the Values GPIO_SUCCESS or GPIO_FAILED. GPIO_FAILED gets returned when the specified Output Pin is exceding the GPIO_OUTAMOUNT specified by the config.mk 
 * \return    or when a wrong value is received
 */
int8_t gpio_set_output(uint8_t outputpin, uint8_t value);

/**
 * \fn        int8_t gpio_get_output(uint8_t outputpin, uint8_t* out)
 * \brief     Gets the Output Pin
 * 
 * Gets the given Output Pin specified in the function parameter outputpin, the pointer out is used to access this value. 
 * For help on outputpin enumeration check the README in the <ParaNut directory>/sw/libgpio.h
 * 
 * \return    the Values GPIO_SUCCESS or GPIO_FAILED. GPIO_FAILED gets returned when the specified Output Pin is exceding the GPIO_OUTAMOUNT specified by the config.mk
 */
int8_t gpio_get_output(uint8_t outputpin, uint8_t* out);


/**
 * \fn        uint32_t gpio_get_output()
 * \brief     Returns the full gpio register
 * 
 * Returns the full gpio register
 * 
 * \return    uint32_t with the value of the full gpio_register;
 */
uint32_t gpio_read_register();

/**
 * \fn        int8_t gpio_get_input(uint8_t inputpin, uint8_t* out)
 * \brief     Gets the Input Pin
 * 
 * Reads the given Input Pin specified in the function parameter inputpin, the pointer out is used to access this value. 
 * For help on inputpin enumeration check the README in the <ParaNut directory>/sw/libgpio.h
 * 
 * \return    the Values GPIO_SUCCESS or GPIO_FAILED. GPIO_FAILED gets returned when the specified Output Pin is exceding the GPIO_INAMOUNT specified by the config.mk
 */
int8_t gpio_get_input(uint8_t inputpin, uint8_t* out);

/**
 * \fn        void gpio_done()
 * \brief     This function is used to cleanup the gpio
 * This function is used to cleanup the gpio module 
 * No functionality yet
 * can be added if needed
 * 
 * \return     void
 */
void gpio_done();



