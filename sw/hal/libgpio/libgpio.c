/*************************************************************************

  This file is part of the ParaNut project.

  Copyright (C) 2023   Elias Schuler <elias.schuler@hs-augsburg.de>
                            
      Hochschule Augsburg, University of Applied Sciences

  Description:
     This File is the API to use the UART as a developer

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

#include "libgpio.h"


void gpio_init(){
   // If later on the gpio needs initialising you can add the initialisation code here
}

int8_t gpio_toogle_output(uint8_t outputpin){
  volatile uint32_t* gpio_toogle = (uint32_t*) GPIO_PINS;
  uint32_t gpio_result = *gpio_toogle;
  //checks to see if the given outputpin is valid
  if(outputpin <= GPIO_OUTAMOUNT && outputpin > 0){
    //Shifts a 1 to the outputpin specified by the function parameter and XORs the outputpin so only this outputpin gets changed
    gpio_result ^= (1<<(outputpin-1));
    *gpio_toogle = gpio_result;
    return GPIO_SUCCESS;
  }else{
    return GPIO_FAILED;
  }
}

int8_t gpio_set_output(uint8_t outputpin, uint8_t value){
  volatile uint32_t* gpio_set = (uint32_t*) GPIO_PINS;
  uint32_t gpio_result = *gpio_set;
  //checks to see if the given outputpin is valid
  if(outputpin <= GPIO_OUTAMOUNT && outputpin > 0){
    if (value == GPIO_HIGH){
      //Shifts a 1 to the outputpin specified by the function parameter and ORs the outputpin so only this outputpin gets changed
      gpio_result |= (1<<(outputpin-1));
      *gpio_set = gpio_result;
    }else if(value == GPIO_LOW){
      //Shifts a 0 to the outputpin specified by the function parameter and ORs the outputpin so only this outputpin gets changed
      gpio_result &= ~(1<<(outputpin-1));
      *gpio_set = gpio_result;
    }else{
      return GPIO_FAILED;
    }
    return GPIO_SUCCESS;
  }else{
    return GPIO_FAILED;
  }
}

int8_t gpio_get_output(uint8_t outputpin, uint8_t* out){
  volatile uint32_t* gpio_out = (uint32_t*) GPIO_PINS;
  uint32_t gpio_result = *gpio_out;
  //checks to see if the given outputpin is valid
  if(outputpin <= GPIO_OUTAMOUNT && outputpin > 0){
    // Shifts a 1 to the outputpin specified with the function parameter checks if it is active or not
    if (gpio_result & (1<<(outputpin-1))) {
     *out = 1;
    } else {
     *out = 0;
    }
    return GPIO_SUCCESS;
  }else{
    return GPIO_FAILED;
  }
}

// Reads the entire 32bit register of the GPIO Module
uint32_t gpio_read_register(){
    volatile uint32_t* gpio_register = (uint32_t*) GPIO_PINS;
    uint32_t data;
    data = *(gpio_register);
    return data;
}

// Gets the input on the given inputpin 
int8_t gpio_get_input(uint8_t inputpin, uint8_t* out){
  volatile uint32_t* gpio_in = (uint32_t*) GPIO_PINS;
  uint32_t gpio_result = *gpio_in;
  //checks to see if the given inputpin is valid
  if(inputpin <= GPIO_INAMOUNT && inputpin > 0){
    // Shifts a 1 to the pin specified with the function parameter + the offset of the GPIO_OUTAMOUNT and checks if it is active or not
    if (gpio_result & (1<<(inputpin+GPIO_OUTAMOUNT-1))) {
     *out = 1;
    } else {
     *out = 0;
    }
    return GPIO_SUCCESS;
  }else{
    return GPIO_FAILED;
  }
}


// This function is needed to cleanup the gpio 
// No functionality yet
void gpio_done(){
    // If later on the gpio needs cleaning you can add the cleanup code here
}