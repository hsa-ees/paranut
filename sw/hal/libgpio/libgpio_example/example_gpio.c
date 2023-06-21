/*************************************************************************

  This file is part of the ParaNut project.

  Copyright (C) 2023 Elias Schuler <elias.schuler@hs-augsburg.de>
        Augsburg, University of Applied Sciences

  Description:
     Demo application showcasing the functionality of the GPIO-Module

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

#include <libgpio.h>


int main () {

   uint8_t input = 0;
   uint8_t output = 0;
   while (1){
      // Reads in all the Input Pins and returns their value
      for(size_t i = 1; i <= GPIO_INAMOUNT; i++){
         if(gpio_get_input(i, &input) == GPIO_SUCCESS){
            printf("Input Pin %d is %d\n", i, input);
         }else{
            printf("Input Pin is not available\n");
         }
      }
      // Toogles all the Outputs on and prints the status of the pins and the register
      for(size_t i = 1; i <= GPIO_OUTAMOUNT; i++){
         if(gpio_toogle_output(i)){
            if(gpio_get_output(i, &output) == GPIO_SUCCESS){
               printf("Output Pin using toggleGPIO %d is %d\n", i, output);
               uint32_t data = gpio_read_register();
               printf("Register is now 0x%08x\n", data);
            }else{
               printf("Output Pin is not available\n");
            }
         }else{
            printf("Output Pin is not available\n");
         }
         for(size_t j = 0; j < 1000000; j++);
      }
      // Sets all the Outputs to GPIO_LOW and prints the status of the pins and the register
      for(size_t i = 1; i <= GPIO_OUTAMOUNT; i++){
         if(gpio_set_output(i, GPIO_LOW)){
            if(gpio_get_output(i, &output) == GPIO_SUCCESS){
               printf("Output Pin using set GPIO_LOW %d is %d\n", i, output);
               uint32_t data = gpio_read_register();
               printf("Register is now 0x%08x\n", data);
            }else{
               printf("Output Pin is not available\n");
            }
         }else{
            printf("Output Pin is not available\n");
         }
         //wait for 1000000 paranut cycles so the user can read the console output and see the output on salea better
         for(size_t j = 0; j < 1000000; j++);
      }
      // Sets all the Outputs to GPIO_HIGH and prints the status of the pins and the register
      for(size_t i = 1; i <= GPIO_OUTAMOUNT; i++){
         if(gpio_set_output(i, GPIO_HIGH)){
            if(gpio_get_output(i, &output) == GPIO_SUCCESS){
               printf("Output Pin using set GPIO_HIGH %d is %d\n", i, output);
               uint32_t data = gpio_read_register();
               printf("Register is now 0x%08x\n", data);
            }else{
               printf("Output Pin is not available\n");
            }
         }else{
            printf("Output Pin is not available\n");
         }
         //wait for 1000000 paranut cycles so the user can read the console output and see the output on salea better
         for(size_t j = 0; j < 1000000; j++);
      }
      uint32_t data = gpio_read_register();
      printf("Register is now 0x%08x\n", data);
      //wait for 1000000 paranut cycles so the user can read the console output and see the output on salea better
      for(size_t i = 0; i < 1000000; i++);
   }

   return 0;
}