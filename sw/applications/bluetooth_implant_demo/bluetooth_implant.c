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
#include <string.h>

#include <libbluetooth.h>


int main () {
   printf("\nWelcome to the ParaNut bluetooth demo\n----------------------------------\n\n");
   char command[4];
   char output[50];
   uint8_t sensor_value = 0;
   uint8_t old_sensor_value = 0;
   printf("Initialising the bluetooth module \n");
   if(bluetooth_init(1,2,5) == BLUETOOTH_SUCCESS){
      while (1){   
         printf("Waiting for connection to the bluetooth module\n");
         if(bluetooth_wait_status(BLUETOOTH_TRANSPARENT_UART_READY, 2000, 0) == BLUETOOTH_POLL_REACHED){
            old_sensor_value = sensor_value;
            if(gpio_get_input(3, &sensor_value)){
               if(sensor_value != old_sensor_value){
                  sprintf(output, "Sensor value updated: %d", sensor_value);
                  if(bluetooth_send_data(23, output) == BLUETOOTH_SUCCESS){
                     printf("Kommunikation finished send updated sensor value \n");
                  } else {
                     printf("Oops something went wrong\n");
                  }
               }else if(bluetooth_receive_data(3, command, 1, 10000000) > 0){
                  if(!strncmp(command, "$S1", 3)){
                     sprintf(output, "Sensor Value: %d", sensor_value);
                     if(bluetooth_send_data(15, output) == BLUETOOTH_SUCCESS){
                        printf("Kommunikation finished with Sensorrequest\n");
                     } else {
                        printf("Oops something went wrong\n");
                     }
                  } else if (!strncmp(command, "$PI", 3)) {
                     sprintf(output, "This is the ParaNut Implant Demo");
                     if(bluetooth_send_data(32, output) == BLUETOOTH_SUCCESS){
                        printf("Kommunikation finished with ParaNut Info\n");
                     } else {
                        printf("Oops something went wrong\n");
                     }
                  } else {
                     if(bluetooth_send_data(27, "Please send a valid command") == BLUETOOTH_SUCCESS){
                        printf("Kommunikation finished invalid command \n");
                     } else {
                        printf("Oops something went wrong\n");
                     }
                  }
               } else {
                  printf("Timeout attempts reached\n");
               }
            }
         } else {
            printf("Timed Out\n");
         }
      }
   }else{
      printf("Bitte geben sie valide GPIO Pins in der bluetooth_init an\n");
   }
}