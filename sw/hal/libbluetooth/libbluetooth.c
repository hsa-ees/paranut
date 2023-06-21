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

#include "libbluetooth.h"


uint8_t inputPinNumberStatusPin1;
uint8_t inputPinNumberStatusPin2;
uint8_t outputPinNumberWakeUpPin;


int8_t bluetooth_init(uint8_t statusPin1, uint8_t statusPin2, uint8_t wakeUpPin){
  gpio_init();
  struct uart_setup uart_setup;
  uart_setup.baudrate = BAUDRATE_115200;
  uart_setup.wordLength = 8;
  uart_setup.fifo64ByteMode = 1;
  uart_init(uart_setup);
  inputPinNumberStatusPin1 = statusPin1;
  inputPinNumberStatusPin2 = statusPin2;
  outputPinNumberWakeUpPin = wakeUpPin;
  if(gpio_set_output(outputPinNumberWakeUpPin, GPIO_HIGH) == GPIO_SUCCESS){
    return BLUETOOTH_SUCCESS;
  }else{
    return BLUETOOTH_GPIO_ERROR;
  }
}

int8_t bluetooth_get_inputPinNumberStatusPin1(){
  return inputPinNumberStatusPin1;
}
int8_t bluetooth_get_inputPinNumberStatusPin2(){
  return inputPinNumberStatusPin2;
}
int8_t bluetooth_get_outputPinNumberWakeUpPin(){
  return outputPinNumberWakeUpPin;
}

int8_t bluetooth_get_status(){
  uint8_t status1;
  uint8_t status2;

  if(gpio_get_input(inputPinNumberStatusPin1, &status1) == GPIO_SUCCESS){
    if(gpio_get_input(inputPinNumberStatusPin2, &status2) == GPIO_SUCCESS){
      if(status1 == GPIO_HIGH){
        if(status2 == GPIO_HIGH){
          return BLUETOOTH_CONNECTED_MODE;
        } else if (status2 == GPIO_LOW){
          return BLUETOOTH_STANDBYE_MODE;
        } else {
          return BLUETOOTH_STATUS_ERROR;
        }
      } else if (status1 == GPIO_LOW){
        if(status2 == GPIO_HIGH){
          return BLUETOOTH_TRANSPARENT_UART_READY;
        } else if (status2 == GPIO_LOW){
          return BLUETOOTH_SHUTDOWN_MODE;
        } else {
          return BLUETOOTH_STATUS_ERROR;
        }
      } else {
        return BLUETOOTH_STATUS_ERROR;
      }
    } else {
      return BLUETOOTH_GPIO_ERROR;
    }
  } else {
    return BLUETOOTH_GPIO_ERROR;
  }
}

int8_t bluetooth_wait_status(int8_t statusToReach, int64_t uSecIntervall, uint32_t timeOutAttempts){
  uint8_t bluetoothConnectionStatus = bluetooth_get_status();
  int attemptCounter = 0;
  //checks if the status is reached and if the uSecTimeOut has been reached if it's set
  while(statusToReach != bluetoothConnectionStatus){
    if((timeOutAttempts <= attemptCounter && timeOutAttempts != 0)){
      return BLUETOOTH_TIME_OUT;
    } else if(bluetoothConnectionStatus == BLUETOOTH_SHUTDOWN_MODE){
      if(bluetooth_wake_up() == BLUETOOTH_SUCCESS){
        pn_usleep(uSecIntervall);
      } else {
        return BLUETOOTH_GPIO_ERROR;
      }
    } else if (bluetoothConnectionStatus == BLUETOOTH_STATUS_ERROR){
      return BLUETOOTH_STATUS_ERROR;
    } else {
      pn_usleep(uSecIntervall);
    }
    attemptCounter++;
    bluetoothConnectionStatus = bluetooth_get_status();
  }
  return BLUETOOTH_POLL_REACHED;
}

int8_t bluetooth_send_data(int dataLength, char* transmitString){
  if(bluetooth_get_status() == BLUETOOTH_TRANSPARENT_UART_READY){
    for(int i = 0; i < dataLength; i++){
      uart_send_char(transmitString[i]);
    }
    return BLUETOOTH_SUCCESS;
  } else {
    return BLUETOOTH_STATUS_ERROR;
  }
};

int8_t bluetooth_data_ready(){
  if(uart_data_ready() == UART_SUCCESS){
    return BLUETOOTH_DATA;
  } else {
    return BLUETOOTH_NO_DATA;
  }
}

int8_t bluetooth_receive_data(int dataLength, char* receivedString, uint8_t blockingEnabled, uint32_t  timeOutAttempts){
  char outChar;
  char receiveCharArr[dataLength]; 
  int charCounter = 0;
  if(blockingEnabled == 1){
    if(bluetooth_get_status() == BLUETOOTH_TRANSPARENT_UART_READY){
      // Blocking Receive and checks if we are still in BLUETOOTH_TRANSPARENT_UART_READY after each read, waiting for timeOutAttempts and receives until the dataLength is reached or we receive a Terminatorcharacter
      while(uart_receive_char(1, &outChar, timeOutAttempts) == UART_SUCCESS && ((charCounter < dataLength) && (outChar != '\n'))&& bluetooth_get_status() == BLUETOOTH_TRANSPARENT_UART_READY){
        receiveCharArr[charCounter] = outChar;
        charCounter++;
      }
      if(charCounter > 0){
        strncpy(receivedString, receiveCharArr, charCounter);
        return charCounter;
      } else {
        return charCounter;
      }
    } else {
      return charCounter; 
    }
  } else {
    if(bluetooth_get_status() == BLUETOOTH_TRANSPARENT_UART_READY){
      while(uart_receive_char(0, &outChar, timeOutAttempts) == UART_SUCCESS && ((charCounter < dataLength) && (outChar != '\n'))&& bluetooth_get_status() == BLUETOOTH_TRANSPARENT_UART_READY){
        receiveCharArr[charCounter] = outChar;
        charCounter++;
        //Sleeps for 20 usecs so if we are in the process of receiving we actually get the whole message
        pn_usleep(20);
      }
      if(charCounter > 0){
        strncpy(receivedString, receiveCharArr, charCounter);
        return charCounter;
      } else {
        return charCounter;
      }
    } else {
      return charCounter; 
    }
  }
}

int8_t bluetooth_wake_up(){
  if(gpio_set_output(outputPinNumberWakeUpPin, GPIO_LOW) == GPIO_SUCCESS){
    pn_usleep(10000);
    if(gpio_set_output(outputPinNumberWakeUpPin, GPIO_HIGH) == GPIO_SUCCESS){
      //Wait for 33ms+10ms so the board has reached an Active State again
      pn_usleep(33000);
      return BLUETOOTH_SUCCESS;
    } else {
      return BLUETOOTH_GPIO_ERROR;
    }
  } else {
    return BLUETOOTH_GPIO_ERROR;
  }
}


int8_t bluetooth_done(){
  gpio_done();
  uart_done();
  if(gpio_set_output(outputPinNumberWakeUpPin, GPIO_LOW) == GPIO_SUCCESS){
    return BLUETOOTH_SUCCESS;
  }else{
    return BLUETOOTH_GPIO_ERROR;
  }
}
