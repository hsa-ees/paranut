/*************************************************************************

  This file is part of the lab excercises for "Entwurf digitaler Systeme 2".

  (C) 2020 Michael Schaeferling
           Hochschule Augsburg, University of Applied Sciences

  Software driver for the Analog Devices SSM2603 audio codec, 
  as used on the Zybo board.

  See the header file for more information.

 *************************************************************************/


#include "ssm2603.h"


XStatus iic_reg_set(XIicPs *iic_inst, u8 dev_addr, u8 reg_addr, u16 reg_data){
  XStatus status;
  u8 SendBuffer[2];

  SendBuffer[0] = reg_addr << 1;
  SendBuffer[0] = SendBuffer[0] | ((reg_data >> 8) & 0b1);

  SendBuffer[1] = reg_data & 0xFF;

  status = XIicPs_MasterSendPolled(iic_inst, SendBuffer, 2, dev_addr);
  if (status != XST_SUCCESS) {
    xil_printf("IIC: send failed\n\r");
    return XST_FAILURE;
  }
  while (XIicPs_BusIsBusy(iic_inst)) {}

  return XST_SUCCESS;
}

XStatus SSM2603_init(XIicPs *iic_inst, u16 xiicps_inst_id){
  XStatus status;
  XIicPs_Config *xiicps_inst_config;

  xiicps_inst_config = XIicPs_LookupConfig(xiicps_inst_id);
  if (NULL == xiicps_inst_config) { return XST_FAILURE; }

  status = XIicPs_CfgInitialize(iic_inst, xiicps_inst_config, xiicps_inst_config->BaseAddress);
  if (status != XST_SUCCESS) { return XST_FAILURE; }

  /* Set 100kHz IIC clock */
  status = XIicPs_SetSClk(iic_inst, 100000);
  if (status != XST_SUCCESS) { return XST_FAILURE; }

  status  = iic_reg_set(iic_inst, SSM2603_IIC_SLAVE_ADDR, 0x15, 0b000000000); // Software Reset -> Reset=all 0
  //status |= iic_reg_set(iic_inst, SSM2603_IIC_SLAVE_ADDR, 0x06, 0b011111111); // Power Management -> all off
  //~ status |= iic_reg_set(iic_inst, SSM2603_IIC_SLAVE_ADDR, 0x02, 0b101100000); // Set volume: -25dB for left and right channel (dfl=0b101111001 = 0dB)
  status |= iic_reg_set(iic_inst, SSM2603_IIC_SLAVE_ADDR, 0x02, 0b101010000); // Set volume: -41dB for left and right channel (dfl=0b101111001 = 0dB)
  //~ status |= iic_reg_set(iic_inst, SSM2603_IIC_SLAVE_ADDR, 0x02, 0b101000000); // Set volume: -57dB for left and right channel (dfl=0b101111001 = 0dB)
  status |= iic_reg_set(iic_inst, SSM2603_IIC_SLAVE_ADDR, 0x04, 0b000010000); // Analog Out Path -> DACSEL=1
  status |= iic_reg_set(iic_inst, SSM2603_IIC_SLAVE_ADDR, 0x05, 0b000000000); // Digital Audio Path -> DACMU=0
  status |= iic_reg_set(iic_inst, SSM2603_IIC_SLAVE_ADDR, 0x07, 0b000000010); // Digital Audio IF -> 16bits, I2S format
  
  if (AUDIO_MODE == 0)
    status |= iic_reg_set(iic_inst, SSM2603_IIC_SLAVE_ADDR, 0x08, 0b000000000); // Sample Rate -> SampleRate ADC/DAC=48kHz  
  if (AUDIO_MODE == 1)
    status |= iic_reg_set(iic_inst, SSM2603_IIC_SLAVE_ADDR, 0x08, 0b000110000); // Sample Rate -> SampleRate ADC/DAC=11.025kHz
  
  status |= iic_reg_set(iic_inst, SSM2603_IIC_SLAVE_ADDR, 0x06, 0b001100111); // Power Management -> PWROFF=0; Out=0; DAC=0
  status |= iic_reg_set(iic_inst, SSM2603_IIC_SLAVE_ADDR, 0x09, 0b000000001); // Activate Digital Core -> Active=1

  if (status != XST_SUCCESS) {
    return XST_FAILURE;
  }
  return XST_SUCCESS;
}


