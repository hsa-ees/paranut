/*************************************************************************

  This file is part of the lab excercises for "Entwurf digitaler Systeme 2".

  (C) 2020 Michael Schaeferling
           Hochschule Augsburg, University of Applied Sciences

  Software driver for the Analog Devices SSM2603 audio codec, 
  as used on the Zybo board.

  The SSM2603 audio codec is configured via I2C as follows:
    - Digital interface: I2S data format
    - Channels: 2
    - Resolution (per channel): 16bit
    - SampleRate: needs to be specified by the AUDIO_MODE setting. 
          The actual sample rate depends on the hardware module (this 
          generates MCLK, BCLK and LRCLK) but the SSM2603 also needs to
          get this information via I2S.

 *************************************************************************/


#ifndef SSM2603_H_
#define SSM2603_H_

#include "xstatus.h"
#include "xiicps.h"


/*
 * AUDIO_MODE:
 * 0: 48kHz
 * 1: 11.025kHz
 * */
#define AUDIO_MODE 0


/* SSM2603 IIC config */
#define SSM2603_IIC_ID XPAR_XIICPS_0_DEVICE_ID
#define SSM2603_IIC_SLAVE_ADDR  0b0011010

XIicPs SSM2603_iic;

XStatus SSM2603_init(XIicPs *iic_inst, u16 xiicps_inst_id);

#endif /* SSM2603_H_ */
