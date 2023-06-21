/*************************************************************************

  This file is part of the ParaNut project.

  Copyright (C) 2022-2023 Elias Schuler <elias.schuler@hs-augsburg.de>
                2022-2023 Lukas Bauer <lukas.bauer@hs-ausgsburg.de>
      Hochschule Augsburg, University of Applied Sciences

    Description : This file is based on the work of Sebastian Witt
                  UART Interrupt Control

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

 **************************************************************************/

#include "uart_interrupt.h"



#ifndef __SYNTHESIS__
void UartInterrupt::Trace(sc_trace_file *tf, int level){
    if (!tf || pn_trace_verbose) printf ("\nSignals of Module \"%s\":\n", name ());

    //PN_TRACE
    PN_TRACE(tf, clk_i);
    PN_TRACE(tf, rst_i);
    PN_TRACE(tf, ire_i);
    PN_TRACE(tf, lsr_i);
    PN_TRACE(tf, thi_i);
    PN_TRACE(tf, rda_i);
    PN_TRACE(tf, cti_i);
    PN_TRACE(tf, afe_i);
    PN_TRACE(tf, msr_i);
    PN_TRACE(tf, iir_o);
    PN_TRACE(tf, int_o);

    PN_TRACE(tf, RLSInterrupt);
    PN_TRACE(tf, RDAInterrupt);
    PN_TRACE(tf, CTIInterrupt);
    PN_TRACE(tf, THRInterrupt);
    PN_TRACE(tf, MSRInterrupt);
    PN_TRACE(tf, IIR);
}
#endif

void UartInterrupt::OutputMethod(){

    //internal variables
    sc_uint<4> ire_internal = ire_i.read();
    sc_uint<5> lsr_internal = lsr_i.read();
    sc_uint<4> msr_internal = msr_i.read();
    sc_uint<4> iir_internal = IIR.read();

    // Priority 1: Receiver line status interrupt on: Overrun error, parity error, framing error or break interrupt
    if(ire_internal[2] == 1 && (lsr_internal[1] == 1 || lsr_internal[2] == 1 || lsr_internal[3] == 1 || lsr_internal[4] == 1)){

        RLSInterrupt = 1;

    } else {

        RLSInterrupt = 0;
    }

    // Priority 2: Received data available or trigger level reached in FIFO mode
    if(ire_internal[2] == 1 && rda_i.read() == 1){

        RDAInterrupt = 1;

    } else {

        RDAInterrupt = 0;
    }

    // Priority 2: Character timeout indication
    if(ire_internal[0] == 1 && cti_i.read() == 1){

        CTIInterrupt = 1;

    } else {

        CTIInterrupt = 0;
    }

    // Priority 3: Transmitter holding register empty
    if(ire_internal[1] == 1 && thi_i.read() == 1){

        THRInterrupt = 1;
    } else {

        THRInterrupt = 0;
    }

    // Priority 4: Modem status interrupt: dCTS (when AFC is disabled), dDSR, TERI, dDCD

    if(ire_internal[3] == 1 && ((msr_internal[0] == 1 && afe_i.read() == 0) || msr_internal[1] == 1 || msr_internal[2] == 1 || msr_internal[3] == 1)){

        MSRInterrupt = 1;

    } else {

        MSRInterrupt = 0;
    }

    // Outputs

    iir_o = iir_internal;
    int_o = !iir_internal[0];

}

void UartInterrupt::InterruptMethod(){

    IIR = 0x1; // 0001

    wait();

    while(true){

        if(RLSInterrupt.read() == 1){

            IIR = 0x6; // 0110

        }else if (CTIInterrupt.read() == 1){

            IIR = 0xC; // 1100

        }else if (RDAInterrupt.read() == 1){

            IIR = 0x4; // 0100

        }else if (THRInterrupt.read() == 1){

            IIR = 0x2; // 0010

        }else if (MSRInterrupt.read() == 1){

            IIR = 0x0; // 0000

        }else{

            IIR = 0x1; // 0001
        }

        wait();

    }

}

