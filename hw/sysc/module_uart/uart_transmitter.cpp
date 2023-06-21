/*************************************************************************

  This file is part of the ParaNut project.

  Copyright (C) 2022-2023 Elias Schuler <elias.schuler@hs-augsburg.de>
                2022-2023 Lukas Bauer <lukas.bauer@hs-ausgsburg.de>
      Hochschule Augsburg, University of Applied Sciences

    Description : This file is based on the work of Sebastian Witt
                  UART Transmitter

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
#include "uart_transmitter.h"



#ifndef __SYNTHESIS__
void UartTransmitter::Trace(sc_trace_file *tf, int level){
    if (!tf || pn_trace_verbose) printf ("\nSignals of Module \"%s\":\n", name ());

    //PN_TRACE
    PN_TRACE(tf, clk_i);
    PN_TRACE(tf, rst_i);
    PN_TRACE(tf, tx_clk_i);
    PN_TRACE(tf, tx_start_i);
    PN_TRACE(tf, clear_i);
    PN_TRACE(tf, wls_i);
    PN_TRACE(tf, stb_i);
    PN_TRACE(tf, pen_i);
    PN_TRACE(tf, eps_i);
    PN_TRACE(tf, sp_i);
    PN_TRACE(tf, bc_i);
    PN_TRACE(tf, din_i);
    PN_TRACE(tf, tx_finished_o);
    PN_TRACE(tf, sout_o);

    PN_TRACE(tf, iTx2);
    PN_TRACE(tf, iSout);
    PN_TRACE(tf, iParity);
    PN_TRACE(tf, iFinished);
    PN_TRACE(tf, tx_state);
    PN_TRACE(tf, next_tx_state);
}
#endif

void UartTransmitter::OutputMethod(){

    if (bc_i.read() == 0){

        sout_o = iSout.read();

    }else{

        sout_o = 0;
    }

    tx_finished_o = iFinished.read();


}

void UartTransmitter::TXMethod(){

    tx_state = IDLE;
    iTx2 = 0;

    wait();

    while(true){

        if(tx_clk_i.read() == 1){                       // Tx clock
            if(iTx2.read() == 0){                       //Two TX clocks per step

                tx_state = next_tx_state.read();        // Next step
                iTx2 = 1;

            }else{

                if((wls_i.read() == 0) && (stb_i.read() == 1) && (tx_state.read() == STOP2)){

                    tx_state = next_tx_state.read();    // 1.5 stop bits for 5 bit word mode
                    iTx2 = 1;

                }else{

                    tx_state = tx_state.read();     // First Tx clock,wait
                    iTx2 = 0;

                }
            }
        }

        wait();

    }

}

void UartTransmitter::TX_FMSMethod(){

    sc_uint<8> din_internal = din_i.read();
    next_tx_state = IDLE;
    iSout  = 1;

    switch (tx_state.read()){

        case IDLE:

            if(tx_start_i.read() == 1){

                next_tx_state = START;

            }
            break;

        case START:

            iSout = 0;
            next_tx_state = BIT0;
            break;

        case BIT0:

            iSout = din_internal[0];
            next_tx_state = BIT1;
            break;

        case BIT1:

            iSout = din_internal[1];
            next_tx_state = BIT2;
            break;

        case BIT2:

            iSout = din_internal[2];
            next_tx_state = BIT3;
            break;

        case BIT3:

            iSout = din_internal[3];
            next_tx_state = BIT4;
            break;

        case BIT4:

            iSout = din_internal[4];

            if(wls_i.read() == 0){          // 5 bits

                if(pen_i.read() == 1){

                    next_tx_state = PAR;    // Parity enabled

                }else{

                    next_tx_state = STOP;   // No Parity

                }

            }else{

                next_tx_state = BIT5;

            }

            break;

        case BIT5:

            iSout = din_internal[5];

            if(wls_i.read() == 1){          // 6 bits

                if(pen_i.read() == 1){

                    next_tx_state = PAR;    // Parity enabled

                }else{

                    next_tx_state = STOP;   // No Parity

                }

            }else{

                next_tx_state = BIT6;

            }

            break;

        case BIT6:

            iSout = din_internal[6];

            if(wls_i.read() == 2){          // 7 bits

                if(pen_i.read() == 1){

                    next_tx_state = PAR;    // Parity enabled

                }else{

                    next_tx_state = STOP;   // No Parity

                }

            }else{

                next_tx_state = BIT7;

            }

            break;

        case BIT7:

            iSout = din_internal[7];

            if(pen_i.read() == 1){

                next_tx_state = PAR;    // Parity enabled

            }else{

                next_tx_state = STOP;   // No Parity

            }

            break;

        case PAR:

            if(sp_i.read() == 1){           // Sticky parity

                if(eps_i.read() == 1){

                    iSout = 0;              // Even parity -> cleared

                }else{

                    iSout = 1;              // Odd parity -> set

                }

            }else{

                if (eps_i.read() == 1){

                    iSout = iParity.read(); // Even parity

                }else{

                    iSout = !iParity.read();// Odd parity

                }
            }

            next_tx_state = STOP;

            break;

        case STOP:

            if(stb_i.read() == 1){

                next_tx_state = STOP2;          // 2 stop bits

            }else{

                if(tx_start_i.read() == 1){

                    next_tx_state = START;      // Next transmission

                }

            }

            break;

        case STOP2:

            if(tx_start_i.read() == 1){

                next_tx_state = START;        // Next transmission

            }

        break;

        default:

            break;
    }

}

void UartTransmitter::TX_PARMethod(){

    bool iP40, iP50, iP60, iP70;
    sc_uint<8> din_internal = din_i.read();


    iP40 = din_internal[4] ^ din_internal[3] ^ din_internal [2] ^ din_internal[1] ^ din_internal[0];
    iP50 = din_internal[5] ^ iP40;
    iP60 = din_internal[6] ^ iP50;
    iP70 = din_internal[7] ^ iP60;

    switch (wls_i.read()){

        case 0:

            iParity = iP40;
            break;

        case 1:

            iParity = iP50;
            break;

        case 2:

            iParity = iP60;
            break;

        default:

            iParity = iP70;
            break;

    }

}

void UartTransmitter::TX_FINMethod(){

    bool iLast;

    iFinished = 0;
    iLast = 0;

    wait();

    while(true){

        iFinished = 0;

        if(iLast == 0 && tx_state.read() == STOP){

            iFinished = 1;
        }

        if(tx_state.read() == STOP){

            iLast = 1;

        }else{

            iLast = 0;
        }


        wait();
    }

}

