/*************************************************************************

  This file is part of the ParaNut project.

  Copyright (C) 2022-2023 Elias Schuler <elias.schuler@hs-augsburg.de>
                2022-2023 Lukas Bauer <lukas.bauer@hs-ausgsburg.de>
      Hochschule Augsburg, University of Applied Sciences

    Description : This file is based on the work of Sebastian Witt
                  UART Receiver

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
#include "uart_receiver.h"



void UartReceiver::InitSubModules(){

    iNoConnect4 = 0;
    iNoConnect = 0;

    rx_brc.clk_i (clk_i);
    rx_brc.rst_i (rst_i);
    rx_brc.clear_i (iBaudCountClear);
    rx_brc.load_i (iNoConnect);
    rx_brc.enable_i (rx_clk_i);
    rx_brc.down_i (iNoConnect);
    rx_brc.d_i(iNoConnect4);
    rx_brc.q_o (iBaudCount);
    rx_brc.overflow_o (iBaudStep);


    rxmvf.clk_i (clk_i);
    rxmvf.rst_i (rst_i);
    rxmvf.sample_i (rx_clk_i);
    rxmvf.clear_i (iFilterClear);
    rxmvf.d_i (sin_i);
    rxmvf.q_o (iFSIN);

    rx_isfsb.clk_i (clk_i);
    rx_isfsb.rst_i (rst_i);
    rx_isfsb.ce_i (rx_clk_i);
    rx_isfsb.d_i (sin_i);
    rx_isfsb.q_o (iFStopBit);

}
#ifndef __SYNTHESIS__
void UartReceiver::Trace(sc_trace_file *tf, int level){
    if (!tf || pn_trace_verbose) printf ("\nSignals of Module \"%s\":\n", name ());

    rx_brc.Trace(tf, level);
    rx_isfsb.Trace(tf, level);
    rxmvf.Trace(tf, level);

    //PN_TRACE
    PN_TRACE(tf, clk_i);    // Clock
    PN_TRACE(tf, rst_i);    // Reset
    PN_TRACE(tf, rx_clk_i); // Receiver clock(16x baudrate)
    PN_TRACE(tf, rx_clear_i);   // Reset receiver state
    PN_TRACE(tf, wls_i);    // Word length select
    PN_TRACE(tf, stb_i);    // Number of stop bits
    PN_TRACE(tf, pen_i);    // Parity enable
    PN_TRACE(tf, eps_i);    // Even parity select
    PN_TRACE(tf, sp_i); // Stick parity
    PN_TRACE(tf, sin_i);    // Receiver input
    PN_TRACE(tf, pe_o); // Parity error
    PN_TRACE(tf, fe_o); // Framing error
    PN_TRACE(tf, bi_o); // Break interrupt
    PN_TRACE(tf, dout_o);   // Output data
    PN_TRACE(tf, rx_finished_o);    // Receiver operation finished

    PN_TRACE(tf, iBaudCount);   // Baud counter output
    PN_TRACE(tf, iBaudCountClear);  // Baud counter clear
    PN_TRACE(tf, iBaudStep);    // Next symbol pulse
    PN_TRACE(tf, iBaudStepD);   //Next symbol pulse delayed by one clock
    PN_TRACE(tf, iFilterClear); // Reset input filter
    PN_TRACE(tf, iFSIN);    // Filtered SIN
    PN_TRACE(tf, iFStopBit);    // Filtered SIN for stop bit detection
    PN_TRACE(tf, iParity);  // Data parity
    PN_TRACE(tf, iParityReceived);  // Parity received
    PN_TRACE(tf, iDataCount);   // Data bit counter
    PN_TRACE(tf, iDataCountInit);   // Initialized data bit counter to word length
    PN_TRACE(tf, iDataCountFinish); // Data bit counter finished
    PN_TRACE(tf, iRXFinished);  // Word received, output data valid
    PN_TRACE(tf, iFE);  // Internal frame error
    PN_TRACE(tf, iBI);  // Internal break interrupt
    PN_TRACE(tf, iNoStopReceived);  // No valid stop bit received
    PN_TRACE(tf, iDOUT);    // Data output
    PN_TRACE(tf, iRxState); //Rx State
    PN_TRACE(tf, iRxNextState); //Rx Next State

}
#endif

void UartReceiver::OutputMethod(){


    if((wls_i.read() == 0 && iDataCount.read() == 5) ||
        (wls_i.read() == 1 && iDataCount.read() == 6 )||
        (wls_i.read() == 2 && iDataCount.read() == 7)||
        (wls_i.read() == 3 && iDataCount.read() == 8)){
            iDataCountFinish = 1;
    }else{
        iDataCountFinish = 0;
    }

    if(iFStopBit.read() == 0 && iRxState.read() == STOP){
        iNoStopReceived = 1;
    } else {
        iNoStopReceived = 0;
    }
    if(iDOUT.read() == 0 && iParityReceived.read() == 0 && iNoStopReceived.read() == 1){
        iBI = 1;
    }else{
        iBI = 0;
    }
    if(iNoStopReceived.read() == 1){
        iFE = 1;
    }else{
        iFE = 0;
    }
    iFilterClear = (iBaudStepD.read() == 1) || (iBaudCountClear.read() == 1);
    dout_o = iDOUT;
    bi_o = iBI;
    fe_o = iFE;
    rx_finished_o = iRXFinished;
}

void UartReceiver::ParityGenerationMethod(){


    sc_uint<8> iDoutInternal = iDOUT.read();
    iParity = iDoutInternal[7] ^ iDoutInternal[6] ^ iDoutInternal[5] ^ iDoutInternal[4] ^ iDoutInternal[3] ^ iDoutInternal[2] ^ iDoutInternal[1] ^ iDoutInternal[0] ^ !eps_i.read();
}

void UartReceiver::StateMachineMethod(){

    iRxNextState = IDLE;
    iBaudCountClear = 0;
    iDataCountInit = 0;
    iRXFinished = 0;
    sc_uint<4> iBaudCountInternal = iBaudCount;

    switch (iRxState.read())
    {
    case IDLE:
        if(sin_i.read() == 0){
            iRxNextState = START;
        }
        iBaudCountClear = 1;
        iDataCountInit = 1;
        break;
    case START:
        iDataCountInit = 1;
        if(iBaudStep.read() == 1){
            if(iFSIN.read() == 0){
                iRxNextState = DATA;
            }
        }else{
            iRxNextState = START;
        }
        break;
    case DATA:
        if(iDataCountFinish.read() == 1){
            if(pen_i == 1){
                iRxNextState = PAR;
            }else{
                iRxNextState = STOP;
            }
        }else{
            iRxNextState = DATA;
        }
        break;
    case PAR:
        if(iBaudStep.read() == 1){
            iRxNextState = STOP;
        }else{
            iRxNextState = PAR;
        }
        break;
    case STOP:
        if(iBaudCountInternal[3] == 1){
            if(iFStopBit == 0){
                iRXFinished = 1;
                iRxNextState = MWAIT;
            }else{
                iRXFinished = 1;
                iRxNextState = IDLE;
            }
        }else{
            iRxNextState = STOP;
        }
        break;
    case MWAIT:
        if(sin_i == 0){
            iRxNextState = MWAIT;
        }
    default:
        break;
    }
}

void UartReceiver::BaudstepDMethod(){


    iBaudStepD = 0;

    wait();

    while (true){

        iBaudStepD = iBaudStep;

        wait();

    }

}

void UartReceiver::FSMUpdateMethod(){


    iRxState = IDLE;

    wait();

    while (true){

        iRxState = iRxNextState;

        wait();

    }
}

void UartReceiver::CheckParityMethod(){

    pe_o = 0;
    iParityReceived = 0;

    wait();

    while(true){

        if (iRxState.read() == PAR && iBaudStep.read() == 1){
            iParityReceived = iFSIN;
        }

        if(pen_i.read() == 1){
            pe_o = 0;
            if(sp_i.read() == 1){
                if((eps_i.read() ^ iParityReceived.read()) == 0){
                    pe_o = 1;
                }
            }else{
                if(iParity.read() != iParityReceived.read()){
                    pe_o = 1;
                }
            }
        }else{
            pe_o = 0;
            iParityReceived = 0;
        }

        wait();
    }
}

void UartReceiver::DataBitCaptureMethod(){

    iDataCount = 0;
    iDOUT = 0;

    wait();

    while (true){

        if(iDataCountInit.read() == 1){

            iDataCount = 0;
            iDOUT = 0;

        }else{

            if(iBaudStep.read() == 1 && iDataCountFinish.read() == 0){
                // fprintf(stderr,"\niBaudstep is:\t %d iDataCountFinish is:\t %d\n",iBaudStep.read(),iDataCountFinish.read());
                sc_uint<8> iDOUT_internal = iDOUT;
                iDOUT_internal[iDataCount.read()] = iFSIN.read();
                iDataCount = (iDataCount.read() + 1);
                iDOUT = iDOUT_internal;

            }
        }

        wait();
    }
}

