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

#ifndef _UART_RECIEVER_
#define _UART_RECIEVER_

#include "base.h"
#include "slib_mv_filter.h"
#include "slib_input_filter.h"
#include "slib_counter.h"

#include <systemc.h>

// **************** Defines *************
// **************** DM Register Addresses *************
// **************** Module *************

class UartReceiver: ::sc_core::sc_module{
public:
    //Ports (Wishbone)
    sc_in_clk clk_i  {"clk_i"};    // Clock
    sc_in<bool> rst_i  {"rst_i"};  // Reset
    sc_in<bool> rx_clk_i  {"rx_clk_i"}; // Receiver clock(16x baudrate)
    sc_in<bool> rx_clear_i  {"rx_clear_i"}; // Reset receiver state
    sc_in<sc_uint<2> > wls_i  {"wls_i"}; // Word length select
    sc_in<bool> stb_i  {"stb_i"}; // Number of stop bits
    sc_in<bool> pen_i  {"pen_i"}; // Parity enable
    sc_in<bool> eps_i  {"eps_i"}; // Even parity select
    sc_in<bool> sp_i  {"sp_i"}; // Stick parity
    sc_in<bool> sin_i  {"sin_i"}; // Receiver input
    sc_out<bool> pe_o  {"pe_o"}; // Parity error
    sc_out<bool> fe_o   {"fe_o"}; // Framing error
    sc_out<bool> bi_o   {"bi_o"}; // Break interrupt
    sc_out<sc_uint<8> > dout_o   {"dout_o"}; // Output data
    sc_out<bool> rx_finished_o   {"rx_finished_o"}; // Receiver operation finished



    // **************** Receiver States ****************

    typedef enum {
        IDLE,
        START,
        DATA,
        PAR,
        STOP,
        MWAIT,
    } RxStates;


    //Ports
    //Constructor
    //Functions
    SC_HAS_PROCESS (UartReceiver);
    UartReceiver (sc_module_name name)
        : sc_module (name) {
        InitSubModules();
        SC_METHOD(OutputMethod);
        sensitive << wls_i << iDataCount << iFStopBit << iRxState << iDOUT << iParityReceived << iNoStopReceived << iBI << iFE << iRXFinished << iBaudStepD << iBaudCountClear;
        SC_METHOD(ParityGenerationMethod);
        sensitive << iDOUT << eps_i;
        SC_METHOD(StateMachineMethod);
        sensitive << iRxState << sin_i << iFSIN << iFStopBit << iBaudStep << iBaudCount << iDataCountFinish << pen_i << wls_i << stb_i;
        SC_CTHREAD (BaudstepDMethod, clk_i.pos());
            reset_signal_is(rst_i, true);
        SC_CTHREAD (DataBitCaptureMethod, clk_i.pos());
            reset_signal_is(rst_i, true);
        SC_CTHREAD (FSMUpdateMethod, clk_i.pos());
            reset_signal_is(rst_i,true);
        SC_CTHREAD ( CheckParityMethod, clk_i.pos());
            reset_signal_is(rst_i, true);
    };
    void Trace (sc_trace_file *tf, int levels = 1);
#ifndef __SYNTHESIS__
#endif
    //Submodules
    SlibCounter rx_brc{"rx_brc"};
    SlibMVFilter rxmvf{"rxmvf"};
    SlibInputFilter rx_isfsb{"rx_isfsb"};

    //Processes
    void InitSubModules();
    void BaudstepDMethod();
    void DataBitCaptureMethod();
    void FSMUpdateMethod();
    void CheckParityMethod();
    void OutputMethod();
    void ParityGenerationMethod();
    void StateMachineMethod();

protected:
    //Registerss
    sc_signal<sc_uint<4> > iBaudCount  {"iBaudCount"}; // Baud counter output
    sc_signal<bool> iBaudCountClear  {"iBaudCountClear"}; // Baud counter clear
    sc_signal<bool> iBaudStep  {"iBaudStep"}; // Next symbol pulse
    sc_signal<bool> iBaudStepD  {"iBaudStepD"}; // Next symbol pulse delayed by one clock
    sc_signal<bool> iFilterClear  {"iFilterClear"}; // Reset input filter
    sc_signal<bool> iFSIN  {"iFSIN"}; // Filtered SIN
    sc_signal<bool> iFStopBit  {"iFStopBit"}; // Filtered SIN for stop bit detection
    sc_signal<bool> iParity  {"iParity"}; // Data parity
    sc_signal<bool> iParityReceived  {"iParityReceived"}; // Parity received
    sc_signal<sc_uint<5> > iDataCount  {"iDataCount"};   // Data bit counter
    sc_signal<bool> iDataCountInit  {"iDataCountInit"};  // Initialize data bit counter to word length
    sc_signal<bool> iDataCountFinish  {"iDataCountFinish"};  // Data bit counter finished
    sc_signal<bool> iRXFinished  {"iRXFinished"};    // Word received, output data valid
    sc_signal<bool> iFE  {"iFE"};    // Internal frame error
    sc_signal<bool> iBI  {"iBI"};    // Internal break interrupt
    sc_signal<bool> iNoStopReceived  {"iNoStopReceived"};    // No valid stop bit received
    sc_signal<sc_uint<8> > iDOUT  {"iDOUt"}; // Data output
    sc_signal<sc_uint<4> > iRxState  {"iRxState"}; //Rx State
    sc_signal<sc_uint<4> > iRxNextState  {"iRxNextState"};    //Rx Next State
    sc_signal<bool> iNoConnect  {"noconnect"};
    sc_signal<sc_uint<4> > iNoConnect4  {"noConnect4bit"};
    //Internal Signals
};
#endif