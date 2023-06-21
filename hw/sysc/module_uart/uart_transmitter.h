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

#ifndef _UART_TRANSMITTER_
#define _UART_TRANSMITTER_

#include "base.h"

#include <systemc.h>

// **************** Defines *************
// **************** DM Register Addresses *************
// **************** Module *************

class UartTransmitter : ::sc_core::sc_module
{
public:
    // Ports (Wishbone)
    sc_in_clk clk_i {"clk_i"};                    // Clock
    sc_in<bool> rst_i {"rst_i"};                  // Reset
    sc_in<bool> tx_clk_i {"tx_clk_i"};            // Receiver clock(2x baudrate)
    sc_in<bool> tx_start_i {"rx_start_i"};        // Start transmitter
    sc_in<bool> clear_i {"clear_i"};              // Clear transmitter state
    sc_in<sc_uint<2> > wls_i {"wls_i"};            // Word length select
    sc_in<bool> stb_i {"stb_i"};                  // Number of stop bits
    sc_in<bool> pen_i {"pen_i"};                  // Parity enable
    sc_in<bool> eps_i {"eps_i"};                  // Even parity select
    sc_in<bool> sp_i {"sp_i"};                    // Stick parity
    sc_in<bool> bc_i {"bc_i"};                    // Break control
    sc_in<sc_uint<8> > din_i {"din_i"};            // Input data
    sc_out<bool> tx_finished_o {"tx_finished_o"}; // Transmitter operation finished
    sc_out<bool> sout_o {"sout_o"};               // Transmitter output

    // **************** Receiver States ****************

    typedef enum
    {
        IDLE,
        START,
        BIT0,
        BIT1,
        BIT2,
        BIT3,
        BIT4,
        BIT5,
        BIT6,
        BIT7,
        PAR,
        STOP,
        STOP2,
    } TxStates;

    // Ports
    // Constructor
    // Functions
    SC_HAS_PROCESS(UartTransmitter);
    UartTransmitter(sc_module_name name)
        : sc_module(name)
    {
        SC_METHOD(OutputMethod);
        sensitive << iSout << iFinished << bc_i;

        SC_METHOD(TX_FMSMethod);
        sensitive << tx_state << tx_start_i << din_i << wls_i << pen_i
        << sp_i << eps_i << stb_i << iParity;

        SC_METHOD(TX_PARMethod);
        sensitive << din_i << wls_i;

        SC_CTHREAD(TX_FINMethod, clk_i.pos());
        reset_signal_is(rst_i, true);

        SC_CTHREAD(TXMethod, clk_i.pos());
        reset_signal_is(rst_i, true);
    };
    void Trace(sc_trace_file *tf, int levels = 1);
#ifndef __SYNTHESIS__
#endif
    // Submodules
    // Processes
    void TXMethod();
    void TX_FMSMethod();
    void TX_PARMethod();
    void TX_FINMethod();
    void OutputMethod();

protected:
    // Registerss
    sc_signal<bool> iTx2  {"iTx2"}; // Next TX step
    sc_signal<bool> iSout  {"iSout"}; // Transmitter output
    sc_signal<bool> iParity  {"iParity"}; // Parity
    sc_signal<bool> iFinished  {"iFinished"}; // Tx finished
    sc_signal<sc_uint<5> > tx_state  {"tx_state"}; // Tx state
    sc_signal<sc_uint<5> > next_tx_state  {"next_tx_state"}; // next Tx state
    // Internal Signals
};
#endif