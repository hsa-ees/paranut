/*************************************************************************

  This file is part of the ParaNut project.

  Copyright (C) 2022-2023 Elias Schuler <elias.schuler@hs-augsburg.de>
                2022-2023 Lukas Bauer <lukas.bauer@hs-ausgsburg.de>
      Hochschule Augsburg, University of Applied Sciences

    Description : This file is based on the work of Sebastian Witt and Federico Aglietti
                  Wishbone Uart

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

#ifndef _WB8_UART_16750_
#define _WB8_UART_16750_

#include "base.h"
#include "paranut-config.h"

#include "uart_transmitter.h"
#include "uart_receiver.h"
#include "uart_interrupt.h"
#include "uart_baudgen.h"
#include "slib_fifo.h"
#include "slib_fifo_11.h"
#include "slib_edge_detect.h"
#include "slib_input_sync.h"
#include "slib_input_filter.h"
#include "slib_input_filter_2.h"
#include "slib_clock_div.h"
#include "slib_clock_div_1.h"


#include <systemc.h>

// **************** Defines *************


typedef enum {TXIDLE,
              TXSTART,
              TXRUN,
              TXEND}TX_States;

typedef enum {RXIDLE,
              RXSAVE}RX_States;

class Wb8Uart16750: ::sc_core::sc_module{
public:
    //Ports (Wishbone)
    sc_in_clk clk_i  {"clk_i"};                                  // Clock
    sc_in<bool> rst_i  {"rst_i"};                                // Reset
    sc_in<bool> baudce_i  {"baudce_i"};                          // Baudrate generator clock enable
    sc_in<bool> wb_cyc_i  {"wb_cyc_i"};                          // Wishbone cycle signal
    sc_in<bool> wb_stb_i  {"wb_stb_i"};                          // Wishbone strobe signal
    sc_in<bool> wb_we_i  {"wb_we_i"};                            // Wishbone write enable
    sc_in<sc_uint<32> > wb_adr_i  {"wb_adr_i"};                  // Wishbone address bus
    sc_in<sc_uint<32> > wb_din_i  {"wb_din_i"};                   // Wishbone data in bus TODO: Dose this bus need to be 32 bit
    sc_out<sc_uint<32> > wb_dout_o  {"wb_dout_o"};                // Wishbone data out bus TODO: Dose this bus need to be 32 bit
    sc_out<bool> wb_ack_o  {"wb_ack_o"};                         // Wishbone transaction acknowledge
    sc_out<bool> int_o  {"int_o"};                               // Interrupt output
    sc_out<bool> out1_o  {"out1_o"};                             // Output 1
    sc_out<bool> out2_o  {"out2_o"};                             // Output 2
    sc_in<bool> rclk_i  {"rclk_i"};                              // Receiver clock (16x baudrate)
    sc_out<bool> baudoutn_o  {"baudoutn_o"};                     // Baudrate generator output (16x baudrate)
    sc_out<bool> rtsn_o  {"rtsn_o"};                             // RTS output
    sc_out<bool> dtrn_o  {"dtrn_o"};                             // DTR output
    sc_in<bool> ctsn_i  {"ctsn_i"};                              // CTS input
    sc_in<bool> dsrn_i  {"dsrn_i"};                              // DSR input
    sc_in<bool> dcd_i  {"dcd_i"};                                // DCD input
    sc_in<bool> rin_i  {"rin_i"};                                // RI input
    sc_in<bool> sin_i  {"sin_i"};                                // Receiver input
    sc_out<bool> sout_o  {"sout_o"};                             // Transmitter output



    //Ports
    //Constructor
    //Functions
    SC_HAS_PROCESS (Wb8Uart16750);
    Wb8Uart16750 (sc_module_name name)
        : sc_module (name) {

        SC_METHOD(TransitionMethod);
        sensitive << wb_cyc_i << wb_stb_i << wb_we_i << iRead << iWrite
                  << iA << iLCR_DLAB << wb_adr_i << wb_din_i << iIER
                  << iFCR_FIFOEnable << iLSR_DR << iRXFIFOTrigger << iIIR
                  << iFCR_FIFO64E << iFCR_RXFIFOReset << iFCR_TXFIFOReset
                  << iFCR_DMAMode << iFCR_RXTrigger << iLCR << iMCR
                  << iRXFIFOEmpty << iRXFIFOQ << iRXFIFOWrite << iRXFIFOD
                  << iFECounter << iPERE << iFERE << iBIRE << iLSR_OE << iLSR_PE
                  << iLSR_FE << iLSR_BI << iLSR_THRE << iLSR_TEMT
                  << iLSR_FIFOERR  << iTXFIFOEmpty << iTXRunning << iMCR_LOOP << iRTS
                  << iMCR_DTR << iMCR_OUT1 << iMCR_OUT2 << iCTSn << iDSRn << iRIn << iDCDn
                  << iMSR_dCTS << iMSR_dDSR << iMSR_TERI << iMSR_TERI << iMSR_dDCD
                  << iMSR_CTS << iMSR_DSR << iMSR_RI << iMSR_DCD << iTXFIFOUsage
                  << iTXFIFO64Full << iTXFIFOFull << iTHRWrite << iTXFIFO16Full << iRBRRead
                  << iRXFIFOUsage << iRXFIFO64Full << iRXFIFO16Full << iRXFIFO16Trigger
                  << iRXFIFO64Trigger << iSINr  << iMCR_AFE << iDLM << iLSR << iMSR << iDLL
                  << iSOUT << iFCR << wb_dout_o << iBaudgenDiv << WB_ACK_R;

        SC_METHOD(WriteMethod);
        sensitive << iIER << iLSR << iMSR << iIIR;

        SC_CTHREAD(WBAckPrMethod, clk_i.pos());
        reset_signal_is(rst_i, true);

        SC_CTHREAD(UartDlrMethod, clk_i.pos());
        reset_signal_is(rst_i, true);

        SC_CTHREAD(UartIerMethod, clk_i.pos());
        reset_signal_is(rst_i, true);

        SC_CTHREAD(UartIicThreiMethod, clk_i.pos());
        reset_signal_is(rst_i, true);

        SC_CTHREAD(UartCtiMethod, clk_i.pos());
        reset_signal_is(rst_i, true);

        SC_CTHREAD(UartFcrMethod, clk_i.pos());
        reset_signal_is(rst_i, true);

        SC_CTHREAD(UartLcrMethod, clk_i.pos());
        reset_signal_is(rst_i, true);

        SC_CTHREAD(UartMcrMethod, clk_i.pos());
        reset_signal_is(rst_i, true);

        SC_CTHREAD(UartLsrMethod, clk_i.pos());
        reset_signal_is(rst_i, true);

        SC_CTHREAD(UartMsrMethod, clk_i.pos());
        reset_signal_is(rst_i, true);

        SC_CTHREAD(UartScrMethod, clk_i.pos());
        reset_signal_is(rst_i, true);

        SC_CTHREAD(UartTxMethod, clk_i.pos());
        reset_signal_is(rst_i, true);

        SC_CTHREAD(UartRxMethod, clk_i.pos());
        reset_signal_is(rst_i, true);

        SC_CTHREAD(UartAfcMethod, clk_i.pos());
        reset_signal_is(rst_i, true);

        SC_CTHREAD(UartOutRegsMethod, clk_i.pos());
        reset_signal_is(rst_i, true);

        SC_METHOD(UartDoutMethod);
        sensitive << WB_ACK_R << iLCR_DLAB << iRBR << iDLL << iDLM << iIER << iIIR
                  << iLCR << iMCR << iLSR << iMSR << iSCR << iA << wb_dout_o;

        SC_METHOD(IERCombinationMethod);
        sensitive << iIER_3_0 << iIER_7_4 << iIER;

        SC_METHOD(IIRCombinationMethod);
        sensitive << iIIR_3_0 << iIIR_7_4 << iIIR;

        SC_METHOD(UartRxStates);
        sensitive << rx_state << iRXBI << iRXFE << iRXPE << iRXData << iRXFinished
                  << iFCR_FIFOEnable << iRXFIFOFull << iFCR_RXFIFOReset << rst_i << iRXFIFOD;

        SC_METHOD(UartTxStates);
        sensitive << tx_state << iTXFIFOQ << rst_i << iTXEnable << iTXFinished << iFCR_RXFIFOReset << iTSR;

        InitSubModules();

    };
    void Trace (sc_trace_file *tf, int levels = 1);
#ifndef __SYNTHESIS__
#endif
    //Submodules
    SlibInputSync uart_is_sin{"uart_is_sin"};
    SlibInputSync uart_is_cts{"uart_is_cts"};
    SlibInputSync uart_is_dsr{"uart_is_dsr"};
    SlibInputSync uart_is_dcd{"uart_is_dcd"};
    SlibInputSync uart_is_ri{"uart_is_ri"};

    SlibInputFilter2 uart_if_cts{"uart_if_cts"};
    SlibInputFilter2 uart_if_dsr{"uart_if_dsr"};
    SlibInputFilter2 uart_if_dcd{"uart_if_dcd"};
    SlibInputFilter2 uart_if_ri{"uart_if_ri"};

    SlibEdgeDetect uart_iic_thre_ed{"uart_iic_thre_ed"};
    SlibEdgeDetect uart_pedet{"uart_pedet"};
    SlibEdgeDetect uart_fedet{"uart_fedet"};
    SlibEdgeDetect uart_bidet{"uart_bidet"};
    SlibEdgeDetect uart_ed_cts{"uart_ed_cts"};
    SlibEdgeDetect uart_ed_dsr{"uart_ed_dsr"};
    SlibEdgeDetect uart_ed_ri{"uart_ed_ri"};
    SlibEdgeDetect uart_ed_dcd{"uart_ed_dcd"};
    SlibEdgeDetect uart_rclk{"uart_rclk"};

    SlibClockDiv1 uart_bg2{"uart_bg2"};
    SlibClockDiv  uart_bg1 {"uart_bg1"};

    SlibFifo uart_txff{"uart_txff"};
    SlibFifo11 uart_rxff{"uart_rxff"};

    UartInterrupt uart_iic{"uart_iic"};

    UartBaudgen uart_bg16{"uart_bg16"};

    UartTransmitter uart_tx{"uart_tx"};

    UartReceiver uart_rx{"uart_rx"};

    //Processes

    void WriteMethod();
    void TransitionMethod();
    void WBAckPrMethod();
    void UartDlrMethod();
    void UartIerMethod();
    void UartIicThreiMethod();
    void UartCtiMethod();
    void UartFcrMethod();
    void UartLcrMethod();
    void UartMcrMethod();
    void UartLsrMethod();
    void UartMsrMethod();
    void UartScrMethod();
    void UartAfcMethod();
    void UartOutRegsMethod();
    void UartDoutMethod();
    void IERCombinationMethod();
    void IIRCombinationMethod();

    // State Machines
    void UartTxMethod();
    void UartRxMethod();
    void UartRxStates();
    void UartTxStates();

    // Functions
    void InitSubModules();

protected:
    //Registers

    //Internal Signals

    // Global device signals
    sc_signal<bool> iWriteFE  {"iWriteFE"};                      // Write falling edge
    sc_signal<bool> iReadFE  {"iReadFE"};                        // Read falling edge
    sc_signal<bool> iWrite  {"iWrite"};                          // Write to UART
    sc_signal<bool> iRead  {"iRead"};                            // Read from UART
    sc_signal<sc_uint<3> > iA  {"iA"};                           // UART register address
    sc_signal<sc_uint<8> > iDIN  {"iDIN"};                       // UART data input

    // UART registers read/write signals
    sc_signal<bool> iRBRRead  {"iRBRRead"};                      // Read from RBR
    sc_signal<bool> iTHRWrite  {"iTHRWrite"};                    // Write to THR
    sc_signal<bool> iDLLWrite  {"iDLLWrite"};                    // Write to DLL
    sc_signal<bool> iDLMWrite  {"iDLMWrite"};                    // Write to DLM
    sc_signal<bool> iIERWrite  {"iIERWrite"};                    // Write to IER
    sc_signal<bool> iIIRRead  {"iIIRead"};                       // Read from IIR
    sc_signal<bool> iFCRWrite  {"iFCRWrite"};                    // Write to FCR
    sc_signal<bool> iLCRWrite  {"iLCRWrite"};                    // Write to LCR
    sc_signal<bool> iMCRWrite  {"iMCRWrite"};                    // Write to MCR
    sc_signal<bool> iLSRRead  {"iLSRRead"};                      // Read from LSR
    sc_signal<bool> iMSRRead  {"iMSRRead"};                      // Read from MSR
    sc_signal<bool> iSCRWrite  {"iSCRWrite"};                    // Write to SCR

    // UART registers
    sc_signal<sc_uint<8> > iTSR  {"iTSR"};                       // Transmitter holding register
    sc_signal<sc_uint<8> > iRBR  {"iRBR"};                       // Receiver buffer register
    sc_signal<sc_uint<8> > iDLL  {"iDLL"};                       // Divisor latch LSB
    sc_signal<sc_uint<8> > iDLM  {"iDLM"};                       // Divisor latch MSB
    sc_signal<sc_uint<8> > iIER  {"iIER"};                       // Interrupt enable register
    sc_signal<sc_uint<8> > iIIR  {"iIIR"};                       // Interrupt  identification register
    sc_signal<sc_uint<8> > iFCR  {"iFCR"};                       // FIFO control register
    sc_signal<sc_uint<8> > iLCR  {"iLCR"};                       // Line control register
    sc_signal<sc_uint<8> > iMCR  {"iMCR"};                       // Modem control register
    sc_signal<sc_uint<8> > iLSR  {"iLSR"};                       // Line status register
    sc_signal<sc_uint<8> > iMSR  {"iMSR"};                       // Modem status register
    sc_signal<sc_uint<8> > iSCR  {"iSCR"};                       // Scratch register

    // IER register signals
    sc_signal<bool> iIER_ERBI  {"iIER_ERBI"};                    // IER: Enable received data available interrupt
    sc_signal<bool> iIER_ETBEI  {"iIER_ETBEI"};                  // IER: Enable transmitter holding register empty interrupt
    sc_signal<bool> iIER_ELSI  {"iIER_ELSI"};                    // IER: Enable receiver line status interrupt
    sc_signal<bool> iIER_EDSSI  {"iIER_EDSSI"};                  // IER: Enable modem status interrupt

    // IIR register signals
    sc_signal<bool> iIIR_PI  {"iIIR_PI"};                        // IIR: Pending interrupt
    sc_signal<bool> iIIR_ID0  {"iIIR_ID0"};                      // IIR: Interrupt ID0
    sc_signal<bool> iIIR_ID1  {"iIIR_ID1"};                      // IIR: Interrupt ID1
    sc_signal<bool> iIIR_ID2  {"iIIR_ID2"};                      // IIR: Interrupt ID2
    sc_signal<bool> iIIR_FIFO64  {"iIIR_FIFO64"};                // IIR: 64 byte FIFO enabled

    // FCR register signals
    sc_signal<bool> iFCR_FIFOEnable  {"iFCR_FIFOEnable"};        // FCR: FIFO enable
    sc_signal<bool> iFCR_RXFIFOReset  {"iFCR_RXFIFOReset"};      // FCR: Receiver FIFO reset
    sc_signal<bool> iFCR_TXFIFOReset  {"iFCR_TXFIFOReset"};      // FCR: Transmitter FIFO reset
    sc_signal<bool> iFCR_DMAMode  {"iFCR_DMAMode"};              // FCR: DMA mode select
    sc_signal<bool> iFCR_FIFO64E  {"iFCR_FIFO64E"};              // FCR: 64 byte FIFO enable
    sc_signal<sc_uint<2> > iFCR_RXTrigger  {"iFCR_RXTrigger"};   // FCR: Receiver Trigger

    // LCR register signals
    sc_signal<sc_uint<2> > iLCR_WLS  {"iLCR_WLS"};               // LCR: Word length select
    sc_signal<bool> iLCR_STB  {"iLCR_STB"};                      // LCR: Number of stop bits
    sc_signal<bool> iLCR_PEN  {"iLCR_PEN"};                      // LCR: Parity enable
    sc_signal<bool> iLCR_EPS  {"iLCR_EPS"};                      // LCR: Even parity select
    sc_signal<bool> iLCR_SP  {"iLCR_SP"};                        // LCR: Sticky parity
    sc_signal<bool> iLCR_BC  {"iLCR_BC"};                        // LCR: Break control
    sc_signal<bool> iLCR_DLAB  {"iLCR_DLAB"};                    // LCR: Divisor latch access bit

    // MCR register signals
    sc_signal<bool> iMCR_DTR  {"iMCR_DTR"};                      // MCR: Data terminal ready
    sc_signal<bool> iMCR_RTS  {"iMCR_RTS"};                      // MCR: Request to send
    sc_signal<bool> iMCR_OUT1  {"iMCR_OUT1"};                    // MCR: OUT1
    sc_signal<bool> iMCR_OUT2  {"iMCR_OUT2"};                    // MCR: OUT2
    sc_signal<bool> iMCR_LOOP  {"iMCR_LOOP"};                    // MCR: Loop
    sc_signal<bool> iMCR_AFE  {"iMCR_AFE"};                      // MCR: Auto flow control enable

    // LSR register signals
    sc_signal<bool> iLSR_DR  {"iLSR_DR"};                        // LSR: Data ready
    sc_signal<bool> iLSR_OE  {"iLSR_OE"};                        // LSR: Overrun error
    sc_signal<bool> iLSR_PE  {"iLSR_PE"};                        // LSR: Parity error
    sc_signal<bool> iLSR_FE  {"iLSR_FE"};                        // LSR: Framing error
    sc_signal<bool> iLSR_BI  {"iLSR_BI"};                        // LSR: Break Interrupt
    sc_signal<bool> iLSR_THRE  {"iLSR_THRE"};                    // LSR: Transmitter holding register empty
    sc_signal<bool> iLSR_TEMT  {"iLSR_TEMT"};                    // LSR: Transmitter empty
    sc_signal<bool> iLSR_FIFOERR  {"iLSR_FIFOERR"};              // LSR: EError in receiver FIFO


    // MSR register signals
    sc_signal<bool> iMSR_dCTS  {"iMSR_dCTS"};                    // MSR: Delta CTS
    sc_signal<bool> iMSR_dDSR  {"iMSR_dDSR"};                    // MSR: Delta DSR
    sc_signal<bool> iMSR_TERI  {"iMSR_TERI"};                    // MSR: Trailing edge ring indicator
    sc_signal<bool> iMSR_dDCD  {"iMSR_dDCD"};                    // MSR: Delta DCD
    sc_signal<bool> iMSR_CTS  {"iMSR_CTS"};                      // MSR: CTS
    sc_signal<bool> iMSR_DSR  {"iMSR_DSR"};                      // MSR: DSR
    sc_signal<bool> iMSR_RI  {"iMSR_RI"};                        // MSR: RI
    sc_signal<bool> iMSR_DCD  {"iMSR_DCD"};                      // MSR: DCD

    // UART MSR signals
    sc_signal<bool> iCTSNs  {"iCTSNs"};                          // Synchronized CTSN input
    sc_signal<bool> iDSRNs  {"iDSRNs"};                          // Synchronized DSRN input
    sc_signal<bool> iDCDNs  {"iDCDNs"};                          // Synchronized DCDN input
    sc_signal<bool> iRINs  {"iRINs"};                            // Synchronized RIN input
    sc_signal<bool> iCTSn  {"iCTSn"};                            // Filtered CTSN input
    sc_signal<bool> iDSRn  {"iDSRn"};                            // Filtered DSRN input
    sc_signal<bool> iDCDn  {"iDCDn"};                            // Filtered DCDN input
    sc_signal<bool> iRIn  {"iRIn"};                              // Filtered RIN input
    sc_signal<bool> iCTSnRE  {"iCTSnRE"};                        // CTSn rising edge
    sc_signal<bool> iCTSnFE  {"iCTSnFE"};                        // CTSn falling edge
    sc_signal<bool> iDSRnRE  {"iDSRnRE"};                        // DSRn rising edge
    sc_signal<bool> iDSRnFE  {"iDSRnFE"};                        // DSRn falling edge
    sc_signal<bool> iDCDnRE  {"iDCDnRE"};                        // DCDn rising edge
    sc_signal<bool> iDCDnFE  {"iDCDnFE"};                        // DCDn falling edge
    sc_signal<bool> iRInRE  {"iRInRE"};                          // RIn rising edge
    sc_signal<bool> iRInFE  {"iRInFE"};                          // RIn falling edge

    // UART baudrate generation signals
    sc_signal<sc_uint<16> > iBaudgenDiv  {"iBaudgenDiv"};        // Baudrate divider
    sc_signal<bool> iBaudtick16x  {"iBaudtick16x"};              // 16x Baudrate output form baudrate generator
    sc_signal<bool> iBaudtick2x  {"iBaudtick2x"};                // 2x Baudrate for transmitter
    sc_signal<bool> iRCLK  {"iRCLK"};                            // 16x Baudrate for receiver

    // UART FIFO signals
    sc_signal<bool> iTXFIFOClear  {"iTXFIFOClear"};              // Clear TX FIFO
    sc_signal<bool> iTXFIFOWrite  {"iTXFIFOWrite"};              // Write to TX FIFO
    sc_signal<bool> iTXFIFORead  {"iTXFIFORead"};                // Read from TX FIFO
    sc_signal<bool> iTXFIFOEmpty  {"iTXFIFOEmpty"};              // TX FIFO is empty
    sc_signal<bool> iTXFIFOFull  {"iTXFIFOFull"};                // TX FIFO is full
    sc_signal<bool> iTXFIFO16Full  {"iTXFIFO16Full"};            // TX FIFO 16 bytes mode is full
    sc_signal<bool> iTXFIFO64Full  {"iTXFIFO64Full"};            // TX FIFO 64 bytes mode is full
    sc_signal<sc_uint<6> > iTXFIFOUsage  {"iTXFIFOUsage"};       // TX FIFO usage
    sc_signal<sc_uint<8> > iTXFIFOQ  {"iTXFIFOQ"};               // TX FIFO output
    sc_signal<bool> iRXFIFOClear  {"iRXFIFOClear"};              // Clear RX FIFO
    sc_signal<bool> iRXFIFOWrite  {"iRXFIFOWrite"};              // Write to RX FIFO
    sc_signal<bool> iRXFIFORead  {"iRXFIFORead"};                // Read from RX FIFO
    sc_signal<bool> iRXFIFOEmpty  {"iRXFIFOEmpty"};              // RX FIFO is empty
    sc_signal<bool> iRXFIFOFull  {"iRXFIFOFull"};                // RX FIFO is full
    sc_signal<bool> iRXFIFO16Full  {"iRXFIFO16Full"};            // RX FIFO 16 bytes mode is full
    sc_signal<bool> iRXFIFO64Full  {"iRXFIFO64Full"};            // RX FIFO 64 bytes mode is full
    sc_signal<sc_uint<11> > iRXFIFOD  {"iRXFIFOD"};              // RX FIFO input
    sc_signal<sc_uint<11> > iRXFIFOQ  {"iRXFIFOQ"};              // RX FIFO output
    sc_signal<sc_uint<6> > iRXFIFOUsage  {"iRXFIFOUsage"};       // RX FIFO usage
    sc_signal<bool> iRXFIFOTrigger  {"iRXFIFOTrigger"};          // FIFO trigger level reached
    sc_signal<bool> iRXFIFO16Trigger  {"iRXFIFO16Trigger"};      // FIFO 16 byte mode trigger level reached
    sc_signal<bool> iRXFIFO64Trigger  {"iRXFIFO64Trigger"};      // FIFO 64 byte mode trigger level reached
    sc_signal<bool> iRXFIFOPE  {"iRXFIFOPE"};                    // Parity error from FIFO
    sc_signal<bool> iRXFIFOFE  {"iRXFIFOFE"};                    // Frame error from FIFO
    sc_signal<bool> iRXFIFOBI  {"iRXFIFOBI"};                    // Break interrupt from FIFO

    // UART transmitter signals
    sc_signal<bool> iSOUT  {"iSOUT"};                            // Transmitter output
    sc_signal<bool> iTXStart  {"iTXStart"};                      // Start transmitter
    sc_signal<bool> iTXClear  {"iTXClear"};                      // Clear transmitter status
    sc_signal<bool> iTXFinished  {"iTXFinished"};                // TX finished, character transmitted
    sc_signal<bool> iTXRunning  {"iTXRunning"};                  // TX in progress

    // UART receiver signals
    sc_signal<bool> iSINr  {"iSINr"};                            // Synchronized SIN input
    sc_signal<bool> iSIN  {"iSIN"};                              // Receiver input
    sc_signal<bool> iRXFinished  {"iRXFinished"};                // RX finished, character received
    sc_signal<bool> iRXClear  {"iRXClear"};                      // Clear receiver status
    sc_signal<sc_uint<8> > iRXData  {"iRXData"};                 // RX data
    sc_signal<bool> iRXPE  {"iRXPE"};                            // RX parity error
    sc_signal<bool> iRXFE  {"iRXFE"};                            // RX frame error
    sc_signal<bool> iRXBI  {"iRXBI"};                            // RX break interrupt

    // UART control signals
    sc_signal<bool> iFERE  {"iFERE"};                            // Frame error detected
    sc_signal<bool> iPERE  {"iPERE"};                            // Parity error detected
    sc_signal<bool> iBIRE  {"iBIRE"};                            // Break interrupt detected
    sc_signal<sc_uint<7> > iFECounter  {"iFECounter"};           // FIFO error counter
    sc_signal<bool> iFEIncrement  {"iFEIncrement"};              // FIFO error counter increment
    sc_signal<bool> iFEDecrement  {"iFEDecrement"};              // FIFO error counter decrement
    sc_signal<bool> iRDAInterrupt  {"iRDAInterrupt"};            // Receiver data available interrupt (DA or FIFO trigger level)
    sc_signal<sc_uint<6> > iTimeoutCount  {"iTimeoutCount"};     // Character timeout counter (FIFO mode)
    sc_signal<bool> iCharTimeout  {"iCharTimeout"};              // Character timeout indication (FIFO mode)
    sc_signal<bool> iLSR_THRERE  {"iLSR_THRERE"};                // LSR THRE rising edge for interrupt generation
    sc_signal<bool> iTHRInterrupt  {"iTHRInterrupt"};            // Transmitter holding register empty interrupt
    sc_signal<bool> iTXEnable  {"iTXEnable"};                    // Transmitter enable signal
    sc_signal<bool> iRTS  {"iRTS"};                              // Internal RTS signal with/without automatic flow control

    sc_signal<bool> WB_ACK_R  {"WB_ACK_R"};
    sc_signal<bool> clock_enable {"Clock_enabled"};


    // partial signals
    sc_signal<sc_uint<4> > iIER_3_0  {"iIER_3_0"};
    sc_signal<sc_uint<4> > iIER_7_4  {"iIER_7_4"};
    sc_signal<sc_uint<5> > iLSR_4_0  {"iLSR_4_0"};
    sc_signal<sc_uint<4> > iMSR_3_0  {"iMSR_3_0"};
    sc_signal<sc_uint<4> > iIIR_3_0  {"iIIR_3_0"};
    sc_signal<sc_uint<8> > iLSR_7_0  {"iLSR_7_0"};
    sc_signal<sc_uint<8> > iMSR_7_0  {"iMSR_7_0"};
    sc_signal<sc_uint<4> > iIIR_7_4  {"iIIR_7_4"};

    // no connect signals
    sc_signal<bool> NC_fe_o  {"NC_fe_o"};
    sc_signal<bool> NC_fe_o_2  {"NC_fe_o_2"};
    sc_signal<bool> NC_fe_o_3  {"NC_fe_o_3"};
    sc_signal<bool> NC_fe_o_4  {"NC_fe_o_4"};
    sc_signal<bool> NC_fe_o_5  {"NC_fe_o_5"};
    sc_signal<bool> NC_i  {"NC_i"};


    // State signals
    sc_signal<sc_uint<1> > rx_state  {"rx_state"};
    sc_signal<sc_uint<1> > next_rx_state {"next_rx_state"};
    sc_signal<sc_uint<2> > tx_state  {"tx_state"};
    sc_signal<sc_uint<2> > next_tx_state {"next_tx_state"};

    sc_signal<sc_uint<11> > next_iRXFIFOD {"next_iRXFIFOD"};
    sc_signal<sc_uint<32> > iTSR_next {"iTSR_next"};




};
#endif