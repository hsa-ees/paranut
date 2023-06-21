/*************************************************************************

  This file is part of the ParaNut project.

  Copyright (C) 2022-2023 Elias Schuler <elias.schuler@hs-augsburg.de>
                2022-2023 Lukas Bauer <lukas.bauer@hs-ausgsburg.de>
      Hochschule Augsburg, University of Applied Sciences

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

#include "wb8_uart_16750.h"
#include "slib_clock_div.h"

#include <stdio.h>
#include <signal.h>
#include <systemc.h>
#include <fstream>
#include <string>
#include <sstream>
#include <vector>



#define CLK_PERIOD 38.5
#define STIM_FILENAME "stim.dat"

struct Tb : sc_module
{
    sc_in_clk clk {"clk"};                                  // Clock
    sc_signal<bool> rst {"rst"};                                // Reset
    sc_signal<bool> baudce {"baudce"};                          // Baudrate generator clock enable
    sc_signal<bool> wb_cyc {"wb_cyc"};                          // Wishbone cycle signal
    sc_signal<bool> wb_stb {"wb_stb"};                          // Wishbone strobe signal
    sc_signal<bool> wb_we {"wb_we"};                            // Wishbone write enable
    sc_signal<sc_uint<32> > wb_adr {"wb_adr"};                  // Wishbone address bus
    sc_signal<sc_uint<32> > wb_din {"wb_din"};                   // Wishbone data in bus
    sc_signal<sc_uint<32> > wb_dout {"wb_dout"};                // Wishbone data out bus
    sc_signal<bool> wb_ack {"wb_ack"};                         // Wishbone transaction acknowledge
    sc_signal<bool> int_o {"int"};                               // Interrupt output
    sc_signal<bool> out1 {"out1"};                             // Output 1
    sc_signal<bool> out2 {"out2"};                             // Output 2
    sc_signal<bool> rclk {"rclk"};                              // Receiver clock (16x baudrate)
    sc_signal<bool> baudoutn {"baudoutn"};                     // Baudrate generator output (16x baudrate)
    sc_signal<bool> rtsn {"rtsn"};                             // RTS output
    sc_signal<bool> dtrn {"dtrn"};                             // DTR output
    sc_signal<bool> ctsn {"ctsn"};                              // CTS input
    sc_signal<bool> dsrn {"dsrn"};                              // DSR input
    sc_signal<bool> dcd {"dcd"};                                // DCD input
    sc_signal<bool> rin {"rin"};                                // RI input
    sc_signal<bool> sin {"sin"};                                // Receiver input
    sc_signal<bool> sout {"sout"};                             // Transmitter output
    sc_signal<bool> baudce_i {"baudce_i"};                      // Baudce intermediate signal

    Wb8Uart16750 wb8_uart_16750 {"wb8_uart_16750"};


    SC_CTOR(Tb)
    {
        // UART
        wb8_uart_16750.clk_i (clk);
        wb8_uart_16750.rst_i (rst);
        wb8_uart_16750.baudce_i (baudce_i);
        wb8_uart_16750.wb_cyc_i (wb_cyc);
        wb8_uart_16750.wb_stb_i (wb_stb);
        wb8_uart_16750.wb_we_i (wb_we);
        wb8_uart_16750.wb_adr_i (wb_adr);
        wb8_uart_16750.wb_din_i (wb_din);
        wb8_uart_16750.wb_dout_o (wb_dout);
        wb8_uart_16750.wb_ack_o (wb_ack);
        wb8_uart_16750.int_o (int_o);
        wb8_uart_16750.out1_o (out1);
        wb8_uart_16750.out2_o (out2);
        wb8_uart_16750.rclk_i (baudoutn);
        wb8_uart_16750.baudoutn_o (baudoutn);
        wb8_uart_16750.rtsn_o (rtsn);
        wb8_uart_16750.dtrn_o (dtrn);
        wb8_uart_16750.ctsn_i (ctsn);
        wb8_uart_16750.dsrn_i (dsrn);
        wb8_uart_16750.dcd_i (dcd);
        wb8_uart_16750.rin_i (rin);
        wb8_uart_16750.sin_i (sin);
        wb8_uart_16750.sout_o (sout);



        SC_CTHREAD(test_proc, clk.pos());
    }

    void InitWBWrite (uint32_t adr, uint32_t val) {
        // Initalizes a Wishbone write operation
        wb_stb = 1;
        wb_cyc = 1;
        wb_we = 1;
        wb_din = val;
        wb_adr = adr;
    }

    void CompleteWBWrite (uint32_t adr, uint32_t val) {
        // Performs a complete Wishbone write operation
        InitWBWrite (adr, val);

        while (!wb_ack.read ()) wait(1);

        wb_stb = 0;
        wb_cyc = 0;
        wb_we = 0;
        wb_din = 0;
        wait(1);

    }

    void InitWBRead (uint32_t adr) {
        // Initializes a Wishbone read operation

        wb_stb = 1;
        wb_cyc = 1;
        wb_we = 0;
        wb_adr = adr;
    }

    uint32_t CompleteWBRead (uint32_t adr) {
        // Performs a complete Wishbone read operation

        InitWBRead(adr);

        while(!wb_ack.read()) wait(1);

        uint32_t ret = wb_dout.read();

        wb_stb = 0;
        wb_cyc = 0;
        wb_we = 0;

        wait(1);
        return ret;
    }

    void WriteReceive(uint8_t dout){
        // Sends a byte over UART (for 33 MHz System with Clockdivider  Ratio 18 and Baudrate 115200)

        //Start Bit
        sin = 0;
        wait(230);

        // Data
        for(size_t i = 0; i < 8; i++){
            bool bit =  dout & 0x01;
            dout >>= 1;
            sin = bit;
            wait(230);
        }

        //Stop bit
        sin = 1;
        wait(230);
    }

    void TestRegisters(uint32_t adr){

        #ifndef __SYNTHESIS__
        // Tests all combinations of a given Register
        uint32_t data = 0;
        wait(5);
        uint32_t temp = CompleteWBRead(adr);
        wait(5);
        for(size_t i = 0; i < 256; i++){
            CompleteWBWrite(adr, (uint32_t) i);
            wait(5);
            data = CompleteWBRead(adr);
            PN_INFOF(("Adr: 0x%08x Data: 0x%08x I: 0x%08x", adr, data, i));
            PN_ASSERTM(data == i, "Couldnt Write Register");
            wait(5);
            if(adr == CFG_UART_BASE_ADDRESS + 0xF){
                if(i>=0x3f){
                    i = 255;
                }
            }
        }
        CompleteWBWrite(adr, temp);
        wait(5);

        #endif
    }

    void simStimFile(){
        // Perform the Actions contained in a Stim File

        #ifndef __SYNTHESIS__

        std::fstream testfile;
        std::string line;
        std::string arg;
        std::string arg2;
        testfile.open(STIM_FILENAME, std::ios::in);

        if(!testfile.is_open()){
            PN_ERROR("Stimfile couldn\'t be opened");
            return;
        }else{

            while(getline(testfile, line)){
                // Reads the time to wait
                if(line.find("#WAIT") != std::string::npos){

                    arg = line.substr(6, std::string::npos);
                    wait(std::stoi(arg));

                // Reads from a Register and checks for the expected value
                }else if(line.find("#RD")!= std::string::npos){

                    arg = line.substr(4,7);
                    arg2 = line.substr(8,std::string::npos);
                    uint32_t adr = (uint32_t)std::stoi(arg, nullptr, 2);
                    adr *= 4;
                    adr += (uint32_t)CFG_UART_BASE_ADDRESS;
                    uint32_t data = std::stoi(arg2, nullptr, 2);
                    uint32_t value = CompleteWBRead((adr));
                    PN_INFOF(("Expected: \t 0x%08x Got: \t 0x%08x \t adr: 0x%08x", (uint32_t)data, (uint32_t)value, (uint32_t)(adr)));
                    PN_ASSERTM(value == data, "Failed");

                // Writes a given Value to the Register
                }else if(line.find("#WR") != std::string::npos){

                    arg = line.substr(4,7);
                    uint32_t adr = std::stoi(arg, nullptr, 2);
                    adr *= 4;
                    adr += (uint32_t)CFG_UART_BASE_ADDRESS;
                    arg2 = line.substr(8,std::string::npos);
                    uint32_t data = std::stoi(arg2, nullptr, 2);
                    PN_INFOF(("Writing to Address: \t 0x%08x Data: \t 0x%08x ", (uint32_t) (adr), (uint32_t)data));
                    CompleteWBWrite((adr), data);

                // Prints out log messages
                }else if(line.find("#LOG UART") != std::string::npos){

                    arg = line.substr(11, std::string::npos);
                    fprintf(stderr,"Currently: \t %s\n", arg.c_str());


                }
            }

            testfile.close();
        }

        #endif

    }


    void test_proc(){


        #ifndef __SYNTHESIS__
        // Set cfg_debug_mode to suppress some simulation warnings
        pn_cfg_debug_mode = true;
        // Trace file ...
        sc_trace_file *tf;
        if (pn_cfg_vcd_level > 0) {
            tf = sc_create_vcd_trace_file ("wb8_uart_16750_tb");
            tf->delta_cycles (false);


            wb8_uart_16750.Trace (tf, pn_cfg_vcd_level);
        } else {
            fprintf (stderr, "Tracing is disabled.\n");
            tf = NULL;
        }


        PN_INFO("Reseting...");
        rst = 1;
        wait(10);
        rst = 0;
        wait(2);

        while(true){
            PN_INFO("Running");

            //Setting up all the connecting pins
            uint32_t data = 0;
            // clkdiv_enabled = 1;
            baudce_i = 1;
            rin = 1;
            dcd = 1;
            dsrn = 1;
            ctsn = 1;
            sin  = 1;
            rst = 0;

            //Executing the test from a given Stimfile
            simStimFile();



            //Reseting the UART Module
            rst = 1;
            wait(5);
            rst = 0;
            wait(5);


            //Enabling the LCR_DLAB
            CompleteWBWrite(CFG_UART_BASE_ADDRESS + 0xc ,0x80);
            wait(5);

            //Setting the FIFO Enable and FIFO 64 bit mode
            CompleteWBWrite(CFG_UART_BASE_ADDRESS + 0x8 ,0x21);
            wait(5);


            //Setting the Baudrate to 115200
            CompleteWBWrite(CFG_UART_BASE_ADDRESS + 0x4 ,0x00);
            wait(5);

            CompleteWBWrite(CFG_UART_BASE_ADDRESS ,0x01);
            wait(5);

            //Disabling LCR_DLAB and setting the Word Length to 8 Bit
            CompleteWBWrite(CFG_UART_BASE_ADDRESS + 0xC ,0x03);
            wait(5);


            //Testing to see if Read and Write to RBR Work
            CompleteWBWrite(CFG_UART_BASE_ADDRESS,0x00);
            data = CompleteWBRead(CFG_UART_BASE_ADDRESS);
            PN_INFOF(("Data: 0x%08x",data));

            wait(5);

            //Writing 66 Values on the RX Pin
            for(size_t i = 0; i <66; i++){

                WriteReceive(0x00+i);
            }

            //Reading the RXFIFO to check if 64 Bytes are read correctly and assert that we dont overrun
            for(size_t i = 0; i <66; i++){

                data = CompleteWBRead(CFG_UART_BASE_ADDRESS);
                PN_INFOF(("Data: 0x%08x, %d",data, i));
                if(i < 64){
                    PN_ASSERTM(data == i, "Data isnt read correct");
                }else{
                    PN_ASSERTM(data == 0, "Data isnt read correct");
                }
                wait(5);
            }


            uint32_t Receive = CompleteWBRead(CFG_UART_BASE_ADDRESS);
            PN_INFOF(("Data: 0x%08x",Receive));
            wait(5);

            //Enable LCR_DLAB
            CompleteWBWrite(CFG_UART_BASE_ADDRESS + 0xC ,0x80);
            wait(5);
            //Test Writing and reading all the registers of DLL
            TestRegisters(CFG_UART_BASE_ADDRESS);
            wait(1000);
            //Test Writign and Reading all the registers of DLM
            TestRegisters(CFG_UART_BASE_ADDRESS + 0x4);
            wait(1000);
            //Test Writing and Reading all the registers of LCR
            TestRegisters(CFG_UART_BASE_ADDRESS + 0xC);
            wait(1000);
            //Test Writing and Reading all the registers of MCR
            TestRegisters(CFG_UART_BASE_ADDRESS + 0xF);
            wait(1000);
            //Test Writing and Reading all the register of SCR
            TestRegisters(CFG_UART_BASE_ADDRESS + 0x1C);
            wait(1000);

            wait(20448);

            PN_INFO ("Simulation finished.");
            sc_stop();
            wait();

            if (tf) sc_close_vcd_trace_file (tf);
        }
            #endif

    }


};

// **************** Main ************************

int sc_main (int argc, char *argv[]) {
    int arg, cfg_help = 0;

    // Parse command line...
    arg = 1;
    while (arg < argc && argv[arg][0] == '-') {
        switch (argv[arg][1]) {
        case 't':
            pn_cfg_vcd_level = MAX (0, MIN (9, argv[arg][2] - '0'));
            fprintf (stderr, "(cfg) vcdLevel = %i\n", pn_cfg_vcd_level);
            break;
        case 'h':
            cfg_help = 1;
            break;
        default:
            printf ("PN_ERROR: Unknown option '%s'.\n", argv[arg]);
            arg = argc;
        }
        arg++;
    }
    if (cfg_help) {
        puts ("Usage: dm_tb [<options>]\n"
              "\n"
              "Options:\n"
              "  -t<n>: set VCD trace level (0 = no trace file; default = 2)\n");
        return 3;
    }


    sc_clock clk{"clk", sc_time(CLK_PERIOD, SC_NS)};
    Tb tb("tb");
    tb.clk(clk);
    sc_start ();
    cout <<"\n\t\t*****Simulation complete*****" << endl;


    return 0;
}