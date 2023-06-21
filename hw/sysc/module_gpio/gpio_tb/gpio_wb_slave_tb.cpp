/*************************************************************************

  This file is part of the ParaNut project.

  Copyright (C) 2023 Elias Schuler <elias.schuler@hs-augsburg.de>
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

#include "gpio_wb_slave.h"

#include "base.h"

#include <stdio.h>
#include <signal.h>
#include <systemc.h>
#include <fstream>
#include <string>
#include <sstream>
#include <vector>



#define CLK_PERIOD 20

struct Tb : sc_module
{
    sc_in_clk           wb_clk_i;          // clock input
    sc_signal<bool>         wb_rst_i;          // reset

    sc_signal<bool>         wb_stb_i;          // strobe input
    sc_signal<bool>         wb_cyc_i;          // cycle valid input
    sc_signal<bool>         wb_we_i;           // indicates write transfer
    sc_signal<sc_uint<3> >  wb_cti_i;          // cycle type identifier (optional, for registered feedback)
    sc_signal<sc_uint<2> >  wb_bte_i;          // burst type extension (optional, for registered feedback)
    sc_signal<sc_uint<WB_WIDTH/8> > wb_sel_i;  // byte select inputs
    sc_signal<bool>        wb_ack_o;          // normal termination
    sc_signal<bool>        wb_err_o;          // termination w/ error (optional)
    sc_signal<bool>        wb_rty_o;          // termination w/ retry (optional)

    sc_signal<sc_uint<32> >        wb_adr_i;   // address bus inputs
    sc_signal<sc_uint<WB_WIDTH> >  wb_dat_i;   // input data bus
    sc_signal<sc_uint<WB_WIDTH> > wb_dat_o;   // output data bus

    // User Ports...
    sc_vector<sc_signal<bool> > gpio_input_port_tb;
    sc_vector<sc_signal<bool> > gpio_output_port_tb;
    gpio_wb_slave gpio_wb_slave1 {"gpio_wb_slave"};


    SC_CTOR(Tb)
    : gpio_input_port_tb("gpio_input_port", CFG_GPIO_INAMOUNT)
    , gpio_output_port_tb("gpio_output_port", CFG_GPIO_OUTAMOUNT)
    {
        // UART
        gpio_wb_slave1.wb_clk_i(wb_clk_i);          // clock input
        gpio_wb_slave1.wb_rst_i(wb_rst_i);          // reset
        gpio_wb_slave1.wb_stb_i(wb_stb_i);          // strobe input
        gpio_wb_slave1.wb_cyc_i(wb_cyc_i);          // cycle valid input
        gpio_wb_slave1.wb_we_i (wb_we_i);           // indicates write transfer
        gpio_wb_slave1.wb_cti_i(wb_cti_i);          // cycle type identifier (optional, for registered feedback)
        gpio_wb_slave1.wb_bte_i(wb_bte_i);          // burst type extension (optional, for registered feedback)
        gpio_wb_slave1.wb_sel_i(wb_sel_i);  // byte select inputs
        gpio_wb_slave1.wb_ack_o(wb_ack_o);          // normal termination
        gpio_wb_slave1.wb_err_o(wb_err_o);          // termination w/ error (optional)
        gpio_wb_slave1.wb_rty_o(wb_rty_o);          // termination w/ retry (optional)
        gpio_wb_slave1.wb_adr_i(wb_adr_i);   // address bus inputs
        gpio_wb_slave1.wb_dat_i(wb_dat_i);   // input data bus
        gpio_wb_slave1.wb_dat_o(wb_dat_o);   // output data bus
        gpio_wb_slave1.gpio_input_port(gpio_input_port_tb);
        gpio_wb_slave1.gpio_output_port(gpio_output_port_tb);

        SC_CTHREAD(test_proc, wb_clk_i.pos());
    }

    void InitWBWrite (uint32_t adr, uint32_t val) {
        // Initalizes a Wishbone write operation
        wb_stb_i = 1;
        wb_cyc_i = 1;
        wb_we_i = 1;
        wb_sel_i.write(0xF);
        wb_dat_i = val;
        wb_adr_i = adr;
    }

    void CompleteWBWrite (uint32_t adr, uint32_t val) {
        // Performs a complete Wishbone write operation
        InitWBWrite (adr, val);

        while (!wb_ack_o.read ()){
            wait(1);
        }

        wb_stb_i = 0;
        wb_cyc_i = 0;
        wb_sel_i.write(0);
        wb_we_i = 0;
        wb_dat_i = 0;
        wait(1);

    }

    void InitWBRead (uint32_t adr) {
        // Initializes a Wishbone read operation

        wb_stb_i = 1;
        wb_cyc_i = 1;
        wb_sel_i.write(0xF);
        wb_we_i = 0;
        wb_adr_i = adr;
    }

    uint32_t CompleteWBRead (uint32_t adr) {
        // Performs a complete Wishbone read operation

        InitWBRead(adr);

        while(!wb_ack_o.read()) wait(1);

        uint32_t ret = wb_dat_o.read();

        wb_stb_i = 0;
        wb_cyc_i = 0;
        wb_we_i = 0;
        wb_sel_i.write(0);


        wait(1);
        return ret;
    }
    void checkGpioOut(){
        sc_uint<32> temp = CompleteWBRead(0x40000000);
        PN_INFOF(("Enabling GPIOs OUT %d, %d, %d, %d", (uint8_t)temp[3], (uint8_t)temp[2], (uint8_t)temp[1], (uint8_t)temp[0]));
        for(size_t i = 0; i < CFG_GPIO_OUTAMOUNT; i++){
            PN_ASSERTM(gpio_output_port_tb[i] == temp[i], "OUTPUT ISNT CORRECT");    
        }
    }

    void test_proc(){


        #ifndef __SYNTHESIS__
        // Set cfg_debug_mode to suppress some simulation warnings
        pn_cfg_debug_mode = true;
        // Trace file ...
        sc_trace_file *tf;
        if (pn_cfg_vcd_level > 0) {
            tf = sc_create_vcd_trace_file ("gpio_wb_slave_tb");
            tf->delta_cycles (false);


            gpio_wb_slave1.trace (tf, pn_cfg_vcd_level);
        } else {
            fprintf (stderr, "Tracing is disabled.\n");
            tf = NULL;
        }

        PN_INFO("Reseting...");
        wb_rst_i = 1;
        wait(10);
        wb_rst_i = 0;
        wait(2);

        while(true){
            PN_INFO("Running");
            PN_INFO("Checking if all the GPIO OUTs work\n");
            for(uint32_t i = 0; i < (CFG_GPIO_OUTAMOUNT*4); i++){
                CompleteWBWrite(CFG_GPIO_BASE_ADDRESS, i);
                wait(1);
                checkGpioOut();
                wait(1);
            }
            PN_INFO("Trying to write in Input Pin\n");
            CompleteWBWrite(CFG_GPIO_BASE_ADDRESS, ((CFG_GPIO_OUTAMOUNT*4)+1));
            sc_uint<32> data = CompleteWBRead(CFG_GPIO_BASE_ADDRESS);
            PN_INFOF(("Data is : %d", (uint8_t) data[1+CFG_GPIO_OUTAMOUNT]));
            PN_ASSERTM(data[1+CFG_GPIO_OUTAMOUNT] == 0,"Wrote on Input Pin this shouldnt happen");
            PN_INFO("Reseting...");
            wb_rst_i = 1;
            wait(10);
            wb_rst_i = 0;
            wait(2);
            checkGpioOut();
            PN_INFO("Checking if all the GPIO INs work\n");
            for(size_t i = 0; i < (CFG_GPIO_INAMOUNT); i++){
                gpio_input_port_tb[i] = 1;
                wait(2000);
                data = CompleteWBRead(CFG_GPIO_BASE_ADDRESS);
                PN_INFOF(("Data is : %d", (uint8_t) data[i+CFG_GPIO_OUTAMOUNT]));
                PN_ASSERTM(data[i+CFG_GPIO_OUTAMOUNT] == gpio_input_port_tb[i],"IoRegs set wrong while reading the GPIO INs");
                wait(1);
            }
            PN_INFO("Reseting...");
            wb_rst_i = 1;
            // Reseting all the input pins
            for(size_t i = 0; i < (CFG_GPIO_INAMOUNT); i++){
                gpio_input_port_tb[i] = 0;
            }
            wait(10);
            wb_rst_i = 0;
            wait(2);
            data = CompleteWBRead(CFG_GPIO_BASE_ADDRESS);
            PN_INFOF(("Data is : 0x%08x", (uint32_t) data));
            PN_ASSERTM(data == 0x00, "Register reset didnt work");
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
    tb.wb_clk_i(clk);
    sc_start ();
    cout <<"\n\t\t*****Simulation complete*****" << endl;


    return 0;
}