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

#include "uart_transmitter.h"

#include <stdio.h>
#include <signal.h>
#include <systemc.h>

#define CLK_PERIOD 10.0

struct Tb : sc_module
{
    sc_in_clk clk{"clk"};                        // Clock
    sc_signal<bool> rst{"rst"};                  // Reset
    sc_signal<bool> tx_clk{"tx_clk"};            // Receiver clock(2x baudrate)
    sc_signal<bool> tx_start{"rx_start"};        // Start transmitter
    sc_signal<bool> clear{"clear"};              // Clear transmitter state
    sc_signal<sc_uint<2> > wls{"wls"};            // Word length select
    sc_signal<bool> stb{"stb"};                  // Number of stop bits
    sc_signal<bool> pen{"pen"};                  // Parity enable
    sc_signal<bool> eps{"eps"};                  // Even parity select
    sc_signal<bool> sp{"sp"};                    // Stick parity
    sc_signal<bool> bc{"bc"};                    // Break control
    sc_signal<sc_uint<8> > din{"din"};            // Input data
    sc_signal<bool> tx_finished{"tx_finished"};  // Transmitter operation finished
    sc_signal<bool> sout{"sout"};                // Transmitter output

    UartTransmitter uart_transmitter {"uart_transmitter"};


    SC_CTOR(Tb)
    {
        uart_transmitter.clk_i (clk);
        uart_transmitter.rst_i (rst);
        uart_transmitter.tx_clk_i (tx_clk);
        uart_transmitter.tx_start_i (tx_start);
        uart_transmitter.clear_i (clear);
        uart_transmitter.wls_i (wls);
        uart_transmitter.stb_i (stb);
        uart_transmitter.pen_i (pen);
        uart_transmitter.eps_i (eps);
        uart_transmitter.sp_i (sp);
        uart_transmitter.bc_i (bc);
        uart_transmitter.din_i (din);
        uart_transmitter.tx_finished_o (tx_finished);
        uart_transmitter.sout_o (sout);


        SC_CTHREAD(test_proc, clk.pos());
    }

    void test_proc(){


        #ifndef __SYNTHESIS__
        // Set cfg_debug_mode to suppress some simulation warnings
        pn_cfg_debug_mode = true;
        // Trace file ...
        sc_trace_file *tf;
        if (pn_cfg_vcd_level > 0) {
            tf = sc_create_vcd_trace_file ("uart_transmitter_tb");
            tf->delta_cycles (false);

            //PN_TRACE
            PN_TRACE(tf, clk);
            PN_TRACE(tf, rst);
            PN_TRACE(tf, tx_clk);
            PN_TRACE(tf, tx_start);
            PN_TRACE(tf, clear);
            PN_TRACE(tf, wls);
            PN_TRACE(tf, stb);
            PN_TRACE(tf, pen);
            PN_TRACE(tf, eps);
            PN_TRACE(tf, sp);
            PN_TRACE(tf, bc);
            PN_TRACE(tf, din);
            PN_TRACE(tf, tx_finished);
            PN_TRACE(tf, sout);

            uart_transmitter.Trace (tf, pn_cfg_vcd_level);
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
            PN_INFOF(("Output Data was:\t %d", sout.read()));
            PN_INFOF(("Transmitter  operation was:\t %d", tx_finished.read()));
            PN_ASSERTM (sout.read() == 1, "Output Data is zero" );
            PN_ASSERTM (tx_finished.read() == 0, "Receiver operation finished is not zero" );
            wait(10);

            bc = 1;
            wait(2);

            PN_INFO("Break Control");
            PN_INFOF(("Output Data was:\t %d", sout.read()));
            PN_INFOF(("Transmitter  operation was:\t %d", tx_finished.read()));
            PN_ASSERTM (sout.read() == 0, "Output Data is zero" );
            PN_ASSERTM (tx_finished.read() == 0, "Receiver operation finished is not zero" );
            wait(10);

            bc = 0;
            wait(2);

            PN_INFO("Break Control off");
            PN_INFOF(("Output Data was:\t %d", sout.read()));
            PN_INFOF(("Transmitter  operation was:\t %d", tx_finished.read()));
            PN_ASSERTM (sout.read() == 1, "Output Data is zero" );
            PN_ASSERTM (tx_finished.read() == 0, "Receiver operation finished is not zero" );
            wait(10);

            din = 0xaa;
            tx_start = 1;
            wls = 3;
            pen = 1;
            stb = 1;
            sp = 1;
            tx_clk = 1;
            wait(2);

            PN_INFO("Start Bit");
            PN_INFOF(("Output Data was:\t %d", sout.read()));
            PN_INFOF(("Transmitter  operation was:\t %d", tx_finished.read()));
            PN_ASSERTM (sout.read() == 0, "Output Data is zero" );
            PN_ASSERTM (tx_finished.read() == 0, "Receiver operation finished is not zero" );
            wait(2);

            PN_INFO("Bit0");
            PN_INFOF(("Output Data was:\t %d", sout.read()));
            PN_INFOF(("Transmitter  operation was:\t %d", tx_finished.read()));
            PN_ASSERTM (sout.read() == 0, "Output Data is zero" );
            PN_ASSERTM (tx_finished.read() == 0, "Receiver operation finished is not zero" );
            wait(2);

            PN_INFO("Bit1");
            PN_INFOF(("Output Data was:\t %d", sout.read()));
            PN_INFOF(("Transmitter  operation was:\t %d", tx_finished.read()));
            PN_ASSERTM (sout.read() == 1, "Output Data is not zero" );
            PN_ASSERTM (tx_finished.read() == 0, "Receiver operation finished is not zero" );
            wait(2);

            PN_INFO("Bit2");
            PN_INFOF(("Output Data was:\t %d", sout.read()));
            PN_INFOF(("Transmitter  operation was:\t %d", tx_finished.read()));
            PN_ASSERTM (sout.read() == 0, "Output Data is zero" );
            PN_ASSERTM (tx_finished.read() == 0, "Receiver operation finished is not zero" );
            wait(2);

            PN_INFO("Bit3");
            PN_INFOF(("Output Data was:\t %d", sout.read()));
            PN_INFOF(("Transmitter  operation was:\t %d", tx_finished.read()));
            PN_ASSERTM (sout.read() == 1, "Output Data is not zero" );
            PN_ASSERTM (tx_finished.read() == 0, "Receiver operation finished is not zero" );
            wait(2);

            PN_INFO("Bit4");
            PN_INFOF(("Output Data was:\t %d", sout.read()));
            PN_INFOF(("Transmitter  operation was:\t %d", tx_finished.read()));
            PN_ASSERTM (sout.read() == 0, "Output Data is zero" );
            PN_ASSERTM (tx_finished.read() == 0, "Receiver operation finished is not zero" );
            wait(2);

            PN_INFO("Bit5");
            PN_INFOF(("Output Data was:\t %d", sout.read()));
            PN_INFOF(("Transmitter  operation was:\t %d", tx_finished.read()));
            PN_ASSERTM (sout.read() == 1, "Output Data is not zero" );
            PN_ASSERTM (tx_finished.read() == 0, "Receiver operation finished is not zero" );
            wait(2);

            PN_INFO("Bit6");
            PN_INFOF(("Output Data was:\t %d", sout.read()));
            PN_INFOF(("Transmitter  operation was:\t %d", tx_finished.read()));
            PN_ASSERTM (sout.read() == 0, "Output Data is not zero" );
            PN_ASSERTM (tx_finished.read() == 0, "Receiver operation finished is not zero" );
            wait(2);

            PN_INFO("Bit7");
            PN_INFOF(("Output Data was:\t %d", sout.read()));
            PN_INFOF(("Transmitter  operation was:\t %d", tx_finished.read()));
            PN_ASSERTM (sout.read() == 1, "Output Data is zero" );
            PN_ASSERTM (tx_finished.read() == 0, "Receiver operation finished is not zero" );
            wait(2);

            PN_INFO("Parity Bit");
            PN_INFOF(("Output Data was:\t %d", sout.read()));
            PN_INFOF(("Transmitter  operation was:\t %d", tx_finished.read()));
            PN_ASSERTM (sout.read() == 1, "Output Data is zero" );
            PN_ASSERTM (tx_finished.read() == 0, "Receiver operation finished is not zero" );
            wait(2);

            PN_INFO("Stop Bit");
            PN_INFOF(("Output Data was:\t %d", sout.read()));
            PN_INFOF(("Transmitter  operation was:\t %d", tx_finished.read()));
            PN_ASSERTM (sout.read() == 1, "Output Data is zero" );
            PN_ASSERTM (tx_finished.read() == 0, "Receiver operation finished is not zero" );
            wait(1);

            PN_INFO("Stop Bit Second Clock");
            PN_INFOF(("Output Data was:\t %d", sout.read()));
            PN_INFOF(("Transmitter  operation was:\t %d", tx_finished.read()));
            PN_ASSERTM (sout.read() == 1, "Output Data is zero" );
            PN_ASSERTM (tx_finished.read() == 1, "Receiver operation finished is not zero" );
            wait(1);

            PN_INFO("Second Stop Bit");
            PN_INFOF(("Output Data was:\t %d", sout.read()));
            PN_INFOF(("Transmitter  operation was:\t %d", tx_finished.read()));
            PN_ASSERTM (sout.read() == 1, "Output Data is zero" );
            PN_ASSERTM (tx_finished.read() == 0, "Receiver operation finished is not zero" );
            wait(2);

            clear = 1;
            wait(2);

            PN_INFO("Clear");
            PN_INFOF(("Output Data was:\t %d", sout.read()));
            PN_INFOF(("Transmitter  operation was:\t %d", tx_finished.read()));
            PN_ASSERTM (sout.read() == 0, "Output Data is not zero" );
            PN_ASSERTM (tx_finished.read() == 0, "Receiver operation finished is not zero" );
            wait(2);

            rst = 1;
            wait(2);

            PN_INFO("Reset");
            PN_INFOF(("Output Data was:\t %d", sout.read()));
            PN_INFOF(("Transmitter  operation was:\t %d", tx_finished.read()));
            PN_ASSERTM (sout.read() == 1, "Output Data is zero" );
            PN_ASSERTM (tx_finished.read() == 0, "Receiver operation finished is not zero" );
            wait(20);

            rst = 0;
            tx_start = 0;
            wait(20);

            din = 0x3a;
            tx_start = 1;
            wls = 1;
            pen = 0;
            stb = 0;
            sp = 0;
            wait(2);

            PN_INFO("Bit0");
            PN_INFOF(("Output Data was:\t %d", sout.read()));
            PN_INFOF(("Transmitter  operation was:\t %d", tx_finished.read()));
            PN_ASSERTM (sout.read() == 0, "Output Data is zero" );
            PN_ASSERTM (tx_finished.read() == 0, "Receiver operation finished is not zero" );
            wait(2);

            PN_INFO("Bit1");
            PN_INFOF(("Output Data was:\t %d", sout.read()));
            PN_INFOF(("Transmitter  operation was:\t %d", tx_finished.read()));
            PN_ASSERTM (sout.read() == 0, "Output Data is not zero" );
            PN_ASSERTM (tx_finished.read() == 0, "Receiver operation finished is not zero" );
            wait(2);

            PN_INFO("Bit2");
            PN_INFOF(("Output Data was:\t %d", sout.read()));
            PN_INFOF(("Transmitter  operation was:\t %d", tx_finished.read()));
            PN_ASSERTM (sout.read() == 1, "Output Data is zero" );
            PN_ASSERTM (tx_finished.read() == 0, "Receiver operation finished is not zero" );
            wait(2);

            PN_INFO("Bit3");
            PN_INFOF(("Output Data was:\t %d", sout.read()));
            PN_INFOF(("Transmitter  operation was:\t %d", tx_finished.read()));
            PN_ASSERTM (sout.read() == 0, "Output Data is not zero" );
            PN_ASSERTM (tx_finished.read() == 0, "Receiver operation finished is not zero" );
            wait(2);

            PN_INFO("Bit4");
            PN_INFOF(("Output Data was:\t %d", sout.read()));
            PN_INFOF(("Transmitter  operation was:\t %d", tx_finished.read()));
            PN_ASSERTM (sout.read() == 1, "Output Data is zero" );
            PN_ASSERTM (tx_finished.read() == 0, "Receiver operation finished is not zero" );
            wait(2);

            PN_INFO("Bit5");
            PN_INFOF(("Output Data was:\t %d", sout.read()));
            PN_INFOF(("Transmitter  operation was:\t %d", tx_finished.read()));
            PN_ASSERTM (sout.read() == 1, "Output Data is not zero" );
            PN_ASSERTM (tx_finished.read() == 0, "Receiver operation finished is not zero" );
            wait(2);

            PN_INFO("Parity Bit");
            PN_INFOF(("Output Data was:\t %d", sout.read()));
            PN_INFOF(("Transmitter  operation was:\t %d", tx_finished.read()));
            PN_ASSERTM (sout.read() == 1, "Output Data is zero" );
            PN_ASSERTM (tx_finished.read() == 0, "Receiver operation finished is not zero" );
            wait(2);

            PN_INFO("Stop Bit");
            PN_INFOF(("Output Data was:\t %d", sout.read()));
            PN_INFOF(("Transmitter  operation was:\t %d", tx_finished.read()));
            PN_ASSERTM (sout.read() == 1, "Output Data is zero" );
            PN_ASSERTM (tx_finished.read() == 0, "Receiver operation finished is not zero" );
            wait(1);

            PN_INFO("Stop Bit Second Clock");
            PN_INFOF(("Output Data was:\t %d", sout.read()));
            PN_INFOF(("Transmitter  operation was:\t %d", tx_finished.read()));
            PN_ASSERTM (sout.read() == 1, "Output Data is zero" );
            PN_ASSERTM (tx_finished.read() == 1, "Receiver operation finished is not zero" );
            wait(1);

            wait(20);



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