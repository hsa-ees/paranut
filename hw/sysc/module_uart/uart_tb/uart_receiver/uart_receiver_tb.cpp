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

#include "uart_receiver.h"

#include <stdio.h>
#include <signal.h>
#include <systemc.h>

#define CLK_PERIOD 10.0

struct Tb : sc_module
{
    sc_in_clk clk {"clk"};    // Clock
    sc_signal<bool> rst {"rst"};  // Reset
    sc_signal<bool> rx_clk {"rx_clk"}; // Receiver clock(16x baudrate)
    sc_signal<bool> rx_clear {"rx_clear"}; // Reset receiver state
    sc_signal<sc_uint<2> > wls {"wls"}; // Word length select
    sc_signal<bool> stb {"stb"}; // Number of stop bits
    sc_signal<bool> pen {"pen"}; // Parity enable
    sc_signal<bool> eps {"eps"}; // Even parity select
    sc_signal<bool> sp {"sp"}; // Stick parity
    sc_signal<bool> sin {"sin"}; // Receiver input
    sc_signal<bool> pe {"pe"}; // Parity error
    sc_signal<bool> fe  {"fe"}; // Framing error
    sc_signal<bool> bi  {"bi"}; // Break interrupt
    sc_signal<sc_uint<8> > dout  {"dout"}; // Output data
    sc_signal<bool> rx_finished  {"rx_finished"}; // Receiver operation finished

    UartReceiver uart_receiver {"uart_receiver"};


    SC_CTOR(Tb)
    {

        uart_receiver.clk_i (clk);
        uart_receiver.rst_i (rst);
        uart_receiver.rx_clk_i (rx_clk);
        uart_receiver.rx_clear_i (rx_clear);
        uart_receiver.wls_i (wls);
        uart_receiver.stb_i (stb);
        uart_receiver.pen_i (pen);
        uart_receiver.eps_i (eps);
        uart_receiver.sp_i (sp);
        uart_receiver.sin_i (sin);
        uart_receiver.pe_o (pe);
        uart_receiver.fe_o (fe);
        uart_receiver.bi_o (bi);
        uart_receiver.dout_o (dout);
        uart_receiver.rx_finished_o (rx_finished);

        SC_CTHREAD(test_proc, clk.pos());
    }

    void rx_clock_step(){
        for(int i = 0; i < 16; i++){
            wait(27);
            rx_clk = 1;
            wait(1);
            rx_clk = 0;
            wait(26);
        }
    }
    void test_proc(){


        #ifndef __SYNTHESIS__
        // Set cfg_debug_mode to suppress some simulation warnings
        pn_cfg_debug_mode = true;
        // Trace file ...
        sc_trace_file *tf;
        if (pn_cfg_vcd_level > 0) {
            tf = sc_create_vcd_trace_file ("uart_receiver_tb");
            tf->delta_cycles (false);

            //PN_TRACE
            PN_TRACE(tf, clk);    // Clock
            PN_TRACE(tf, rst);    // Reset
            PN_TRACE(tf, rx_clk); // Receiver clock(16x baudrate)
            PN_TRACE(tf, rx_clear);   // Reset receiver state
            PN_TRACE(tf, wls);    // Word length select
            PN_TRACE(tf, stb);    // Number of stop bits
            PN_TRACE(tf, pen);    // Parity enable
            PN_TRACE(tf, eps);    // Even parity select
            PN_TRACE(tf, sp); // Stick parity
            PN_TRACE(tf, sin);    // Receiver input
            PN_TRACE(tf, pe); // Parity error
            PN_TRACE(tf, fe); // Framing error
            PN_TRACE(tf, bi); // Break interrupt
            PN_TRACE(tf, dout);   // Output data
            PN_TRACE(tf, rx_finished);    // Receiver operation finished


            uart_receiver.Trace (tf, pn_cfg_vcd_level);
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
            PN_INFOF(("Parity Error was:\t %d", pe.read()));
            PN_INFOF(("Framing Error was:\t %d", fe.read()));
            PN_INFOF(("Break Interrupt was:\t %d", bi.read()));
            PN_INFOF(("Output Data was:\t 0x%03x", (u_int16_t) dout.read()));
            PN_INFOF(("Receiver operation was:\t %d", rx_finished.read()));
            PN_ASSERTM (pe.read() == 0, "Parity Error is not zero.");
            PN_ASSERTM (fe.read() == 0, "Framing Error is not zero" );
            PN_ASSERTM (bi.read() == 0, "Break Interrupt is not zero.");
            PN_ASSERTM (dout.read() == 0, "Output Data is not zero" );
            PN_ASSERTM (rx_finished.read() == 0, "Receiver operation finished is not zero" );
            wait(10);

            wls = 3;
            pen = 1;
            sin = 0;
            rx_clock_step();

            PN_INFO("StartbitReceiving");
            PN_INFOF(("Parity Error was:\t %d", pe.read()));
            PN_INFOF(("Framing Error was:\t %d", fe.read()));
            PN_INFOF(("Break Interrupt was:\t %d", bi.read()));
            PN_INFOF(("Output Data was:\t 0x%03x", (u_int16_t) dout.read()));
            PN_INFOF(("Receiver operation was:\t %d", rx_finished.read()));
            PN_ASSERTM (fe.read() == 0, "Framing Error is not zero" );
            PN_ASSERTM (bi.read() == 0, "Break Interrupt is not zero.");
            PN_ASSERTM (dout.read() == 0, "Output Data is not zero" );
            PN_ASSERTM (rx_finished.read() == 0, "Receiver operation finished is not zero" );

            sin = 1;
            rx_clock_step();

            PN_INFO("First Data Bit");
            PN_INFOF(("Parity Error was:\t %d", pe.read()));
            PN_INFOF(("Framing Error was:\t %d", fe.read()));
            PN_INFOF(("Break Interrupt was:\t %d", bi.read()));
            PN_INFOF(("Output Data was:\t 0x%03x", (u_int16_t) dout.read()));
            PN_INFOF(("Receiver operation was:\t %d", rx_finished.read()));
            PN_ASSERTM (fe.read() == 0, "Framing Error is not zero" );
            PN_ASSERTM (bi.read() == 0, "Break Interrupt is zero.");
            PN_ASSERTM (dout.read() == 1, "Output Data is not zero" );
            PN_ASSERTM (rx_finished.read() == 0, "Receiver operation finished is not zero" );

            sin = 1;
            rx_clock_step();

            PN_INFO("Second Data Bit");
            PN_INFOF(("Parity Error was:\t %d", pe.read()));
            PN_INFOF(("Framing Error was:\t %d", fe.read()));
            PN_INFOF(("Break Interrupt was:\t %d", bi.read()));
            PN_INFOF(("Output Data was:\t 0x%03x", (u_int16_t) dout.read()));
            PN_INFOF(("Receiver operation was:\t %d", rx_finished.read()));
            PN_ASSERTM (fe.read() == 0, "Framing Error is not zero" );
            PN_ASSERTM (bi.read() == 0, "Break Interrupt is not zero.");
            PN_ASSERTM (dout.read() == 0x3, "Output Data is zero" );
            PN_ASSERTM (rx_finished.read() == 0, "Receiver operation finished is not zero" );

            sin = 0;
            rx_clock_step();

            PN_INFO("Third Data Bit");
            PN_INFOF(("Parity Error was:\t %d", pe.read()));
            PN_INFOF(("Framing Error was:\t %d", fe.read()));
            PN_INFOF(("Break Interrupt was:\t %d", bi.read()));
            PN_INFOF(("Output Data was:\t 0x%03x", (u_int16_t) dout.read()));
            PN_INFOF(("Receiver operation was:\t %d", rx_finished.read()));
            PN_ASSERTM (fe.read() == 0, "Framing Error is not zero" );
            PN_ASSERTM (bi.read() == 0, "Break Interrupt is not zero.");
            PN_ASSERTM (dout.read() == 0x3, "Output Data is zero" );
            PN_ASSERTM (rx_finished.read() == 0, "Receiver operation finished is not zero" );

            sin = 0;
            rx_clock_step();

            PN_INFO("Forth Data Bit");
            PN_INFOF(("Parity Error was:\t %d", pe.read()));
            PN_INFOF(("Framing Error was:\t %d", fe.read()));
            PN_INFOF(("Break Interrupt was:\t %d", bi.read()));
            PN_INFOF(("Output Data was:\t 0x%03x", (u_int16_t) dout.read()));
            PN_INFOF(("Receiver operation was:\t %d", rx_finished.read()));
            PN_ASSERTM (fe.read() == 0, "Framing Error is not zero" );
            PN_ASSERTM (bi.read() == 0, "Break Interrupt is not zero.");
            PN_ASSERTM (dout.read() == 0x3, "Output Data is zero" );
            PN_ASSERTM (rx_finished.read() == 0, "Receiver operation finished is not zero" );

            sin = 0;
            rx_clock_step();

            PN_INFO("Fifth Data Bit");
            PN_INFOF(("Parity Error was:\t %d", pe.read()));
            PN_INFOF(("Framing Error was:\t %d", fe.read()));
            PN_INFOF(("Break Interrupt was:\t %d", bi.read()));
            PN_INFOF(("Output Data was:\t 0x%03x", (u_int16_t) dout.read()));
            PN_INFOF(("Receiver operation was:\t %d", rx_finished.read()));
            PN_ASSERTM (fe.read() == 0, "Framing Error is not zero" );
            PN_ASSERTM (bi.read() == 0, "Break Interrupt is not zero.");
            PN_ASSERTM (dout.read() == 0x3, "Output Data is zero" );
            PN_ASSERTM (rx_finished.read() == 0, "Receiver operation finished is not zero" );

            sin = 1;
            rx_clock_step();

            PN_INFO("Sixted Data Bit");
            PN_INFOF(("Parity Error was:\t %d", pe.read()));
            PN_INFOF(("Framing Error was:\t %d", fe.read()));
            PN_INFOF(("Break Interrupt was:\t %d", bi.read()));
            PN_INFOF(("Output Data was:\t 0x%03x", (u_int16_t) dout.read()));
            PN_INFOF(("Receiver operation was:\t %d", rx_finished.read()));
            PN_ASSERTM (fe.read() == 0, "Framing Error is not zero" );
            PN_ASSERTM (bi.read() == 0, "Break Interrupt is not zero.");
            PN_ASSERTM (dout.read() == 0x23, "Output Data is zero" );
            PN_ASSERTM (rx_finished.read() == 0, "Receiver operation finished is not zero" );

            sin = 0;
            rx_clock_step();

            PN_INFO("Seven Data Bit");
            PN_INFOF(("Parity Error was:\t %d", pe.read()));
            PN_INFOF(("Framing Error was:\t %d", fe.read()));
            PN_INFOF(("Break Interrupt was:\t %d", bi.read()));
            PN_INFOF(("Output Data was:\t 0x%03x", (u_int16_t) dout.read()));
            PN_INFOF(("Receiver operation was:\t %d", rx_finished.read()));
            PN_ASSERTM (fe.read() == 0, "Framing Error is not zero" );
            PN_ASSERTM (bi.read() == 0, "Break Interrupt is not zero.");
            PN_ASSERTM (dout.read() == 0x23, "Output Data is zero" );
            PN_ASSERTM (rx_finished.read() == 0, "Receiver operation finished is not zero" );

            sin = 1;
            rx_clock_step();

            PN_INFO("Eigth Data Bit");
            PN_INFOF(("Parity Error was:\t %d", pe.read()));
            PN_INFOF(("Framing Error was:\t %d", fe.read()));
            PN_INFOF(("Break Interrupt was:\t %d", bi.read()));
            PN_INFOF(("Output Data was:\t 0x%03x", (u_int16_t) dout.read()));
            PN_INFOF(("Receiver operation was:\t %d", rx_finished.read()));
            PN_ASSERTM (pe.read() == 1, "Parity Error is not zero.");
            PN_ASSERTM (fe.read() == 0, "Framing Error is not zero" );
            PN_ASSERTM (bi.read() == 0, "Break Interrupt is not zero.");
            PN_ASSERTM (dout.read() == 0xa3, "Output Data is zero" );
            PN_ASSERTM (rx_finished.read() == 0, "Receiver operation finished is not zero" );

            sin = 1;
            rx_clock_step();

            PN_INFO("Stop Bit");
            PN_INFOF(("Parity Error was:\t %d", pe.read()));
            PN_INFOF(("Framing Error was:\t %d", fe.read()));
            PN_INFOF(("Break Interrupt was:\t %d", bi.read()));
            PN_INFOF(("Output Data was:\t 0x%03x", (u_int16_t) dout.read()));
            PN_INFOF(("Receiver operation was:\t %d", rx_finished.read()));
            PN_ASSERTM (fe.read() == 0, "Framing Error is not zero" );
            PN_ASSERTM (bi.read() == 0, "Break Interrupt is not zero.");
            PN_ASSERTM (dout.read() == 0xa3, "Output Data is zero" );
            PN_ASSERTM (rx_finished.read() == 0, "Receiver operation finished is not zero" );

            rx_clock_step();

            PN_INFO("Ended");
            PN_INFOF(("Parity Error was:\t %d", pe.read()));
            PN_INFOF(("Framing Error was:\t %d", fe.read()));
            PN_INFOF(("Break Interrupt was:\t %d", bi.read()));
            PN_INFOF(("Output Data was:\t 0x%03x", (u_int16_t) dout.read()));
            PN_INFOF(("Receiver operation was:\t %d", rx_finished.read()));
            PN_ASSERTM (fe.read() == 0, "Framing Error is not zero" );
            PN_ASSERTM (bi.read() == 0, "Break Interrupt is not zero.");
            PN_ASSERTM (dout.read() == 0x000, "Output Data is zero" );
            PN_ASSERTM (rx_finished.read() == 0, "Receiver operation finished is not zero" );

            sin = 0;
            wait(10);


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