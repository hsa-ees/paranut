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

#include "uart_interrupt.h"

#include <stdio.h>
#include <signal.h>
#include <systemc.h>

#define CLK_PERIOD 10.0

struct Tb : sc_module
{
    sc_in_clk clk {"clk"};
    sc_signal<bool> rst {"rst"};
    sc_signal<sc_uint<4> > ire {"ire"}; // IER 3:0
    sc_signal<sc_uint<5> > lsr {"lsr"}; // LSR 4:0
    sc_signal<bool> thi {"thi"}; // Transmitter holding register empty interrupt
    sc_signal<bool> rda {"rda"}; // Receiver data available
    sc_signal<bool> cti {"cti"}; // Character timeout indication
    sc_signal<bool> afe {"afe"}; // Automatic flow control enable
    sc_signal<sc_uint<4> > msr {"msr"}; // MSR 3:0
    sc_signal<sc_uint<4> > iir {"iir"}; // IIR 3:0
    sc_signal<bool> int_i_o  {"int_i_o"}; // Interrupt

    UartInterrupt uart_interrupt {"uart_interrupt"};


    SC_CTOR(Tb)
    {
        uart_interrupt.clk_i (clk);
        uart_interrupt.rst_i (rst);
        uart_interrupt.ire_i (ire);
        uart_interrupt.lsr_i (lsr);
        uart_interrupt.thi_i (thi);
        uart_interrupt.rda_i (rda);
        uart_interrupt.cti_i (cti);
        uart_interrupt.afe_i (afe);
        uart_interrupt.msr_i (msr);
        uart_interrupt.iir_o (iir);
        uart_interrupt.int_o (int_i_o);

        SC_CTHREAD(test_proc, clk.pos());
    }

    void test_proc(){


        #ifndef __SYNTHESIS__
        // Set cfg_debug_mode to suppress some simulation warnings
        pn_cfg_debug_mode = true;
        // Trace file ...
        sc_trace_file *tf;
        if (pn_cfg_vcd_level > 0) {
            tf = sc_create_vcd_trace_file ("uart_interrupt_tb");
            tf->delta_cycles (false);

            //PN_TRACE
            PN_TRACE(tf, clk);
            PN_TRACE(tf, rst);
            PN_TRACE(tf, ire);
            PN_TRACE(tf, lsr);
            PN_TRACE(tf, thi);
            PN_TRACE(tf, rda);
            PN_TRACE(tf, cti);
            PN_TRACE(tf, afe);
            PN_TRACE(tf, msr);
            PN_TRACE(tf, iir);
            PN_TRACE(tf, int_i_o);


            uart_interrupt.Trace (tf, pn_cfg_vcd_level);
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
            PN_INFOF(("Interrupt was:\t %d", int_i_o.read()));
            PN_INFOF(("IIR was:\t 0x%01x", (uint16_t) iir.read()));
            PN_ASSERTM (int_i_o.read() == 0, "Interrupt is zero.");
            PN_ASSERTM (iir.read() == 1, "IIR is not 0001" );
            wait(10);

            ire = 1;
            cti = 1;

            wait(2);

            PN_INFO("Character timeout indication");
            PN_INFOF(("Interrupt was:\t %d", int_i_o.read()));
            PN_INFOF(("IIR was:\t 0x%01x", (uint16_t) iir.read()));
            PN_ASSERTM (int_i_o.read() == 1, "Interrupt is zero.");
            PN_ASSERTM (iir.read() == 0xC, "IIR is not 0xC" );
            wait(10);

            ire = 4;
            rda = 1;
            cti = 0;
            wait(2);

            PN_INFO("Received data available or trigger level reached in Fifo ");
            PN_INFOF(("Interrupt was:\t %d", int_i_o.read()));
            PN_INFOF(("IIR was:\t 0x%01x", (uint16_t) iir.read()));
            PN_ASSERTM (int_i_o.read() == 1, "Interrupt is zero.");
            PN_ASSERTM (iir.read() == 0x4, "IIR is not 0x4" );
            wait(10);

            ire = 4;
            lsr = 0xF;
            wait(2);

            PN_INFO("Receiver line status on && Received data available or trigger level reached in Fifo ");
            PN_INFOF(("Interrupt was:\t %d", int_i_o.read()));
            PN_INFOF(("IIR was:\t 0x%01x", (uint16_t) iir.read()));
            PN_ASSERTM (int_i_o.read() == 1, "Interrupt is zero.");
            PN_ASSERTM (iir.read() == 0x6, "IIR is not 0x6" );
            wait(10);

            rda = 0;

            for(int i = 2; i <= 0xF; i++){
                lsr = i;
                wait(2);
                PN_INFO("Receiver line status on testing each Error");
                PN_INFOF(("LSR was: \t 0x%01x", (uint16_t) lsr.read()));
                PN_INFOF(("Interrupt was:\t %d", int_i_o.read()));
                PN_INFOF(("IIR was:\t 0x%01x", (uint16_t) iir.read()));
                PN_ASSERTM (int_i_o.read() == 1, "Interrupt is zero.");
                PN_ASSERTM (iir.read() == 0x6, "IIR is not 0x6" );
            }
            wait(10);

            lsr = 0;
            ire = 2;
            thi = 1;
            wait(2);

            PN_INFO("Transmitter holding register empty");
            PN_INFOF(("Interrupt was:\t %d", int_i_o.read()));
            PN_INFOF(("IIR was:\t 0x%01x", (uint16_t) iir.read()));
            PN_ASSERTM (int_i_o.read() == 1, "Interrupt is zero.");
            PN_ASSERTM (iir.read() == 0x2, "IIR is not 0x4" );
            wait(10);

            rst = 1;
            wait(2);
            PN_INFO("Reset");
            PN_INFOF(("Interrupt was:\t %d", int_i_o.read()));
            PN_INFOF(("IIR was:\t 0x%01x", (uint16_t) iir.read()));
            PN_ASSERTM (int_i_o.read() == 0, "Interrupt is not zero.");
            PN_ASSERTM (iir.read() == 0x1, "IIR is not 0x4" );

            wait(2);

            PN_INFO("Reset check again");
            PN_INFOF(("Interrupt was:\t %d", int_i_o.read()));
            PN_INFOF(("IIR was:\t 0x%01x", (uint16_t) iir.read()));
            PN_ASSERTM (int_i_o.read() == 0, "Interrupt is not zero.");
            PN_ASSERTM (iir.read() == 0x1, "IIR is not 0x4" );
            wait(10);

            ire = 8;
            msr = 1;
            afe = 0;
            rst = 0;
            wait(2);

            PN_INFO("Modem status intterupt dCTs(AFC is disabled)");
            PN_INFOF(("Interrupt was:\t %d", int_i_o.read()));
            PN_INFOF(("IIR was:\t 0x%01x", (uint16_t) iir.read()));
            PN_ASSERTM (int_i_o.read() == 1, "Interrupt is not zero.");
            PN_ASSERTM (iir.read() == 0x0, "IIR is not 0x4" );
            wait(10);

            for(int i = 2; i <= 0xF; i++){
                msr = i;
                wait(2);

                PN_INFO("Modem Status Interrupt");
                PN_INFOF(("MSR was: \t 0x%01x", (uint16_t) msr.read()));
                PN_INFOF(("Interrupt was:\t %d", int_i_o.read()));
                PN_INFOF(("IIR was:\t 0x%01x", (uint16_t) iir.read()));
                PN_ASSERTM (int_i_o.read() == 1, "Interrupt is zero.");
                PN_ASSERTM (iir.read() == 0x0, "IIR is not 0x6" );
            }
            wait(10);

            ire = 0;
            msr = 0;
            wait(2);

            PN_INFO("No Interrupts");
            PN_INFOF(("Interrupt was:\t %d", int_i_o.read()));
            PN_INFOF(("IIR was:\t 0x%01x", (uint16_t) iir.read()));
            PN_ASSERTM (int_i_o.read() == 0, "Interrupt is not zero.");
            PN_ASSERTM (iir.read() == 0x1, "IIR is not 0x4" );
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