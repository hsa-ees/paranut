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

#include "slib_counter.h"

#include <stdio.h>
#include <signal.h>
#include <systemc.h>

#define CLK_PERIOD 10.0

struct Tb : sc_module
{
    sc_in_clk clk {"clk"};
    sc_signal<bool> rst {"rst"};
    sc_signal<bool> clear {"clear"};
    sc_signal<bool> load {"load"};
    sc_signal<bool> enable {"enable"};
    sc_signal<bool> down {"down"};
    sc_signal<sc_uint<SLIB_COUNTER_WIDTH> > d {"d"};
    sc_signal<sc_uint<SLIB_COUNTER_WIDTH> > q {"q"};
    sc_signal<bool> overflow {"overflow"};

    SlibCounter slib_cnt {"slib_cnt"};


    SC_CTOR(Tb)
    {
        slib_cnt.clk_i (clk);
        slib_cnt.rst_i (rst);
        slib_cnt.clear_i (clear);
        slib_cnt.load_i (load);
        slib_cnt.enable_i (enable);
        slib_cnt.down_i (down);
        slib_cnt.d_i (d);
        slib_cnt.overflow_o (overflow);
        slib_cnt.q_o (q);

        SC_CTHREAD(test_proc, clk.pos());
    }

    void test_proc(){


        #ifndef __SYNTHESIS__
        // Set cfg_debug_mode to suppress some simulation warnings
        pn_cfg_debug_mode = true;
        // Trace file ...
        sc_trace_file *tf;
        if (pn_cfg_vcd_level > 0) {
            tf = sc_create_vcd_trace_file ("slib_counter_tb");
            tf->delta_cycles (false);

            //PN_TRACE
            PN_TRACE(tf, clk);
            PN_TRACE(tf, rst);
            PN_TRACE(tf, clear);
            PN_TRACE(tf, load);
            PN_TRACE(tf, enable);
            PN_TRACE(tf, down);
            PN_TRACE(tf, d);
            PN_TRACE(tf, q);
            PN_TRACE(tf, overflow);

            slib_cnt.Trace (tf, pn_cfg_vcd_level);
        } else {
            fprintf (stderr, "Tracing is disabled.\n");
            tf = NULL;
        }

        PN_INFO("Reseting...");
        rst = 1;
        wait(10);
        rst = 0;
        wait();


        while(true){
            PN_INFO("Running");
            PN_INFOF(("Q was:\t 0x%02x", (uint8_t)q.read()));
            PN_ASSERTM (q.read() == sc_uint<SLIB_COUNTER_WIDTH>(0), "Q is not zero.");
            wait(2);

            d = sc_uint<SLIB_COUNTER_WIDTH>(2);
            load = 1;
            wait(1);
            load = 0;
            wait(1);
            PN_INFO("Loaded Counter");
            PN_INFOF(("Q was:\t 0x%02x", (uint8_t)q.read()));
            PN_ASSERTM (q.read() == sc_uint<SLIB_COUNTER_WIDTH>(2), "Q is not 2 while it should have counted up two times.");


            enable = 1;
            down = 1;
            wait(1);
            PN_INFO ("Counter counting down.");
            wait(1);
            PN_INFOF(("Q was:\t 0x%02x", (uint8_t)q.read()));
            PN_ASSERTM (q.read() == sc_uint<SLIB_COUNTER_WIDTH>(1), "Q is not 1 after counting down once.");

            wait(2);
            PN_INFO ("Counter overflow into the negative.");
            PN_INFOF(("Q was:\t 0x%02x", (uint8_t)q.read()));
            PN_ASSERTM (overflow.read() == 1, "Overflow isn't set even though we reached an overflow");
            PN_ASSERTM (q.read() == sc_uint<SLIB_COUNTER_WIDTH>(0xf), "Q not 0xf after counting over 0x0 and going to overflow");

            clear = 1;
            down = 0;
            wait(1);
            clear = 0;
            wait(1);
            PN_INFO ("Counter counting up from 0x0 after clearing.");
            PN_INFOF(("Q was:\t 0x%02x", (uint8_t)q.read()));
            PN_ASSERTM (q.read() == sc_uint<SLIB_COUNTER_WIDTH>(0x0), "Q is not 0 after clearing our count");

            wait(8);

            PN_INFO ("Counter counted up 8 clock cycles.");
            PN_INFOF(("Q was:\t 0x%02x", (uint8_t)q.read()));
            PN_ASSERTM (q.read() == sc_uint<SLIB_COUNTER_WIDTH>(0x8), "Q is not 8 after 8 clock counts counting up");

            enable = 0;
            wait(2);
            PN_INFO ("Counter beein halted.");
            PN_INFOF(("Q was:\t 0x%02x", (uint8_t)q.read()));
            PN_ASSERTM (q.read() == sc_uint<SLIB_COUNTER_WIDTH>(0x9), "Q is not 9 after beein halted after one clock cycle");
            enable = 1;
            wait(1);

            wait(6);
            PN_INFO ("Counter should have reached 0x0F.");
            PN_INFOF(("Q was:\t 0x%02x", (uint8_t)q.read()));
            PN_ASSERTM (overflow.read() == 0, "Overflow is set even though we haven't reached an overflow");
            PN_ASSERTM (q.read() == sc_uint<SLIB_COUNTER_WIDTH>(0xF), "Q not 0xF after counting 16 clock counts on enable high");


            wait(1);
            PN_INFO ("Counter should have reached overflow.");
            PN_INFOF(("Q was:\t 0x%02x", (uint8_t)q.read()));
            PN_ASSERTM (overflow.read() == 1, "Overflow isn't set even though we reached an overflow");
            PN_ASSERTM (q.read() == sc_uint<SLIB_COUNTER_WIDTH>(0x0), "Q not 0x0 after counting over 0xF and going to overflow");

            rst = 1;
            wait(2);

            PN_INFO ("Counter was reset.");
            PN_INFOF(("Q was:\t 0x%02x", (uint8_t)q.read()));
            PN_ASSERTM (overflow.read() == 0, "Overflow isn't set even though we reached an overflow");
            PN_ASSERTM (q.read() == sc_uint<SLIB_COUNTER_WIDTH>(0x0), "Q not 0x0 after counting over 0xF and going to overflow");

            rst = 0;
            wait(5);
            PN_INFO ("Counter counting again.");
            PN_INFOF(("Q was:\t 0x%02x", (uint8_t)q.read()));
            PN_ASSERTM (overflow.read() == 0, "Overflow isn't set even though we reached an overflow");
            PN_ASSERTM (q.read() == sc_uint<SLIB_COUNTER_WIDTH>(0x4), "Q not 0x0 after counting over 0xF and going to overflow");



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
    sc_start();
    cout <<"\n\t\t*****Simulation complete*****" << endl;


    return 0;
}