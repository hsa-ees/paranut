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

#include "slib_clock_div.h"

#include <stdio.h>
#include <signal.h>
#include <systemc.h>

#define CLK_PERIOD 10.0

struct Tb : sc_module
{
    sc_in_clk clk {"clk"};
    sc_signal<bool> rst {"rst"};
    sc_signal<bool> ce {"ce"};
    sc_signal<bool> q {"q"};

    SlibClockDiv slib_clk_div {"slib_clk_div"};


    SC_CTOR(Tb)
    {
        slib_clk_div.clk_i (clk);
        slib_clk_div.rst_i (rst);
        slib_clk_div.ce_i (ce);
        slib_clk_div.q_o (q);

        SC_CTHREAD(test_proc, clk.pos());
    }

    void test_proc(){


        #ifndef __SYNTHESIS__
        // Set cfg_debug_mode to suppress some simulation warnings
        pn_cfg_debug_mode = true;
        // Trace file ...
        sc_trace_file *tf;
        if (pn_cfg_vcd_level > 0) {
            tf = sc_create_vcd_trace_file ("slib_clock_div_tb");
            tf->delta_cycles (false);

            PN_TRACE (tf, rst);
            PN_TRACE (tf, clk);
            PN_TRACE (tf, ce);
            PN_TRACE (tf, q);


            slib_clk_div.Trace (tf, pn_cfg_vcd_level);
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
            PN_INFOF(("Q was:\t %d", q.read()));
            PN_ASSERTM (q.read() == 0, "Q high while reset low and no other signals.");
            wait(2);


            PN_INFO("Counting Test");
            ce = 1;
            wait(1);
            PN_INFOF(("Q was:\t %d", q.read()));
            PN_ASSERTM (q.read() == 0, "Q high while reset low and clock enable high.");
            wait(SLIB_CLOCKDIV_RATIO);
            PN_INFO("Reached CLOCKDIV_RATIO");
            PN_INFOF(("Q was:\t %d", q.read()));
            PN_ASSERTM (q.read() == 1, "Q low after enough rates in clock enable high.");


            wait(SLIB_CLOCKDIV_RATIO-2);
            rst = 1;
            wait(2);
            PN_INFO("Reached CLOCKDIV_RATIO while rst on");
            PN_INFOF(("Q was:\t %d", q.read()));
            PN_ASSERTM (q.read() == 0, "Q low after enough rates in clock enable high.");


            rst = 0;
            wait(1);
            PN_INFO("Testing Reset to low while clock enable");
            PN_INFOF(("Q was:\t %d", q.read()));
            PN_ASSERTM (q.read() == 0, "Q low after enough rates in clock enable high.");

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