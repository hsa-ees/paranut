/*************************************************************************

  This file is part of the ParaNut project.

  Copyright (C) 2022 Christian Meyer <christian.meyer@hs-augsburg.de>
      Hochschule Augsburg, University of Applied Sciences

  Description:
    This is a test bench for the translation lookaside buffer (TLB) of the ParaNut.

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

 *************************************************************************/

#include <systemc.h>
#include "paranut-config.h"
#include "tlb.h"

#include <random>

#include <stdio.h>
#include <stdlib.h>

// **************** Signals *********************


sc_signal<bool> clk, reset;

sc_signal<bool> tlb_flush;

// from/to PTW
sc_signal<bool> ptw_req, ptw_wr;
sc_signal<sc_uint<20> > ptw_va_i;
sc_signal<sc_uint<20> > ptw_pa_i;
sc_signal<bool> ptw_superpage_i;
sc_signal<bool> ptw_ac_r_i;
sc_signal<bool> ptw_ac_w_i;
sc_signal<bool> ptw_ac_x_i;
sc_signal<bool> ptw_ac_u_i;
sc_signal<bool> ptw_ac_a_i;
sc_signal<bool> ptw_ac_d_i;

sc_signal<bool> ptw_superpage_o;
sc_signal<sc_uint<20> > ptw_adr_o;
sc_signal<bool> ptw_hit_o;
sc_signal<bool> ptw_miss_o;

sc_signal<bool> ptw_ac_r_o;
sc_signal<bool> ptw_ac_w_o;
sc_signal<bool> ptw_ac_x_o;
sc_signal<bool> ptw_ac_u_o;
sc_signal<bool> ptw_ac_a_o;
sc_signal<bool> ptw_ac_d_o;


// **************** Helpers *********************

typedef struct {
    sc_uint<20> va;
    sc_uint<20> pa;
    bool superpage;
    bool a;
    bool d;
    bool r;
    bool w;
    bool x;
    bool u;
} adr_t;

#define TEST_COUNT CFG_MMU_TLB_ENTRIES*2

#define CLK_PERIOD 10.0
//#define CLK_PERIOD (sc_time (10.0, SC_NS).to_double ())

    sc_trace_file *tf = NULL;
void trace (sc_trace_file *tf) {
    PN_TRACE(tf, clk);
    PN_TRACE(tf, reset);
    PN_TRACE(tf, tlb_flush);
    PN_TRACE(tf, ptw_req);
    PN_TRACE(tf, ptw_wr);
    PN_TRACE(tf, ptw_va_i);
    PN_TRACE(tf, ptw_pa_i);
    PN_TRACE(tf, ptw_superpage_i);
    PN_TRACE(tf, ptw_ac_r_i);
    PN_TRACE(tf, ptw_ac_w_i);
    PN_TRACE(tf, ptw_ac_x_i);
    PN_TRACE(tf, ptw_ac_u_i);
    PN_TRACE(tf, ptw_ac_a_i);
    PN_TRACE(tf, ptw_ac_d_i);

    PN_TRACE(tf, ptw_superpage_o);
    PN_TRACE(tf, ptw_adr_o);
    PN_TRACE(tf, ptw_hit_o);
    PN_TRACE(tf, ptw_miss_o);

    PN_TRACE(tf, ptw_ac_r_o);
    PN_TRACE(tf, ptw_ac_w_o);
    PN_TRACE(tf, ptw_ac_x_o);
    PN_TRACE(tf, ptw_ac_u_o);
    PN_TRACE(tf, ptw_ac_a_o);
    PN_TRACE(tf, ptw_ac_d_o);
}

void RunCycle (int n = 1) {
    for (int k = 0; k < n; k++) {
        clk = 1;
        sc_start (CLK_PERIOD / 2, SC_NS);
        clk = 0;
        sc_start (CLK_PERIOD / 2, SC_NS);
    }
}

void fill_address_array(adr_t *addresses, unsigned int count) {
    std::random_device rd; // obtain a random number from hardware
    std::mt19937 gen(rd()); // seed the generator
    std::uniform_int_distribution<> distr(0, 1 << 20); // define the range
    std::uniform_int_distribution<> bool_distr(0, 3); // define the range

    for (unsigned int i = 0; i < count; i++) {
        addresses[i].a = !bool_distr(gen);
        addresses[i].d = !bool_distr(gen);
        addresses[i].r = !bool_distr(gen);
        addresses[i].w = !bool_distr(gen);
        addresses[i].x = !bool_distr(gen);
        addresses[i].u = !bool_distr(gen);
        addresses[i].superpage = !bool_distr(gen);;

        if (addresses[i].superpage) {
            addresses[i].va = distr(gen);
            addresses[i].pa = distr(gen);
        } else {
            addresses[i].va = distr(gen) & 0xFFC00;
            addresses[i].pa = distr(gen) & 0xFFC00;
        }
    }

}

void run_test () {
    adr_t addresses[TEST_COUNT];

    fill_address_array(addresses, TEST_COUNT);

    for (unsigned int i = 0; i < TEST_COUNT; i++) {
        printf("va=0x%x, pa=0x%x, ac_a=%x, ac_d=%x, ac_r=%x, ac_w=%x, ac_x=%x, ac_u=%x\n", 
            (unsigned int) addresses[i].va, (unsigned int) addresses[i].pa, addresses[i].a, addresses[i].d, addresses[i].r, addresses[i].w, addresses[i].x, addresses[i].u);
    }

    reset = 1;
    RunCycle(5);
    reset = 0;

    for (unsigned int z = 0; z < 2; z++) {
        for (unsigned int i = 0; i < TEST_COUNT; i++) {
            // read address -> not available
            ptw_va_i = addresses[i].va;
            ptw_req = 1;
            RunCycle(1);
            PN_ASSERTF (ptw_miss_o.read () && !ptw_hit_o.read (), ("Expected miss, but miss=%i and hit=%i", (bool) ptw_miss_o, (bool) ptw_hit_o));
            ptw_req = 0;
            RunCycle(1);
            PN_ASSERTF (!ptw_miss_o.read () && !ptw_hit_o.read (), ("Didn't expect hit or miss when no request, but miss=%i and hit=%i", (bool) ptw_miss_o, (bool) ptw_hit_o));
            
            // write address
            ptw_va_i = addresses[i].va;
            ptw_pa_i = addresses[i].pa;
            ptw_superpage_i = addresses[i].superpage;
            ptw_ac_a_i = addresses[i].a;
            ptw_ac_d_i = addresses[i].d;
            ptw_ac_r_i = addresses[i].r;
            ptw_ac_w_i = addresses[i].w;
            ptw_ac_x_i = addresses[i].x;
            ptw_ac_u_i = addresses[i].u;
            ptw_wr = 1;
            RunCycle(1);
            ptw_wr = 0;

            // read address -> should be available
            ptw_va_i = addresses[i].va;
            ptw_pa_i = 0;
            ptw_superpage_i = 0;
            ptw_ac_a_i = 0;
            ptw_ac_d_i = 0;
            ptw_ac_r_i = 0;
            ptw_ac_w_i = 0;
            ptw_ac_x_i = 0;
            ptw_ac_u_i = 0;
            ptw_req = 1;
            RunCycle(1);
            PN_ASSERTF (ptw_hit_o.read (), ("Expected hit.", addresses[i].pa, ptw_adr_o.read ()));
            PN_ASSERTF (ptw_adr_o.read () == addresses[i].pa, ("Expected address to be 0x%x, but is 0x%x.", addresses[i].pa, ptw_adr_o.read ()));
            PN_ASSERTF (ptw_ac_a_o.read () == addresses[i].a, ("Expected ac_a_o to be 0x%x, but is 0x%x.", addresses[i].a, ptw_ac_a_o.read ()));
            PN_ASSERTF (ptw_ac_d_o.read () == addresses[i].d, ("Expected ac_d_o to be 0x%x, but is 0x%x.", addresses[i].d, ptw_ac_d_o.read ()));
            PN_ASSERTF (ptw_ac_r_o.read () == addresses[i].r, ("Expected ac_r_o to be 0x%x, but is 0x%x.", addresses[i].r, ptw_ac_r_o.read ()));
            PN_ASSERTF (ptw_ac_w_o.read () == addresses[i].w, ("Expected ac_w_o to be 0x%x, but is 0x%x.", addresses[i].w, ptw_ac_w_o.read ()));
            PN_ASSERTF (ptw_ac_x_o.read () == addresses[i].x, ("Expected ac_x_o to be 0x%x, but is 0x%x.", addresses[i].x, ptw_ac_x_o.read ()));
            PN_ASSERTF (ptw_ac_u_o.read () == addresses[i].u, ("Expected ac_u_o to be 0x%x, but is 0x%x.", addresses[i].u, ptw_ac_u_o.read ()));
            ptw_req = 0;

            RunCycle(1);

            // Check if other addresses are still available: min(i, CFG_MMU_TLB_ENTRIES-1) other addresses should be inside of TLB
            const unsigned int maximum_buffer_entries = CFG_MMU_TLB_ENTRIES;
            const unsigned int expected_buffer_entries = std::min(i+1, maximum_buffer_entries);
            unsigned int actual_buffer_entries = 0;

            for (int j = 0; j < TEST_COUNT; j++) {
                ptw_va_i = addresses[j].va;
                ptw_req = 1;
                RunCycle(1);


                actual_buffer_entries += ptw_hit_o
                                         && ptw_adr_o.read () == addresses[j].pa
                                         && ptw_ac_a_o.read () == addresses[j].a
                                         && ptw_ac_d_o.read () == addresses[j].d
                                         && ptw_ac_r_o.read () == addresses[j].r
                                         && ptw_ac_w_o.read () == addresses[j].w
                                         && ptw_ac_x_o.read () == addresses[j].x
                                         && ptw_ac_u_o.read () == addresses[j].u;
                ptw_req = 0;
                RunCycle(2);
            }
            PN_ASSERTF (actual_buffer_entries == expected_buffer_entries, ("Expected %u buffer entries in loop %i, but got %u", expected_buffer_entries, i, actual_buffer_entries));

            RunCycle(3); // run for some clocks to simulate real usage
        }

        // run several cycles
        // check if still CFG_MMU_TLB_ENTRIES entries are in TLB

        // flush TLB
        tlb_flush = 1;
        RunCycle(1);
        tlb_flush = 0;

        // no entries should be in TLB anymore

    }
}

int sc_main (int argc, char *argv[]) {
    sc_trace_file *tf = NULL;
    int n, arg;
    bool cfg_help = false;

    // Parse command line...
    arg = 1;
    while (arg < argc && argv[arg][0] == '-') {
        switch (argv[arg][1]) {
        case 't':
            pn_cfg_vcd_level = MAX (0, MIN (9, argv[arg][2] - '0'));
            fprintf (stderr, "(cfg) vcdLevel = %i\n", pn_cfg_vcd_level);
            break;
        case 'h':
            cfg_help = true;
            break;
        default:
            printf ("PN_ERROR: Unknown option '%s'.\n", argv[arg]);
            cfg_help = true;
            arg = argc;
        }
        arg++;
    }
    if (cfg_help) {
        printf ("Usage: %s [<options>] <OR32 ELF file>\n"
              "\n"
              "Options:\n"
              "  -t<n>: set VCD trace level (0 = no trace file; default = 2)\n", argv[0]);
        return 3;
    }

    sc_set_time_resolution (1.0, SC_NS);

    MTlb tlb("TLB");
    tlb.clk (clk);
    tlb.reset (reset);
    tlb.flush (tlb_flush);
    tlb.ptw_req (ptw_req);
    tlb.ptw_wr (ptw_wr);
    tlb.ptw_va_i (ptw_va_i);
    tlb.ptw_pa_i (ptw_pa_i);
    tlb.ptw_superpage_i (ptw_superpage_i);
    tlb.ptw_ac_r_i (ptw_ac_r_i);
    tlb.ptw_ac_w_i (ptw_ac_w_i);
    tlb.ptw_ac_x_i (ptw_ac_x_i);
    tlb.ptw_ac_u_i (ptw_ac_u_i);
    tlb.ptw_ac_a_i (ptw_ac_a_i);
    tlb.ptw_ac_d_i (ptw_ac_d_i);

    tlb.ptw_superpage_o (ptw_superpage_o);
    tlb.ptw_adr_o (ptw_adr_o);
    tlb.ptw_hit_o (ptw_hit_o);
    tlb.ptw_miss_o (ptw_miss_o);

    tlb.ptw_ac_r_o (ptw_ac_r_o);
    tlb.ptw_ac_w_o (ptw_ac_w_o);
    tlb.ptw_ac_x_o (ptw_ac_x_o);
    tlb.ptw_ac_u_o (ptw_ac_u_o);
    tlb.ptw_ac_a_o (ptw_ac_a_o);
    tlb.ptw_ac_d_o (ptw_ac_d_o);

    // Init trace file...
    if (pn_cfg_vcd_level > 0) {
        tf = sc_create_vcd_trace_file ("tlb_tb");
        tf->delta_cycles (false);

        trace(tf);
        tlb.Trace(tf, pn_cfg_vcd_level);
    }

    PN_ASSERTM (CFG_MMU_TLB_ENABLE, "TLB is disabled in the ParaNut hardware configuration. Please enable it before running this testbench!");

    // Run simulation...
    sc_start (SC_ZERO_TIME);
    run_test ();

    // Finish...
    if (tf) sc_close_vcd_trace_file (tf);
    puts("Testbench successfully executed.");
    return 0;
}