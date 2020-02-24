/*************************************************************************

  This file is part of the ParaNut project.

  Copyright (C) 2010-2019 Alexander Bahle <alexander.bahle@hs-augsburg.de>
                          Gundolf Kiefer <gundolf.kiefer@hs-augsburg.de>
      Hochschule Augsburg, University of Applied Sciences

  Description:
    This is a testbench for the ParaNut.

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


#include "dm.h"

#include <stdio.h>

#include <signal.h>
#include <systemc.h>


// **************** Signals *********************

// DMI
sc_signal<bool> dmi_rd, dmi_wr;
sc_signal<sc_uint<DTM_ADDR_WIDTH> > dmi_adr;
sc_signal<sc_uint<32> > dmi_dat_o;
sc_signal<sc_uint<32> > dmi_dat_i;
// DBG
sc_signal<sc_uint<CFG_NUT_CPU_CORES>> dbg_request;
sc_signal<bool> dbg_reset;
// WB
sc_signal<bool> clk, reset;
sc_signal<bool> wb_stb, wb_cyc, wb_we, wb_ack, wb_err, wb_rty;
sc_signal<sc_uint<CFG_MEMU_BUSIF_WIDTH/8> > wb_sel;
sc_signal<TWord> wb_adr;
sc_signal<sc_uint<CFG_MEMU_BUSIF_WIDTH> > wb_dat_w, wb_dat_r;

// **************** Helpers *********************

#define CLK_PERIOD 10.0

void RunCycles (int n = 1) {
    for (int k = 0; k < n; k++) {
        clk = 0;
        sc_start (CLK_PERIOD / 2, SC_NS);
        clk = 1;
        sc_start (CLK_PERIOD / 2, SC_NS);
    }
}

void InitDMIRead (uint32_t adr) {
    dmi_adr = adr;
    dmi_dat_i = 0;
    dmi_wr = 0;
    dmi_rd = 1;
}

uint32_t GetDMIData () { return dmi_dat_o.read (); }

uint32_t CompleteDMIRead (uint32_t adr) {
    InitDMIRead (adr);

    RunCycles (1);
    dmi_rd = 0;
    RunCycles (REG_STAGES);
    return GetDMIData ();
}

void InitDMIWrite (uint32_t adr, uint32_t val) {
    dmi_adr = adr;
    dmi_dat_i = val;
    dmi_wr = 1;
    dmi_rd = 0;
}

void CompleteDMIWrite (uint32_t adr, uint32_t val) {
    InitDMIWrite (adr, val);
    RunCycles (1);
    dmi_wr = 0;
    RunCycles (REG_STAGES);
}

void InitWBWrite (uint32_t adr, uint32_t val) {
    wb_stb = 1;
    wb_cyc = 1;
    wb_we = 1;
    wb_sel = 0xf;
    wb_dat_w = val;
    wb_adr = adr;
}

void CompleteWBWrite (uint32_t adr, uint32_t val) {
    InitWBWrite (adr, val);

    while (!wb_ack.read ()) RunCycles (1);

    wb_stb = 0;
    wb_cyc = 0;
    wb_we = 0;
}


// **************** Main ************************

int sc_main (int argc, char *argv[]) {
    int arg, cfg_help = 0;

    // Parse command line...
    arg = 1;
    while (arg < argc && argv[arg][0] == '-') {
        switch (argv[arg][1]) {
        case 't':
            cfg_vcd_level = MAX (0, MIN (9, argv[arg][2] - '0'));
            fprintf (stderr, "(cfg) vcdLevel = %i\n", cfg_vcd_level);
            break;
        case 'h':
            cfg_help = 1;
            break;
        default:
            printf ("ERROR: Unknown option '%s'.\n", argv[arg]);
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

    // SystemC elaboration...
    fprintf (stderr, "(sim) Starting SystemC elaboration...\n");
    sc_set_time_resolution (1.0, SC_NS);

    // JTAG DTM
    MDebugModule dm ("dm");
    dm.clk_i (clk);
    dm.rst_i (reset);
    dm.stb_i (wb_stb);
    dm.cyc_i (wb_cyc);
    dm.we_i (wb_we);
    dm.sel_i (wb_sel);
    dm.ack_o (wb_ack);
    dm.err_o (wb_err);
    dm.rty_o (wb_rty);
    dm.adr_i (wb_adr);
    dm.dat_i (wb_dat_w);
    dm.dat_o (wb_dat_r);
    dm.dbg_request (dbg_request);
    dm.dbg_reset (dbg_reset);
    dm.dmi_adr_i (dmi_adr);
    dm.dmi_dat_i (dmi_dat_i);
    dm.dmi_dat_o (dmi_dat_o);
    dm.dmi_wr (dmi_wr);
    dm.dmi_rd (dmi_rd);

    // Trace file...
    sc_trace_file *tf;
    if (cfg_vcd_level > 0) {
        tf = sc_create_vcd_trace_file ("dm_tb");
        tf->delta_cycles (false);

        TRACE (tf, reset);
        TRACE (tf, wb_stb);
        TRACE (tf, wb_cyc);
        TRACE (tf, wb_adr);
        TRACE (tf, wb_dat_w);
        TRACE (tf, wb_dat_r);
        TRACE (tf, wb_we);
        TRACE (tf, wb_ack);
        TRACE (tf, wb_rty);
        TRACE (tf, wb_err);
        TRACE (tf, wb_sel);
        TRACE (tf, dbg_request);
        TRACE (tf, dbg_reset);
        TRACE (tf, dmi_adr);
        TRACE (tf, dmi_dat_i);
        TRACE (tf, dmi_dat_o);
        TRACE (tf, dmi_rd);
        TRACE (tf, dmi_wr);

        dm.Trace (tf, cfg_vcd_level);
    } else {
        fprintf (stderr, "Tracing is disabled.\n");
        tf = NULL;
    }

    // Run simulation...
    fprintf (stderr, "(sim) Starting SystemC simulation...\n\n");
    sc_start (SC_ZERO_TIME);


    INFO ("Reset...");
    reset = 1;
    RunCycles (5);
    INFO ("Running...");
    reset = 0;

    uint32_t ret;

    // DMI read:
    // --------------------
    INFO ("DMI read test:");
    // Read dmcontrol
    ret = CompleteDMIRead (dmcontrol);
    INFOF (("dmcontrol: \t0x%08x", ret));
    ASSERTM (ret == 0x0, "Could not read correct dmcontrol");
    // Read dmstatus
    RunCycles (1);
    ret = CompleteDMIRead (dmstatus);
    INFOF (("dmstatus: \t0x%08x", ret));
    ASSERTM (ret == 0x00000c82, "Could not read correct dmstatus");
    // Read abstracts
    RunCycles (1);
    ret = CompleteDMIRead (abstracts);
    INFOF (("abstracts: \t0x%08x", ret));
    ASSERTM (ret == (sc_uint<3> (0), sc_uint<5> (DBG_NUM_PROGBUF), sc_uint<11> (0), sc_uint<5> (0),
                     sc_uint<4> (0), sc_uint<4> (DBG_NUM_DATA)),
             "Could not read correct abstracts");

    // DMI read multicycle:
    // --------------------
    INFO ("DMI read multicycle test:");
    // Read dmstatus
    RunCycles (1);
    InitDMIRead (dmstatus);
    RunCycles (5);
    ret = CompleteDMIRead (dmstatus);
    INFOF (("dmstatus: \t0x%08x", ret));
    ASSERTM (ret == 0x00000c82, "Could not read correct dmstatus");

    // DMI write:
    // --------------------
    INFO ("DMI write test:");
    // Write dmcontrol (haltreq, ndmreset and active is set)
    CompleteDMIWrite (dmcontrol, 0x80000003);
    ret = CompleteDMIRead (dmcontrol);
    INFOF (("dmcontrol: \t0x%08x", ret));
    ASSERTM (ret == 0x80000003, "Could not read correct dmcontrol");
    ASSERTM (dbg_request.read () == 1, "dbg_request signal is not set");
    ASSERTM (dbg_reset.read () == 1, "dbg_reset signal is not set");

    // DMI write  multicycle:
    // --------------------
    INFO ("DMI write multicycle test:");
    // Write dmcontrol (haltreq, ndmreset and active is set)
    InitDMIWrite (dmcontrol, 0x00000001);
    RunCycles (2);
    // Change value -> should not be written
    dmi_dat_i = 0x80000002;
    RunCycles (2);
    ret = CompleteDMIRead (dmcontrol);
    INFOF (("dmcontrol: \t0x%08x", ret));
    ASSERTM (ret == 0x00000001, "Could not read correct dmcontrol");
    ASSERTM (dbg_request.read () == 0, "dbg_request signal is not zero");
    ASSERTM (dbg_reset.read () == 0, "dbg_reset signal is not zero");

    // WB write:
    // --------------------
    INFO ("WB write test:");
    // Write halted
    CompleteWBWrite (DBG_HALTED_OFFSET, 0x0);
    // Read dmstatus
    RunCycles (1);
    ret = CompleteDMIRead (dmstatus);
    INFOF (("dmstatus: \t0x%08x", ret));
    ASSERTM (ret == (sc_uint<16> (0),
                     sc_uint<4> (0), // no CPU should be nonexistant/unavailable
                     sc_uint<2> (0),
                     sc_uint<2> (3), // allhalted, anyhalted
                     sc_uint<1> (1), // authenticated
                     sc_uint<3> (0), sc_uint<4> (2)),
             "Could not read correct dmstatus");

    // WB write during DMI write:
    // --------------------
    INFO ("WB and DMI write test:");
    // Write dmcontrol (haltreq, ndmreset and active is set)
    InitDMIWrite (progbuf0, 0xCAFEBABE);
    CompleteWBWrite (DBG_RESUMING_OFFSET, 0x0);
//    RunCycles(DMI_CYCLES);
    ret = CompleteDMIRead (progbuf0);
    INFOF (("progbuf0: \t0x%08x", ret));
    ASSERTM (ret == 0xCAFEBABE, "Could not read correct progbuf0");
    ret = CompleteDMIRead (dmstatus);
    INFOF (("dmstatus: \t0x%08x", ret));
    ASSERTM (ret == (sc_uint<14> (0),
                     sc_uint<2> (3), // allresumeack, anyresumeack
                     sc_uint<4> (0), // no CPU should be nonexistant/unavailable
                     sc_uint<2> (3), // allrunning, anyrunning
                     sc_uint<2> (0),
                     sc_uint<1> (1), // authenticated
                     sc_uint<3> (0), sc_uint<4> (2)),
             "Could not read correct dmstatus");

    RunCycles (10);

    INFO ("Simulation finished.");
    if (tf) sc_close_vcd_trace_file (tf);

    return 0;
}
