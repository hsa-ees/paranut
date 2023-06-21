/*************************************************************************

  This file is part of the ParaNut project.

  Copyright (C) 2010-2019 Alexander Bahle <alexander.bahle@hs-augsburg.de>
                          Gundolf Kiefer <gundolf.kiefer@hs-augsburg.de>
                2022-2023 Lukas Bauer <lukas.bauer@hs-augsburg.de>
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
sc_signal<bool> dmi_rd{"dmi_rd"}, dmi_wr{"dmi_wr"};
sc_signal<sc_uint<DTM_ADDR_WIDTH> > dmi_adr{"dmi_adr"};
sc_signal<sc_uint<32> > dmi_dat_o{"dmi_dat_o"};
sc_signal<sc_uint<32> > dmi_dat_i{"dmi_dat_i"};
// DBG
sc_signal<sc_uint<CFG_NUT_CPU_CORES>> dbg_request{"dbg_request"};
sc_signal<bool> dbg_reset{"dbg_reset"};
// WB
sc_signal<bool> clk{"clk"}, reset{"reset"};
sc_signal<bool> wb_stb{"wb_stb"}, wb_cyc{"wb_cyc"}, wb_we{"wb_we"}, wb_ack{"wb_ack"}, wb_err{"wb_err"}, wb_rty{"wb_rty"};
sc_signal<sc_uint<CFG_MEMU_BUSIF_WIDTH/8> > wb_sel{"wb_sel"};
sc_signal<sc_uint<32>> wb_adr{"wb_adr"};
sc_signal<sc_uint<CFG_MEMU_BUSIF_WIDTH> > wb_dat_w{"wb_dat_w"}, wb_dat_r{"wb_dat_r"};

// **************** Helpers *********************


#define CLK_PERIOD 10.0

void RunCycle (int n = 1) {
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

    RunCycle (1);
    dmi_rd = 0;
    RunCycle (REG_STAGES);
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
    RunCycle (1);
    dmi_wr = 0;
    RunCycle (REG_STAGES);
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

    while (!wb_ack.read ()) RunCycle (1);

    wb_stb = 0;
    wb_cyc = 0;
    wb_we = 0;
}

void InitWBRead (uint32_t adr) {
    wb_stb = 1;
    wb_cyc = 1;
    wb_we = 0;
    wb_sel = 0xf;
    wb_adr = adr;
}

uint32_t CompleteWBRead (uint32_t adr) {
    InitWBRead(adr);

    while(!wb_ack.read()) RunCycle (1);

    uint32_t ret = wb_dat_r.read();

    wb_stb = 0;
    wb_cyc = 0;
    wb_we = 0;

    return ret;
}

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

    // Set cfg_debug_mode to suppress some simulation warnings
    pn_cfg_debug_mode = true;

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
    if (pn_cfg_vcd_level > 0) {
        tf = sc_create_vcd_trace_file ("dm_tb");
        tf->delta_cycles (false);

        PN_TRACE (tf, reset);
        PN_TRACE (tf, wb_stb);
        PN_TRACE (tf, wb_cyc);
        PN_TRACE (tf, wb_adr);
        PN_TRACE (tf, wb_dat_w);
        PN_TRACE (tf, wb_dat_r);
        PN_TRACE (tf, wb_we);
        PN_TRACE (tf, wb_ack);
        PN_TRACE (tf, wb_rty);
        PN_TRACE (tf, wb_err);
        PN_TRACE (tf, wb_sel);
        PN_TRACE (tf, dbg_request);
        PN_TRACE (tf, dbg_reset);
        PN_TRACE (tf, dmi_adr);
        PN_TRACE (tf, dmi_dat_i);
        PN_TRACE (tf, dmi_dat_o);
        PN_TRACE (tf, dmi_rd);
        PN_TRACE (tf, dmi_wr);

        dm.Trace (tf, pn_cfg_vcd_level);
    } else {
        fprintf (stderr, "Tracing is disabled.\n");
        tf = NULL;
    }

    // Run simulation...
    fprintf (stderr, "(sim) Starting SystemC simulation...\n\n");
    sc_start (SC_ZERO_TIME);


    PN_INFO ("Reset...");
    reset = 1;
    RunCycle (5);
    PN_INFO ("Running...");
    reset = 0;

    uint32_t ret;

    // DMI read:
    // --------------------
    PN_INFO ("DMI read test:");
    // Read dmcontrol
    ret = CompleteDMIRead (dmcontrol);
    PN_INFOF (("dmcontrol: \t0x%08x", ret));
    PN_ASSERTM (ret == 0x0, "Could not read correct dmcontrol");
    // Read dmstatus
    RunCycle (1);
    ret = CompleteDMIRead (dmstatus);
    PN_INFOF (("dmstatus: \t0x%08x", ret));
    PN_ASSERTM (ret == 0x00000c82, "Could not read correct dmstatus");
    // Read abstracts
    RunCycle (1);
    ret = CompleteDMIRead (abstracts);
    PN_INFOF (("abstracts: \t0x%08x", ret));
    PN_ASSERTM (ret == (sc_uint<3> (0), sc_uint<5> (DBG_NUM_PROGBUF), sc_uint<11> (0), sc_uint<5> (0),
                     sc_uint<4> (0), sc_uint<4> (DBG_NUM_DATA)),
             "Could not read correct abstracts");

    // DMI read multicycle:
    // --------------------
    PN_INFO ("DMI read multicycle test:");
    // Read dmstatus
    RunCycle (1);
    InitDMIRead (dmstatus);
    RunCycle (5);
    ret = CompleteDMIRead (dmstatus);
    PN_INFOF (("dmstatus: \t0x%08x", ret));
    PN_ASSERTM (ret == 0x00000c82, "Could not read correct dmstatus");

    // DMI write:
    // --------------------
    PN_INFO ("DMI write test:");
    // Write dmcontrol (haltreq, ndmreset and active is set)
    CompleteDMIWrite (dmcontrol, 0x80000003);
    ret = CompleteDMIRead (dmcontrol);
    PN_INFOF (("dmcontrol: \t0x%08x", ret));
    PN_ASSERTM (ret == 0x80000003, "Could not read correct dmcontrol");
    PN_ASSERTM (dbg_request.read () == 1, "dbg_request signal is not set");
    PN_ASSERTM (dbg_reset.read () == 1, "dbg_reset signal is not set");

    // DMI write  multicycle:
    // --------------------
    PN_INFO ("DMI write multicycle test:");
    // Write dmcontrol (haltreq, ndmreset and active is set)
    InitDMIWrite (dmcontrol, 0x00000001);
    RunCycle (2);
    // Change value -> should not be written
    dmi_dat_i = 0x80000002;
    RunCycle (2);
    ret = CompleteDMIRead (dmcontrol);
    PN_INFOF (("dmcontrol: \t0x%08x", ret));
    PN_ASSERTM (ret == 0x00000001, "Could not read correct dmcontrol");
    PN_ASSERTM (dbg_request.read () == 0, "dbg_request signal is not zero");
    PN_ASSERTM (dbg_reset.read () == 0, "dbg_reset signal is not zero");

    // WB write:
    // --------------------
    PN_INFO ("WB write test:");
    // Write halted
    CompleteWBWrite (DBG_HALTED_OFFSET, 0x0);
    // Read dmstatus
    RunCycle (1);
    ret = CompleteDMIRead (dmstatus);
    PN_INFOF (("dmstatus: \t0x%08x", ret));
    PN_ASSERTM (ret == (sc_uint<16> (0),
                     sc_uint<4> (0), // no CPU should be nonexistant/unavailable
                     sc_uint<2> (0),
                     sc_uint<2> (3), // allhalted, anyhalted
                     sc_uint<1> (1), // authenticated
                     sc_uint<3> (0), sc_uint<4> (2)),
             "Could not read correct dmstatus");

    // WB write during DMI write:
    // --------------------
    PN_INFO ("WB and DMI write test:");
    // Write dmcontrol (haltreq, ndmreset and active is set)
    InitDMIWrite (progbuf0, 0xCAFEBABE);
    CompleteWBWrite (DBG_RESUMING_OFFSET, 0x0);
//    RunCycle(DMI_CYCLES);
    ret = CompleteDMIRead (progbuf0);
    PN_INFOF (("progbuf0: \t0x%08x", ret));
    PN_ASSERTM (ret == 0xCAFEBABE, "Could not read correct progbuf0");
    ret = CompleteDMIRead (dmstatus);
    PN_INFOF (("dmstatus: \t0x%08x", ret));
    PN_ASSERTM (ret == (sc_uint<14> (0),
                     sc_uint<2> (3), // allresumeack, anyresumeack
                     sc_uint<4> (0), // no CPU should be nonexistant/unavailable
                     sc_uint<2> (3), // allrunning, anyrunning
                     sc_uint<2> (0),
                     sc_uint<1> (1), // authenticated
                     sc_uint<3> (0), sc_uint<4> (2)),
             "Could not read correct dmstatus");


    // abstract command type test
    // --------------------
    PN_INFO("abstract command type test:");
    // write abstract command to dm command register
    CompleteDMIWrite(command, (sc_uint<8> (4), // invalid command type
                               sc_uint<1> (0),
                               sc_uint<3> (2), // register size
                               sc_uint<1> (0), // no postincrement
                               sc_uint<1> (0), // no postexec
                               sc_uint<1> (1), // transfer register
                               sc_uint<1> (1), // write to register
                               sc_uint<16>(0x1000))); // register no.

    RunCycle(1);

    // read abstracts register
    ret = CompleteDMIRead(abstracts);
    PN_INFOF(("abstracts: \t0x%08x", ret));
    // check for not supported error
    PN_ASSERTM(ret == (sc_uint<3>  (0),
                       sc_uint<5>  (DBG_NUM_PROGBUF),
                       sc_uint<11> (0),
                       sc_uint<1>  (0),
                       sc_uint<1>  (0),
                       sc_uint<3>  (CMDERR_NOTSUP),
                       sc_uint<4>  (0),
                       sc_uint<4>  (DBG_NUM_DATA)),
                       "cmd error \"not supported\" not raised");

    // clearing error in abstracts reg
    ret = ret &= (0x0000700);
    CompleteDMIWrite(abstracts, ret);


    // abstract register size test:
    // --------------------
    PN_INFO("abstract register size test:");
    // write abstract command to dm command register
    CompleteDMIWrite(command, (sc_uint<8> (0), // register access cmd type
                               sc_uint<1> (0),
                               sc_uint<3> (3), // wrong register size (64bit)
                               sc_uint<1> (0), // no postincrement
                               sc_uint<1> (0), // no postexec
                               sc_uint<1> (1), // transfer register
                               sc_uint<1> (1), // write to register
                               sc_uint<16>(0x1000))); // register no.
    RunCycle(1);
    // read abstracts register
    ret = CompleteDMIRead(abstracts);
    PN_INFOF(("abstracts: \t0x%08x", ret));
    // check for not supported error
    PN_ASSERTM(ret == (sc_uint<3>  (0),
                       sc_uint<5>  (DBG_NUM_PROGBUF),
                       sc_uint<11> (0),
                       sc_uint<1>  (0),
                       sc_uint<1>  (0),
                       sc_uint<3>  (CMDERR_NOTSUP),
                       sc_uint<4>  (0),
                       sc_uint<4>  (DBG_NUM_DATA)),
                       "cmd error \"not supported\" not raised");

    // reset command error
    ret = ret &= (0x0000700);
    CompleteDMIWrite(abstracts, ret);


    // abstract not supported regierst test:
    // --------------------
    PN_INFO("abstract not supported register test:");
    // write abstract command to dm command register
    CompleteDMIWrite(command, (sc_uint<8> (0), // register access cmd type
                               sc_uint<1> (0),
                               sc_uint<3> (2), // register size
                               sc_uint<1> (0), // no postincrement
                               sc_uint<1> (0), // no postexec
                               sc_uint<1> (1), // transfer register
                               sc_uint<1> (1), // write to register
                               sc_uint<16>(1))); // register number outside supported range 0x1
    RunCycle(1);
    // read abstracts register
    ret = CompleteDMIRead(abstracts);
    PN_INFOF(("abstracts: \t0x%08x", ret));
    // check for not supported error
    PN_ASSERTM(ret == (sc_uint<3>  (0),
                       sc_uint<5>  (DBG_NUM_PROGBUF),
                       sc_uint<11> (0),
                       sc_uint<1>  (0),
                       sc_uint<1>  (0),
                       sc_uint<3>  (CMDERR_NOTSUP),
                       sc_uint<4>  (0),
                       sc_uint<4>  (DBG_NUM_DATA)),
                       "cmd error \"not supported\" not raised");

    // abstract command reset test:
    // --------------------
    PN_INFO("abstract command reset test:");
    // write abstract command to dm command register
    CompleteDMIWrite(command, (sc_uint<8> (0), // register access cmd type
                               sc_uint<1> (0),
                               sc_uint<3> (2), // register size
                               sc_uint<1> (0), // no postincrement
                               sc_uint<1> (0), // no postexec
                               sc_uint<1> (1), // transfer register
                               sc_uint<1> (1), // write to register
                               sc_uint<16>(0x1000))); // register number outside supported range 0x1000
    RunCycle(1);

    ret = CompleteDMIRead(abstracts);
    PN_INFOF(("abstracts: \t0x%08x", ret));
    // check for not supported error
    PN_ASSERTM(ret == (sc_uint<3>  (0),
                       sc_uint<5>  (DBG_NUM_PROGBUF),
                       sc_uint<11> (0),
                       sc_uint<1>  (0),
                       sc_uint<1>  (0),
                       sc_uint<3>  (CMDERR_NOTSUP),
                       sc_uint<4>  (0),
                       sc_uint<4>  (DBG_NUM_DATA)),
                       "cmd error value changed");

    // reset command error
    ret = ret &= (0x0000700);
    CompleteDMIWrite(abstracts, ret);

    // abstract hart not halted test:
    // --------------------
    PN_INFO("abstract halt not halted test:");
    // write abstract command to dm command register
    CompleteDMIWrite(command, (sc_uint<8> (0), // register access cmd type
                               sc_uint<1> (0),
                               sc_uint<3> (2), // register size
                               sc_uint<1> (0), // no postincrement
                               sc_uint<1> (0), // no postexec
                               sc_uint<1> (1), // transfer register
                               sc_uint<1> (1), // write to register
                               sc_uint<16>(0x1000))); // register number outside supported range 0x1000
    RunCycle(1);

    ret = CompleteDMIRead(abstracts);
    PN_INFOF(("abstracts: \t0x%08x", ret));
    // check for not supported error
    PN_ASSERTM(ret == (sc_uint<3>  (0),
                       sc_uint<5>  (DBG_NUM_PROGBUF),
                       sc_uint<11> (0),
                       sc_uint<1>  (0),
                       sc_uint<1>  (0),
                       sc_uint<3>  (CMDERR_HALTRESUME),
                       sc_uint<4>  (0),
                       sc_uint<4>  (DBG_NUM_DATA)),
                       "cmd error value changed");

    // abstract write register test:
    // --------------------
    PN_INFO("abstract write register test:");

    // Halt Hart
    CompleteWBWrite (DBG_HALTED_OFFSET, 0x0);

    // reset command error
    ret = ret &= (0x0000700);
    CompleteDMIWrite(abstracts, ret);

    // write abstract command to dm command register
    CompleteDMIWrite(command, (sc_uint<8> (0), // register access cmd type
                               sc_uint<1> (0),
                               sc_uint<3> (2), // register size
                               sc_uint<1> (0), // no postincrement
                               sc_uint<1> (0), // no postexec
                               sc_uint<1> (1), // transfer register
                               sc_uint<1> (1), // write to register
                               sc_uint<16>(0x1000))); // GPR 0x1000
    RunCycle(1);

    ret = CompleteDMIRead(abstracts);
    PN_INFOF(("abstracts: \t0x%08x", ret));
    // check for not supported error
    PN_ASSERTM(ret == (sc_uint<3>  (0),
                       sc_uint<5>  (DBG_NUM_PROGBUF),
                       sc_uint<11> (0),
                       sc_uint<1>  (1),
                       sc_uint<1>  (0),
                       sc_uint<3>  (CMDERR_NONE),
                       sc_uint<4>  (0),
                       sc_uint<4>  (DBG_NUM_DATA)),
                       "command error occurred");

    ret = CompleteWBRead(DBG_ABSTRACT_OFFSET);

    PN_INFOF(("abstract0: \t0x%08x", ret));

    RunCycle(2);

    PN_ASSERTM(ret == (sc_uint<12> (0),
                       sc_uint<5>  (0),
                       sc_uint<3>  (2),
                       sc_uint<5>  (0),
                       sc_uint<7>  (0x03)),
                       "lw command not valid");

    ret = CompleteWBRead(DBG_ABSTRACT_OFFSET + 4);
    PN_INFOF(("abstract1: \t0x%08x", ret));

    PN_ASSERTM(ret == (sc_uint<11> (1),
                       sc_uint<13> (0),
                       sc_uint<7>  (0x73)),
                       "no ebreak when no postexec is enabled");


    // abstract read register test:
    // --------------------
    PN_INFO("abstract read register test:");

    //reset going flag
    RunCycle(1);
    CompleteWBWrite(DBG_GOING_OFFSET, 0x0);

    //reset busy flag
    RunCycle(1);
    CompleteWBWrite(DBG_HALTED_OFFSET, 0x0);

    RunCycle(1);

    // Halt Hart
    CompleteWBWrite (DBG_HALTED_OFFSET, 0x0);

    // write abstract command to dm command register
    CompleteDMIWrite(command, (sc_uint<8> (0), // register access cmd type
                               sc_uint<1> (0),
                               sc_uint<3> (2), // register size
                               sc_uint<1> (0), // no postincrement
                               sc_uint<1> (0), // no postexec
                               sc_uint<1> (1), // transfer register
                               sc_uint<1> (0), // read from register
                               sc_uint<16>(0x1000))); // GPR 0x1000
    RunCycle(1);

    ret = CompleteDMIRead(abstracts);
    PN_INFOF(("abstracts: \t0x%08x", ret));
    // check for not supported error
    PN_ASSERTM(ret == (sc_uint<3>  (0),
                       sc_uint<5>  (DBG_NUM_PROGBUF),
                       sc_uint<11> (0),
                       sc_uint<1>  (1),
                       sc_uint<1>  (0),
                       sc_uint<3>  (CMDERR_NONE),
                       sc_uint<4>  (0),
                       sc_uint<4>  (DBG_NUM_DATA)),
                       "command error occurred");

    ret = CompleteWBRead(DBG_ABSTRACT_OFFSET);

    PN_INFOF(("abstract0: \t0x%08x", ret));

    RunCycle(1);

    PN_ASSERTM(ret == (sc_uint<7>  (0),
                       sc_uint<5>  (0),
                       sc_uint<5>  (0),
                       sc_uint<3>  (2),
                       sc_uint<5>  (0),
                       sc_uint<7>  (0x23)),
                       "sw command not valid");

    ret = CompleteWBRead(DBG_ABSTRACT_OFFSET + 4);
    PN_INFOF(("abstract1: \t0x%08x", ret));

    PN_ASSERTM(ret == (sc_uint<11> (1),
                       sc_uint<13> (0),
                       sc_uint<7>  (0x73)),
                       "no ebreak when no postexec is enabled");


    // abstract no transfer test:
    // --------------------
    PN_INFO("abstract no transfer test:");

    //reset going flag
    RunCycle(1);
    CompleteWBWrite(DBG_GOING_OFFSET, 0x0);

    //reset busy flag
    RunCycle(1);
    CompleteWBWrite(DBG_HALTED_OFFSET, 0x0);

    RunCycle(1);

    // Halt Hart
    CompleteWBWrite (DBG_HALTED_OFFSET, 0x0);

    // write abstract command to dm command register
    CompleteDMIWrite(command, (sc_uint<8> (0), // register access cmd type
                               sc_uint<1> (0),
                               sc_uint<3> (2), // register size
                               sc_uint<1> (0), // no postincrement
                               sc_uint<1> (0), // no postexec
                               sc_uint<1> (0), // no transfer
                               sc_uint<1> (0), // read from register
                               sc_uint<16>(0x1000))); // GPR 0x1000
    RunCycle(1);

    ret = CompleteDMIRead(abstracts);
    PN_INFOF(("abstracts: \t0x%08x", ret));
    // check for not supported error
    PN_ASSERTM(ret == (sc_uint<3>  (0),
                       sc_uint<5>  (DBG_NUM_PROGBUF),
                       sc_uint<11> (0),
                       sc_uint<1>  (1),
                       sc_uint<1>  (0),
                       sc_uint<3>  (CMDERR_NONE),
                       sc_uint<4>  (0),
                       sc_uint<4>  (DBG_NUM_DATA)),
                       "command error occurred");

    ret = CompleteWBRead(DBG_ABSTRACT_OFFSET);

    PN_INFOF(("abstract0: \t0x%08x", ret));

    RunCycle(1);

    PN_ASSERTM(ret == 0x13, "no nop asm command");

    ret = CompleteWBRead(DBG_ABSTRACT_OFFSET + 4);
    PN_INFOF(("abstract1: \t0x%08x", ret));

    PN_ASSERTM(ret == (sc_uint<11> (1),
                       sc_uint<13> (0),
                       sc_uint<7>  (0x73)),
                       "no ebreak when no postexec is enabled");


    // abstract postexec test:
    // --------------------
    PN_INFO("abstract postexec test:");

    //reset going flag
    RunCycle(1);
    CompleteWBWrite(DBG_GOING_OFFSET, 0x0);

    //reset busy flag
    RunCycle(1);
    CompleteWBWrite(DBG_HALTED_OFFSET, 0x0);

    RunCycle(1);

    // Halt Hart
    CompleteWBWrite (DBG_HALTED_OFFSET, 0x0);

    // write abstract command to dm command register
    CompleteDMIWrite(command, (sc_uint<8> (0), // register access cmd type
                               sc_uint<1> (0),
                               sc_uint<3> (2), // register size
                               sc_uint<1> (0), // no postincrement
                               sc_uint<1> (1), // postexec
                               sc_uint<1> (0), // no transfer
                               sc_uint<1> (0), // read from register
                               sc_uint<16>(0x1000))); // GPR 0x1000
    RunCycle(1);

    ret = CompleteDMIRead(abstracts);
    PN_INFOF(("abstracts: \t0x%08x", ret));
    // check for not supported error
    PN_ASSERTM(ret == (sc_uint<3>  (0),
                       sc_uint<5>  (DBG_NUM_PROGBUF),
                       sc_uint<11> (0),
                       sc_uint<1>  (1),
                       sc_uint<1>  (0),
                       sc_uint<3>  (CMDERR_NONE),
                       sc_uint<4>  (0),
                       sc_uint<4>  (DBG_NUM_DATA)),
                       "command error occurred");

    ret = CompleteWBRead(DBG_ABSTRACT_OFFSET);

    PN_INFOF(("abstract0: \t0x%08x", ret));

    RunCycle(1);

    PN_ASSERTM(ret == 0x13,
                       "no nop asm command");

    sc_uint<21> progbuf_jump = -DBG_PROGBUF_JUMP;

    ret = CompleteWBRead(DBG_ABSTRACT_OFFSET + 4);

    PN_ASSERTM(ret ==  (progbuf_jump[20], progbuf_jump (10, 1), progbuf_jump[11],
                        progbuf_jump (19, 12), sc_uint<5> (0), sc_uint<7> (0x6F))
                        , "no jump asm command");

    // test go flag:
    // --------------------
    PN_INFO("test go flag:");

    RunCycle(1);
    ret = CompleteWBRead(DBG_FLAG_OFFSET);

    PN_INFOF(("flags: \t0x%08x", ret));

    PN_ASSERTM(ret == 0x1, "\"go\" flag not set");


    // test memory access size:
    // --------------------

    PN_INFO("Test abstract memory access command size:");

    //reset going flag
    RunCycle(1);
    CompleteWBWrite(DBG_GOING_OFFSET, 0x0);

    //reset busy flag
    RunCycle(1);
    CompleteWBWrite(DBG_HALTED_OFFSET, 0x0);

    RunCycle(1);

    // Halt Hart
    CompleteWBWrite (DBG_HALTED_OFFSET, 0x0);

    // Sending Memory access command with wrong size

    CompleteDMIWrite(command, (sc_uint<8>(2), // command type memmory access
                               sc_uint<1>(0), // aamvirtual set to virtual addresses
                               sc_uint<3>(3), // ammsize set to 64 bit instead of 32 bit
                               sc_uint<1>(0), // aampostincrement not set
                               sc_uint<2>(0), // not used
                               sc_uint<1>(0), // write not set
                               sc_uint<2>(0), // target-specific (not used)
                               sc_uint<14>(0))); // not used

    RunCycle(1);

    // read abstracts register
    ret = CompleteDMIRead(abstracts);
    PN_INFOF(("abstracts: \t0x%08x", ret));
    // check for not supported error
    PN_ASSERTM(ret == (sc_uint<3>  (0),
                       sc_uint<5>  (DBG_NUM_PROGBUF),
                       sc_uint<11> (0),
                       sc_uint<1>  (0),
                       sc_uint<1>  (0),
                       sc_uint<3>  (CMDERR_NOTSUP),
                       sc_uint<4>  (0),
                       sc_uint<4>  (DBG_NUM_DATA)),
                       "cmd error \"not supported\" not raised");


    // reset command error
    ret = ret &= (0x0000700);
    CompleteDMIWrite(abstracts, ret);

    // abstract memory virtual address mode test:
    // --------------------

    PN_INFO("abstract memory virtual address test:");

    CompleteDMIWrite(command, (sc_uint<8>(2), // command type memmory access
                               sc_uint<1>(1), // aamvirtual set to virtual addresses
                               sc_uint<3>(2), // ammsize set to 32 bit
                               sc_uint<1>(0), // aampostincrement not set
                               sc_uint<2>(0), // not used
                               sc_uint<1>(0), // write not set
                               sc_uint<2>(0), // target-specific (not used)
                               sc_uint<14>(0))); // not used

    RunCycle(1);

    // read abstracts register
    ret = CompleteDMIRead(abstracts);
    PN_INFOF(("abstracts: \t0x%08x", ret));
    // check for not supported error
    PN_ASSERTM(ret == (sc_uint<3>  (0),
                       sc_uint<5>  (DBG_NUM_PROGBUF),
                       sc_uint<11> (0),
                       sc_uint<1>  (0),
                       sc_uint<1>  (0),
                       sc_uint<3>  (CMDERR_NOTSUP),
                       sc_uint<4>  (0),
                       sc_uint<4>  (DBG_NUM_DATA)),
                       "cmd error \"not supported\" not raised");


    // reset command error
    ret = ret &= (0x0000700);
    CompleteDMIWrite(abstracts, ret);

    //reset busy flag
    RunCycle(1);
    CompleteWBWrite(DBG_HALTED_OFFSET, 0x0);

    //halt exu
    RunCycle(1);
    CompleteWBWrite(DBG_HALTED_OFFSET, 0x0);




    // abstract memory read test:
    // --------------------

    for(size_t incr = 0; incr < 2; incr++){

        for(size_t size = 0; size < 3; size++){

            PN_INFOF(("abstract memory read test for size %d and %s postincrement:"
            , size, incr ? "":"no"));

            // Write Address to Memory
            CompleteDMIWrite(data1, 0x10000000);

            // writing the abstract memory command
            CompleteDMIWrite(command, (sc_uint<8>(2), // command type memmory access
                                    sc_uint<1>(0), // aamvirtual not set to virtual addresses
                                    sc_uint<3>(size), // ammsize set to 8/16/32 bit
                                    sc_uint<1>(incr), // aampostincrement set
                                    sc_uint<2>(0), // not used
                                    sc_uint<1>(0), // write not set
                                    sc_uint<2>(0), // target-specific (not used)
                                    sc_uint<14>(0))); // not used

            // Waiting until the asm instructions are written to the abstract registers
            RunCycle(5);

            // Check if the written command was valid
            ret = CompleteDMIRead(abstracts);
            PN_INFOF(("abstracts: \t0x%08x", ret));
            // check for not supported error
            PN_ASSERTM(ret == (sc_uint<3>  (0),
                            sc_uint<5>  (DBG_NUM_PROGBUF),
                            sc_uint<11> (0),
                            sc_uint<1>  (1), // Busy
                            sc_uint<1>  (0),
                            sc_uint<3>  (CMDERR_NONE),
                            sc_uint<4>  (0),
                            sc_uint<4>  (DBG_NUM_DATA)),
                            "cmd error raised");



            // read abstract register 0
            ret = CompleteWBRead(DBG_ABSTRACT_OFFSET);
            RunCycle(1);
            PN_INFOF(("abstract0: \t0x%08x", ret));
            // check if the lw instruction was written correctly
            PN_ASSERTM(ret == (sc_uint<12> (4), // offset for data1
                            sc_uint<5> (0), // src register address (x0)
                            sc_uint<3> (2), // constant for lw
                            sc_uint<5>(31), // dst register for loaded data (x31)
                            sc_uint<7> (0x03) // lw op code
                            ), "lw for address not set");


            // read abstract register 1
            ret = CompleteWBRead(DBG_ABSTRACT_OFFSET + 4);
            RunCycle(1);
            PN_INFOF(("abstract1: \t0x%08x", ret));
            // check if the l(b/h/w) command was written correctly
            PN_ASSERTM(ret == (sc_uint<12> (0), // offset 0
                            sc_uint<5> (31), // src register for address (x31)
                            sc_uint<3> (size), // determines if the instruction is l(b/h/w)
                            sc_uint<5>(31), // dst register for loaded data (x31)
                            sc_uint<7> (0x03) // lw op code
                            ), "lw for address not set");


            // read abstract register 2
            ret = CompleteWBRead(DBG_ABSTRACT_OFFSET + 8);
            RunCycle(1);
            PN_INFOF(("abstract2: \t0x%08x", ret));
            // check if the s(b/h/w) command was written correctly
            PN_ASSERTM(ret == (sc_uint<7> (0), // offset [12:5]
                            sc_uint<5> (31), // dst register for for (x31)
                            sc_uint<5> (0), // src register for address (x0)
                            sc_uint<3> (size), // determines if the instruction is s(b/h/w)
                            sc_uint<5>(0), // offset[4:0]
                            sc_uint<7> (0x23) // sw op code
                            ), "lw for address not set");


            // read abstract register 3
            ret = CompleteWBRead(DBG_ABSTRACT_OFFSET + 12);
            RunCycle(1);
            PN_INFOF(("abstract3: \t0x%08x", ret));
            // check if the mem increment flag was set correctly
            PN_ASSERTM(ret == (sc_uint<7> (0x18), // offset [12:5]
                            sc_uint<5> (0), // src register for data (x31)
                            sc_uint<5> (0), // src register for address (x0)
                            sc_uint<3> (2), // constant for sw
                            sc_uint<5>(0x10), // offset[4:0]
                            sc_uint<7> (0x23) // sw op code
                            ), "sw for address not set");


            // read abstract register 4
            ret = CompleteWBRead(DBG_ABSTRACT_OFFSET + 16);
            RunCycle(1);
            PN_INFOF(("abstract4: \t0x%08x", ret));
            // check if the ebreak was set correctly
            PN_ASSERTM(ret == (sc_uint<11> (1),
                            sc_uint<13> (0),
                            sc_uint<7>  (0x73)),
                            "no ebreak is set");



            //reset going flag
            RunCycle(1);
            CompleteWBWrite(DBG_GOING_OFFSET, 0x0);

            //reset busy flag
            RunCycle(1);
            CompleteWBWrite(DBG_HALTED_OFFSET, 0x0);

            // reset the mem incr flag and increment memory
            RunCycle(1);
            CompleteWBWrite(DGB_MEM_INCR_OFFSET, 0x0);

            // read data1 register
            RunCycle(1);
            ret = CompleteWBRead(0x4);

            PN_INFOF(("data1: \t0x%08x", ret));
            // check if memory address was incremented correctly

            if (incr == 1){
                PN_ASSERTM(ret == (0x10000000 + (1 << size)),
                "mem addr in data1 was not incremented correctly");
            }else{
                PN_ASSERTM(ret == 0x10000000,
                "mem addr in data1 was not incremented correctly");
            }
        }
    }

    // abstract memory write test:
    // --------------------

    for(size_t incr = 0; incr < 2; incr++){

        for(size_t size = 0; size < 3; size++){

            PN_INFOF(("abstract memory write test for size %d and %s postincrement:"
            , size, incr ? "": "no"));

            // write address to memory
            CompleteDMIWrite(data1, 0x10000000);

            // write abstract memory command
            CompleteDMIWrite(command, (sc_uint<8>(2), // command type memmory access
                                    sc_uint<1>(0), // aamvirtual not set to virtual addresses
                                    sc_uint<3>(size), // ammsize set to 8/16/32 bit
                                    sc_uint<1>(incr), // aampostincrement (not) set
                                    sc_uint<2>(0), // not used
                                    sc_uint<1>(1), // write not set
                                    sc_uint<2>(0), // target-specific (not used)
                                    sc_uint<14>(0))); // not used

            // wait until all asm instructions are written to the abstract registers
            RunCycle(5);

            // check if command is valid
            ret = CompleteDMIRead(abstracts);
            PN_INFOF(("abstracts: \t0x%08x", ret));
            // check for not supported error
            PN_ASSERTM(ret == (sc_uint<3>  (0),
                            sc_uint<5>  (DBG_NUM_PROGBUF),
                            sc_uint<11> (0),
                            sc_uint<1>  (1), // Busy
                            sc_uint<1>  (0),
                            sc_uint<3>  (CMDERR_NONE),
                            sc_uint<4>  (0),
                            sc_uint<4>  (DBG_NUM_DATA)),
                            "cmd error raised");


            // read abstract register 0
            ret = CompleteWBRead(DBG_ABSTRACT_OFFSET);
            RunCycle(1);
            PN_INFOF(("abstract0: \t0x%08x", ret));
            // check if the lw asm instruction was written correctly
            PN_ASSERTM(ret == (sc_uint<12> (4), // offset for data1
                            sc_uint<5> (0), // src register address (x0)
                            sc_uint<3> (2), // constant for lw
                            sc_uint<5>(31), // dst register for loaded data (x31)
                            sc_uint<7> (0x03) // lw op code
                            ), "lw for address not set");


            // read abstract register 1
            ret = CompleteWBRead(DBG_ABSTRACT_OFFSET + 4);
            RunCycle(1);
            PN_INFOF(("abstract1: \t0x%08x", ret));
            // check if the l(b/h/w) asm instruction was written correctly
            PN_ASSERTM(ret == (sc_uint<12> (0), // offset 0
                            sc_uint<5> (0), // src register for address (x0)
                            sc_uint<3> (size), // determines if the instruction is l(b/h/w)
                            sc_uint<5>(30), // dst register for loaded data (x30)
                            sc_uint<7> (0x03) // lw op code
                            ), "lw for address not set");


            // read abstract register 2
            ret = CompleteWBRead(DBG_ABSTRACT_OFFSET + 8);
            RunCycle(1);
            PN_INFOF(("abstract2: \t0x%08x", ret));
            // check if the s(b/h/w) asm instruction was written correctly
            PN_ASSERTM(ret == (sc_uint<7> (0), // offset [12:5]
                            sc_uint<5> (30), // src register for data (x30)
                            sc_uint<5> (31), // src register for address (x31)
                            sc_uint<3> (size), // // determines if the instruction is s(b/h/w)
                            sc_uint<5>(0), // offset[4:0]
                            sc_uint<7> (0x23) // sw op code
                            ), "lw for address not set");


            // read abstract register 3
            ret = CompleteWBRead(DBG_ABSTRACT_OFFSET + 12);
            RunCycle(1);
            PN_INFOF(("abstract3: \t0x%08x", ret));
            // check if the mem increment flag was set correctly
            PN_ASSERTM(ret == (sc_uint<7> (0x18), // offset [12:5]
                            sc_uint<5> (0), // dst register for for (x31)
                            sc_uint<5> (0), // src register for address (x0)
                            sc_uint<3> (2), // constant for sw
                            sc_uint<5> (0x10), // offset[4:0]
                            sc_uint<7> (0x23) // sw op code
                            ), "sw for address not set");



            // read abstract register 4
            ret = CompleteWBRead(DBG_ABSTRACT_OFFSET + 16);
            RunCycle(1);
            PN_INFOF(("abstract4: \t0x%08x", ret));
            // check if the ebreak instruction was written correctly
            PN_ASSERTM(ret == (sc_uint<11> (1),
                            sc_uint<13> (0),
                            sc_uint<7>  (0x73)),
                            "no ebreak is set");


            //reset going flag
            RunCycle(1);
            CompleteWBWrite(DBG_GOING_OFFSET, 0x0);

            //reset busy flag
            RunCycle(1);
            CompleteWBWrite(DBG_HALTED_OFFSET, 0x0);

            // reset mem incr flag and increment mem addr in data1
            RunCycle(1);
            CompleteWBWrite(DGB_MEM_INCR_OFFSET, 0x0);

            // read data1 register
            RunCycle(1);
            ret = CompleteWBRead(0x4);
            PN_INFOF(("data1: \t0x%08x", ret));
            // check if the memory address was incremented correctly
            if (incr == 1){
                PN_ASSERTM(ret == (0x10000000 + (1U << size)),
                "the memory address in data1 was not incremented correctly");
            }else{
                PN_ASSERTM(ret == (0x10000000),
                "the memory address in data1 was not incremented correctly");
            }

        }

    }

    RunCycle(10);

    PN_INFO ("Simulation finished.");
    if (tf) sc_close_vcd_trace_file (tf);


    return 0;
}
