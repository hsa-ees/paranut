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
#include "jtag_dtm.h"

#include <stdio.h>

#include <signal.h>
#include <systemc.h>


// **************** Signals *********************

sc_signal<bool> reset;
// DMI
sc_signal<bool> dmi_rd, dmi_wr;
sc_signal<sc_uint<DTM_ADDR_WIDTH> > dmi_adr;
sc_signal<sc_uint<32> > dmi_dat_o;
sc_signal<sc_uint<32> > dmi_dat_i;
// JTAG
sc_signal<bool> tck, tms, tdi, tdo;


// **************** Helpers *********************

#define CLK_PERIOD 10.0

void JtagClock (bool tms_i = 0, bool tdi_i = 0, int n = 1) {
    // Set tms and tdi
    tms = tms_i;
    tdi = tdi_i;

    // Run clock n cycles
    for (int k = 0; k < n; k++) {
        tck = 0;
        sc_start (CLK_PERIOD / 2, SC_NS);
        tck = 1;
        sc_start (CLK_PERIOD / 2, SC_NS);
    }
}

void JtagReset () {
    // five cycles with tms enabled should be a reset
    JtagClock (1, 0, 5);
}

uint64_t JtagRd (int n) {
    sc_uint<64> data = 0;
    int i;
    for (i = 0; i < n - 1; i++) {
        // Read tdo and run a cycle
        data[i] = tdo.read ();
        JtagClock ();
    }
    // Read last output and set TMS to get out of SHIFT state
    data[i] = tdo.read ();
    JtagClock (1, 0, 1);

    return data.value ();
}

uint64_t JtagRdWr (int n, uint64_t val) {
    sc_uint<64> data = 0;
    sc_uint<64> wr_data = val;
    int i;

    for (i = 0; i < n - 1; i++) {
        // Read tdo and run a cycle with set tdi
        data[i] = tdo.read ();
        JtagClock (0, wr_data[i], 1);
    }
    // Read last output and set TMS to get out of SHIFT state
    data[i] = tdo.read ();
    JtagClock (1, wr_data[i], 1);

    return data.value ();
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
        puts ("Usage: jtag_dtb_tb [<options>]\n"
              "\n"
              "Options:\n"
              "  -t<n>: set VCD trace level (0 = no trace file; default = 2)\n");
        return 3;
    }

    // SystemC elaboration...
    fprintf (stderr, "(sim) Starting SystemC elaboration...\n");
    sc_set_time_resolution (1.0, SC_NS);

    // JTAG DTM
    MDtm jtag_dtm ("jtag_dtm");
    jtag_dtm.reset (reset);
    jtag_dtm.tck (tck);
    jtag_dtm.tms (tms);
    jtag_dtm.tdi (tdi);
    jtag_dtm.tdo (tdo);
    jtag_dtm.dmi_adr (dmi_adr);
    jtag_dtm.dmi_dat_i (dmi_dat_i);
    jtag_dtm.dmi_dat_o (dmi_dat_o);
    jtag_dtm.dmi_wr (dmi_wr);
    jtag_dtm.dmi_rd (dmi_rd);

    // Trace file...
    sc_trace_file *tf;
    if (pn_cfg_vcd_level > 0) {
        tf = sc_create_vcd_trace_file ("jtag_dtm_tb");
        tf->delta_cycles (false);

        PN_TRACE (tf, reset);
        PN_TRACE (tf, tck);
        PN_TRACE (tf, tms);
        PN_TRACE (tf, tdi);
        PN_TRACE (tf, tdo);
        PN_TRACE (tf, dmi_adr);
        PN_TRACE (tf, dmi_dat_i);
        PN_TRACE (tf, dmi_dat_o);
        PN_TRACE (tf, dmi_rd);
        PN_TRACE (tf, dmi_wr);

        jtag_dtm.Trace (tf, pn_cfg_vcd_level);
    } else {
        fprintf (stderr, "Tracing is disabled.\n");
        tf = NULL;
    }

    // Run simulation...
    fprintf (stderr, "(sim) Starting SystemC simulation...\n\n");
    sc_start (SC_ZERO_TIME);

    PN_INFO ("Reset...");
    reset = 1;
    JtagClock (0, 0, 3);
    PN_INFO ("Running...");
    reset = 0;

    uint64_t ret;

    // Test read IDCODE:
    // --------------------
    JtagReset ();
    // Go to SHIFT_DR state
    JtagClock (0);
    JtagClock (1);
    JtagClock (0);
    JtagClock (0);
    // Read IDCODE
    ret = JtagRdWr (32, 0xffffffff);
    PN_INFOF (("IDCODE of JTAG DTM: 0x%x", ret));
    PN_ASSERTM (ret == 0xdeadbeef, "Could not read correct IDCODE");

    // Test read DTMCS
    // --------------------
    JtagReset ();
    // Go to SHIFT_IR state
    JtagClock (0);
    JtagClock (1);
    JtagClock (1);
    JtagClock (0);
    JtagClock (0);
    // Set ir to 0x10 (DTMCS)
    ret = JtagRdWr (DTM_IR_WIDTH, DTMCS);
    PN_INFOF (("IR was: 0x%x", ret));
    PN_ASSERTM (ret == IDCODE, "IR was not set to IDCODE");
    // Go to SHIFT_DR
    JtagClock (1);
    JtagClock (1);
    JtagClock (0);
    JtagClock (0);
    // Read DTMCS
    ret = JtagRd (32);
    PN_INFOF (("DTMCS was: 0x%x", ret));
    PN_ASSERTM (ret == (DTM_ADDR_WIDTH << 4) | 1, "DTMSC does not contain the reset value");

    PN_INFOF (("\nTest OpenOCD sequence:\n----------------------------------"));
    // Test OpenOCD sequence:
    // --------------------
    JtagReset ();
    // Go to SHIFT_DR state
    JtagClock (0);
    JtagClock (1);
    JtagClock (0);
    JtagClock (0);
    // Read IDCODE
    ret = JtagRdWr (64, 0xffffffffffffffff);
    PN_INFOF (("IDCODE of JTAG DTM: 0x%x", ret));
    PN_ASSERTM (ret == 0xffffffffdeadbeef, "Could not read correct IDCODE");
    // Now in EXIT1_DR, go to PAUSE_DR
    JtagClock (0);
    JtagClock (1, 0, 9); // reset
    JtagClock (0); // RTI
    JtagClock (1); // SELECT_DR_SCAN
    JtagClock (1); // SELECT_IR_SCAN
    JtagClock (0); // CAPTURE_IR
    JtagClock (0); // SHIFT_IR
    ret = JtagRdWr (DTM_IR_WIDTH, 0xff);
    PN_INFOF (("IR was: 0x%x", ret));
    PN_ASSERTM (ret == IDCODE, "IR was not set to IDCODE");
    JtagClock (1); // UPDATE_IR
    JtagClock (0); // RTI
    JtagClock (0); // Stay one cycle
    // Go to SHIFT_IR state
    JtagClock (1);
    JtagClock (1);
    JtagClock (0);
    JtagClock (0);
    // Set ir to 0x10 (DTMCS)
    ret = JtagRdWr (DTM_IR_WIDTH, DTMCS);
    PN_INFOF (("IR was: 0x%x", ret));
    PN_ASSERTM (ret == 0x1f, "IR was not set to 0x1f");
    // Go to SHIFT_DR
    JtagClock (1); // UPDATE_IR
    JtagClock (0); // RTI
    JtagClock (0); // Stay one cycle
    JtagClock (1); // SELECT_DR_SCAN
    JtagClock (0); // CAPTURE_DR
    JtagClock (0); // SHIFT_DR
    // Read DTMCS
    ret = JtagRd (32);
    PN_INFOF (("DTMCS was: 0x%x", ret));
    PN_ASSERTM (ret == (4 << 12) | (DTM_ADDR_WIDTH << 4) | 1,
             "DTMSC does not contain the reset value");
    JtagClock (1); // UPDATE_DR
    JtagClock (1); // SELECT_DR_SCAN
    JtagClock (1); // SELECT_IR_SCAN
    JtagClock (0); // CAPTURE_IR
    JtagClock (0); // SHIFT_IR
    ret = JtagRdWr (DTM_IR_WIDTH, DMI);
    PN_INFOF (("IR was: 0x%x", ret));
    PN_ASSERTM (ret == DTMCS, "IR was not set to DTMCS");
    JtagClock (1); // UPDATE_IR
    JtagClock (0); // RTI
    JtagClock (1); // SELECT_DR_SCAN
    JtagClock (0); // CAPTURE_DR
    JtagClock (0); // SHIFT_DR
    ret = JtagRdWr (40, 0x4000000002);
    PN_INFOF (("DMI was: 0x%lx", ret));
    PN_ASSERTM (ret == 0x0, "DMI was not set 0");
    JtagClock (1); // UPDATE_DR
    JtagClock (0); // RTI
    JtagClock (1); // SELECT_DR_SCAN
    JtagClock (0); // CAPTURE_DR
    JtagClock (0); // SHIFT_DR
    ret = JtagRdWr (40, 0x4000000000);
    PN_INFOF (("DMI was: 0x%lx", ret));
    PN_ASSERTM (ret == 0x4000000000, "DMI op was not successfull");

    PN_INFO("\nTest DM Read\n---------------------------------------");

    // command to request dmstatus
    uint64_t cmd = 0x4400000001;

    JtagReset();

    // Go into the shift IR state
    JtagClock(0);
    JtagClock(1);
    JtagClock(1);
    JtagClock(0);
    JtagClock(0);

    // set IR to DMI
    ret = JtagRdWr(DTM_IR_WIDTH, DMI);



    // go into the dr shift mode
    JtagClock(1);
    JtagClock(1);
    JtagClock(0);
    JtagClock(0);

    //Write ReadInstruction
    ret = JtagRdWr(40, cmd);

    PN_INFOF(("DMI_ADR was: \t0x%02x", (uint16_t)dmi_adr.read()));
    PN_ASSERTM(dmi_adr.read() == sc_uint<6>(17), "Not address of dmstatus");
    PN_INFOF(("DMI_DAT_O was: \t0x%08x", (uint32_t)dmi_dat_o.read()));
    PN_ASSERTM(dmi_dat_o.read() == 0, "data present");
    PN_INFOF(("DMI_RD was: \t%d", (uint8_t)dmi_rd.read()));
    PN_ASSERTM(dmi_rd.read() == sc_uint<1> (0), "read bit not set");

    PN_INFO ("Simulation finished.");
    if (tf) sc_close_vcd_trace_file (tf);

    return 0;
}
