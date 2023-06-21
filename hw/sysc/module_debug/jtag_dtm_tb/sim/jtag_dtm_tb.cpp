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
sc_signal<sc_uint<DTM_ADDR_WIDTH>> dmi_adr;
sc_signal<sc_uint<32>> dmi_dat_o;
sc_signal<sc_uint<32>> dmi_dat_i;
// JTAG
sc_signal<bool> tck, tms, tdi, tdo;

// **************** Helpers *********************

#define CLK_PERIOD 10.0
bool ClockSwitch = false;

void JtagSwitchClock()
{
    ClockSwitch = !ClockSwitch;
}

void ToggleClock()
{
    tck.write(!tck.read());
}



void JtagClock(jtag_dtm_t *dtm, bool tms_i = 0, bool tdi_i = 0, int n = 1)
{
    // Set tms and tdi
    tms = tms_i;
    tdi = tdi_i;

    if (!ClockSwitch){
        // Run clock n cycles
        for (int k = 0; k < n; k++)
        {
            dtm->set_pins(0, tms_i, tdi_i);
            ToggleClock();
            dtm->set_pins(1, tms_i, tdi_i);
            ToggleClock();
            sc_start(CLK_PERIOD / 2, SC_NS);
        }
    }else{
        for (int k = 0; k < n; k++)
        {
            dtm->set_pins(1, tms_i, tdi_i);
            ToggleClock();
            sc_start(CLK_PERIOD / 2, SC_NS);
            dtm->set_pins(0, tms_i, tdi_i);
            ToggleClock();
            sc_start(CLK_PERIOD / 2, SC_NS);
        }
    }
    // fprintf(stderr, "\n");
}

void JtagClockOne(jtag_dtm_t *dtm, bool tck = 0,bool tms_i = 0, bool tdi_i = 0, int n = 1)
{
    // Set tms and tdi
    tms = tms_i;
    tdi = tdi_i;

    // Run clock n cycles
    for (int k = 0; k < n; k++)
    {
        dtm->set_pins(tck, tms_i, tdi_i);
        ToggleClock();
        sc_start(CLK_PERIOD / 2, SC_NS);
    }
}


void JtagReset(jtag_dtm_t *dtm)
{
    // five cycles with tms enabled should be a reset
    ClockSwitch = false;
    dtm->reset();
    JtagClock(dtm, 1, 0, 5);
    JtagClockOne(dtm, 0, 1, 0, 1);
    JtagClock(dtm, 1, 0, 3);

}

uint64_t JtagRd(jtag_dtm_t *dtm, int n)
{
    sc_uint<64> data = 0;
    int i;
    for (i = 0; i < n - 1; i++)
    {
        // Read tdo and run a cycle
        if(i == 0){
            JtagClockOne(dtm, 0, 0, 0, 1);
            JtagClockOne(dtm, 0, 0, 1, 1);

            JtagSwitchClock();
        }else{
            JtagClock(dtm);
        }
        data[i] = dtm->tdo();

    }
    // Read last output and set TMS to get out of SHIFT state
    JtagClockOne(dtm,1, 0, 0, 1);
    JtagClockOne(dtm,0, 1, 0, 1);
    JtagSwitchClock();
    data[i] = dtm->tdo();
    JtagClockOne(dtm,1, 1, 0, 1);

    return data.value();
}

uint64_t JtagRdWr(jtag_dtm_t *dtm, int n, uint64_t val)
{
    sc_uint<64> data = 0;
    sc_uint<64> wr_data = val;
    int i;

    for (i = 0; i < n - 1; i++)
    {
        // Read tdo and run a cycle with set tdi
        if(i == 0){
            JtagClockOne(dtm, 0, 0, 0, 1);
            JtagClockOne(dtm, 0, 0, wr_data[i], 1);
            JtagSwitchClock();
        }else{
            JtagClock(dtm, 0, wr_data[i], 1);
        }
        data[i] = dtm->tdo();


    }
    // Read last output and set TMS to get out of SHIFT state
    JtagClockOne(dtm, 1, 0, wr_data[i], 1);
    JtagClockOne(dtm, 0, 1, wr_data[i], 1);
    JtagSwitchClock();
    data[i] = dtm->tdo();
    JtagClockOne(dtm, 1, 1, wr_data[i], 1);






    return data.value();
}

// **************** Main ************************

int sc_main(int argc, char *argv[])
{
    int arg, cfg_help = 0;

    // Parse command line...
    arg = 1;
    while (arg < argc && argv[arg][0] == '-')
    {
        switch (argv[arg][1])
        {
        case 't':
            pn_cfg_vcd_level = MAX(0, MIN(9, argv[arg][2] - '0'));
            fprintf(stderr, "(cfg) vcdLevel = %i\n", pn_cfg_vcd_level);
            break;
        case 'h':
            cfg_help = 1;
            break;
        default:
            printf("PN_ERROR: Unknown option '%s'.\n", argv[arg]);
            arg = argc;
        }
        arg++;
    }
    if (cfg_help)
    {
        puts("Usage: jtag_dtb_tb [<options>]\n"
             "\n"
             "Options:\n"
             "  -t<n>: set VCD trace level (0 = no trace file; default = 2)\n");
        return 3;
    }

    // SystemC elaboration...
    fprintf(stderr, "(sim) Starting SystemC elaboration...\n");
    sc_set_time_resolution(1.0, SC_NS);

    // JTAG DTM
    MDtm *dtm = new MDtm("dtm");
    jtag_dtm_t *jtag_dtm = new jtag_dtm_t(dtm, 1);
    dtm->reset(reset);
    dtm->tck(tck);
    dtm->tms(tms);
    dtm->tdi(tdi);
    dtm->tdo(tdo);
    dtm->dmi_adr(dmi_adr);
    dtm->dmi_dat_i(dmi_dat_i);
    dtm->dmi_dat_o(dmi_dat_o);
    dtm->dmi_wr(dmi_wr);
    dtm->dmi_rd(dmi_rd);

    // Trace file...
    sc_trace_file *tf;
    if (pn_cfg_vcd_level > 0)
    {
        tf = sc_create_vcd_trace_file("jtag_dtm_tb");
        tf->delta_cycles(false);

        PN_TRACE(tf, reset);
        PN_TRACE(tf, tck);
        PN_TRACE(tf, tms);
        PN_TRACE(tf, tdi);
        PN_TRACE(tf, tdo);
        PN_TRACE(tf, dmi_adr);
        PN_TRACE(tf, dmi_dat_i);
        PN_TRACE(tf, dmi_dat_o);
        PN_TRACE(tf, dmi_rd);
        PN_TRACE(tf, dmi_wr);

        dtm->Trace(tf, pn_cfg_vcd_level);
    }
    else
    {
        fprintf(stderr, "Tracing is disabled.\n");
        tf = NULL;
    }

    // Run simulation...
    fprintf(stderr, "(sim) Starting SystemC simulation...\n\n");
    sc_start(SC_ZERO_TIME);

    PN_INFO("Reset...");
    JtagReset(jtag_dtm);
    PN_INFO("Running...");

    uint64_t ret;

    // Test read IDCODE:
    // --------------------
    // JtagReset(jtag_dtm);
    // Go to SHIFT_DR state
    JtagClock(jtag_dtm, 0);
    JtagClock(jtag_dtm, 1);
    JtagClock(jtag_dtm, 0);
    JtagClock(jtag_dtm, 0);
    // Read IDCODE
    ret = JtagRdWr(jtag_dtm, 32, 0xffffffff);
    PN_INFOF(("IDCODE of JTAG DTM: 0x%x", ret));
    PN_ASSERTM(ret == 0xdeadbeef, "Could not read correct IDCODE");

    // abort();

    // Test read DTMCS
    // --------------------
    JtagReset(jtag_dtm);
    // Go to SHIFT_IR state
    JtagClock(jtag_dtm, 0);
    JtagClock(jtag_dtm, 1);
    JtagClock(jtag_dtm, 1);
    JtagClock(jtag_dtm, 0);
    JtagClock(jtag_dtm, 0);
    // Set ir to 0x10 (DTMCS)
    ret = JtagRdWr(jtag_dtm, DTM_IR_WIDTH, DTMCS);
    PN_INFOF(("IR was: 0x%x", ret));
    PN_ASSERTM(ret == IDCODE, "IR was not set to IDCODE");
    // Go to SHIFT_DR
    JtagClock(jtag_dtm, 1);
    JtagClock(jtag_dtm, 0);
    JtagClockOne(jtag_dtm, 0, 0, 0, 1); // Stay one cycle
    JtagClock(jtag_dtm, 1);
    JtagClock(jtag_dtm, 0);
    JtagClock(jtag_dtm, 0);
    // Read DTMCS
    ret = JtagRd(jtag_dtm, 32);
    PN_INFOF(("DTMCS was: 0x%x", ret));
    PN_ASSERTM(ret == (DTM_ADDR_WIDTH << 4) | 1, "DTMSC does not contain the reset value");

    PN_INFOF(("\nTest OpenOCD sequence:\n----------------------------------"));
    // Test OpenOCD sequence:
    // --------------------
    JtagReset(jtag_dtm);
    JtagClock(jtag_dtm, 1);
    // Go to SHIFT_DR state
    JtagClock(jtag_dtm, 0);
    JtagClock(jtag_dtm, 1);
    JtagClock(jtag_dtm, 0);
    JtagClock(jtag_dtm, 0);
    // Read IDCODE
    ret = JtagRdWr(jtag_dtm, 64, 0xffffffffffffffff);
    PN_INFOF(("IDCODE of JTAG DTM: 0x%x", ret));
    PN_ASSERTM(ret == 0xffffffffdeadbeef, "Could not read correct IDCODE");
    // Now in EXIT1_DR, go to PAUSE_DR
    JtagClock(jtag_dtm, 0);
    ClockSwitch = false;
    JtagClock(jtag_dtm, 1, 0, 9); // reset
    JtagClock(jtag_dtm, 0);       // RTI
    JtagClock(jtag_dtm, 1);       // SELECT_DR_SCAN
    JtagClock(jtag_dtm, 1);       // SELECT_IR_SCAN
    JtagClock(jtag_dtm, 0);       // CAPTURE_IR
    JtagClock(jtag_dtm, 0);       // SHIFT_IR
    ret = JtagRdWr(jtag_dtm, DTM_IR_WIDTH, 0xff);
    PN_INFOF(("IR was: 0x%x", ret));
    PN_ASSERTM(ret == IDCODE, "IR was not set to IDCODE");
    JtagClock(jtag_dtm, 1); // UPDATE_IR
    JtagClock(jtag_dtm, 0); // RTI
    JtagClockOne(jtag_dtm, 0, 0, 0, 1); // Stay one cycle
    // Go to SHIFT_IR state
    JtagClock(jtag_dtm, 1);
    JtagClock(jtag_dtm, 1);
    JtagClock(jtag_dtm, 0);
    JtagClock(jtag_dtm, 0);
    // Set ir to 0x10 (DTMCS)
    ret = JtagRdWr(jtag_dtm, DTM_IR_WIDTH, DTMCS);
    PN_INFOF(("IR was: 0x%x", ret));
    PN_ASSERTM(ret == 0x1f, "IR was not set to 0x1f");
    // Go to SHIFT_DR
    JtagClock(jtag_dtm, 1); // UPDATE_IR
    JtagClock(jtag_dtm, 0); // RTI
    JtagClockOne(jtag_dtm, 0, 0, 0, 1); // Stay one cycle
    JtagClock(jtag_dtm, 1); // SELECT_DR_SCAN
    JtagClock(jtag_dtm, 0);
    JtagClock(jtag_dtm, 0);
    // Read DTMCS
    ret = JtagRd(jtag_dtm, 32);
    PN_INFOF(("DTMCS was: 0x%x", ret));
    PN_ASSERTM(ret == (4 << 12) | (DTM_ADDR_WIDTH << 4) | 1,
               "DTMSC does not contain the reset value");
    JtagClock(jtag_dtm, 1); // UPDATE_DR
    JtagClock(jtag_dtm, 0); // RTI
    JtagClockOne(jtag_dtm, 0, 0, 0, 1); // Stay one cycle
    JtagClock(jtag_dtm, 1); // SELECT_IR_SCAN
    JtagClock(jtag_dtm, 1); // SELECT_IR_SCAN
    JtagClock(jtag_dtm, 0); // CAPTURE_IR
    JtagClock(jtag_dtm, 0); // SHIFT_IR
    ret = JtagRdWr(jtag_dtm, DTM_IR_WIDTH, DMI);
    PN_INFOF(("IR was: 0x%x", ret));
    PN_ASSERTM(ret == DTMCS, "IR was not set to DTMCS");
    JtagClock(jtag_dtm, 1); // UPDATE_IR
    JtagClock(jtag_dtm, 0); // RTI
    JtagClockOne(jtag_dtm, 0, 0, 0, 1); // Stay one cycle
    JtagClock(jtag_dtm, 1); // SELECT_DR_SCAN
    JtagClock(jtag_dtm, 0); // CAPTURE_DR
    JtagClock(jtag_dtm, 0); // SHIFT_DR
    ret = JtagRdWr(jtag_dtm, 40, 0x4000000002);
    PN_INFOF(("DMI was: 0x%lx", ret));
    PN_ASSERTM(ret == 0x0, "DMI was not set 0");
    JtagClock(jtag_dtm, 1); // UPDATE_DR
    JtagClock(jtag_dtm, 0); // RTI
    JtagClockOne(jtag_dtm, 0, 0, 0, 1); // Stay one cycle
    JtagClock(jtag_dtm, 1); // SELECT_DR_SCAN
    JtagClock(jtag_dtm, 0); // CAPTURE_DR
    JtagClock(jtag_dtm, 0); // SHIFT_DR
    ret = JtagRdWr(jtag_dtm, 40, 0x4000000000);
    PN_INFOF(("DMI was: 0x%lx", ret));
    PN_ASSERTM(ret == 0x4000000000, "DMI op was not successfull");

    PN_INFO("\nTest DM Read\n---------------------------------------");

    // command to request dmstatus
    uint64_t cmd = 0x4400000001;

    JtagReset(jtag_dtm);
    JtagClock(jtag_dtm, 1);

    // Go into the shift IR state
    JtagClock(jtag_dtm, 0);
    JtagClock(jtag_dtm, 1);
    JtagClock(jtag_dtm, 1);
    JtagClock(jtag_dtm, 0);
    JtagClock(jtag_dtm, 0);

    // set IR to DMI
    ret = JtagRdWr(jtag_dtm, DTM_IR_WIDTH, DMI);

    // go into the dr shift mode
    JtagClock(jtag_dtm, 1);
    JtagClock(jtag_dtm, 1);
    JtagClock(jtag_dtm, 0);
    JtagClock(jtag_dtm, 0);

    dmi_dat_i = 0x0003000001;
    // Write ReadInstruction
    ret = JtagRdWr(jtag_dtm, 40, cmd);

    JtagClock(jtag_dtm, 1 );
    JtagClock(jtag_dtm, 0 );


    PN_INFOF(("DMI_ADR was: \t0x%02x", (uint16_t)dmi_adr.read()));
    PN_ASSERTM(dmi_adr.read() == sc_uint<6>(17), "Not address of dmstatus");
    PN_INFOF(("DMI_DAT_O was: \t0x%08x", (uint32_t)dmi_dat_o.read()));
    PN_ASSERTM(dmi_dat_o.read() == 0, "data present");
    PN_INFOF(("DMI_RD was: \t%d", (uint8_t)dmi_rd.read()));
    PN_ASSERTM(dmi_rd.read() == sc_uint<1>(1), "read bit not set");


    PN_INFO("Simulation finished.");
    if (tf)
        sc_close_vcd_trace_file(tf);

    return 0;
}
