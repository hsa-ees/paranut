/*************************************************************************

  This file is part of the ParaNut project.

  Copyright (C) 2010-2020 Alexander Bahle <alexander.bahle@hs-augsburg.de>
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


#include "paranut.h"
#include "peripherals.h"
#include "remote_bitbang.h"
#include "config.h"

#include <stdio.h>

#include <signal.h>
#include <systemc.h>


// **************** Signals *********************

sc_signal<bool> clk, reset;
sc_signal<bool> wb_stb, wb_cyc, wb_we, wb_ack, wb_err, wb_rty;
sc_signal<sc_uint<CFG_MEMU_BUSIF_WIDTH/8>> wb_sel;
sc_signal<TWord> wb_adr;
sc_signal<sc_uint<CFG_MEMU_BUSIF_WIDTH> > wb_dat_w, wb_dat_r;
// Interrupt:
sc_signal<sc_uint<CFG_NUT_EX_INT>> ex_int;
// JTAG
sc_signal<bool> tck, tms, tdi, tdo;


// **************** Helpers *********************

#define CLK_PERIOD (1000000000 / CFG_NUT_SIM_CLK_SPEED)

void RunCycles (int n = 1) {
    for (int k = 0; k < n; k++) {
        clk = 1;
        sc_start (CLK_PERIOD / 2, SC_NS);
        clk = 0;
        sc_start (CLK_PERIOD / 2, SC_NS);
    }
}

void SimHost (CMemory *mem) {
    char tohost;
    if (mem->tohost_adr != 0) {
        tohost = mem->ReadByte (mem->tohost_adr);
        if (tohost != 0) {
            fprintf (stdout, "%c", tohost);
            fflush (stdout);
            mem->WriteByte (mem->tohost_adr, 0);
        }
    }
}

static volatile bool signal_exit = false;

static void intHandler (int sig) {
    if (sig == SIGABRT || signal_exit) // someone set up us the bomb!
        exit (-1);
    // fprintf(stderr, "\r\n\r\nInteractive Debug Mode: \r\n");
//    cfg_insn_trace = 1;
    signal_exit = true;

    signal (sig, &intHandler);
}

void usage () {
    puts ("Usage: paranut_tb [<options>] <ELF file>\n"
          "\n"
          "Options:\n"
          "  -t<n>: set VCD trace level (0 = no trace file = default)\n"
          "  -p<n>: set number of cores to display performance statistics of (0 = no display = default)\n"
          "  -i: generate instruction trace\n"
          "  -c: disable caching\n"
          "  -m <from> <to>: dump memory region before/after running the program\n"
          "  -v: dump program memory content to VHDL file\n"
          "  -s: dump singature to file\n"
          "  -d: start in interactive debugging mode");
}

// **************** Main ************************

int sc_main (int argc, char *argv[]) {
    CMemory memory (CFG_NUT_SIM_MEM_ADDR, CFG_NUT_MEM_SIZE);
    MParanut *g_nut;
    MPeripherals *g_peri;

    char *elf_filename;
    int arg, dump_from, dump_to;
    int perf_level = 0;
    bool dump_VHDL, dump_signature, dbg_interactive, last_reset;
    jtag_dtm_t *jtag_dtm;
    remote_bitbang_t *rbb;


    // Register interrupt handler...
    signal (SIGINT, &intHandler);
    signal (SIGTERM, &intHandler);
    signal (SIGABRT, &intHandler); // we still want to call static destructors

    // Parse command line...
    dump_VHDL = dump_signature = dbg_interactive = false;
    dump_from = dump_to = 0;
    elf_filename = NULL;
    arg = 1;
    while (arg < argc && argv[arg][0] == '-') {
        switch (argv[arg][1]) {
        case 'h':
            usage ();
            exit(0);
            break;
        case 't':
            cfg_vcd_level = MAX (0, MIN (9, argv[arg][2] - '0'));
            INFOF (("Args: Setting VCD trace level to %i.", cfg_vcd_level));
            break;
        case 'p':
            perf_level = MAX (0, MIN (CFG_NUT_CPU_CORES, (int)strtol (&argv[arg][2], NULL, 0)));
            INFOF (("Args: Setting number of cores to display performance statistics to %i.", perf_level));
            break;
        case 'i':
            cfg_insn_trace = 1;
            INFO ("Args: Activating instruction trace.");
            break;
        case 'c':
            cfg_disable_cache = 1;
            INFO ("Args: Disabling caches.");
            break;
        case 'm':
            dump_from = (int)strtol (argv[++arg], NULL, 0);
            dump_to = (int)strtol (argv[++arg], NULL, 0);
            INFOF (("Args: Dumping memory from 0x%x to 0x%x (%s to %s)", dump_from,
                     dump_to, argv[arg - 1], argv[arg]));
            break;
        case 'v':
            dump_VHDL = true;
            INFO ("Args: Dumping program memory to VHDL file.");
            break;
        case 's':
            dump_signature = true;
            INFO ("Args: Dumping singature to file.");
            break;
        case 'd':
            dbg_interactive = true;
            INFO ("Args: Starting interactive debugging mode.");
            break;
        default:
            WARNINGF (("ERROR: Unknown option '%s'.\n", argv[arg]));
            arg = argc;
        }
        arg++;
    }
    if (arg < argc) elf_filename = argv[arg];
    if (!elf_filename) {
        usage ();
        return 3;
    }

    // Read ELF file...
    INFOF (("Starting read of ELF file '%s'...", elf_filename));
    if (!memory.ReadFile (elf_filename, dump_VHDL)) {
        ERRORF (("Unable to read ELF file '%s'.", elf_filename));
    }
    INFO ("Read of ELF file completed.");

    // Dump memory before simulation
    if (dump_from < dump_to) {
        INFOF (("Starting memory dump from 0x%x to 0x%x...", dump_from, dump_to));
        memory.Dump (dump_from, dump_to);
        INFO ("Memory dump completed.");
    }

    // SystemC elaboration...
    INFO ("Starting SystemC elaboration...");
    sc_set_time_resolution (1.0, SC_NS);

    // Peripherals ...
    MPeripherals peri ("peripherals", &memory);
    g_peri = &peri;
    peri.clk_i (clk);
    peri.rst_i (reset);
    peri.stb_i (wb_stb);
    peri.cyc_i (wb_cyc);
    peri.we_i (wb_we);
    peri.sel_i (wb_sel);
    peri.adr_i (wb_adr);
    peri.dat_i (wb_dat_w);
    peri.ack_o (wb_ack);
    peri.err_o (wb_err);
    peri.rty_o (wb_rty);
    peri.dat_o (wb_dat_r);


    // ParaNut ...
    MParanut nut ("nut");
    g_nut = &nut;
    nut.clk_i (clk);
    nut.rst_i (reset);
    nut.stb_o (wb_stb);
    nut.cyc_o (wb_cyc);
    nut.we_o (wb_we);
    nut.ack_i (wb_ack);
    nut.err_i (wb_err);
    nut.rty_i (wb_rty);
    nut.sel_o (wb_sel);
    nut.adr_o (wb_adr);
    nut.dat_o (wb_dat_w);
    nut.dat_i (wb_dat_r);

    nut.ex_int (ex_int);

    // These don't do anything during simulation
    nut.tck (clk); // JTAG_DTM gets clk so the JTAG_METHOD gets executed
    nut.tms (tms);
    nut.tdi (tdi);
    nut.tdo (tdo);

    // Trace file...
    sc_trace_file *tf;
    if (cfg_vcd_level > 0) {
        tf = sc_create_vcd_trace_file ("paranut_tb");
        tf->set_time_unit(1, SC_NS);
        tf->delta_cycles (false);

        TRACE (tf, clk);
        TRACE (tf, reset);
        TRACE (tf, wb_stb);
        TRACE (tf, wb_cyc);
        TRACE (tf, wb_we);
        TRACE (tf, wb_ack);
        TRACE (tf, wb_err);
        TRACE (tf, wb_rty);
        TRACE (tf, wb_sel);
        TRACE (tf, wb_adr);
        TRACE (tf, wb_dat_w);
        TRACE (tf, wb_dat_r);

        nut.Trace (tf, cfg_vcd_level);
    } else {
        INFO ("Tracing is disabled.");
        tf = NULL;
    }
    INFO ("SystemC elaboration completed.");

    // Run simulation...
    INFO ("Starting SystemC simulation...");
    sc_start (SC_ZERO_TIME);

    // Reset system for 5 cycles
    reset = 1;
    RunCycles (5);
    reset = 0;

    if (dbg_interactive) {
        // Set up debugging
        INFO ("Setting up debugging...");
        jtag_dtm = new jtag_dtm_t (nut.dtm, 1);
        rbb = new remote_bitbang_t (9824, jtag_dtm);
        INFO ("Setting up debugging completed, waiting for connection...");

        // Halt all CPUs if started in interactive mode
        nut.dm->SetHaltreq (1);
        while (1) // User ends simulation
        {
            // Speed up debugging by automatically reading the ELF File (once) again on reset
            if (nut.dm->dbg_reset.read () == 1 && last_reset == 0) {
                INFO ("Reset asserted, reading ELF file...");
                if (!memory.ReadFile (elf_filename, dump_VHDL)) {
                    ERRORF (("Unable to read ELF file '%s'.", elf_filename));
                }
                INFO ("Read of ELF file completed.");
            }
            last_reset = nut.dm->dbg_reset.read ();

            // Run simulation for a few ciycles (DM needs at least 3 cycles to
            // execute DMI read/write operation)
            RunCycles (5);
            SimHost (&memory);

            // Run RBB server for one tick
            rbb->tick ();
        }
    } else {
        // Normal execution of simulation loop
        while (!nut.IsHalted ()) { // CePU ends simulation
            RunCycles (1);
            SimHost (&memory);
        }
        INFO ("CePU has reached HALT.");
    }

    // Run for 10 cycles more to get a clear ending in VCD file
    RunCycles (10);
    INFO ("SystemC Simulation completed.");

    if (tf) sc_close_vcd_trace_file (tf);

    // Display statistics
    if (perf_level > 0 ) {
        INFOF (("Starting performance dump for %d cores.", perf_level));
        for (int i = 0; i < perf_level; i++)
            nut.DisplayStatistics (i);
        INFO ("Performance dump completed.");
    }

    // Dump memory after simulation is finished
    if (dump_from < dump_to) {
        INFOF (("Starting memory dump from 0x%x to 0x%x...", dump_from, dump_to));
        memory.Dump (dump_from, dump_to);
        INFO ("Memory dump completed.");
    }

    // Dump signature information
    if (dump_signature) {
        memory.DumpSignature (elf_filename);
    }

    return 0;
}
