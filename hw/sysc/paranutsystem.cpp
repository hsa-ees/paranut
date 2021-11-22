/*************************************************************************

  This file is part of the ParaNut project.

  Copyright (C) 2020 Alexander Bahle <alexander.bahle@hs-augsburg.de>
      Hochschule Augsburg, University of Applied Sciences

  Description:
    This is the top-level SystemC model of a complete ParaNut system including
    the ParaNut itself, a Wishbone interconnect and the main memory.

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


#include "paranutsystem.h"

// **************** Helpers *********************

#define CLK_PERIOD (1000000000 / CFG_NUT_SIM_CLK_SPEED)

#define PS_PRINTCAT(CAT)    fprintf (stderr, "\n%7s %s\n", " ", CAT)
#define PS_PRINTCONF(CONF)  fprintf (stderr, "%11s %-30s %d (0x%x)\n", " ", #CONF, CONF, CONF)

void MParaNutSystem::RunCycles (const int n) {
    for (int k = 0; k < n; k++) {
        clk = 1;
        sc_start (CLK_PERIOD / 2, SC_NS);
        clk = 0;
        sc_start (CLK_PERIOD / 2, SC_NS);
    }
}

void MParaNutSystem::SimHost () {
    char tohost;
    if (mmemory->tohost_adr != 0) {
        tohost = mmemory->ReadByte (mmemory->tohost_adr);
        if (tohost != 0) {
            fprintf (stdout, "%c", tohost);
            fflush (stdout);
            mmemory->WriteByte (mmemory->tohost_adr, 0);
        }
    }
}

void MParaNutSystem::Usage () {
    extern const char *__progname; // Use glibc __progname instead of argv[0]
    std::cout << "Usage: " << __progname << " [<options>] <ELF file>\n"
                  "\n"
                  "Options:\n"
                  "  -t<n>: set VCD trace level (0 = no trace file = default)\n"
                  "  -p<n>: set number of cores to display performance statistics of (0 = no display = default)\n"
                  "  -i<n>: set generate instruction trace level (0= no trace = default)\n"
                  "  -c: disable caching\n"
                  "  -m <from> <to>: dump memory region before/after running the program\n"
                  "  -v: dump program memory content to VHDL file\n"
                  "  -s: dump singature to file\n"
                  "  -d: start in interactive debugging mode\n";
}

void MParaNutSystem::AddInterrupt(TWord index, sc_out<bool> *port) {
    if (index > CFG_NUT_EX_INT-1)
        PN_ERRORF (("AddInterrupt: Can not add port \"%s\" at index %d - Not enough interrupt inputs: %d", port->name (), index, CFG_NUT_EX_INT));

    port->bind (ex_int[index]);
}

void MParaNutSystem::PrintConfig() {
    PN_INFO ("Print full configuration:");
    PS_PRINTCAT ("SystemC Simulation options...");
    PS_PRINTCONF (CFG_NUT_SIM_CLK_SPEED);
    PS_PRINTCONF (CFG_NUT_RESET_ADDR);
    PS_PRINTCONF (CFG_NUT_SIM_MAX_PERIPHERY);

    PS_PRINTCAT ("General options...");
    PS_PRINTCONF (CFG_NUT_CPU_CORES);
    PS_PRINTCONF (CFG_NUT_CPU_MODE1_CORES);
    PS_PRINTCONF (CFG_NUT_CPU_MODE2_CORES);
    PS_PRINTCONF (CFG_EXU_PNM2CAP);
    PS_PRINTCONF (CFG_NUT_MEM_SIZE);
    PS_PRINTCONF (CFG_NUT_EX_INT);

    PS_PRINTCAT ("EXU options...");
    PS_PRINTCONF (CFG_EXU_M_EXTENSION);
    PS_PRINTCONF (CFG_EXU_A_EXTENSION);
    PS_PRINTCONF (CFG_EXU_PERFCOUNT_ENABLE);
    PS_PRINTCONF (CFG_EXU_PERFCOUNTER_BITS);
    PS_PRINTCONF (CFG_EXU_PERFCOUNTERS);

    PS_PRINTCAT ("MemU options...");
    PS_PRINTCONF (CFG_MEMU_CACHE_SIZE);
    PS_PRINTCONF (CFG_MEMU_CACHE_BANKS);
    PS_PRINTCONF (CFG_MEMU_CACHE_SETS);
    PS_PRINTCONF (CFG_MEMU_CACHE_WAYS);
    PS_PRINTCONF (CFG_MEMU_BANK_RAM_PORTS);
    PS_PRINTCONF (CFG_MEMU_CACHE_REPLACE_LRU);
    PS_PRINTCONF (CFG_MEMU_ARBITER_METHOD);
    PS_PRINTCONF (CFG_MEMU_BUSIF_WIDTH);

    PS_PRINTCAT ("IFU options...");
    PS_PRINTCONF (CFG_IFU_IBUF_SIZE);

    PS_PRINTCAT ("LSU options...");
    PS_PRINTCONF (CFG_LSU_WBUF_SIZE);
}


MParaNutSystem::MParaNutSystem(const int argc, char *argv[]) {
    int arg;

    // Init Submodules...
    paranut = new MParanut ("paranut");
    interconnect = new MInterconnect ("interconnect");
    mmemory = new MWBMemory ("mmemory", CFG_NUT_RESET_ADDR, CFG_NUT_MEM_SIZE);

    jtag_dtm = NULL;
    rbb = NULL;

    // Put a newline after the SystemC header
    std::cout << "\n";

    // Parse command line...
    cfg_dump_VHDL = cfg_dump_signature = false;
    cfg_dump_from = cfg_dump_to = 0;
    cfg_elf_filename = NULL;
    arg = 1;
    while (arg < argc && argv[arg][0] == '-') {
        switch (argv[arg][1]) {
        case 'h':
            Usage ();
            exit(0);
            break;
        case 't':
            pn_cfg_vcd_level = MAX (0, MIN (9, argv[arg][2] - '0'));
            PN_INFOF (("Args: Setting VCD trace level to %i.", pn_cfg_vcd_level));
            break;
        case 'p':
            cfg_perf_lvl = MAX (0, MIN (CFG_NUT_CPU_CORES, (int)strtol (&argv[arg][2], NULL, 0)));
            PN_INFOF (("Args: Setting number of cores to display performance statistics to %i.", cfg_perf_lvl));
            break;
        case 'i':
            pn_cfg_insn_trace = MAX (0, MIN (9, argv[arg][2] - '0'));
            PN_INFO ("Args: Activating instruction trace.");
            break;
        case 'c':
            pn_cfg_disable_cache = 1;
            PN_INFO ("Args: Disabling caches.");
            break;
        case 'm':
            cfg_dump_from = (int)strtol (argv[++arg], NULL, 0);
            cfg_dump_to = (int)strtol (argv[++arg], NULL, 0);
            PN_INFOF (("Args: Dumping memory from 0x%x to 0x%x (%s to %s)", cfg_dump_from,
                     cfg_dump_to, argv[arg - 1], argv[arg]));
            break;
        case 'v':
            cfg_dump_VHDL = true;
            PN_INFO ("Args: Dumping program memory to VHDL file.");
            break;
        case 's':
            cfg_dump_signature = true;
            PN_INFO ("Args: Dumping singature to file.");
            break;
        case 'd':
            pn_cfg_debug_mode = true;
            PN_INFO ("Args: Starting interactive debugging mode.");
            break;
        default:
            PN_WARNINGF (("PN_ERROR: Unknown option '%s'.\n", argv[arg]));
            arg = argc;
        }
        arg++;
    }
    if (arg < argc) cfg_elf_filename = argv[arg];
    if (!cfg_elf_filename) {
        Usage ();
        exit(0);
    }

    // Read ELF file...
    PN_INFOF (("Starting read of ELF file '%s'...", cfg_elf_filename));
    if (!mmemory->ReadFile (cfg_elf_filename, cfg_dump_VHDL)) {
        PN_ERRORF (("Unable to read ELF file '%s'.", cfg_elf_filename));
    }
    PN_INFO ("Read of ELF file completed.");

    // Dump memory before simulation
    if (cfg_dump_from < cfg_dump_to) {
        PN_INFOF (("Starting memory dump from 0x%x to 0x%x...", cfg_dump_from, cfg_dump_to));
        mmemory->Dump (cfg_dump_from, cfg_dump_to);
        PN_INFO ("Memory dump completed.");
    }

    // SystemC elaboration...
    PN_INFO ("Starting SystemC elaboration...");
    sc_set_time_resolution (1.0, SC_NS);

    // interconnect->..
    interconnect->clk_i (clk);
    interconnect->rst_i (reset);
    interconnect->stb_i (wb_stb);
    interconnect->cyc_i (wb_cyc);
    interconnect->we_i (wb_we);
    interconnect->cti_i (wb_cti);
    interconnect->bte_i (wb_bte);
    interconnect->sel_i (wb_sel);
    interconnect->adr_i (wb_adr);
    interconnect->dat_i (wb_dat_w);
    interconnect->ack_o (wb_ack);
    interconnect->err_o (wb_err);
    interconnect->rty_o (wb_rty);
    interconnect->dat_o (wb_dat_r);

    // WBMemory ...
    interconnect->AddSlave (CFG_NUT_RESET_ADDR, CFG_NUT_MEM_SIZE, mmemory);

    // Add a dummy periphery for the debug module (is routed internally in paranut->h/cpp)
    // We need this to check for address space collisions in the Interconnect
    // TBD: Would be cleaner to add a WB slave interface to the ParaNut top-level and route
    //      the signals on this level. But that would be in contrast to how it is done in HW.
    interconnect->AddSlave (0x0, 0x500, new MPeripheral("DebugDummy"));

    // ParaNut ...
    paranut->clk_i (clk);
    paranut->rst_i (reset);
    paranut->stb_o (wb_stb);
    paranut->cyc_o (wb_cyc);
    paranut->we_o (wb_we);
    paranut->cti_o (wb_cti);
    paranut->bte_o (wb_bte);
    paranut->ack_i (wb_ack);
    paranut->err_i (wb_err);
    paranut->rty_i (wb_rty);
    paranut->sel_o (wb_sel);
    paranut->adr_o (wb_adr);
    paranut->dat_o (wb_dat_w);
    paranut->dat_i (wb_dat_r);

    for (int n = 0; n < CFG_NUT_EX_INT; ++n)
        paranut->ex_int[n] (ex_int[n]);

    // These don't do anything during simulation
    paranut->tck (clk); // JTAG_DTM gets clk so the JTAG_METHOD gets executed
    paranut->tms (tms);
    paranut->tdi (tdi);
    paranut->tdo (tdo);

    // Trace file...
    if (pn_cfg_vcd_level > 0) {
        tf = sc_create_vcd_trace_file ("paranut_tb");
        tf->set_time_unit(1, SC_NS);
        tf->delta_cycles (false);
        pn_trace_file = tf;

        PN_TRACE (tf, clk);
        PN_TRACE (tf, reset);
        PN_TRACE (tf, wb_stb);
        PN_TRACE (tf, wb_cyc);
        PN_TRACE (tf, wb_we);
        PN_TRACE (tf, wb_cti);
        PN_TRACE (tf, wb_bte);
        PN_TRACE (tf, wb_ack);
        PN_TRACE (tf, wb_err);
        PN_TRACE (tf, wb_rty);
        PN_TRACE (tf, wb_sel);
        PN_TRACE (tf, wb_adr);
        PN_TRACE (tf, wb_dat_w);
        PN_TRACE (tf, wb_dat_r);

        paranut->Trace (tf, pn_cfg_vcd_level);
        interconnect->Trace(tf, pn_cfg_vcd_level);
    } else {
        PN_INFO ("Tracing is disabled.");
        tf = NULL;
    }
    PN_INFO ("SystemC elaboration completed.");
}

MParaNutSystem::~MParaNutSystem() {
    delete paranut;
    delete mmemory;
    delete interconnect;
    delete jtag_dtm;
    delete rbb;
}

void MParaNutSystem::Run() {
    bool last_reset;

    // Run simulation...
    PN_INFO ("Starting SystemC simulation...");
    sc_start (SC_ZERO_TIME);

    // Reset system for 5 cycles
    reset = 1;
    RunCycles (5);
    reset = 0;

    if (pn_cfg_debug_mode) {
        // Set up debugging
        PN_INFO ("Setting up debugging...");
        jtag_dtm = new jtag_dtm_t (paranut->dtm, 1);
        rbb = new remote_bitbang_t (9824, jtag_dtm);
        PN_INFO ("Setting up debugging completed, waiting for connection...");

        // Halt all CPUs if started in interactive mode
        paranut->dm->SetHaltreq (1);
        while (1) // User ends simulation
        {
            // Speed up debugging by automatically reading the ELF File (once) again on reset
            if (paranut->dm->dbg_reset.read () == 1 && last_reset == false) {
                PN_INFO ("Reset asserted, reading ELF file...");
                if (!mmemory->ReadFile (cfg_elf_filename, cfg_dump_VHDL)) {
                    PN_ERRORF (("Unable to read ELF file '%s'.", cfg_elf_filename));
                }
                PN_INFO ("Read of ELF file completed.");
            }
            last_reset = paranut->dm->dbg_reset.read ();

            // Run simulation for a few ciycles (DM needs at least 3 cycles to
            // execute DMI read/write operation)
            RunCycles (5);
            SimHost ();

            // Run RBB server for one tick
            rbb->tick ();
        }
    } else {
        // Normal execution of simulation loop
        while (!paranut->IsHalted ()) { // CePU ends simulation
            RunCycles (1);
            SimHost ();
        }
        PN_INFO ("CePU has reached HALT.");
    }

    // Run for 10 cycles more to get a clear ending in VCD file
    RunCycles (10);
    PN_INFO ("SystemC Simulation completed.");

    if (tf) sc_close_vcd_trace_file (tf);

    // Display statistics
    if (cfg_perf_lvl > 0 ) {
        PN_INFOF (("Starting performance dump for %d cores.", cfg_perf_lvl));
        for (int i = 0; i < cfg_perf_lvl; i++)
            paranut->DisplayStatistics (i);
        PN_INFO ("Performance dump completed.");
    }

    // Dump memory after simulation is finished
    if (cfg_dump_from < cfg_dump_to) {
        PN_INFOF (("Starting memoryory dump from 0x%x to 0x%x...", cfg_dump_from, cfg_dump_to));
        mmemory->Dump (cfg_dump_from, cfg_dump_to);
        PN_INFO ("Memory dump completed.");
    }

    // Dump signature information
    if (cfg_dump_signature) {
        mmemory->DumpSignature (cfg_elf_filename);
    }

}


