/*************************************************************************

  This file is part of the ParaNut project.

  Copyright (C) 2010-2015 Gundolf Kiefer <gundolf.kiefer@hs-augsburg.de>
      Hochschule Augsburg, University of Applied Sciences

  Description:
    This is a test bench for the memory unit (MEMU) of the ParaNut.

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

// include hardware sources
#include "memu_cache.h"
#include "paranut-config.h"

// all of this is unnecessary in synthesis
#ifndef __SYNTHESIS__
    // IO
    #include <stdio.h>

    #define WORDS_BIGENDIAN 0
    #if WORDS_BIGENDIAN == 1
    // Big Endian
    #define BSEL
    #else
    // Little Endian
    #define BSEL(bsel) (((sc_uint<8>)(bsel << 3) & 0x8) |\
                    ((sc_uint<8>)(bsel << 1) & 0x4) |\
                    ((sc_uint<8>)(bsel >> 1) & 0x2) |\
                    ((sc_uint<8>)(bsel >> 3) & 0x1))
    #endif


    // **************** Helpers *********************
    #define CLK_PERIOD 10.0

    #define ASSERT(expr) \
    if (!(expr)){ \
    printf("%s:%i: %s: Assertion '%s' failed\n", \
    __FILE__, __LINE__, __ASSERT_FUNCTION, #expr); \
    cleanExitOnAssert(); \
    }

    #define ASSERTF(expr, msg, ...) \
    if (!(expr)){ \
    printf("%s:%i: %s: Assertion '%s' failed\n", \
    __FILE__, __LINE__, __ASSERT_FUNCTION, #expr); \
    printf ((msg), ##__VA_ARGS__); \
    printf ("\n"); \
    cleanExitOnAssert(); \
    }

    // **************** Testbench *********************
#endif



// **************** Trace level *******************
struct Tb : sc_module 
{
    // Trace file pointer
    sc_trace_file *tf = NULL;

    sc_in_clk clk{"clk"};
    sc_signal<bool> reset;
    sc_signal<bool> rd[CFG_MEMU_BANK_RAM_PORTS], wr[CFG_MEMU_BANK_RAM_PORTS];

    sc_signal<sc_uint<4> > wen[CFG_MEMU_BANK_RAM_PORTS]; // byte write enable bits

    sc_signal<sc_uint<32> > wiadr[CFG_MEMU_BANK_RAM_PORTS]; // line adress (way & index bits of adress)
    sc_signal<sc_uint<32> > wdata[CFG_MEMU_BANK_RAM_PORTS];
    sc_signal<sc_uint<32> > rdata[CFG_MEMU_BANK_RAM_PORTS];
    
    // MWBMemory memory;
    public:
    MBankRam bankRam{"bankRam"};

    // Constructor
    SC_CTOR(Tb)
    {
        // Connect signals
        bankRam.clk (clk);
        for (uint k = 0; k < CFG_MEMU_BANK_RAM_PORTS; k++) {
            bankRam.rd[k] (rd[k]);
            bankRam.wr[k] (wr[k]);
            bankRam.wen[k] (wen[k]);
            bankRam.wiadr[k] (wiadr[k]);
            bankRam.wdata[k] (wdata[k]);
            bankRam.rdata[k] (rdata[k]);
        }
        
       // register routine
        SC_CTHREAD(RunTest, clk.pos());  
    }

    void Trace(sc_trace_file *tf, int level = 3){
            // no trace in hardware

    }



    void RunTest () {

        #ifndef __SYNTHESIS__
        pn_cfg_vcd_level = 1;
        if (pn_cfg_vcd_level > 0) {
            tf = sc_create_vcd_trace_file ("memu_tb");
            tf->delta_cycles (false);

            Trace(tf, pn_cfg_vcd_level);
        }

        PN_INFO ("Simulation starting...");
        reset = 1;
        wait (5);
        reset = 0;

    
        finish();
        #endif
    }

    void finish(){
        // Simulation finished...
        PN_INFO ("Simulation finished.");

        wait (); // Wait some time...
        sc_stop();
        #ifndef __SYNTHESIS__
            if (tf) sc_close_vcd_trace_file (tf);
        #endif
    }

    void cleanExitOnAssert(){
        finish();
        exit(0);
    }
};


// **************** Main ************************

int cfg_help = 0;

int sc_main (int argc, char *argv[]) {
    // Synthesis needs sc_main to be defined, however it does not 
    // have to implement any functionality
    #ifndef __SYNTHESIS__
    int arg;

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
            cfg_help = 1;
            arg = argc;
        }
        arg++;
    }
    if (cfg_help) {
        puts ("Usage: memu_tb [<options>] <OR32 ELF file>\n"
              "\n"
              "Options:\n"
              "  -t<n>: set VCD trace level (0 = no trace file; default = 2)\n");
        return 3;
    }
    // Init Testbench
    sc_clock clk("clk", sc_time(1, SC_NS));
    Tb tb("tb");
    tb.clk(clk);

    
    // Run simulation...
    sc_start ();

    // Finish...
    cout <<"\n\t\t*****Simulation complete*****" << endl;
    #else
    // this is all we need for synthesis
    // Init Testbench
    sc_clock clk("clk", sc_time(1, SC_NS));
    Tb tb("tb");
    tb.clk(clk);
    sc_start ();
    
    cout << "\n*** To get a simulation output here, run \"cmake -DSYN=OFF ../\" in terminal ***" << endl;
    #endif
    return 0;
}
