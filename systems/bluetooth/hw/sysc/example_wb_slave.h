/**************************************************************************
 *
 *  This file is part of the ParaNut project.
 *
 *  Copyright (C) 2020-2021 Gundolf Kiefer <gundolf.kiefer@hs-augsburg.de>
 *                          Michael Schaeferling <michael.schaeferling@hs-augsburg.de>
 *                          Alexander Bahle <alexander.bahle@hs-augsburg.de>
 *                          Felix Wagner <felix.wagner1@hs-augsburg.de>
 *      Hochschule Augsburg, University of Applied Sciences
 *
 *  Description:
 *    This is an example of a Wishbone slave peripheral module.
 *
 *  --------------------- LICENSE -----------------------------------------------
 *  Redistribution and use in source and binary forms, with or without modification,
 *  are permitted provided that the following conditions are met:
 *
 *  1. Redistributions of source code must retain the above copyright notice, this
 *     list of conditions and the following disclaimer.
 *
 *  2. Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation and/or
 *     other materials provided with the distribution.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 *  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 *  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 *  ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 *  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 *  ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 *  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 **************************************************************************/


#ifndef _EXAMPLE_WB_SLAVE_H
#define _EXAMPLE_WB_SLAVE_H


#include <systemc.h>

// Wishbone bus width of this module.
// Typically, it is 32 and the same as the bus width of the ParaNut processor,
// which is defined through CFG_MEMU_BUSIF_WIDTH in the 'config.mk' file.
#define WB_WIDTH 32           


SC_MODULE (example_wb_slave) {
public:

    // Ports (WISHBONE slave)...
    // <-- DON'T CHANGE!
    sc_in_clk           wb_clk_i;          // clock input
    sc_in<bool>         wb_rst_i;          // reset

    sc_in<bool>         wb_stb_i;          // strobe input
    sc_in<bool>         wb_cyc_i;          // cycle valid input
    sc_in<bool>         wb_we_i;           // indicates write transfer
    sc_in<sc_uint<3> >  wb_cti_i;          // cycle type identifier (optional, for registered feedback)
    sc_in<sc_uint<2> >  wb_bte_i;          // burst type extension (optional, for registered feedback)
    sc_in<sc_uint<WB_WIDTH/8> > wb_sel_i;  // byte select inputs
    sc_out<bool>        wb_ack_o;          // normal termination
    sc_out<bool>        wb_err_o;          // termination w/ error (optional)
    sc_out<bool>        wb_rty_o;          // termination w/ retry (optional)

    sc_in<sc_uint<32> >        wb_adr_i;   // address bus inputs
    sc_in<sc_uint<WB_WIDTH> >  wb_dat_i;   // input data bus
    sc_out<sc_uint<WB_WIDTH> > wb_dat_o;   // output data bus
    // DON'T CHANGE! -->

    // User Ports...
    //   EDIT HERE: Add ports here & don't forget to connect them
    //sc_out<sc_int<16> > audio_l, audio_r;

    // Signal Tracing (simulation) ...
#ifndef __SYNTHESIS__
    void trace (sc_trace_file * tf, int level = 1);
      // Add signals to the trace file 'tf'.
      // The 'level' argument can be used to control a level of detail.
      // Only signals above the given level are be traced.
#endif

    // I/O Registers: Processes ...
    void proc_ioregs_transition ();
      // Perform all actions to maintain the IO registers and handle WB read and write transactions.
      // (DON'T CHANGE!)

    // I/O Registers: Callback for status registers ...
    //   Inside user processes it is not allowed to write to any I/O registers
    //   ('ioregs_regs[]') directly. Instead this callback needs to be extended
    //   accordingly. This method is called synchronously with each clock cycle.
    //   'val' is the preset default value, which may by the current content of
    //   the register or a value originating from a bus write access.
    sc_uint<32> callback_ioregs_writeback (int n, sc_uint<32> val) {
      switch (n) {
        // EDIT HERE: Add your combinational logic for status registers here
        //case 2: return (sc_uint<32>) counter_reg.read();
        //case 3: return (val & ~0xff) | somebyte_reg.read ();
        default: return val;
      }
    }

    // User-Specific Processes ...
    //   EDIT HERE: Add SystemC processes below & don't forget to define the type
    //   in the constructor and set the sensitivity list accordingly.

    // Constructor ...
    SC_CTOR (example_wb_slave) {
        SC_METHOD (proc_ioregs_transition);
            sensitive << wb_clk_i.pos();
        // EDIT HERE: Add your user-specific processes below ...

    }

private:

    // I/O Registers ...
    static const int ioregs_num_ld = 2;                     // Log2 of the number of WB slave registers
    static const int ioregs_num = (1 << ioregs_num_ld);     // Number of WB slave registers
    sc_signal<sc_uint<32> > ioregs_regs[ioregs_num];

    // User-Specific Signals ...
    //   EDIT HERE: Add your user-specific signals below.

};


#endif // _EXAMPLE_WB_SLAVE_H
