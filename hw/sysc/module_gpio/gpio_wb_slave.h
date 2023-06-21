/**************************************************************************
 *
 *  This file is part of the ParaNut project.
 *
 *  Copyright (C) 2020-2021 Gundolf Kiefer <gundolf.kiefer@hs-augsburg.de>
 *                          Michael Schaeferling <michael.schaeferling@hs-augsburg.de>
 *                          Alexander Bahle <alexander.bahle@hs-augsburg.de>
 *                     2023 Elias Schuler <elias.schuler@hs-augsburg.de>
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


#ifndef _GPIO_WB_SLAVE_H
#define _GPIO_WB_SLAVE_H


#include <systemc.h>
#include "paranut-config.h"

// Wishbone bus width of this module.
// Typically, it is 32 and the same as the bus width of the ParaNut processor,
// which is defined through CFG_MEMU_BUSIF_WIDTH in the 'config.mk' file.
#define WB_WIDTH 32        


SC_MODULE (gpio_wb_slave) {
public:

    // Ports (WISHBONE slave)...
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

    // User Ports...
    sc_vector<sc_in<bool> > gpio_input_port;
    sc_vector<sc_out<bool> > gpio_output_port;

    // Signal Tracing (simulation) ...
#ifndef __SYNTHESIS__
    void trace (sc_trace_file * tf, int level = 1);
      // Add signals to the trace file 'tf'.
      // The 'level' argument can be used to control a level of detail.
      // Only signals above the given level are be traced.
#endif

    // I/O Registers: Processes ...
    void proc_ioregs_transition ();
    // Process to change the GPIO Output Pins
    void proc_clk_gpio_out();
    // Process to read the GPIO Input Pins
    void proc_clk_gpio_in();

    // Constructor ...
    SC_CTOR (gpio_wb_slave)
    : gpio_input_port("gpio_input_port", CFG_GPIO_INAMOUNT)
    , gpio_output_port("gpio_output_port", CFG_GPIO_OUTAMOUNT)
    {
        SC_CTHREAD (proc_ioregs_transition, wb_clk_i.pos());
        reset_signal_is(wb_rst_i, true);

        SC_CTHREAD(proc_clk_gpio_out,wb_clk_i.pos());
        reset_signal_is(wb_rst_i,true);

        SC_CTHREAD(proc_clk_gpio_in,wb_clk_i.pos());
        reset_signal_is(wb_rst_i,true);
    }

private:

    // I/O Register ...
    sc_signal<sc_uint<32> > ioregs_regs;
    sc_signal<sc_uint<CFG_GPIO_INAMOUNT> > gpio_in_sync;

};


#endif // _GPIO_WB_SLAVE_H
