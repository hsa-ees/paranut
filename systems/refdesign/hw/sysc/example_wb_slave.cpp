/**************************************************************************
 *
 *  This file is part of the ParaNut project.
 *
 *  Copyright (C) 2020-2021 Gundolf Kiefer <gundolf.kiefer@hs-augsburg.de>
 *                          Michael Schaeferling <michael.schaeferling@hs-augsburg.de>
 *                          Alexander Bahle <alexander.bahle@hs-augsburg.de>
  *                         Felix Wagner <felix.wagner1@hs-augsburg.de>
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


#include "example_wb_slave.h"


#define PN_TRACE_VERBOSE 0     // EDIT HERE: Change to 1 to add extra console output





// *************************** Helpers *****************************************


// Compiled outside the ParaNut environment: Define some helper macros locally ...
#ifndef _PARANUT_PERIPHERAL_


#ifndef __SYNTHESIS__


static char *GetTraceName (sc_object *obj, const char *name, int dim, int arg1, int arg2) {
    static char buf[200];
    char *p;

    strcpy (buf, obj->name ());
    p = strrchr (buf, '.');
    p = p ? p + 1 : buf;
    sprintf (p, dim == 0 ? "%s" : dim == 1 ? "%s(%i)" : "%s(%i)(%i)", name, arg1, arg2);
    // printf ("### %s, %i, %i, %i -> %s\n", ((sc_object *) obj)->name (), dim, arg1, arg2, buf);
    return buf;
}


#define PN_TRACE(TF, OBJ) {                                                     \
  if (TF) sc_trace (TF, OBJ, GetTraceName (&(OBJ), #OBJ, 0, 0, 0));             \
  if (!TF || PN_TRACE_VERBOSE) cout << "  " #OBJ " = '" << (OBJ).name () << "'\n"; \
}


#define PN_TRACE_BUS(TF, OBJ, N_MAX) {                                          \
  for (int n = 0; n < N_MAX; n++) {                                             \
    if (TF) sc_trace (TF, (OBJ)[n], GetTraceName (&(OBJ)[n], #OBJ, 1, n, 0));   \
    if (!TF || PN_TRACE_VERBOSE)                                                \
      cout << "  " #OBJ "[" << n << "] = '" << (OBJ)[n].name () << "'\n";       \
  }                                                                             \
}


#endif // __SYNTHESIS__





// Compiled with the ParaNut environment ...
#else // _PARANUT_PERIPHERAL_

#undef PN_TRACE_VERBOSE
#define PN_TRACE_VERBOSE pn_trace_verbose   // map PN_TRACE_VERBOSE to run-time switch

#endif // _PARANUT_PERIPHERAL_





// ***************** Signal Tracing (Simulation) *******************************


#ifndef __SYNTHESIS__


void example_wb_slave::trace (sc_trace_file *tf, int level) {
  if (!tf || PN_TRACE_VERBOSE) printf ("\nSignals of Module \"%s\":\n", name ());
  if (!tf) return;

  // WishBone Slave Ports (usually not to change) ...
  if (level >= 1) {
    PN_TRACE (tf, wb_clk_i);
    PN_TRACE (tf, wb_rst_i);

    PN_TRACE (tf, wb_cyc_i);
    PN_TRACE (tf, wb_stb_i);
    PN_TRACE (tf, wb_we_i);
    PN_TRACE (tf, wb_cti_i);
    PN_TRACE (tf, wb_bte_i);
    PN_TRACE (tf, wb_sel_i);
    PN_TRACE (tf, wb_adr_i);
    PN_TRACE (tf, wb_dat_i);
    PN_TRACE (tf, wb_dat_o);

    PN_TRACE (tf, wb_ack_o);
    PN_TRACE (tf, wb_rty_o);
    PN_TRACE (tf, wb_err_o);
  }

  // User-Specific Ports and Registers ...
  // EDIT HERE...
}


#endif //  __SYNTHESIS__





// *************************** I/O Registers ***********************************


/* The following method(s) implement a generic WishBone slave interface.
 *
 * Usually, it is not necessary to make any changes here.
 * The number of slave registers can be adjusted by setting 'ioregs_num_ld'.
 */


// Clocked main method (every signal/output assigned here will be a register) ...
void example_wb_slave::proc_ioregs_transition () {
    // Vivado HLS pragmas (see UG902 Chapter 4)...
    #pragma HLS ARRAY_PARTITION variable=ioregs_regs complete dim=1    // Make sure "ioregs_regs" will be registers not BR

    // Local variables...
    sc_uint<ioregs_num_ld+2> reg_idx;
    sc_uint<32> wb_adr_i_var, wb_dat_i_var, wb_dat_o_var;
    sc_uint<32> regs_var[ioregs_num], sel_reg;
    sc_uint<4> wb_sel_i_var;

    // Read inputs...
#if CFG_MEMU_BUSIF_WIDTH == 64
    // This implementation assumes that either top or bottom 32 bits are read/written, never both!
    bool top_word = wb_sel_i.read()(7, 4).or_reduce();
    wb_adr_i_var = top_word ? wb_adr_i.read() + 4 : wb_adr_i.read();
    wb_dat_i_var = top_word ? wb_dat_i.read().range(63, 32) : wb_dat_i.read().range(31, 0);
    wb_sel_i_var = top_word ? wb_sel_i.read ()(7, 4) : wb_sel_i.read ()(3, 0);
#else
    wb_adr_i_var = wb_adr_i.read ();
    wb_dat_i_var = wb_dat_i.read ();
    wb_sel_i_var = wb_sel_i.read ();
#endif

    // Read registers...
    for (int n = 0; n < ioregs_num; ++n)
        #pragma HLS LOOP_FLATTEN
        regs_var[n] = ioregs_regs[n].read ();

    // Determine index of addressed register and select it...
    // (Single point of read access into the array -> only one Multiplexor will be generated)
    reg_idx = wb_adr_i_var (2 + ioregs_num_ld, 2);
    sel_reg = regs_var[reg_idx];

    // Write default to outputs...
    wb_ack_o = 0;
    wb_err_o = 0;
    wb_rty_o = 0;
    wb_dat_o_var = 0;

    // Synchronous reset...
    if (wb_rst_i) {
        for (int n = 0; n < ioregs_num; ++n) regs_var[n] = 0;

    } else {

        // Handle wishbone read/write...
        if (wb_stb_i == 1 && wb_cyc_i == 1) {

            // Write access ...
            if (wb_we_i) {
                // Use sel_i for byte/half-word/word writes...
                // (doesn't look super clean, but works for HLS)
                for (int n = 0; n < 4; n++)
                    #pragma HLS LOOP_FLATTEN
                    if (wb_sel_i_var[n] == 1)
                        sel_reg = (sel_reg & ~(0x000000ff << (8 * n))) |
                                  (wb_dat_i_var & (0x000000ff << (8 * n)));
                regs_var[reg_idx] = sel_reg;

            // Read access ...
            } else {
               wb_dat_o_var = sel_reg;
            }
            wb_ack_o = 1;
        }

        // Handle user logic...
        for (int n = 0; n < ioregs_num; ++n)
            #pragma HLS LOOP_FLATTEN
            regs_var[n] = callback_ioregs_writeback (n, regs_var[n]);
    }

    // Write back registers...
    for (int n = 0; n < ioregs_num; ++n)
        #pragma HLS LOOP_FLATTEN
        ioregs_regs[n] = regs_var[n];

    // Write derived outputs...
#if WB_WIDTH == 64
    wb_dat_o = top_word ? (wb_dat_o_var, sc_uint<32>(0)) : (sc_uint<32>(0), wb_dat_o_var);
#else
    wb_dat_o = wb_dat_o_var;
#endif
}





// *************************** User-Specific Logic *****************************

// EDIT HERE
