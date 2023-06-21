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


#include "gpio_wb_slave.h"


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


void gpio_wb_slave::trace (sc_trace_file *tf, int level) {
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
    PN_TRACE (tf, gpio_in_sync);
    PN_TRACE (tf, ioregs_regs);
    PN_TRACE_BUS (tf, gpio_input_port, CFG_GPIO_INAMOUNT);
    PN_TRACE_BUS (tf, gpio_output_port, CFG_GPIO_OUTAMOUNT);    
  }
}


#endif //  __SYNTHESIS__





// *************************** I/O Registers ***********************************


/* The following method(s) implement a generic WishBone slave interface.
 *
 * Usually, it is not necessary to make any changes here.
 * The number of slave registers can be adjusted by setting 'ioregs_num_ld'.
 */


// Clocked main method (every signal/output assigned here will be a register) ...
void gpio_wb_slave::proc_ioregs_transition () {

    // Local variables...
    sc_uint<32> wb_adr_i_var;
    sc_uint<32> wb_dat_i_var;
    sc_uint<32> wb_dat_o_var;
    sc_uint<4> wb_sel_i_var;
    sc_uint<32> regs_var;
    ioregs_regs = 0;
    wait();
    while(true){
        bool isAddressed = (CFG_GPIO_BASE_ADDRESS & wb_adr_i.read()) && (wb_adr_i.read() <= CFG_GPIO_BASE_ADDRESS + 4);
        if(isAddressed){
            
            wb_dat_i_var = wb_dat_i.read ();

            // Read registers...
            regs_var = ioregs_regs.read ();

            // Write default to outputs...
            wb_ack_o = 0;
            wb_err_o = 0;
            wb_rty_o = 0;
            wb_dat_o_var = 0;

            // Handle wishbone read/write...
            if (wb_stb_i.read() == 1 && wb_cyc_i.read() == 1) {
                
                // Write access ...
                if (wb_we_i.read()) {
                    regs_var = wb_dat_i_var;
                // Read access ...
                } else {
                    wb_dat_o_var = regs_var;
                }
                wb_ack_o = 1;
            }

            // Handle user logic...
            sc_uint<CFG_GPIO_INAMOUNT> gpio_temp = gpio_in_sync.read();
            regs_var(CFG_GPIO_AMOUNT-1, CFG_GPIO_OUTAMOUNT) = gpio_temp((CFG_GPIO_INAMOUNT-1), 0);

            // Write back registers...
            ioregs_regs = regs_var;

            wb_dat_o = wb_dat_o_var;
        }
        wait();
    }
}





// *************************** User-Specific Logic *****************************

void gpio_wb_slave::proc_clk_gpio_out(){
    wait();
    while(true){
        sc_uint<32> gpio_temp;
        gpio_temp = ioregs_regs.read();
        for(size_t i = 0; i < CFG_GPIO_OUTAMOUNT; i++){
            gpio_output_port[i] = gpio_temp[i];
        }
        wait();
    }
}


void gpio_wb_slave::proc_clk_gpio_in(){
    gpio_in_sync = 0;
    wait();
    while(true){
        sc_uint<CFG_GPIO_INAMOUNT> gpio_temp;
        for(size_t i = 0; i < CFG_GPIO_INAMOUNT; i++){
            gpio_temp[i] = gpio_input_port[i];
        }
        gpio_in_sync = gpio_temp;
        wait();
    }
}