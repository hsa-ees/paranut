/*************************************************************************

  This file is part of the ParaNut project.

  Copyright (C) 2010-2019 Alexander Bahle <alexander.bahle@hs-augsburg.de>
                          Gundolf Kiefer <gundolf.kiefer@hs-augsburg.de>
      Hochschule Augsburg, University of Applied Sciences

  Description:
    This module simulates the system bus together with the memory and
    peripherals for the SystemC testbench of the ParaNut.

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


#ifndef _PERIPHERALS_
#define _PERIPHERALS_

#include "memory.h"

#include <systemc.h>


SC_MODULE (MPeripherals) {
    SC_HAS_PROCESS (MPeripherals);

    // Ports (WISHBONE slave)...
    sc_in_clk           clk_i; // clock input
    sc_in<bool>         rst_i; // reset

    sc_in<bool>         stb_i; // strobe input
    sc_in<bool>         cyc_i; // cycle valid input
    sc_in<bool>         we_i;  // indicates write transfer
    sc_in<sc_uint<CFG_MEMU_BUSIF_WIDTH/8> >  sel_i; // byte select inputs
    sc_out<bool>        ack_o; // normal termination
    sc_out<bool>        err_o; // termination w/ error
    sc_out<bool>        rty_o; // termination w/ retry

    sc_in<TWord>        adr_i; // address bus inputs
    sc_in<sc_uint<CFG_MEMU_BUSIF_WIDTH> >       dat_i; // input data bus
    sc_out<sc_uint<CFG_MEMU_BUSIF_WIDTH> >       dat_o; // output data bus


    // Constructor...
    MPeripherals (sc_module_name name, CMemory * memory)
    : sc_module (name) {
        SC_CTHREAD (MainThread, clk_i.pos ());
            reset_signal_is (rst_i, true);

        this->memory = memory;
    }

    // SC methods...
    void MainThread (void);

    // Other methods...

    // Fields...
    CMemory *memory;
};


#endif
