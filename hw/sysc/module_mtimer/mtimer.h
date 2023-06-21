 /*************************************************************************

  This file is part of the ParaNut project.

  Copyright (C) 2010-2022 Mark Endres <mark.endres@hs-augsburg.de>
                          Christian H. Meyer <christian.meyer@hs-augsburg.de>
      Hochschule Augsburg, University of Applied Sciences

  Description:
    This is a SystemC model of the mtimer module of the ParaNut.
  
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

#pragma once

#include "base.h"
#include "paranut-config.h"

/*#define TIMEREG_LOW (reg_mtime.read () (31, 0))
#define TIMEREG_HIGH (reg_mtime.read () (63, 32))
#define TIMECMPREG_LOW (reg_mtimecmp.read () (31, 0))
#define TIMECMPREG_HIGH (reg_mtimecmp.read () (63, 32))*/
#define MTIMER_PRESCALER ((__uint64_t)((float)CFG_NUT_SIM_CLK_SPEED / (1.0f / ((float)CFG_NUT_MTIMER_TIMEBASE_US / (1000*1000)))))

typedef enum {   
    mtime = 0,      // 64 bit memory mapped machine timer register
    mtimeh = 4,
    mtimecmp = 8,   // 64 bit memory mapped machine timer compare register
    mtimecmph = 12
} ETIMER;

class Mtimer : ::sc_core::sc_module {
public:
    // ports
    sc_in<bool> irq_out_enable;
    sc_out<bool> irq_out;
    
    sc_in_clk           wb_clk_i;      ///< WB Clock input.
    sc_in<bool>         wb_rst_i;      ///< WB Reset input.

    sc_in<bool>         wb_stb_i;      ///< WB Strobe input
    sc_in<bool>         wb_we_i;       ///< WB write enable intput.
    //sc_in<sc_uint<CFG_MEMU_BUSIF_WIDTH/8> > wb_sel_i; ///< WB byte select inputs.
    sc_out<bool>        wb_ack_o;      ///< WB normal termination.

    sc_in<sc_uint<32> >             wb_adr_i;  ///< WB address bus inputs.
    sc_in<sc_uint<CFG_MEMU_BUSIF_WIDTH> >   wb_dat_i;  ///< WB input data bus.
    sc_out<sc_uint<CFG_MEMU_BUSIF_WIDTH> >  wb_dat_o;  ///< WB output data bus.

    // processes
    void TransitionMethod();  
    
    //SC_CTOR (Mtimer) {
    //typedef Mtimer SC_CURRENT_USER_MODULE;    // damit geht make
    SC_HAS_PROCESS(Mtimer);
    Mtimer(const sc_module_name& name) : sc_module (name) {
        SC_METHOD (TransitionMethod);
            sensitive << wb_clk_i.pos (); 
    }

    // Functions ...
    void Trace (sc_trace_file * tf, int levels = 1);
private:
    // memory mapped registers
    sc_signal<sc_uint<64> > reg_mtime;      
    sc_signal<sc_uint<64> > reg_mtimecmp;   
    // internal registers
    sc_signal<sc_uint<32> > reg_mtime_prescale_cnt;
};
