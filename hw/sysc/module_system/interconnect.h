/*************************************************************************

  This file is part of the ParaNut project.

  Copyright (C) 2020 Alexander Bahle <alexander.bahle@hs-augsburg.de>
      Hochschule Augsburg, University of Applied Sciences

  Description:
    This is a SystemC model of a Wishbone interconnect exclusively for
    simulation purpose.
    Features 1 master input and CFG_NUT_SIM_MAX_PERIPHERY slave outputs (see config)

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


#ifndef _INTERCONNECT_H
#define _INTERCONNECT_H


#include "paranut-peripheral.h"

struct  SInterPeriph {
    TWord adr;
    size_t size;
};

// **************** MInterconnect *************
class MInterconnect : ::sc_core::sc_module {
public:
    // Ports (WISHBONE slave)...
    sc_in_clk           clk_i; // clock input
    sc_in<bool>         rst_i; // reset

    sc_in<bool>         stb_i; // strobe input
    sc_in<bool>         cyc_i; // cycle valid input
    sc_in<bool>         we_i;  // indicates write transfer
    sc_out<sc_uint<3> > cti_i; // cycle type identifier
    sc_out<sc_uint<2> > bte_i; // burst type extension
    sc_in<sc_uint<CFG_MEMU_BUSIF_WIDTH/8> >  sel_i; // byte select inputs
    sc_out<bool>        ack_o; // normal termination
    sc_out<bool>        err_o; // termination w/ error
    sc_out<bool>        rty_o; // termination w/ retry

    sc_in<sc_uint<32> > adr_i; // address bus inputs
    sc_in<sc_uint<CFG_MEMU_BUSIF_WIDTH> >    dat_i; // input data bus
    sc_out<sc_uint<CFG_MEMU_BUSIF_WIDTH> >   dat_o; // output data bus

    // Constructor...
    SC_CTOR (MInterconnect){
        SC_METHOD (InterconnectMethod);
            sensitive << stb_i << cyc_i << we_i << sel_i << cti_i << bte_i;
            sensitive << adr_i << dat_i;
            for (int n = 0; n < CFG_NUT_SIM_MAX_PERIPHERY; n++)
                sensitive << ack[n] << rty[n] << err[n] << dat[n];

        num_peri_ = 0;
    }

    // Functions...
    void Trace (sc_trace_file * tf, int level = 1);
    void AddSlave(TWord start_adr, size_t size, MPeripheral *slave);
    void AddSlave(TWord start_adr, size_t size, 
        sc_in_clk *wb_clk_i, sc_in<bool> *wb_rst_i, sc_in<bool> *wb_stb_i, 
        sc_in<bool> *wb_cyc_i, sc_in<bool> *wb_we_i, sc_in<sc_uint<3> > *wb_cti_i, 
        sc_in<sc_uint<2> > *wb_bte_i, sc_in<sc_uint<WB_PORT_SIZE/8> > *wb_sel_i, 
        sc_out<bool> *wb_ack_o, sc_out<bool> *wb_err_o, sc_out<bool> *wb_rty_o, 
        sc_in<sc_uint<32> > *wb_adr_i, sc_in<sc_uint<WB_PORT_SIZE> > *wb_dat_i, 
        sc_out<sc_uint<WB_PORT_SIZE> > *wb_dat_o);

    // Processes...
    void InterconnectMethod ();

private:
    int num_peri_;
    SInterPeriph peripherals_[CFG_NUT_SIM_MAX_PERIPHERY];

    // Internal signals...
    sc_signal<bool> stb[CFG_NUT_SIM_MAX_PERIPHERY],
                    ack[CFG_NUT_SIM_MAX_PERIPHERY],
                    rty[CFG_NUT_SIM_MAX_PERIPHERY],
                    err[CFG_NUT_SIM_MAX_PERIPHERY];
    sc_signal<sc_uint<CFG_MEMU_BUSIF_WIDTH> > dat[CFG_NUT_SIM_MAX_PERIPHERY];


};

#endif // _INTERCONNECT_H
