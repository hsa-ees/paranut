/*************************************************************************

  This file is part of the ParaNut project.

  Copyright (C) 2010-2022 Alexander Bahle <alexander.bahle@hs-augsburg.de>
                          Gundolf Kiefer <gundolf.kiefer@hs-augsburg.de>
                          Christian H. Meyer <christian.meyer@hs-augsburg.de>
      Hochschule Augsburg, University of Applied Sciences

  Description:
    FELIX: TODO

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
#ifndef _MEMU_READPORT_
#define _MEMU_READPORT_

#include "memu_common.h"
#include "memu_busif.h"
// **************** MReadPort *******************

typedef enum {
    s_rp_init = 0,
    s_rp_direct_wait_busif = 1,
    s_rp_request_tag_only = 2,
    s_rp_read_tag = 3,
    s_rp_read_bank = 4,
    s_rp_miss_wait_busif = 5,
    s_rp_miss_request_tag = 6,
    s_rp_miss_read_tag = 7,
    s_rp_miss_replace = 8
} EReadportState;


class MReadPort : ::sc_core::sc_module {
public:
    // Ports ...
    sc_in<bool> clk{"clk"};
    sc_in<bool> reset{"reset"};

    //   With (CPU) port...
    //   - All input ports must be held until 'port_ack' is asserted, at least for one cycle.
    //   - 'port_ack' is issued for exactly one cycle (no full handshake).
    //   - 'port_ack' is issued one cycle before the data is valid (pipelining).
    //   - The next request may be issued at earliest one cycle after 'port_ack' was asserted (which is the same cycle the data is delivered).
    //   - ('port_ack' may be asserted already in the same cycle as the request was issued.) -> minimum latency on cache hit is presently 2 clocks
    //   - If 'port_direct' is set, nothing happens with the cache (i.e. no invalidation).
    //     Hence, 'port_direct' = 0/1 should not be mixed for one adress.
    sc_in<bool> port_rd{"port_rd"}; 
    sc_in<bool> port_direct{"port_direct"};
    sc_out<bool> port_ack{"port_ack"};
    sc_in<sc_uint<32> > port_adr{"port_adr"};
    sc_out<sc_uint<32> > port_data{"port_data"};
    sc_out<bool> port_ac_r{"port_ac_r"};
    sc_out<bool> port_ac_x{"port_ac_x"};
    sc_out<bool> port_ac_u{"port_ac_u"};

    sc_in<bool> port_lres_scond{"port_lres_scond"};    // for LR/SC: reserve current adress
    sc_out<bool> port_scond_ok{"port_scond_ok"};     // for LR/SC: set, if store-conditional would be sucessful

    //   With BUSIF...
    sc_in<sc_uint<32> > busif_adr{"busif_adr"};     // adress BUSIF is currently working on (to check for BUSIF hits)
    sc_in<sc_uint<32> > busif_data{"busif_data"};    // must be routed here from 'busif.data_out[]' according to 'port_adr'
    sc_vector<sc_in<bool> > busif_data_valid{"busif_data_valid", CFG_MEMU_CACHE_BANKS};
    sc_out<EBusIfOperation> busif_op{"busif_op"}; // This can be replaced by one bit (only 'bioReplace' is invoked)
    sc_in<bool> busif_busy{"busif_busy"};
    sc_in<bool> busif_ac_r{"busif_ac_r"};
    sc_in<bool> busif_ac_x{"busif_ac_x"};
    sc_in<bool> busif_ac_u{"busif_ac_u"};

    //   With cache...
    //   - Adress information for tag is routed around this module from 'port_adr', for bank from 'port_adr' AND 'way_out'.
    sc_out<bool> tag_rd{"tag_rd"};
    sc_out<bool> bank_rd{"bank_rd"};   // 'bank_rd' and 'cache_data' must be routed according to 'port_adr'
    sc_in<sc_uint<32> > bank_data_in{"bank_data_in"};      // must be routed according to 'bank_sel'
    sc_out<sc_uint<32> > bank_sel{"bank_sel"};
    sc_in<SCacheTag> tag_in{"tag_in"};
    sc_out<sc_uint<32> > way_out{"way_out"};

    //   Request & grant lines...
    sc_out<bool> req_tagr{"req_tagr"};
    sc_vector<sc_out<bool> >req_bank{"req_bank", CFG_MEMU_CACHE_BANKS};
    sc_out<bool> req_busif{"req_busif"};
    sc_in<bool> gnt_tagr{"gnt_tagr"};
    sc_vector<sc_in<bool> >gnt_bank{"gnt_bank", CFG_MEMU_CACHE_BANKS};
    sc_in<bool> gnt_busif{"gnt_busif"};

    //   With snoop unit (arbiter)...
    sc_in<sc_uint<32> > snoop_adr{"snoop_adr"};
    sc_in<bool> snoop_stb{"snoop_stb"};

    // Constructor...
    SC_HAS_PROCESS (MReadPort);
    MReadPort (sc_module_name name = "MReadPort")
        : sc_module (name) {
        SC_METHOD (proc_cmb_hit);
            sensitive << port_adr << busif_adr;
            for (uint b = 0; b < CFG_MEMU_CACHE_BANKS; b++)
                sensitive << busif_data_valid[b];
        SC_CTHREAD (proc_clk_readport, clk.pos ());
        SC_METHOD (proc_cmb_readport);
            sensitive << reset << state_reg << bank_sel_reg << busif_hit;
            sensitive << port_rd << port_direct << port_adr << port_lres_scond;
            sensitive << busif_ac_r << busif_ac_x << busif_ac_u;
            sensitive << ac_r_reg << ac_x_reg << ac_u_reg;
            sensitive << busif_data << busif_busy;
            sensitive << bank_data_in << tag_in;
            sensitive << gnt_tagr << gnt_busif;
            sensitive << snoop_adr << snoop_stb;
            sensitive << link_valid_reg << link_adr_reg << next_link_adr_reg;
            sensitive << req_tagr << tag_rd;
            for (uint b = 0; b < CFG_MEMU_CACHE_BANKS; b++)
                sensitive << gnt_bank[b];
    }

    // Functions...
    void Trace (sc_trace_file * tf, int level = 1);

    // Processes...
    void proc_cmb_hit ();
    void proc_clk_readport ();
    void proc_cmb_readport ();

protected:
    // Registers...
    sc_signal<EReadportState> state_reg;
    sc_signal<int> state_trace;

    sc_signal<sc_uint<32> > bank_sel_reg;

    sc_signal<sc_uint<32> > link_adr_reg;
    sc_signal<bool> link_valid_reg;
    sc_signal<bool> ac_r_reg;
    sc_signal<bool> ac_x_reg;
    sc_signal<bool> ac_u_reg;


    // Internal signals...
    sc_signal<bool> busif_hit;
    sc_signal<EReadportState> next_state;
    sc_signal<sc_uint<32> > next_bank_sel;
    sc_signal<sc_uint<32> > next_link_adr_reg;
    sc_signal<bool> next_link_valid_reg;
    sc_signal<bool> next_ac_r_reg;
    sc_signal<bool> next_ac_x_reg;
    sc_signal<bool> next_ac_u_reg;
};
#endif