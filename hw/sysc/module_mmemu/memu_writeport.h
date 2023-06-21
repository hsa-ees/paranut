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
#ifndef _MEMU_WRITEPORT_
#define _MEMU_WRITEPORT_

#include "memu_common.h"
#include "memu_busif.h"
// **************** MWritePort *******************

typedef enum {
    s_wp_init = 0,
    s_wp_direct,
    s_wp_request_linelock_only,
    s_wp_read_tag,
    s_wp_write_tag1_and_bank,
    s_wp_write_tag1,
    s_wp_write_bank,
    s_wp_miss,
    s_wp_request_busif_only,
    s_wp_recheck,
    s_wp_recheck_read_tag,
    s_wp_replace,
    s_wp_replace_wait_busif,
    s_wp_special_request_busif_only,
    s_wp_special,
    s_wp_special_wait_complete,
    s_wp_page_fault
} EWritePortState;


class MWritePort : ::sc_core::sc_module {
public:
    // Ports ...
    sc_in<bool> clk, reset;

    //   With (CPU) port...
    //   - All input ports must be held until 'port_ack' is asserted, at least one cycle.
    //   - 'port_ack' is issued for exactly one cycle (no full handshake).
    //   - 'port_ack' may be asserted already in the same cycle as the request was issued.
    //   - The next request may be issued at earliest one cycle after 'port_ack' was asserted. (TBD: allow in the ACK cycle?)
    //   - If 'port_direct' is set, nothing happens with the cache (i.e. no invalidation).
    //     Hence, 'port_direct' = 0/1 should not be mixed for one adress.
    sc_in<bool> port_wr, port_direct;
    sc_in<sc_uint<4> > port_bsel;
    sc_out<bool> port_ack;
    sc_out<bool> port_ac_w;
    sc_in<bool> port_trap_u;
    sc_in<bool> port_trap_no_u;

    sc_in<sc_uint<3> > port_cache_op;            // Cache control; | all | writeback | invalidate |

    sc_in<sc_uint<32> > port_adr;
    sc_in<sc_uint<32> > port_data;

    sc_in<bool> port_lres_scond;    // for LR/SC: store only if lres_valid
    sc_in<bool> port_scond_ok;

    //   With BUSIF...
    //   - There is no direct transfer except for uncached memory access.
    sc_in<sc_uint<32> > busif_adr; // adress BUSIF is currently working on (to check for BUSIF hits)
    sc_out<EBusIfOperation> busif_op;
    sc_out<bool> busif_nolinelock; // if set, no line lock is acquired (for write misses: lock is already held by write port) (TBD: eliminate this signal)
    sc_in<bool> busif_busy;
    sc_in<bool> busif_ac_w;

    //   With cache...
    //   - Adress information for tag must be routed from 'port_adr', for the banks from 'port_adr' and 'tag_out.way'.
    sc_out<bool> tag_rd, tag_wr, bank_rd, bank_wr;  // 'bank_rd', 'bank_wr' must be routed according to 'port_adr'
    sc_in<sc_uint<32> > bank_data_in;                      // must be routed according to 'port_adr'
    sc_out<sc_uint<32> > bank_data_out;                    // must be routed according to 'port_adr'
    sc_out<sc_uint<4> > bank_bsel;
    sc_in<SCacheTag> tag_in;
    sc_out<SCacheTag> tag_out;

    //   Request & grant lines...
    sc_out<bool> req_linelock, req_tagr, req_tagw, req_bank[CFG_MEMU_CACHE_BANKS], req_busif;
    sc_in<bool> gnt_linelock, gnt_tagr, gnt_tagw, gnt_bank[CFG_MEMU_CACHE_BANKS], gnt_busif;

    // Constructor...
    SC_HAS_PROCESS (MWritePort);
    MWritePort (sc_module_name name = "MWritePort")
        : sc_module (name) {
        SC_CTHREAD (proc_clk_writeport, clk.pos ());
        SC_METHOD (proc_cmb_writeport);
            sensitive << reset << state_reg;
            sensitive << port_wr << port_direct << port_bsel << port_lres_scond << port_cache_op
                      << port_adr << port_data << port_lres_scond << port_scond_ok;
            sensitive << busif_adr << busif_busy << bank_data_in << busif_ac_w << tag_in;
            sensitive << gnt_linelock << gnt_tagr << gnt_tagw << gnt_busif;
            sensitive << port_trap_u << port_trap_no_u;
            sensitive << tag_reg << data_reg;
            for (uint b = 0; b < CFG_MEMU_CACHE_BANKS; b++)
                sensitive << gnt_bank[b];
    }

    // Functions...
    void Trace (sc_trace_file * tf, int level = 1);

    // Processes...
    void proc_clk_writeport ();
    void proc_cmb_writeport ();

protected:
    // Registers...
    sc_signal<EWritePortState> state_reg;
    sc_signal<int> state_trace;

    sc_signal<SCacheTag> tag_reg;
    sc_signal<sc_uint<32> > data_reg;
     
    // Internal signals..
    sc_signal<EWritePortState> next_state;
    sc_signal<SCacheTag> next_tag_reg;
    sc_signal<sc_uint<32> > next_data_reg;
};
#endif