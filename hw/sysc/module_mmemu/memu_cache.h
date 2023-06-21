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
#ifndef _MEMU_CACHE_
#define _MEMU_CACHE_
#include "memu_common.h"



// **************** MTagRam ******************

#define TR_PORTS CFG_NUT_CPU_CORES // see comment to 'MArbiter'

// Tag layout in memory
// | Valid bit | Dirty bit | Read (Paging) | Write (Paging) | Execute (Paging) | User (Paging) | Accessed (Paging) | Dirty (Paging) | Tag address |
struct STag {
    bool valid, dirty;
    bool ac_r, ac_w, ac_x, ac_u; // Paging flags
    sc_uint<32-CFG_MEMU_CACHE_SETS_LD-CFG_MEMU_CACHE_BANKS_LD-2> tadr;
};

// Tag entry layout in memory
// | Tags (way(0)..way(n-1)) | LRU info |
struct STagEntry {

    bool tag_valid[CFG_MEMU_CACHE_WAYS];
    bool tag_dirty[CFG_MEMU_CACHE_WAYS];
    bool tag_ac_r[CFG_MEMU_CACHE_WAYS];
    bool tag_ac_w[CFG_MEMU_CACHE_WAYS];
    bool tag_ac_x[CFG_MEMU_CACHE_WAYS];
    bool tag_ac_u[CFG_MEMU_CACHE_WAYS]; // Paging flags
    sc_uint<32-CFG_MEMU_CACHE_SETS_LD-CFG_MEMU_CACHE_BANKS_LD-2> tag_tadr[CFG_MEMU_CACHE_WAYS];

    sc_uint<6> use; // only used with LRU replacement (Note: saved in its own block ram in hardware)
};

/*
  // Necessary operators for using this structure as signals...
  bool operator== (const STagEntry& t) const;
  STagEntry& operator= (const STagEntry& t);
};

ostream& operator<< (ostream& os, const STagEntry& t);
void sc_trace (sc_trace_file *tf, const STagEntry& t, const std::string& name);
*/


class MTagRam : ::sc_core::sc_module {
public:
    // Ports ...
    sc_in<bool> clk, reset;
    sc_out<bool> ready; // indicates that reset has been completed

    sc_in<bool> rd[TR_PORTS],
                wr[TR_PORTS],
                rd_way;

    sc_in<sc_uint<32> > adr[TR_PORTS]; // complete adress (lower CFG_MEMU_CACHE_BANKS_LD + 2 bits can be omitted)
    sc_in<sc_uint<32> > wadr[TR_PORTS]; // complete write adress (lower CFG_MEMU_CACHE_BANKS_LD + 2 bits can be omitted)
    sc_vector<sc_in<SCacheTag> > tag_in{"tag_in", TR_PORTS};
    // TBD: only the BusIf needs this port, write ports only set the dirty bit -> a 'wr_dirty' signal is sufficient
    sc_vector<sc_out<SCacheTag> > tag_out{"tag_out", TR_PORTS};

    // Constructor...
    SC_HAS_PROCESS (MTagRam);
    MTagRam (sc_module_name name)
        : sc_module (name) {
        SC_CTHREAD (proc_clk_tagram, clk.pos ());
            reset_signal_is (reset, true);
    }

    // Functions...
    void Trace (sc_trace_file * tf, int level = 1);

    // Processes...
    void proc_clk_tagram ();

protected:
    STagEntry ram_[CFG_MEMU_CACHE_SETS];

    // Registers...
    sc_signal<bool> write_tag;
    sc_signal<sc_uint<CFG_MEMU_CACHE_WAYS_LD> > wtag_way;
    sc_signal<sc_uint<32> > wtag_iadr;
    sc_signal<sc_uint<32> > wtag_port;
    //   for LRU replacement...
    sc_signal<sc_uint<6>> use_reg[TR_PORTS+1], use_iadr_reg[TR_PORTS+1]; // One for each reader + one for all writer
    sc_signal<bool> use_wr_reg[TR_PORTS+1];                         // One for each reader + one for all writer
    //   for random replacement...
    sc_signal<sc_uint<8> > counter;
};

// **************** MBankRam ********************

class MBankRam : ::sc_core::sc_module {
public:
    // Ports ...
    sc_in<bool> clk;

    sc_in<bool> rd[CFG_MEMU_BANK_RAM_PORTS], wr[CFG_MEMU_BANK_RAM_PORTS];

    sc_in<sc_uint<4> > wen[CFG_MEMU_BANK_RAM_PORTS]; // byte write enable bits

    sc_in<sc_uint<32> > wiadr[CFG_MEMU_BANK_RAM_PORTS]; // line adress (way & index bits of adress)
    sc_in<sc_uint<32> > wdata[CFG_MEMU_BANK_RAM_PORTS];
    sc_out<sc_uint<32> > rdata[CFG_MEMU_BANK_RAM_PORTS];

    // Constructor...
    SC_HAS_PROCESS (MBankRam);
    MBankRam (sc_module_name name = "BankRam")
        : sc_module (name) { 
            SC_CTHREAD (proc_clk_bankram, clk.pos ()); 
            }

    // Functions...
    void Trace (sc_trace_file * tf, int level = 1);

    // Processes...
    void proc_clk_bankram ();

protected:
    sc_uint<32>  ram_[CFG_MEMU_CACHE_SETS * CFG_MEMU_CACHE_WAYS];
};

#endif