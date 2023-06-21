/*************************************************************************

  This file is part of the ParaNut project.

  Copyright (C) 2021-2022 Christian H. Meyer <christian.meyer@hs-augsburg.de>
      Hochschule Augsburg, University of Applied Sciences

  Description:
    This is a SystemC model of the Translation Lookaside Buffer (TLB)
    of the ParaNut project. It interacts only with the MMU and buffers
    page table entries to speed up virtual address translation.

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.

 *************************************************************************/

#ifndef _TLB_
#define _TLB_

#include "base.h"
#include "paranut-config.h"

#include <systemc.h>

struct STlbTag {
    bool valid;
    bool superpage;

    sc_uint<10> vpn1;
    sc_uint<10> vpn0;

    STlbTag operator= (const STlbTag &t) {
        valid = t.valid;
        superpage = t.superpage;
        vpn1 = t.vpn1;
        vpn0 = t.vpn0;
        return *this;
    }

    // Necessary operators for using this structure as signals...
    bool operator== (const STlbTag &t) const {
        return valid == t.valid && superpage == t.superpage && vpn1 == t.vpn1 && vpn0 == t.vpn0;
    }

    //Overload function
    friend void sc_trace( sc_trace_file* f, const STlbTag& t, const std::string& _s ) {
        sc_trace( f, t.valid, _s + ".valid" );
        sc_trace( f, t.superpage, _s + ".superpage" );
        sc_trace( f, t.vpn1, _s + ".vpn1" );
        sc_trace( f, t.vpn0, _s + ".vpn0" );
    }

    // Displaying
    friend ostream& operator << ( ostream& os, const STlbTag &t ) {
        os << "valid:" << t.valid << " superpage:" << t.superpage <<  " vpn1:" << t.vpn1 <<  " vpn0:" << t.vpn0;

        return os;
    }

};


struct STlbData {
    sc_uint<20> padr;
    bool ac_r;
    bool ac_w;
    bool ac_x;
    bool ac_u;
    bool ac_a;
    bool ac_d;

    // Necessary operators for using this structure as signals...
    bool operator== (const STlbData &t) const {
        return padr == t.padr && ac_r == t.ac_r && ac_w == t.ac_w && ac_x == t.ac_x && ac_u == t.ac_u && ac_a == t.ac_a && ac_d == t.ac_d;
    }

    friend ostream &operator<< (ostream &os, const STlbData &t);

    STlbData operator= (const sc_uint<26> &t) {
        padr = t.range(19, 0);
        ac_r = t[20];
        ac_w = t[21];
        ac_x = t[22];
        ac_u = t[23];
        ac_a = t[24];
        ac_d = t[25];
        return *this;
    }

    operator sc_uint<26>() {
        sc_uint<26> out = padr;
        out[20] = ac_r;
        out[21] = ac_w;
        out[22] = ac_x;
        out[23] = ac_u;
        out[24] = ac_a;
        out[25] = ac_d;
        return out;
    }

    STlbData operator= (const STlbData &t) {
        padr = t.padr;
        ac_r = t.ac_r;
        ac_w = t.ac_w;
        ac_x = t.ac_x;
        ac_u = t.ac_u;
        ac_a = t.ac_a;
        ac_d = t.ac_d;
        return *this;
    }

    // Displaying
    friend ostream& operator << ( ostream& os, const STlbData &t ) {
        os << " padr:" << t.padr <<  " ac_r:" << t.ac_r <<  " ac_w:" << t.ac_w <<  " ac_x:" << t.ac_x <<  " ac_u:" << t.ac_u << " ac_a:" << t.ac_a << " ac_d:" << t.ac_d;

        return os;
    }
    //Overload function
    
    friend void sc_trace( sc_trace_file* f, const STlbData& t, const std::string& _s ) {
        sc_trace( f, t.padr, _s + ".padr" );
        sc_trace( f, t.ac_r, _s + ".ac_r" );
        sc_trace( f, t.ac_w, _s + ".ac_w" );
        sc_trace( f, t.ac_x, _s + ".ac_x" );
        sc_trace( f, t.ac_u, _s + ".ac_u" );
        sc_trace( f, t.ac_a, _s + ".ac_a" );
        sc_trace( f, t.ac_d, _s + ".ac_d" );
    }
};

class MTlb : ::sc_core::sc_module {
public:
    // Ports ...
    sc_in<bool> clk, reset;

    // from 
    sc_in<bool> flush;

    // from/to PTW
    sc_in<bool> ptw_req, ptw_wr;
    sc_in<sc_uint<20> > ptw_va_i;
    sc_in<sc_uint<20> > ptw_pa_i;
    sc_in<bool> ptw_superpage_i;
    sc_in<bool> ptw_ac_r_i;
    sc_in<bool> ptw_ac_w_i;
    sc_in<bool> ptw_ac_x_i;
    sc_in<bool> ptw_ac_u_i;
    sc_in<bool> ptw_ac_a_i;
    sc_in<bool> ptw_ac_d_i;

    sc_out<bool> ptw_superpage_o;
    sc_out<sc_uint<20> > ptw_adr_o;
    sc_out<bool> ptw_hit_o;
    sc_out<bool> ptw_miss_o;
    
    sc_out<bool> ptw_ac_r_o;
    sc_out<bool> ptw_ac_w_o;
    sc_out<bool> ptw_ac_x_o;
    sc_out<bool> ptw_ac_u_o;
    sc_out<bool> ptw_ac_a_o;
    sc_out<bool> ptw_ac_d_o;

    // Constructor...
    SC_HAS_PROCESS (MTlb);
    MTlb (sc_module_name name)
        : sc_module (name) {
        SC_CTHREAD (TransitionMethod, clk.pos ());
        }

    // Functions...
    void Trace (sc_trace_file * tf, int level = 1);
    // Processes...
    void TransitionMethod ();

protected:
    // Registers...
    sc_signal<STlbTag> tlb_tag[CFG_MMU_TLB_ENTRIES];
    sc_signal<STlbData> tlb_data[CFG_MMU_TLB_ENTRIES];

    // regs
    sc_signal<sc_uint<CFG_MMU_TLB_ENTRIES-1> > plru_reg;
    sc_signal<sc_uint<CFG_MMU_TLB_ENTRIES_LD> > latest_hit;
    sc_signal<bool> update_plru_reg;
    sc_signal<bool> wr_done_reg;

    // internal signals
    sc_signal<bool> superpage_sig;
    sc_signal<bool> hit_sig;
    sc_signal<bool> miss_sig;
};


#endif