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

#ifndef _MEMU_COMMON_
#define _MEMU_COMMON_
#include "base.h"
#include "paranut-config.h"

// **************** Records *****************************

// MArbiter Selector input/output type...
template<int DWIDTH = 1, int SEL_MAX = 1>
struct SSelectorIO {
    sc_uint<DWIDTH> dat;
    sc_uint<NUM_BITS(SEL_MAX)> sel;
    bool sel_valid;

    SSelectorIO( sc_uint<DWIDTH> _dat = 0,  sc_uint<NUM_BITS(SEL_MAX)> _sel = 0, bool _sel_valid = 0): 
        dat(_dat), sel(_sel), sel_valid(_sel_valid) { }

    // Necessary operators for using this structure as signals...
    // Compare (necessary for signal updates)...
    bool operator == (const SSelectorIO &t) {
       return (dat == t.dat) && (sel == t.sel) && (sel_valid == t.sel_valid);
    }
    // Display...
    friend ostream& operator<< (ostream &o, const SSelectorIO &t ) {
       o << "{" << t.dat << "," << t.sel << "," << t.sel_valid << "}" ;
       return o;
    }
    // Tracing...
    friend void sc_trace( sc_trace_file *tf, const SSelectorIO &t, const std::string &name ) {
        PN_TRACE_R (tf, t, dat, name);
        PN_TRACE_R (tf, t, sel, name);
        PN_TRACE_R (tf, t, sel_valid, name);
    }
 };

template<int DWIDTH = 1, int SEL_MAX = 1>
struct MSelectorPass: sc_module {
public:
    // Ports ...
    sc_in<SSelectorIO<DWIDTH, SEL_MAX> > f_in;
    sc_out<SSelectorIO<DWIDTH, SEL_MAX> > out;

    void proc_cmb_fin() {
        out = f_in.read ();
    }

    SC_HAS_PROCESS(MSelectorPass); // Necessary when not using SC_MODULE macro
    MSelectorPass (const sc_module_name &name) : sc_module (name),
          f_in ("f_in"), out ("out")
    {
        SC_METHOD(proc_cmb_fin);
            sensitive << f_in ;
    }

    // Functions...
    void Trace (sc_trace_file *tf, int level) {
        PN_TRACE (tf, f_in);
        PN_TRACE (tf, out);
    }
};

template<int DWIDTH = 1, int SEL_MAX = 1>
struct MSelector: sc_module {
public:
    // Ports ...
    sc_in<bool> clk, reset;
    sc_in<SSelectorIO<DWIDTH, SEL_MAX> > f_in, s_in;
    sc_out<SSelectorIO<DWIDTH, SEL_MAX> > out;
    sc_in<sc_uint<MAX(1, CFG_NUT_CPU_CORES_LD)> > prio;

    void proc_clk_mselector() {
        while(1){
            if (reset == 1) {
                SSelectorIO<DWIDTH, SEL_MAX> reg; // Default constructor is (0, 0, 0)
                s_in_reg = reg;
            } else
                s_in_reg = next_s_in_reg.read ();
            wait();
        }
    }

    void proc_cmb_mselector() {
        SSelectorIO<DWIDTH, SEL_MAX> reg, f_in_var, s_in_var, out_var;
        bool slow_sel_valid;

        // Read inputs...
        f_in_var = f_in.read ();
        s_in_var = s_in.read ();

        // Set defaults...
        reg = s_in_reg.read ();
        slow_sel_valid = (reg.sel_valid && s_in_var.sel_valid) && (reg.sel == s_in_var.sel);

        if (prio.read () <= FAST_INDEX)
            // Fast input has priority...
            if (f_in_var.sel_valid)
                out_var = f_in_var;
            else
                out_var = reg;
         else
            // Slow input has priority...
            if (slow_sel_valid)
                out_var = reg;
            else
                out_var = f_in_var;

        // Set output valid...
        out_var.sel_valid = f_in_var.sel_valid || slow_sel_valid;

        // Write results...
        next_s_in_reg = s_in_var;
        out = out_var;
    }

    SC_HAS_PROCESS(MSelector); // Necessary when not using SC_MODULE macro
    MSelector (const sc_module_name &name, sc_uint<32> FAST_INDEX = 0) : sc_module(name),
           clk ("clk"), reset ("reset"), f_in ("f_in"), s_in ("s_in"), out ("out"), prio ("prio"),
           FAST_INDEX(FAST_INDEX)
    {
        SC_CTHREAD(proc_clk_mselector, clk.pos());
        SC_METHOD(proc_cmb_mselector);
            sensitive << f_in << s_in << s_in_reg << prio;
    }

    // Functions...
    void Trace (sc_trace_file *tf, int level) {
        PN_TRACE (tf, clk);
        PN_TRACE (tf, reset);
        PN_TRACE (tf, f_in);
        PN_TRACE (tf, s_in);
        PN_TRACE (tf, out);
        PN_TRACE (tf, prio);
        PN_TRACE (tf, s_in_reg);
    }
 private:
    const sc_uint<32> FAST_INDEX;

    // Registers...
    sc_signal<SSelectorIO<DWIDTH, SEL_MAX> > s_in_reg;

    // Signals...
    sc_signal<SSelectorIO<DWIDTH, SEL_MAX> > next_s_in_reg;
};

struct SCacheTag {
    bool valid;
    bool dirty;
    bool ac_r;
    bool ac_w;
    bool ac_x;
    bool ac_u; // Paging flags
    sc_uint<CFG_MEMU_CACHE_WAYS_LD>  way;
    sc_uint<32-CFG_MEMU_CACHE_SETS_LD-CFG_MEMU_CACHE_BANKS_LD-2>  tadr;
    

    // Necessary operators for using this structure as signals...
    // Compare (necessary for signal updates)...
    bool operator== (const SCacheTag &t) const{
        return(
               valid == t.valid 
            && dirty == t.dirty 
            && ac_r == t.ac_r 
            && ac_w == t.ac_w 
            && ac_x == t.ac_x 
            && ac_u == t.ac_u
            && tadr == t.tadr 
            && way == t.way);
    };

    // Tracing...
    friend void sc_trace( sc_trace_file *tf, const SCacheTag &t, const std::string &name ) {
        PN_TRACE_R(tf, t, valid, name);
        PN_TRACE_R(tf, t, dirty, name);
        PN_TRACE_R(tf, t, ac_r, name);
        PN_TRACE_R(tf, t, ac_w, name);
        PN_TRACE_R(tf, t, ac_x, name);
        PN_TRACE_R(tf, t, ac_u, name);
        PN_TRACE_R(tf, t, tadr, name);
        PN_TRACE_R(tf, t, way, name);

    }
    friend ostream& operator<< (::std::ostream& os, const SCacheTag& t) {
        os << "{" << t.valid << ", " << t.dirty << ", " << t.ac_r << ", "  
        << t.ac_u << ", " << t.ac_w << ", " << t.ac_x << ", " 
        << t.tadr << ", " << t.way << "}";
        return os;
    }
};


// **************** Helpers *********************

static inline sc_uint<32> GetBankOfAdr (sc_uint<32>  adr) { return (adr >> 2) & ((1U<<CFG_MEMU_CACHE_BANKS_LD) - 1); }
static inline sc_uint<32> GetIndexOfAdr (sc_uint<32>  adr) {
    //PN_INFOF(("IndexOfAdr: %d",(adr >> (CFG_MEMU_CACHE_BANKS_LD + 2)) & ((1<<CFG_MEMU_CACHE_SETS_LD) - 1)));
    return (adr >> (CFG_MEMU_CACHE_BANKS_LD + 2U)) & ((1U<<CFG_MEMU_CACHE_SETS_LD) - 1U);
}
static inline sc_uint<32> GetWayIndexOfAdr (sc_uint<32>  adr, sc_uint<32>  way) {
    return GetIndexOfAdr (adr) | (way << CFG_MEMU_CACHE_SETS_LD);
}
static inline sc_uint<32> GetIndexOfWayIndex (sc_uint<32>  adr) { return adr & ((1U<<CFG_MEMU_CACHE_SETS_LD) - 1); }
static inline sc_uint<32> GetTagOfAdr (sc_uint<32>  adr) {
    return adr >> (CFG_MEMU_CACHE_SETS_LD + CFG_MEMU_CACHE_BANKS_LD + 2);
}
static inline sc_uint<32> GetLineOfAdr (sc_uint<32>  adr) { return adr >> (CFG_MEMU_CACHE_BANKS_LD + 2); }

static inline sc_uint<32> ComposeAdress (sc_uint<32>  tadr, sc_uint<32>  iadr, sc_uint<32>  bank, sc_uint<32>  byteNo = 0) {
    return (tadr << (CFG_MEMU_CACHE_SETS_LD + CFG_MEMU_CACHE_BANKS_LD + 2)) |
           (iadr << (CFG_MEMU_CACHE_BANKS_LD + 2)) | (bank << 2) | byteNo;
}


static inline void RequestRelease (sc_out<bool> *req) { *req = 0; }

#endif