/*************************************************************************

  This file is part of the ParaNut project.

  Copyright (C) 2010-2020 Alexander Bahle <alexander.bahle@hs-augsburg.de>
                          Gundolf Kiefer <gundolf.kiefer@hs-augsburg.de>
      Hochschule Augsburg, University of Applied Sciences

  Description:
    This is the memory unit (MEMU) of the ParaNut.
    The MEMU interfaces with the main memory bus over a wishbone interface
    and with the ParaNut CPUs over an arbitrary number of read and write
    ports. The MEMU contains the (shared) cache and is optimize to handle
    parallel memory accesses from different ports efficiently.
    Also, the support for synchronization primitives is due to the MEMU.

    The MEMU (class 'MMemu') contains the following sub-modules:
    - 1 tag RAM ('MTagRAM) for storing and supplying cache tag information
    - CFG_MEMU_CACHE_BANKS cache memory banks ('MBankRam') for storing cached data
    - 1 bus interface ('MBusIF') for the Wishbone interconnection
    - RPORTS read ports ('MReadPort')
    - WPORTS write ports ('MWritePort')
    - 1 arbiter ('MArbiter') for controlling the access to the caches' tag
      and bank data

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


#ifndef _MEMU_
#define _MEMU_

#include "base.h"
#include "paranut-config.h"

#include <systemc.h>


/*
   To-Do (TBD)
   -----

   - (EXAMINE: priority: RD/RI/WD per core, not global?  -> starvation may occur if global)
   - (optimize BusIf: reset 'busy' before WB write-back -> does not really help)
*/

// MArbiter Selector input/output type...
template<int DWIDTH = 1, int SEL_MAX = 1>
struct SSelectorIO {
    sc_uint<DWIDTH> dat;
    sc_uint<NUM_BITS(SEL_MAX)> sel;
    bool sel_valid;

    SSelectorIO( sc_uint<DWIDTH> _dat = 0,  sc_uint<NUM_BITS(SEL_MAX)> _sel = 0, bool _sel_valid = 0): dat(_dat), sel(_sel), sel_valid(_sel_valid) { }

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

    void CombMethod() {
        out = f_in.read ();
    }

    SC_HAS_PROCESS(MSelectorPass); // Necessary when not using SC_MODULE macro
    MSelectorPass (const sc_module_name &name) : sc_module (name),
          f_in ("f_in"), out ("out")
    {
        SC_METHOD(CombMethod);
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

    void TransitionMethod() {
        if (reset == 1) {
            SSelectorIO<DWIDTH, SEL_MAX> reg; // Default constructor is (0, 0, 0)
            s_in_reg = reg;
        } else
            s_in_reg = next_s_in_reg.read ();
    }

    void CombMethod() {
        SSelectorIO<DWIDTH, SEL_MAX> reg, f_in_var, s_in_var, out_var;
        bool slow_sel_valid;

        // Read inputs...
        f_in_var = f_in.read ();
        s_in_var = s_in.read ();

        // Set defaults...
        reg = s_in_reg.read ();
        slow_sel_valid = (reg.sel_valid & s_in_var.sel_valid) && (reg.sel == s_in_var.sel);

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
        out_var.sel_valid = f_in_var.sel_valid | slow_sel_valid;

        // Write results...
        next_s_in_reg = s_in_var;
        out = out_var;
    }

    SC_HAS_PROCESS(MSelector); // Necessary when not using SC_MODULE macro
    MSelector (const sc_module_name &name, unsigned int FAST_INDEX = 0) : sc_module(name),
           clk ("clk"), reset ("reset"), f_in ("f_in"), s_in ("s_in"), out ("out"), prio ("prio"),
           FAST_INDEX(FAST_INDEX)
    {
        SC_METHOD(TransitionMethod);
            sensitive << clk.pos();
        SC_METHOD(CombMethod);
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
    const unsigned int FAST_INDEX;

    // Registers...
    sc_signal<SSelectorIO<DWIDTH, SEL_MAX> > s_in_reg;

    // Signals...
    sc_signal<SSelectorIO<DWIDTH, SEL_MAX> > next_s_in_reg;
};



struct SCacheTag {
    bool valid, dirty;
    // bool ac_r, ac_w, ac_x;   // access control (for later extension)
    TWord tadr; // requires 32 - CFG_MEMU_CACHE_SETS_LD - CFG_MEMU_CACHE_BANKS_LD - 2 bits
    TWord way; // requires CFG_MEMU_CACHE_WAYS_LD bits

    // Necessary operators for using this structure as signals...
    // Compare (necessary for signal updates)...
    bool operator== (const SCacheTag &t) const;
    // Display...
    friend ostream &operator<< (ostream &os, const SCacheTag &t);
    // Tracing...
    friend void sc_trace (sc_trace_file *tf, const SCacheTag &t, const std::string &name);
};


// Examples:
// a) 4 banks/16 bytes per line, 512 lines
//    -> size = 16 * 512 = 8 KB
//    -> 32-9-2-2 = 19 adress bits / 21 tag bits -> tag memory must be 32 bits wide
//
// b) 8 banks/32 bytes per line, 8192 lines
//    -> size = 32 * 8192 = 256 KB
//    -> 32-13-3-2 = 14 adress bits / 16 tag bits -> tag memory may be 16 bits wide


// **************** MTagRam ******************

#define TR_PORTS CFG_NUT_CPU_CORES // see comment to 'MArbiter'

// Tag layout in memory
// | Valid bit | Dirty bit | Tag address |
struct STag {
    bool valid, dirty;
    TWord tadr;
};

// Tag entry layout in memory
// | Tags (way(0)..way(n-1)) | LRU info |
struct STagEntry {
    STag tag[CFG_MEMU_CACHE_WAYS];
    TWord use; // only used with LRU replacement (Note: saved in its own block ram in hardware)
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

    sc_in<TWord> adr[TR_PORTS]; // complete adress (lower CFG_MEMU_CACHE_BANKS_LD + 2 bits can be omitted)
    sc_in<TWord> wadr[TR_PORTS]; // complete write adress (lower CFG_MEMU_CACHE_BANKS_LD + 2 bits can be omitted)
    sc_in<SCacheTag> tag_in[TR_PORTS];
    // TBD: only the BusIf needs this port, write ports only set the dirty bit -> a 'wr_dirty' signal is sufficient
    sc_out<SCacheTag> tag_out[TR_PORTS];

    // Constructor...
    SC_HAS_PROCESS (MTagRam);
    MTagRam (sc_module_name name)
        : sc_module (name) {
        SC_THREAD (MainThread);
            sensitive << clk.pos ();
            reset_signal_is (reset, true);
    }

    // Functions...
    void Trace (sc_trace_file * tf, int level = 1);

    // Processes...
    void MainThread ();

protected:
    STagEntry ram_[CFG_MEMU_CACHE_SETS];

    // Registers...
    sc_signal<bool> write_tag;
    sc_signal<TWord> wtag_way;
    sc_signal<TWord> wtag_iadr;
    sc_signal<TWord> wtag_port;
    //   for LRU replacement...
    sc_signal<TWord> use_reg[TR_PORTS+1], use_iadr_reg[TR_PORTS+1]; // One for each reader + one for all writer
    sc_signal<bool> use_wr_reg[TR_PORTS+1];                         // One for each reader + one for all writer
    //   for random replacement...
    sc_signal<TWord> counter;
};


// **************** MBankRam ********************

class MBankRam : ::sc_core::sc_module {
public:
    // Ports ...
    sc_in<bool> clk;

    sc_in<bool> rd[CFG_MEMU_BANK_RAM_PORTS], wr[CFG_MEMU_BANK_RAM_PORTS];

    sc_in<sc_uint<4> > wen[CFG_MEMU_BANK_RAM_PORTS]; // byte write enable bits

    sc_in<TWord> wiadr[CFG_MEMU_BANK_RAM_PORTS]; // line adress (way & index bits of adress)
    sc_in<TWord> wdata[CFG_MEMU_BANK_RAM_PORTS];
    sc_out<TWord> rdata[CFG_MEMU_BANK_RAM_PORTS];

    // Constructor...
    SC_HAS_PROCESS (MBankRam);
    MBankRam (sc_module_name name)
        : sc_module (name) { SC_CTHREAD (MainThread, clk.pos ()); }

    // Functions...
    void Trace (sc_trace_file * tf, int level = 1);

    // Processes...
    void MainThread ();

protected:
    TWord ram_[CFG_MEMU_CACHE_SETS * CFG_MEMU_CACHE_WAYS];
};


// **************** MBusIf **********************

typedef enum {
    BifIdle = 0,
    BifDirectRead1,
    BifDirectRead2,
    BifDirectWrite,
    BifCacheRequestLLWait,
    BifCacheRequestRTWait,
    BifCacheReadTag,
    BifCacheReplaceReadIdata,
    BifCacheReadDirtyBanks,
    BifCacheReplaceInvalidateTag,
    BifCacheReplaceWriteBanks,
    BifCacheWriteTag,
    BifCacheWriteBackVictim,
    BifCacheAck,
    BifCacheAll,
} EBusIfState;

/*
typedef enum {
  bioNothing = 0,     // nothing
  bioReplace = 7,     // replace cache line;
  bioWriteback = 1,   // write cache line back
  bioFlush = 3,       // write back and invalidate afterwards
  bioInvalidate = 2,  // just invalidate, do not write back
  bioDirectRead = 5,  // direct (uncached) read operation
  bioDirectWrite = 6  // direct (uncached) write operation
} EBusIfOperation;
*/

typedef sc_uint<4> EBusIfOperation;
#define bioNothing 0 // nothing
#define bioWriteback 1 // write back cache line to main memory
#define bioInvalidate 2 // just invalidate, do not write back
#define bioFlush 3 // write back and invalidate afterwards
// 4 undefined so we can use bit 2 to determine if its a cache all operation
#define bioWritebackAll 5 // write back whole cache
#define bioInvalidateAll 6 // invalidate whole cache
#define bioFlushAll 7 // write back and invalidate whole cache
#define bioDirectRead 8 // direct (uncached) read operation
#define bioDirectWrite 9 // direct (uncached) write operation
#define bioReplace 10 // replace cache line

#define BUSIF_DATA_REG_NUM (CFG_MEMU_CACHE_BANKS/(CFG_MEMU_BUSIF_WIDTH/32))
#define BUSIF_DATA_REG_NUM_LD (CFG_MEMU_CACHE_BANKS_LD-(CFG_MEMU_BUSIF_WIDTH/64))

struct SBusIfRegs {
    EBusIfState                         state;
    EBusIfOperation                     op;
    bool                                linelock;
    sc_uint<4>                          bsel;
    sc_uint<32>                         adr;
    sc_uint<CFG_MEMU_CACHE_BANKS_LD>    adr_ofs;
    bool                                idata_valid [CFG_MEMU_CACHE_BANKS];
    sc_uint<CFG_MEMU_BUSIF_WIDTH>       idata [BUSIF_DATA_REG_NUM];
    sc_uint<32>                         odata [CFG_MEMU_CACHE_BANKS]; // TBD: rename to "rdata...", "wdata..."
    sc_uint<BUSIF_DATA_REG_NUM_LD>      cnt;
    sc_uint<CFG_MEMU_CACHE_BANKS>       banks_left,
                                        last_banks_left;
    SCacheTag                           tag;

    // Necessary operators for using this structure as signals...
    // Compare (necessary for signal updates)...
    bool operator == (const SBusIfRegs &t);
    // Display...
    friend ostream& operator << ( ostream& o, const SBusIfRegs &t );
    // Tracing...
    friend void sc_trace(sc_trace_file *tf, const SBusIfRegs &t, const std::string &name);
 };



class MBusIf : ::sc_core::sc_module {
public:
    // Ports ...
    sc_in<bool> clk, reset;

    //   Bus interface (Wishbone)...
    sc_out<bool>        wb_cyc_o;                       // cycle valid output
    sc_out<bool>        wb_stb_o;                       // strobe output
    sc_out<bool>        wb_we_o;                        // indicates write transfer
    sc_out<sc_uint<3> > wb_cti_o;                       // cycle type identifier
    sc_out<sc_uint<2> > wb_bte_o;                       // burst type extension
    sc_out<sc_uint<CFG_MEMU_BUSIF_WIDTH/8> > wb_sel_o;  // byte select outputs
    sc_out<sc_uint<32> > wb_adr_o;                      // address bus outputs
    sc_out<sc_uint<CFG_MEMU_BUSIF_WIDTH> > wb_dat_o;    // output data bus

    sc_in<bool>         wb_ack_i;                       // normal termination
    // sc_in<bool>          wb_err_i;                   // termination w/ error
    // sc_in<bool>          wb_rty_i;                   // termination w/ retry
    sc_in<sc_uint<CFG_MEMU_BUSIF_WIDTH> > wb_dat_i;     // input data bus

    //   Control inputs/outputs...
    sc_in<EBusIfOperation> busif_op;    // all operations are atomic & by default acquire a line lock as needed
    sc_in<bool> busif_nolinelock;       // if set, no line lock is acquired (for write misses: lock is already held by write port)
    sc_in<sc_uint<4> > busif_bsel;      // byte select (only for uncached read & write access)
    sc_out<bool> busif_busy;            // if 0, the BusIF is ready for a new operation
                                        // if 0, data writes to the cache (tag & bank) are guaranteed to be completed

    //   Control lines to Tag & Cache banks...
    sc_out<bool> tag_rd, tag_rd_way, tag_wr, bank_rd[CFG_MEMU_CACHE_BANKS], bank_wr[CFG_MEMU_CACHE_BANKS];

    //   Adress & data busses...
    sc_in<TWord> adr_in;
    sc_out<TWord> adr_out; // to cache banks, tag bank, and r/w ports to determine BUSIF hits
                           // Note: If data width is set to 64Bit bit the 3rd bit of adr_out will be
                           // fixed to 0, to ensure correct hit detection in RPs
    sc_in<TWord> data_in[CFG_MEMU_CACHE_BANKS];     // data from cache banks (bank_rd[n] = 1) or write ports (else)...
    sc_out<TWord> data_out[CFG_MEMU_CACHE_BANKS];   // data read from bus
    sc_out<bool> data_out_valid[CFG_MEMU_CACHE_BANKS];
    sc_in<SCacheTag> tag_in;
    sc_out<SCacheTag> tag_out;

    //   Request & grant lines...
    sc_out<bool> req_linelock, req_tagw, req_tagr, req_bank[CFG_MEMU_CACHE_BANKS];
    sc_in<bool> gnt_linelock, gnt_tagw, gnt_tagr, gnt_bank[CFG_MEMU_CACHE_BANKS];

    // Constructor...
    SC_HAS_PROCESS (MBusIf);
    MBusIf (sc_module_name name)
        : sc_module (name) {
        SC_METHOD (MainMethod);
            sensitive << clk.pos();
        SC_METHOD (TransitionMethod);
            sensitive << regs;
            sensitive << busif_op << busif_nolinelock << busif_bsel << adr_in << tag_in;
            sensitive << gnt_linelock << gnt_tagw << gnt_tagr;
            sensitive << wb_ack_i << wb_dat_i;
            for (int n = 0; n < CFG_MEMU_CACHE_BANKS; n++)
                sensitive << data_in[n] << gnt_bank[n];
    }

    // Functions...
    void Trace (sc_trace_file * tf, int level = 1);

    // Processes...
    void OutputMethod ();
    void MainThread ();
    void MainMethod ();
    void TransitionMethod ();

protected:
    // Helpers...
    void AcceptNewOp ();

    // Registers...
    sc_signal<SBusIfRegs> regs;

    // Internal Signals...
    sc_signal<SBusIfRegs> next_regs;
    sc_signal<TWord> state_trace;

};



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
    sc_in<bool> clk, reset;

    //   With (CPU) port...
    //   - All input ports must be held until 'port_ack' is asserted, at least for one cycle.
    //   - 'port_ack' is issued for exactly one cycle (no full handshake).
    //   - 'port_ack' is issued one cycle before the data is valid (pipelining).
    //   - The next request may be issued at earliest one cycle after 'port_ack' was asserted (which is the same cycle the data is delivered).
    //   - ('port_ack' may be asserted already in the same cycle as the request was issued.) -> minimum latency on cache hit is presently 2 clocks
    //   - If 'port_direct' is set, nothing happens with the cache (i.e. no invalidation).
    //     Hence, 'port_direct' = 0/1 should not be mixed for one adress.
    sc_in<bool> port_rd, port_direct;
    sc_out<bool> port_ack;
    sc_in<TWord> port_adr;
    sc_out<TWord> port_data;

    sc_in<bool> port_lres_scond;    // for LR/SC: reserve current adress
    sc_out<bool> port_scond_ok;     // for LR/SC: set, if store-conditional would be sucessful

    //   With BUSIF...
    sc_in<TWord> busif_adr;     // adress BUSIF is currently working on (to check for BUSIF hits)
    sc_in<TWord> busif_data;    // must be routed here from 'busif.data_out[]' according to 'port_adr'
    sc_in<bool> busif_data_valid[CFG_MEMU_CACHE_BANKS];
    sc_out<EBusIfOperation> busif_op; // This can be replaced by one bit (only 'bioReplace' is invoked)
    sc_in<bool> busif_busy;

    //   With cache...
    //   - Adress information for tag is routed around this module from 'port_adr', for bank from 'port_adr' AND 'way_out'.
    sc_out<bool> tag_rd, bank_rd;   // 'bank_rd' and 'cache_data' must be routed according to 'port_adr'
    sc_in<TWord> bank_data_in;      // must be routed according to 'bank_sel'
    sc_out<TWord> bank_sel;
    sc_in<SCacheTag> tag_in;
    sc_out<TWord> way_out;

    //   Request & grant lines...
    sc_out<bool> req_tagr, req_bank[CFG_MEMU_CACHE_BANKS], req_busif;
    sc_in<bool> gnt_tagr, gnt_bank[CFG_MEMU_CACHE_BANKS], gnt_busif;

    //   With snoop unit (arbiter)...
    sc_in<TWord> snoop_adr;
    sc_in<bool> snoop_stb;

    // Constructor...
    SC_HAS_PROCESS (MReadPort);
    MReadPort (sc_module_name name)
        : sc_module (name) {
        SC_METHOD (HitMethod);
            sensitive << port_adr << busif_adr;
            for (int b = 0; b < CFG_MEMU_CACHE_BANKS; b++)
                sensitive << busif_data_valid[b];
        SC_METHOD (TransitionMethod);
            sensitive << clk.pos ();
        SC_METHOD (MainMethod);
            sensitive << reset << state_reg << bank_sel_reg << busif_hit;
            sensitive << port_rd << port_direct << port_adr << port_lres_scond;
            sensitive << busif_data << busif_busy;
            sensitive << bank_data_in << tag_in;
            sensitive << gnt_tagr << gnt_busif;
            sensitive << snoop_adr << snoop_stb;
            sensitive << link_valid_reg << link_adr_reg;
            for (int b = 0; b < CFG_MEMU_CACHE_BANKS; b++)
                sensitive << gnt_bank[b];
    }

    // Functions...
    void Trace (sc_trace_file * tf, int level = 1);

    // Processes...
    void HitMethod ();
    void TransitionMethod ();
    void MainMethod ();

protected:
    // Registers...
    sc_signal<EReadportState> state_reg;
    sc_signal<int> state_trace;

    sc_signal<TWord> bank_sel_reg;

    sc_signal<TWord> link_adr_reg;
    sc_signal<bool> link_valid_reg;

    // Internal signals...
    sc_signal<bool> busif_hit;
    sc_signal<EReadportState> next_state;
    sc_signal<TWord> next_bank_sel;
    sc_signal<TWord> next_link_adr_reg;
    sc_signal<bool> next_link_valid_reg;
};


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
    s_wp_special_wait_complete
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

    sc_in<sc_uint<3> > port_cache_op;            // Cache control; | 0: writeback | 1: invalidate | 2: all |
    
    sc_in<TWord> port_adr;
    sc_in<TWord> port_data;

    sc_in<bool> port_lres_scond;    // for LR/SC: store only if lres_valid
    sc_in<bool> port_scond_ok;

    //   With BUSIF...
    //   - There is no direct transfer except for uncached memory access.
    sc_in<TWord> busif_adr; // adress BUSIF is currently working on (to check for BUSIF hits)
    sc_out<EBusIfOperation> busif_op;
    sc_out<bool> busif_nolinelock; // if set, no line lock is acquired (for write misses: lock is already held by write port) (TBD: eliminate this signal)
    sc_in<bool> busif_busy;

    //   With cache...
    //   - Adress information for tag must be routed from 'port_adr', for the banks from 'port_adr' and 'tag_out.way'.
    sc_out<bool> tag_rd, tag_wr, bank_rd, bank_wr;  // 'bank_rd', 'bank_wr' must be routed according to 'port_adr'
    sc_in<TWord> bank_data_in;                      // must be routed according to 'port_adr'
    sc_out<TWord> bank_data_out;                    // must be routed according to 'port_adr'
    sc_out<sc_uint<4> > bank_bsel;
    sc_in<SCacheTag> tag_in;
    sc_out<SCacheTag> tag_out;

    //   Request & grant lines...
    sc_out<bool> req_linelock, req_tagr, req_tagw, req_bank[CFG_MEMU_CACHE_BANKS], req_busif;
    sc_in<bool> gnt_linelock, gnt_tagr, gnt_tagw, gnt_bank[CFG_MEMU_CACHE_BANKS], gnt_busif;

    // Constructor...
    SC_HAS_PROCESS (MWritePort);
    MWritePort (sc_module_name name)
        : sc_module (name) {
        SC_METHOD (TransitionMethod);
            sensitive << clk.pos ();
        SC_METHOD (MainMethod);
            sensitive << reset << state_reg;
            sensitive << port_wr << port_direct << port_bsel << port_lres_scond << port_cache_op
                      << port_adr << port_data << port_lres_scond << port_scond_ok;
            sensitive << busif_adr << busif_busy << bank_data_in << tag_in;
            sensitive << gnt_linelock << gnt_tagr << gnt_tagw << gnt_busif;
            sensitive << tag_reg << data_reg;
            for (int b = 0; b < CFG_MEMU_CACHE_BANKS; b++)
                sensitive << gnt_bank[b];
    }

    // Functions...
    void Trace (sc_trace_file * tf, int level = 1);

    // Processes...
    void TransitionMethod ();
    void MainMethod ();

protected:
    // Registers...
    sc_signal<EWritePortState> state_reg;
    sc_signal<int> state_trace;

    sc_signal<SCacheTag> tag_reg;
    sc_signal<TWord> data_reg;

    // Internal signals..
    sc_signal<EWritePortState> next_state;
    sc_signal<SCacheTag> next_tag_reg;
    sc_signal<TWord> next_data_reg;
};


// **************** MArbiter ********************


class MArbiter : ::sc_core::sc_module {
public:
    // Ports ...
    sc_in<bool> clk, reset;

    sc_in<TWord> wiadr_busif, wiadr_rp[CFG_MEMU_RPORTS], adr_wp[CFG_MEMU_WPORTS],
        way_wp[CFG_MEMU_WPORTS]; // (way+index) adresses from various ports;
                        // must be kept constant as long as 'req_*' lines are held
    sc_out<TWord> wiadr_bank[CFG_MEMU_CACHE_BANKS][CFG_MEMU_BANK_RAM_PORTS]; // adress lines to the banks

    //   Write snooping...
    sc_out<TWord> snoop_adr;
    sc_out<bool> snoop_stb[CFG_MEMU_WPORTS];

    //   Line lock...
    //   - Implements mutex for a single cache line (tag + all data banks).
    //   - Writers are supposed to acquire a line lock.
    //   - Reader do not acquire anything => Writers must perform their actions in a safe order.
    //   - if a line lock request is held, the associated address must not change
    //   - A line lock may be released during the clock cycle that the last bank/tag access is made,
    //     given that the bank/tag lock is still held. This allows faster write cycles without write
    //     port monopolizing a line lock (keeping it permanently up).
    sc_in<bool> req_busif_linelock, req_wp_linelock[CFG_MEMU_WPORTS];
    sc_out<bool> gnt_busif_linelock, gnt_wp_linelock[CFG_MEMU_WPORTS];

    //   Tag RAM...
    //   - Tag RAM must have CFG_NUT_CPU_CORES ports.
    //   - Each port is reserved for one CPU (three ports: RP #n, RP #CFG_NUT_CPU_CORES+n, WP #n).
    //   - BUSIF uses port #CFG_NUT_CPU_CORES-1.
    //   - Arbiter provides:
    //     a) Write-/Read-Lock (a write-lock excludes all readers)
    //     b) Port arbitration: Multiple readers are allowed, but only one per port (see above)
    //     c) Priority selection amoung a port: 0. BUSIF (if applicable), 1. Data read, 2. Insn Read, 3. Data Write
    //     EXAMINE: Would two ports/CPU bring speed improvement? Would a seperate port for BUSIF bring speed improvement?
    //   - The tag RAM arbitration is also used to prevent writers from replacing/changing a cache line
    //     while it is read during a cache read hit. Hence, a reader must keep its 'tagr' lock until its bank access
    //     is completed, too. This is not necessary, if a line lock is held (i.e. for cache line reading during a write miss).
    sc_in<bool> tagram_ready;
    sc_in<bool> req_busif_tagw, req_wp_tagw[CFG_MEMU_WPORTS], req_busif_tagr, req_wp_tagr[CFG_MEMU_WPORTS], req_rp_tagr[CFG_MEMU_RPORTS];
    sc_out<bool> gnt_busif_tagw, gnt_wp_tagw[CFG_MEMU_WPORTS], gnt_busif_tagr, gnt_wp_tagr[CFG_MEMU_WPORTS], gnt_rp_tagr[CFG_MEMU_RPORTS];
    sc_out<bool> gnt_busif_tagw_r, gnt_wp_tagw_r[CFG_MEMU_WPORTS], gnt_busif_tagr_r, gnt_wp_tagr_r[CFG_MEMU_WPORTS], gnt_rp_tagr_r[CFG_MEMU_RPORTS];

    //   Bank RAMs...
    //   - All ports of CPU n must be linked to port n % CFG_MEMU_BANK_RAM_PORTS of each bank RAM.
    //   - The BUSIF is linked to port #(CFG_MEMU_BANK_RAM_PORTS-1).
    //   - Multiple usually conflicting grant signals may be set, if the adresses match.
    //   - As long as a request signal is set, the adress must not change!
    //   - Amoung write ports and the BUSIF, only one grant will be given to avoid writing conflicts.
    sc_in<bool> req_busif_bank[CFG_MEMU_CACHE_BANKS], req_wp_bank[CFG_MEMU_WPORTS][CFG_MEMU_CACHE_BANKS],
        req_rp_bank[CFG_MEMU_RPORTS][CFG_MEMU_CACHE_BANKS];
    sc_out<bool> gnt_busif_bank[CFG_MEMU_CACHE_BANKS], gnt_wp_bank[CFG_MEMU_WPORTS][CFG_MEMU_CACHE_BANKS],
        gnt_rp_bank[CFG_MEMU_RPORTS][CFG_MEMU_CACHE_BANKS];

    //   BUSIF...
    sc_in<bool> req_rp_busif[CFG_MEMU_RPORTS], req_wp_busif[CFG_MEMU_WPORTS];
    sc_out<bool> gnt_rp_busif[CFG_MEMU_RPORTS], gnt_wp_busif[CFG_MEMU_WPORTS];
    // select/routing information can be derived from grant lines (only one is set at a time)

    // Note on deadlock prevention:
    // 1. Requests/grants must always in the following order (-> break cyclic wait condition):
    //      busif < linelock < tagr/tagw < bank
    // 2. (corollary to 1.) R/W ports must not request anything when waiting for BusIF (i.e. 'busif_busy')
    // 3. tag/bank access may never be requested in a hold-and-wait manner: Either request simultaneously or use & complete serially.

    // Constructor...
    SC_HAS_PROCESS (MArbiter);
    MArbiter (sc_module_name name)
        : sc_module (name) {
        int n, b;

        SC_METHOD (LineLockMethod);
            sensitive << req_busif_linelock << wiadr_busif;
            sensitive << linelock_sel_out;
            for (n = 0; n < CFG_MEMU_WPORTS; n++)
                sensitive << req_wp_linelock[n] << adr_wp[n];
        SC_METHOD (TagMethod);
            sensitive << tagram_ready << req_busif_tagr << req_busif_tagw;
            sensitive << tagr_reg << tagw_reg << req_tagw_reg;
            for (n = 0; n < CFG_MEMU_RPORTS; n++)
                sensitive << req_rp_tagr[n];
            for (n = 0; n < CFG_MEMU_WPORTS; n++)
                sensitive << req_wp_tagr[n] << req_wp_tagw[n];
        SC_METHOD (BankMethod);
            sensitive << wiadr_busif;
            for (n = 0; n < CFG_MEMU_RPORTS; n++)
                sensitive << wiadr_rp[n];
            for (n = 0; n < CFG_MEMU_WPORTS; n++)
                sensitive << adr_wp[n] << way_wp[n];
            for (b = 0; b < CFG_MEMU_CACHE_BANKS; b++) {
                sensitive << req_busif_bank[b];
                for (n = 0; n < CFG_MEMU_BANK_RAM_PORTS; n++)
                    sensitive << bank_sel_out[b][n];
                for (n = 0; n < CFG_MEMU_RPORTS; n++)
                    sensitive << req_rp_bank[n][b];
                for (n = 0; n < CFG_MEMU_WPORTS; n++)
                    sensitive << req_wp_bank[n][b];
            }
        SC_METHOD (BusIfMethod);
            sensitive << busif_reg;
            for (n = 0; n < CFG_MEMU_RPORTS; n++)
                sensitive << req_rp_busif[n];
            for (n = 0; n < CFG_MEMU_WPORTS; n++)
                sensitive << req_wp_busif[n];
            sensitive << busif_sel_out;
        SC_METHOD (SnoopMethod);
            sensitive << next_linelock_reg;
            for (n = 0; n < CFG_MEMU_WPORTS; n++)
                sensitive << adr_wp[n];
        SC_METHOD (PrioCPUMethod);
            sensitive << counter_reg;

        SC_CTHREAD (TransitionThread, clk.pos ());
            reset_signal_is (reset, true);

        for (int b = 0; b < CFG_MEMU_CACHE_BANKS; b++) {
            // Generate the Selector or Passthrough for the Bank BusIf Arbitration...
            if (CFG_NUT_CPU_CORES >= 2 || CFG_MEMU_BANK_RAM_PORTS == 1) {
                // The BankBusIf step has the BusIF and all other CPUs as input
                BankSelector_t *sel = new BankSelector_t(std::string("BankBusIfSel" + std::to_string (b)).c_str());
                sel->clk (clk);
                sel->reset (reset);
                sel->f_in (bank_sel_in[b][CFG_MEMU_BANK_RAM_PORTS-1][0]); // BusIf
                sel->s_in (bank_sel_in[b][CFG_MEMU_BANK_RAM_PORTS-1][1]);
                sel->out (bank_sel_out[b][CFG_MEMU_BANK_RAM_PORTS-1]);
                sel->prio (cpu_prio);
                bank_sel[b][CFG_MEMU_BANK_RAM_PORTS-1] = sel;

            } else /*if (CFG_NUT_CPU_CORES < 2)*/ {
                BankSelectorPass_t *pass = new BankSelectorPass_t(std::string("BankBusIfSel" + std::to_string (b)).c_str());
                pass->f_in (bank_sel_in[b][CFG_MEMU_BANK_RAM_PORTS-1][0]); // BusIf
                pass->out (bank_sel_out[b][CFG_MEMU_BANK_RAM_PORTS-1]);
                bank_sel[b][CFG_MEMU_BANK_RAM_PORTS-1] = pass;
            }

            // Generate the Selector or Passthrough for the Bank CPU Arbitration for 2 BRAM ports ...
            if (CFG_MEMU_BANK_RAM_PORTS > 1) {
                if (CFG_NUT_CPU_CORES >= 4) {
                    // The Bank step has the BusIF and all other CPUs as input
                    BankSelector_t *sel = new BankSelector_t(std::string("BankSel" + std::to_string (b)).c_str());
                    sel->clk (clk);
                    sel->reset (reset);
                    sel->f_in (bank_sel_in[b][0][0]); // CePU
                    sel->s_in (bank_sel_in[b][0][1]);
                    sel->out (bank_sel_out[b][0]);
                    sel->prio (cpu_prio);
                    bank_sel[b][0] = sel;

                } else /*if (CFG_NUT_CPU_CORES < 4)*/ {
                    BankSelectorPass_t *pass = new BankSelectorPass_t(std::string("BankSel" + std::to_string (b)).c_str());
                    pass->f_in (bank_sel_in[b][0][0]); // CePU
                    pass->out (bank_sel_out[b][0]);
                    bank_sel[b][0] = pass;
                }
            }
        }


        // Generate the Selector or Passthrough for the BusIf Arbitration...
        if (CFG_NUT_CPU_CORES >= 2) {
            // The step has the CePU and all other CoPUs as input
            BusifSelector_t *sel = new BusifSelector_t("BusIfSel");
            sel->clk (clk);
            sel->reset (reset);
            sel->f_in (busif_sel_in[0]); //CePU
            sel->s_in (busif_sel_in[1]);
            sel->out (busif_sel_out);
            sel->prio (cpu_prio);
            busif_sel = sel;
        }
        if (CFG_NUT_CPU_CORES < 2) {
            BusIfSelectorPass_t *pass = new BusIfSelectorPass_t("BusIfSel");
            pass->f_in (busif_sel_in[0]); //CePU
            pass->out (busif_sel_out);
            busif_sel = pass;
        }

       // Generate the Selector or Passthrough for the LineLock Arbitration...
        if (CFG_NUT_CPU_CORES >= 2) {
            // The step has the CePU WPORT and all other CoPUs WPORTs as input
            LineLockSelector_t *sel = new LineLockSelector_t("LineLockSel");
            sel->clk (clk);
            sel->reset (reset);
            sel->f_in (linelock_sel_in[0]); //CePU
            sel->s_in (linelock_sel_in[1]);
            sel->out (linelock_sel_out);
            sel->prio (cpu_prio);
            linelock_sel = sel;
        }
        if (CFG_NUT_CPU_CORES < 2) {
            LineLockSelectorPass_t *pass = new LineLockSelectorPass_t("LineLockSel");
            pass->f_in (linelock_sel_in[0]); //CePU
            pass->out (linelock_sel_out);
            linelock_sel = pass;
        }
    }
    ~MArbiter ();

    // Functions...
    void Trace (sc_trace_file * tf, int level = 1);

    // Processes...
    void LineLockMethod ();
    void TagMethod ();
    void BankMethod ();
    void BusIfMethod ();
    void SnoopMethod ();
    void PrioCPUMethod ();

    void TransitionThread ();


protected:

    // Helpers...
    int GetPrioCpu ();

    // Registers...
    sc_signal<TWord> counter_reg;
    sc_signal<sc_uint<CFG_MEMU_WPORTS + 1> > linelock_reg;
    sc_signal<sc_uint<CFG_MEMU_RPORTS + CFG_MEMU_WPORTS + 1> > tagr_reg;
    sc_signal<sc_uint<CFG_MEMU_WPORTS + 1> > req_tagw_reg;
    sc_signal<sc_uint<CFG_MEMU_WPORTS + 1> > tagw_reg;
    sc_signal<sc_uint<((CFG_MEMU_RPORTS + CFG_MEMU_WPORTS+(CFG_NUT_CPU_CORES_LD==0)) / CFG_MEMU_BANK_RAM_PORTS) + 1> > bank_reg[CFG_MEMU_CACHE_BANKS][CFG_MEMU_BANK_RAM_PORTS];
    sc_signal<sc_uint<CFG_MEMU_RPORTS + CFG_MEMU_WPORTS> > busif_reg;

    // Signals...
    sc_signal<sc_uint<CFG_MEMU_WPORTS + 1> > next_linelock_reg;
    sc_signal<sc_uint<CFG_MEMU_RPORTS + CFG_MEMU_WPORTS + 1> > next_tagr_reg;
    sc_signal<sc_uint<CFG_MEMU_WPORTS + 1> > next_req_tagw_reg;
    sc_signal<sc_uint<CFG_MEMU_WPORTS + 1> > next_tagw_reg;
    sc_signal<sc_uint<((CFG_MEMU_RPORTS + CFG_MEMU_WPORTS+(CFG_NUT_CPU_CORES_LD==0)) / CFG_MEMU_BANK_RAM_PORTS) + 1> > next_bank_reg[CFG_MEMU_CACHE_BANKS][CFG_MEMU_BANK_RAM_PORTS];
    sc_signal<sc_uint<CFG_MEMU_RPORTS + CFG_MEMU_WPORTS> > next_busif_reg;

    sc_signal<sc_uint<MAX(1, CFG_NUT_CPU_CORES_LD)> > cpu_prio;

    //  BusIf Arbitration Signals...
    typedef MSelectorPass<1, (CFG_MEMU_RPORTS+CFG_MEMU_WPORTS+1)> BusIfSelectorPass_t;
    typedef MSelector<1, (CFG_MEMU_RPORTS+CFG_MEMU_WPORTS+1)> BusifSelector_t;
    sc_module* busif_sel;

    sc_signal<SSelectorIO<1, (CFG_MEMU_RPORTS+CFG_MEMU_WPORTS+1)> > busif_sel_out;
    sc_signal<SSelectorIO<1, (CFG_MEMU_RPORTS+CFG_MEMU_WPORTS+1)> > busif_sel_in [2];

    //  LineLock Arbitration Signals...
    typedef MSelectorPass<1, CFG_MEMU_WPORTS> LineLockSelectorPass_t;
    typedef MSelector<1, CFG_MEMU_WPORTS> LineLockSelector_t;
    sc_module* linelock_sel;

    sc_signal<SSelectorIO<1, CFG_MEMU_WPORTS> > linelock_sel_out;
    sc_signal<SSelectorIO<1, CFG_MEMU_WPORTS> > linelock_sel_in [2];

    //  Bank Arbitration Signals...
    static const bool SINGLE_CPU = CFG_NUT_CPU_CORES == 1;
    typedef MSelectorPass<CFG_MEMU_CACHE_WAYS_LD+CFG_MEMU_CACHE_SETS_LD, (CFG_MEMU_RPORTS+CFG_MEMU_WPORTS+SINGLE_CPU)/CFG_MEMU_BANK_RAM_PORTS> BankSelectorPass_t;
    typedef MSelector<CFG_MEMU_CACHE_WAYS_LD+CFG_MEMU_CACHE_SETS_LD, (CFG_MEMU_RPORTS+CFG_MEMU_WPORTS+SINGLE_CPU)/CFG_MEMU_BANK_RAM_PORTS> BankSelector_t;
    sc_module* bank_sel[CFG_MEMU_CACHE_BANKS][CFG_MEMU_BANK_RAM_PORTS];

    sc_signal<SSelectorIO<CFG_MEMU_CACHE_WAYS_LD+CFG_MEMU_CACHE_SETS_LD, (CFG_MEMU_RPORTS+CFG_MEMU_WPORTS+SINGLE_CPU)/CFG_MEMU_BANK_RAM_PORTS> > bank_sel_out[CFG_MEMU_CACHE_BANKS][CFG_MEMU_BANK_RAM_PORTS];
    sc_signal<SSelectorIO<CFG_MEMU_CACHE_WAYS_LD+CFG_MEMU_CACHE_SETS_LD, (CFG_MEMU_RPORTS+CFG_MEMU_WPORTS+SINGLE_CPU)/CFG_MEMU_BANK_RAM_PORTS> > bank_sel_in[CFG_MEMU_CACHE_BANKS][CFG_MEMU_BANK_RAM_PORTS][2];

};


// **************** MMemu ***********************

class MMemu : ::sc_core::sc_module {
public:
    // Ports ...
    sc_in<bool> clk, reset;

    //   Bus interface (Wishbone)...
    sc_out<bool>        wb_cyc_o;                       // cycle valid output
    sc_out<bool>        wb_stb_o;                       // strobe output
    sc_out<bool>        wb_we_o;                        // indicates write transfer
    sc_out<sc_uint<3> > wb_cti_o;                       // cycle type identifier
    sc_out<sc_uint<2> > wb_bte_o;                       // burst type extension
    sc_out<sc_uint<CFG_MEMU_BUSIF_WIDTH/8> > wb_sel_o;  // byte select outputs
    sc_out<sc_uint<32> > wb_adr_o;                      // address bus outputs
    sc_out<sc_uint<CFG_MEMU_BUSIF_WIDTH> > wb_dat_o;    // output data bus

    sc_in<bool>         wb_ack_i;                       // normal termination
    // sc_in<bool>          wb_err_i;                   // termination w/ error
    // sc_in<bool>          wb_rty_i;                   // termination w/ retry
    sc_in<sc_uint<CFG_MEMU_BUSIF_WIDTH> > wb_dat_i;     // input data bus

    //   Read ports...
    //     ports 0 .. WPORT-1 are considerd to be data ports, the others to be instruction ports (with lower priority)
    sc_in<bool> rp_rd[CFG_MEMU_RPORTS], rp_direct[CFG_MEMU_RPORTS];
    sc_in<sc_uint<4> > rp_bsel[CFG_MEMU_RPORTS];
    sc_out<bool> rp_ack[CFG_MEMU_RPORTS];
    sc_in<TWord> rp_adr[CFG_MEMU_RPORTS];
    sc_out<TWord> rp_data[CFG_MEMU_RPORTS];

    //   Write ports...
    sc_in<bool> wp_wr[CFG_MEMU_WPORTS], wp_direct[CFG_MEMU_WPORTS];
    sc_in<sc_uint<4> > wp_bsel[CFG_MEMU_WPORTS];
    sc_out<bool> wp_ack[CFG_MEMU_WPORTS];
    sc_in<bool> wp_lres_scond[CFG_MEMU_WPORTS];
    sc_out<bool> wp_scond_ok[CFG_MEMU_WPORTS];
    sc_in<sc_uint<3> > wp_cache_op[CFG_MEMU_WPORTS];
    sc_in<TWord> wp_adr[CFG_MEMU_WPORTS];
    sc_in<TWord> wp_data[CFG_MEMU_WPORTS];

    // Constructor/Destructor...
    SC_HAS_PROCESS (MMemu);
    MMemu (sc_module_name name)
        : sc_module (name) {
        InitSubmodules ();
        SC_METHOD (TransitionMethod);
            sensitive << clk.pos ();
        InitInterconnectMethod ();
    }
    ~MMemu () { FreeSubmodules (); }

    // Functions...
    void Trace (sc_trace_file * tf, int levels = 1);

    // Processes...
    void TransitionMethod ();
    void InterconnectMethod ();

    // Submodules...
    MTagRam *tagRam;
    MBankRam *bankRam[CFG_MEMU_CACHE_BANKS];
    MBusIf *busIf;
    MArbiter *arbiter;
    MReadPort *readPorts[CFG_MEMU_RPORTS];
    MWritePort *writePorts[CFG_MEMU_WPORTS];

protected:
    // Tag RAM...
    sc_signal<bool> tagram_ready, tagram_rd[TR_PORTS], tagram_wr[TR_PORTS];
    sc_signal<TWord> tagram_adr[TR_PORTS];
    sc_signal<TWord> tagram_wadr[TR_PORTS];
    sc_signal<SCacheTag> tagram_tag_in[TR_PORTS], tagram_tag_out[TR_PORTS];

    // Bank RAM...
    sc_signal<bool> bankram_rd[CFG_MEMU_CACHE_BANKS][CFG_MEMU_BANK_RAM_PORTS], bankram_wr[CFG_MEMU_CACHE_BANKS][CFG_MEMU_BANK_RAM_PORTS];
    sc_signal<sc_uint<4> > bankram_wen[CFG_MEMU_CACHE_BANKS][CFG_MEMU_BANK_RAM_PORTS];
    sc_signal<TWord> bankram_wiadr[CFG_MEMU_CACHE_BANKS][CFG_MEMU_BANK_RAM_PORTS];
    sc_signal<TWord> bankram_wdata[CFG_MEMU_CACHE_BANKS][CFG_MEMU_BANK_RAM_PORTS];
    sc_signal<TWord> bankram_rdata[CFG_MEMU_CACHE_BANKS][CFG_MEMU_BANK_RAM_PORTS];

    // BUSIF...
    sc_signal<EBusIfOperation> busif_op;
    sc_signal<bool> busif_nolinelock, busif_busy;
    sc_signal<bool> busif_tag_rd, busif_tag_rd_way, busif_tag_wr, busif_bank_rd[CFG_MEMU_CACHE_BANKS],
    busif_bank_wr[CFG_MEMU_CACHE_BANKS];
    sc_signal<TWord> busif_adr_in, busif_adr_out, busif_data_in[CFG_MEMU_CACHE_BANKS],
    busif_data_out[CFG_MEMU_CACHE_BANKS];
    sc_signal<bool> busif_data_out_valid[CFG_MEMU_CACHE_BANKS];
    sc_signal<SCacheTag> busif_tag_in, busif_tag_out;
    sc_signal<sc_uint<4> > busif_bsel;

    // Read ports...
    sc_signal<TWord> rp_busif_data_reg[CFG_MEMU_BUSIF_WIDTH/32]; // register to delay BusIF data for one clock cycle in accordance with the protocol
    sc_signal<TWord> rp_busif_data[CFG_MEMU_RPORTS];
    sc_signal<EBusIfOperation> rp_busif_op[CFG_MEMU_RPORTS];
    sc_signal<bool> rp_tag_rd[CFG_MEMU_RPORTS], rp_bank_rd[CFG_MEMU_RPORTS];
    sc_signal<SCacheTag> rp_tag_in[CFG_MEMU_RPORTS];
    sc_signal<TWord> rp_way_out[CFG_MEMU_RPORTS];
    sc_signal<TWord> rp_bank_data_in[CFG_MEMU_RPORTS];
    sc_signal<TWord> rp_bank_sel[CFG_MEMU_RPORTS];

    // Write ports...
    sc_signal<EBusIfOperation> wp_busif_op[CFG_MEMU_WPORTS];
    sc_signal<bool> wp_busif_nolinelock[CFG_MEMU_WPORTS];
    sc_signal<bool> wp_tag_rd[CFG_MEMU_WPORTS], wp_tag_wr[CFG_MEMU_WPORTS], wp_bank_rd[CFG_MEMU_WPORTS], wp_bank_wr[CFG_MEMU_WPORTS];
    sc_signal<SCacheTag> wp_tag_in[CFG_MEMU_WPORTS], wp_tag_out[CFG_MEMU_WPORTS];
    sc_signal<TWord> wp_bank_data_in[CFG_MEMU_WPORTS], wp_bank_data_out[CFG_MEMU_WPORTS];
    sc_signal<sc_uint<4> > wp_bank_bsel[CFG_MEMU_WPORTS];

    // Arbiter: request/grant signals (find comments in 'MArbiter')...
    sc_signal<bool> req_busif_linelock, req_wp_linelock[CFG_MEMU_WPORTS];
    sc_signal<bool> gnt_busif_linelock, gnt_wp_linelock[CFG_MEMU_WPORTS];

    sc_signal<bool> req_busif_tagw, req_wp_tagw[CFG_MEMU_WPORTS], req_busif_tagr, req_wp_tagr[CFG_MEMU_WPORTS],
        req_rp_tagr[CFG_MEMU_RPORTS];
    sc_signal<bool> gnt_busif_tagw, gnt_wp_tagw[CFG_MEMU_WPORTS], gnt_busif_tagr, gnt_wp_tagr[CFG_MEMU_WPORTS],
        gnt_rp_tagr[CFG_MEMU_RPORTS];
    sc_signal<bool> gnt_busif_tagw_r, gnt_wp_tagw_r[CFG_MEMU_WPORTS], gnt_busif_tagr_r, gnt_wp_tagr_r[CFG_MEMU_WPORTS],
        gnt_rp_tagr_r[CFG_MEMU_RPORTS];

    sc_signal<bool> req_busif_bank[CFG_MEMU_CACHE_BANKS], req_wp_bank[CFG_MEMU_WPORTS][CFG_MEMU_CACHE_BANKS],
    req_rp_bank[CFG_MEMU_RPORTS][CFG_MEMU_CACHE_BANKS];
    sc_signal<bool> gnt_busif_bank[CFG_MEMU_CACHE_BANKS], gnt_wp_bank[CFG_MEMU_WPORTS][CFG_MEMU_CACHE_BANKS],
        gnt_rp_bank[CFG_MEMU_RPORTS][CFG_MEMU_CACHE_BANKS];

    sc_signal<bool> req_rp_busif[CFG_MEMU_RPORTS], gnt_rp_busif[CFG_MEMU_RPORTS];
    sc_signal<bool> req_wp_busif[CFG_MEMU_WPORTS], gnt_wp_busif[CFG_MEMU_WPORTS];

    // Arbiter: other signals ...
    sc_signal<TWord> wiadr_busif, wiadr_rp[CFG_MEMU_RPORTS], adr_wp[CFG_MEMU_WPORTS], way_wp[CFG_MEMU_WPORTS];
    sc_signal<TWord> snoop_adr;
    sc_signal<bool> snoop_stb[CFG_MEMU_WPORTS];

    // Methods...
    void InitSubmodules ();
    void FreeSubmodules ();
    void InitInterconnectMethod ();
};


#endif
