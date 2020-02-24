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
#include "config.h"

#include <systemc.h>


/*
   To-Do (TBD)
   -----

   - (EXAMINE: priority: RD/RI/WD per core, not global?  -> starvation may occur if global)
   - (optimize BusIf: reset 'busy' before WB write-back -> does not really help)
*/


struct SCacheTag {
    bool valid, dirty;
    // bool ac_r, ac_w, ac_x;   // access control (for later extension)
    TWord tadr; // requires 32 - CFG_MEMU_CACHE_SETS_LD - CFG_MEMU_CACHE_BANKS_LD - 2 bits
    TWord way; // requires CFG_MEMU_CACHE_WAYS_LD bits

    // Necessary operators for using this structure as signals...
    bool operator== (const SCacheTag &t) const;
    SCacheTag &operator= (const SCacheTag &t);
};

ostream &operator<< (ostream &os, const SCacheTag &t);
void sc_trace (sc_trace_file *tf, const SCacheTag &t, const std::string &name);

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


struct STagEntry {
    bool valid[CFG_MEMU_CACHE_WAYS], dirty[CFG_MEMU_CACHE_WAYS];
    TWord tadr[CFG_MEMU_CACHE_WAYS];
    TWord use; // only used with LRU replacement
};

/*
  // Necessary operators for using this structure as signals...
  bool operator== (const STagEntry& t) const;
  STagEntry& operator= (const STagEntry& t);
};

ostream& operator<< (ostream& os, const STagEntry& t);
void sc_trace (sc_trace_file *tf, const STagEntry& t, const std::string& name);
*/


SC_MODULE (MTagRam) {
public:
    // Ports ...
    sc_in<bool> clk, reset;
    sc_out<bool> ready; // indicates that reset has been completed

    sc_in<bool> rd[TR_PORTS],
    wr[TR_PORTS]; // write accesses require two clock cycles; 'wr', 'adr' and 'tag_in' must be asserted this long

    sc_in<TWord> adr[TR_PORTS]; // complete adress (lower CFG_MEMU_CACHE_BANKS_LD + 2 bits can be omitted)
    sc_in<SCacheTag> tag_in[TR_PORTS];
    // TBD: only the BusIf needs this port, write ports only set the dirty bit -> a 'wr_dirty' signal is sufficient
    sc_out<SCacheTag> tag_out[TR_PORTS];

    // Constructor...
    SC_CTOR (MTagRam) {
        // SC_CTHREAD(MainThread, clk.pos());
        //  reset_signal_is (reset, true);
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
    //   for LRU replacement...
    sc_signal<TWord> use_reg[TR_PORTS], use_iadr_reg[TR_PORTS];
    sc_signal<bool> use_wr_reg[TR_PORTS];
    //   for random replacement...
    sc_signal<TWord> counter;
};


// **************** MBankRam ********************

#define BR_PORTS 2


SC_MODULE (MBankRam) {
public:
    // Ports ...
    sc_in<bool> clk;

    sc_in<bool> rd[BR_PORTS], wr[BR_PORTS];

    sc_in<TWord> wiadr[BR_PORTS]; // line adress (way & index bits of adress)
    sc_in<TWord> wdata[BR_PORTS];
    sc_out<TWord> rdata[BR_PORTS];

    // Constructor...
    SC_CTOR (MBankRam) { SC_CTHREAD (MainThread, clk.pos ()); }

    // Processes...
    void MainThread ();

protected:
    TWord ram_[CFG_MEMU_CACHE_SETS * CFG_MEMU_CACHE_WAYS];
};


// **************** MBusIf **********************

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

typedef sc_uint<3> EBusIfOperation;
#define bioNothing 0 // nothing
#define bioWriteback 1 // write back cache line to main memory
#define bioInvalidate 2 // just invalidate, do not write back
#define bioFlush 3 // write back and invalidate afterwards
#define bioReplace 7 // replace cache line
#define bioDirectRead 5 // direct (uncached) read operation
#define bioDirectWrite 6 // direct (uncached) write operation

#define BUSIF_DATA_REG_NUM (CFG_MEMU_CACHE_BANKS/(CFG_MEMU_BUSIF_WIDTH/32))

SC_MODULE (MBusIf) {
public:
    // Ports ...
    sc_in<bool> clk, reset;

    //   Bus interface (Wishbone)...
    sc_out<bool>        wb_cyc_o;                       // cycle valid output
    sc_out<bool>        wb_stb_o;                       // strobe output
    sc_out<bool>        wb_we_o;                        // indicates write transfer
    sc_out<sc_uint<CFG_MEMU_BUSIF_WIDTH/8> > wb_sel_o;  // byte select outputs
    sc_in<bool>         wb_ack_i;                       // normal termination
    // sc_in<bool>          wb_err_i;                   // termination w/ error
    // sc_in<bool>          wb_rty_i;                   // termination w/ retry
    sc_out<TWord>       wb_adr_o;                       // address bus outputs
    sc_in<sc_uint<CFG_MEMU_BUSIF_WIDTH> > wb_dat_i;     // input data bus
    sc_out<sc_uint<CFG_MEMU_BUSIF_WIDTH> > wb_dat_o;    // output data bus

    //   Control inputs/outputs...
    sc_in<EBusIfOperation> busif_op;    // all operations are atomic & by default acquire a line lock as needed
    sc_in<bool> busif_nolinelock;       // if set, no line lock is acquired (for write misses: lock is already held by write port)
    sc_in<sc_uint<4> > busif_bsel;      // byte select (only for uncached read & write access)
    sc_out<bool> busif_busy;            // if 0, the BusIF is ready for a new operation
                                        // if 0, data writes to the cache (tag & bank) are guaranteed to be completed

    //   Control lines to Tag & Cache banks...
    sc_out<bool> tag_rd, tag_wr, bank_rd[CFG_MEMU_CACHE_BANKS], bank_wr[CFG_MEMU_CACHE_BANKS];

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
    SC_CTOR (MBusIf) {
        SC_METHOD (OutputMethod);
            sensitive << adr_reg << tag_reg;
            for (int n = 0; n < BUSIF_DATA_REG_NUM; n++)
                sensitive << idata_reg[n];
            for (int n = 0; n < CFG_MEMU_CACHE_BANKS; n++)
                sensitive << idata_valid_reg[n];
        SC_CTHREAD (MainThread, clk.pos ());
            reset_signal_is (reset, true);
    }

    // Functions...
    void Trace (sc_trace_file * tf, int level = 1);

    // Processes...
    void OutputMethod ();
    void MainThread ();

protected:
    // Helpers...
    void AcceptNewOp ();

    // Registers...
    sc_signal<EBusIfOperation> op_reg;
    sc_signal<bool> nolinelock_reg;
    sc_signal<sc_uint<4> > bsel_reg;
    sc_signal<sc_uint<32> > adr_reg;
    sc_signal<sc_uint<CFG_MEMU_BUSIF_WIDTH> > idata_reg[BUSIF_DATA_REG_NUM];
    sc_signal<sc_uint<32> > odata_reg[CFG_MEMU_CACHE_BANKS]; // TBD: rename to "rdata...", "wdata..."
    sc_signal<bool> idata_valid_reg[CFG_MEMU_CACHE_BANKS];
    sc_signal<SCacheTag> tag_reg;
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


SC_MODULE (MReadPort) {
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

    // Constructor...
    SC_CTOR (MReadPort) {
        SC_METHOD (HitMethod);
            sensitive << port_adr << busif_adr << tag_in;
            for (int b = 0; b < CFG_MEMU_CACHE_BANKS; b++)
                sensitive << busif_data_valid[b];
        SC_METHOD (TransitionMethod);
            sensitive << clk.pos ();
        SC_METHOD (MainMethod);
            sensitive << reset << state_reg << bank_sel_reg << busif_hit;
            sensitive << port_rd << port_direct << port_adr;
            sensitive << busif_data << busif_busy;
            sensitive << bank_data_in << tag_in;
            sensitive << gnt_tagr << gnt_busif;
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

    // Internal signals...
    sc_signal<bool> busif_hit;
    sc_signal<EReadportState> next_state;
    sc_signal<TWord> next_bank_sel;
};


// **************** MWritePort *******************

typedef enum {
    s_wp_init = 0,
    s_wp_direct = 1,
    s_wp_request_linelock_only = 2,
    s_wp_read_tag = 3,
    s_wp_have_tag_request_bank = 4,
    s_wp_read_bank = 5,
    s_wp_write_tag1_and_bank = 6,
    s_wp_write_tag2_and_bank = 7,
    s_wp_write_tag1 = 8,
    s_wp_write_tag2 = 9,
    s_wp_write_bank = 10,
    s_wp_miss = 11,
    s_wp_request_busif_only = 12,
    s_wp_recheck = 13,
    s_wp_recheck_read_tag = 14,
    s_wp_replace = 15,
    s_wp_replace_wait_busif = 16,
    s_wp_special_request_busif_only = 17,
    s_wp_special = 18,
    s_wp_special_wait_complete = 19
} EWritePortState;


SC_MODULE (MWritePort) {
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

    sc_in<bool> port_lres_scond;    // for LR/SC: reserve current adress (wr = 0) / store only when reservation is valid (wr = 1)
    sc_out<bool> port_scond_ok;     // for LR/SC: set, if store-conditional was sucessful

    sc_in<bool> port_writeback, port_invalidate; // Cache control; writeback + invalidate = flush  (TBD: not implemented yet)

    sc_in<TWord> port_adr;
    sc_in<TWord> port_data;

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
    sc_in<SCacheTag> tag_in;
    sc_out<SCacheTag> tag_out;

    //   Request & grant lines...
    sc_out<bool> req_linelock, req_tagr, req_tagw, req_bank[CFG_MEMU_CACHE_BANKS], req_busif;
    sc_in<bool> gnt_linelock, gnt_tagr, gnt_tagw, gnt_bank[CFG_MEMU_CACHE_BANKS], gnt_busif;

    //   With snoop unit (arbiter)...
    sc_in<TWord> snoop_adr;
    sc_in<bool> snoop_stb;

    // Constructor...
    SC_CTOR (MWritePort) {
        SC_METHOD (TransitionMethod);
            sensitive << clk.pos ();
        SC_METHOD (MainMethod);
            sensitive << reset << state_reg;
            sensitive << port_wr << port_direct << port_bsel << port_lres_scond << port_writeback
                      << port_invalidate << port_adr << port_data;
            sensitive << busif_adr << busif_busy << bank_data_in << tag_in;
            sensitive << gnt_linelock << gnt_tagr << gnt_tagw << gnt_busif << snoop_adr << snoop_stb;
            sensitive << link_valid_reg << link_adr_reg << snoop_stb_reg << snoop_adr_reg;
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

    sc_signal<TWord> link_adr_reg;
    sc_signal<bool> link_valid_reg;

    sc_signal<bool> snoop_stb_reg;
    sc_signal<TWord> snoop_adr_reg;

    // Internal signals..
    sc_signal<EWritePortState> next_state;
    sc_signal<SCacheTag> next_tag_reg;
    sc_signal<TWord> next_data_reg;
    sc_signal<TWord> next_link_adr_reg;
    sc_signal<bool> next_link_valid_reg;
};


// **************** MArbiter ********************


SC_MODULE (MArbiter) {
public:
    // Ports ...
    sc_in<bool> clk, reset;

    sc_in<TWord> wiadr_busif, wiadr_rp[RPORTS], adr_wp[WPORTS],
        way_wp[WPORTS]; // (way+index) adresses from various ports;
                        // must be kept constant as long as 'req_*' lines are held
    sc_out<TWord> wiadr_bank[CFG_MEMU_CACHE_BANKS][BR_PORTS]; // adress lines to the banks

    //   Write snooping...
    sc_out<TWord> snoop_adr;
    sc_out<bool> snoop_stb[WPORTS];

    //   Line lock...
    //   - Implements mutex for a single cache line (tag + all data banks).
    //   - Writers are supposed to acquire a line lock.
    //   - Reader do not acquire anything => Writers must perform their actions in a safe order.
    //   - if a line lock request is held, the associated address must not change
    //   - A line lock may be released during the clock cycle that the last bank/tag access is made,
    //     given that the bank/tag lock is still held. This allows faster write cycles without write
    //     port monopolizing a line lock (keeping it permanently up).
    sc_in<bool> req_busif_linelock, req_wp_linelock[WPORTS];
    sc_out<bool> gnt_busif_linelock, gnt_wp_linelock[WPORTS];

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
    sc_in<bool> req_busif_tagw, req_wp_tagw[WPORTS], req_busif_tagr, req_wp_tagr[WPORTS], req_rp_tagr[RPORTS];
    sc_out<bool> gnt_busif_tagw, gnt_wp_tagw[WPORTS], gnt_busif_tagr, gnt_wp_tagr[WPORTS], gnt_rp_tagr[RPORTS];

    //   Bank RAMs...
    //   - All ports of CPU n must be linked to port n % BR_PORTS of each bank RAM.
    //   - The BUSIF is linked to port #(BR_PORTS-1).
    //   - Multiple usually conflicting grant signals may be set, if the adresses match.
    //   - As long as a request signal is set, the adress must not change!
    //   - Amoung write ports and the BUSIF, only one grant will be given to avoid writing conflicts.
    sc_in<bool> req_busif_bank[CFG_MEMU_CACHE_BANKS], req_wp_bank[WPORTS][CFG_MEMU_CACHE_BANKS],
        req_rp_bank[RPORTS][CFG_MEMU_CACHE_BANKS];
    sc_out<bool> gnt_busif_bank[CFG_MEMU_CACHE_BANKS], gnt_wp_bank[WPORTS][CFG_MEMU_CACHE_BANKS],
        gnt_rp_bank[RPORTS][CFG_MEMU_CACHE_BANKS];

    //   BUSIF...
    sc_in<bool> req_rp_busif[RPORTS], req_wp_busif[WPORTS];
    sc_out<bool> gnt_rp_busif[RPORTS], gnt_wp_busif[WPORTS];
    // select/routing information can be derived from grant lines (only one is set at a time)

    // Note on deadlock prevention:
    // 1. Requests/grants must always in the following order (-> break cyclic wait condition):
    //      busif < linelock < tagr/tagw < bank
    // 2. (corollary to 1.) R/W ports must not request anything when waiting for BusIF (i.e. 'busif_busy')
    // 3. tag/bank access may never be requested in a hold-and-wait manner: Either request simultaneously or use & complete serially.

    // Constructor...
    SC_CTOR (MArbiter) {
        int n, b;

        SC_METHOD (LineLockMethod);
            sensitive << req_busif_linelock << wiadr_busif;
            for (n = 0; n < WPORTS; n++)
                sensitive << req_wp_linelock[n] << adr_wp[n];
        SC_METHOD (TagMethod);
            sensitive << tagram_ready << req_busif_tagr << req_busif_tagw;
            for (n = 0; n < RPORTS; n++)
                sensitive << req_rp_tagr[n];
            for (n = 0; n < WPORTS; n++)
                sensitive << req_wp_tagr[n] << req_wp_tagw[n];
        SC_METHOD (BankMethod);
            sensitive << wiadr_busif;
            for (n = 0; n < RPORTS; n++)
                sensitive << wiadr_rp[n];
            for (n = 0; n < WPORTS; n++)
                sensitive << adr_wp[n] << way_wp[n];
            for (b = 0; b < CFG_MEMU_CACHE_BANKS; b++) {
                sensitive << req_busif_bank[b];
                for (n = 0; n < RPORTS; n++)
                    sensitive << req_rp_bank[n][b];
                for (n = 0; n < WPORTS; n++)
                    sensitive << req_wp_bank[n][b];
            }
        SC_METHOD (BusIfMethod);
            for (n = 0; n < RPORTS; n++)
                sensitive << req_rp_busif[n];
            for (n = 0; n < WPORTS; n++)
                sensitive << req_wp_busif[n];
        SC_METHOD (SnoopMethod);
            sensitive << next_linelock_reg;
            for (n = 0; n < WPORTS; n++)
                sensitive << adr_wp[n];

        SC_CTHREAD (TransitionThread, clk.pos ());
            reset_signal_is (reset, true);
    }

    // Functions...
    void Trace (sc_trace_file * tf, int level = 1);

    // Processes...
    void LineLockMethod ();
    void TagMethod ();
    void BankMethod ();
    void BusIfMethod ();
    void SnoopMethod ();

    void TransitionThread ();

    protected:
    // Helpers...
    int GetPrioCpu ();

    // Registers...
    sc_signal<TWord> counter_reg;
    sc_signal<sc_uint<WPORTS + 1> > linelock_reg;
    sc_signal<sc_uint<RPORTS + WPORTS + 1> > tagr_reg;
    sc_signal<sc_uint<WPORTS + 1> > tagw_reg;
    sc_signal<sc_uint<RPORTS + WPORTS + 1> > bank_reg[CFG_MEMU_CACHE_BANKS];
    sc_signal<sc_uint<RPORTS + WPORTS> > busif_reg;

    // Signals...
    sc_signal<sc_uint<WPORTS + 1> > next_linelock_reg;
    sc_signal<sc_uint<RPORTS + WPORTS + 1> > next_tagr_reg;
    sc_signal<sc_uint<WPORTS + 1> > next_tagw_reg;
    sc_signal<sc_uint<RPORTS + WPORTS + 1> > next_bank_reg[CFG_MEMU_CACHE_BANKS];
    sc_signal<sc_uint<RPORTS + WPORTS> > next_busif_reg;
};


// **************** MMemu ***********************

SC_MODULE (MMemu) {
public:
    // Ports ...
    sc_in<bool> clk, reset;

    //   Bus interface (Wishbone)...
    sc_out<bool>        wb_cyc_o; // cycle valid output
    sc_out<bool>        wb_stb_o; // strobe output
    sc_out<bool>        wb_we_o; // indicates write transfer
    sc_out<sc_uint<CFG_MEMU_BUSIF_WIDTH/8> > wb_sel_o; // byte select outputs
    sc_in<bool>         wb_ack_i; // normal termination
    // sc_in<bool>          wb_err_i;     // termination w/ error
    // sc_in<bool>          wb_rty_i;     // termination w/ retry
    sc_out<TWord>       wb_adr_o; // address bus outputs
    sc_in<sc_uint<CFG_MEMU_BUSIF_WIDTH> >        wb_dat_i; // input data bus
    sc_out<sc_uint<CFG_MEMU_BUSIF_WIDTH> >       wb_dat_o; // output data bus

    //   Read ports...
    //     ports 0 .. WPORT-1 are considerd to be data ports, the others to be instruction ports (with lower priority)
    sc_in<bool> rp_rd[RPORTS], rp_direct[RPORTS];
    sc_in<sc_uint<4> > rp_bsel[RPORTS];
    sc_out<bool> rp_ack[RPORTS];
    sc_in<TWord> rp_adr[RPORTS];
    sc_out<TWord> rp_data[RPORTS];

    //   Write ports...
    sc_in<bool> wp_wr[WPORTS], wp_direct[WPORTS];
    sc_in<sc_uint<4> > wp_bsel[WPORTS];
    sc_out<bool> wp_ack[WPORTS];
    sc_in<bool> wp_lres_scond[WPORTS];
    sc_out<bool> wp_scond_ok[WPORTS];
    sc_in<bool> wp_writeback[WPORTS], wp_invalidate[WPORTS];
    sc_in<TWord> wp_adr[WPORTS];
    sc_in<TWord> wp_data[WPORTS];

    // Constructor/Destructor...
    SC_CTOR (MMemu) {
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
    MReadPort *readPorts[RPORTS];
    MWritePort *writePorts[WPORTS];

protected:
    // Tag RAM...
    sc_signal<bool> tagram_ready, tagram_rd[TR_PORTS], tagram_wr[TR_PORTS];
    sc_signal<TWord> tagram_adr[TR_PORTS];
    sc_signal<SCacheTag> tagram_tag_in[TR_PORTS], tagram_tag_out[TR_PORTS];

    // Bank RAM...
    sc_signal<bool> bankram_rd[CFG_MEMU_CACHE_BANKS][BR_PORTS], bankram_wr[CFG_MEMU_CACHE_BANKS][BR_PORTS];
    sc_signal<TWord> bankram_wiadr[CFG_MEMU_CACHE_BANKS][BR_PORTS];
    sc_signal<TWord> bankram_wdata[CFG_MEMU_CACHE_BANKS][BR_PORTS];
    sc_signal<TWord> bankram_rdata[CFG_MEMU_CACHE_BANKS][BR_PORTS];

    // BUSIF...
    sc_signal<EBusIfOperation> busif_op;
    sc_signal<bool> busif_nolinelock, busif_busy;
    sc_signal<bool> busif_tag_rd, busif_tag_wr, busif_bank_rd[CFG_MEMU_CACHE_BANKS],
    busif_bank_wr[CFG_MEMU_CACHE_BANKS];
    sc_signal<TWord> busif_adr_in, busif_adr_out, busif_data_in[CFG_MEMU_CACHE_BANKS],
    busif_data_out[CFG_MEMU_CACHE_BANKS];
    sc_signal<bool> busif_data_out_valid[CFG_MEMU_CACHE_BANKS];
    sc_signal<SCacheTag> busif_tag_in, busif_tag_out;
    sc_signal<sc_uint<4> > busif_bsel;

    // Read ports...
    sc_signal<TWord> rp_busif_data_reg[CFG_MEMU_BUSIF_WIDTH/32]; // register to delay BusIF data for one clock cycle in accordance with the protocol
    sc_signal<TWord> rp_busif_data[RPORTS];
    sc_signal<EBusIfOperation> rp_busif_op[RPORTS];
    sc_signal<bool> rp_tag_rd[RPORTS], rp_bank_rd[RPORTS];
    sc_signal<SCacheTag> rp_tag_in[RPORTS];
    sc_signal<TWord> rp_way_out[RPORTS];
    sc_signal<TWord> rp_bank_data_in[RPORTS];
    sc_signal<TWord> rp_bank_sel[RPORTS];

    // Write ports...
    sc_signal<EBusIfOperation> wp_busif_op[WPORTS];
    sc_signal<bool> wp_busif_nolinelock[WPORTS];
    sc_signal<bool> wp_tag_rd[WPORTS], wp_tag_wr[WPORTS], wp_bank_rd[WPORTS], wp_bank_wr[WPORTS];
    sc_signal<SCacheTag> wp_tag_in[WPORTS], wp_tag_out[WPORTS];
    sc_signal<TWord> wp_bank_data_in[WPORTS], wp_bank_data_out[WPORTS];

    // Arbiter: request/grant signals (find comments in 'MArbiter')...
    sc_signal<bool> req_busif_linelock, req_wp_linelock[WPORTS];
    sc_signal<bool> gnt_busif_linelock, gnt_wp_linelock[WPORTS];

    sc_signal<bool> req_busif_tagw, req_wp_tagw[WPORTS], req_busif_tagr, req_wp_tagr[WPORTS],
        req_rp_tagr[RPORTS];
    sc_signal<bool> gnt_busif_tagw, gnt_wp_tagw[WPORTS], gnt_busif_tagr, gnt_wp_tagr[WPORTS],
        gnt_rp_tagr[RPORTS];

    sc_signal<bool> req_busif_bank[CFG_MEMU_CACHE_BANKS], req_wp_bank[WPORTS][CFG_MEMU_CACHE_BANKS],
    req_rp_bank[RPORTS][CFG_MEMU_CACHE_BANKS];
    sc_signal<bool> gnt_busif_bank[CFG_MEMU_CACHE_BANKS], gnt_wp_bank[WPORTS][CFG_MEMU_CACHE_BANKS],
        gnt_rp_bank[RPORTS][CFG_MEMU_CACHE_BANKS];

    sc_signal<bool> req_rp_busif[RPORTS], gnt_rp_busif[RPORTS];
    sc_signal<bool> req_wp_busif[WPORTS], gnt_wp_busif[WPORTS];

    // Arbiter: other signals ...
    sc_signal<TWord> wiadr_busif, wiadr_rp[RPORTS], adr_wp[WPORTS], way_wp[WPORTS];
    sc_signal<TWord> snoop_adr;
    sc_signal<bool> snoop_stb[WPORTS];

    // Methods...
    void InitSubmodules ();
    void FreeSubmodules ();
    void InitInterconnectMethod ();
};


#endif
