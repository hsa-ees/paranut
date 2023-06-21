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
#ifndef _MEMU_BUFSIF_
#define _MEMU_BUFSIF_

#include "memu_common.h"
// **************** MBusController **********************

// defines the multiplexing behaviour of MBusController
// can currently only be 2, 0: busif; 1: ptw
#define MASTER_NO 2 

SC_MODULE(MBusController) {
public:
    //   Bus interface (Wishbone)...
    sc_out<bool>        wb_cyc_o{"wb_cyc_o"};                       // cycle valid output
    sc_out<bool>        wb_stb_o{"wb_stb_o"};                      // strobe output
    sc_out<bool>        wb_we_o{"wb_we_o"};                        // indicates write transfer
    sc_out<sc_uint<3> > wb_cti_o{"wb_cti_o"};                       // cycle type identifier
    sc_out<sc_uint<2> > wb_bte_o{"wb_bte_o"};                       // burst type extension
    sc_out<sc_uint<CFG_MEMU_BUSIF_WIDTH/8> > wb_sel_o{"wb_sel_o"};  // byte select outputs
    sc_out<sc_uint<32> > wb_adr_o{"wb_adr_o"};                      // address bus outputs
    sc_out<sc_uint<CFG_MEMU_BUSIF_WIDTH> > wb_dat_o{"wb_dat_o"};    // output data bus

    sc_vector<sc_in<bool> >     master_cyc{"master_cyc", MASTER_NO};
    sc_vector<sc_in<bool> >       master_stb{"master_stb", MASTER_NO};
    sc_vector<sc_in<bool> >       master_we{"master_we", MASTER_NO};
    sc_vector<sc_in<sc_uint<3> > > master_cti{"master_cti", MASTER_NO};
    sc_vector<sc_in<sc_uint<2> > > master_bte{"master_bte", MASTER_NO};
    sc_vector<sc_in<sc_uint<CFG_MEMU_BUSIF_WIDTH/8> > > master_sel{"master_sel", MASTER_NO};
    sc_vector<sc_in<sc_uint<32> > > master_adr{"master_adr", MASTER_NO};
    sc_vector<sc_in<sc_uint<CFG_MEMU_BUSIF_WIDTH> > > master_dat{"master_dat", MASTER_NO};

    //   Control inputs...
    sc_in<bool> switch_master{"switch_master"};


    // Constructor...
    SC_CTOR (MBusController) {
        SC_METHOD (proc_cmb_mbusc);
            for (unsigned int m = 0; m < MASTER_NO; m++) {
                sensitive << master_cyc[m] << master_stb[m] << master_we[m] 
                          << master_cti[m] << master_bte[m] << master_sel[m] 
                          << master_adr[m] << master_dat[m];
            }
            sensitive << switch_master;
    };

    // Functions...
    void Trace (sc_trace_file * tf, int level = 1);

protected:
    // Processes...
    void proc_cmb_mbusc ();
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
    BifCacheFillIdataPageFault,
} EBusIfState;

typedef enum {
    BifMmuIdle = 0,
    BifMmuReq,
    BifMmuAwaitResponse,
    BifMmuLoadPageEntry,
    BifMmuPteSuccess,
    BifMmuDone,
} EBusIfMmuState;

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

#define BUSIF_DATA_REG_NUM (CFG_MEMU_CACHE_BANKS/(CFG_MEMU_BUSIF_WIDTH/32U))
#define BUSIF_DATA_REG_NUM_LD (CFG_MEMU_CACHE_BANKS_LD-(CFG_MEMU_BUSIF_WIDTH/64U))

struct SBusIfRegs {
    EBusIfState                         state;
    EBusIfOperation                     op;
    bool                                linelock;
    sc_uint<4>                          bsel;
    sc_uint<32>                         phys_adr;
    sc_uint<32>                         virt_adr;
    sc_uint<CFG_MEMU_CACHE_BANKS_LD>    adr_ofs;
    bool                                idata_valid[CFG_MEMU_CACHE_BANKS];
    sc_uint<CFG_MEMU_BUSIF_WIDTH>       idata[BUSIF_DATA_REG_NUM];
    sc_uint<32>                         odata[CFG_MEMU_CACHE_BANKS]; // TBD: rename to "rdata...", "wdata..."
    sc_uint<BUSIF_DATA_REG_NUM_LD>      cnt;
    sc_uint<CFG_MEMU_CACHE_BANKS>       banks_left;
    sc_uint<CFG_MEMU_CACHE_BANKS>       last_banks_left;
   
    // tag
    bool tag_valid;
    bool tag_dirty;
    bool tag_ac_r;
    bool tag_ac_w;
    bool tag_ac_x;
    bool tag_ac_u; // Paging flags
    sc_uint<32-CFG_MEMU_CACHE_SETS_LD-CFG_MEMU_CACHE_BANKS_LD-2>  tag_tadr;
    sc_uint<CFG_MEMU_CACHE_WAYS_LD>  tag_way;


    // MMU related registers
    bool                                ac_r;
    bool                                ac_w;
    bool                                ac_x;
    bool                                ac_u;
    bool                                transl_done;
    bool                                paging_en;
    
    // Necessary operators for using this structure as signals...
    // Compare (necessary for signal updates)...
    bool operator== (const SBusIfRegs &t){
        return(state == t.state && op == t.op && linelock == t.linelock
        && bsel == t.bsel && phys_adr == t.phys_adr && virt_adr == t.virt_adr
        && adr_ofs == t.adr_ofs && cnt == t.cnt && banks_left == t.banks_left
        && last_banks_left == t.last_banks_left && ac_r == t.ac_r
        && ac_w == t.ac_w && ac_x == t.ac_x && ac_u == t.ac_u && transl_done == t.transl_done
        && paging_en == t.paging_en && idata_valid == t.idata_valid && idata == t.idata
        && odata == t.odata && tag_valid == t.tag_valid && tag_dirty == t.tag_dirty
        && tag_ac_r == t.tag_ac_r && tag_ac_w == t.tag_ac_w && tag_ac_x == t.tag_ac_x
        && tag_ac_u == t.tag_ac_u && tag_tadr == t.tag_tadr && tag_way == t.tag_way);
    };

    // Display...
    friend ostream& operator<< ( ostream& os, const SBusIfRegs &t ){
        os << "{" << t.state << t.op << "}";
        return os;
    };

    // Tracing...
    friend void sc_trace(sc_trace_file *tf, const SBusIfRegs &t, const std::string &name){
        PN_TRACE_R(tf, t, state, name);
        PN_TRACE_R(tf, t, op, name);
        PN_TRACE_R(tf, t, linelock, name);
        PN_TRACE_R(tf, t, bsel, name);
        PN_TRACE_R(tf, t, phys_adr, name);
        PN_TRACE_R(tf, t, virt_adr, name);
        PN_TRACE_R(tf, t, adr_ofs, name);
        PN_TRACE_R_BUS(tf, t, idata_valid, name, CFG_MEMU_CACHE_BANKS);
        PN_TRACE_R_BUS(tf, t, idata, name, BUSIF_DATA_REG_NUM);
        PN_TRACE_R_BUS(tf, t, odata, name, CFG_MEMU_CACHE_BANKS);
        PN_TRACE_R(tf, t, cnt, name);
        PN_TRACE_R(tf, t, banks_left, name);
        PN_TRACE_R(tf, t, last_banks_left, name);
        PN_TRACE_R(tf, t, ac_r, name);
        PN_TRACE_R(tf, t, ac_w, name);
        PN_TRACE_R(tf, t, ac_x, name);
        PN_TRACE_R(tf, t, ac_u, name)
        PN_TRACE_R(tf, t, transl_done, name);
        PN_TRACE_R(tf, t, paging_en, name)

        // Trace Tag:
        PN_TRACE_R(tf, t, tag_valid, name);
        PN_TRACE_R(tf, t, tag_dirty, name);
        PN_TRACE_R(tf, t, tag_ac_r, name);
        PN_TRACE_R(tf, t, tag_ac_w, name);
        PN_TRACE_R(tf, t, tag_ac_x, name);
        PN_TRACE_R(tf, t, tag_ac_u, name);
        PN_TRACE_R(tf, t, tag_tadr, name)
        PN_TRACE_R(tf, t, tag_way, name);

    };

    // icsc does not support structs in structs.
    // this is a workaround.
    void setTag(SCacheTag tag){
        tag_valid = tag.valid;
        tag_dirty = tag.dirty;
        tag_ac_r = tag.ac_r;
        tag_ac_w = tag.ac_w;
        tag_ac_x = tag.ac_x;
        tag_ac_u = tag.ac_u;
        tag_tadr = tag.tadr;
        tag_way = tag.way;
    }

    SCacheTag getTag() const{
        SCacheTag tag;
        tag.valid = tag_valid;
        tag.dirty = tag_dirty;
        tag.ac_r = tag_ac_r;
        tag.ac_w = tag_ac_w;
        tag.ac_x = tag_ac_x;
        tag.ac_u = tag_ac_u;
        tag.tadr = tag_tadr;
        tag.way = tag_way;
        return tag;
    }


 };


struct SBusIfMmuRegs {
    EBusIfMmuState state;
    sc_uint<32> adr;
    bool ack;
    bool ac_r;
    bool ac_w;
    bool ac_x;
    bool ac_u;
    bool ac_d;
    bool ac_a;
    // Necessary operators for using this structure as signals...
    // Compare (necessary for signal updates)...
    bool operator == (const SBusIfMmuRegs &t);
    // Display...
    friend ostream& operator << ( ostream& o, const SBusIfMmuRegs &t );
    // Tracing...
    friend void sc_trace(sc_trace_file *tf, const SBusIfMmuRegs &t, const std::string &name);
};

class MBusIf : ::sc_core::sc_module {
public:
    // Ports ...
    sc_in<bool> clk, reset;

    //   Bus interface (Wishbone)...
    sc_out<bool>        wb_cyc_o{"wb_cyc_o"};                       // cycle valid output
    sc_out<bool>        wb_stb_o{"wb_stb_o"};                       // strobe output
    sc_out<bool>        wb_we_o{"wb_we_o"};                        // indicates write transfer
    sc_out<sc_uint<3> > wb_cti_o{"wb_cti_o"};                       // cycle type identifier
    sc_out<sc_uint<2> > wb_bte_o{"wb_bte_o"};                       // burst type extension
    sc_out<sc_uint<CFG_MEMU_BUSIF_WIDTH/8> > wb_sel_o{"wb_sel_o"};  // byte select outputs
    sc_out<sc_uint<32> > wb_adr_o{"wb_adr_o"};                      // address bus outputs
    sc_out<sc_uint<CFG_MEMU_BUSIF_WIDTH> > wb_dat_o{"wb_dat_o"};    // output data bus

    sc_in<bool>         wb_ack_i{"wb_ack_i"};                       // normal termination
    // sc_in<bool>          wb_err_i;                   // termination w/ error
    // sc_in<bool>          wb_rty_i;                   // termination w/ retry
    sc_in<sc_uint<CFG_MEMU_BUSIF_WIDTH> > wb_dat_i{"wb_dat_i"};     // input data bus

    //   Control inputs/outputs...
    sc_in<EBusIfOperation> busif_op{"busif_op"};    // all operations are atomic & by default acquire a line lock as needed
    sc_in<bool> busif_nolinelock{"busif_nolinelock"};       // if set, no line lock is acquired (for write misses: lock is already held by write port)
    sc_in<sc_uint<4> > busif_bsel{"busif_bsel"};      // byte select (only for uncached read & write access)
    sc_out<bool> busif_busy{"busif_busy"};            // if 0, the BusIF is ready for a new operation
                                        // if 0, data writes to the cache (tag & bank) are guaranteed to be completed

    //   Control lines to Tag & Cache banks...
    sc_out<bool> tag_rd{"tag_rd"};
    sc_out<bool> tag_rd_way{"tag_rd_way"};
    sc_out<bool> tag_wr{"tag_wr"};
    sc_vector<sc_out<bool> > bank_rd{"bank_rd", CFG_MEMU_CACHE_BANKS}; 
    sc_vector<sc_out<bool> > bank_wr{"bank_wr", CFG_MEMU_CACHE_BANKS};

    //   Adress & data busses...
    sc_in<sc_uint<32> > adr_in{"adr_in"};
    sc_out<sc_uint<32> > adr_out{"adr_out"}; // to cache banks, tag bank, and r/w ports to determine BUSIF hits
                           // Note: If data width is set to 64Bit bit the 3rd bit of adr_out will be
                           // fixed to 0, to ensure correct hit detection in RPs
    sc_vector<sc_in<sc_uint<32> > > data_in{"data_in", CFG_MEMU_CACHE_BANKS};     // data from cache banks (bank_rd[n] = 1) or write ports (else)...
    sc_vector<sc_out<sc_uint<32> > > data_out{"data_out", CFG_MEMU_CACHE_BANKS};   // data read from bus
    sc_vector<sc_out<bool> > data_out_valid{"data_out_valid", CFG_MEMU_CACHE_BANKS};
    sc_in<SCacheTag> tag_in{"tag_in"};
    sc_out<SCacheTag> tag_out{"tag_out"};

    // Permission lines
    sc_out<bool> ac_r_out{"ac_r_out"};
    sc_out<bool> ac_w_out{"ac_w_out"};
    sc_out<bool> ac_x_out{"ac_x_out"};
    sc_out<bool> ac_u_out{"ac_u_out"};

    sc_in<bool> trap_u{"trap_u"};
    sc_in<bool> trap_no_u{"trap_no_u"};

    //   Request & grant lines...
    sc_out<bool> req_linelock{"req_linelock"};
    sc_out<bool> req_tagw{"req_tagw"};
    sc_out<bool> req_tagr{"req_tagr"};
    sc_vector<sc_out<bool> > req_bank{"req_bank", CFG_MEMU_CACHE_BANKS};
    sc_in<bool> gnt_linelock{"gnt_linelock"};
    sc_in<bool> gnt_tagw{"gnt_tagw"};
    sc_in<bool> gnt_tagr{"gnt_tagr"};
    sc_vector<sc_in<bool> > gnt_bank{"gnt_bank", CFG_MEMU_CACHE_BANKS};

    // Constructor...
    SC_HAS_PROCESS (MBusIf);
    MBusIf (sc_module_name name)
        : sc_module (name) {
        SC_CTHREAD (proc_clk_busif, clk.pos());
        reset_signal_is(reset, true);

        SC_METHOD (proc_cmb_busif);
            sensitive << regs;
            sensitive << ptw_ack << ptw_phys_adr << ptw_ac_a << ptw_ac_d << ptw_ac_r << ptw_ac_w << ptw_ac_x << ptw_ac_u;
            sensitive << trap_u << trap_no_u;
            sensitive << busif_op << busif_nolinelock << busif_bsel << adr_in << tag_in;
            sensitive << gnt_linelock << gnt_tagw << gnt_tagr;
            sensitive << wb_ack_i << wb_dat_i;
            sensitive << paging_mode << ptw_virt_adr;
            for (uint n = 0; n < CFG_MEMU_CACHE_BANKS; n++)
                sensitive << data_in[n] << gnt_bank[n];
    }

    // To PTW
    sc_out<sc_uint<32> > ptw_virt_adr; // Translated Physical Address
    sc_in<sc_uint<32> > ptw_phys_adr; // input data from bus interface

    sc_out<bool> ptw_req;  // Start translation
    sc_in<bool> ptw_ack; // Address translation successful

    sc_in<bool> ptw_ac_r;
    sc_in<bool> ptw_ac_w;
    sc_in<bool> ptw_ac_x;
    sc_in<bool> ptw_ac_u;
    sc_in<bool> ptw_ac_d;
    sc_in<bool> ptw_ac_a;

    sc_in<bool> paging_mode;

    // Functions...
    void Trace (sc_trace_file * tf, int level = 1);

    // Processes...
    void proc_clk_busif ();
    void proc_cmb_busif ();
protected:
    // Helpers...
    void AcceptNewOp ();

    // BusIf Registers...
    sc_signal<SBusIfRegs> regs;

    // Internal BusIf Signals...
    sc_signal<SBusIfRegs> next_regs;

    // Special signal for simulation. Only required because the other state signals display non-sense.
    sc_signal<TWord> state_trace;
};
#endif