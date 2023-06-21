/*************************************************************************

  This file is part of the ParaNut project.

  Copyright (C) 2010-2022 Alexander Bahle <alexander.bahle@hs-augsburg.de>
                          Gundolf Kiefer <gundolf.kiefer@hs-augsburg.de>
                          Christian H. Meyer <christian.meyer@hs-augsburg.de>
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

#include "memu_arbiter.h"
#include "memu_busif.h"
#include "memu_cache.h"
#include "memu_common.h"
#include "memu_readport.h"
#include "memu_writeport.h"

#include <systemc.h>
#include "ptw.h"
#include "tlb.h"


// **************** MMemu ***********************

class MMemu : ::sc_core::sc_module {
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


    //   Read ports...
    //     ports 0 .. WPORT-1 are considerd to be data ports, the others to be instruction ports (with lower priority)
    sc_vector<sc_in<bool> > rp_rd{"rp_rd", CFG_MEMU_RPORTS};                 // read request
    sc_vector<sc_in<sc_uint<4> > > rp_bsel{"rp_bsel", CFG_MEMU_RPORTS};        // byte select
    sc_vector<sc_out<bool> > rp_ack{"rp_ack", CFG_MEMU_RPORTS};
    sc_vector<sc_in<sc_uint<32> > > rp_adr{"rp_adr", CFG_MEMU_RPORTS};
    sc_vector<sc_out<sc_uint<32> > > rp_data{"rp_data", CFG_MEMU_RPORTS};
    sc_vector<sc_in<bool> > rp_direct{"rp_direct", CFG_MEMU_RPORTS};             // direct access (without cache, ordered/without delay)
    sc_vector<sc_in<bool> > rp_paging{"rp_paging", CFG_MEMU_RPORTS};             // (MMU) ??
    sc_vector<sc_out<bool> > rp_ac_r{"rp_ac_r", CFG_MEMU_RPORTS};              // (MMU) access control ??
    sc_vector<sc_out<bool> > rp_ac_x{"rp_ac_x", CFG_MEMU_RPORTS};              // (MMU) access control ??
    sc_vector<sc_out<bool> > rp_ac_u{"rp_ac_u", CFG_MEMU_RPORTS};              // (MMU) access control ??

    //   Write ports...
    sc_vector<sc_in<bool> > wp_wr{"wp_wr", CFG_MEMU_WPORTS};                 // write request
    sc_vector<sc_in<sc_uint<4> > > wp_bsel{"wp_bsel", CFG_MEMU_WPORTS};        // byte select
    sc_vector<sc_out<bool> > wp_ack{"wp_ack", CFG_MEMU_WPORTS};               // acknowledge
    sc_vector<sc_in<sc_uint<32> > > wp_adr{"wp_adr", CFG_MEMU_WPORTS};        // adress
    sc_vector<sc_in<sc_uint<32> > > wp_data{"wp_data", CFG_MEMU_WPORTS};       // data
    sc_vector<sc_in<bool> > wp_direct{"wp_direct", CFG_MEMU_WPORTS};             // direct access
    sc_vector<sc_in<bool> > wp_lres_scond{"wp_lres_scond", CFG_MEMU_WPORTS};         // load reserved / store conditional (?? notification for a "load reserved" on the given address ??)
    sc_vector<sc_out<bool> > wp_scond_ok{"wp_scond_ok", CFG_MEMU_WPORTS};          // indicates whether a "store conditional" was actually executed
    sc_vector<sc_in<sc_uint<3> > > wp_cache_op{"wp_cache_op", CFG_MEMU_WPORTS};    // cache operation
    sc_vector<sc_in<bool> > wp_paging{"wp_paging", CFG_MEMU_WPORTS};             // (MMU) TBD
    sc_vector<sc_out<bool> > wp_ac_w{"wp_ac_w", CFG_MEMU_WPORTS};              // (MMU) TBD
    sc_vector<sc_in<bool> > wp_trap_u{"wp_trap_u", CFG_MEMU_WPORTS};             // (MMU) TBD
    sc_vector<sc_in<bool> > wp_trap_no_u{"wp_trap_no_u", CFG_MEMU_WPORTS};          // (MMU) TBD

    sc_in<bool>         wb_ack_i{"wb_ack_i"};                       // normal termination
    // sc_in<bool>          wb_err_i;                   // termination w/ error
    // sc_in<bool>          wb_rty_i;                   // termination w/ retry
    sc_in<sc_uint<CFG_MEMU_BUSIF_WIDTH> > wb_dat_i{"wb_dat_i"};     // input data bus

    // MMU related ports...
    sc_in<sc_uint<20> > root_ppn{"root_ppn"};                       // TBD: Rename to 'mmu_...' (??)
    sc_in<bool> tlb_flush{"tlb_flush"};          

    // Constructor/Destructor...
    SC_HAS_PROCESS (MMemu);
    MMemu (sc_module_name name)
        : sc_module (name) {

        InitSubmodules ();

        SC_CTHREAD (proc_clk_memu, clk.pos ());
            reset_signal_is(reset, true);

         SC_METHOD (proc_cmb_interconnect);
            // Tag RAM...
            for (uint n = 0; n < TR_PORTS; n++) sensitive << tagram_tag_out[n] << tagram_rd[n] << tagram_wr[n];
             // Bank RAM...
            for (uint n = 0; n < CFG_MEMU_CACHE_BANKS; n++)
                for (uint k = 0; k < CFG_MEMU_BANK_RAM_PORTS; k++) sensitive << bankram_rdata[n][k];

            // BUSIF...
            sensitive << busif_busy << busif_tag_rd << busif_tag_wr << busif_adr_out << busif_tag_out;
            for (uint n = 0; n < CFG_MEMU_CACHE_BANKS; n++)
                sensitive << busif_bank_rd[n] << busif_bank_wr[n] << busif_data_out[n] << busif_data_out_valid[n];
            // Read ports...
            for (uint n = 0; n < CFG_MEMU_RPORTS; n++)
                sensitive << rp_busif_op[n] << rp_tag_rd[n] << rp_bank_rd[n] << rp_adr[n] << rp_way_out[n]
                            << rp_bank_sel[n] << rp_paging[n] << rp_bsel[n];
            // Write ports...
            for (uint n = 0; n < CFG_MEMU_WPORTS; n++)
                sensitive << wp_busif_op[n] << wp_busif_nolinelock[n] << wp_tag_rd[n] << wp_tag_wr[n]
                            << wp_bank_rd[n] << wp_bank_wr[n] << wp_tag_out[n] << wp_bank_data_out[n]
                            << wp_trap_u[n] << wp_trap_no_u[n] << wp_paging[n]
                            << wp_bank_bsel[n] << wp_adr[n] << wp_data[n] << wp_bsel[n];
            // Arbiter: request/grant signals ...
            sensitive << gnt_busif_linelock << gnt_busif_tagw << gnt_busif_tagr;
            for (uint n = 0; n < CFG_MEMU_RPORTS; n++){
                sensitive << gnt_rp_tagr[n] << gnt_rp_busif[n];
                for (uint k = 0; k < CFG_MEMU_CACHE_BANKS; k++)
                    sensitive << gnt_rp_bank[n][k];
            } 
            for (uint n = 0; n < CFG_MEMU_WPORTS; n++) {
                sensitive << gnt_wp_linelock[n] << gnt_wp_tagw[n] << gnt_wp_tagr[n] << gnt_wp_busif[n];
                for (uint k = 0; k < CFG_MEMU_CACHE_BANKS; k++)
                    sensitive << gnt_wp_bank[n][k];
            }
            for (uint n = 0; n < CFG_MEMU_CACHE_BANKS; n++) {
                sensitive << gnt_busif_bank[n];
                for (uint k = 0; k < CFG_MEMU_BANK_RAM_PORTS; k++) sensitive << bankram_wiadr[n][k];
            }
            // Internal registers...
            for (uint n = 0; n < CFG_MEMU_BUSIF_WIDTH/32; n++)
                sensitive << rp_busif_data_reg[n];
    }
    ~MMemu () { FreeSubmodules (); }

    // Submodules..
    MTagRam *tagRam;
    MBankRam *bankRam;
    MBusController *busController;
    MBusIf *busIf;
    MPtw *ptw;
    MTlb *tlb;
    MReadPort *readPorts;
    MWritePort *writePorts;
    MArbiter *arbiter;

    // Tracing...
    void Trace (sc_trace_file * tf, int levels = 1);
;
    
protected:
    // Processes...
    void proc_clk_memu ();
    void proc_cmb_interconnect ();

    // Methods...
    void InitSubmodules ();
    void FreeSubmodules ();

    // INTERNAL SIGNALS
    // Tag RAM...
    sc_signal<bool> tagram_ready, tagram_rd[TR_PORTS], tagram_wr[TR_PORTS];
    sc_signal<sc_uint<32> > tagram_adr[TR_PORTS];
    sc_signal<sc_uint<32> > tagram_wadr[TR_PORTS];
    sc_vector<sc_signal<SCacheTag> > tagram_tag_in{"tagram_tag_in",TR_PORTS};
    sc_vector<sc_signal<SCacheTag> >  tagram_tag_out{"tagram_tag_out",TR_PORTS};

     // Bank RAM...
    sc_signal<bool> bankram_rd[CFG_MEMU_CACHE_BANKS][CFG_MEMU_BANK_RAM_PORTS], bankram_wr[CFG_MEMU_CACHE_BANKS][CFG_MEMU_BANK_RAM_PORTS];
    sc_signal<sc_uint<4> > bankram_wen[CFG_MEMU_CACHE_BANKS][CFG_MEMU_BANK_RAM_PORTS];
    sc_signal<sc_uint<32> > bankram_wiadr[CFG_MEMU_CACHE_BANKS][CFG_MEMU_BANK_RAM_PORTS];
    sc_signal<sc_uint<32> > bankram_wdata[CFG_MEMU_CACHE_BANKS][CFG_MEMU_BANK_RAM_PORTS];
    sc_signal<sc_uint<32> > bankram_rdata[CFG_MEMU_CACHE_BANKS][CFG_MEMU_BANK_RAM_PORTS];


    // BusController
    sc_signal<bool>        master_cyc[MASTER_NO];                       // cycle valid output
    sc_signal<bool>        master_stb[MASTER_NO];                       // strobe output
    sc_signal<bool>        master_we[MASTER_NO];                        // indicates write transfer
    sc_signal<sc_uint<3> > master_cti[MASTER_NO];                       // cycle type identifier
    sc_signal<sc_uint<2> > master_bte[MASTER_NO];                       // burst type extension
    sc_signal<sc_uint<CFG_MEMU_BUSIF_WIDTH/8> > master_sel[MASTER_NO];  // byte select outputs
    sc_signal<sc_uint<32> > master_adr[MASTER_NO];                      // address bus outputs
    sc_signal<sc_uint<CFG_MEMU_BUSIF_WIDTH> > master_dat[MASTER_NO];    // output data bus
    sc_signal<bool> switch_master;

    // BUSIF...
    sc_signal<EBusIfOperation> busif_op;
    sc_signal<bool> busif_nolinelock, busif_busy;
    sc_signal<bool> busif_tag_rd, busif_tag_rd_way, busif_tag_wr, busif_bank_rd[CFG_MEMU_CACHE_BANKS],
    busif_bank_wr[CFG_MEMU_CACHE_BANKS];
    sc_signal<sc_uint<32> > busif_adr_in, busif_adr_out, busif_data_in[CFG_MEMU_CACHE_BANKS],
    busif_data_out[CFG_MEMU_CACHE_BANKS];
    sc_signal<bool> busif_data_out_valid[CFG_MEMU_CACHE_BANKS];
    sc_signal<SCacheTag> busif_tag_in, busif_tag_out;
    sc_signal<sc_uint<4> > busif_bsel;
    sc_signal<bool> busif_ac_r, busif_ac_w, busif_ac_x, busif_ac_u;
    sc_signal<bool> busif_trap_u, busif_trap_no_u;
    sc_signal<bool> busif_paging;
    
    // Read ports...
    sc_signal<sc_uint<32> > rp_busif_data_reg[CFG_MEMU_BUSIF_WIDTH/32]; // register to delay BusIF data for one clock cycle in accordance with the protocol
    sc_signal<sc_uint<32> > rp_busif_data[CFG_MEMU_RPORTS];
    sc_signal<EBusIfOperation> rp_busif_op[CFG_MEMU_RPORTS];
    sc_signal<bool> rp_tag_rd[CFG_MEMU_RPORTS], rp_bank_rd[CFG_MEMU_RPORTS];
    sc_signal<SCacheTag> rp_tag_in[CFG_MEMU_RPORTS];
    sc_signal<sc_uint<32> > rp_way_out[CFG_MEMU_RPORTS];
    sc_signal<sc_uint<32> > rp_bank_data_in[CFG_MEMU_RPORTS];
    sc_signal<sc_uint<32> > rp_bank_sel[CFG_MEMU_RPORTS];

    // Write ports...
    sc_signal<EBusIfOperation> wp_busif_op[CFG_MEMU_WPORTS];
    sc_signal<bool> wp_busif_nolinelock[CFG_MEMU_WPORTS];
    sc_signal<bool> wp_tag_rd[CFG_MEMU_WPORTS], wp_tag_wr[CFG_MEMU_WPORTS], wp_bank_rd[CFG_MEMU_WPORTS], wp_bank_wr[CFG_MEMU_WPORTS];
    sc_signal<SCacheTag> wp_tag_in[CFG_MEMU_WPORTS], wp_tag_out[CFG_MEMU_WPORTS];
    sc_signal<sc_uint<32> > wp_bank_data_in[CFG_MEMU_WPORTS], wp_bank_data_out[CFG_MEMU_WPORTS];
    sc_signal<sc_uint<4> > wp_bank_bsel[CFG_MEMU_WPORTS];

    // PTW
    sc_signal<sc_uint<32> > ptw_phys_adr;
    sc_signal<sc_uint<32> > ptw_virt_adr;
    sc_signal<bool> ptw_req;
    sc_signal<bool> ptw_ack;
    
    sc_signal<bool> ptw_ac_r;
    sc_signal<bool> ptw_ac_w;
    sc_signal<bool> ptw_ac_x;
    sc_signal<bool> ptw_ac_u;
    sc_signal<bool> ptw_ac_d;
    sc_signal<bool> ptw_ac_a;

    // TLB
    sc_signal<bool> tlb_req, tlb_wr;
    sc_signal<sc_uint<20> > tlb_va_o;
    sc_signal<sc_uint<20> > tlb_pa_o;
    sc_signal<bool> tlb_superpage_o;
    sc_signal<bool> tlb_ac_r_o;
    sc_signal<bool> tlb_ac_w_o;
    sc_signal<bool> tlb_ac_x_o;
    sc_signal<bool> tlb_ac_u_o;
    sc_signal<bool> tlb_ac_d_o;
    sc_signal<bool> tlb_ac_a_o;

    sc_signal<bool> tlb_superpage_i;
    sc_signal<sc_uint<20> > tlb_adr_i;
    sc_signal<bool> tlb_hit;
    sc_signal<bool> tlb_miss;

    sc_signal<bool> tlb_ac_r_i;
    sc_signal<bool> tlb_ac_w_i;
    sc_signal<bool> tlb_ac_x_i;
    sc_signal<bool> tlb_ac_u_i;
    sc_signal<bool> tlb_ac_d_i;
    sc_signal<bool> tlb_ac_a_i;


    // Arbiter: request/grant signals (find comments in 'MArbiter')...
    sc_signal<bool> req_busif_linelock;
    sc_signal<bool> req_wp_linelock[CFG_MEMU_WPORTS];
    sc_signal<bool> gnt_busif_linelock;
    sc_signal<bool> gnt_wp_linelock[CFG_MEMU_WPORTS];
    sc_signal<bool> req_busif_tagw;
    sc_signal<bool> req_wp_tagw[CFG_MEMU_WPORTS];
    sc_signal<bool> req_busif_tagr;
    sc_signal<bool> req_wp_tagr[CFG_MEMU_WPORTS];
    sc_signal<bool> req_rp_tagr[CFG_MEMU_RPORTS];
    sc_signal<bool> gnt_busif_tagw;
    sc_signal<bool> gnt_wp_tagw[CFG_MEMU_WPORTS];
    sc_signal<bool> gnt_busif_tagr;
    sc_signal<bool> gnt_wp_tagr[CFG_MEMU_WPORTS];
    sc_signal<bool> gnt_rp_tagr[CFG_MEMU_RPORTS];
    sc_signal<bool> gnt_busif_tagw_r;
    sc_signal<bool> gnt_wp_tagw_r[CFG_MEMU_WPORTS];
    sc_signal<bool> gnt_busif_tagr_r;
    sc_signal<bool> gnt_wp_tagr_r[CFG_MEMU_WPORTS];
    sc_signal<bool> gnt_rp_tagr_r[CFG_MEMU_RPORTS];

    sc_signal<bool> req_busif_bank[CFG_MEMU_CACHE_BANKS];
    sc_signal<bool> req_wp_bank[CFG_MEMU_WPORTS][CFG_MEMU_CACHE_BANKS];
    sc_signal<bool> req_rp_bank[CFG_MEMU_RPORTS][CFG_MEMU_CACHE_BANKS];
    sc_signal<bool> gnt_busif_bank[CFG_MEMU_CACHE_BANKS];
    sc_signal<bool> gnt_wp_bank[CFG_MEMU_WPORTS][CFG_MEMU_CACHE_BANKS];
    sc_signal<bool> gnt_rp_bank[CFG_MEMU_RPORTS][CFG_MEMU_CACHE_BANKS];

    sc_signal<bool> req_rp_busif[CFG_MEMU_RPORTS];
    sc_signal<bool> gnt_rp_busif[CFG_MEMU_RPORTS];
    sc_signal<bool> req_wp_busif[CFG_MEMU_WPORTS];
    sc_signal<bool> gnt_wp_busif[CFG_MEMU_WPORTS];

    // Arbiter: other signals ...
    sc_signal<sc_uint<32> > wiadr_busif;
    sc_signal<sc_uint<32> > wiadr_rp[CFG_MEMU_RPORTS];
    sc_signal<sc_uint<32> > adr_wp[CFG_MEMU_WPORTS]; 
    sc_signal<sc_uint<32> > way_wp[CFG_MEMU_WPORTS];
    sc_signal<sc_uint<32> > snoop_adr;
    sc_signal<bool> snoop_stb[CFG_MEMU_WPORTS];

    // no connect signals
    sc_signal<bool> NC_bool_out[CFG_MEMU_RPORTS];
    sc_signal<bool> NC_bool_in;
    sc_signal<sc_uint<32>> NC_uint32;
};


#endif
