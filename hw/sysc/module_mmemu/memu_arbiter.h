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
#ifndef _MEMU_ARBITER_
#define _MEMU_ARBITER_

#include "base.h"
#include "paranut-config.h"
#include "memu_common.h"
// **************** MArbiter ********************

class MArbiter : ::sc_core::sc_module {
public:
    // Ports ...
    sc_in<bool> clk, reset;

    sc_in<sc_uint<32> > wiadr_busif, wiadr_rp[CFG_MEMU_RPORTS], adr_wp[CFG_MEMU_WPORTS],
        way_wp[CFG_MEMU_WPORTS]; // (way+index) adresses from various ports;
                        // must be kept constant as long as 'req_*' lines are held
    sc_out<sc_uint<32> > wiadr_bank[CFG_MEMU_CACHE_BANKS][CFG_MEMU_BANK_RAM_PORTS]; // adress lines to the banks

    //   Write snooping...
    sc_out<sc_uint<32> > snoop_adr;
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

        // Init Multidimensional Vectors
        // bank_sel_in[CFG_MEMU_CACHE_BANKS][CFG_MEMU_BANK_RAM_PORTS][2]
        for(uint i = 0; i < CFG_MEMU_CACHE_BANKS; i++){
            bank_sel_in[i].init(CFG_MEMU_BANK_RAM_PORTS);
            for(uint j = 0; j < CFG_MEMU_BANK_RAM_PORTS; j++){
                bank_sel_in[i][j].init(2);
            }
        }

        // bank_sel_out[CFG_MEMU_CACHE_BANKS][CFG_MEMU_BANK_RAM_PORTS]
        for(uint i = 0; i < CFG_MEMU_CACHE_BANKS; i++){
            bank_sel_out[i].init(CFG_MEMU_BANK_RAM_PORTS);
        }
        
        //bank_reg[CFG_MEMU_CACHE_BANKS][CFG_MEMU_BANK_RAM_PORTS]
        //next_bank_reg[CFG_MEMU_CACHE_BANKS][CFG_MEMU_BANK_RAM_PORTS]
        for(uint i = 0; i < CFG_MEMU_CACHE_BANKS; i++){
            bank_reg[i].init(CFG_MEMU_BANK_RAM_PORTS);
            next_bank_reg[i].init(CFG_MEMU_BANK_RAM_PORTS);
        }

        // Methods
        SC_METHOD (proc_cmb_linelock);
            sensitive << req_busif_linelock << linelock_reg << wiadr_busif << cpu_prio;
            sensitive << linelock_sel_out;
            for (uint n = 0; n < CFG_MEMU_WPORTS; n++)
                sensitive << req_wp_linelock[n] << adr_wp[n];
        SC_METHOD (proc_cmb_tag);
            sensitive << tagram_ready << req_busif_tagr << req_busif_tagw << cpu_prio;
            sensitive << tagr_reg << tagw_reg << req_tagw_reg;
            for (uint n = 0; n < CFG_MEMU_RPORTS; n++)
                sensitive << req_rp_tagr[n];
            for (uint n = 0; n < CFG_MEMU_WPORTS; n++)
                sensitive << req_wp_tagr[n] << req_wp_tagw[n];
        SC_METHOD (proc_cmb_bank);
            sensitive << wiadr_busif << cpu_prio;
            for (uint n = 0; n < CFG_MEMU_RPORTS; n++)
                sensitive << wiadr_rp[n];
            for (uint n = 0; n < CFG_MEMU_WPORTS; n++)
                sensitive << adr_wp[n] << way_wp[n];
            for (uint b = 0; b < CFG_MEMU_CACHE_BANKS; b++) {
                sensitive << req_busif_bank[b];
                for (uint n = 0; n < CFG_MEMU_BANK_RAM_PORTS; n++)
                    sensitive << bank_sel_out[b][n];
                for (uint n = 0; n < CFG_MEMU_RPORTS; n++)
                    sensitive << req_rp_bank[n][b];
                for (uint n = 0; n < CFG_MEMU_WPORTS; n++)
                    sensitive << req_wp_bank[n][b];
            }
        SC_METHOD (proc_cmb_busif);
            sensitive << busif_reg << cpu_prio;
            for (uint n = 0; n < CFG_MEMU_RPORTS; n++)
                sensitive << req_rp_busif[n];
            for (uint n = 0; n < CFG_MEMU_WPORTS; n++)
                sensitive << req_wp_busif[n];
            sensitive << busif_sel_out;
        SC_METHOD (proc_cmb_snoop);
            sensitive << next_linelock_reg;
            for (uint n = 0; n < CFG_MEMU_WPORTS; n++)
                sensitive << adr_wp[n];
        SC_METHOD (proc_cmb_priocpu);
            sensitive << counter_reg;

        SC_CTHREAD (proc_clk_arbiter, clk.pos ());
            reset_signal_is (reset, true);

        // Connections
        for (uint b = 0; b < CFG_MEMU_CACHE_BANKS; b++) {
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
    void proc_cmb_linelock ();
    void proc_cmb_tag ();
    void proc_cmb_bank ();
    void proc_cmb_busif ();
    void proc_cmb_snoop ();
    void proc_cmb_priocpu ();

    void proc_clk_arbiter ();


protected:

    // Helpers...
    uint GetPrioCpu ();

    // Registers...
    sc_signal<sc_uint<16> > counter_reg;
    sc_signal<sc_uint<CFG_MEMU_WPORTS + 1> > linelock_reg;
    sc_signal<sc_uint<CFG_MEMU_RPORTS + CFG_MEMU_WPORTS + 1> > tagr_reg;
    sc_signal<sc_uint<CFG_MEMU_WPORTS + 1> > req_tagw_reg;
    sc_signal<sc_uint<CFG_MEMU_WPORTS + 1> > tagw_reg;
    // bank_reg[CFG_MEMU_CACHE_BANKS][CFG_MEMU_BANK_RAM_PORTS];
    sc_vector<sc_vector<sc_signal<sc_uint<((CFG_MEMU_RPORTS + CFG_MEMU_WPORTS+(CFG_NUT_CPU_CORES_LD==0)) / CFG_MEMU_BANK_RAM_PORTS) + 1> > > >bank_reg{"bank_reg", CFG_MEMU_CACHE_BANKS};
    sc_signal<sc_uint<CFG_MEMU_RPORTS + CFG_MEMU_WPORTS> > busif_reg;

    // Signals...
    sc_signal<sc_uint<CFG_MEMU_WPORTS + 1> > next_linelock_reg;
    sc_signal<sc_uint<CFG_MEMU_RPORTS + CFG_MEMU_WPORTS + 1> > next_tagr_reg;
    sc_signal<sc_uint<CFG_MEMU_WPORTS + 1> > next_req_tagw_reg;
    sc_signal<sc_uint<CFG_MEMU_WPORTS + 1> > next_tagw_reg;
    // next_bank_reg[CFG_MEMU_CACHE_BANKS][CFG_MEMU_BANK_RAM_PORTS];
    sc_vector<sc_vector<sc_signal<sc_uint<((CFG_MEMU_RPORTS + CFG_MEMU_WPORTS+(CFG_NUT_CPU_CORES_LD==0)) / CFG_MEMU_BANK_RAM_PORTS) + 1> > > > next_bank_reg{"next_bank_reg", CFG_MEMU_CACHE_BANKS};
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
    // Vector sizes are initialized in the constructor
    // bank_sel_out[CFG_MEMU_CACHE_BANKS][CFG_MEMU_BANK_RAM_PORTS]
    sc_vector<sc_vector<sc_signal<SSelectorIO<CFG_MEMU_CACHE_WAYS_LD+CFG_MEMU_CACHE_SETS_LD, (CFG_MEMU_RPORTS+CFG_MEMU_WPORTS+SINGLE_CPU)/CFG_MEMU_BANK_RAM_PORTS> > > > bank_sel_out{"bank_sel_out", CFG_MEMU_CACHE_BANKS};
    // bank_sel_in[CFG_MEMU_CACHE_BANKS][CFG_MEMU_BANK_RAM_PORTS][2]
    sc_vector<sc_vector<sc_vector<sc_signal<SSelectorIO<CFG_MEMU_CACHE_WAYS_LD+CFG_MEMU_CACHE_SETS_LD, (CFG_MEMU_RPORTS+CFG_MEMU_WPORTS+SINGLE_CPU)/CFG_MEMU_BANK_RAM_PORTS> > > > > bank_sel_in{"bank_sel_in", CFG_MEMU_CACHE_BANKS};
    

};
#endif