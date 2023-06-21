
#include "memu_arbiter.h"
// **************** MArbiter ********************

void MArbiter::Trace (sc_trace_file *tf, int level) {
    if (!tf || pn_trace_verbose) printf ("\nSignals of Module \"%s\":\n", name ());

    // Ports ...
    PN_TRACE (tf, clk);
    PN_TRACE (tf, reset);

    PN_TRACE (tf, wiadr_busif);
    PN_TRACE_BUS (tf, wiadr_rp, CFG_MEMU_RPORTS);
    PN_TRACE_BUS (tf, adr_wp, CFG_MEMU_WPORTS);
    PN_TRACE_BUS (tf, way_wp, CFG_MEMU_WPORTS);
    PN_TRACE_BUS_BUS (tf, wiadr_bank, CFG_MEMU_CACHE_BANKS, CFG_MEMU_BANK_RAM_PORTS);

    //   Write snooping...
    PN_TRACE (tf, snoop_adr);
    PN_TRACE_BUS (tf, snoop_stb, CFG_MEMU_WPORTS);

    //   Line lock...
    PN_TRACE (tf, req_busif_linelock);
    PN_TRACE_BUS (tf, req_wp_linelock, CFG_MEMU_WPORTS);
    PN_TRACE (tf, gnt_busif_linelock);
    PN_TRACE_BUS (tf, gnt_wp_linelock, CFG_MEMU_WPORTS);

    //   Tag RAM...
    PN_TRACE (tf, tagram_ready);
    PN_TRACE (tf, req_busif_tagw);
    PN_TRACE_BUS (tf, req_wp_tagw, CFG_MEMU_WPORTS);
    PN_TRACE (tf, req_busif_tagr);
    PN_TRACE_BUS (tf, req_wp_tagr, CFG_MEMU_WPORTS);
    PN_TRACE_BUS (tf, req_rp_tagr, CFG_MEMU_RPORTS);
    PN_TRACE (tf, gnt_busif_tagw);
    PN_TRACE_BUS (tf, gnt_wp_tagw, CFG_MEMU_WPORTS);
    PN_TRACE (tf, gnt_busif_tagr);
    PN_TRACE_BUS (tf, gnt_wp_tagr, CFG_MEMU_WPORTS);
    PN_TRACE_BUS (tf, gnt_rp_tagr, CFG_MEMU_RPORTS);

    //   Bank RAMs...
    PN_TRACE_BUS (tf, req_busif_bank, CFG_MEMU_CACHE_BANKS);
    PN_TRACE_BUS_BUS (tf, req_wp_bank, CFG_MEMU_WPORTS, CFG_MEMU_CACHE_BANKS);
    PN_TRACE_BUS_BUS (tf, req_rp_bank, CFG_MEMU_RPORTS, CFG_MEMU_CACHE_BANKS);
    PN_TRACE_BUS (tf, gnt_busif_bank, CFG_MEMU_CACHE_BANKS);
    PN_TRACE_BUS_BUS (tf, gnt_wp_bank, CFG_MEMU_WPORTS, CFG_MEMU_CACHE_BANKS);
    PN_TRACE_BUS_BUS (tf, gnt_rp_bank, CFG_MEMU_RPORTS, CFG_MEMU_CACHE_BANKS);

    //   BUSIF...
    PN_TRACE_BUS (tf, req_rp_busif, CFG_MEMU_RPORTS);
    PN_TRACE_BUS (tf, req_wp_busif, CFG_MEMU_WPORTS);
    PN_TRACE_BUS (tf, gnt_rp_busif, CFG_MEMU_RPORTS);
    PN_TRACE_BUS (tf, gnt_wp_busif, CFG_MEMU_WPORTS);

    // Registers...
    PN_TRACE (tf, counter_reg);
    PN_TRACE (tf, linelock_reg);
    PN_TRACE (tf, tagr_reg);
    PN_TRACE (tf, tagw_reg);
    PN_TRACE_BUS_BUS (tf, bank_reg, CFG_MEMU_CACHE_BANKS, CFG_MEMU_BANK_RAM_PORTS);
    PN_TRACE (tf, busif_reg);

    // Internal signals...
    PN_TRACE (tf, next_linelock_reg);
    PN_TRACE (tf, next_tagr_reg);
    PN_TRACE (tf, next_tagw_reg);
    PN_TRACE_BUS_BUS (tf, next_bank_reg, CFG_MEMU_CACHE_BANKS, CFG_MEMU_BANK_RAM_PORTS);
    PN_TRACE (tf, next_busif_reg);

    // Trace submodules...
    if (level > 1) {
        level--;

        for (uint b = 0; b < CFG_MEMU_CACHE_BANKS; b++) {
            if (CFG_NUT_CPU_CORES >= 2 || CFG_MEMU_BANK_RAM_PORTS == 1) {
                BankSelector_t *sel = (BankSelector_t*)bank_sel[b][CFG_MEMU_BANK_RAM_PORTS-1];
                sel->Trace (tf, level);
            } else {
                BankSelectorPass_t *sel = (BankSelectorPass_t*)bank_sel[b][CFG_MEMU_BANK_RAM_PORTS-1];
                sel->Trace (tf, level);
            }

            if (CFG_MEMU_BANK_RAM_PORTS > 1) {
                if (CFG_NUT_CPU_CORES >= 4) {
                    BankSelector_t *sel = (BankSelector_t*)bank_sel[b][0];
                    sel->Trace (tf, level);
                } else {
                    BankSelectorPass_t *sel = (BankSelectorPass_t*)bank_sel[b][0];
                    sel->Trace (tf, level);
                }
            }
        }

        // BusIf Arbitration Steps...
        if (CFG_NUT_CPU_CORES > 1) {
            BusifSelector_t *sel = (BusifSelector_t*)busif_sel;
            sel->Trace (tf, level);
        } else {
            BusIfSelectorPass_t *sel = (BusIfSelectorPass_t*)busif_sel;
            sel->Trace (tf, level);
        }
        // LineLock Arbitration Steps...
        if (CFG_NUT_CPU_CORES > 1) {
            LineLockSelector_t *sel = (LineLockSelector_t*)linelock_sel;
            sel->Trace (tf, level);
        } else {
            LineLockSelectorPass_t *sel = (LineLockSelectorPass_t*)linelock_sel;
            sel->Trace (tf, level);
        }
    }
}

MArbiter::~MArbiter (){
    delete busif_sel;
    delete linelock_sel;
}

void MArbiter::proc_cmb_priocpu () {
    cpu_prio = GetPrioCpu ();
}

void MArbiter::proc_cmb_linelock () {
    sc_uint<CFG_MEMU_WPORTS + 1> req_linelock, gnt_linelock;
    SSelectorIO<1, CFG_MEMU_WPORTS> linelock_wport_sel_in [CFG_MEMU_WPORTS-1];

    // Current policy (to save area):
    // - WPORT requests always exclude each other, indepent of the index adress
    // - concurrent BUSIF and WPORT grants are possible, if they adress different lines

    // Collect all request signals...
    req_linelock[CFG_MEMU_WPORTS] = req_busif_linelock;
    for (uint n = 0; n < CFG_MEMU_WPORTS; n++) req_linelock[n] = req_wp_linelock[n];

    // Determine existing & to-keep grants...
    gnt_linelock = linelock_reg.read () & req_linelock;

    // Handle BUSIF request (highest priority)...
    if (req_linelock[CFG_MEMU_WPORTS] & !gnt_linelock[CFG_MEMU_WPORTS]) {
        gnt_linelock[CFG_MEMU_WPORTS] = 1;
        for (uint n = 0; n < CFG_MEMU_WPORTS; n++)
            if (gnt_linelock[n] && GetIndexOfAdr (adr_wp[n].read ()) == GetIndexOfWayIndex (wiadr_busif.read ()))
                gnt_linelock[CFG_MEMU_WPORTS] = 0;
    }

    // Set selector inputs...
    for (uint n = 0; n < CFG_MEMU_WPORTS; n++) {
        SSelectorIO<1, CFG_MEMU_WPORTS> sel_in (0, n, 0);
        // Make sure to only request a grant if the cache line is different from the BusIf line...
        if (req_linelock[n] == 1 && (!gnt_linelock[CFG_MEMU_WPORTS] || GetIndexOfAdr (adr_wp[n].read ()) != GetIndexOfWayIndex (wiadr_busif.read ())))
            sel_in.sel_valid = 1;
        else
            sel_in.sel_valid = 0;
        if (n == 0)
            linelock_sel_in[0] = sel_in;
        else
            linelock_wport_sel_in[n-1] = sel_in;
    }

    // Select slow input...
    linelock_sel_in[1] = linelock_wport_sel_in[0];
    for (uint n = 0; n < CFG_MEMU_WPORTS-1; n++) {
        uint i = (n + cpu_prio.read()) % (CFG_MEMU_WPORTS-1);
        if (linelock_wport_sel_in[i].sel_valid) {
            linelock_sel_in[1] = linelock_wport_sel_in[i];
            break;
        }
    }

    // Handle result of write port requests...
    if (gnt_linelock(CFG_MEMU_WPORTS-1, 0) == 0) {
        // New grant...
        if (linelock_sel_out.read ().sel_valid == 1)
            gnt_linelock[linelock_sel_out.read ().sel] = 1;
    }

    // Write results...
    next_linelock_reg = gnt_linelock;

    gnt_busif_linelock = gnt_linelock[CFG_MEMU_WPORTS];
    for (uint n = 0; n < CFG_MEMU_WPORTS; n++) gnt_wp_linelock[n] = gnt_linelock[n];
}


void MArbiter::proc_cmb_tag () {
    sc_uint<CFG_MEMU_RPORTS + CFG_MEMU_WPORTS + 1> req_tagr, gnt_tagr;
    sc_uint<CFG_MEMU_WPORTS + 1> req_tagw, gnt_tagw;

    if (tagram_ready == 0) { // Tag RAM not yet ready...
        gnt_tagr = 0;
        gnt_tagw = 0;
    } else {
        // Collect all request signals...
        for (uint n = 0; n < CFG_MEMU_RPORTS; n++) req_tagr[n] = req_rp_tagr[n];
        for (uint n = 0; n < CFG_MEMU_WPORTS; n++) {
            req_tagr[CFG_MEMU_RPORTS + n] = req_wp_tagr[n];
            req_tagw[n] = req_wp_tagw[n];
        }
        req_tagr[CFG_MEMU_RPORTS + CFG_MEMU_WPORTS] = req_busif_tagr;
        req_tagw[CFG_MEMU_WPORTS] = req_busif_tagw;

        // Determine existing & to-keep grants...
        gnt_tagr = tagr_reg.read () & req_tagr;
        gnt_tagw = tagw_reg.read () & req_tagw;

        // Handle read requests...
        if (req_tagw_reg.read () == 0) { // Writer priority: only accept new reader if no write request waited for more than one cycle ...
            for (uint n = 0; n < CFG_NUT_CPU_CORES; n++) { // Select highest priority acquired bit for each CPU
                if (!gnt_tagr[n] && !gnt_tagr[CFG_MEMU_WPORTS + n] && !gnt_tagr[2 * CFG_MEMU_WPORTS + n] &&
                    (n != CFG_NUT_CPU_CORES - 1 || !gnt_tagr[CFG_MEMU_RPORTS + CFG_MEMU_WPORTS])) {
                    // no exisiting grant...
                    if (n == CFG_NUT_CPU_CORES - 1 && req_tagr[CFG_MEMU_RPORTS + CFG_MEMU_WPORTS]) // Prio 0: BUSIF (shares port with last CPU)
                        gnt_tagr[CFG_MEMU_RPORTS + CFG_MEMU_WPORTS] = 1;
                    else if (req_tagr[n]) // Prio 1: Data read
                        gnt_tagr[n] = 1;
                    else if (req_tagr[CFG_MEMU_WPORTS + n]) // Prio 2: Insn read
                        gnt_tagr[CFG_MEMU_WPORTS + n] = 1;
                    else if (req_tagr[CFG_MEMU_RPORTS + n]) // Prio 3: Data write
                        gnt_tagr[CFG_MEMU_RPORTS + n] = 1;
                }
            }
        }

        // cout << "### tagw_reg = " << tagw_reg << "  req_tagw = " << req_tagw << "  gnt_tagw = " << gnt_tagw << endl;

        // Handle write requests...
        if (gnt_tagw == 0) { // can only accept new writers if no other writer active
            if (req_tagw[CFG_MEMU_WPORTS])
                gnt_tagw[CFG_MEMU_WPORTS] = 1; // give BUSIF highest priority (good? -> request may originate from a read miss)
            else {
                for (uint n = 0; n < CFG_MEMU_WPORTS; n++) {
                    int i = (int)(n + cpu_prio.read()) % (int)CFG_MEMU_WPORTS;
                    if (req_tagw[i]) {
                        gnt_tagw[i] = 1;
                        break;
                    }
                }
            }
        }
    }

    // Write results...
    next_req_tagw_reg = req_tagw;
    next_tagr_reg = gnt_tagr;
    next_tagw_reg = gnt_tagw;

    for (uint n = 0; n < CFG_MEMU_RPORTS; n++) {
        gnt_rp_tagr[n] = gnt_tagr[n];
        gnt_rp_tagr_r[n] = tagr_reg.read ()[n];
    }
    for (uint n = 0; n < CFG_MEMU_WPORTS; n++) {
        gnt_wp_tagr[n] = gnt_tagr[CFG_MEMU_RPORTS + n];
        gnt_wp_tagw[n] = gnt_tagw[n];
        gnt_wp_tagr_r[n] = tagr_reg.read ()[CFG_MEMU_RPORTS + n];
        gnt_wp_tagw_r[n] = tagw_reg.read ()[n];
    }
    gnt_busif_tagr = gnt_tagr[CFG_MEMU_RPORTS + CFG_MEMU_WPORTS];
    gnt_busif_tagw = gnt_tagw[CFG_MEMU_WPORTS];
    gnt_busif_tagr_r = tagr_reg.read ()[CFG_MEMU_RPORTS + CFG_MEMU_WPORTS];
    gnt_busif_tagw_r = tagw_reg.read ()[CFG_MEMU_WPORTS];
}


// Mappings for the bank ports to the 'req_bank'/'gnt_bank' bus ...
#define SINGLE_CPU (CFG_NUT_CPU_CORES_LD==0)// used for special case of a single cpu
#define IDX_OF_CPU_IN_PORT(cpu) (CFG_MEMU_BANK_RAM_PORTS == 2 ? (cpu>>1) : cpu)  // remove rightmost bit
#define IDX_OF_CPU(cpu) (CFG_MEMU_BANK_RAM_PORTS == 2 ? (cpu>>1)*3 : cpu*3)        // remove rightmost bit and multiply by number of ports (2RPs + 1WP)
#define IDX_RP(cpu) (IDX_OF_CPU(cpu))       // data read ports
#define IDX_IP(cpu) (IDX_OF_CPU(cpu) + 1)   // insn read ports
#define IDX_WP(cpu) (IDX_OF_CPU(cpu) + 2)   // write ports
#define IDX_BUSIF ((CFG_MEMU_RPORTS+CFG_MEMU_WPORTS+SINGLE_CPU) / CFG_MEMU_BANK_RAM_PORTS)         // BusIF port


// Mappings of CPUs and BusIf to the bank RAM ports...
#define RAMPORT_BUSIF (CFG_MEMU_BANK_RAM_PORTS - 1)
#define RAMPORT(idx) ((idx) == IDX_BUSIF ? RAMPORT_BUSIF : ((idx) % CFG_NUT_CPU_CORES) % CFG_MEMU_BANK_RAM_PORTS)


void MArbiter::proc_cmb_bank () {
    sc_uint<((CFG_MEMU_RPORTS+CFG_MEMU_WPORTS+SINGLE_CPU) / CFG_MEMU_BANK_RAM_PORTS) + 1> req_bank[CFG_MEMU_BANK_RAM_PORTS], gnt_bank[CFG_MEMU_BANK_RAM_PORTS];
    uint32_t sel_wiadr[CFG_MEMU_BANK_RAM_PORTS]/*, wiadr[CFG_MEMU_BANK_RAM_PORTS][(CFG_MEMU_RPORTS+CFG_MEMU_WPORTS+SINGLE_CPU) / CFG_MEMU_BANK_RAM_PORTS + 1]*/;
    // For only one Bankram port all CPUs get arbitrated to the BusIfSelector
    SSelectorIO<CFG_MEMU_CACHE_WAYS_LD+CFG_MEMU_CACHE_SETS_LD, (CFG_MEMU_RPORTS+CFG_MEMU_WPORTS+SINGLE_CPU)/CFG_MEMU_BANK_RAM_PORTS> bank_cpu_sel_in [CFG_MEMU_BANK_RAM_PORTS][CFG_NUT_CPU_CORES/CFG_MEMU_BANK_RAM_PORTS];
    int sel_br[CFG_MEMU_BANK_RAM_PORTS];

    for (uint b = 0; b < CFG_MEMU_CACHE_BANKS; b++) {
        
        // Collect all request signals for this bank, sorted the same way as wiadr...
        for (uint n = 0; n < CFG_NUT_CPU_CORES; n++) {
            req_bank[RAMPORT(n)][IDX_RP (n)] = req_rp_bank[n][b];
            req_bank[RAMPORT(n)][IDX_IP (n)] = req_rp_bank[CFG_NUT_CPU_CORES + n][b];
            req_bank[RAMPORT(n)][IDX_WP (n)] = req_wp_bank[n][b];
        }
        if (!SINGLE_CPU) req_bank[0][IDX_BUSIF] = 0; // Again port 0 has one less input - don't care in hardware
        req_bank[RAMPORT(IDX_BUSIF)][IDX_BUSIF] = req_busif_bank[b];

        //  All bank accesses are done in 1 cycle, no need to keep track of already granted ports......
        for (uint p = 0; p < CFG_MEMU_BANK_RAM_PORTS; p++)
            gnt_bank[p] = 0;

        // Preset sel_port and sel_br...
        for (uint n = 0; n < CFG_NUT_CPU_CORES; n++){
            for (uint p = 0; p < CFG_MEMU_BANK_RAM_PORTS; p++) sel_br[p] = -1;
        }
        // Determine the selected ports per CPU...
        for (uint n = 0; n < CFG_NUT_CPU_CORES; n++) {
            // Preset sel_in for this CPU (wiadr = -1 (don't car), sel = -1 (max. value), valid = 0)
            SSelectorIO<CFG_MEMU_CACHE_WAYS_LD+CFG_MEMU_CACHE_SETS_LD, (CFG_MEMU_RPORTS+CFG_MEMU_WPORTS+SINGLE_CPU)/CFG_MEMU_BANK_RAM_PORTS> sel_in (-1, -1, 0);
            if (req_bank[RAMPORT(n)][IDX_RP (n)]) {
                sel_in.sel = IDX_RP (n); // Prio 1: Data read
                sel_in.dat = wiadr_rp[n];
                sel_in.sel_valid = 1;
            } else if (req_bank[RAMPORT(n)][IDX_IP (n)]) {
                sel_in.sel = IDX_IP (n); // Prio 2: Insn read
                sel_in.dat = wiadr_rp[CFG_NUT_CPU_CORES + n];
                sel_in.sel_valid = 1;
            } else if (req_bank[RAMPORT(n)][IDX_WP (n)]) {
                sel_in.sel = IDX_WP (n); // Prio 3: Data write
                sel_in.dat = GetWayIndexOfAdr (adr_wp[n].read (), way_wp[n].read ());
                sel_in.sel_valid = 1;
            }
            if (n == 0 && CFG_MEMU_BANK_RAM_PORTS > 1){
                // At 2 ports CePU gets its own selector...
                bank_sel_in[b][0][0] = sel_in;
            }else{
                bank_cpu_sel_in[RAMPORT(n)][IDX_OF_CPU_IN_PORT(n)-(CFG_MEMU_BANK_RAM_PORTS > 1 && RAMPORT(n) == 0)] = sel_in;
            }
        }
        // BusIf always gets the fast input of the BankBusIfSel...
        SSelectorIO<CFG_MEMU_CACHE_WAYS_LD+CFG_MEMU_CACHE_SETS_LD, (CFG_MEMU_RPORTS+CFG_MEMU_WPORTS+SINGLE_CPU)/CFG_MEMU_BANK_RAM_PORTS> sel_in (wiadr_busif.read (), IDX_BUSIF, req_bank[RAMPORT(IDX_BUSIF)][IDX_BUSIF]);
        bank_sel_in[b][CFG_MEMU_BANK_RAM_PORTS-1][0] = sel_in;

        // Select slow input to selectors...
        for (uint p = 0; p < CFG_MEMU_BANK_RAM_PORTS; p++) {
            bank_sel_in[b][p][1] = bank_cpu_sel_in[p][0];
            
            uint n_max = CFG_NUT_CPU_CORES/CFG_MEMU_BANK_RAM_PORTS;
            if(CFG_MEMU_BANK_RAM_PORTS > 1 && p == 0)
                n_max = n_max-1;
            
            for (uint n = 0; n < n_max; n++) {
                uint i = (uint)(n + cpu_prio.read()) % n_max;
                if (bank_cpu_sel_in[p][i].sel_valid) {
                    bank_sel_in[b][p][1] = bank_cpu_sel_in[p][i];
                    break;
                }
            }
        }

        // Find selected port & grant...
        for (uint p = 0; p < CFG_MEMU_BANK_RAM_PORTS; p++) {
            sel_wiadr[p] = 0xffffffff; // should be don't care
            // New grant
            if (bank_sel_out[b][p].read ().sel_valid == 1) {
                sel_br[p] = bank_sel_out[b][p].read ().sel;
                sel_wiadr[p] = bank_sel_out[b][p].read ().dat;
                gnt_bank[p][sel_br[p]] = 1;
            }
        }

        // Write results...
        for (uint p = 0; p < CFG_MEMU_BANK_RAM_PORTS; p++)
            next_bank_reg[b][p] = gnt_bank[p];

        for (uint n = 0; n < CFG_NUT_CPU_CORES; n++) {
            gnt_rp_bank[n][b] = gnt_bank[RAMPORT(n)][IDX_RP (n)];
            gnt_rp_bank[CFG_NUT_CPU_CORES + n][b] = gnt_bank[RAMPORT(n)][IDX_IP (n)];
            gnt_wp_bank[n][b] = gnt_bank[RAMPORT(n)][IDX_WP (n)];
        }
        gnt_busif_bank[b] = gnt_bank[RAMPORT(IDX_BUSIF)][IDX_BUSIF];

        for (uint p = 0; p < CFG_MEMU_BANK_RAM_PORTS; p++) wiadr_bank[b][p] = sel_wiadr[p];
    }
}


void MArbiter::proc_cmb_busif () {
    sc_uint<CFG_MEMU_RPORTS + CFG_MEMU_WPORTS> req_busif, gnt_busif;
    SSelectorIO<1, (CFG_MEMU_RPORTS+CFG_MEMU_WPORTS+1)> busif_cpu_sel_in [CFG_NUT_CPU_CORES-1];
    int busif_sel;

    // Collect all request signals...
    for (uint n = 0; n < CFG_MEMU_RPORTS; n++) req_busif[n] = req_rp_busif[n];
    for (uint n = 0; n < CFG_MEMU_WPORTS; n++) req_busif[CFG_MEMU_RPORTS + n] = req_wp_busif[n];

    // Determine existing & to-keep grants...
    gnt_busif = busif_reg.read () & req_busif;

    // Handle new requests...
    busif_sel = CFG_MEMU_RPORTS+CFG_MEMU_WPORTS;
    for (uint n = 0; n < CFG_NUT_CPU_CORES; n++) {
        SSelectorIO<1, (CFG_MEMU_RPORTS+CFG_MEMU_WPORTS+1)> sel_in (0, CFG_MEMU_RPORTS+CFG_MEMU_WPORTS, 0);
        // Prio 0: Granted ports
        if (gnt_busif[n]) {
            // Data read port
            busif_sel = n;
            sel_in.sel = n;
            sel_in.sel_valid = 1;
        } else if (gnt_busif[CFG_NUT_CPU_CORES + n]) {
            // Insn read port
            busif_sel = CFG_NUT_CPU_CORES + n;
            sel_in.sel = CFG_NUT_CPU_CORES + n;
            sel_in.sel_valid = 1;
        } else if (gnt_busif[CFG_MEMU_RPORTS + n]) {
            // Data write port
            busif_sel = CFG_MEMU_RPORTS + n;
            sel_in.sel = CFG_MEMU_RPORTS + n;
            sel_in.sel_valid = 1;
        } else {
            if (req_busif[n]) {
                // Data read port
                sel_in.sel = n;
                sel_in.sel_valid = 1;
            } else if (req_busif[CFG_NUT_CPU_CORES + n]) {
                // Insn read port
                sel_in.sel = CFG_NUT_CPU_CORES + n;
                sel_in.sel_valid = 1;
            } else if (req_busif[CFG_MEMU_RPORTS + n]) {
                // Data write port
                sel_in.sel = CFG_MEMU_RPORTS + n;
                sel_in.sel_valid = 1;
            }
        }
        if(n == 0)
             busif_sel_in[0] = sel_in;
        else
            busif_cpu_sel_in[n-1] = sel_in;
    }

    // Select slow input...
    busif_sel_in[1] = busif_cpu_sel_in[0];
    for (uint n = 0; n < CFG_NUT_CPU_CORES-1; n++) {
        uint i = (int)(n + cpu_prio.read()) % (int)(CFG_NUT_CPU_CORES-1);
        if (busif_cpu_sel_in[i].sel_valid) {
            busif_sel_in[1] = busif_cpu_sel_in[i];
            break;
        }
    }

    // Find selected port & grant...
    if (busif_sel == CFG_MEMU_RPORTS+CFG_MEMU_WPORTS) {
        // New grant
        if (busif_sel_out.read ().sel_valid == 1) {
            busif_sel = busif_sel_out.read ().sel;
            gnt_busif[busif_sel] = 1;
        }
    }

    // Write results...
    next_busif_reg = gnt_busif;

    for (uint n = 0; n < CFG_MEMU_RPORTS; n++) gnt_rp_busif[n] = gnt_busif[n];
    for (uint n = 0; n < CFG_MEMU_WPORTS; n++) gnt_wp_busif[n] = gnt_busif[CFG_MEMU_RPORTS + n];
}


void MArbiter::proc_cmb_snoop () {
    int writer = -1;

    // Determine a/the writer...
    //   NOTE: only cached writes are supported for snooping (LL/SC)
    for (uint n = 0; n < CFG_MEMU_WPORTS; n++) {
        if (next_linelock_reg.read ()[n] == 1) { // to catch a writer to the cache
            PN_ASSERT (writer < 0); // there should be only one!!
            writer = n;
        }
    }

    // Generate output signals...
    if (writer >= 0) {
        snoop_adr = adr_wp[writer];
        for (uint n = 0; n < CFG_MEMU_WPORTS; n++) snoop_stb[n] = 1; // signal to all CPUs ...
        snoop_stb[writer] = 0; // ... except the one that caused the write to avoid race condition
    } else {
        snoop_adr = 0xffffffff; // don't care
        for (uint n = 0; n < CFG_MEMU_WPORTS; n++) snoop_stb[n] = 0;
    }
}

// using uint32 as return value here limits the paranut to a maximum of 32 cores
uint32_t MArbiter::GetPrioCpu () {
#if CFG_MEMU_ARBITER_METHOD >= 0 // round robin...
    return (counter_reg.read () >> CFG_MEMU_ARBITER_METHOD) % CFG_NUT_CPU_CORES;
#else // LFSR...
    return counter_reg.read () % CFG_NUT_CPU_CORES;
#endif
}


void MArbiter::proc_clk_arbiter () {
    // Reset...
    counter_reg = 0xffff;
    linelock_reg = 0;

    // Main loop...
    wait ();
    while (1) {
        // 'counter_reg'...
#if CFG_MEMU_ARBITER_METHOD >= 0 // round robin...
        counter_reg = (counter_reg.read () + 1) & ((CFG_NUT_CPU_CORES << CFG_MEMU_ARBITER_METHOD) - 1);
#else // LFSR...
        counter_reg = GetNextLfsrState (counter_reg.read ()));
#endif

        linelock_reg = next_linelock_reg;
        tagr_reg = next_tagr_reg;
        req_tagw_reg = next_req_tagw_reg;
        tagw_reg = next_tagw_reg;
        for (uint n = 0; n < CFG_MEMU_CACHE_BANKS; n++)
            for (uint p = 0; p < CFG_MEMU_BANK_RAM_PORTS; p++)
                bank_reg[n][p] = next_bank_reg[n][p];
        busif_reg = next_busif_reg;
        wait ();
    }
}
