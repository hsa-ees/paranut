/*************************************************************************

  This file is part of the ParaNut project.

  Copyright (C) 2010-2020 Alexander Bahle <alexander.bahle@hs-augsburg.de>
                          Gundolf Kiefer <gundolf.kiefer@hs-augsburg.de>
      Hochschule Augsburg, University of Applied Sciences

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


#include "memu.h"

#include "lfsr.h"

#define C(MEMBER) (MEMBER == t.MEMBER)
#define C_ARR(MEMBER, NUM) ({ bool ret = 1; for (int n = 0; n < NUM; ++n) if (!C(MEMBER[n])) ret = 0; ret; })

// ***** SCacheTag *****

// Compare (necessary for signal updates)...
bool SCacheTag::operator== (const SCacheTag &t) const {
    return C(tadr) && C(valid) && C(dirty) && C(way);
}

// Display...
ostream &operator<< (ostream &os, const SCacheTag &t) {
    os << "valid=" << t.valid << " dirty=" << t.dirty << " tadr=" << t.tadr << " way=" << t.way << endl;
    return os;
}

// Trace...
void sc_trace (sc_trace_file *tf, const SCacheTag &t, const std::string &name) {
    PN_TRACE_R (tf, t, tadr, name);
    PN_TRACE_R (tf, t, valid, name);
    PN_TRACE_R (tf, t, dirty, name);
    PN_TRACE_R (tf, t, way, name);
}


// ***** SBusIfRegs *****

// Compare (necessary for signal updates)...
bool SBusIfRegs::operator ==(const SBusIfRegs &t) {
   return C(state) && C(op) && C(linelock) && C(bsel) && C(adr) && C(adr_ofs) && C(cnt) && C(banks_left) && C(last_banks_left) && C(tag)
           && C_ARR(idata_valid, CFG_MEMU_CACHE_BANKS) && C_ARR(idata, BUSIF_DATA_REG_NUM) && C_ARR(odata, CFG_MEMU_CACHE_BANKS);
}

// Display...
ostream &operator<<(std::ostream &o, const SBusIfRegs &t) {
    o << "{" << t.state << "}" ;
    return o;
}

// Tracing...
void sc_trace(sc_trace_file *tf, const SBusIfRegs &t, const std::string &name) {
    PN_TRACE_R (tf, t, state, name);
    PN_TRACE_R (tf, t, op, name);
    PN_TRACE_R (tf, t, linelock, name);
    PN_TRACE_R (tf, t, bsel, name);
    PN_TRACE_R (tf, t, adr, name);
    PN_TRACE_R (tf, t, adr_ofs, name);
    PN_TRACE_R_BUS (tf, t, idata_valid, name, CFG_MEMU_CACHE_BANKS);
    PN_TRACE_R_BUS (tf, t, idata, name, BUSIF_DATA_REG_NUM);
    PN_TRACE_R_BUS (tf, t, odata, name, CFG_MEMU_CACHE_BANKS);
    PN_TRACE_R (tf, t, cnt, name);
    PN_TRACE_R (tf, t, banks_left, name);
    PN_TRACE_R (tf, t, last_banks_left, name);
    PN_TRACE_R (tf, t, tag, name);
}


// **************** Helpers *********************

static inline TWord GetBankOfAdr (TWord adr) { return (adr >> 2) & ((1<<CFG_MEMU_CACHE_BANKS_LD) - 1); }
static inline TWord GetIndexOfAdr (TWord adr) {
    //PN_INFOF(("IndexOfAdr: %d",(adr >> (CFG_MEMU_CACHE_BANKS_LD + 2)) & ((1<<CFG_MEMU_CACHE_SETS_LD) - 1)));
    return (adr >> (CFG_MEMU_CACHE_BANKS_LD + 2)) & ((1<<CFG_MEMU_CACHE_SETS_LD) - 1);
}
static inline TWord GetWayIndexOfAdr (TWord adr, TWord way) {
    return GetIndexOfAdr (adr) | (way << CFG_MEMU_CACHE_SETS_LD);
}
static inline TWord GetIndexOfWayIndex (TWord adr) { return adr & ((1<<CFG_MEMU_CACHE_SETS_LD) - 1); }
static inline TWord GetTagOfAdr (TWord adr) {
    return adr >> (CFG_MEMU_CACHE_SETS_LD + CFG_MEMU_CACHE_BANKS_LD + 2);
}
static inline TWord GetLineOfAdr (TWord adr) { return adr >> (CFG_MEMU_CACHE_BANKS_LD + 2); }

static inline TWord ComposeAdress (TWord tadr, TWord iadr, TWord bank, TWord byteNo = 0) {
    return (tadr << (CFG_MEMU_CACHE_SETS_LD + CFG_MEMU_CACHE_BANKS_LD + 2)) |
           (iadr << (CFG_MEMU_CACHE_BANKS_LD + 2)) | (bank << 2) | byteNo;
}


static void RequestWait (sc_out<bool> *req, sc_in<bool> *gnt) {
    *req = 1;
    // wait (1, SC_NS); // (SC_ZERO_TIME);   // TBD: allow to wait for 0 cycles (arbiter may respond in the same cycle)
    // while (*gnt == 0) wait (gnt->value_changed_event ());
    while (*gnt == 0) wait ();
}


static inline void RequestRelease (sc_out<bool> *req) { *req = 0; }


// **************** MCacheBank ******************

void MBankRam::Trace(sc_trace_file *tf, int level)
{
    if (!tf || pn_trace_verbose) printf ("\nSignals of Module \"%s\":\n", name ());

    // Ports ...
    PN_TRACE (tf, clk);

    PN_TRACE_BUS (tf, rd, CFG_MEMU_BANK_RAM_PORTS);
    PN_TRACE_BUS (tf, wr, CFG_MEMU_BANK_RAM_PORTS);

    PN_TRACE_BUS (tf, wen, CFG_MEMU_BANK_RAM_PORTS);

    PN_TRACE_BUS (tf, wiadr, CFG_MEMU_BANK_RAM_PORTS);
    PN_TRACE_BUS (tf, wdata, CFG_MEMU_BANK_RAM_PORTS);
    PN_TRACE_BUS (tf, rdata, CFG_MEMU_BANK_RAM_PORTS);

}

void MBankRam::MainThread () {
    TWord combData;

    while (1) {
        // Simulated BankRam has a read first policy to match the
        // standard Hardware configuration
        for (int n = 0; n < CFG_MEMU_BANK_RAM_PORTS; n++)
            if (rd[n] == 1) rdata[n] = ram_[wiadr[n]];
        for (int n = 0; n < CFG_MEMU_BANK_RAM_PORTS; n++)
            if (wr[n] == 1) {
                combData = ram_[wiadr[n]];
                for (int i = 0; i < 4; i++) {
                    if (wen[n].read()[i] == 1) {
                        combData &= ~(0xff << (8 * i));
                        combData |= ( wdata[n] & (0xff << (8 * i)));
                    }
                }
                ram_[wiadr[n]] = combData;
            }
        wait ();
    }
}


// **************** MTagRam ********************

/*
bool STagEntry::operator== (const STagEntry& t) const {
  for (int n = 0; n < CFG_MEMU_CACHE_WAYS; n++)
    if (valid[n] != t.valid[n] || dirty[n] != t.dirty[n] || tadr[n] != t.tadr[n]) return false;
  if (use != t.use) return false;
  return true;
}


STagEntry& STagEntry::operator= (const STagEntry& t) {
  for (int n = 0; n < CFG_MEMU_CACHE_WAYS; n++) {
    valid[n] = t.valid[n];
    dirty[n] = t.dirty[n];
    tadr[n] = t.tadr[n];
  }
  use = t.use;
}


ostream& operator<< (ostream& os, const STagEntry& t) {
  for (int n = 0; n < CFG_MEMU_CACHE_WAYS; n++)
    os << "valid[" << n <<"]=" << t.valid[n] << " dirty[" << n <<"]=" << t.dirty[n] << " tadr[" << n
<<"]=" << t.tadr[n] << " "; os << "use=" << t.use << endl; return os;
}


void sc_trace (sc_trace_file *tf, const STagEntry& t, const std::string& name) {
  for (int n = 0; n < CFG_MEMU_CACHE_WAYS; n++) {
    sc_trace (tf, t.valid[n], name + ".valid[" + n + "]");
    sc_trace (tf, t.dirty[n], name + ".dirty[" + n + "]");
    sc_trace (tf, t.tadr[n], name + ".tadr[" + n + "]");
  }
  sc_trace (tf, t.use, name + ".use");
}
*/


void MTagRam::Trace (sc_trace_file *tf, int level) {
    if (!tf || pn_trace_verbose) printf ("\nSignals of Module \"%s\":\n", name ());

    // Ports ...
    PN_TRACE (tf, clk);
    PN_TRACE (tf, reset);
    PN_TRACE (tf, ready);

    PN_TRACE_BUS (tf, rd, TR_PORTS);
    PN_TRACE_BUS (tf, wr, TR_PORTS);

    PN_TRACE_BUS (tf, adr, TR_PORTS); // complete adress (lower CFG_MEMU_CACHE_BANKS_LD + 2 bits can be omitted)
    PN_TRACE_BUS (tf, wadr, TR_PORTS); 
    PN_TRACE_BUS (tf, tag_in, TR_PORTS);
    PN_TRACE_BUS (tf, tag_out, TR_PORTS);

    // Registers...
    PN_TRACE (tf, write_tag);
    PN_TRACE (tf, wtag_way);
    PN_TRACE (tf, wtag_iadr);
    PN_TRACE (tf, wtag_port);
    PN_TRACE_BUS (tf, use_reg, TR_PORTS+1)
    PN_TRACE_BUS (tf, use_iadr_reg, TR_PORTS+1);
    PN_TRACE_BUS (tf, use_wr_reg, TR_PORTS+1);
    PN_TRACE (tf, counter);
}


static TWord GetNewUse (TWord old_use, int way) {
    TWord new_use;
    int n;

    if (CFG_MEMU_CACHE_WAYS == 1)
        new_use = 0; // not really used
    else if (CFG_MEMU_CACHE_WAYS == 2)
        new_use = 1 - way;
    else if (CFG_MEMU_CACHE_WAYS == 4) {
        // The 'use' word is organized as follows:
        // - bits 1:0 represent the least recently used way (prio #0)
        // - bits 3:2 represent way with prio #1
        // - bits 5:4 represent way with prio #2 (future improvement: only use one bit for this)
        // - the most recently used way is not stored (can be derived)
        new_use = old_use;

        // Set missing (most recently used) way to bits 7:6;
        // the XOR of all numbers from 00..11 is 00. Hence XOR'ing 3 different 2-bit numbers results in the missing one.
        new_use |= (((new_use >> 4) ^ (new_use >> 2) ^ new_use) & 3) << 6;

        // Search for the current way in the string and remove it if found...
        for (n = 0; n < 6; n += 2)
            if (((new_use >> n) & 3) == way)
                new_use = (new_use & ((1 << n) - 1)) | ((new_use & ~((4 << n) - 1)) >> 2);

        // Remove the uppermost bits...
        new_use &= 0x3f;
    } else
        PN_ASSERTM (false,
                 "Only 1-, 2-, or 4-way associative caches are supported with LRU replacement");

    return new_use;
}


void MTagRam::MainThread () {
    SCacheTag tag;
    STagEntry entry;
    TWord new_use;
    int way, p, n, k;
    bool write_tag_var;

    // Reset...
    ready = 0;
    write_tag = 0;
    wtag_iadr = 0;
    wtag_way = 0;
    wtag_port = 0;
    if (CFG_MEMU_CACHE_REPLACE_LRU == 0)
        counter = 0xff;
    else
        for (n = 0; n < TR_PORTS; n++) use_wr_reg[n] = 0;


    for (n = 0; n < CFG_MEMU_CACHE_SETS; n++) {
        /* Reset of the cache tags deactivated for now - not present in hardware
        for (k = 0; k < CFG_MEMU_CACHE_WAYS; k++) {
            ram_[n].tag[k].valid = 0;
            ram_[n].tag[k].dirty = 0;
        } */
        if (CFG_MEMU_CACHE_REPLACE_LRU == 1)
            ram_[n].use = CFG_MEMU_CACHE_WAYS != 4 ? 0 : 0x24; // 4-way: 0x24 = 10.01.00b (see coding below)
    }

    // wait (CFG_MEMU_CACHE_SETS);
    wait (); // FPGAs can do the initialisation during personalisation
    ready = 1;

    // Main loop...
    while (true) {
        write_tag_var = 0;

        // Step random counter...
        if (CFG_MEMU_CACHE_REPLACE_LRU == 0)
            counter = GetNextLfsrState (counter.read (), 8, GetPrimePoly (8, 0)); // (counter + 1) & 0xff;

        // Read access...
        for (p = 0; p < TR_PORTS; p++) {
            if (rd[p] == 1) {
                entry = ram_[GetIndexOfAdr (adr[p])];
                way = -1; // no hit
                for (n = 0; n < CFG_MEMU_CACHE_WAYS; n++)
                    if (entry.tag[n].valid && entry.tag[n].tadr == GetTagOfAdr (adr[p])) {
                        PN_ASSERTF (way < 0, ("same data stored in two ways"));
                        way = n;
                    }
                if (way >= 0) { // we had a hit...
                    tag.valid = 1;
                    if (CFG_MEMU_CACHE_REPLACE_LRU == 1) {
                        new_use = GetNewUse (entry.use, way);
                        // PN_INFOF (("    port #%i cache hit for adress %x (iadr=%x, tadr=%x): way =
                        // %i, use = %x, new_use = %x", p, adr[p].read (), GetIndexOfAdr(adr[p].read ()), GetTagOfAdr(adr[p].read ()), way, entry.use, new_use));
                        if (new_use != entry.use) {
                            entry.use = new_use;
                            use_iadr_reg[p] = GetIndexOfAdr (adr[p]);
                            use_reg[p] = new_use;
                            use_wr_reg[p] = 1;
                        } else
                            use_wr_reg[p] = 0;
                    } // CFG_MEMU_CACHE_REPLACE_LRU == 1
                } else { // we had a miss...
                    tag.valid = 0;
                    if (CFG_MEMU_CACHE_REPLACE_LRU == 0)
                        way = counter & (CFG_MEMU_CACHE_WAYS - 1); // random replacement
                    else {
                        way = entry.use & (CFG_MEMU_CACHE_WAYS - 1);
                        // PN_INFOF (("    port #%i cache miss for adress %x (iadr=%x): way = %i", p, adr[p].read (), GetIndexOfAdr(adr[p].read ()), way));
                    }
                }
                tag.dirty = entry.tag[way].dirty;
                tag.tadr = entry.tag[way].tadr;
                tag.way = (TWord)way;
            }
            if (p == TR_PORTS-1 && rd_way == 1) {
                // Special read for BusIf (connected to last port)
                way = tag_in[TR_PORTS-1].read().way;
                tag.valid = entry.tag[way].valid;
                tag.dirty = entry.tag[way].dirty;
                tag.tadr = entry.tag[way].tadr;
                tag.way = (TWord)way;
            }
            tag_out[p] = tag;
        }

        // Writing ...
        p = -1;
        for (n = 0; n < TR_PORTS; n++)
            if (wr[n] == 1) {
                PN_ASSERTF (p == -1, ("multiple tag write signals asserted"));
                p = n;
                write_tag_var = 1;
            }
        if (p >= 0) {
            // Handle a write request...
            //  - Implemented in hardware with asymetric block ram (read port width > write port width)
            //  - For simulation purpose we just read a whole tag entry and replace the tag in one cycle
            entry = ram_[GetIndexOfAdr (wadr[p])];
            tag = tag_in[p].read ();
            way = tag.way;
            entry.tag[way].valid = tag.valid;
            entry.tag[way].dirty = tag.dirty;
            entry.tag[way].tadr = tag.tadr;

            ram_[GetIndexOfAdr (wadr[p])] = entry;

//            INFOF (("Write tag [%d]: wadr=0x%08x, iadr=0x%08x, way=%d, tadr=0x%08x, valid=%d, diry=%d", p,
//                    wadr[p].read (), GetIndexOfAdr (wadr[p]), way, tag.tadr, tag.valid, tag.dirty));

            wtag_iadr = GetIndexOfAdr (wadr[p]);
            wtag_way = way;
            wtag_port = p;
        } else if (CFG_MEMU_CACHE_REPLACE_LRU == 1) {
            // Check if we need to update lru information after a write last cycle
            if (write_tag == 1) {
                entry = ram_[use_iadr_reg[wtag_port]]; // 'entry' must have been saved in a register during the past read cycle!
                new_use = GetNewUse (entry.use, wtag_way);
                if (new_use != entry.use) {
                    use_iadr_reg[TR_PORTS] = GetIndexOfAdr (wtag_iadr);
                    use_reg[TR_PORTS] = new_use;
                    use_wr_reg[TR_PORTS] = 1;
                } else
                    use_wr_reg[TR_PORTS] = 0;
            }

            // No external write request: Write back some use info if pending...
            for (n = 0; n < TR_PORTS+1 && p < 0; n++)
                if (use_wr_reg[n] == 1) p = n;
            if (p >= 0) {
                entry = ram_[use_iadr_reg[p]]; // 'entry' must have been saved in a register during the past read cycle!
                // PN_INFOF (("    writing back tag use: iadr = %x, port #%i", use_iadr_reg[p].read (), p));
                entry.use = use_reg[p];

                ram_[use_iadr_reg[p]] = entry;
                use_wr_reg[p] = 0;
            }
        }

        write_tag = write_tag_var;

        wait (); // End of clock cycle
    }
}


// **************** MBusIf **********************

void MBusIf::Trace (sc_trace_file *tf, int level) {
    if (!tf || pn_trace_verbose) printf ("\nSignals of Module \"%s\":\n", name ());

    // Ports ...
    PN_TRACE (tf, clk);
    PN_TRACE (tf, reset);

    //   Bus interface (Wishbone)...
    PN_TRACE (tf, wb_cyc_o);
    PN_TRACE (tf, wb_stb_o);
    PN_TRACE (tf, wb_we_o);
    PN_TRACE (tf, wb_cti_o);
    PN_TRACE (tf, wb_bte_o);
    PN_TRACE (tf, wb_sel_o);
    PN_TRACE (tf, wb_ack_i);
    PN_TRACE (tf, wb_adr_o);
    PN_TRACE (tf, wb_dat_i);
    PN_TRACE (tf, wb_dat_o);

    //   Control inputs/outputs...
    PN_TRACE (tf, busif_op);
    PN_TRACE (tf, busif_nolinelock);
    PN_TRACE (tf, busif_bsel);
    PN_TRACE (tf, busif_busy);

    //   Control lines to Tag & Cache banks...
    PN_TRACE (tf, tag_rd);
    PN_TRACE (tf, tag_rd_way);
    PN_TRACE (tf, tag_wr);
    PN_TRACE_BUS (tf, bank_rd, CFG_MEMU_CACHE_BANKS);
    PN_TRACE_BUS (tf, bank_wr, CFG_MEMU_CACHE_BANKS);

    //   Adress & data busses...
    PN_TRACE (tf, adr_in);
    PN_TRACE (tf, adr_out);
    PN_TRACE_BUS (tf, data_in, CFG_MEMU_CACHE_BANKS);
    PN_TRACE_BUS (tf, data_out, CFG_MEMU_CACHE_BANKS);
    PN_TRACE_BUS (tf, data_out_valid, CFG_MEMU_CACHE_BANKS);
    PN_TRACE (tf, tag_in);
    PN_TRACE (tf, tag_out);

    //   Request & grant lines...
    PN_TRACE (tf, req_linelock);
    PN_TRACE (tf, req_tagw);
    PN_TRACE (tf, req_tagr);
    PN_TRACE_BUS (tf, req_bank, CFG_MEMU_CACHE_BANKS);
    PN_TRACE (tf, gnt_linelock);
    PN_TRACE (tf, gnt_tagw);
    PN_TRACE (tf, gnt_tagr);
    PN_TRACE_BUS (tf, gnt_bank, CFG_MEMU_CACHE_BANKS);

    // Registers...
    PN_TRACE (tf, regs);
    PN_TRACE (tf, next_regs);
    PN_TRACE (tf, state_trace);
}


void MBusIf::MainMethod () {
    if (reset) {
        SBusIfRegs rst;
        rst.state = BifIdle;
        rst.op = bioNothing;
        rst.linelock = 0;
        rst.adr = 0;
        rst.adr_ofs = 0;
        rst.banks_left = -1;
        rst.last_banks_left = -1;
        rst.bsel = 0;
        rst.cnt = 0;
        for (int n = 0; n < CFG_MEMU_CACHE_BANKS; ++n)
            rst.idata_valid[n] = 0;
        regs = rst;
    } else {
        regs = next_regs.read ();
    }
}


void MBusIf::TransitionMethod () {
    const SBusIfRegs regs_var = regs.read (); // Use the const specifier to check at compile time that regs_var is never written
    SBusIfRegs next_regs_var;
    SCacheTag tag_out_var;

    // Read registers...
    next_regs_var = regs_var;

    // Defaults for outputs...
    wb_cyc_o = 0;
    wb_stb_o = 0;
    wb_we_o = 0;
    wb_dat_o = 0;
    wb_cti_o = 0;
#if CFG_MEMU_BUSIF_WIDTH == 64
    wb_adr_o = regs_var.adr & ~4;
    wb_dat_o =  regs_var.adr[2] ? (regs_var.odata[GetBankOfAdr (regs_var.adr)], sc_uint<32>(0)) : (sc_uint<32>(0), regs_var.odata[GetBankOfAdr (regs_var.adr)]);
    wb_sel_o = regs_var.adr[2] ? (regs_var.bsel, sc_uint<4>(0)) : (sc_uint<4>(0), regs_var.bsel);
#else
    wb_adr_o = regs_var.adr;
    wb_dat_o = regs_var.odata[GetBankOfAdr (regs_var.adr)];
    wb_sel_o = regs_var.bsel;
#endif
    // registered feedback bus cycle only for 4-16 banks
    if (BUSIF_DATA_REG_NUM == 4)
        wb_bte_o = 0b01;
    else if (BUSIF_DATA_REG_NUM == 8)
        wb_bte_o = 0b10;
    else if (BUSIF_DATA_REG_NUM == 16)
        wb_bte_o = 0b11;
    else
        wb_bte_o = 0b00;

    tag_rd = 0;
    tag_rd_way = 0;
    tag_wr = 0;
    req_tagr = 0;
    req_tagw = 0;
    req_linelock = regs_var.linelock;
    for (int n = 0; n < CFG_MEMU_CACHE_BANKS; n++) {
        req_bank[n] = 0;
        bank_rd[n] = 0;
        bank_wr[n] = 0;
    }

    busif_busy = 1;

    tag_out_var = regs_var.tag;

    next_regs_var.last_banks_left = regs_var.banks_left;

    // State Machine...
    switch (regs_var.state) {
        // Idle state...
        case BifIdle: // 0
            busif_busy = 0;

            // Latch all inputs since the BUSIF requester may release the request after one clock cycle...
            next_regs_var.op = busif_op.read ();
            next_regs_var.bsel = busif_bsel.read ();
            next_regs_var.adr = adr_in.read ();
    //        next_regs_var.adr_ofs = GetBankOfAdr (adr_in.read ());
            next_regs_var.adr_ofs = 0; // TBD: The optmization above was deactivated because of the missing wrap mode in the Wishbone/AXI-Bridge hardware
            next_regs_var.cnt = 0;
            next_regs_var.banks_left = -1;
            for (int n = 0; n < CFG_MEMU_CACHE_BANKS; n++)
                next_regs_var.odata[n] = data_in[n].read (); // only needed for direct writes

//            switch (busif_op.read ()) {
            switch (regs_var.op) {
                case bioDirectRead:
                    busif_busy = 1;
                    wb_cyc_o = 1;
                    wb_stb_o = 1;
                    if (wb_ack_i.read () == 1) {
                        next_regs_var.idata[GetBankOfAdr (regs_var.adr) / (CFG_MEMU_BUSIF_WIDTH/32)] = wb_dat_i.read ();
                        next_regs_var.idata_valid[GetBankOfAdr (regs_var.adr)] = 1;
                        next_regs_var.state = BifDirectRead2;
                    } else
                        next_regs_var.state = BifDirectRead1;
                    break;
                case bioDirectWrite:
                    wb_cyc_o = 1;
                    wb_stb_o = 1;
                    wb_we_o = 1;
                    next_regs_var.state = BifDirectWrite;
                    break;
                case bioNothing:
                    // Do nothing, just wait...
                    break;
                default: // Includes all Cache operations (bioFlush, bioInvalidate, ...)
                    busif_busy = 1;
                    if (busif_op.read()[2]) { // Cache all operations....
                      next_regs_var.tag.way = 0;
                      next_regs_var.adr = 0;
                    }
                    next_regs_var.linelock = !busif_nolinelock;
                    next_regs_var.adr(1, 0) = 0; // Remove last two bits
                    if (busif_nolinelock)
                        next_regs_var.state = BifCacheRequestRTWait;
                    else
                        next_regs_var.state = BifCacheRequestLLWait;
                    break;
            } // switch (busif_op.read ())
            break;

        // Perform WB read cycle..
        case BifDirectRead1: // 1
            wb_cyc_o = 1;
            wb_stb_o = 1;
            if (wb_ack_i.read () == 1) {
                next_regs_var.idata[GetBankOfAdr (regs_var.adr) / (CFG_MEMU_BUSIF_WIDTH/32)] = wb_dat_i.read ();
                next_regs_var.idata_valid[GetBankOfAdr (regs_var.adr)] = 1;
                next_regs_var.state = BifDirectRead2;
            }
            break;

        case BifDirectRead2: // 2
            // Reset valid signal to avoid wrong BusIF hits
            next_regs_var.idata_valid[GetBankOfAdr (regs_var.adr)] = 0;
            next_regs_var.state = BifIdle;

            // Latch all inputs since the BUSIF requester may release the request after one clock cycle...
            next_regs_var.op = busif_op.read ();
            next_regs_var.bsel = busif_bsel.read ();
            next_regs_var.adr = adr_in.read ();
    //        next_regs_var.adr_ofs = GetBankOfAdr (adr_in.read ());
            next_regs_var.adr_ofs = 0; // TBD: The optmization above was deactivated because of the missing wrap mode in the Wishbone/AXI-Bridge hardware
            next_regs_var.cnt = 0;
            next_regs_var.banks_left = -1;
            for (int n = 0; n < CFG_MEMU_CACHE_BANKS; n++)
                next_regs_var.odata[n] = data_in[n].read (); // only needed for direct writes
            break;

        // Perform WB write cycle...
        case BifDirectWrite: // 3
            wb_cyc_o = 1;
            wb_stb_o = 1;
            wb_we_o = 1;
            // Set to BIO_NOTHING because the next instruction will be ready at the earliest
            // in the idle state (make sure we don't execute the same instruction twice)
            next_regs_var.op = bioNothing;
            if (wb_ack_i)
                next_regs_var.state = BifIdle;
            break;

        // Perform all cache-related operations...
        case BifCacheRequestLLWait: // 4
            if (gnt_linelock)
                next_regs_var.state = BifCacheRequestRTWait;
            break;

        case BifCacheRequestRTWait: // 5
            req_tagr = 1;
            tag_rd = 1;
            tag_rd_way = regs_var.op[2];
            if (gnt_tagr)
                next_regs_var.state = BifCacheReadTag;
            break;

        case BifCacheReadTag: // 6
            req_tagr = 1;
            tag_rd = 1;
            tag_rd_way = regs_var.op[2];
            // Read tag..
            next_regs_var.tag = tag_in.read ();
            if (regs_var.op[2]) next_regs_var.tag.way = regs_var.tag.way; // Preserve current way
            switch (regs_var.op) {
                case bioReplace:
                    next_regs_var.state = BifCacheReplaceReadIdata;
                    break;
                case bioWriteback:
                case bioFlush:
                case bioWritebackAll:
                case bioFlushAll:
                    if (next_regs_var.tag.dirty)
                        next_regs_var.state = BifCacheReadDirtyBanks;
                    else
                        next_regs_var.state = BifCacheWriteTag;
                    break;
                default:
                    next_regs_var.tag.dirty = 0; // Set dirty to 0 for bioInvalidate and bioInvalidateAll command
                    next_regs_var.state = BifCacheWriteTag;
                    break;
            }
            break;

        case BifCacheReplaceReadIdata: // 7
            // Read (new) bus data into 'idata' register...
            // ... allowing to serve a read request as early as possible
            // Always select full words/double words
            wb_cyc_o = 1;
            wb_stb_o = 1;
            wb_sel_o = -1; // Always read full (double) words
            wb_adr_o = (regs_var.adr(31, CFG_MEMU_CACHE_BANKS_LD+2), regs_var.adr_ofs, sc_uint<2>(0));
            if (BUSIF_DATA_REG_NUM >= 4 && BUSIF_DATA_REG_NUM <= 16) {
                // registered feedback bus cycle only for 4-16 banks
                if (regs_var.cnt == BUSIF_DATA_REG_NUM-1)
                    wb_cti_o = 0b111; // End-of-Burst
                else
                    wb_cti_o = 0b010; // Incrementing burst cycle
            }
            if (wb_ack_i) {
                // Select idata register based on adr_ofs_reg (for 64Bit width drop last bit)
                next_regs_var.idata[regs_var.adr_ofs(CFG_MEMU_CACHE_BANKS_LD-1, (CFG_MEMU_BUSIF_WIDTH/32)-1)] = wb_dat_i.read ();
                next_regs_var.idata_valid[regs_var.adr_ofs] = 1;
#if CFG_MEMU_BUSIF_WIDTH == 64
                next_regs_var.idata_valid[regs_var.adr_ofs + 1] = 1; // Validate 2 entrys per read
 #endif
                next_regs_var.adr = (regs_var.adr(31, CFG_MEMU_CACHE_BANKS_LD+2), regs_var.adr_ofs, sc_uint<2>(0));

                if (regs_var.cnt != BUSIF_DATA_REG_NUM-1) {
                    // Increment address offset and cnt...
                    // Note: Without the wrap mode cnt and adr_ofs are kind of redundant
                    next_regs_var.adr_ofs = regs_var.adr_ofs + (CFG_MEMU_BUSIF_WIDTH/32);
                    next_regs_var.cnt = regs_var.cnt + 1;
                } else {
                    // Finished reading...
                    next_regs_var.cnt = 0;
                    if (regs_var.tag.dirty)
                        next_regs_var.state = BifCacheReadDirtyBanks;
                    else
                        next_regs_var.state = BifCacheReplaceInvalidateTag;
                }
            }
            break;

        case BifCacheReadDirtyBanks: // 8
            // Read (old) cache bank into 'odata' register (with maximum parallelism)
            // 'bank_rd' also selects the 'data_in' mux input for banks in
            // the memu so it has to be kept up in the cycle after
            // 'gnt_bank' when data is finally arriving
            for (int n = 0; n < CFG_MEMU_CACHE_BANKS; n++) {
                req_bank[n] = regs_var.banks_left[n];
                bank_rd[n] = regs_var.last_banks_left[n];
            }

            // Write bank data to odata_reg...
            for (int n = 0; n < CFG_MEMU_CACHE_BANKS; n++)
                if (regs_var.banks_left[n] == 0 && regs_var.last_banks_left[n] == 1)
                    next_regs_var.odata[n] = data_in[n].read (); // data comes from cache bank

            // Check for bank gnt...
            // bank data will arrive in the next cycle!
            for (int n = 0; n < CFG_MEMU_CACHE_BANKS; n++)
                if (gnt_bank[n] == 1)
                    next_regs_var.banks_left[n] = 0;

            // Finished?...
            if (regs_var.banks_left == 0) {
                next_regs_var.banks_left = -1;
                if (regs_var.op == bioReplace)
                    next_regs_var.state = BifCacheReplaceInvalidateTag;
                else
                    next_regs_var.state = BifCacheWriteTag;
            }
            break;

        case BifCacheReplaceInvalidateTag: // 9
            // Write invalid tag (to make sure that concurrent readers do not receive invalid data)
            req_tagw = 1;
            tag_out_var.valid = 0;
            if (gnt_tagw) {
                tag_wr = 1;
                next_regs_var.state = BifCacheReplaceWriteBanks;
            }
            break;

        case BifCacheReplaceWriteBanks: // 10
            // Write data to cache banks; operate in parallel as much as possible
            for (int n = 0; n < CFG_MEMU_CACHE_BANKS; n++) {
                req_bank[n] = regs_var.banks_left[n];
                bank_wr[n] = regs_var.banks_left[n];
                // Check for bank gnt...
                if (gnt_bank[n] == 1)
                    next_regs_var.banks_left[n] = 0;
            }

            // Finished?...
            if (regs_var.banks_left == 0) {
                next_regs_var.banks_left = -1;
                next_regs_var.state = BifCacheWriteTag;
            }
            break;

        case BifCacheWriteTag: // 11
            // Write new tag...
            req_tagw = 1;
            if (/*!regs_var.op[3] &*/ regs_var.op[1])
                // Bit 1 is only set for bioInvalidate(All) and bioFlush(All) (and bioReplace,
                // but that is handled explicitly below)
                tag_out_var.valid = 0;
            if (regs_var.op == bioReplace) {
                tag_out_var.valid = 1;
                tag_out_var.tadr = GetTagOfAdr(regs_var.adr);
            }
            tag_out_var.dirty = 0;
            if (gnt_tagw) {
                if (!regs_var.op[3] & regs_var.op[1])
                    // Bit 3 is set for bioReplace and Bit 1 is only set for bioInvalidate(All) and bioFlush(All)
                    tag_wr = regs_var.tag.valid;
                else
                    tag_wr = 1;
                if (regs_var.tag.dirty)
                    next_regs_var.state = BifCacheWriteBackVictim;
                else {
                    // Reset valid signal to avoid wrong BusIF hits
                    for (int n = 0; n < CFG_MEMU_CACHE_BANKS; ++n)
                        next_regs_var.idata_valid[n] = 0;
                    next_regs_var.state = regs_var.op[2] ? BifCacheAll : BifCacheAck;
                }
            }
            break;

        case BifCacheWriteBackVictim: // 12
            // Write back victim data to bus...
            wb_cyc_o = 1;
            wb_stb_o = 1;
            wb_we_o = 1;
            wb_sel_o = -1; // Always write full (double) words
            if (BUSIF_DATA_REG_NUM >= 4 && BUSIF_DATA_REG_NUM <= 16) {
                // registered feedback bus cycle only for 4-16 banks
                if (regs_var.cnt == BUSIF_DATA_REG_NUM-1)
                    wb_cti_o = 0b111; // End-of-Burst
                else
                    wb_cti_o = 0b010; // Incrementing burst cycle
            }
            wb_adr_o = ComposeAdress (regs_var.tag.tadr, GetIndexOfAdr (regs_var.adr), regs_var.cnt, 0);
 #if CFG_MEMU_BUSIF_WIDTH == 64
            wb_dat_o = (sc_uint<CFG_MEMU_BUSIF_WIDTH>)(regs_var.odata[(regs_var.cnt<<1)+1], regs_var.odata[regs_var.cnt<<1]);
#else
            wb_dat_o = regs_var.odata[regs_var.cnt];
#endif
            if (wb_ack_i) {
                if (regs_var.cnt != BUSIF_DATA_REG_NUM-1) {
                    // Increment cnt...
                    next_regs_var.cnt = regs_var.cnt + 1;
                } else {
                    // Finished writing...
                    next_regs_var.cnt = 0;
                    // Reset valid signal to avoid wrong BusIF hits
                    for (int n = 0; n < CFG_MEMU_CACHE_BANKS; ++n)
                        next_regs_var.idata_valid[n] = 0;
                    next_regs_var.state = regs_var.op[2] ? BifCacheAll : BifCacheAck;
                }
            }
            break;

        case BifCacheAck: // 13
            busif_busy = 0;
            // Latch all inputs since the BUSIF requester may release the request after one clock cycle...
            next_regs_var.op = busif_op.read ();
            next_regs_var.bsel = busif_bsel.read ();
            next_regs_var.adr = adr_in.read ();
    //        next_regs_var.adr_ofs = GetBankOfAdr (adr_in.read ());
            next_regs_var.adr_ofs = 0; // TBD: The optmization above was deactivated because of the missing wrap mode in the Wishbone/AXI-Bridge hardware
            next_regs_var.cnt = 0;
            next_regs_var.banks_left = -1;
            for (int n = 0; n < CFG_MEMU_CACHE_BANKS; n++)
                next_regs_var.odata[n] = data_in[n].read (); // only needed for direct writes
            next_regs_var.linelock = 0;

            next_regs_var.state = BifIdle;
            break;
            
        case BifCacheAll: // 14
            // Loop over all cache sets...
            if (regs_var.tag.way == (1 << CFG_MEMU_CACHE_WAYS_LD) - 1) { 
              next_regs_var.adr(CFG_MEMU_CACHE_SETS_LD + CFG_MEMU_CACHE_BANKS_LD + 2, CFG_MEMU_CACHE_BANKS_LD + 2) = regs_var.adr(CFG_MEMU_CACHE_SETS_LD + CFG_MEMU_CACHE_BANKS_LD + 2, CFG_MEMU_CACHE_BANKS_LD + 2) + 1;
              next_regs_var.tag.way = 0;
            // and loop over all cache ways...
            } else {  
              next_regs_var.tag.way = regs_var.tag.way + 1;
            }
            // Finished? ...
            if (regs_var.adr(CFG_MEMU_CACHE_SETS_LD + CFG_MEMU_CACHE_BANKS_LD + 2, CFG_MEMU_CACHE_BANKS_LD + 2) == (1 << CFG_MEMU_CACHE_SETS_LD) -1 && regs_var.tag.way == (1 << CFG_MEMU_CACHE_WAYS_LD) - 1) {
               next_regs_var.state = BifCacheAck;
            } else
              next_regs_var.state = BifCacheRequestRTWait;
            break;

    } // switch (regs_var.state)

    // Write results...
    next_regs = next_regs_var;
    tag_out = tag_out_var;
    state_trace = regs_var.state;

#if CFG_MEMU_BUSIF_WIDTH == 64
    adr_out = regs_var.adr & ~4;
    for (int n = 0, j = 0; n < BUSIF_DATA_REG_NUM; n++, j+=2) {
//        PN_INFOF(("Output: %d=0x%08x, %d=0x%08x", j, (TWord)regs_var.idata[n].range(31, 0), j+1, (TWord)regs_var.idata[n].range(63, 32) ));
        data_out[j] = (TWord)regs_var.idata[n].range(31, 0);
        data_out[j+1] = (TWord)regs_var.idata[n].range(63, 32);
#else
    adr_out = regs_var.adr;
    for (int n = 0; n < BUSIF_DATA_REG_NUM; n++) {
        data_out[n] = (TWord)regs_var.idata[n];
#endif
    }
    for (int n = 0; n < CFG_MEMU_CACHE_BANKS; n++) data_out_valid[n] = regs_var.idata_valid[n];

}

// **************** MReadPort *******************

void MReadPort::Trace (sc_trace_file *tf, int level) {
    if (!tf || pn_trace_verbose) printf ("\nSignals of Module \"%s\":\n", name ());

    // Ports ...
    PN_TRACE (tf, clk);
    PN_TRACE (tf, reset);

    PN_TRACE (tf, port_rd);
    PN_TRACE (tf, port_direct);
    PN_TRACE (tf, port_ack);
    PN_TRACE (tf, port_adr);
    PN_TRACE (tf, port_data);
    PN_TRACE (tf, port_scond_ok);

    //   Towards BUSIF...
    PN_TRACE (tf, busif_adr);
    PN_TRACE (tf, busif_data);
    PN_TRACE_BUS (tf, busif_data_valid, CFG_MEMU_CACHE_BANKS);
    PN_TRACE (tf, busif_op);
    PN_TRACE (tf, busif_busy);

    //   Towards cache...
    //   - Adress information for tag/banks are routed around this module from 'port_adr'.
    PN_TRACE (tf, tag_rd);
    PN_TRACE (tf, bank_rd);
    PN_TRACE (tf, bank_data_in);
    PN_TRACE (tf, bank_sel);
    PN_TRACE (tf, tag_in);
    PN_TRACE (tf, way_out);

    //   Request & grant lines...
    PN_TRACE (tf, req_tagr);
    PN_TRACE_BUS (tf, req_bank, CFG_MEMU_CACHE_BANKS);
    PN_TRACE (tf, req_busif);
    PN_TRACE (tf, gnt_tagr);
    PN_TRACE_BUS (tf, gnt_bank, CFG_MEMU_CACHE_BANKS);
    PN_TRACE (tf, gnt_busif);

    //   From snoop unit (arbiter)...
    PN_TRACE (tf, snoop_adr);
    PN_TRACE (tf, snoop_stb);

    // Registers...
    PN_TRACE (tf, state_trace);
    PN_TRACE (tf, bank_sel_reg);
    PN_TRACE (tf, link_adr_reg);
    PN_TRACE (tf, link_valid_reg);

    // Internal signals...
    PN_TRACE (tf, busif_hit);
    PN_TRACE (tf, next_link_adr_reg);
    PN_TRACE (tf, next_link_valid_reg);
}


void MReadPort::HitMethod () {
    //TODO: Add busif_bsel to hit detection
#if CFG_MEMU_BUSIF_WIDTH == 64
    // Match address for 64bit aligned read
    busif_hit = busif_data_valid[GetBankOfAdr (port_adr)].read () && (port_adr.read () & ~0x4) == (busif_adr.read ());
#else
    busif_hit = busif_data_valid[GetBankOfAdr (port_adr)].read () && port_adr.read () == busif_adr.read ();
#endif
}


void MReadPort::TransitionMethod () {
    state_reg = next_state;
    state_trace = (int)next_state.read ();

    bank_sel_reg = next_bank_sel;

    link_adr_reg = next_link_adr_reg;
    link_valid_reg = next_link_valid_reg;
}


void MReadPort::MainMethod () {
    TWord index, bank;
    bool tagr_req_rd, bank_req_rd; // 'req' and 'rd' signals can be identical
    bool tagr_gnt, bank_gnt;
    int n;

    // Defaults for outputs...
    port_ack = 0;
    port_scond_ok = link_valid_reg && (link_adr_reg == port_adr);
    port_data = busif_data;
    req_busif = 0;
    for (n = 0; n < CFG_MEMU_CACHE_BANKS; n++) req_bank[n] = 0;
    busif_op = bioNothing;

    bank_sel = bank_sel_reg;
    way_out = tag_in.read ().way; // TBD: need a register for this?

    next_state = state_reg;
    next_bank_sel = bank_sel_reg;

    // Defaults for link registers...
    next_link_adr_reg = link_adr_reg;
    next_link_valid_reg = link_valid_reg;

    // Helper variable...
    index = GetIndexOfAdr (port_adr);
    bank = GetBankOfAdr (port_adr);
    tagr_req_rd = bank_req_rd = 0;
    tagr_gnt = gnt_tagr;
    bank_gnt = gnt_bank[bank];

    if (reset == 1) {
        next_state = s_rp_init;
        next_link_valid_reg = 0;
    } else
        switch (state_reg.read ()) {

        case s_rp_init: // 0
            // Initial state: On new request, initiate all actions for the first cycle
            busif_op = bioDirectRead;
            if (port_rd == 1) {
                if (port_direct == 1) {
                    // uncached memory access...
                    req_busif = 1;
                    next_state = s_rp_direct_wait_busif;
                } else {
                    // cached memory access...
                    if (busif_hit == 1) {
                        port_ack = 1; // next state must set "port_data = busif_data"
                    } else {
                        tagr_req_rd = 1;
                        if (gnt_tagr == 1)
                            next_state = s_rp_read_tag;
                    }
                    if (port_lres_scond == 1) {
                        // Reserve current adress (LR operation)...
                        next_link_adr_reg = port_adr;
                        next_link_valid_reg = 1;
                    }
                }
            }
            break;

        case s_rp_direct_wait_busif: // 1
            // Direct access: Wait for response from the BusIf
            busif_op = bioDirectRead;
            req_busif = !busif_hit;
            if (busif_hit == 1) {
                port_ack = 1; // next state must set "port_data = busif_data"
                next_state = s_rp_init;
            }
            break;

        case s_rp_read_tag: // 3
            // capture the tag (which was granted last state)
            tagr_req_rd = 1;
//            if (tagr_gnt == 1) {
                bank_req_rd = 1;
                if (tag_in.read ().valid == 1) {
                    // Cache hit...
                    if (bank_gnt == 1) {
                        port_ack = 1; // next state must set "port_data = bank_data_in"
                        next_bank_sel = bank;
                        next_state = s_rp_read_bank;
                    }
                } else
                    next_state = s_rp_miss_wait_busif;
//            }
            break;

        case s_rp_read_bank: // 4
            // read the bank & complete ...
//            tagr_req_rd = 1;

            port_data = bank_data_in;
            next_state = s_rp_init;

            // On new request, initiate all actions for the first cycle
            if (port_rd == 1) {
                if (port_direct == 1) {
                    // uncached memory access...
                    busif_op = bioDirectRead;
                    req_busif = 1;
                    next_state = s_rp_direct_wait_busif;
                } else {
                    // cached memory access...
                    if (busif_hit == 1) {
                        port_ack = 1; // next state must set "port_data = busif_data"
                    } else {
                        tagr_req_rd = 1;
                        next_state = s_rp_read_tag;
                    }
                }
            }
            break;

        case s_rp_miss_wait_busif: // 5
            // Cache miss detected: Wait until we get the BusIf and the BusIf becomes idle and catch
            // an incidental BusIf hit if it happens.
            req_busif = 1;
            if (busif_hit == 1) {
                port_ack = 1; // next state must set "port_data = busif_data"
                next_state = s_rp_init;
            } else if (gnt_busif == 1 && busif_busy == 0)
                next_state = s_rp_miss_request_tag;
            break;

        case s_rp_miss_request_tag: // 6
            // Request the tag again to check for a cache hit. The BusIf might already have replaced the
            // cache line during 's_rp_miss_wait_busif' on the request of some other port, and that data may have
            // even been modified by a write port! Hence, replacing that cache line again would lead to wrong data.
            req_busif = 1;
            tagr_req_rd = 1;
            if (gnt_tagr == 1)
                next_state = s_rp_miss_read_tag;
            break;

        case s_rp_miss_read_tag: // 7
            // read the tag & check for a cache hit
            req_busif = 1;
            tagr_req_rd = 1;
            if (tagr_gnt == 1) {
                bank_req_rd = 1;
                if (tag_in.read ().valid == 1) {
                    if (bank_gnt == 1) {
                        next_bank_sel = bank;
                        port_ack = 1; // next state must set "port_data = bank_data_in"
                        next_state = s_rp_read_bank;
                    }
                } else
                    next_state = s_rp_miss_replace;
            }
            break;

        case s_rp_miss_replace: // 8
            // Run the replacement and wait for the BusIf hit which MUST come some time
            req_busif = 1;
            busif_op = bioReplace;
            if (busif_hit == 1) {
                port_ack = 1; // next state must set "port_data = busif_data"
                next_state = s_rp_init;
            }
            break;

        } // switch (state_reg)

    // Need to invalidate link_valid_reg?
    if (snoop_stb == 1) {
        if (snoop_adr == link_adr_reg || snoop_adr == next_link_adr_reg) { // need to invalidate?
            next_link_valid_reg = 0;
        }
    }

    // Set derived outputs...
    req_tagr = tagr_req_rd;
    tag_rd = tagr_req_rd;
    req_bank[bank] = bank_req_rd;
    bank_rd = bank_req_rd;
}


// **************** MWritePort *******************


void MWritePort::Trace (sc_trace_file *tf, int level) {
    if (!tf || pn_trace_verbose) printf ("\nSignals of Module \"%s\":\n", name ());

    // Ports ...
    PN_TRACE (tf, clk);
    PN_TRACE (tf, reset);

    //   Towards (CPU) port...
    //   - All input ports must be held until 'port_ack' is asserted.
    PN_TRACE (tf, port_wr);
    PN_TRACE (tf, port_direct);
    PN_TRACE (tf, port_bsel);
    PN_TRACE (tf, port_ack);

    PN_TRACE (tf, port_lres_scond);

    PN_TRACE (tf, port_cache_op);

    PN_TRACE (tf, port_adr);
    PN_TRACE (tf, port_data);

    //   Towards BUSIF...
    //   - There is no direct transfer except for uncached memory access.
    PN_TRACE (tf, busif_adr);
    PN_TRACE (tf, busif_op);
    PN_TRACE (tf, busif_nolinelock);
    PN_TRACE (tf, busif_busy);

    //   Towards cache...
    //   - Adress information for tag/banks must be routed from 'port_adr'.
    PN_TRACE (tf, tag_rd);
    PN_TRACE (tf, tag_wr);
    PN_TRACE (tf, bank_rd);
    PN_TRACE (tf, bank_wr);
    PN_TRACE (tf, bank_data_in);
    PN_TRACE (tf, bank_data_out);
    PN_TRACE (tf, bank_bsel);
    PN_TRACE (tf, tag_in);
    PN_TRACE (tf, tag_out);

    //   Request & grant lines...
    PN_TRACE (tf, req_linelock);
    PN_TRACE (tf, req_tagr);
    PN_TRACE (tf, req_tagw);
    PN_TRACE_BUS (tf, req_bank, CFG_MEMU_CACHE_BANKS);
    PN_TRACE (tf, req_busif);
    PN_TRACE (tf, gnt_linelock);
    PN_TRACE (tf, gnt_tagr);
    PN_TRACE (tf, gnt_tagw);
    PN_TRACE_BUS (tf, gnt_bank, CFG_MEMU_CACHE_BANKS);
    PN_TRACE (tf, gnt_busif);

    // Internal register & signals...
    PN_TRACE (tf, state_trace);
    PN_TRACE (tf, tag_reg);
    PN_TRACE (tf, data_reg);

    PN_TRACE (tf, next_tag_reg);
    PN_TRACE (tf, next_data_reg);
}


void MWritePort::TransitionMethod () {
    state_reg = next_state;
    state_trace = (int)next_state.read ();

    tag_reg = next_tag_reg;
    data_reg = next_data_reg;
}


static TWord CombineData (sc_uint<4> bsel, TWord word0, TWord word1) {
    TWord ret;

    ret = word1;
#if PN_BIG_ENDIAN == 1
    if (bsel != 0xf) { // full word skipped (just to improve simulation speed)
        for (int n = 0; n < 4; n++)
            if (bsel[3 - n] == 0) {
                ret &= ~(0xff << (8 * n));
                ret |= (word0 & (0xff << (8 * n)));
            }
    }
#else
    if (bsel != 0xf) { // full word skipped (just to improve simulation speed)
        for (int n = 0; n < 4; n++)
            if (bsel[n] == 0) {
                ret &= ~(0xff << (8 * n));
                ret |= (word0 & (0xff << (8 * n)));
            }
    }
#endif
    return ret;
}


void MWritePort::MainMethod () {
    SCacheTag tag;
    TWord index, bank;
    bool tagr_req_rd, bank_req_rd; // 'req' and 'rd' signals can be identical
    bool bank_gnt;

    int n;

    // To avoid over-length paths, 'gnt_*' inputs only influence the next state, but not the outputs of this module.
    // Besides this, this is a Mealy-type machine for performance reasons. (Do we need more restrictions?)

    // Defaults for outputs...
    port_ack = 0;

    busif_op = bioNothing;
    busif_nolinelock = 1;

    tag_wr = 0;
    bank_wr = 0;
    tag_out = tag_reg;
    bank_data_out = port_data;
    bank_bsel = port_bsel;

    req_linelock = 0;
    req_tagr = 0;
    req_tagw = 0;
    for (n = 0; n < CFG_MEMU_CACHE_BANKS; n++) req_bank[n] = 0;
    req_busif = 0;

    next_state = state_reg;
    next_tag_reg = tag_reg;
    next_data_reg = data_reg;

    // Helper variables...
    index = GetIndexOfAdr (port_adr);
    bank = GetBankOfAdr (port_adr);
    tagr_req_rd = bank_req_rd = 0;
    bank_gnt = gnt_bank[bank];

    // Main "switch"...
    if (reset == 1) {
        next_state = s_wp_init;
    } else
        switch (state_reg.read ()) {

        case s_wp_init: // 0
            if (port_wr == 1) {
                if (port_direct == 1) {
                    // Direct (uncached) memory access...
                    req_busif = 1;
                    busif_op = bioDirectWrite;
                    if (gnt_busif == 1 && busif_busy == 0) next_state = s_wp_direct;
                } else {
                    // Normal (cached) access...
                    if (port_lres_scond == 1 && port_scond_ok == 0) {
                        // This is a "store conditional" (SC), which has failed right away...
                        // (a failure may still occur later, even if this condition has not yet been fulfilled here)
                        port_ack = 1;
                    } else {
                        // Normal and (so far) successful SC cache accesses...
                        req_linelock = 1;
                        tagr_req_rd = 1;
                        if (gnt_tagr == 1) {
                            if (gnt_linelock == 1)
                                next_state = s_wp_read_tag;
                            else
                                next_state = s_wp_request_linelock_only;
                        }
                    }
                }
            } else if (port_cache_op.read()[0] == 1 || port_cache_op.read()[1] == 1) {
                // Special operation...
                req_busif = 1;
                req_linelock = 1;
                if (gnt_linelock == 1 && gnt_busif == 0)
                    next_state = s_wp_special_request_busif_only;
                if (gnt_busif == 1 && busif_busy == 0) next_state = s_wp_special;
            }
            break;

            // Additional state for direct memory access...

        case s_wp_direct: // 1
            // Issue "direct write" operation...
            req_busif = 1;
            busif_op = bioDirectWrite;
            if (busif_busy == 1) {
                // Now the BUSIF is busy and has captured the data -> can ack and complete
                port_ack = 1;
                next_state = s_wp_init;
            }
            break;

            // Additional states for normal access...

        case s_wp_request_linelock_only: // 2
            // We got a grant for 'tagr', but not the 'linelock'
            // => We must release everything except the 'linelock' to avoid a deadlock
            req_linelock = 1;
            if (gnt_linelock == 1) next_state = s_wp_init;
            break;

        case s_wp_read_tag: // 3
            // Capture the tag and request the bank...
            req_linelock = 1;
            tagr_req_rd = 1;
            tag_out = tag_in; // for bank reading to output the correct cache way
            next_tag_reg = tag_in;
//            if (gnt_tagr == 1) {
                if (tag_in.read ().valid == 1) {
                    // Cache hit...
                    if (tag_in.read ().dirty == 1)
                        next_state = s_wp_write_bank;
                    else
                        next_state = s_wp_write_tag1_and_bank;
                } else
                    next_state = s_wp_miss;
//            }
            break;

        case s_wp_write_tag1_and_bank: // 4
            req_linelock = 1;
            req_tagw = 1;
            tag = tag_reg;
            tag.dirty = 1;
            tag_out = tag;
            bank_req_rd = 1;
            if (bank_gnt == 1) {
                bank_wr = 1;
                next_state = s_wp_write_tag1;
            }
            if (gnt_tagw == 1) {
                tag_wr = 1;
                next_state = s_wp_write_bank;
            }
            if (bank_gnt == 1 && gnt_tagw == 1) {
                port_ack = 1; // can acknowledge to port now (write to bank and tag must be committed!)
//                req_linelock = 0; // avoid that this write port monopolizes the line lock
                bank_wr = 1;
                next_state = s_wp_init;
            }
            break;

        case s_wp_write_tag1: // 5
            req_linelock = 1;
            req_tagw = 1;
            tag = tag_reg;
            tag.dirty = 1;
            tag_out = tag;
            if (gnt_tagw == 1) {
                tag_wr = 1;
                port_ack = 1;
//                req_linelock = 0; // avoid that this write port monopolizes the line lock (code below "Handle new access" can be simplified accordingly)
                next_state = s_wp_init;
            }
            break;

        case s_wp_write_bank: // 6
            req_linelock = 1;
            bank_req_rd = 1;
            bank_wr = 1;
            if (bank_gnt == 1) {
                bank_wr = 1;
                port_ack = 1; // can acknowledge to port now (write to bank must be committed!)
//                req_linelock = 0; // avoid that this write port monopolizes the line lock
                next_state = s_wp_init;
            }
            // Can we accept a new request already in this state?
            // -> No, 'tagr' must not be requested while bank is held (deadlock)
            break;

            // The following states handle a cache miss & replace a cache line.

        case s_wp_miss: // 7
            // Entry state for a cache miss. First, we must request acquire the BusIf and potentially re-acquire the line lock...
            req_busif = 1;
            req_linelock = 1;
            if (gnt_busif == 1 && busif_busy == 0 && gnt_linelock == 1)
                next_state = s_wp_recheck;
            else if (gnt_busif == 0 && gnt_linelock == 1)
                next_state = s_wp_request_busif_only;
            break;

        case s_wp_request_busif_only: // 8
            // Release the line lock and request the BusIf only to avoid deadlocks.
            req_busif = 1;
            if (gnt_busif == 1) next_state = s_wp_miss;
            break;

        case s_wp_recheck: // 9
            // Now we have the BusIf and the line lock, and the BusIf is idle. We must re-check if there is a cache hit now, since
            // some other port may have replaced the cache line in between.
            req_busif = 1;
            req_linelock = 1;
            tagr_req_rd = 1;
            if (gnt_tagr == 1)
                next_state = s_wp_recheck_read_tag;
            break;

        case s_wp_recheck_read_tag: // 10
            // Capture the tag and check it for a cache hit.
            tagr_req_rd = 1;
            req_busif = 1;
            req_linelock = 1;
            next_tag_reg = tag_in;
//            if (gnt_tagr == 1) {
                if (tag_in.read ().valid == 1) {
                    if (tag_in.read ().dirty == 1)
                        next_state = s_wp_write_bank;
                    else
                        next_state = s_wp_write_tag1_and_bank;
                } else
                    next_state = s_wp_replace;
//            }
            break;

        case s_wp_replace: // 11
            // Start the replacement by the BusIf.
            req_busif = 1;
            req_linelock = 1;
            busif_op = bioReplace;
            if (busif_busy == 1) next_state = s_wp_replace_wait_busif;
            break;

        case s_wp_replace_wait_busif: // 12
            // Wait for the BusIf to complete the replacement.
            req_busif = 1;
            req_linelock = 1;
            if (busif_busy == 0) {
                tagr_req_rd = 1;
                if (gnt_tagr == 1)
                    next_state = s_wp_read_tag;
                else
                    next_state = s_wp_init;
            }
            break;

            // States for special operations...

        case s_wp_special_request_busif_only: // 13
            req_busif = 1;
            if (gnt_busif == 1) next_state = s_wp_init;
            break;

        case s_wp_special: // 14
            req_busif = 1;
            req_linelock = 1;
            busif_op = port_cache_op.read ();
            if (busif_busy == 1) next_state = s_wp_special_wait_complete;
            break;

        case s_wp_special_wait_complete: // 15
            req_busif = 1;
            req_linelock = 1;
            if (busif_busy == 0) {
                // The BUSIF has completed -> can ack and complete
                port_ack = 1;
                next_state = s_wp_init;
            }
            break;

        } // switch (state_reg)

    // Set derived outputs...
    req_tagr = tagr_req_rd;
    tag_rd = tagr_req_rd;
    req_bank[bank] = bank_req_rd;
    bank_rd = bank_req_rd;
}


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

        for (int b = 0; b < CFG_MEMU_CACHE_BANKS; b++) {
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

void MArbiter::PrioCPUMethod () {
    cpu_prio = GetPrioCpu ();
}

void MArbiter::LineLockMethod () {
    sc_uint<CFG_MEMU_WPORTS + 1> req_linelock, gnt_linelock;
    SSelectorIO<1, CFG_MEMU_WPORTS> linelock_wport_sel_in [CFG_MEMU_WPORTS-1];
    int n, i;

    // Current policy (to save area):
    // - WPORT requests always exclude each other, indepent of the index adress
    // - concurrent BUSIF and WPORT grants are possible, if they adress different lines

    // Collect all request signals...
    req_linelock[CFG_MEMU_WPORTS] = req_busif_linelock;
    for (n = 0; n < CFG_MEMU_WPORTS; n++) req_linelock[n] = req_wp_linelock[n];

    // Determine existing & to-keep grants...
    gnt_linelock = linelock_reg.read () & req_linelock;

    // Handle BUSIF request (highest priority)...
    if (req_linelock[CFG_MEMU_WPORTS] & !gnt_linelock[CFG_MEMU_WPORTS]) {
        gnt_linelock[CFG_MEMU_WPORTS] = 1;
        for (n = 0; n < CFG_MEMU_WPORTS; n++)
            if (gnt_linelock[n] && GetIndexOfAdr (adr_wp[n]) == GetIndexOfWayIndex (wiadr_busif))
                gnt_linelock[CFG_MEMU_WPORTS] = 0;
    }

    // Set selector inputs...
    for (n = 0; n < CFG_MEMU_WPORTS; n++) {
        SSelectorIO<1, CFG_MEMU_WPORTS> sel_in (0, n, 0);
        // Make sure to only request a grant if the cache line is different from the BusIf line...
        if (req_linelock[n] == 1 && (!gnt_linelock[CFG_MEMU_WPORTS] || GetIndexOfAdr (adr_wp[n]) != GetIndexOfWayIndex (wiadr_busif)))
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
    for (n = 0; n < CFG_MEMU_WPORTS-1; n++) {
        i = (n + GetPrioCpu ()) % (CFG_MEMU_WPORTS-1);
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
    for (n = 0; n < CFG_MEMU_WPORTS; n++) gnt_wp_linelock[n] = gnt_linelock[n];
}


void MArbiter::TagMethod () {
    sc_uint<CFG_MEMU_RPORTS + CFG_MEMU_WPORTS + 1> req_tagr, gnt_tagr;
    sc_uint<CFG_MEMU_WPORTS + 1> req_tagw, gnt_tagw;
    int n, i;

    if (tagram_ready == 0) { // Tag RAM not yet ready...
        gnt_tagr = 0;
        gnt_tagw = 0;
    } else {
        // Collect all request signals...
        for (n = 0; n < CFG_MEMU_RPORTS; n++) req_tagr[n] = req_rp_tagr[n];
        for (n = 0; n < CFG_MEMU_WPORTS; n++) {
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
            for (n = 0; n < CFG_NUT_CPU_CORES; n++) { // Select highest priority acquired bit for each CPU
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
                for (n = 0; n < CFG_MEMU_WPORTS; n++) {
                    i = (n + GetPrioCpu ()) % CFG_MEMU_WPORTS;
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

    for (n = 0; n < CFG_MEMU_RPORTS; n++) {
        gnt_rp_tagr[n] = gnt_tagr[n];
        gnt_rp_tagr_r[n] = tagr_reg.read ()[n];
    }
    for (n = 0; n < CFG_MEMU_WPORTS; n++) {
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


void MArbiter::BankMethod () {
    sc_uint<((CFG_MEMU_RPORTS+CFG_MEMU_WPORTS+SINGLE_CPU) / CFG_MEMU_BANK_RAM_PORTS) + 1> req_bank[CFG_MEMU_BANK_RAM_PORTS], gnt_bank[CFG_MEMU_BANK_RAM_PORTS];
    TWord sel_wiadr[CFG_MEMU_BANK_RAM_PORTS]/*, wiadr[CFG_MEMU_BANK_RAM_PORTS][(CFG_MEMU_RPORTS+CFG_MEMU_WPORTS+SINGLE_CPU) / CFG_MEMU_BANK_RAM_PORTS + 1]*/;
    // For only one Bankram port all CPUs get arbitrated to the BusIfSelector
    SSelectorIO<CFG_MEMU_CACHE_WAYS_LD+CFG_MEMU_CACHE_SETS_LD, (CFG_MEMU_RPORTS+CFG_MEMU_WPORTS+SINGLE_CPU)/CFG_MEMU_BANK_RAM_PORTS> bank_cpu_sel_in [CFG_MEMU_BANK_RAM_PORTS][CFG_NUT_CPU_CORES/CFG_MEMU_BANK_RAM_PORTS];
    int sel_br[CFG_MEMU_BANK_RAM_PORTS];
    int n, b, i, p;

//    // Collect all way & index adresses and assign to port...
//    for (n = 0; n < CFG_NUT_CPU_CORES; n++) {
//        wiadr[RAMPORT(n)][IDX_RP (n)] = wiadr_rp[n];
//        wiadr[RAMPORT(n)][IDX_IP (n)] = wiadr_rp[CFG_NUT_CPU_CORES + n];
//        wiadr[RAMPORT(n)][IDX_WP (n)] = GetWayIndexOfAdr (adr_wp[n], way_wp[n]);
//    }
//    if (!SINGLE_CPU) wiadr[0][IDX_BUSIF] = 0xffffffff;  // Port 0 has one less input - don't care in hardware
//    wiadr[RAMPORT(IDX_BUSIF)][IDX_BUSIF] = wiadr_busif;

    for (b = 0; b < CFG_MEMU_CACHE_BANKS; b++) {

        // Collect all request signals for this bank, sorted the same way as wiadr...
        for (n = 0; n < CFG_NUT_CPU_CORES; n++) {
            req_bank[RAMPORT(n)][IDX_RP (n)] = req_rp_bank[n][b];
            req_bank[RAMPORT(n)][IDX_IP (n)] = req_rp_bank[CFG_NUT_CPU_CORES + n][b];
            req_bank[RAMPORT(n)][IDX_WP (n)] = req_wp_bank[n][b];
        }
        if (!SINGLE_CPU) req_bank[0][IDX_BUSIF] = 0; // Again port 0 has one less input - don't care in hardware
        req_bank[RAMPORT(IDX_BUSIF)][IDX_BUSIF] = req_busif_bank[b];

        //  All bank accesses are done in 1 cycle, no need to keep track of already granted ports......
        for (p = 0; p < CFG_MEMU_BANK_RAM_PORTS; p++)
            gnt_bank[p] = 0;

        // Preset sel_port and sel_br...
        for (n = 0; n < CFG_NUT_CPU_CORES; n++);
        for (p = 0; p < CFG_MEMU_BANK_RAM_PORTS; p++) sel_br[p] = -1;

        // Determine the selected ports per CPU...
        for (n = 0; n < CFG_NUT_CPU_CORES; n++) {
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
                sel_in.dat = GetWayIndexOfAdr (adr_wp[n], way_wp[n]);
                sel_in.sel_valid = 1;
            }
            if (n == 0 && CFG_MEMU_BANK_RAM_PORTS > 1)
                // At 2 ports CePU gets its own selector...
                bank_sel_in[b][0][0] = sel_in;
            else
                bank_cpu_sel_in[RAMPORT(n)][IDX_OF_CPU_IN_PORT(n)-(CFG_MEMU_BANK_RAM_PORTS > 1 && RAMPORT(n) == 0)] = sel_in;
        }
        // BusIf always gets the fast input of the BankBusIfSel...
        {
            SSelectorIO<CFG_MEMU_CACHE_WAYS_LD+CFG_MEMU_CACHE_SETS_LD, (CFG_MEMU_RPORTS+CFG_MEMU_WPORTS+SINGLE_CPU)/CFG_MEMU_BANK_RAM_PORTS> sel_in (wiadr_busif.read (), IDX_BUSIF, req_bank[RAMPORT(IDX_BUSIF)][IDX_BUSIF]);
            bank_sel_in[b][CFG_MEMU_BANK_RAM_PORTS-1][0] = sel_in;
        }

        // Select slow input to selectors...
        for (p = 0; p < CFG_MEMU_BANK_RAM_PORTS; p++) {
            bank_sel_in[b][p][1] = bank_cpu_sel_in[p][0];
            for (n = 0; n < CFG_NUT_CPU_CORES/CFG_MEMU_BANK_RAM_PORTS-(CFG_MEMU_BANK_RAM_PORTS > 1 && p == 0); n++) {
                i = (n + GetPrioCpu ()) % (CFG_NUT_CPU_CORES/CFG_MEMU_BANK_RAM_PORTS-(CFG_MEMU_BANK_RAM_PORTS > 1 && p == 0));
                if (bank_cpu_sel_in[p][i].sel_valid) {
                    bank_sel_in[b][p][1] = bank_cpu_sel_in[p][i];
                    break;
                }
            }
        }

        // Find selected port & grant...
        for (p = 0; p < CFG_MEMU_BANK_RAM_PORTS; p++) {
            sel_wiadr[p] = 0xffffffff; // should be don't care
            // New grant
            if (bank_sel_out[b][p].read ().sel_valid == 1) {
                sel_br[p] = bank_sel_out[b][p].read ().sel;
                sel_wiadr[p] = bank_sel_out[b][p].read ().dat;
                gnt_bank[p][sel_br[p]] = 1;
            }
        }

        // Deactivated parallel execution for now
//        for (n = 0; n < CFG_MEMU_RPORTS + CFG_MEMU_WPORTS + 1; n++) {
//            p = RAMPORT (n);
//            if (sel_port[p] >= 0 && req_bank[n] == 1 && wiadr[n] == sel_wiadr[p]) gnt_bank[n] = 1;
//            // PN_INFOF (("  n = %i, p = %i, sel_port[p] = %i, sel_wiadr[p] = %x, req_bank[n] = %i, wiadr[n] = %x",
//            //        n, p, sel_port[p], sel_wiadr[p], (int) req_bank[n], wiadr[n]));
//        }

        // PN_INFOF (("a) req_bank = %x, gnt_bank = %x, sel_port = { %i, %i }, sel_wiadr = { %x, %x }",
        //        (TWord) req_bank.value (), (TWord) gnt_bank.value (), sel_port[0], sel_port[1], sel_wiadr[0], sel_wiadr[1]));
        // cout << gnt_bank << endl;

        // PN_INFOF (("b) req_bank = %x, gnt_bank = %x, sel_port = { %i, %i }, sel_wiadr = { %x, %x }",
        //        (TWord) req_bank.value (), (TWord) gnt_bank.value (), sel_port[0], sel_port[1], sel_wiadr[0], sel_wiadr[1]));

        // Write results...
        for (p = 0; p < CFG_MEMU_BANK_RAM_PORTS; p++)
            next_bank_reg[b][p] = gnt_bank[p];

        for (n = 0; n < CFG_NUT_CPU_CORES; n++) {
            gnt_rp_bank[n][b] = gnt_bank[RAMPORT(n)][IDX_RP (n)];
            gnt_rp_bank[CFG_NUT_CPU_CORES + n][b] = gnt_bank[RAMPORT(n)][IDX_IP (n)];
            gnt_wp_bank[n][b] = gnt_bank[RAMPORT(n)][IDX_WP (n)];
        }
        gnt_busif_bank[b] = gnt_bank[RAMPORT(IDX_BUSIF)][IDX_BUSIF];

        for (p = 0; p < CFG_MEMU_BANK_RAM_PORTS; p++) wiadr_bank[b][p] = sel_wiadr[p];

        // Check result...
        // for (n = 0; n < CFG_MEMU_RPORTS+CFG_MEMU_WPORTS+1; n++) {
        //  p = RAMPORT(n);
        //  PN_ASSERTF (gnt_bank[n] == 0 || sel_wiadr[p] == wiadr[n], ("grant for cache bank %i given to port %i, but sel_wiadr[%i] = %x and wiadr[%i] = %x\n", b, n, p, sel_wiadr[p], n, wiadr[n]));
        //}
    }
}


void MArbiter::BusIfMethod () {
    sc_uint<CFG_MEMU_RPORTS + CFG_MEMU_WPORTS> req_busif, gnt_busif;
    SSelectorIO<1, (CFG_MEMU_RPORTS+CFG_MEMU_WPORTS+1)> busif_cpu_sel_in [CFG_NUT_CPU_CORES-1];
    int busif_sel;
    int n, i;

    // Collect all request signals...
    for (n = 0; n < CFG_MEMU_RPORTS; n++) req_busif[n] = req_rp_busif[n];
    for (n = 0; n < CFG_MEMU_WPORTS; n++) req_busif[CFG_MEMU_RPORTS + n] = req_wp_busif[n];

    // Determine existing & to-keep grants...
    gnt_busif = busif_reg.read () & req_busif;

    // Handle new requests...
    busif_sel = CFG_MEMU_RPORTS+CFG_MEMU_WPORTS;
    for (n = 0; n < CFG_NUT_CPU_CORES; n++) {
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
    for (n = 0; n < CFG_NUT_CPU_CORES-1; n++) {
        i = (n + GetPrioCpu ()) % (CFG_NUT_CPU_CORES-1);
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

    for (n = 0; n < CFG_MEMU_RPORTS; n++) gnt_rp_busif[n] = gnt_busif[n];
    for (n = 0; n < CFG_MEMU_WPORTS; n++) gnt_wp_busif[n] = gnt_busif[CFG_MEMU_RPORTS + n];
}


void MArbiter::SnoopMethod () {
    int n, writer = -1;

    // Determine a/the writer...
    //   NOTE: only cached writes are supported for snooping (LL/SC)
    for (n = 0; n < CFG_MEMU_WPORTS; n++) {
        if (next_linelock_reg.read ()[n] == 1) { // to catch a writer to the cache
            PN_ASSERT (writer < 0); // there should be only one!!
            writer = n;
        }
    }

    // Generate output signals...
    if (writer >= 0) {
        snoop_adr = adr_wp[writer];
        for (n = 0; n < CFG_MEMU_WPORTS; n++) snoop_stb[n] = 1; // signal to all CPUs ...
        snoop_stb[writer] = 0; // ... except the one that caused the write to avoid race condition
    } else {
        snoop_adr = 0xffffffff; // don't care
        for (n = 0; n < CFG_MEMU_WPORTS; n++) snoop_stb[n] = 0;
    }
}


int MArbiter::GetPrioCpu () {
#if CFG_MEMU_ARBITER_METHOD >= 0 // round robin...
    return (counter_reg.read () >> CFG_MEMU_ARBITER_METHOD) % CFG_NUT_CPU_CORES;
#else // LFSR...
    return counter_reg.read () % CFG_NUT_CPU_CORES;
#endif
}


void MArbiter::TransitionThread () {
    // Reset...
    counter_reg = 0xffff;
    linelock_reg = 0;

    // Main loop...
    while (1) {
        wait ();

        // 'counter_reg'...
#if CFG_MEMU_ARBITER_METHOD >= 0 // round robin...
        counter_reg = (counter_reg + 1) & ((CFG_NUT_CPU_CORES << CFG_MEMU_ARBITER_METHOD) - 1);
#else // LFSR...
        counter_reg = GetNextLfsrState (counter_reg, 16, GetPrimePoly (16, 0));
#endif

        linelock_reg = next_linelock_reg;
        tagr_reg = next_tagr_reg;
        req_tagw_reg = next_req_tagw_reg;
        tagw_reg = next_tagw_reg;
        for (int n = 0; n < CFG_MEMU_CACHE_BANKS; n++)
            for (int p = 0; p < CFG_MEMU_BANK_RAM_PORTS; p++)
                bank_reg[n][p] = next_bank_reg[n][p];
        busif_reg = next_busif_reg;
    }
}


// **************** MMemu ***********************

void MMemu::Trace (sc_trace_file *tf, int levels) {
    if (!tf || pn_trace_verbose) printf ("\nSignals of Module \"%s\":\n", name ());

    // Ports ...
    PN_TRACE (tf, clk);
    PN_TRACE (tf, reset);

    //   Bus interface (Wishbone)...
    PN_TRACE (tf, wb_cyc_o);
    PN_TRACE (tf, wb_stb_o);
    PN_TRACE (tf, wb_we_o);
    PN_TRACE (tf, wb_sel_o);
    PN_TRACE (tf, wb_ack_i);
    // PN_TRACE(tf, wb_err_i);
    // PN_TRACE(tf, wb_rty_i);
    PN_TRACE (tf, wb_adr_o);
    PN_TRACE (tf, wb_dat_i);
    PN_TRACE (tf, wb_dat_o);

    //   Read ports...
    PN_TRACE_BUS (tf, rp_rd, CFG_MEMU_RPORTS);
    PN_TRACE_BUS (tf, rp_direct, CFG_MEMU_RPORTS);
    PN_TRACE_BUS (tf, rp_ack, CFG_MEMU_RPORTS);
    PN_TRACE_BUS (tf, rp_bsel, CFG_MEMU_RPORTS);
    PN_TRACE_BUS (tf, rp_adr, CFG_MEMU_RPORTS);
    PN_TRACE_BUS (tf, rp_data, CFG_MEMU_RPORTS);

    //   Write ports...
    PN_TRACE_BUS (tf, wp_wr, CFG_MEMU_WPORTS);
    PN_TRACE_BUS (tf, wp_direct, CFG_MEMU_WPORTS);
    PN_TRACE_BUS (tf, wp_bsel, CFG_MEMU_WPORTS);
    PN_TRACE_BUS (tf, wp_ack, CFG_MEMU_WPORTS);
    PN_TRACE_BUS (tf, wp_lres_scond, CFG_MEMU_WPORTS);
    PN_TRACE_BUS (tf, wp_scond_ok, CFG_MEMU_WPORTS);
    PN_TRACE_BUS (tf, wp_cache_op, CFG_MEMU_WPORTS);
    PN_TRACE_BUS (tf, wp_adr, CFG_MEMU_WPORTS);
    PN_TRACE_BUS (tf, wp_data, CFG_MEMU_WPORTS);

    // Tag RAM...
    PN_TRACE (tf, tagram_ready);
    PN_TRACE_BUS (tf, tagram_rd, TR_PORTS);
    PN_TRACE_BUS (tf, tagram_wr, TR_PORTS);
    PN_TRACE_BUS (tf, tagram_adr, TR_PORTS);
    PN_TRACE_BUS (tf, tagram_tag_in, TR_PORTS);
    PN_TRACE_BUS (tf, tagram_tag_out, TR_PORTS);

    // Bank RAM...
    PN_TRACE_BUS_BUS (tf, bankram_rd, CFG_MEMU_CACHE_BANKS, CFG_MEMU_BANK_RAM_PORTS);
    PN_TRACE_BUS_BUS (tf, bankram_wr, CFG_MEMU_CACHE_BANKS, CFG_MEMU_BANK_RAM_PORTS);
    PN_TRACE_BUS_BUS (tf, bankram_wiadr, CFG_MEMU_CACHE_BANKS, CFG_MEMU_BANK_RAM_PORTS);
    PN_TRACE_BUS_BUS (tf, bankram_wdata, CFG_MEMU_CACHE_BANKS, CFG_MEMU_BANK_RAM_PORTS);
    PN_TRACE_BUS_BUS (tf, bankram_rdata, CFG_MEMU_CACHE_BANKS, CFG_MEMU_BANK_RAM_PORTS);

    // BUSIF...
    PN_TRACE (tf, busif_op);
    PN_TRACE (tf, busif_nolinelock);
    PN_TRACE (tf, busif_busy);
    PN_TRACE (tf, busif_tag_rd);
    PN_TRACE (tf, busif_tag_wr);
    PN_TRACE_BUS (tf, busif_bank_rd, CFG_MEMU_CACHE_BANKS);
    PN_TRACE_BUS (tf, busif_bank_wr, CFG_MEMU_CACHE_BANKS);
    PN_TRACE (tf, busif_adr_in);
    PN_TRACE (tf, busif_adr_out);
    PN_TRACE_BUS (tf, busif_data_in, CFG_MEMU_CACHE_BANKS);
    PN_TRACE_BUS (tf, busif_data_out, CFG_MEMU_CACHE_BANKS);
    PN_TRACE_BUS (tf, busif_data_out_valid, CFG_MEMU_CACHE_BANKS);
    PN_TRACE (tf, busif_tag_in);
    PN_TRACE (tf, busif_tag_out);
    PN_TRACE (tf, busif_bsel);

    // Read ports...
    PN_TRACE_BUS (tf, rp_busif_data_reg, CFG_MEMU_RPORTS);
    PN_TRACE_BUS (tf, rp_busif_data, CFG_MEMU_RPORTS);
    PN_TRACE_BUS (tf, rp_busif_op, CFG_MEMU_RPORTS);
    PN_TRACE_BUS (tf, rp_tag_rd, CFG_MEMU_RPORTS);
    PN_TRACE_BUS (tf, rp_bank_rd, CFG_MEMU_RPORTS);
    PN_TRACE_BUS (tf, rp_tag_in, CFG_MEMU_RPORTS);
    PN_TRACE_BUS (tf, rp_way_out, CFG_MEMU_RPORTS);
    PN_TRACE_BUS (tf, rp_bank_data_in, CFG_MEMU_RPORTS);
    PN_TRACE_BUS (tf, rp_bank_sel, CFG_MEMU_RPORTS);

    // Write ports...
    PN_TRACE_BUS (tf, wp_busif_op, CFG_MEMU_WPORTS);
    PN_TRACE_BUS (tf, wp_busif_nolinelock, CFG_MEMU_WPORTS);
    PN_TRACE_BUS (tf, wp_tag_rd, CFG_MEMU_WPORTS);
    PN_TRACE_BUS (tf, wp_tag_wr, CFG_MEMU_WPORTS);
    PN_TRACE_BUS (tf, wp_bank_rd, CFG_MEMU_WPORTS);
    PN_TRACE_BUS (tf, wp_bank_wr, CFG_MEMU_WPORTS);
    PN_TRACE_BUS (tf, wp_tag_in, CFG_MEMU_WPORTS);
    PN_TRACE_BUS (tf, wp_tag_out, CFG_MEMU_WPORTS);
    PN_TRACE_BUS (tf, wp_bank_data_in, CFG_MEMU_WPORTS);
    PN_TRACE_BUS (tf, wp_bank_data_out, CFG_MEMU_WPORTS);

    // Arbiter: request/grant signals (find comments in 'MArbiter')...
    PN_TRACE (tf, req_busif_linelock);
    PN_TRACE_BUS (tf, req_wp_linelock, CFG_MEMU_WPORTS);
    PN_TRACE (tf, gnt_busif_linelock);
    PN_TRACE_BUS (tf, gnt_wp_linelock, CFG_MEMU_WPORTS);

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

    PN_TRACE_BUS (tf, req_busif_bank, CFG_MEMU_CACHE_BANKS);
    PN_TRACE_BUS_BUS (tf, req_wp_bank, CFG_MEMU_WPORTS, CFG_MEMU_CACHE_BANKS);
    PN_TRACE_BUS_BUS (tf, req_rp_bank, CFG_MEMU_RPORTS, CFG_MEMU_CACHE_BANKS);
    PN_TRACE_BUS (tf, gnt_busif_bank, CFG_MEMU_CACHE_BANKS);
    PN_TRACE_BUS_BUS (tf, gnt_wp_bank, CFG_MEMU_WPORTS, CFG_MEMU_CACHE_BANKS);
    PN_TRACE_BUS_BUS (tf, gnt_rp_bank, CFG_MEMU_RPORTS, CFG_MEMU_CACHE_BANKS);

    PN_TRACE_BUS (tf, req_rp_busif, CFG_MEMU_RPORTS);
    PN_TRACE_BUS (tf, gnt_rp_busif, CFG_MEMU_RPORTS);
    PN_TRACE_BUS (tf, req_wp_busif, CFG_MEMU_WPORTS);
    PN_TRACE_BUS (tf, gnt_wp_busif, CFG_MEMU_WPORTS);

    // Arbiter: other signals ...
    PN_TRACE (tf, wiadr_busif);
    PN_TRACE_BUS (tf, wiadr_rp, CFG_MEMU_RPORTS);
    PN_TRACE_BUS (tf, adr_wp, CFG_MEMU_WPORTS);
    PN_TRACE_BUS (tf, way_wp, CFG_MEMU_WPORTS);
    PN_TRACE (tf, snoop_adr);
    PN_TRACE_BUS (tf, snoop_stb, CFG_MEMU_WPORTS);

    // Sub-Modules...
    if (levels > 1) {
        levels--;
        tagRam->Trace (tf, levels);
        busIf->Trace (tf, levels);
        for (int n = 0; n < CFG_MEMU_RPORTS; n++) readPorts[n]->Trace (tf, levels);
        for (int n = 0; n < CFG_MEMU_WPORTS; n++) writePorts[n]->Trace (tf, levels);
        arbiter->Trace (tf, levels);
        if (levels > 1){
            levels--;
            for (int n = 0; n < CFG_MEMU_CACHE_BANKS; ++n) bankRam[n]->Trace (tf, levels);
        }
    }
}


void MMemu::InitSubmodules () {
    MBankRam *br;
    MReadPort *rp;
    MWritePort *wp;
    char name[80];
    int n, k, b, p;

    // Tag RAM...
    tagRam = new MTagRam ("TagRAM");

    tagRam->clk (clk);
    tagRam->reset (reset);
    tagRam->ready (tagram_ready);
    tagRam->rd_way (busif_tag_rd_way);
    for (n = 0; n < TR_PORTS; n++) {
        tagRam->rd[n](tagram_rd[n]);
        tagRam->wr[n](tagram_wr[n]);
        tagRam->adr[n](tagram_adr[n]);
        tagRam->wadr[n] (tagram_wadr[n]);
        tagRam->tag_in[n](tagram_tag_in[n]);
        tagRam->tag_out[n](tagram_tag_out[n]);
    }

    // Bank RAM...
    for (n = 0; n < CFG_MEMU_CACHE_BANKS; n++) {
        sprintf (name, "BankRAM%i", n);
        br = new MBankRam (name);
        bankRam[n] = br;

        br->clk (clk);
        for (k = 0; k < CFG_MEMU_BANK_RAM_PORTS; k++) {
            br->rd[k](bankram_rd[n][k]);
            br->wr[k](bankram_wr[n][k]);
            br->wen[k](bankram_wen[n][k]);
            br->wiadr[k](bankram_wiadr[n][k]);
            br->wdata[k](bankram_wdata[n][k]);
            br->rdata[k](bankram_rdata[n][k]);
        }
    }

    // BUSIF...
    busIf = new MBusIf ("BusIf");

    busIf->clk (clk);
    busIf->reset (reset);

    busIf->wb_cyc_o (wb_cyc_o);
    busIf->wb_stb_o (wb_stb_o);
    busIf->wb_we_o (wb_we_o);
    busIf->wb_sel_o (wb_sel_o);
    busIf->wb_cti_o (wb_cti_o);
    busIf->wb_bte_o (wb_bte_o);
    busIf->wb_ack_i (wb_ack_i);
    busIf->wb_adr_o (wb_adr_o);
    busIf->wb_dat_i (wb_dat_i);
    busIf->wb_dat_o (wb_dat_o);

    busIf->busif_op (busif_op);
    busIf->busif_nolinelock (busif_nolinelock);
    busIf->busif_bsel (busif_bsel);
    busIf->busif_busy (busif_busy);

    busIf->tag_rd (busif_tag_rd);
    busIf->tag_rd_way (busif_tag_rd_way);
    busIf->tag_wr (busif_tag_wr);
    for (n = 0; n < CFG_MEMU_CACHE_BANKS; n++) {
        busIf->bank_rd[n](busif_bank_rd[n]);
        busIf->bank_wr[n](busif_bank_wr[n]);
    }
    busIf->adr_in (busif_adr_in);
    busIf->adr_out (busif_adr_out);
    for (n = 0; n < CFG_MEMU_CACHE_BANKS; n++) {
        busIf->data_in[n](busif_data_in[n]);
        busIf->data_out[n](busif_data_out[n]);
        busIf->data_out_valid[n](busif_data_out_valid[n]);
    }
    busIf->tag_in (busif_tag_in);
    busIf->tag_out (busif_tag_out);

    busIf->req_linelock (req_busif_linelock);
    busIf->gnt_linelock (gnt_busif_linelock);
    busIf->req_tagw (req_busif_tagw);
    busIf->gnt_tagw (gnt_busif_tagw);
    busIf->req_tagr (req_busif_tagr);
    busIf->gnt_tagr (gnt_busif_tagr);
    for (n = 0; n < CFG_MEMU_CACHE_BANKS; n++) {
        busIf->req_bank[n](req_busif_bank[n]);
        busIf->gnt_bank[n](gnt_busif_bank[n]);
    }

    // Read ports...
    for (n = 0; n < CFG_MEMU_RPORTS; n++) {
        sprintf (name, "ReadPort%i", n);
        rp = new MReadPort (name);
        readPorts[n] = rp;

        rp->clk (clk);
        rp->reset (reset);

        rp->port_rd (rp_rd[n]);
        rp->port_direct (rp_direct[n]);
        rp->port_ack (rp_ack[n]);
        rp->port_adr (rp_adr[n]);
        rp->port_data (rp_data[n]);

        rp->busif_adr (busif_adr_out);
        rp->busif_data (rp_busif_data[n]);
        for (b = 0; b < CFG_MEMU_CACHE_BANKS; b++) rp->busif_data_valid[b](busif_data_out_valid[b]);
        rp->busif_op (rp_busif_op[n]);
        rp->busif_busy (busif_busy);

        rp->tag_rd (rp_tag_rd[n]);
        rp->tag_in (rp_tag_in[n]);
        rp->way_out (rp_way_out[n]);
        rp->bank_rd (rp_bank_rd[n]);
        rp->bank_data_in (rp_bank_data_in[n]);
        rp->bank_sel (rp_bank_sel[n]);

        rp->req_tagr (req_rp_tagr[n]);
        rp->req_busif (req_rp_busif[n]);
        for (k = 0; k < CFG_MEMU_CACHE_BANKS; k++) rp->req_bank[k](req_rp_bank[n][k]);
        rp->gnt_tagr (gnt_rp_tagr[n]);
        rp->gnt_busif (gnt_rp_busif[n]);
        for (k = 0; k < CFG_MEMU_CACHE_BANKS; k++) rp->gnt_bank[k](gnt_rp_bank[n][k]);

        if (n < CFG_MEMU_WPORTS) {
            // Route LR/SC signals (only needed for WP acompanied RPs
            rp->port_scond_ok (wp_scond_ok[n]);
            rp->snoop_adr (snoop_adr);
            rp->snoop_stb (snoop_stb[n]);
            rp->port_lres_scond (wp_lres_scond[n]);
        } else {
            rp->port_scond_ok (vh_open);
            rp->snoop_adr (vh_const<TWord> (0));
            rp->snoop_stb (vh_const<bool> (0));
            rp->port_lres_scond (vh_const<bool> (0));
        }
    }

    // Write ports...
    for (n = 0; n < CFG_MEMU_WPORTS; n++) {
        sprintf (name, "WritePort%i", n);
        wp = new MWritePort (name);
        writePorts[n] = wp;

        wp->clk (clk);
        wp->reset (reset);

        wp->port_wr (wp_wr[n]);
        wp->port_direct (wp_direct[n]);
        wp->port_bsel (wp_bsel[n]);
        wp->port_ack (wp_ack[n]);
        wp->port_lres_scond (wp_lres_scond[n]);
        wp->port_cache_op (wp_cache_op[n]); 
        wp->port_adr (wp_adr[n]);
        wp->port_data (wp_data[n]);
        wp->port_scond_ok (wp_scond_ok[n]);

        wp->busif_adr (busif_adr_out);
        wp->busif_op (wp_busif_op[n]);
        wp->busif_nolinelock (wp_busif_nolinelock[n]);
        wp->busif_busy (busif_busy);

        wp->tag_rd (wp_tag_rd[n]);
        wp->tag_wr (wp_tag_wr[n]);
        wp->tag_in (wp_tag_in[n]);
        wp->tag_out (wp_tag_out[n]);
        wp->bank_rd (wp_bank_rd[n]);
        wp->bank_wr (wp_bank_wr[n]);
        wp->bank_data_in (wp_bank_data_in[n]);
        wp->bank_data_out (wp_bank_data_out[n]);
        wp->bank_bsel (wp_bank_bsel[n]);

        wp->req_linelock (req_wp_linelock[n]);
        wp->req_tagr (req_wp_tagr[n]);
        wp->req_tagw (req_wp_tagw[n]);
        wp->req_busif (req_wp_busif[n]);
        for (k = 0; k < CFG_MEMU_CACHE_BANKS; k++) wp->req_bank[k](req_wp_bank[n][k]);
        wp->gnt_linelock (gnt_wp_linelock[n]);
        wp->gnt_tagr (gnt_wp_tagr[n]);
        wp->gnt_tagw (gnt_wp_tagw[n]);
        wp->gnt_busif (gnt_wp_busif[n]);
        for (k = 0; k < CFG_MEMU_CACHE_BANKS; k++) wp->gnt_bank[k](gnt_wp_bank[n][k]);

    }

    // Arbiter...
    arbiter = new MArbiter ("Arbiter");

    arbiter->clk (clk);
    arbiter->reset (reset);

    arbiter->tagram_ready (tagram_ready);
    arbiter->wiadr_busif (wiadr_busif);
    arbiter->req_busif_linelock (req_busif_linelock);
    arbiter->gnt_busif_linelock (gnt_busif_linelock);
    arbiter->req_busif_tagw (req_busif_tagw);
    arbiter->req_busif_tagr (req_busif_tagr);
    arbiter->gnt_busif_tagw (gnt_busif_tagw);
    arbiter->gnt_busif_tagr (gnt_busif_tagr);

    arbiter->gnt_busif_tagw_r (gnt_busif_tagw_r);
    arbiter->gnt_busif_tagr_r (gnt_busif_tagr_r);

    arbiter->snoop_adr (snoop_adr);

    for (n = 0; n < CFG_MEMU_RPORTS; n++) {
        arbiter->wiadr_rp[n](wiadr_rp[n]);
        arbiter->req_rp_tagr[n](req_rp_tagr[n]);
        arbiter->gnt_rp_tagr[n](gnt_rp_tagr[n]);
        arbiter->req_rp_busif[n](req_rp_busif[n]);
        arbiter->gnt_rp_busif[n](gnt_rp_busif[n]);

        arbiter->gnt_rp_tagr_r[n](gnt_rp_tagr_r[n]);
    }

    for (n = 0; n < CFG_MEMU_WPORTS; n++) {
        arbiter->adr_wp[n](adr_wp[n]);
        arbiter->way_wp[n](way_wp[n]);
        arbiter->req_wp_linelock[n](req_wp_linelock[n]);
        arbiter->gnt_wp_linelock[n](gnt_wp_linelock[n]);
        arbiter->req_wp_tagw[n](req_wp_tagw[n]);
        arbiter->req_wp_tagr[n](req_wp_tagr[n]);
        arbiter->gnt_wp_tagw[n](gnt_wp_tagw[n]);
        arbiter->gnt_wp_tagr[n](gnt_wp_tagr[n]);
        arbiter->req_wp_busif[n](req_wp_busif[n]);
        arbiter->gnt_wp_busif[n](gnt_wp_busif[n]);
        arbiter->snoop_stb[n](snoop_stb[n]);

        arbiter->gnt_wp_tagw_r[n](gnt_wp_tagw_r[n]);
        arbiter->gnt_wp_tagr_r[n](gnt_wp_tagr_r[n]);
    }
    for (b = 0; b < CFG_MEMU_CACHE_BANKS; b++) {
        arbiter->req_busif_bank[b](req_busif_bank[b]);
        arbiter->gnt_busif_bank[b](gnt_busif_bank[b]);
        for (n = 0; n < CFG_MEMU_RPORTS; n++) {
            arbiter->req_rp_bank[n][b](req_rp_bank[n][b]);
            arbiter->gnt_rp_bank[n][b](gnt_rp_bank[n][b]);
        }
        for (n = 0; n < CFG_MEMU_WPORTS; n++) {
            arbiter->req_wp_bank[n][b](req_wp_bank[n][b]);
            arbiter->gnt_wp_bank[n][b](gnt_wp_bank[n][b]);
        }
        for (n = 0; n < CFG_MEMU_BANK_RAM_PORTS; n++) arbiter->wiadr_bank[b][n](bankram_wiadr[b][n]);
    }
}


void MMemu::FreeSubmodules () {
    int n;

    delete tagRam;
    for (n = 0; n < CFG_MEMU_CACHE_BANKS; n++) delete bankRam[n];
    delete busIf;
    for (n = 0; n < CFG_MEMU_RPORTS; n++) delete readPorts[n];
    for (n = 0; n < CFG_MEMU_WPORTS; n++) delete writePorts[n];
    delete arbiter;
}


void MMemu::InitInterconnectMethod () {
    int n, k;

    SC_METHOD (InterconnectMethod);
        // Tag RAM...
        for (n = 0; n < TR_PORTS; n++) sensitive << tagram_tag_out[n];

        // Bank RAM...
        for (n = 0; n < CFG_MEMU_CACHE_BANKS; n++)
            for (k = 0; k < CFG_MEMU_BANK_RAM_PORTS; k++) sensitive << bankram_rdata[n][k];

        // BUSIF...
        sensitive << busif_busy << busif_tag_rd << busif_tag_wr << busif_adr_out << busif_tag_out;
        for (n = 0; n < CFG_MEMU_CACHE_BANKS; n++)
            sensitive << busif_bank_rd[n] << busif_bank_wr[n] << busif_data_out[n] << busif_data_out_valid[n];

        // Read ports...
        for (n = 0; n < CFG_MEMU_RPORTS; n++)
            sensitive << rp_busif_op[n] << rp_tag_rd[n] << rp_bank_rd[n] << rp_adr[n] << rp_way_out[n]
                      << rp_bank_sel[n];

        // Write ports...
        for (n = 0; n < CFG_MEMU_WPORTS; n++)
            sensitive << wp_busif_op[n] << wp_busif_nolinelock[n] << wp_tag_rd[n] << wp_tag_wr[n]
                      << wp_bank_rd[n] << wp_bank_wr[n] << wp_tag_out[n] << wp_bank_data_out[n]
                      << wp_bank_bsel[n] << wp_adr[n] << wp_data[n];

        // Arbiter: request/grant signals ...
        sensitive << gnt_busif_linelock << gnt_busif_tagw << gnt_busif_tagr;
        for (n = 0; n < CFG_MEMU_RPORTS; n++) sensitive << gnt_rp_tagr[n] << gnt_rp_busif[n];
        for (n = 0; n < CFG_MEMU_WPORTS; n++) {
            sensitive << gnt_wp_linelock[n] << gnt_wp_tagw[n] << gnt_wp_tagr[n] << gnt_wp_busif[n];
            for (k = 0; k < CFG_MEMU_CACHE_BANKS; k++)
                sensitive << gnt_wp_bank[n][k] << gnt_rp_bank[n][k];
        }
        for (n = 0; n < CFG_MEMU_CACHE_BANKS; n++) {
            sensitive << gnt_busif_bank[n];
            for (k = 0; k < CFG_MEMU_BANK_RAM_PORTS; k++) sensitive << bankram_wiadr[n][k];
        }

        // Internal registers...
        for (n = 0; n < CFG_MEMU_BUSIF_WIDTH/32; n++)
            sensitive << rp_busif_data_reg[n];
}


void MMemu::TransitionMethod () {
    for (int n = 0; n < CFG_MEMU_BUSIF_WIDTH/32; n++)
        rp_busif_data_reg[n] = busif_data_out[GetBankOfAdr (busif_adr_out + n*4)];
}


void MMemu::InterconnectMethod () {
    TWord busif_adr_in_var; // TBD: eliminate
    bool x;
    int n, b, p, cpu;

    // To Tag RAM...
    for (n = 0; n < CFG_NUT_CPU_CORES; n++) {
        tagram_rd[n] = (n == CFG_NUT_CPU_CORES - 1 ? busif_tag_rd : false) | rp_tag_rd[n] |
                       rp_tag_rd[CFG_NUT_CPU_CORES + n] | wp_tag_rd[n];
        tagram_wr[n] = (n == CFG_NUT_CPU_CORES - 1 ? busif_tag_wr : false) | wp_tag_wr[n];
        tagram_adr[n] = n == CFG_NUT_CPU_CORES - 1 && gnt_busif_tagr ?
                        busif_adr_out :
                        gnt_wp_tagr[n] ?
                        wp_adr[n] :
                        gnt_rp_tagr[n] ?
                        rp_adr[n] :
                        gnt_rp_tagr[CFG_NUT_CPU_CORES + n] ? rp_adr[CFG_NUT_CPU_CORES + n] : 0xffffffff; // don't care
        tagram_wadr[n] = n == CFG_NUT_CPU_CORES - 1 && gnt_busif_tagw ? busif_adr_out : wp_adr[n];
        tagram_tag_in[n] = n == CFG_NUT_CPU_CORES - 1 ? (gnt_wp_tagw[n] ? wp_tag_out[n] : busif_tag_out) : wp_tag_out[n];
    }

    // To Bank RAM...
    for (b = 0; b < CFG_MEMU_CACHE_BANKS; b++) {
        for (p = 0; p < CFG_MEMU_BANK_RAM_PORTS; p++) {
            // defaults...
            bankram_rd[b][p] = 0;
            bankram_wr[b][p] = 0;
            bankram_wen[b][p] = 0xf; // don't care
            // bankram_wiadr[b][p] = 0xffffffff;     // don't care
            bankram_wdata[b][p] = 0xffffffff; // don't care
            // find CPU...
            for (cpu = p; cpu < CFG_NUT_CPU_CORES; cpu += CFG_MEMU_BANK_RAM_PORTS) {
                if (gnt_rp_bank[cpu][b] == 1) {
                    if (rp_bank_rd[cpu] == 1) bankram_rd[b][p] = 1;
                }
                if (gnt_rp_bank[CFG_NUT_CPU_CORES + cpu][b] == 1) {
                    if (rp_bank_rd[CFG_NUT_CPU_CORES + cpu] == 1) bankram_rd[b][p] = 1;
                }
                if (gnt_wp_bank[cpu][b] == 1) {
                    if (wp_bank_rd[cpu] == 1) bankram_rd[b][p] = 1;
                    if (wp_bank_wr[cpu] == 1) bankram_wr[b][p] = 1;
                    bankram_wen[b][p] = wp_bank_bsel[cpu];
                    bankram_wdata[b][p] = wp_bank_data_out[cpu];
                }
            }
        }
        // eventually link BUSIF to last port...
        if (gnt_busif_bank[b] == 1) {
            if (busif_bank_rd[b] == 1) bankram_rd[b][CFG_MEMU_BANK_RAM_PORTS - 1] = 1;
            if (busif_bank_wr[b] == 1) bankram_wr[b][CFG_MEMU_BANK_RAM_PORTS - 1] = 1;
            bankram_wdata[b][CFG_MEMU_BANK_RAM_PORTS - 1] = busif_data_out[b];
            bankram_wen[b][CFG_MEMU_BANK_RAM_PORTS - 1] = 0b1111;
        }
    }

    // To BUSIF...
    //   defaults ...
    busif_op = bioNothing;
    busif_nolinelock = 0;
    busif_adr_in_var = 0;
    busif_bsel = 0;
    //   from tag RAMs...
    busif_tag_in = tagram_tag_out[CFG_NUT_CPU_CORES - 1];
    //   from bank RAMs...
    for (b = 0; b < CFG_MEMU_CACHE_BANKS; b++) busif_data_in[b] = bankram_rdata[b][CFG_MEMU_BANK_RAM_PORTS - 1];
    //   from read ports...
    for (p = 0; p < CFG_MEMU_RPORTS; p++)
        if (gnt_rp_busif[p]) {
            busif_op = rp_busif_op[p];
            busif_nolinelock = 0;
            busif_adr_in_var = rp_adr[p];
            busif_bsel = rp_bsel[p];
        }
    //   from write ports...
    for (p = 0; p < CFG_MEMU_WPORTS; p++)
        if (gnt_wp_busif[p]) {
            busif_op = wp_busif_op[p];
            busif_nolinelock = wp_busif_nolinelock[p];
            busif_adr_in_var = wp_adr[p];
            for (b = 0; b < CFG_MEMU_CACHE_BANKS; b++)
                if (busif_bank_rd[b] == 0) busif_data_in[b] = wp_data[p];
            busif_bsel = wp_bsel[p];
        }
    busif_adr_in = busif_adr_in_var;


    // To read ports...
    for (p = 0; p < CFG_MEMU_RPORTS; p++) {
        cpu = p % CFG_NUT_CPU_CORES;
        rp_tag_in[p] = tagram_tag_out[cpu];
        rp_bank_data_in[p] = bankram_rdata[rp_bank_sel[p].read ()][cpu % CFG_MEMU_BANK_RAM_PORTS];
#if CFG_MEMU_BUSIF_WIDTH == 64
        // Either top or bottom 32bit
        rp_busif_data[p] = rp_busif_data_reg[(rp_adr[p].read() & 4) == 4]; // busif_data_out[GetBankOfAdr (busif_adr_out)];//rp_busif_data_reg; //
#else
        // All get the same busif_data_reg
        rp_busif_data[p] = rp_busif_data_reg[0];
#endif
    }

    // To write ports...
    for (p = 0; p < CFG_MEMU_WPORTS; p++) {
        cpu = p;
        wp_tag_in[p] = tagram_tag_out[cpu];
        wp_bank_data_in[p] = bankram_rdata[GetBankOfAdr (wp_adr[p])][cpu % CFG_MEMU_BANK_RAM_PORTS];
    }

    // To arbiter ...
    wiadr_busif = GetWayIndexOfAdr (busif_adr_out, busif_tag_out.read ().way);
    for (p = 0; p < CFG_MEMU_RPORTS; p++) wiadr_rp[p] = GetWayIndexOfAdr (rp_adr[p], rp_way_out[p]);
    for (p = 0; p < CFG_MEMU_WPORTS; p++) {
        adr_wp[p] = wp_adr[p];
        way_wp[p] = wp_tag_out[p].read ().way;
    }
}

