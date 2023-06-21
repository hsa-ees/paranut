
#include "memu_cache.h"
#include "lfsr.h"
// **************** MTagRam ********************
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

static sc_uint<6> GetNewUse (uint32_t old_use, uint way) {
    sc_uint<6> new_use;

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
        for (uint n = 0; n < 6; n += 2)
            if (((new_use >> n) & 3U) == way)
                new_use = (new_use & ((1U << n) - 1)) | ((new_use & ~((4U << n) - 1)) >> 2);

        // Remove the uppermost bits...
        new_use &= 0x3f;
    } else
        PN_ASSERTM (false,
                 "Only 1-, 2-, or 4-way associative caches are supported with LRU replacement");

    return new_use;
}


void MTagRam::proc_clk_tagram () {
    SCacheTag tag;
    STagEntry entry;
    sc_uint<6> new_use;
    int way = -1;
    bool write_tag_var;

    // Reset...
    ready = 0;
    write_tag = 0;
    wtag_iadr = 0;
    wtag_way = 0;
    wtag_port = 0;
    if (CFG_MEMU_CACHE_REPLACE_LRU == 0)
        counter = 0xffU;
    else
        for (uint n = 0; n < TR_PORTS; n++) use_wr_reg[n] = 0;


    for (uint n = 0; n < CFG_MEMU_CACHE_SETS; n++) {
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
    // Main loop...
    while (true) {
        ready = 1;
        write_tag_var = 0;

        // Step random counter...
        if (CFG_MEMU_CACHE_REPLACE_LRU == 0)
            counter = GetNextLfsrState (counter.read ()); // (counter + 1) & 0xff;

        // Read access...
        for (uint p = 0; p < TR_PORTS; p++) {
            if (rd[p] == 1) {
                entry = ram_[GetIndexOfAdr (adr[p])];
                way = -1; // no hit
                for (uint n = 0; n < CFG_MEMU_CACHE_WAYS; n++)
                    if (entry.tag_valid[n] && entry.tag_tadr[n] == GetTagOfAdr (adr[p])) {
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
                        way = counter.read () & (CFG_MEMU_CACHE_WAYS - 1); // random replacement
                    else {
                        way = entry.use & (CFG_MEMU_CACHE_WAYS - 1);
                        // PN_INFOF (("    port #%i cache miss for adress %x (iadr=%x): way = %i", p, adr[p].read (), GetIndexOfAdr(adr[p].read ()), way));
                    }
                }
                tag.dirty = entry.tag_dirty[way];
                tag.tadr = entry.tag_tadr[way];
                tag.ac_r = entry.tag_ac_r[way];
                tag.ac_w = entry.tag_ac_w[way];
                tag.ac_x = entry.tag_ac_x[way];
                tag.ac_u = entry.tag_ac_u[way];
                tag.way = (sc_uint<CFG_MEMU_CACHE_WAYS_LD>)way;

                tag_out[p] = tag;
            }
            if (p == TR_PORTS-1 && rd_way == 1) {
                // Special read for BusIf (connected to last port)
                // TODO: is this entry correct?
                entry = ram_[GetIndexOfAdr (adr[p])];
                // way = tag_in[TR_PORTS-1].read().way;
                tag.valid = entry.tag_valid[way];
                tag.dirty = entry.tag_dirty[way];
                tag.tadr = entry.tag_tadr[way];
                tag.ac_r = entry.tag_ac_r[way];
                tag.ac_w = entry.tag_ac_w[way];
                tag.ac_x = entry.tag_ac_x[way];
                tag.ac_u = entry.tag_ac_u[way];
                tag.way = (sc_uint<CFG_MEMU_CACHE_WAYS_LD>)way;
            }
            tag_out[p] = tag;
        }

        // Writing ...
        int p = -1;
        for (uint n = 0; n < TR_PORTS; n++){
            if (wr[n] == 1) {
                PN_ASSERTF (p == -1, ("multiple tag write signals asserted"));
                p = n;
                write_tag_var = 1;
            }
        }
        if (p >= 0) {
            // Handle a write request...
            //  - Implemented in hardware with asymetric block ram (read port width > write port width)
            //  - For simulation purpose we just read a whole tag entry and replace the tag in one cycle
            entry = ram_[GetIndexOfAdr (wadr[p])];
            tag = tag_in[p].read ();
            way = tag.way;
            entry.tag_valid[way] = tag.valid;
            entry.tag_dirty[way] = tag.dirty;
            entry.tag_tadr[way] = tag.tadr;
            entry.tag_ac_r[way] = tag.ac_r;
            entry.tag_ac_w[way] = tag.ac_w;
            entry.tag_ac_x[way] = tag.ac_x;
            entry.tag_ac_u[way] = tag.ac_u;

            ram_[GetIndexOfAdr (wadr[p])] = entry;

//            INFOF (("Write tag [%d]: wadr=0x%08x, iadr=0x%08x, way=%d, tadr=0x%08x, valid=%d, diry=%d", p,
//                    wadr[p].read (), GetIndexOfAdr (wadr[p]), way, tag.tadr, tag.valid, tag.dirty));

            wtag_iadr = GetIndexOfAdr (wadr[p]);
            wtag_way = way;
            wtag_port = p;
        } else if (CFG_MEMU_CACHE_REPLACE_LRU == 1) {
            // Check if we need to update lru information after a write last cycle
            if (write_tag == 1) {
                entry = ram_[use_iadr_reg[wtag_port.read ()].read ()]; // 'entry' must have been saved in a register during the past read cycle!
                new_use = GetNewUse (entry.use, wtag_way.read ());
                if (new_use != entry.use) {
                    use_iadr_reg[TR_PORTS] = GetIndexOfAdr (wtag_iadr);
                    use_reg[TR_PORTS] = new_use;
                    use_wr_reg[TR_PORTS] = 1;
                } else
                    use_wr_reg[TR_PORTS] = 0;
            }

            // No external write request: Write back some use info if pending...
            for (uint n = 0; n < (TR_PORTS+1); n++){
                if (p >= 0) break;
                if (use_wr_reg[n] == 1) p = n;
            }
                
            if (p >= 0) {
                entry = ram_[use_iadr_reg[p].read ()]; // 'entry' must have been saved in a register during the past read cycle!
                // PN_INFOF (("    writing back tag use: iadr = %x, port #%i", use_iadr_reg[p].read (), p));
                entry.use = use_reg[p];

                ram_[use_iadr_reg[p].read ()] = entry;
                use_wr_reg[p] = 0;
            }
        }

        write_tag = write_tag_var;

        wait (); // End of clock cycle
    }
}

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

void MBankRam::proc_clk_bankram () {
    
    while (1) {
        uint32_t combData;
        // Simulated BankRam has a read first policy to match the
        // standard Hardware configuration
        for (int n = 0; n < CFG_MEMU_BANK_RAM_PORTS; n++){
            if (rd[n] == 1){
                rdata[n] = ram_[wiadr[n].read ()];
                // printf("Reading %.8x from %.8x \n", (uint32_t)ram_[wiadr[n].read()], (uint32_t)wiadr[n].read());
            } 
            
        }
            

        for (int n = 0; n < CFG_MEMU_BANK_RAM_PORTS; n++){
            if (wr[n] == 1) {
                combData = ram_[wiadr[n].read ()];
                for (int i = 0; i < 4; i++) {
                    if (wen[n].read()[i] == 1) {
                        combData &= ~(0xffU << (8 * i));
                        combData |= ( wdata[n].read () & (0xffU << (8 * i)));
                    }
                }
                ram_[wiadr[n].read ()] = combData;
                // printf("Writing %.8x to %.8x \n", combData, (uint32_t)wiadr[n].read());
            }
        }
        wait ();
    }
}
