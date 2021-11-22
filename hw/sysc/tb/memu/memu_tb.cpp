/*************************************************************************

  This file is part of the ParaNut project.

  Copyright (C) 2010-2015 Gundolf Kiefer <gundolf.kiefer@hs-augsburg.de>
      Hochschule Augsburg, University of Applied Sciences

  Description:
    This is a test bench for the memory unit (MEMU) of the ParaNut.

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


#include "paranut-sim.h"
#include "memory.h"

#include <systemc.h>

#include <stdio.h>

#define WORDS_BIGENDIAN 0
#if WORDS_BIGENDIAN == 1
// Big Endian
#define BSEL
#else
// Little Endian
#define BSEL(bsel) (((TByte)(bsel << 3) & 0x8) |\
                   ((TByte)(bsel << 1) & 0x4) |\
                   ((TByte)(bsel >> 1) & 0x2) |\
                   ((TByte)(bsel >> 3) & 0x1))
#endif



// **************** Signals *********************


sc_signal<bool> clk, reset;

// WB / memory...
sc_signal<bool> wb_cyc, wb_stb, wb_we, wb_ack, wb_err, wb_rty;
sc_signal<sc_uint<3>> wb_cti;
sc_signal<sc_uint<2>> wb_bte;
sc_signal<sc_uint<CFG_MEMU_BUSIF_WIDTH/8> > wb_sel;
sc_signal<sc_uint<32>> wb_adr;
sc_signal<sc_uint<CFG_MEMU_BUSIF_WIDTH> > wb_dat_w, wb_dat_r;
sc_signal<bool> dbg_ack, dbg_err, dbg_rty;
sc_signal<TWord> dbg_dat;

// CPU - Read ports...
sc_signal<bool> rp_rd[CFG_MEMU_RPORTS], rp_direct[CFG_MEMU_RPORTS];
sc_signal<bool> rp_ack[CFG_MEMU_RPORTS];
sc_signal<sc_uint<4> > rp_bsel[CFG_MEMU_RPORTS];
sc_signal<TWord> rp_adr[CFG_MEMU_RPORTS];
sc_signal<TWord> rp_data[CFG_MEMU_RPORTS];

// CPU - Write ports...
sc_signal<bool> wp_wr[CFG_MEMU_WPORTS], wp_direct[CFG_MEMU_WPORTS];
sc_signal<sc_uint<4> > wp_bsel[CFG_MEMU_WPORTS];
sc_signal<bool> wp_ack[CFG_MEMU_WPORTS];
sc_signal<bool> wp_lres_scond[CFG_MEMU_WPORTS];
sc_signal<bool> wp_scond_ok[CFG_MEMU_WPORTS];
sc_signal<sc_uint<3> > wp_cache_op[CFG_MEMU_WPORTS]; // warum nicht typ <EBusIfOperation>?
sc_signal<TWord> wp_adr[CFG_MEMU_WPORTS];
sc_signal<TWord> wp_data[CFG_MEMU_WPORTS];


// **************** Helpers *********************

#define CLK_PERIOD 10.0
//#define CLK_PERIOD (sc_time (10.0, SC_NS).to_double ())


void RunCycle (int n = 1) {
    for (int k = 0; k < n; k++) {
        clk = 1;
        sc_start (CLK_PERIOD / 2, SC_NS);
        clk = 0;
        sc_start (CLK_PERIOD / 2, SC_NS);
    }
}


// ***** Init *****

void ClearReadPort (int p) {
    rp_rd[p] = rp_direct[p] = 0;
    rp_bsel[p] = 0;
    rp_adr[p] = 0;
}


void ClearWritePort (int p) {
    wp_wr[p] = wp_direct[p] = 0;
    wp_bsel[p] = 0;
    wp_lres_scond[p] = 0;
    wp_cache_op[p] = bioNothing;
    wp_adr[p] = wp_data[p] = 0;
}


// ***** Read *****

static void ReadInit (int p, TWord adr, sc_uint<4> bsel = 0xf) {
    rp_adr[p] = adr;
    rp_bsel[p] = bsel;
    rp_rd[p] = 1;
}


static bool ReadCheckAck (int p) {
    if (rp_ack[p] == 1) {
        rp_rd[p] = 0;
        rp_adr[p] = 0xffffffff; // change adress to verify that in the delayed cycle the adress is not used
        return true;
    } else
        return false;
}


static inline TWord ReadGetData (int p) { return rp_data[p]; }


static TWord ReadComplete (int p) {
    do {
        RunCycle ();
    } while (!ReadCheckAck (p));
    RunCycle ();
    return ReadGetData (p);
}


static TWord Read (int p, TWord adr, sc_uint<4> bsel = 0xf) {
    TWord data;

    ReadInit (p, adr, bsel);
    data = ReadComplete (p);
    // printf ("### Read (0x%08x) = 0x%08x\n", adr, data);
    return data;
}


// ***** Write *****

void WriteInit (int p, TWord adr, TWord data, sc_uint<4> bsel = 0xf) {
    // Set rp signals for LR/SC instructions
    rp_adr[p] = adr;
    rp_bsel[p] = bsel;

    wp_adr[p] = adr;
    wp_data[p] = data;
    wp_bsel[p] = bsel;
    wp_wr[p] = 1;
}


void WriteInitSpecial (int p, TWord adr, bool writeback, bool invalidate) {
    wp_adr[p] = adr;
    sc_uint<3> op = bioNothing;
    if(writeback) op |= bioWriteback;
    if(invalidate)  op |= bioInvalidate;
    wp_cache_op[p] = op;
    
}


bool WriteTryComplete (int p) {
    if (wp_ack[p].read ()) {
        wp_wr[p] = 0;
        wp_cache_op[p] = bioNothing;
        return true;
    } else
        return false;
}


void WriteComplete (int p) {
    do {
        RunCycle ();
    } while (!WriteTryComplete (p));
}


void Write (int p, TWord adr, TWord data, sc_uint<4> bsel = 0xf) {
    WriteInit (p, adr, data, bsel);
    WriteComplete (p);
    // printf ("### Write (0x%08x) = 0x%08x\n", adr, data);
}


void WriteSpecial (int p, TWord adr, bool writeback, bool invalidate) {
    WriteInitSpecial (p, adr, writeback, invalidate);
    WriteComplete (p);
}


// ****** run_test ******

void RunPartwordReadWrite (int port, TByte id, TWord base, TWord stride = 4, int count = 1) {
    TWord data;
    int n;

    // Write & Read full words...
    PN_INFO ("    write & read full words ...");
    for (n = 0; n < count; n++)
        Write (port, base + n * stride, 0x01010101 * (id + 4 * n) + 0x00010203);
    for (n = 0; n < count; n++) {
        data = Read (port, base + n * stride);
        PN_ASSERT (data == 0x01010101 * (id + 4 * n) + 0x00010203);
    }

    // Read half words...
    PN_INFO ("    write words & read as half words...");
    for (n = 0; n < count; n++)
        Write (port, base + n * stride, 0x01010101 * (id + 4 * n) + 0x00010203);
    for (n = 0; n < count; n++) {
        data = Read (port, base + n * stride, BSEL (0x3));
        PN_ASSERT ((data & 0xffff0000) == 0x01010000 * (id + 4 * n) + 0x00010000);
    }
    for (n = 0; n < count; n++) {
        data = Read (port, base + n * stride, BSEL (0xc));
        PN_ASSERT ((data & 0x0000ffff) == 0x00000101 * (id + 4 * n) + 0x00000203);
    }
    base += count * stride;
    id += 4 * count;

    // Read bytes...
    PN_INFO ("    write words & read as bytes...");
    for (n = 0; n < count; n++)
        Write (port, base + n * stride, 0x01010101 * (id + 4 * n) + 0x00010203);
    for (n = 0; n < count; n++) {
        data = Read (port, base + n * stride, BSEL (0x1));
        PN_ASSERT ((data & 0xff000000) == 0x01000000 * (id + 4 * n) + 0x00000000);
    }
    for (n = 0; n < count; n++) {
        data = Read (port, base + n * stride, BSEL (0x2));
        PN_ASSERT ((data & 0x00ff0000) == 0x00010000 * (id + 4 * n) + 0x00010000);
    }
    for (n = 0; n < count; n++) {
        data = Read (port, base + n * stride, BSEL (0x4));
        PN_ASSERT ((data & 0x0000ff00) == 0x00000100 * (id + 4 * n) + 0x00000200);
    }
    for (n = 0; n < count; n++) {
        data = Read (port, base + n * stride, BSEL (0x8));
        PN_ASSERT ((data & 0x000000ff) == 0x00000001 * (id + 4 * n) + 0x00000003);
    }
    base += count * stride;
    id += 4 * count;

    // Write half words...
    PN_INFO ("    write half words & read as words...");
    for (n = 0; n < count; n++)
        Write (port, base + n * stride, 0x01010000 * (id + 4 * n) + 0x00010000, BSEL (0x3));
    for (n = 0; n < count; n++)
        Write (port, base + n * stride, 0x00000101 * (id + 4 * n) + 0x00000203, BSEL (0xc));
    for (n = 0; n < count; n++) {
        data = Read (port, base + n * stride);
        PN_ASSERT (data == 0x01010101 * (id + 4 * n) + 0x00010203);
    }
    base += count * stride;
    id += 4 * count;

    // Write bytes...
    PN_INFO ("    write bytes & read as words...");
    for (n = 0; n < count; n++)
        Write (port, base + n * stride, 0x01000000 * (id + 4 * n) + 0x00000000, BSEL (0x1));
    for (n = 0; n < count; n++)
        Write (port, base + n * stride, 0x00010000 * (id + 4 * n) + 0x00010000, BSEL (0x2));
    for (n = 0; n < count; n++)
        Write (port, base + n * stride, 0x00000100 * (id + 4 * n) + 0x00000200, BSEL (0x4));
    for (n = 0; n < count; n++)
        Write (port, base + n * stride, 0x00000001 * (id + 4 * n) + 0x00000003, BSEL (0x8));
    for (n = 0; n < count; n++) {
        data = Read (port, base + n * stride);
        PN_ASSERT (data == 0x01010101 * (id + 4 * n) + 0x00010203);
    }
    base += count * stride;
    id += 4 * count;
}


void RunParallelReadWrite (TWord id, TWord base0, TWord stride0, TWord base1, TWord stride1) {
    int n, completed, now_completed;

    // Parallel write...
    PN_INFO ("    parallel write...");
    for (n = 0; n < CFG_NUT_CPU_CORES; n++) WriteInit (n, base0 + stride0 * n, id + n);
    completed = 0;
    while (completed != (1 << CFG_NUT_CPU_CORES) - 1) {
        for (n = 0; n < CFG_NUT_CPU_CORES; n++)
            if (WriteTryComplete (n)) completed |= 1 << n;
        RunCycle ();
    }

    // Parallel read (data ports)...
    PN_INFO ("    parallel read (data ports)...");
    for (n = 0; n < CFG_NUT_CPU_CORES; n++) ReadInit (n, base0 + stride0 * n);
    completed = 0;
    while (completed != (1 << CFG_NUT_CPU_CORES) - 1) {
        now_completed = 0;
        for (n = 0; n < CFG_NUT_CPU_CORES; n++) {
            if (ReadCheckAck (n)) {
                now_completed |= 1 << n;
            }
        }
        completed |= now_completed;
        RunCycle ();
        for (n = 0; n < CFG_NUT_CPU_CORES; n++)
            if (now_completed & (1 << n)) PN_ASSERT (ReadGetData (n) == id + n);
    }

    // Parallel write & read (insn ports)...
    PN_INFO ("    parallel write & read (insn ports)...");
    for (n = 0; n < CFG_NUT_CPU_CORES; n++) {
        WriteInit (n, base1 + stride1 * n, id + CFG_NUT_CPU_CORES + n);
        ReadInit (CFG_NUT_CPU_CORES + n, base0 + stride0 * n);
    }
    completed = 0;
    while (completed != (1 << (2 * CFG_NUT_CPU_CORES)) - 1) {
        now_completed = 0;
        for (n = 0; n < CFG_NUT_CPU_CORES; n++) {
            if (WriteTryComplete (n)) {
                PN_ASSERTM (completed & (1 << (CFG_NUT_CPU_CORES + n)),
                         "Direct write completes before direct read: wrong priority");
                completed |= 1 << n;
            }
            if (ReadCheckAck (CFG_NUT_CPU_CORES + n)) {
                now_completed |= 1 << (CFG_NUT_CPU_CORES + n);
            }
        }
        completed |= now_completed;
        RunCycle ();
        for (n = 0; n < CFG_NUT_CPU_CORES; n++)
            if (now_completed & (1 << (CFG_NUT_CPU_CORES + n)))
                PN_ASSERT (ReadGetData (CFG_NUT_CPU_CORES + n) == id + n);
    }

    // Parallel read (data ports) to verify the last writes...
    PN_INFO ("    parallel read (data ports) to verify the last writes...");
    for (n = 0; n < CFG_NUT_CPU_CORES; n++) {
        ReadInit (CFG_NUT_CPU_CORES + n, base0 + stride0 * n);
        ReadInit (n, base1 + stride1 * n);
    }
    completed = 0;
    while (completed != (1 << (2 * CFG_NUT_CPU_CORES)) - 1) {
        now_completed = 0;
        for (n = 0; n < CFG_NUT_CPU_CORES; n++) {
            if (ReadCheckAck (CFG_NUT_CPU_CORES + n)) now_completed |= 1 << (CFG_NUT_CPU_CORES + n);
            if (ReadCheckAck (n)) now_completed |= 1 << n;
        }
        completed |= now_completed;
        RunCycle ();
        for (n = 0; n < CFG_NUT_CPU_CORES; n++) {
            if (now_completed & (1 << (CFG_NUT_CPU_CORES + n)))
                PN_ASSERT (ReadGetData (CFG_NUT_CPU_CORES + n) == id + n);
            if (now_completed & (1 << n)) PN_ASSERT (ReadGetData (n) == id + CFG_NUT_CPU_CORES + n);
        }
    }
}


static inline int GetRandom (int max) { return random () % max; }


void RunRandomReadWrite (TWord adrrange, int iterations) { // iterations must be << adrRange/4!
    int to_wait[CFG_MEMU_RPORTS], to_read[CFG_MEMU_RPORTS], to_write[CFG_MEMU_WPORTS], read_idx[CFG_MEMU_RPORTS], write_idx[CFG_MEMU_WPORTS];
    bool rp_ack[CFG_MEMU_RPORTS];
    TWord *adr_list, *data_list;
    TWord adr, data;
    bool *adr_used, *readable_list;
    char buf[80];
    int write_ptr, n, idx, loops;
    bool done;

    while (iterations > 0) {
        sprintf (buf, "  %i iterations left", iterations);
        PN_INFO (buf);
        loops = adrrange / 4;
        if (loops > iterations) loops = iterations;
        if (loops > 1000) loops = 1000;
        iterations -= loops;

        // Create random data...
        adr_used = new bool[adrrange];
        for (n = 0; n < adrrange / sizeof (TWord); n++) adr_used[n] = false;
        adr_list = new TWord[loops];
        data_list = new TWord[loops];
        for (n = 0; n < loops; n++) {
            do {
                adr = GetRandom (adrrange);
            } while (adr_used[adr / sizeof (TWord)]);
            adr_used[adr / sizeof (TWord)] = true;
            adr_list[n] = adr & ~3;
            data_list[n] = (TWord)random ();
        }
        delete[] adr_used;
        readable_list = new bool[loops];
        for (n = 0; n < loops; n++) readable_list[n] = 0;

        // Init state...
        write_ptr = 0;
        for (n = 0; n < CFG_MEMU_RPORTS; n++) {
            to_wait[n] = to_read[n] = 0;
            rp_ack[n] = false;
        }
        for (n = 0; n < CFG_MEMU_WPORTS; n++) to_write[n] = 0;
        done = false;

        // Main loop...
        do {
            for (n = 0; n < CFG_MEMU_RPORTS; n++) {

                // Handle state actions...
                if (to_wait[n] > 0) { // State is "waiting" (for read or write)
                    to_wait[n]--;
                    if (to_wait[n] <= 0) {
                        if (n < CFG_MEMU_WPORTS && to_write[n] > 0) {
                            // Issue a new write operation...
                            if (write_ptr < loops) {
                                WriteInit (n, adr_list[write_ptr], data_list[write_ptr]);
                                write_idx[n] = write_ptr;
                                write_ptr++;
                            } else {
                                to_write[n] = 0;
                                to_wait[n] = 1; // set state to "waiting"
                            }
                        } else if (to_read[n] > 0) {
                            // Issue a new read operation...
                            if (write_ptr > 0) {
                                idx = GetRandom (write_ptr);
                                while (idx >= 0 && !readable_list[idx]) idx--;
                            } else
                                idx = -1;
                            if (idx < 0)
                                to_wait[n] = GetRandom (10) + 1; // no readable adresses yet
                            else {
                                read_idx[n] = idx;
                                ReadInit (n, adr_list[idx]);
                            }
                        }
                    }
                } else if (n < CFG_MEMU_WPORTS && to_write[n] > 0) { // State is "writing"...
                    if (WriteTryComplete (n)) {
                        readable_list[write_idx[n]] = true;
                        to_write[n]--;
                        to_wait[n] = GetRandom (4) + 1; // set state to "waiting"
                    }
                } else if (to_read[n] > 0) { // State is "reading"...
                    if (rp_ack[n]) {
                        PN_ASSERTF (ReadGetData (n) == data_list[read_idx[n]],
                                 ("port #%i read wrong data from adress %08x: read %08x instead of "
                                  "%08x",
                                  n, adr_list[read_idx[n]], data, data_list[read_idx[n]]));
                        to_read[n]--;
                        to_wait[n] = GetRandom (4) + 1; // set state to "waiting"
                    }
                    rp_ack[n] = ReadCheckAck (n);
                } else { // State is "all counters zero"
                    if (write_ptr < loops) {
                        to_wait[n] = GetRandom (4) + 1; // set state to "waiting"
                        if (n < CFG_MEMU_WPORTS) to_write[n] = GetRandom (4) + 1;
                        to_read[n] = (n >= CFG_MEMU_WPORTS) ? 1 : (GetRandom (4) + 1);
                    }
                }
            }
            RunCycle ();

            // Check if we're done...
            if (write_ptr < loops)
                done = false;
            else {
                done = true;
                for (n = 0; n < CFG_MEMU_WPORTS; n++)
                    if (to_write[n]) done = false;
                for (n = 0; n < CFG_MEMU_RPORTS; n++)
                    if (to_read[n]) done = false;
            }
        } while (!done);

        // Final read run to check that everything was written properly...
        for (n = 0; n < loops; n++) {
            data = Read (0, adr_list[n]);
            PN_ASSERTF (data == data_list[n],
                     ("port #0 read wrong data from adress %08x: read %08x instead of %08x",
                      adr_list[n], data, data_list[n]));
        }

        // Free ...
        delete[] adr_list;
        delete[] data_list;
        delete[] readable_list;
    }
}


void RunPerformanceTest (TWord adr, TWord id) {
    char buf[80];
    double t0, t1;
    TWord completed, now_completed, idx[CFG_MEMU_WPORTS], last_idx[CFG_MEMU_WPORTS];
    int n, k, itN, set;

    // single-port sequential write & read (no cache misses)...
    itN = CFG_MEMU_CACHE_BANKS * CFG_MEMU_CACHE_SETS;
    sprintf (buf, "  sequentially accessing %i words", itN);
    PN_INFO (buf);
    for (k = 0; k < 2; k++) { // write twice to have no misses in the second run
        t0 = sc_time_stamp ().to_double ();
        for (n = 0; n < itN; n++) Write (0, adr + 4 * n, id + n);
        t1 = sc_time_stamp ().to_double ();
        sprintf (buf, "    writing, run %i: %.2lf clocks/operation", k, (t1 - t0) / itN / CLK_PERIOD);
        PN_INFO (buf);
    }
    t0 = sc_time_stamp ().to_double ();
    ReadInit (0, adr);
    for (n = 0; n < itN; n++) {
        while (!ReadCheckAck (0)) RunCycle ();
        if (n < itN - 1)
            ReadInit (0, adr + 4 * (n + 1));
        else
            t1 = sc_time_stamp ().to_double ();
        RunCycle ();
        PN_ASSERT (ReadGetData (0) == id + n);
    }
    sprintf (buf, "    reading: %.2lf clocks/operation", (t1 - t0) / itN / CLK_PERIOD);
    PN_INFO (buf);
    id += 0x10000;

    // single-port sequential write & read (check replacement for multi-way caches)...
    if (CFG_MEMU_CACHE_WAYS > 1) {
        itN = CFG_MEMU_CACHE_BANKS * CFG_MEMU_CACHE_SETS * CFG_MEMU_CACHE_WAYS;
        sprintf (buf, "  sequentially accessing %i words", itN);
        PN_INFO (buf);
        for (k = 0; k < 2; k++) { // write twice to have no cold misses in the second run
            t0 = sc_time_stamp ().to_double ();
            for (n = 0; n < itN; n++) Write (0, adr + 4 * n, id + n);
            t1 = sc_time_stamp ().to_double ();
            sprintf (buf, "    writing, run %i: %.2lf clocks/operation", k, (t1 - t0) / itN / CLK_PERIOD);
            PN_INFO (buf);
        }
        t0 = sc_time_stamp ().to_double ();
        ReadInit (0, adr);
        for (n = 0; n < itN; n++) {
            while (!ReadCheckAck (0)) RunCycle ();
            if (n < itN - 1)
                ReadInit (0, adr + 4 * (n + 1));
            else
                t1 = sc_time_stamp ().to_double ();
            RunCycle ();
            PN_ASSERT (ReadGetData (0) == id + n);
        }
        sprintf (buf, "    reading: %.2lf clocks/operation", (t1 - t0) / itN / CLK_PERIOD);
        PN_INFO (buf);
    }
    id += 0x10000;

    // Parallel writing and reading...
    itN = CFG_MEMU_CACHE_BANKS * CFG_MEMU_CACHE_SETS / CFG_MEMU_WPORTS;
    sprintf (buf, "  parallel writes and reads: %i words, %i ports", itN * CFG_MEMU_WPORTS, CFG_MEMU_WPORTS);
    PN_INFO (buf);

    // ... write adjacent...
    t0 = sc_time_stamp ().to_double ();
    for (n = 0; n < itN; n++) {
        for (k = 0; k < CFG_MEMU_WPORTS; k++) WriteInit (k, adr + 4 * (CFG_MEMU_WPORTS * n + k), id + CFG_MEMU_WPORTS * n + k);
        completed = 0;
        while (completed != (1 << CFG_MEMU_WPORTS) - 1) {
            RunCycle ();
            for (k = 0; k < CFG_MEMU_WPORTS; k++)
                if (WriteTryComplete (k)) completed |= (1 << k);
            // if (completed != (1 << CFG_MEMU_WPORTS) - 1) run_cycle ();
        }
    }
    t1 = sc_time_stamp ().to_double ();
    sprintf (buf, "    writing (adjacent words): %.2lf clocks/operation", (t1 - t0) / itN / CLK_PERIOD);
    PN_INFO (buf);

    // ... read adjacent...
    t0 = sc_time_stamp ().to_double ();
    for (k = 0; k < CFG_MEMU_WPORTS; k++) ReadInit (k, adr + 4 * k);
    for (n = 0; n < itN; n++) {
        completed = 0;
        while (completed != (1 << CFG_MEMU_WPORTS) - 1) {
            now_completed = 0;
            for (k = 0; k < CFG_MEMU_WPORTS; k++)
                if (ReadCheckAck (k)) now_completed |= (1 << k);
            completed |= now_completed;
            if (completed == (1 << CFG_MEMU_WPORTS) - 1) {
                if (n < itN - 1) {
                    for (k = 0; k < CFG_MEMU_WPORTS; k++) ReadInit (k, adr + 4 * (CFG_MEMU_WPORTS * (n + 1) + k));
                } else
                    t1 = sc_time_stamp ().to_double ();
            }
            RunCycle ();
            for (k = 0; k < CFG_MEMU_WPORTS; k++)
                if (now_completed & (1 << k)) PN_ASSERT (ReadGetData (k) == id + CFG_MEMU_WPORTS * n + k);
        }
    }
    sprintf (buf, "    reading (adjacent words): %.2lf clocks/operation", (t1 - t0) / itN / CLK_PERIOD);
    PN_INFO (buf);

    id += 0x10000;

    // ... write, different sets & banks...
    t0 = sc_time_stamp ().to_double ();
    for (n = 0; n < itN; n++) {
        for (k = 0; k < CFG_MEMU_WPORTS; k++) {
            set = (n + k * CFG_MEMU_CACHE_BANKS / CFG_MEMU_WPORTS) % itN;
            WriteInit (k, adr + 4 * (CFG_MEMU_WPORTS * set + k), id + CFG_MEMU_WPORTS * set + k);
        }
        completed = 0;
        while (completed != (1 << CFG_MEMU_WPORTS) - 1) {
            RunCycle ();
            for (k = 0; k < CFG_MEMU_WPORTS; k++)
                if (WriteTryComplete (k)) completed |= (1 << k);
            // if (completed != (1 << CFG_MEMU_WPORTS) - 1) run_cycle ();
        }
    }
    t1 = sc_time_stamp ().to_double ();
    sprintf (buf, "    writing (different sets & banks): %.2lf clocks/operation", (t1 - t0) / itN / CLK_PERIOD);
    PN_INFO (buf);

    // ... read same
    t0 = sc_time_stamp ().to_double ();
    for (k = 0; k < CFG_MEMU_WPORTS; k++) ReadInit (k, adr + 4 * 0);
    for (n = 0; n < itN * CFG_MEMU_WPORTS; n++) {
        completed = 0;
        while (completed != (1 << CFG_MEMU_WPORTS) - 1) {
            now_completed = 0;
            for (k = 0; k < CFG_MEMU_WPORTS; k++)
                if (ReadCheckAck (k)) now_completed |= (1 << k);
            completed |= now_completed;
            if (completed == (1 << CFG_MEMU_WPORTS) - 1) {
                if (n < itN * CFG_MEMU_WPORTS - 1) {
                    for (k = 0; k < CFG_MEMU_WPORTS; k++) ReadInit (k, adr + 4 * (n + 1));
                } else
                    t1 = sc_time_stamp ().to_double ();
            }
            RunCycle ();
            for (k = 0; k < CFG_MEMU_WPORTS; k++)
                if (now_completed & (1 << k)) PN_ASSERT (ReadGetData (k) == id + n);
        }
    }
    sprintf (buf, "    reading (same words): %.2lf clocks/operation", (t1 - t0) / itN / CFG_MEMU_WPORTS / CLK_PERIOD);
    PN_INFO (buf);

    // ... read random
    t0 = sc_time_stamp ().to_double ();
    for (k = 0; k < CFG_MEMU_WPORTS; k++) {
        idx[k] = GetRandom (itN * CFG_MEMU_WPORTS);
        ReadInit (k, adr + 4 * idx[k]);
    }
    for (n = 0; n < itN * CFG_MEMU_WPORTS; n++) {
        completed = 0;
        while (completed != (1 << CFG_MEMU_WPORTS) - 1) {
            now_completed = 0;
            for (k = 0; k < CFG_MEMU_WPORTS; k++)
                if (ReadCheckAck (k)) now_completed |= (1 << k);
            completed |= now_completed;
            for (k = 0; k < CFG_MEMU_WPORTS; k++) last_idx[k] = idx[k];
            if (completed == (1 << CFG_MEMU_WPORTS) - 1) {
                if (n < itN * CFG_MEMU_WPORTS - 1) {
                    for (k = 0; k < CFG_MEMU_WPORTS; k++) {
                        idx[k] = GetRandom (itN * CFG_MEMU_WPORTS);
                        ReadInit (k, adr + 4 * idx[k]);
                    }
                } else
                    t1 = sc_time_stamp ().to_double ();
            }
            RunCycle ();
            for (k = 0; k < CFG_MEMU_WPORTS; k++)
                if (now_completed & (1 << k)) PN_ASSERT (ReadGetData (k) == id + last_idx[k]);
        }
    }
    sprintf (buf, "    reading (random words): %.2lf clocks/operation", (t1 - t0) / itN / CFG_MEMU_WPORTS / CLK_PERIOD);
    PN_INFO (buf);
}


void RunSpecial (int port, TWord adr, TWord id) {

    // Check "invalidate"...
    PN_INFO ("  checking 'invalidate'...");
    Write (port, adr, id); // write 'id' to cache
    wp_direct[port] = 1;
    Write (port, adr, id + 1); // write 'id+1' to main memory
    wp_direct[port] = 0;
    PN_ASSERTM (Read (port, adr) == id, "Direct write affects cache - cannot check 'invalidate'");
    WriteSpecial (port, adr, 0, 1); // invoke 'invalidate'
    PN_ASSERTM (Read (port, adr) == id + 1, "Special operation 'invalidate' does not work");

    // Check "writeback"...
    PN_INFO ("  checking 'writeback'...");
    Write (port, adr, id + 2); // write 'id+2' to cache (main memory holds 'id+1')
    rp_direct[port] = 1;
    PN_ASSERTM (Read (port, adr) == id + 1,
             "Cached write affects main memory - cannot check 'writeback'");
    rp_direct[port] = 0;
    WriteSpecial (port, adr, 1, 0); // invoke 'writeback'
    rp_direct[port] = 1;
    PN_ASSERTM (Read (port, adr) == id + 2, "Special operation 'writeback' does not work");
    rp_direct[port] = 0;

    // Check "flush"...
    PN_INFO ("  checking 'flush'...");
    Write (port, adr, id + 3); // write 'id+3' to cache (main memory holds 'id+2')
    WriteSpecial (port, adr, 1, 1); // invoke 'flush'
    rp_direct[port] = 1;
    PN_ASSERTM (Read (port, adr) == id + 3, "Special operation 'flush' does not write back");
    rp_direct[port] = 0;
    wp_direct[port] = 1;
    Write (port, adr, id + 4); // write 'id+4' to main memory, the next read must cause a miss and this value to be loaded
    wp_direct[port] = 0;
    PN_ASSERTM (Read (port, adr) == id + 4, "Special operation 'flush' does not invalidate");
}


void RunLlSc (TWord adr, TWord id) {
    // TBD: make more exhaustive tests

    // Initialize adress...
    Write (0, adr, id);

    // Simulate LL...
    wp_lres_scond[0] = 1;
    wp_adr[0] = adr;
    PN_ASSERT (Read (0, adr) == id);
    wp_lres_scond[0] = 0;

    // Perform WC on different adress (should fail) ...
    wp_lres_scond[0] = 1;
    Write (0, adr + 4, id + 1);
    wp_lres_scond[0] = 0;
    PN_ASSERTM (wp_scond_ok[0] == 0, "'wcond_ok' is set after write-conditional on non-linked adress");

    // Perform successful WC...
    wp_lres_scond[0] = 1;
    Write (0, adr, id + 1);
    wp_lres_scond[0] = 0;
    PN_ASSERTM (wp_scond_ok[0] == 1, "'wcond_ok' not set after successful write-conditional");
    PN_ASSERTM (Read (0, adr) == id + 1, "memory not changed after sucessful write-conditional");

    if (CFG_MEMU_WPORTS < 2)
        PN_WARNINGF (("  need 2 ports to test failing WC, have only %i: skipping test", CFG_MEMU_WPORTS));
    else {

        // Perform concurrent write through other port...
        Write (1, adr, id + 2);

        // Perform failing WC...
        wp_lres_scond[0] = 1;
        Write (0, adr, id + 3);
        wp_lres_scond[0] = 0;
        PN_ASSERTM (wp_scond_ok[0] == 0, "'wcond_ok' is set after failing write-conditional");
        PN_ASSERTM (Read (0, adr) == id + 2, "memory has changed after failing write-conditional");
    }
}


void RunTest () {
    int n;

    // Clear ports...
    for (n = 0; n < CFG_NUT_CPU_CORES; n++) {
        ClearReadPort (n);
        ClearWritePort (n);
    }

    PN_INFO ("Simulation starting...");
    reset = 1;
    RunCycle (5);
    reset = 0;

    /*
     */
    // Direct write & read...
    for (n = 0; n < CFG_MEMU_WPORTS; n++) wp_direct[n] = 1;
    for (n = 0; n < CFG_MEMU_RPORTS; n++) rp_direct[n] = 1;
    PN_INFO ("Direct write & read (with single bytes)...");
    RunPartwordReadWrite (0, 0x10, 0x100);
    PN_INFO ("Direct write & read (parallel)...");
    RunParallelReadWrite (0x20, 0x200, 4, 0x300, 16);
    PN_INFO ("Random write & read...");
    RunRandomReadWrite (16 * CFG_MEMU_CACHE_SIZE, 100);
    for (n = 0; n < CFG_MEMU_WPORTS; n++) wp_direct[n] = 0;
    for (n = 0; n < CFG_MEMU_RPORTS; n++) rp_direct[n] = 0;

    // Cached write & read...
    PN_INFO ("Cached write & read (port #0, with partial words)...");
    PN_INFO ("  same cache line...");
    RunPartwordReadWrite (0, 0x30, 0x300);
    PN_INFO ("  multiple cache lines (no conflicts)...");
    RunPartwordReadWrite (0, 0x40, 0x400, 4, 10);
    PN_INFO ("  with conflicts (to force replacements)...");
    RunPartwordReadWrite (0, 0x50, 0x508, CFG_MEMU_CACHE_SIZE, 3); // provoke replacements

    PN_INFO ("Cached write & read (parallel)...");
    RunParallelReadWrite (0x60, 0x600, 4, 0x680, 16);
    RunParallelReadWrite (0x70, 0x700, 16, 0x780, 4);

    PN_INFO ("Cached random write & read (no conflict & capacity misses)...");
    RunRandomReadWrite (CFG_MEMU_CACHE_SIZE, 100);

    PN_INFO ("Cached random write & read...");
    RunRandomReadWrite (4 * CFG_MEMU_CACHE_SIZE, 1000);

    // Special operations...
    PN_INFO ("Special operations...");
    RunSpecial (0, 0xab00, 0x13);

    // LL/SC...
    PN_INFO ("Load-Link and Store-Conditional...");
    RunLlSc (0xac00, 0x20);

    // Performance test...
    PN_INFO ("Performance measurements...");
    RunPerformanceTest (0x10000, 0x1000);

    // Simulation finished...
    PN_INFO ("Simulation finished.");
    // PN_INFO ("Simulation finished. Final memory content follows.");
    // mem->Dump ();

    RunCycle (1); // Wait some time...
}


// **************** Main ************************

int cfg_help = 0;

int sc_main (int argc, char *argv[]) {
    sc_trace_file *tf = NULL;
    int n, arg;

    // Parse command line...
    arg = 1;
    while (arg < argc && argv[arg][0] == '-') {
        switch (argv[arg][1]) {
        case 't':
            pn_cfg_vcd_level = MAX (0, MIN (9, argv[arg][2] - '0'));
            fprintf (stderr, "(cfg) vcdLevel = %i\n", pn_cfg_vcd_level);
            break;
        case 'h':
            cfg_help = 1;
            break;
        default:
            printf ("PN_ERROR: Unknown option '%s'.\n", argv[arg]);
            cfg_help = 1;
            arg = argc;
        }
        arg++;
    }
    if (cfg_help) {
        puts ("Usage: memu_tb [<options>] <OR32 ELF file>\n"
              "\n"
              "Options:\n"
              "  -t<n>: set VCD trace level (0 = no trace file; default = 2)\n");
        return 3;
    }

    sc_set_time_resolution (1.0, SC_NS);

    MWBMemory memory ("memory", 0x00000000, CFG_NUT_MEM_SIZE);
    memory.wb_clk_i (clk);
    memory.wb_rst_i (reset);
    memory.wb_stb_i (wb_stb);
    memory.wb_cyc_i (wb_cyc);
    memory.wb_we_i (wb_we);
    memory.wb_bte_i (wb_bte);
    memory.wb_cti_i (wb_cti);
    memory.wb_ack_o (wb_ack);
    memory.wb_err_o (wb_err);
    memory.wb_rty_o (wb_rty);
    memory.wb_sel_i (wb_sel);
    memory.wb_adr_i (wb_adr);
    memory.wb_dat_i (wb_dat_w);
    memory.wb_dat_o (wb_dat_r);

    MMemu memu ("MemU");
    memu.clk (clk);
    memu.reset (reset);
    memu.wb_stb_o (wb_stb);
    memu.wb_cyc_o (wb_cyc);
    memu.wb_we_o (wb_we);
    memu.wb_bte_o (wb_bte);
    memu.wb_cti_o (wb_cti);
    memu.wb_ack_i (wb_ack);
    memu.wb_sel_o (wb_sel);
    memu.wb_adr_o (wb_adr);
    memu.wb_dat_o (wb_dat_w);
    memu.wb_dat_i (wb_dat_r);
    for (n = 0; n < CFG_MEMU_RPORTS; n++) {
        memu.rp_rd[n](rp_rd[n]);
        memu.rp_direct[n](rp_direct[n]);
        memu.rp_ack[n](rp_ack[n]);
        memu.rp_bsel[n](rp_bsel[n]);
        memu.rp_adr[n](rp_adr[n]);
        memu.rp_data[n](rp_data[n]);
    }
    for (n = 0; n < CFG_MEMU_WPORTS; n++) {
        memu.wp_wr[n](wp_wr[n]);
        memu.wp_direct[n](wp_direct[n]);
        memu.wp_bsel[n](wp_bsel[n]);
        memu.wp_ack[n](wp_ack[n]);
        memu.wp_lres_scond[n](wp_lres_scond[n]);
        memu.wp_scond_ok[n](wp_scond_ok[n]);

        memu.wp_cache_op[n](wp_cache_op[n]);
        // memu.wp_writeback[n](wp_writeback[n]);
        // memu.wp_invalidate[n](wp_invalidate[n]);
        

        memu.wp_adr[n](wp_adr[n]);
        memu.wp_data[n](wp_data[n]);
    }

    // Init trace file...
    if (pn_cfg_vcd_level > 0) {
        tf = sc_create_vcd_trace_file ("memu_tb");

        tf->delta_cycles (false);

        PN_TRACE (tf, clk);
        PN_TRACE (tf, reset);

        PN_TRACE (tf, wb_stb);
        PN_TRACE (tf, wb_cyc);
        PN_TRACE (tf, wb_we);
        PN_TRACE (tf, wb_ack);
        PN_TRACE (tf, wb_err);
        PN_TRACE (tf, wb_rty);
        PN_TRACE (tf, wb_sel);
        PN_TRACE (tf, wb_adr);
        PN_TRACE (tf, wb_dat_w);
        PN_TRACE (tf, wb_dat_r);

        PN_TRACE_BUS (tf, rp_rd, CFG_MEMU_RPORTS);
        PN_TRACE_BUS (tf, rp_direct, CFG_MEMU_RPORTS);
        PN_TRACE_BUS (tf, rp_ack, CFG_MEMU_RPORTS);
        PN_TRACE_BUS (tf, rp_bsel, CFG_MEMU_RPORTS);
        PN_TRACE_BUS (tf, rp_adr, CFG_MEMU_RPORTS);
        PN_TRACE_BUS (tf, rp_data, CFG_MEMU_RPORTS);

        PN_TRACE_BUS (tf, wp_wr, CFG_MEMU_WPORTS);
        PN_TRACE_BUS (tf, wp_direct, CFG_MEMU_WPORTS);
        PN_TRACE_BUS (tf, wp_bsel, CFG_MEMU_WPORTS);
        PN_TRACE_BUS (tf, wp_ack, CFG_MEMU_WPORTS);
        PN_TRACE_BUS (tf, wp_lres_scond, CFG_MEMU_WPORTS);
        PN_TRACE_BUS (tf, wp_scond_ok, CFG_MEMU_WPORTS);
        PN_TRACE_BUS (tf, wp_cache_op, CFG_MEMU_WPORTS);
        PN_TRACE_BUS (tf, wp_adr, CFG_MEMU_WPORTS);
        PN_TRACE_BUS (tf, wp_data, CFG_MEMU_WPORTS);

        memu.Trace (tf, pn_cfg_vcd_level);

    }

    // Run simulation...
    sc_start (SC_ZERO_TIME);
    RunTest ();

    // Finish...
    if (tf) sc_close_vcd_trace_file (tf);
    return 0;
}
