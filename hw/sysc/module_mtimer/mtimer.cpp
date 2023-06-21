/*************************************************************************

  This file is part of the ParaNut project.

  Copyright (C) 2010-2022 Mark Endres <mark.endres@hs-augsburg.de>
                          Christian H. Meyer <christian.meyer@hs-augsburg.de>
      Hochschule Augsburg, University of Applied Sciences

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.

 *************************************************************************/

#include "mtimer.h"

#ifndef __SYNTHESIS__
void Mtimer::Trace (sc_trace_file *tf, int level) {
    if (!tf || pn_trace_verbose) printf ("\nSignals of Module \"%s\":\n", name ());

    // Ports ...
    PN_TRACE (tf, wb_clk_i);
    PN_TRACE (tf, wb_rst_i);
    
    PN_TRACE (tf, wb_ack_o);
    PN_TRACE (tf, wb_we_i);
    PN_TRACE (tf, wb_stb_i);
    
    PN_TRACE (tf, irq_out_enable);
    PN_TRACE (tf, irq_out);
    PN_TRACE (tf, reg_mtime);
    PN_TRACE (tf, reg_mtimecmp);
    PN_TRACE (tf, reg_mtime_prescale_cnt);
}
#endif

void Mtimer::TransitionMethod () {
    sc_uint<64> mtime_prescale_cnt = reg_mtime_prescale_cnt.read ();
    sc_uint<64> mtime_val = reg_mtime.read ();
    sc_uint<64> mtimecmp_val = reg_mtimecmp.read (); 
    bool irq_out_enabled = irq_out_enable.read();

    if (wb_rst_i) {
        reg_mtime = 0;
        reg_mtimecmp = 0; 
        reg_mtime_prescale_cnt = 0;
        irq_out = 0;
        
        wb_ack_o = 0;
        wb_dat_o = 0;
    } 
    else {
        reg_mtime_prescale_cnt = ++mtime_prescale_cnt;
        
        if (mtime_prescale_cnt == MTIMER_PRESCALER) {    
            reg_mtime = ++mtime_val;
            reg_mtime_prescale_cnt = 0;
        }

        irq_out = irq_out_enabled && (mtime_val >= mtimecmp_val);
    
        // Read inputs...
        sc_uint<CFG_MEMU_BUSIF_WIDTH> adr = wb_adr_i.read ();
        sc_uint<CFG_MEMU_BUSIF_WIDTH> dat = wb_dat_i.read ();
        bool selected = (adr & CFG_NUT_MTIMER_ADDR) && (adr < CFG_NUT_MTIMER_ADDR + 16); // Mask ourselves and exclude higher memory areas
        bool we = wb_we_i.read ();         // write enable 
        wb_ack_o = 0;

        // Internal modules are always strobed for every access
        if (wb_stb_i && selected) {
            sc_uint<CFG_MEMU_BUSIF_WIDTH> reg_adr = adr & 0xF;
            
            if (we) {
                // write
                switch (reg_adr) {
                case mtime: {
                    mtime_val.range(31, 0) = dat.range (31, 0);
                    reg_mtime = mtime_val;
                    break;
                }
                case mtimeh: {
                    mtime_val.range(63, 32) = dat.range (31, 0);
                    reg_mtime = mtime_val;
                    break;
                }
                case mtimecmp: {
                    mtimecmp_val.range(31, 0) = dat.range (31, 0);
                    reg_mtimecmp = mtimecmp_val;
                    break;
                }
                case mtimecmph: {
                    mtimecmp_val.range(63, 32) = dat.range (31, 0);
                    reg_mtimecmp = mtimecmp_val;
                    break;
                }
                default:
                    PN_ERRORF(("invalid mtimer write at address: 0x%08x", reg_adr.value()));
                    break;
                }
            }
            else {
                // read
                switch (reg_adr) {
                case mtime:
                    wb_dat_o = reg_mtime.read () (31, 0);
                    break;
                case mtimeh:
                    wb_dat_o = reg_mtime.read () (63, 32);
                    break;
                case mtimecmp:
                    wb_dat_o = reg_mtimecmp.read () (31, 0);
                    break;
                case mtimecmph:
                    wb_dat_o = reg_mtimecmp.read () (63, 32);
                    break;
                default:
                    PN_ERRORF(("invalid mtimer read at address: 0x%08x", reg_adr.value()));
                    break;
                }
            }

            wb_ack_o = 1;        
        }
    }
} 
