/*************************************************************************

  This file is part of the ParaNut project.

  Copyright (C) 2010-2022 Christian H. Meyer <christian.meyer@hs-augsburg.de>
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

#include "ptw.h"

void MPtw::Trace (sc_trace_file *tf, int level) {  
    if (!tf || pn_trace_verbose) printf ("\nSignals of Module \"%s\":\n", name ());
    
    PN_TRACE (tf, clk);
    PN_TRACE (tf, reset);

    PN_TRACE (tf, pte);
    PN_TRACE (tf, state_trace);
    PN_TRACE (tf, virtadr);

    // BusIf/MemU ports
    PN_TRACE (tf, memu_root_ppn);
    PN_TRACE (tf, busif_req);
    PN_TRACE (tf, busif_ack);
    PN_TRACE (tf, busif_virt_adr);
    PN_TRACE (tf, busif_phys_adr);
    PN_TRACE (tf, busif_ac_r);
    PN_TRACE (tf, busif_ac_w);
    PN_TRACE (tf, busif_ac_x);
    PN_TRACE (tf, busif_ac_u);
    PN_TRACE (tf, busif_ac_d);
    PN_TRACE (tf, busif_ac_a);

    // WB ports
    PN_TRACE (tf, wb_cyc_o);
    PN_TRACE (tf, wb_stb_o);
    PN_TRACE (tf, wb_we_o);
    PN_TRACE (tf, wb_cti_o);
    PN_TRACE (tf, wb_bte_o);
    PN_TRACE (tf, wb_sel_o);
    PN_TRACE (tf, wb_adr_o);
    PN_TRACE (tf, wb_dat_o);

    PN_TRACE (tf, wb_ack_i);
    PN_TRACE (tf, wb_dat_i);

    // TLB ports
    PN_TRACE (tf, tlb_req);
    PN_TRACE (tf, tlb_wr);
    PN_TRACE (tf, tlb_va_o);
    PN_TRACE (tf, tlb_pa_o);
    PN_TRACE (tf, tlb_superpage_o);
    PN_TRACE (tf, tlb_ac_r_o);
    PN_TRACE (tf, tlb_ac_w_o);
    PN_TRACE (tf, tlb_ac_x_o);
    PN_TRACE (tf, tlb_ac_u_o);
    PN_TRACE (tf, tlb_ac_d_o);
    PN_TRACE (tf, tlb_ac_a_o);

    PN_TRACE (tf, tlb_superpage_i);
    PN_TRACE (tf, tlb_adr_i);
    PN_TRACE (tf, tlb_hit);
    PN_TRACE (tf, tlb_miss);

    PN_TRACE (tf, tlb_ac_r_i);
    PN_TRACE (tf, tlb_ac_w_i);
    PN_TRACE (tf, tlb_ac_x_i);
    PN_TRACE (tf, tlb_ac_u_i);
    PN_TRACE (tf, tlb_ac_d_i);
    PN_TRACE (tf, tlb_ac_a_i);


    if (level > 1) {
        level--;
        //tlb.Trace (tf, level);
    }
}

void MPtw::proc_cmb_ptw () {
    SVirtAdr vadr_var;
    SPte pte_var;
    EPtwState current_state;

    // helper variables
    current_state = (EPtwState) (__uint8_t)state.read ();
    vadr_var.setUint32(virtadr.read ());
    pte_var.setUint32(pte.read ());

    // default output
    busif_ack = 0;
    tlb_req = 0;
    tlb_wr = 0;
    tlb_va_o = 0;
    busif_ac_r = pte_var.r;
    busif_ac_w = pte_var.w;
    busif_ac_x = pte_var.x;
    busif_ac_u = pte_var.u;
    busif_ac_d = pte_var.d;
    busif_ac_a = pte_var.a;
    busif_phys_adr = pte_var.page_frame () | vadr_var.page_offset;

    // WB default signals
    wb_cyc_o = 0;
    wb_stb_o = 0;
    wb_cti_o = 0;
    wb_bte_o = 0;
    wb_dat_o = 0;
    wb_we_o = 0;
    wb_adr_o = pte_var.table_pointer () | vadr_var.page_table_offset_vpn1 ();
    wb_sel_o = 0xF;

    switch (current_state) {
    case MmuIdle:
        break;
    case MmuTlbLookup:
        tlb_req = 1;
        tlb_va_o = vadr_var.vpn ();
        break;
    case MmuTlbHit:
        busif_ack = 1;
        break;
    case MmuTlbHitSuperpage:
        busif_ack = 1;
        busif_phys_adr = pte_var.page_frame () | vadr_var.superpage_offset ();
        break;
    case MmuReqPte1:
        wb_stb_o = 1;
        wb_cyc_o = 1;
        break;
    case MmuReqPte2:
        wb_stb_o = 1;
        wb_cyc_o = 1;
        wb_adr_o = pte_var.table_pointer () | vadr_var.page_table_offset_vpn0 ();
        break;
    case MmuFault:
        busif_ack = 1;
        busif_ac_r = 0;
        busif_ac_w = 0;
        busif_ac_x = 0;
        busif_ac_u = 0;
        busif_ac_d = 0;
        busif_ac_a = 0;
        break;
    case MmuSuccessSuperpage:
        busif_ack = 1;
        busif_phys_adr = pte_var.page_frame () | vadr_var.superpage_offset ();

#if CFG_MMU_TLB_ENABLE == 1
        tlb_wr = 1;
        tlb_ac_r_o = pte_var.r;
        tlb_ac_w_o = pte_var.w;
        tlb_ac_x_o = pte_var.x;
        tlb_ac_u_o = pte_var.u;
        tlb_ac_d_o = pte_var.d;
        tlb_ac_a_o = pte_var.a;
        tlb_pa_o = pte_var.page_frame_number ();
        tlb_va_o = vadr_var.vpn ();
        tlb_superpage_o = 1;
#endif
        break;
    case MmuSuccessPage:
        busif_ack = 1;

#if CFG_MMU_TLB_ENABLE == 1
        tlb_wr = 1;
        tlb_ac_r_o = pte_var.r;
        tlb_ac_w_o = pte_var.w;
        tlb_ac_x_o = pte_var.x;
        tlb_ac_u_o = pte_var.u;
        tlb_ac_d_o = pte_var.d;
        tlb_ac_a_o = pte_var.a;
        tlb_pa_o = pte_var.page_frame_number ();
        tlb_va_o = vadr_var.vpn ();
        tlb_superpage_o = 0;
#endif
        break;
    case MmuValidate1:
    case MmuValidate2:
        break;
    }
}

void MPtw::proc_clk_ptw () {

    while(1){
        EPtwState next_state, current_state;
        SPte pte_var;
        SVirtAdr vadr_var;



        current_state = next_state = (EPtwState) (__uint8_t)state.read ();
        pte_var.setUint32(pte.read ());
        vadr_var.setUint32(virtadr.read ());


        // State machine
        switch (current_state) {
        case MmuIdle:
            vadr_var.setUint32(busif_virt_adr.read ());
            pte_var.setUint32((sc_uint<2> (0), memu_root_ppn.read (), sc_uint<10> (0)));

            if (busif_req) {
                next_state = CFG_MMU_TLB_ENABLE ? MmuTlbLookup : MmuReqPte1;
            }
            break;
        case MmuTlbLookup:
            if (tlb_miss) {
                next_state = MmuReqPte1;
            } else if (tlb_hit) {
                next_state = tlb_superpage_i ? MmuTlbHitSuperpage : MmuTlbHit;
                pte_var.r = tlb_ac_r_i;
                pte_var.w = tlb_ac_w_i;
                pte_var.x = tlb_ac_x_i;
                pte_var.u = tlb_ac_u_i;
                pte_var.d = tlb_ac_d_i;
                pte_var.a = tlb_ac_a_i;
                pte_var.ppn1 = tlb_adr_i.read ().range (19, 10);
                pte_var.ppn0 = tlb_adr_i.read ().range (9, 0);
            }
            break;
        case MmuReqPte1:
            if (wb_ack_i) {
                pte_var.setUint32(wb_dat_i.read ());
                next_state = MmuValidate1;
            }
            break;
        case MmuValidate1:
            if (pte_var.is_invalid ()) {
                next_state = MmuFault;
            } else if (pte_var.is_leaf_pte ()) {
                next_state = pte_var.is_misaligned_superpage () ? MmuFault : MmuSuccessSuperpage;
            } else {
                next_state = MmuReqPte2;
            }
            break;
        case MmuReqPte2:
            if (wb_ack_i) {
                pte_var.setUint32(wb_dat_i.read ());
                next_state = MmuValidate2;
            }
            break;
        case MmuValidate2:
            if (pte_var.is_leaf_pte () && !pte_var.is_invalid ()) {
                next_state = MmuSuccessPage;
            } else {
                next_state = MmuFault;
            }
            break;

        case MmuSuccessSuperpage:
        case MmuSuccessPage:
        case MmuTlbHit:
        case MmuTlbHitSuperpage:
        case MmuFault:
        default:
            next_state = MmuIdle;
            break;
        }

        // Handle reset (must dominate)...
        if (reset) {
            next_state = MmuIdle;
            pte_var.setUint32(0);
            vadr_var.setUint32(0);
        }

        // write registers
        virtadr = vadr_var;
        pte = pte_var;
        state = next_state;
        state_trace = next_state;
        wait();
    }
   
}
