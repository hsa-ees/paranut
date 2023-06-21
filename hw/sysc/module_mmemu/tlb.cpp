/*************************************************************************

  This file is part of the ParaNut project.

  Copyright (C) 2021-2022 Christian H. Meyer <christian.meyer@hs-augsburg.de>
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


#include "tlb.h"
#include "paranut-config.h"


void MTlb::Trace (sc_trace_file *tf, int level) {
    if (!tf || pn_trace_verbose) printf ("\nSignals of Module \"%s\":\n", name ());

    // Ports ...
    PN_TRACE (tf, clk);
    PN_TRACE (tf, reset);
    
    PN_TRACE (tf, flush);

    PN_TRACE (tf, ptw_req);
    PN_TRACE (tf, ptw_superpage_i);
    PN_TRACE (tf, ptw_superpage_o);
    PN_TRACE (tf, ptw_adr_o);
    PN_TRACE (tf, ptw_hit_o);
    PN_TRACE (tf, ptw_miss_o);
    
    PN_TRACE (tf, ptw_va_i);
    PN_TRACE (tf, ptw_pa_i);

    PN_TRACE (tf, ptw_wr);
    PN_TRACE (tf, ptw_ac_a_i);
    PN_TRACE (tf, ptw_ac_d_i);
    PN_TRACE (tf, ptw_ac_r_i);
    PN_TRACE (tf, ptw_ac_w_i);
    PN_TRACE (tf, ptw_ac_x_i);
    PN_TRACE (tf, ptw_ac_u_i);

    PN_TRACE (tf, ptw_ac_a_o);
    PN_TRACE (tf, ptw_ac_d_o);
    PN_TRACE (tf, ptw_ac_r_o);
    PN_TRACE (tf, ptw_ac_w_o);
    PN_TRACE (tf, ptw_ac_x_o);
    PN_TRACE (tf, ptw_ac_u_o);


    PN_TRACE_BUS (tf, tlb_tag, CFG_MMU_TLB_ENTRIES);
    //PN_TRACE_BUS (tf, tlb_data, CFG_MMU_TLB_ENTRIES);
}

void MTlb::TransitionMethod () {
#if CFG_MMU_TLB_ENABLE == 1
    #pragma HLS ARRAY_PARTITION variable=tlb_tag complete dim = 1
    #pragma HLS ARRAY_PARTITION variable=tlb_data complete dim = 1

    STlbTag tlb_tag_var[CFG_MMU_TLB_ENTRIES];
    STlbData tlb_data_var[CFG_MMU_TLB_ENTRIES];

    bool plru[CFG_MMU_TLB_ENTRIES];
    sc_uint<CFG_MMU_TLB_ENTRIES-1> next_plru_reg;
    bool hit_var, miss_var, superpage_var, all_valid_var;
    sc_uint<20> va_var;
    sc_uint<CFG_MMU_TLB_ENTRIES> next_latest_hit;
    

    hit_var = 0;
    miss_var = 0;
    superpage_var = 0;
    ptw_adr_o = 0;
    all_valid_var = 1;

    wait();
    while(1){

        // default for registers
        next_plru_reg = plru_reg.read ();

        // helper variables
        va_var = ptw_va_i.read ();

        for (unsigned int i = 0; i < CFG_MMU_TLB_ENTRIES; i++) {
            #pragma HLS unroll
            tlb_tag_var[i] = tlb_tag[i].read ();
            tlb_data_var[i] = tlb_data[i].read ();
            all_valid_var &= tlb_tag_var[i].valid;
        }

        // level 2 index ((2*2) + (i >> 1)) - 1 
        // example:
        // level 0 simply use very first register
        // level 1 index ((1*2) + (i >> 2)) - 1
        for (sc_uint<CFG_MMU_TLB_ENTRIES_LD+1> i = 0; i < CFG_MMU_TLB_ENTRIES; i++) { // find index which plru points to
            #pragma HLS unroll
            bool is_pointed_to = 1;
            
            for (unsigned int level = CFG_MMU_TLB_ENTRIES_LD - 1; level > 0; level--) {
                #pragma HLS unroll
                unsigned int plru_index = level*2 + (i >> (CFG_MMU_TLB_ENTRIES_LD-level)) - 1;
                is_pointed_to &= !(plru_reg.read ()[plru_index] ^ i[(CFG_MMU_TLB_ENTRIES_LD-1)-level]);
            }

            plru[i] = is_pointed_to & !(plru_reg.read ()[0] ^ i[CFG_MMU_TLB_ENTRIES_LD - 1]);
        }

        if (update_plru_reg) {
            for (unsigned int level = CFG_MMU_TLB_ENTRIES_LD - 1; level > 0; level--) {
                unsigned int plru_index = level*2 + (latest_hit.read () >> (CFG_MMU_TLB_ENTRIES_LD-level)) - 1;
                next_plru_reg[plru_index] = !(latest_hit.read () & ((CFG_MMU_TLB_ENTRIES) >> (level + 1)));
            }
            
            next_plru_reg[0] = !(latest_hit.read () >> (CFG_MMU_TLB_ENTRIES_LD - 1));
        }

        update_plru_reg = 0;


        for (unsigned int i = 0; i < CFG_MMU_TLB_ENTRIES; i++) {
            #pragma HLS unroll
            bool is_equal_vpn1 = va_var.range (19, 10) == tlb_tag_var[i].vpn1;
            bool is_equal_vpn0 = va_var.range (9, 0) == tlb_tag_var[i].vpn0;
            bool hit = tlb_tag_var[i].valid && is_equal_vpn1 && (!tlb_tag_var[i].superpage ? is_equal_vpn0 : 1);
            // PN_INFOF(("Index %u: is_equal_vpn1=%u, is_equal_vpn0=%u, superpage=%u, hit=%u", i, is_equal_vpn1, is_equal_vpn0, tlb_tag_var[i].superpage, hit));

            bool is_plru = plru[i];
            sc_uint<CFG_MMU_TLB_ENTRIES-1> potential_next_plru_reg = plru_reg.read ();

            if (ptw_req) {
                if (hit) {
                    // PN_INFOF(("Hit at %u!\n", i));
                    superpage_var = tlb_tag_var[i].superpage;
                    ptw_adr_o = tlb_data_var[i].padr;
                    ptw_ac_a_o = tlb_data_var[i].ac_a;
                    ptw_ac_d_o = tlb_data_var[i].ac_d;
                    ptw_ac_r_o = tlb_data_var[i].ac_r;
                    ptw_ac_w_o = tlb_data_var[i].ac_w;
                    ptw_ac_x_o = tlb_data_var[i].ac_x;
                    ptw_ac_u_o = tlb_data_var[i].ac_u;

                    hit_var = 1;
                    next_latest_hit = i;

                    if (all_valid_var) {
                        update_plru_reg = 1;
                    }
                }
            } else if (ptw_wr) {
                if (is_plru) {
                    // PN_INFOF(("Storing entry at index %u...\n", i));
                    tlb_tag_var[i].valid = 1;
                    tlb_tag_var[i].superpage = ptw_superpage_i.read ();
                    tlb_tag_var[i].vpn1 = va_var.range(19, 10);
                    tlb_tag_var[i].vpn0 = va_var.range(9, 0);
                    tlb_data_var[i].padr = ptw_pa_i.read ();
                    tlb_data_var[i].ac_a = ptw_ac_a_i.read ();
                    tlb_data_var[i].ac_d = ptw_ac_d_i.read ();
                    tlb_data_var[i].ac_r = ptw_ac_r_i.read ();
                    tlb_data_var[i].ac_w = ptw_ac_w_i.read ();
                    tlb_data_var[i].ac_x = ptw_ac_x_i.read ();
                    tlb_data_var[i].ac_u = ptw_ac_u_i.read ();

                    update_plru_reg = 1;
                    next_latest_hit = i;
                }
            }        
        }

        miss_var = !hit_var && ptw_req;

        // reset valid bit is sufficient on reset
        if (reset | flush) {
            for (int i = 0; i < CFG_MMU_TLB_ENTRIES; i++) {
                #pragma HLS unroll
                tlb_tag_var[i].valid = 0;
            }
            next_plru_reg = 0;
        }

        // update output
        ptw_hit_o = hit_var;
        ptw_miss_o = miss_var;
        ptw_superpage_o = superpage_var;

        // update registers
        plru_reg = next_plru_reg;
        latest_hit = next_latest_hit;

        for (int i = 0; i < CFG_MMU_TLB_ENTRIES; i++) {
            #pragma HLS unroll
            tlb_tag[i] = tlb_tag_var[i];
            tlb_data[i] = tlb_data_var[i];
        }
        wait();
    }
#endif
}