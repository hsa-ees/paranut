/*************************************************************************

  This file is part of the ParaNut project.

  Copyright (C) 2020-2022 Christian H. Meyer <christian.meyer@hs-augsburg.de>
      Hochschule Augsburg, University of Applied Sciences

  Description:
    This is a SystemC model of the Page Table Walker (PTW) of the ParaNut,
    which is is closely coupled to the MBusIf.
    At first, it does a lookup in the Translation Lookaside Buffer. If the virtual
    address is not found, it walks the page table.

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

#pragma once

#include "paranut-config.h"
#include "base.h"

#define MAX_ADR_LENGTH 20U

#define PHYSADR_SP_LENGTH 10U
#define PHYSADR_LENGTH 20U

#define VIRTADR_SP_LENGTH 10U
#define VIRTADR_LENGTH 20U

#define TAG_SP_LENGTH (PHYSADR_SP_LENGTH - CFG_MMU_TLB_WAYS_LD)
#define TAG_LENGTH (PHYSADR_LENGTH - CFG_MMU_TLB_WAYS_LD)

#define CFG_MMU_TLB_CNTR_LENGTH (CFG_MMU_TLB_REPLACE_LRU ? (6 + CFG_MMU_SP_TLB_WAYS_LD) : (6 + CFG_MMU_TLB_WAYS_LD))
#define CFG_MMU_SP_TLB_CNTR_LENGTH (CFG_MMU_SP_TLB_REPLACE_LRU ? (6 + CFG_MMU_TLB_WAYS_LD) : (6 + CFG_MMU_SP_TLB_WAYS_LD))

#define CNTR_LENGTH MAX(CFG_MMU_TLB_CNTR_LENGTH, CFG_MMU_SP_TLB_CNTR_LENGTH)


typedef enum {
    MmuIdle = 0,
    MmuTlbLookup,
    MmuTlbHit,
    MmuTlbHitSuperpage,
    MmuReqPte1,
    MmuValidate1,
    MmuReqPte2,
    MmuValidate2,
    MmuFault,
    MmuSuccessSuperpage,
    MmuSuccessPage,
} EPtwState;

struct SVirtAdr
{
    sc_uint<10> vpn1;
    sc_uint<10> vpn0;
    sc_uint<12> page_offset;

    sc_uint<20> vpn () { return (this->vpn1, this->vpn0); }
    sc_uint<22> superpage_offset () { return (this->vpn0, this->page_offset); }
    sc_uint<12> page_table_offset_vpn1 () { return (this->vpn1, sc_uint<2>(0)); }
    sc_uint<12> page_table_offset_vpn0 () { return (this->vpn0, sc_uint<2>(0)); }

    operator const sc_uint<32> () { return (vpn1, vpn0, page_offset); };

    void setUint32 (const sc_uint<32> &data)
    {
        vpn1 = data.range (31, 22);
        vpn0 = data.range (21, 12);
        page_offset = data.range (11, 0);
    };
    
    bool operator==(const SVirtAdr &t) const
    {
        return C(vpn1) && C(vpn0) && C(page_offset);
    }

    friend ostream &operator<<(ostream &os, const SVirtAdr &t)
    {
        os << "vpn1=" << t.vpn1 << "vpn0=" << t.vpn0 << "page_offset=" << t.page_offset;
        return os;
    }

#ifndef __SYNTHESIS__
    // Trace...
    friend void sc_trace(sc_trace_file *tf, const SVirtAdr &t, const std::string &name)
    {
        PN_TRACE_R (tf, t, vpn1, name);
        PN_TRACE_R (tf, t, vpn0, name);
        PN_TRACE_R (tf, t, page_offset, name);
    }
#endif
};

// Page Table Entry
struct SPte
{
    sc_uint<12> ppn1;
    sc_uint<10> ppn0;
    sc_uint<2> reserved_for_software;
    bool v;
    bool g;
    bool r;
    bool w;
    bool u;
    bool x;
    bool a;
    bool d;

    sc_uint<32> page_frame() { return (this->ppn1.range(9, 0), this->ppn0, sc_uint<12>(0)); }
    sc_uint<20> page_frame_number() { return (this->ppn1.range(9, 0), this->ppn0); }
    sc_uint<32> table_pointer() { return (this->ppn1.range(9, 0), this->ppn0, sc_uint<12>(0)); }

    operator sc_uint<32>()
    {
        return (ppn1, ppn0, reserved_for_software, d, a, g, u, x, w, r, v);
    }

    bool is_invalid() { return !this->v || (!this->r && this->w); };
    bool is_misaligned_superpage() { return !this->is_aligned_superpage(); };
    bool is_aligned_superpage() { return this->ppn0 == 0; }
    bool is_leaf_pte() { return this->r || this->x; };

    void setUint32(const sc_uint<32> &data)
    {
        ppn1 = data.range (31, 20);
        ppn0 = data.range (19, 10);
        reserved_for_software = data.range (9, 8);
        d = data[7];
        a = data[6];
        g = data[5];
        u = data[4];
        x = data[3];
        w = data[2];
        r = data[1];
        v = data[0];
    }

    bool operator==(const SPte &t) const
    {
        return C(ppn1) && C(ppn0) && C(reserved_for_software) && C(v) && C(r) && C(w) && C(x) && C(u) && C(g) && C(a) && C(d);
    }

    friend ostream &operator<<(ostream &os, const SPte &t)
    {
        os << "ppn1=" << t.ppn1 << ",ppn0=" << t.ppn0
        << ",reserved_for_software=" << t.reserved_for_software 
        << "v=" << t.v << ",g=" << t.g << "," << ",d:" << t.d << ",a:" << t.a << ",g:" << t.g 
        << ",u:" << t.u << ",x:" << t.x << ",w:" << t.w << ",r:" << t.r << ",v:" << t.v;
        return os;
    }

#ifndef __SYNTHESIS__
    // Trace...
    friend void sc_trace(sc_trace_file *tf, const SPte &t, const std::string &name)
    {
        PN_TRACE_R (tf, t, ppn1, name);
        PN_TRACE_R (tf, t, ppn0, name);
        PN_TRACE_R (tf, t, reserved_for_software, name);
        PN_TRACE_R (tf, t, g, name);
        PN_TRACE_R (tf, t, v, name);
        PN_TRACE_R (tf, t, d, name);
        PN_TRACE_R (tf, t, a, name);
        PN_TRACE_R (tf, t, g, name);
        PN_TRACE_R (tf, t, u, name);
        PN_TRACE_R (tf, t, r, name);
        PN_TRACE_R (tf, t, w, name);
        PN_TRACE_R (tf, t, x, name);
    }
#endif
};

SC_MODULE(MPtw)
{
public:
    sc_in<bool> clk;
    sc_in<bool> reset;

    // Wishbone ports
    sc_out<bool>        wb_cyc_o;                       // cycle valid output
    sc_out<bool>        wb_stb_o;                       // strobe output
    sc_out<bool>        wb_we_o;                        // indicates write transfer
    sc_out<sc_uint<3> > wb_cti_o;                       // cycle type identifier
    sc_out<sc_uint<2> > wb_bte_o;                       // burst type extension
    sc_out<sc_uint<CFG_MEMU_BUSIF_WIDTH/8> > wb_sel_o;  // byte select outputs
    sc_out<sc_uint<32> > wb_adr_o;                      // address bus outputs
    sc_out<sc_uint<CFG_MEMU_BUSIF_WIDTH> > wb_dat_o;    // output data bus

    sc_in<bool>         wb_ack_i;                       // normal termination
    sc_in<sc_uint<CFG_MEMU_BUSIF_WIDTH> > wb_dat_i;     // input data bus

    
    // To MemU
    sc_in<sc_uint<20> > memu_root_ppn;
    
    // To BusIf
    sc_in<bool> busif_req; 
    sc_out<bool> busif_ack;
    sc_in<sc_uint<32> > busif_virt_adr; 
    sc_out<sc_uint<32> > busif_phys_adr; 
    sc_out<bool> busif_ac_r;
    sc_out<bool> busif_ac_w;
    sc_out<bool> busif_ac_x;
    sc_out<bool> busif_ac_u;
    sc_out<bool> busif_ac_d;
    sc_out<bool> busif_ac_a;

    // To TLB
    sc_out<bool> tlb_req, tlb_wr;
    sc_out<sc_uint<20> > tlb_va_o;
    sc_out<sc_uint<20> > tlb_pa_o;
    sc_out<bool> tlb_superpage_o;
    sc_out<bool> tlb_ac_r_o;
    sc_out<bool> tlb_ac_w_o;
    sc_out<bool> tlb_ac_x_o;
    sc_out<bool> tlb_ac_u_o;
    sc_out<bool> tlb_ac_d_o;
    sc_out<bool> tlb_ac_a_o;

    sc_in<bool> tlb_superpage_i;
    sc_in<sc_uint<20> > tlb_adr_i;
    sc_in<bool> tlb_hit;
    sc_in<bool> tlb_miss;

    sc_in<bool> tlb_ac_r_i;
    sc_in<bool> tlb_ac_w_i;
    sc_in<bool> tlb_ac_x_i;
    sc_in<bool> tlb_ac_u_i;
    sc_in<bool> tlb_ac_d_i;
    sc_in<bool> tlb_ac_a_i;

    SC_HAS_PROCESS(MPtw);
    MPtw(sc_module_name name)
       // : sc_module(name), tlb("TLB")
    {
        SC_METHOD(proc_cmb_ptw);
            sensitive << pte << state << virtadr << tlb_ac_r_i << tlb_ac_w_i;
            sensitive << reset << tlb_hit << busif_virt_adr << virtadr;
        SC_CTHREAD(proc_clk_ptw, clk.pos ());
    };
    void Trace(sc_trace_file * tf, int level = 1);
    
protected:
    void proc_cmb_ptw();
    void proc_clk_ptw();

    sc_signal<sc_uint<32> > pte;
    sc_signal<sc_uint<32> > virtadr;
    sc_signal<sc_uint<4> > state;

    sc_signal<int> state_trace;
};




