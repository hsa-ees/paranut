/*************************************************************************

  This file is part of the ParaNut project.

  Copyright (C) 2023 Marco Milenkovic <Marco.Milenkovic@hs-augsburg.de>
      Hochschule Augsburg, University of Applied Sciences

  Description: Testbench Module for CSR for use with ICSC
               TODO: implement the testbench for CSR

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
#include <systemc.h>
#include "base.h"
#include "csr.h"

struct Tb : sc_module
{
    // Ports ...
    sc_in_clk clk;
    sc_signal<bool> reset;
    sc_signal<sc_uint<32> > pc, ir;
    sc_signal<bool> exu_pop_priv_ir_stack_dreg; // trigger pushing the Privilege and Global Interrupt-Enable Stack

    sc_signal<sc_uint<EX_ID_LENGTH> > ex_id_reg;
    sc_signal<bool> exception;  // signal from ExU. Triggers saving of the exception state CSRs (mepc, mcause, ...)
    sc_signal<bool> irq_dreg;  // Set if m3_ir_request is set during instruction decode (handle interrupt request)
    sc_signal<bool> irq_exec;  // Set if m3_ir_request is set during instruction decode (handle interrupt request)

    sc_signal<bool> mtie_out;  // mtie bit of mie register is set
    sc_signal<bool> mtip_in;    // mtip bit of mip register
    sc_signal<bool> meip_in;    // meip bit of mip register

    // misc (HLS reasons, should be optimized out while synthetization)
    // these could be generics in VHDL, but current tools don't support that
    sc_signal<sc_uint<CFG_NUT_CPU_CORES_LD> > hartID;
    sc_signal<bool> inCePU;
    sc_signal<bool> cpu_enabled, linked;
    sc_signal<bool> csr_enable;
    sc_signal<sc_uint<5> > exu_cause;

    sc_signal<bool> exu_dbg, exu_dbg_reg, exu_dbg_req, exu_dbg_enter_dreg;

    // Decode Registers
    sc_signal<bool> exu_delegate_dreg;
    sc_signal<bool> sret_dreg;

    sc_signal<bool> exu_csr_exception; // Writes to read-only CSRs raise an exeption

    sc_signal<bool>
            exu_csr_mstatus_MIE, // Machine Interrupt Enable
            exu_csr_mstatus_SIE,
            exu_csr_mstatus_TSR, // Trap SRET
            exu_csr_mstatus_TVM, // Trap Virtual Memory
            exu_csr_mstatus_SUM,
            exu_csr_dcsr_step,
            exu_csr_dcsr_ebreakm;
    sc_signal<bool> exu_isHalted;

    sc_signal<sc_uint<2> > exu_load_store_priv_mode;

    // Full registers
    sc_signal<sc_uint<32> >
            exu_csr_mepc,
            exu_csr_dpc,
            exu_csr_mtvec,
            exu_csr_mcause,
            exu_csr_sepc,
            exu_csr_stvec,
            exu_csr_mideleg,
            exu_csr_mip;


    sc_signal<bool> exu_ifu_paging_mode; // for instructions
    sc_signal<bool> exu_lsu_paging_mode; // for load-store OPs
    sc_signal<sc_uint<20> > exu_csr_satp_root_ppn;

    sc_signal<sc_uint<CFG_NUT_CPU_CORES> > exu_m3_pnce;
    sc_signal<sc_uint<CFG_NUT_CPU_CORES> > exu_m3_pnlm;
    sc_signal<sc_uint<CFG_NUT_CPU_CORES> > exu_m3_pnxsel;
    sc_signal<sc_uint<2> > exu_m3_priv_mode;

    // EXU attention signals
    sc_signal<bool> exu_cache_flush;
    sc_signal<bool> exu_ack;

    // controller outputs...
    sc_signal<bool> exu_m3_icache_enable, exu_m3_dcache_enable;

    sc_signal<sc_uint<32> > csr_tval;   // Special signal for exception information input
    sc_signal<sc_uint<CFG_NUT_CPU_CORES> > m3_pnhaltreq;

    sc_signal<sc_uint<3> > csr_function_reg;
    sc_signal<sc_uint<5> > csr_rs1_reg;
    sc_signal<sc_uint<12> > csr_adr_reg;
    sc_signal<sc_uint<32> > csr_op_a;
    sc_signal<sc_uint<32> > exu_csr_rdata;

    sc_signal<sc_uint<2> > exu_priv_mode;

    // from CoPUs
    sc_signal<sc_uint<CFG_NUT_CPU_CORES> > m3_pnx;

#if CFG_EXU_PERFCOUNT_ENABLE == 1
    sc_signal<bool> perf_inc;
    sc_signal<sc_uint<CFG_EXU_PERFCOUNTERS_LD> > perf_addr;
#endif

    MCsr dut_inst{"dut_inst"};

    SC_CTOR(Tb)
    {
        dut_inst.clk(clk);
        dut_inst.reset(reset);
        dut_inst.pc(pc);
        dut_inst.ir(ir);
        dut_inst.exu_pop_priv_ir_stack_dreg(exu_pop_priv_ir_stack_dreg);
        dut_inst.ex_id_reg(ex_id_reg);
        dut_inst.exception(exception);
        dut_inst.irq_dreg(irq_dreg);
        dut_inst.mtie_out(mtie_out);
        dut_inst.mtip_in(mtip_in);
        dut_inst.meip_in(meip_in);
        dut_inst.hartID(hartID);
        dut_inst.inCePU(inCePU);
        dut_inst.cpu_enabled(cpu_enabled);
        dut_inst.linked(linked);
        dut_inst.csr_enable(csr_enable);
        dut_inst.exu_cause(exu_cause);
        dut_inst.exu_dbg(exu_dbg);
        dut_inst.exu_dbg_reg(exu_dbg_reg);
        dut_inst.exu_dbg_req(exu_dbg_req);
        dut_inst.exu_dbg_enter_dreg(exu_dbg_enter_dreg);
        dut_inst.exu_delegate_dreg(exu_delegate_dreg);
        dut_inst.sret_dreg(sret_dreg);
        dut_inst.exu_csr_exception(exu_csr_exception);
        dut_inst.exu_csr_mstatus_MIE(exu_csr_mstatus_MIE);
        dut_inst.exu_csr_mstatus_SIE(exu_csr_mstatus_SIE);
        dut_inst.exu_csr_mstatus_TSR(exu_csr_mstatus_TSR);
        dut_inst.exu_csr_mstatus_TVM(exu_csr_mstatus_TVM);
        dut_inst.exu_csr_mstatus_SUM(exu_csr_mstatus_SUM);
        dut_inst.exu_csr_dcsr_step(exu_csr_dcsr_step);
        dut_inst.exu_csr_dcsr_ebreakm(exu_csr_dcsr_ebreakm);
        dut_inst.exu_isHalted(exu_isHalted);
        dut_inst.exu_load_store_priv_mode(exu_load_store_priv_mode);
        dut_inst.exu_csr_mepc(exu_csr_mepc);
        dut_inst.exu_csr_dpc(exu_csr_dpc);
        dut_inst.exu_csr_mtvec(exu_csr_mtvec);
        dut_inst.exu_csr_mcause(exu_csr_mcause);
        dut_inst.exu_csr_sepc(exu_csr_sepc);
        dut_inst.exu_csr_stvec(exu_csr_stvec);
        dut_inst.exu_csr_mideleg(exu_csr_mideleg);
        dut_inst.exu_csr_mip(exu_csr_mip);
        dut_inst.exu_ifu_paging_mode(exu_ifu_paging_mode);
        dut_inst.exu_lsu_paging_mode(exu_lsu_paging_mode);
        dut_inst.exu_csr_satp_root_ppn(exu_csr_satp_root_ppn);
        dut_inst.exu_m3_pnce(exu_m3_pnce);
        dut_inst.exu_m3_pnlm(exu_m3_pnlm);
        dut_inst.exu_m3_pnxsel(exu_m3_pnxsel);
        dut_inst.exu_m3_priv_mode(exu_m3_priv_mode);
        dut_inst.exu_cache_flush(exu_cache_flush);
        dut_inst.exu_ack(exu_ack);
        dut_inst.exu_m3_icache_enable(exu_m3_icache_enable);
        dut_inst.exu_m3_dcache_enable(exu_m3_dcache_enable);
        dut_inst.csr_tval(csr_tval);
        dut_inst.m3_pnhaltreq(m3_pnhaltreq);
        dut_inst.csr_function_reg(csr_function_reg); 
        dut_inst.csr_rs1_reg(csr_rs1_reg);
        dut_inst.csr_adr_reg(csr_adr_reg);
        dut_inst.csr_op_a(csr_op_a);
        dut_inst.exu_csr_rdata(exu_csr_rdata);
        dut_inst.exu_priv_mode(exu_priv_mode);
        dut_inst.m3_pnx(m3_pnx);

#if CFG_EXU_PERFCOUNT_ENABLE == 1
        dut_inst.perf_inc(perf_inc);
        dut_inst.perf_addr(perf_addr);
#endif
    
        SC_CTHREAD(test, clk.pos());
        reset_signal_is(reset, true);
    }

    void test()
    {
        // Trace file...
        sc_trace_file *tf;
        if (pn_cfg_vcd_level > 0) {
            tf = sc_create_vcd_trace_file ("dm_tb");
            tf->delta_cycles (false);
            //Trace DUT signals
            PN_TRACE (tf, clk);
            PN_TRACE (tf, reset);
            PN_TRACE (tf, pc);
            PN_TRACE (tf, ir);
            PN_TRACE (tf, exu_pop_priv_ir_stack_dreg);
            PN_TRACE (tf, ex_id_reg);
            PN_TRACE (tf, exception);
            PN_TRACE (tf, irq_dreg);
            // add further signals here
            
            


            // dut_inst.Trace(tf, pn_cfg_vcd_level);

        } else {
            fprintf (stderr, "Tracing is disabled.\n");
            tf = NULL;
        }
        // add Testbench Code here

        reset = 0;
        wait(5);
        reset = 1;
        wait(5);
        PN_INFO("Testbench started");
        wait();
        PN_INFO("Testbench finished");
        sc_stop();
        wait();
    }   
};



int sc_main (int argc, char **argv) 
{
    sc_clock clk{"clk", sc_time(1, SC_NS)};
    Tb tb("tb");
    tb.clk(clk);
    cout <<"Simulation Start" << endl;
    sc_start();
    cout <<"\n\t\t*****Simulation complete*****" << endl;
    return 0;
}