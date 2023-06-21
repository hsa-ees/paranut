/*************************************************************************

  This file is part of the ParaNut project.

  Copyright (C) 2022-2023 Elias Schuler <elias.schuler@hs-augsburg.de>
                2022-2023 Lukas Bauer <lukas.bauer@hs-ausgsburg.de>
      Hochschule Augsburg, University of Applied Sciences

    Description : This file is based on the work of Sebastian Witt and Federico Aglietti
                  Wishbone UART

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

 **************************************************************************/

#include "wb8_uart_16750.h"
#include "paranut-config.h"


void Wb8Uart16750::InitSubModules(){

     // fprintf(stderr,"InitSubModules");
    NC_fe_o = 0;
    NC_fe_o_2 = 0;
    NC_fe_o_3 = 0;
    NC_fe_o_4 = 0;
    NC_fe_o_5 = 0;
    NC_i = 0;

    //Set Partial Signals to Zero
    iIER = 0;
    iLSR = 0;
    iMSR = 0;
    iIIR = 0;

    uart_is_sin.clk_i (clk_i);
    uart_is_sin.rst_i (rst_i);
    uart_is_sin.d_i (sin_i);
    uart_is_sin.q_o (iSINr);

    uart_is_cts.clk_i (clk_i);
    uart_is_cts.rst_i (rst_i);
    uart_is_cts.d_i (ctsn_i);
    uart_is_cts.q_o (iCTSNs);


    uart_is_dsr.clk_i (clk_i);
    uart_is_dsr.rst_i (rst_i);
    uart_is_dsr.d_i (dsrn_i);
    uart_is_dsr.q_o (iDSRNs);

    uart_is_dcd.clk_i (clk_i);
    uart_is_dcd.rst_i (rst_i);
    uart_is_dcd.d_i (dcd_i);
    uart_is_dcd.q_o (iDCDNs);

    uart_is_ri.clk_i (clk_i);
    uart_is_ri.rst_i (rst_i);
    uart_is_ri.d_i (rin_i);
    uart_is_ri.q_o (iRINs);

    uart_if_cts.clk_i (clk_i);
    uart_if_cts.rst_i (rst_i);
    uart_if_cts.ce_i (iBaudtick2x);
    uart_if_cts.d_i (iCTSNs);
    uart_if_cts.q_o (iCTSn);

    uart_if_dsr.clk_i (clk_i);
    uart_if_dsr.rst_i (rst_i);
    uart_if_dsr.ce_i (iBaudtick2x);
    uart_if_dsr.d_i (iDSRNs);
    uart_if_dsr.q_o (iDSRn);

    uart_if_dcd.clk_i (clk_i);
    uart_if_dcd.rst_i (rst_i);
    uart_if_dcd.ce_i (iBaudtick2x);
    uart_if_dcd.d_i (iDCDNs);
    uart_if_dcd.q_o (iDCDn);

    uart_if_ri.clk_i (clk_i);
    uart_if_ri.rst_i (rst_i);
    uart_if_ri.ce_i (iBaudtick2x);
    uart_if_ri.d_i (iRINs);
    uart_if_ri.q_o (iRIn);

    uart_iic.clk_i (clk_i);
    uart_iic.rst_i (rst_i);
    uart_iic.ire_i (iIER_3_0);
    uart_iic.lsr_i (iLSR_4_0);
    uart_iic.thi_i (iTHRInterrupt);
    uart_iic.rda_i (iRDAInterrupt);
    uart_iic.cti_i (iCharTimeout);
    uart_iic.afe_i (iMCR_AFE);
    uart_iic.msr_i (iMSR_3_0);
    uart_iic.iir_o (iIIR_3_0);
    uart_iic.int_o (int_o);

    uart_iic_thre_ed.clk_i (clk_i);
    uart_iic_thre_ed.rst_i (rst_i);
    uart_iic_thre_ed.d_i (iLSR_THRE);
    uart_iic_thre_ed.re_o (iLSR_THRERE);
    uart_iic_thre_ed.fe_o(NC_fe_o);

    uart_pedet.clk_i (clk_i);
    uart_pedet.rst_i (rst_i);
    uart_pedet.d_i (iRXFIFOPE);
    uart_pedet.re_o (iPERE);
    uart_pedet.fe_o(NC_fe_o_2);

    uart_fedet.clk_i (clk_i);
    uart_fedet.rst_i (rst_i);
    uart_fedet.d_i (iRXFIFOFE);
    uart_fedet.re_o (iFERE);
    uart_fedet.fe_o (NC_fe_o_3);

    uart_bidet.clk_i (clk_i);
    uart_bidet.rst_i (rst_i);
    uart_bidet.d_i (iRXFIFOBI);
    uart_bidet.re_o (iBIRE);
    uart_bidet.fe_o (NC_fe_o_4);

    uart_ed_cts.clk_i (clk_i);
    uart_ed_cts.rst_i (rst_i);
    uart_ed_cts.d_i (iMSR_CTS);
    uart_ed_cts.re_o (iCTSnRE);
    uart_ed_cts.fe_o (iCTSnFE);

    uart_ed_dsr.clk_i (clk_i);
    uart_ed_dsr.rst_i (rst_i);
    uart_ed_dsr.d_i (iMSR_DSR);
    uart_ed_dsr.re_o (iDSRnRE);
    uart_ed_dsr.fe_o (iDSRnFE);

    uart_ed_ri.clk_i (clk_i);
    uart_ed_ri.rst_i (rst_i);
    uart_ed_ri.d_i (iMSR_RI);
    uart_ed_ri.re_o (iRInRE);
    uart_ed_ri.fe_o (iRInFE);

    uart_ed_dcd.clk_i (clk_i);
    uart_ed_dcd.rst_i (rst_i);
    uart_ed_dcd.d_i (iMSR_DCD);
    uart_ed_dcd.re_o (iDCDnRE);
    uart_ed_dcd.fe_o (iDCDnFE);

    uart_bg16.clk_i (clk_i);
    uart_bg16.rst_i (rst_i);
    uart_bg16.ce_i (clock_enable);
    uart_bg16.clear_i (NC_i);
    uart_bg16.divider_i (iBaudgenDiv);
    uart_bg16.baudtick_o (iBaudtick16x);

    uart_bg1.clk_i (clk_i);
    uart_bg1.rst_i(rst_i);
    uart_bg1.ce_i(baudce_i);
    uart_bg1.q_o(clock_enable);

    uart_bg2.clk_i (clk_i);
    uart_bg2.rst_i (rst_i);
    uart_bg2.ce_i (iBaudtick16x);
    uart_bg2.q_o (iBaudtick2x);

    uart_rclk.clk_i (clk_i);
    uart_rclk.rst_i (rst_i);
    uart_rclk.d_i (rclk_i);
    uart_rclk.re_o (iRCLK);
    uart_rclk.fe_o (NC_fe_o_5);

    uart_txff.clk_i (clk_i);
    uart_txff.rst_i (rst_i);
    uart_txff.clear_i (iTXFIFOClear);
    uart_txff.write_i (iTXFIFOWrite);
    uart_txff.read_i (iTXFIFORead);
    uart_txff.d_i (iDIN);
    uart_txff.q_o (iTXFIFOQ);
    uart_txff.empty_o (iTXFIFOEmpty);
    uart_txff.full_o (iTXFIFO64Full);
    uart_txff.usage_o (iTXFIFOUsage);

    uart_rxff.clk_i (clk_i);
    uart_rxff.rst_i (rst_i);
    uart_rxff.clear_i (iRXFIFOClear);
    uart_rxff.write_i (iRXFIFOWrite);
    uart_rxff.read_i (iRXFIFORead);
    uart_rxff.d_i (iRXFIFOD);
    uart_rxff.q_o (iRXFIFOQ);
    uart_rxff.empty_o (iRXFIFOEmpty);
    uart_rxff.full_o (iRXFIFO64Full);
    uart_rxff.usage_o (iRXFIFOUsage);

    uart_tx.clk_i (clk_i);
    uart_tx.rst_i (rst_i);
    uart_tx.tx_clk_i (iBaudtick2x);
    uart_tx.tx_start_i (iTXStart);
    uart_tx.clear_i (iTXClear);
    uart_tx.wls_i (iLCR_WLS);
    uart_tx.stb_i (iLCR_STB);
    uart_tx.pen_i (iLCR_PEN);
    uart_tx.eps_i (iLCR_EPS);
    uart_tx.sp_i (iLCR_SP);
    uart_tx.bc_i (iLCR_BC);
    uart_tx.din_i (iTSR);
    uart_tx.tx_finished_o (iTXFinished);
    uart_tx.sout_o (iSOUT);

    uart_rx.clk_i (clk_i);
    uart_rx.rst_i (rst_i);
    uart_rx.rx_clk_i (iRCLK);
    uart_rx.rx_clear_i (iRXClear);
    uart_rx.wls_i (iLCR_WLS);
    uart_rx.stb_i (iLCR_STB);
    uart_rx.pen_i (iLCR_PEN);
    uart_rx.eps_i (iLCR_EPS);
    uart_rx.sp_i (iLCR_SP);
    uart_rx.sin_i (iSIN);
    uart_rx.pe_o (iRXPE);
    uart_rx.fe_o (iRXFE);
    uart_rx.bi_o (iRXBI);
    uart_rx.dout_o (iRXData);
    uart_rx.rx_finished_o (iRXFinished);

}
#ifndef __SYNTHESIS__
void Wb8Uart16750::Trace(sc_trace_file *tf, int level){
    if (!tf || pn_trace_verbose) printf ("\nSignals of Module \"%s\":\n", name ());


    PN_TRACE(tf, clk_i);
    PN_TRACE(tf, rst_i);
    PN_TRACE(tf, baudce_i);
    PN_TRACE(tf, wb_cyc_i);
    PN_TRACE(tf, wb_stb_i);
    PN_TRACE(tf, wb_we_i);
    PN_TRACE(tf, wb_adr_i);
    PN_TRACE(tf, wb_din_i);
    PN_TRACE(tf, wb_dout_o);
    PN_TRACE(tf, wb_ack_o);
    PN_TRACE(tf, int_o);
    PN_TRACE(tf, out1_o);
    PN_TRACE(tf, out2_o);
    PN_TRACE(tf, rclk_i);
    PN_TRACE(tf, baudoutn_o);
    PN_TRACE(tf, rtsn_o);
    PN_TRACE(tf, dtrn_o);
    PN_TRACE(tf, ctsn_i);
    PN_TRACE(tf, dsrn_i);
    PN_TRACE(tf, dcd_i);
    PN_TRACE(tf, rin_i);
    PN_TRACE(tf, sin_i);
    PN_TRACE(tf, sout_o);

    PN_TRACE(tf, iWriteFE);
    PN_TRACE(tf, iReadFE);
    PN_TRACE(tf, iWrite);
    PN_TRACE(tf, iRead);
    PN_TRACE(tf, iA);
    PN_TRACE(tf, iDIN);

    PN_TRACE(tf, iRBRRead);
    PN_TRACE(tf, iTHRWrite);
    PN_TRACE(tf, iDLLWrite);
    PN_TRACE(tf, iDLMWrite);
    PN_TRACE(tf, iIERWrite);
    PN_TRACE(tf, iIIRRead);
    PN_TRACE(tf, iFCRWrite);
    PN_TRACE(tf, iLCRWrite);
    PN_TRACE(tf, iMCRWrite);
    PN_TRACE(tf, iLSRRead);
    PN_TRACE(tf, iMSRRead);
    PN_TRACE(tf, iSCRWrite);

    PN_TRACE(tf, iTSR);
    PN_TRACE(tf, iRBR);
    PN_TRACE(tf, iDLL);
    PN_TRACE(tf, iDLM);
    PN_TRACE(tf, iIER);
    PN_TRACE(tf, iIIR);
    PN_TRACE(tf, iFCR);
    PN_TRACE(tf, iLCR);
    PN_TRACE(tf, iMCR);
    PN_TRACE(tf, iLSR);
    PN_TRACE(tf, iMSR);
    PN_TRACE(tf, iSCR);

    PN_TRACE(tf, iIER_ERBI);
    PN_TRACE(tf, iIER_ETBEI);
    PN_TRACE(tf, iIER_ELSI);
    PN_TRACE(tf, iIER_EDSSI);

    PN_TRACE(tf, iIIR_PI);
    PN_TRACE(tf, iIIR_ID0);
    PN_TRACE(tf, iIIR_ID1);
    PN_TRACE(tf, iIIR_ID2);
    PN_TRACE(tf, iIIR_FIFO64);

    PN_TRACE(tf, iFCR_FIFOEnable);
    PN_TRACE(tf, iFCR_RXFIFOReset);
    PN_TRACE(tf, iFCR_TXFIFOReset);
    PN_TRACE(tf, iFCR_DMAMode);
    PN_TRACE(tf, iFCR_FIFO64E);
    PN_TRACE(tf, iFCR_RXTrigger);

    PN_TRACE(tf, iLCR_WLS);
    PN_TRACE(tf, iLCR_STB);
    PN_TRACE(tf, iLCR_PEN);
    PN_TRACE(tf, iLCR_EPS);
    PN_TRACE(tf, iLCR_SP);
    PN_TRACE(tf, iLCR_BC);
    PN_TRACE(tf, iLCR_DLAB);

    PN_TRACE(tf, iMCR_DTR);
    PN_TRACE(tf, iMCR_RTS);
    PN_TRACE(tf, iMCR_OUT1);
    PN_TRACE(tf, iMCR_OUT2);
    PN_TRACE(tf, iMCR_LOOP);
    PN_TRACE(tf, iMCR_AFE);

    PN_TRACE(tf, iLSR_DR);
    PN_TRACE(tf, iLSR_OE);
    PN_TRACE(tf, iLSR_PE);
    PN_TRACE(tf, iLSR_FE);
    PN_TRACE(tf, iLSR_BI);
    PN_TRACE(tf, iLSR_THRE);
    PN_TRACE(tf, iLSR_FIFOERR);

    PN_TRACE(tf, iMSR_dCTS);
    PN_TRACE(tf, iMSR_dDSR);
    PN_TRACE(tf, iMSR_TERI);
    PN_TRACE(tf, iMSR_dDCD);
    PN_TRACE(tf, iMSR_CTS);
    PN_TRACE(tf, iMSR_DSR);
    PN_TRACE(tf, iMSR_RI);
    PN_TRACE(tf, iMSR_DCD);

    PN_TRACE(tf, iCTSNs);
    PN_TRACE(tf, iDSRNs);
    PN_TRACE(tf, iDCDNs);
    PN_TRACE(tf, iRINs);
    PN_TRACE(tf, iCTSn);
    PN_TRACE(tf, iDSRn);
    PN_TRACE(tf, iDCDn);
    PN_TRACE(tf, iRIn);
    PN_TRACE(tf, iCTSnRE);
    PN_TRACE(tf, iCTSnFE);
    PN_TRACE(tf, iDSRnRE);
    PN_TRACE(tf, iDSRnFE);
    PN_TRACE(tf, iDCDnRE);
    PN_TRACE(tf, iDCDnFE);
    PN_TRACE(tf, iRInRE);
    PN_TRACE(tf, iRInFE);

    PN_TRACE(tf, iBaudgenDiv);
    PN_TRACE(tf, iBaudtick16x);
    PN_TRACE(tf, iBaudtick2x);
    PN_TRACE(tf, iRCLK);

    PN_TRACE(tf, iTXFIFOClear);
    PN_TRACE(tf, iTXFIFOWrite);
    PN_TRACE(tf, iRXFIFORead);
    PN_TRACE(tf, iTXFIFOEmpty);
    PN_TRACE(tf, iTXFIFOFull);
    PN_TRACE(tf, iTXFIFO16Full);
    PN_TRACE(tf, iTXFIFO64Full);
    PN_TRACE(tf, iTXFIFOUsage);
    PN_TRACE(tf, iTXFIFOQ);
    PN_TRACE(tf, iRXFIFOClear);
    PN_TRACE(tf, iRXFIFOWrite);
    PN_TRACE(tf, iRXFIFORead);
    PN_TRACE(tf, iRXFIFOEmpty);
    PN_TRACE(tf, iRXFIFOFull);
    PN_TRACE(tf, iRXFIFO16Full);
    PN_TRACE(tf, iRXFIFO64Full);
    PN_TRACE(tf, iRXFIFOD);
    PN_TRACE(tf, iRXFIFOQ);
    PN_TRACE(tf, iRXFIFOUsage);
    PN_TRACE(tf, iRXFIFOTrigger);
    PN_TRACE(tf, iRXFIFO16Trigger);
    PN_TRACE(tf, iRXFIFO64Trigger);
    PN_TRACE(tf, iRXFIFOPE);
    PN_TRACE(tf, iRXFIFOFE);
    PN_TRACE(tf, iRXFIFOBI);

    PN_TRACE(tf, iSOUT);
    PN_TRACE(tf, iTXStart);
    PN_TRACE(tf, iTXClear);
    PN_TRACE(tf, iTXFinished);
    PN_TRACE(tf, iTXRunning);

    PN_TRACE(tf, iSINr);
    PN_TRACE(tf, iSIN);
    PN_TRACE(tf, iRXFinished);
    PN_TRACE(tf, iRXClear);
    PN_TRACE(tf, iRXData);
    PN_TRACE(tf, iRXPE);
    PN_TRACE(tf, iRXFE);
    PN_TRACE(tf, iRXBI);

    PN_TRACE(tf, iFERE);
    PN_TRACE(tf, iPERE);
    PN_TRACE(tf, iBIRE);
    PN_TRACE(tf, iFECounter);
    PN_TRACE(tf, iFEIncrement);
    PN_TRACE(tf, iFEDecrement);
    PN_TRACE(tf, iRDAInterrupt);
    PN_TRACE(tf, iTimeoutCount);
    PN_TRACE(tf, iCharTimeout);
    PN_TRACE(tf, iLSR_THRERE);
    PN_TRACE(tf, iTHRInterrupt);
    PN_TRACE(tf, iTXEnable);
    PN_TRACE(tf, iRTS);

    PN_TRACE(tf, WB_ACK_R);

    PN_TRACE(tf, rx_state);
    PN_TRACE(tf, tx_state);


    // Submodule Traces
    // Input Sync
    uart_is_sin.Trace(tf, level);
    uart_is_cts.Trace(tf, level);
    uart_is_dsr.Trace(tf, level);
    uart_is_dcd.Trace(tf, level);
    uart_is_ri.Trace(tf, level);

    // Input Filter
    uart_if_cts.Trace(tf, level);
    uart_if_dsr.Trace(tf, level);
    uart_if_dcd.Trace(tf, level);
    uart_if_ri.Trace(tf, level);

    // Edge Detect
    uart_iic_thre_ed.Trace(tf, level);
    uart_pedet.Trace(tf, level);
    uart_fedet.Trace(tf, level);
    uart_bidet.Trace(tf,level);
    uart_ed_cts.Trace(tf,level);
    uart_ed_dcd.Trace(tf,level);
    uart_ed_dsr.Trace(tf,level);
    uart_ed_ri.Trace(tf,level);
    uart_rclk.Trace(tf, level);

    // Clock Divider
    uart_bg2.Trace(tf, level);

    // FIFOs
    uart_rxff.Trace(tf,level);
    uart_txff.Trace(tf,level);

    // Interrupt
    uart_iic.Trace(tf,level);

    // Baudgen
    uart_bg16.Trace(tf, level);

    // Receiver/Transmitter
    uart_rx.Trace(tf,level);
    uart_tx.Trace(tf,level);

}
#endif

void Wb8Uart16750::WriteMethod(){

    sc_uint<8> iLSR_var = iLSR.read();
    sc_uint<8> iMSR_var = iMSR.read();

    iLSR_4_0 = iLSR_var(4,0);
    iMSR_3_0 = iMSR_var(3,0);
}

void Wb8Uart16750::TransitionMethod(){

    sc_uint<3> iA_var = iA.read();
    sc_uint<3> wb_adr_var = (wb_adr_i.read()/4);
    sc_uint<8> iIER_var = iIER.read();
    sc_uint<8> iIIR_var = iIIR.read();
    sc_uint<8> iLCR_var = iLCR.read();
    sc_uint<8> iMCR_var = iMCR.read();
    sc_uint<11> iRXFIFOD_var = iRXFIFOD.read();
    sc_uint<11> iRXFIFOQ_var = iRXFIFOQ.read();
    sc_uint<8> iFCR_var = iFCR.read();
    sc_uint<8> iLSR_var = iLSR.read();
    sc_uint<8> iMSR_var = iMSR.read();
    sc_uint<6> iTXFIFOUsage_var = iTXFIFOUsage;
    sc_uint<6> iRXFIFOUsage_var = iRXFIFOUsage;
    sc_uint<32> wb_din_i_var = wb_din_i.read();
    sc_uint<16> iBaudgenDiv_var = iBaudgenDiv.read();

    bool isAddressed = (CFG_UART_BASE_ADDRESS & wb_adr_i.read()) && (wb_adr_i.read() <= CFG_UART_BASE_ADDRESS + 28);


    if ( wb_cyc_i.read() == 1 && wb_stb_i.read() == 1 && wb_we_i.read() == 1 && WB_ACK_R.read() == 1 && isAddressed){

        iWrite = 1;

    }else{

        iWrite = 0;
    }

    if ( wb_cyc_i.read() == 1 && wb_stb_i.read() == 1 && wb_we_i.read() == 0 && WB_ACK_R.read() == 1 && isAddressed){

        iRead = 1;

    }else{

        iRead = 0;
    }

    if(iRead.read() == 1 && iA.read() == 0 && iLCR_DLAB.read() == 0){

        iRBRRead = 1;

    }else{

        iRBRRead = 0;
    }

    if(iWrite.read() == 1 && iA.read() == 0 && iLCR_DLAB.read() == 0){

        iTHRWrite = 1;

    }else{

        iTHRWrite = 0;
    }

    if(iWrite.read() == 1 && iA.read() == 0 && iLCR_DLAB.read() == 1){

        iDLLWrite = 1;

    }else{

        iDLLWrite = 0;
    }

    if(iWrite.read() == 1 && iA.read() == 1 && iLCR_DLAB.read() == 1){

        iDLMWrite = 1;

    }else{

        iDLMWrite = 0;
    }

    if(iWrite.read() == 1 && iA.read() == 1 && iLCR_DLAB.read() == 0){

        iIERWrite = 1;

    }else{

        iIERWrite = 0;
    }

    if(iRead.read() == 1 && iA.read() == 2){

        iIIRRead = 1;

    }else{

        iIIRRead = 0;
    }

    if(iWrite.read() == 1 && iA.read() == 2){

        iFCRWrite = 1;

    }else{

        iFCRWrite = 0;
    }

    if(iWrite.read() == 1 && iA.read() == 3){

        iLCRWrite = 1;

    }else{

        iLCRWrite = 0;
    }

    if(iWrite.read() == 1 && iA.read() == 4){

        iMCRWrite = 1;

    }else{

        iMCRWrite = 0;
    }

    if(iRead.read() == 1 && iA.read() == 5){

        iLSRRead = 1;

    }else{

        iLSRRead = 0;
    }

    if(iRead.read() == 1 && iA.read() == 6){

        iMSRRead = 1;

    }else{

        iMSRRead = 0;
    }

    if(iWrite.read() == 1 && iA.read() == 7){

        iSCRWrite = 1;

    }else{

        iSCRWrite = 0;
    }

    iA = wb_adr_var(2,0);
    iDIN = wb_din_i_var(7,0);
    wb_ack_o = WB_ACK_R.read();

    iIER_ERBI = iIER_var[0];
    iIER_ETBEI = iIER_var[1];
    iIER_ELSI = iIER_var[2];
    iIER_EDSSI = iIER_var[3];


    if((iFCR_FIFOEnable.read() == 0 && iLSR_DR.read() == 1) ||
       (iFCR_FIFOEnable.read() == 1 && iRXFIFOTrigger.read() == 1)){

        iRDAInterrupt = 1;

    }else{

        iRDAInterrupt = 0;
    }

    iIIR_PI = iIIR_var[0];
    iIIR_ID0 = iIIR_var[1];
    iIIR_ID1 = iIIR_var[2];
    iIIR_ID2 = iIIR_var[3];
    iIIR_FIFO64 = iIIR_var[5];

    iIIR_var[4] = 0;

    if(iFCR_FIFOEnable.read() == 1){

        iIIR_var[5] = iFCR_FIFO64E.read();

    }else{

        iIIR_var[5] = 0;
    }

    iIIR_var[6] = iFCR_FIFOEnable.read();
    iIIR_var[7] = iFCR_FIFOEnable.read();

    iFCR_var[0] = iFCR_FIFOEnable.read();
    iFCR_var[1] = iFCR_RXFIFOReset.read();
    iFCR_var[2] = iFCR_TXFIFOReset.read();
    iFCR_var[3] = iFCR_DMAMode.read();
    iFCR_var[4] = 0;
    iFCR_var[5] = iFCR_FIFO64E.read();
    iFCR_var(7,6) = iFCR_RXTrigger.read();


    iLCR_WLS = iLCR_var(1,0);
    iLCR_STB = iLCR_var[2];
    iLCR_PEN = iLCR_var[3];
    iLCR_EPS = iLCR_var[4];
    iLCR_SP = iLCR_var[5];
    iLCR_BC = iLCR_var[6];
    iLCR_DLAB = iLCR_var[7];

    iMCR_DTR = iMCR_var[0];
    iMCR_RTS = iMCR_var[1];
    iMCR_OUT1 = iMCR_var[2];
    iMCR_OUT2 = iMCR_var[3];
    iMCR_LOOP = iMCR_var[4];
    iMCR_AFE = iMCR_var[5];
    iMCR_var(7,6) = 0;

    iRXFIFOPE = (iRXFIFOEmpty.read() == 0 && iRXFIFOQ_var[8] == 1) ? 1 : 0;
    iRXFIFOFE = (iRXFIFOEmpty.read() == 0 && iRXFIFOQ_var[9] == 1) ? 1 : 0;
    iRXFIFOBI = (iRXFIFOEmpty.read() == 0 && iRXFIFOQ_var[10] == 1) ? 1 : 0;

    iFEIncrement = (iRXFIFOWrite.read() == 1 && iRXFIFOD_var(10,8) != 0) ? 1 : 0;
    iFEDecrement = (iFECounter.read() != 0 && iRXFIFOEmpty.read() == 0 && (iPERE.read() == 1 || iFERE.read() == 1 || iBIRE.read() == 1)) ? 1 : 0;

    iLSR_var[0] = iLSR_DR;
    iLSR_var[1] = iLSR_OE;
    iLSR_var[2] = iLSR_PE;
    iLSR_var[3] = iLSR_FE;
    iLSR_var[4] = iLSR_BI;
    iLSR_var[5] = iLSR_THRE;
    iLSR_var[6] = iLSR_TEMT;
    iLSR_var[7] = ((iFCR_FIFOEnable.read() == 1 && iLSR_FIFOERR.read() == 1) ? 1 : 0);

    iLSR_DR = (iRXFIFOEmpty.read() == 0 || iRXFIFOWrite.read() == 1) ? 1 : 0;
    iLSR_THRE = (iTXFIFOEmpty.read() == 1) ? 1 : 0;
    iLSR_TEMT = (iTXRunning.read() == 0 && iLSR_THRE.read() == 1) ? 1 : 0;

    iMSR_CTS = ((iMCR_LOOP.read() == 1 && iRTS.read() == 1) || (iMCR_LOOP.read() == 0 && iCTSn.read() == 0)) ? 1 : 0;
    iMSR_DSR = ((iMCR_LOOP.read() == 1 && iMCR_DTR.read() == 1) || (iMCR_LOOP.read() == 0 && iDSRn.read() == 0)) ? 1 : 0;
    iMSR_RI = ((iMCR_LOOP.read() == 1 && iMCR_OUT1.read() == 1) || (iMCR_LOOP.read() == 0 && iRIn.read() == 0)) ? 1 : 0;
    iMSR_DCD = ((iMCR_LOOP.read() == 1 && iMCR_OUT2.read() == 1) || (iMCR_LOOP.read() == 0 && iDCDn.read() == 0)) ? 1 : 0;

    iMSR_var[0] = iMSR_dCTS;
    iMSR_var[1] = iMSR_dDSR;
    iMSR_var[2] = iMSR_TERI;
    iMSR_var[3] = iMSR_dDCD;
    iMSR_var[4] = iMSR_CTS;
    iMSR_var[5] = iMSR_DSR;
    iMSR_var[6] = iMSR_RI;
    iMSR_var[7] = iMSR_DCD;

    // Transmitter FIFO inputs
    iTXFIFO16Full = iTXFIFOUsage_var[4];
    iTXFIFOFull = (iFCR_FIFO64E.read() == 0) ? iTXFIFO16Full.read() : iTXFIFO64Full.read();
    if(((iFCR_FIFOEnable.read() == 0 && iTXFIFOEmpty.read() == 1) ||
       (iFCR_FIFOEnable.read() == 1 && iTXFIFOFull.read() == 0)) && iTHRWrite.read() == 1){

        iTXFIFOWrite = 1;

    }else{

        iTXFIFOWrite = 0;
    }

    iTXFIFOClear = (iFCR_TXFIFOReset.read() == 1) ? 1 : 0;

    // Reciever FIFO inputs
    iRXFIFORead = (iRBRRead.read() == 1) ? 1 : 0;
    iRXFIFO16Full = iRXFIFOUsage_var[4];
    iRXFIFOFull = (iFCR_FIFO64E.read() == 0) ? iRXFIFO16Full.read() : iRXFIFO64Full.read();

    // Receiver IFIFO outputs
    iRBR = iRXFIFOQ_var(7,0);

    // FIFO trigger level: 1, 4, 8, 14
    if((iFCR_RXTrigger.read() == 0 && iRXFIFOEmpty.read() == 0) ||
       (iFCR_RXTrigger.read() == 1 && (iRXFIFOUsage_var[2] == 1 || iRXFIFOUsage_var[3] == 1)) ||
       (iFCR_RXTrigger.read() == 2 && iRXFIFOUsage_var[3] == 1 ) ||
       (iFCR_RXTrigger.read() == 3 && iRXFIFOUsage_var[3] == 1 && iRXFIFOUsage_var[2] == 1 && iRXFIFOUsage_var[1] == 1) ||
       iRXFIFO16Full.read() == 1){

        iRXFIFO16Trigger = 1;

    }else{

        iRXFIFO16Trigger = 0;
    }

    // FIFO trigger level: 1, 16, 32, 56
    if((iFCR_RXTrigger.read() == 0 && iRXFIFOEmpty.read() == 0) ||
       (iFCR_RXTrigger.read() == 1 && (iRXFIFOUsage_var[4] == 1 || iRXFIFOUsage_var[5] == 1)) ||
       (iFCR_RXTrigger.read() == 2 && iRXFIFOUsage_var[5] == 1 ) ||
       (iFCR_RXTrigger.read() == 3 && iRXFIFOUsage_var[5] == 1 && iRXFIFOUsage_var[4] == 1 && iRXFIFOUsage_var[3] == 1) ||
       iRXFIFO64Full.read() == 1){

        iRXFIFO64Trigger = 1;

    }else{

        iRXFIFO64Trigger = 0;
    }

    iRXFIFOTrigger = (iFCR_FIFO64E.read() == 0) ? iRXFIFO16Trigger.read() : iRXFIFO64Trigger.read();

    iRXClear = 0;
    iTXClear = 0;

    iSIN = (iMCR_LOOP.read() == 0) ? iSINr.read() : iSOUT.read();

    // Transmitter enable signal

    if(iTXFIFOEmpty.read() == 0 && (iMCR_AFE.read() == 0 || (iMCR_AFE.read() == 1 && iMSR_CTS.read() == 1))){

        iTXEnable = 1;

    }else{

        iTXEnable = 0;
    }

    //appending iDLM and iDLL to get BaudgenDiv
    iBaudgenDiv_var(15, 8) = iDLM.read();
    iBaudgenDiv_var(7, 0) = iDLL.read();

    // Writebacks
    iBaudgenDiv = iBaudgenDiv_var;
    iIIR_7_4 = iIIR_var(7,4);
    iFCR = iFCR_var;
    iLSR = iLSR_var;
    iMSR = iMSR_var;


}

void Wb8Uart16750::WBAckPrMethod(){


    WB_ACK_R = 1;

    wait();

    while(true){

        if(WB_ACK_R.read() == 1){

            WB_ACK_R = 0;

        }else if (wb_cyc_i.read() == 1 && wb_stb_i.read() == 1){

            WB_ACK_R = 1;

        }

        wait();
    }

}

void Wb8Uart16750::UartDlrMethod(){

    iDLL = 0;
    iDLM = 0;

    wait();

    while(true){

        if(iDLLWrite.read() == 1){

            iDLL = iDIN.read();

        }

        if(iDLMWrite.read() == 1){

            iDLM = iDIN.read();

        }

        wait();
    }

}

void Wb8Uart16750::UartIerMethod(){

    sc_uint <8> iDIN_var;

    iIER_3_0 = 0;

    wait();

    while(true){


        iDIN_var = iDIN.read();

        if(iIERWrite.read() == 1){

            iIER_3_0 = iDIN_var(3,0);
        }

        wait();
    }

}

void Wb8Uart16750::UartIicThreiMethod(){

    sc_uint<8> iDIN_var;
    sc_uint<8> iIIR_var;


    iTHRInterrupt = 0;

    wait();

    while(true){


        iDIN_var = iDIN.read();
        iIIR_var = iIIR.read();

        if(iLSR_THRERE.read() == 1 || iFCR_TXFIFOReset.read() == 1 || ((iIERWrite.read() == 1) && (iDIN_var[1] == 1) && (iLSR_THRE.read() == 1))){

            iTHRInterrupt = 1;

        }else if((iIIRRead.read() == 1 && iIIR_var(3,1) == 1) || iTHRWrite.read() == 1){

            iTHRInterrupt = 0;

        }

        wait();

    }

}

void Wb8Uart16750::UartCtiMethod(){

    sc_uint<8> iTimeoutCount_var;

    iTimeoutCount = 0;
    iCharTimeout = 0;

    wait();

    while(true){

        iTimeoutCount_var = iTimeoutCount.read();

        if(iRXFIFOEmpty.read() == 1 || iRBRRead.read() == 1 || iRXFIFOWrite.read() == 1){

            iTimeoutCount = 0;

        }else if(iRXFIFOEmpty.read() == 0 && iBaudtick2x.read() == 1 && iTimeoutCount_var[5] == 0){

            iTimeoutCount = iTimeoutCount.read() + 1;
        }

        // Timeout indication

        if(iFCR_FIFOEnable.read() == 1){

            if(iRBRRead.read() == 1){

                iCharTimeout = 0;

            }else if (iTimeoutCount_var[5] == 1){

                iCharTimeout = 1;
            }

        }else{

            iCharTimeout = 0;
        }

        wait();
    }


}

void Wb8Uart16750::UartFcrMethod(){

    sc_uint<8> iDIN_var;

    iFCR_FIFOEnable = 0;
    iFCR_RXFIFOReset = 0;
    iFCR_TXFIFOReset = 0;
    iFCR_DMAMode = 0;
    iFCR_FIFO64E = 0;
    iFCR_RXTrigger = 0;

    wait();

    while(true){

        iFCR_RXFIFOReset = 0;
        iFCR_TXFIFOReset = 0;

        iDIN_var = iDIN.read();

        if(iFCRWrite.read() == 1){

            iFCR_FIFOEnable = iDIN_var[0];
            iFCR_DMAMode = iDIN_var[3];
            iFCR_RXTrigger = iDIN_var(7,6);


            if(iLCR_DLAB.read() == 1){

                iFCR_FIFO64E = iDIN_var[5];
            }

            if(iDIN_var[1] == 1 || (iFCR_FIFOEnable.read() == 0 && iDIN_var[0] == 1) ||
            (iFCR_FIFOEnable.read() == 1 && iDIN_var[0] == 0)){

                iFCR_RXFIFOReset = 1;
            }

            if(iDIN_var[2] == 1 || (iFCR_FIFOEnable.read() == 0 && iDIN_var[0] == 1) ||
            (iFCR_FIFOEnable.read() == 1 && iDIN_var[0] == 0)){

                iFCR_TXFIFOReset = 1;
            }
        }

        wait();
    }


}

void Wb8Uart16750::UartLcrMethod(){

    iLCR = 0;

    wait();

    while(true){


        if(iLCRWrite.read() == 1){

            iLCR = iDIN.read();
        }

        wait();
    }


}

void Wb8Uart16750::UartMcrMethod(){

    sc_uint<8> iMCR_var;
    sc_uint<8> iDIN_var;

    iMCR = 0;

    wait();

    while(true){

        iMCR_var = iMCR.read();
        iDIN_var = iDIN.read();

        if(iMCRWrite.read() == 1){

            iMCR_var(5,0) = iDIN_var(5,0);

            iMCR = iMCR_var;

        }

        wait();
    }



}

void Wb8Uart16750::UartLsrMethod(){


    sc_uint<11> iRXFIFOQ_var;

    iLSR_OE = 0;
    iLSR_PE = 0;
    iLSR_FE = 0;
    iLSR_BI = 0;
    iFECounter = 0;

    wait();

    while(true){

        iRXFIFOQ_var = iRXFIFOQ.read();

        // Overrun error
        if((iFCR_FIFOEnable.read() == 0 && iLSR_DR.read() == 1 && iRXFinished.read() == 1) ||
           (iFCR_FIFOEnable.read() == 1 && iRXFIFOFull.read() == 1 && iRXFinished.read() == 1)){

            iLSR_OE = 1;

        }else if (iLSRRead.read() == 1){

            iLSR_OE = 0;
        }

        // Parity error
        if(iPERE.read() == 1){

            iLSR_PE = 1;

        }else if(iLSRRead.read() == 1){

            iLSR_PE = 0;
        }

        // Frame error
        if(iFERE.read() == 1){

            iLSR_FE = 1;

        }else if(iLSRRead.read() == 1){

            iLSR_FE = 0;
        }

        // Break interrupt
        if(iBIRE.read() == 1){

            iLSR_BI = 1;

        }else if (iLSRRead.read() == 1){

            iLSR_BI = 0;
        }

        // FIFO error
        // Datasheet: Cleared by LSR read when no subsequent errors in FIFO
        // Observed:  Cleared when no subsequent errors in FIFO
        if(iFECounter.read() != 0){

            iLSR_FIFOERR = 1;

        }else if(iRXFIFOEmpty.read() == 1 || iRXFIFOQ_var(10,8) == 0){

            iLSR_FIFOERR = 0;
        }

        // FIFO error counter
        if(iRXFIFOClear.read() == 1){

            iFECounter = 0;

        }else{

            if(iFEIncrement.read() == 1 && iFEDecrement.read() == 0){

                iFECounter = iFECounter.read() + 1;

            }else if (iFEIncrement.read() == 0 && iFEDecrement.read() == 1){

                iFECounter = iFECounter.read() - 1;
            }

        }

        wait();
    }


}

void Wb8Uart16750::UartMsrMethod(){


    iMSR_dCTS = 0;
    iMSR_dDSR = 0;
    iMSR_TERI = 0;
    iMSR_dDCD = 0;

    wait();

    while(true){

        // Delta CTS
        if(iCTSnRE.read() == 1 || iCTSnFE.read() == 1){

            iMSR_dCTS = 1;

        }else if(iMSRRead.read() == 1){

            iMSR_dCTS = 0;
        }

        // Delta DSR
        if(iDSRnRE.read() == 1 || iDSRnFE.read() == 1){

            iMSR_dDSR = 1;

        }else if(iMSRRead.read() == 1){

            iMSR_dDSR = 0;
        }

        // Trailing edge RI
        if(iRInFE.read() == 1){

            iMSR_TERI = 1;

        }else if(iMSRRead.read() == 1){

            iMSR_TERI = 0;
        }

        // Dealta DCD
        if(iDCDnRE.read() == 1 || iDCDnFE.read() == 1){

            iMSR_dDCD = 1;

        }else if(iMSRRead.read() == 1){

            iMSR_dDCD = 0;
        }

        wait();
    }


}

void Wb8Uart16750::UartScrMethod(){

    iSCR = 0;

    wait();

    while(true){

        if(iSCRWrite.read() == 1){

            iSCR = iDIN.read();
        }

        wait();
    }


}

void Wb8Uart16750::UartTxMethod(){

    iTSR = 0;
    tx_state = TXIDLE;

    wait();

    while(true){

        tx_state = next_tx_state;
        iTSR  = iTSR_next.read();

        wait();
    }


}

void Wb8Uart16750::UartTxStates(){


    if(rst_i.read() == 1){

        iTSR_next = 0;
        iTXStart = 0;
        iTXFIFORead = 0;
        iTXRunning = 0;
        next_tx_state = TXIDLE;

    } else {

        iTXStart = 0;
        iTXFIFORead = 0;
        iTXRunning = 0;
        iTSR_next = iTSR.read();

        switch(tx_state.read()){

            case TXIDLE:

                if(iTXEnable.read() == 1){

                    iTXStart = 1;
                    next_tx_state = TXSTART;

                }else{

                    next_tx_state = TXIDLE;
                }
                iTSR_next = iTSR.read();



                break;

            case TXSTART:

                iTSR_next = iTXFIFOQ.read();
                iTXStart = 1;
                iTXFIFORead = 1;
                next_tx_state = TXRUN;
                break;

            case TXRUN:

                if(iTXFinished.read() == 1){

                    next_tx_state = TXEND;

                }else{

                    next_tx_state = TXRUN;
                }

                iTSR_next = iTSR.read();

                iTXRunning = 1;
                iTXStart = 1;

                break;

            case TXEND:

                iTSR_next = iTSR.read();

                next_tx_state = TXIDLE;
                break;

            default:

                iTSR_next = iTSR.read();

                next_tx_state = TXIDLE;
                break;
        }
    }
}

void Wb8Uart16750::UartRxMethod(){

    rx_state = RXIDLE;
    iRXFIFOD = 0;

    wait();

    while(true){

        rx_state = next_rx_state;
        iRXFIFOD = next_iRXFIFOD;
        wait();
    }


}
void Wb8Uart16750::UartRxStates(){

    sc_uint<11> iRXFIFOD_var;

    if(rst_i == 1){


        iRXFIFOWrite = 0;
        iRXFIFOClear = 0;
        next_iRXFIFOD = 0;
        next_rx_state = RXIDLE;


    } else {


        iRXFIFOWrite = 0;
        iRXFIFOClear = iFCR_RXFIFOReset;
        next_iRXFIFOD = iRXFIFOD.read();

        switch(rx_state.read()){

            case RXIDLE:

                if(iRXFinished.read() == 1){

                    iRXFIFOD_var[10] = iRXBI.read();
                    iRXFIFOD_var[9] = iRXFE.read();
                    iRXFIFOD_var[8] = iRXPE.read();
                    iRXFIFOD_var(7,0) = iRXData.read();
                    next_iRXFIFOD = iRXFIFOD_var;

                    if(iFCR_FIFOEnable.read() == 0){

                        iRXFIFOClear = 1;

                    }

                    next_rx_state = RXSAVE;

                }else{

                    next_iRXFIFOD = iRXFIFOD.read();
                    next_rx_state = RXIDLE;

                }

                break;

            case RXSAVE:

                if(iFCR_FIFOEnable.read() == 0){

                    iRXFIFOWrite = 1;

                }else if(iRXFIFOFull.read() == 0){

                    iRXFIFOWrite = 1;
                }

                next_iRXFIFOD = iRXFIFOD.read();


                next_rx_state = RXIDLE;

                break;

            default:

                next_iRXFIFOD = iRXFIFOD.read();


                next_rx_state = RXIDLE;

                break;
        }
    }
}

void Wb8Uart16750::UartAfcMethod(){

    iRTS = 0;

    wait();

    while(true){

        if(iMCR_RTS.read() == 0 || (iMCR_AFE.read() == 1 && iRXFIFOTrigger.read() == 1)){

            iRTS = 0;

        }else if (iMCR_RTS.read() == 1 && (iMCR_AFE.read() == 0 || (iMCR_AFE.read() == 1 && iRXFIFOEmpty.read() == 1))){

            iRTS = 1;
        }

        wait();
    }




}

void Wb8Uart16750::UartOutRegsMethod(){


    baudoutn_o = 0;
    out1_o = 0;
    out2_o = 0;
    rtsn_o = 0;
    dtrn_o = 0;
    sout_o = 0;

    wait();

    while(true){

        baudoutn_o = 0;
        out1_o = 0;
        out2_o = 0;
        rtsn_o = 0;
        dtrn_o = 0;
        sout_o = 0;

        if(iBaudtick16x.read() == 0){

            baudoutn_o = 1;
        }

        if(iMCR_LOOP.read() == 1 || iMCR_OUT1.read() == 0){

            out1_o = 1;
        }

        if(iMCR_LOOP.read() == 1 || iMCR_OUT2.read() == 0){

            out1_o = 1;
        }

        if(iMCR_LOOP.read() == 1 || iRTS.read() == 0){

            rtsn_o = 1;
        }

        if(iMCR_LOOP.read() == 1 || iMCR_DTR.read() == 0){

            dtrn_o = 1;
        }

        if(iMCR_LOOP.read() == 1 || iSOUT.read() == 1){

            sout_o = 1;
        }

        wait();
    }


}

void Wb8Uart16750::UartDoutMethod(){

    sc_uint<32> wb_dout_var = 0;

    if (WB_ACK_R.read() == 1){

        switch(iA.read()){

            case 0:

                if(iLCR_DLAB.read() == 0){

                    wb_dout_var(7,0) = iRBR.read();

                }else{

                    wb_dout_var(7,0) = iDLL.read();
                }

                break;

            case 1:

                if(iLCR_DLAB.read() == 0){

                    wb_dout_var(7,0) = iIER.read();

                }else{

                    wb_dout_var(7,0) = iDLM.read();
                }

                break;

            case 2:

                wb_dout_var(7,0) = iIIR.read();
                break;

            case 3:

                wb_dout_var(7,0) = iLCR.read();
                break;

            case 4:

                wb_dout_var(7,0) = iMCR.read();
                break;

            case 5:

                wb_dout_var(7,0) = iLSR.read();
                break;

            case 6:

                wb_dout_var(7,0) = iMSR.read();
                break;

            case 7:

                wb_dout_var(7,0) = iSCR.read();
                break;

            default:

                wb_dout_var(7,0) = iRBR.read();
                break;

        }
    }

    wb_dout_o = wb_dout_var;

}

void Wb8Uart16750::IERCombinationMethod(){

    sc_uint<8> iIER_var = iIER.read();

    iIER_var(3,0) = iIER_3_0.read();
    iIER_var(7,4) = 0;

    iIER = iIER_var;

}

void Wb8Uart16750::IIRCombinationMethod(){

    sc_uint<8> iIIR_var = iIIR.read();

    iIIR_var(3,0) = iIIR_3_0.read();
    iIIR_var(7,4) = iIIR_7_4.read();

    iIIR = iIIR_var;
}