/*************************************************************************

  This file is part of the ParaNut project.

  Copyright (C) 2020 Alexander Bahle <alexander.bahle@hs-augsburg.de>
      Hochschule Augsburg, University of Applied Sciences

  Description:
    This is a SystemC model of a Wishbone interconnect exclusively for
    simulation purpose.
    Features 1 master input and CFG_NUT_SIM_MAX_PERIPHERY slave outputs (see config)

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


#include "interconnect.h"

#ifndef __SYNTHESIS__
void MInterconnect::Trace (sc_trace_file *tf, int level) {
    if (!tf || pn_trace_verbose) printf ("\nSignals of Module \"%s\":\n", name ());

    // Ports ...
    PN_TRACE (tf, clk_i);
    PN_TRACE (tf, rst_i);
    // ... Wishbone slave
    PN_TRACE (tf, stb_i);
    PN_TRACE (tf, cyc_i);
    PN_TRACE (tf, we_i);
    PN_TRACE (tf, sel_i);
    PN_TRACE (tf, cti_i);
    PN_TRACE (tf, bte_i);
    PN_TRACE (tf, ack_o);
    PN_TRACE (tf, rty_o);
    PN_TRACE (tf, err_o);
    PN_TRACE (tf, adr_i);
    PN_TRACE (tf, dat_i);
    PN_TRACE (tf, dat_o);

    // Registers...

    // Internal signals...
    PN_TRACE_BUS (tf, stb, CFG_NUT_SIM_MAX_PERIPHERY);
    PN_TRACE_BUS (tf, dat, CFG_NUT_SIM_MAX_PERIPHERY);
    PN_TRACE_BUS (tf, ack, CFG_NUT_SIM_MAX_PERIPHERY);
    PN_TRACE_BUS (tf, rty, CFG_NUT_SIM_MAX_PERIPHERY);
    PN_TRACE_BUS (tf, err, CFG_NUT_SIM_MAX_PERIPHERY);

}
#endif


void MInterconnect::AddSlave(TWord adr, size_t size, MPeripheral *slave)
{
    SInterPeriph *per;

    if (num_peri_ > CFG_NUT_SIM_MAX_PERIPHERY) {
        PN_WARNING("No more space to add peripheral: (Check config to increase number of possible peripherals)");
        PN_WARNINGF(("ADR: 0x%08x, SIZE: %d (0x%x), PTR: %p", adr, size, size, slave));
        return;
    }
    PN_INFO("Add peripheral to Interconnect:");
    PN_INFOF(("ADR: 0x%08x, SIZE: %d (0x%x), PTR: %p", adr, size, size, slave));

    // Check for conflicting address space
    for (int n = 0; n < num_peri_; n++) {
        per = &peripherals_[n];
        if (adr >= per->adr && adr < per->adr+per->size ||                  // Start address inside another range
                adr+size > per->adr && adr+size < per->adr+per->size ||     // End address inside another range
                adr <= per->adr && adr+size >= per->adr+per->size ) {       // New range includes another range
            PN_ERRORF(("Conflicts with MODULE: \"%s\", ADR: 0x%08x, SIZE: %d (0x%x), PTR: %p",
                    per->p->name (), per->adr, per->size, per->size, per->p));
        }
    }

    // Add to peripherals array
    per = &peripherals_[num_peri_];
    per->adr = adr;
    per->size = size;
    per->p = slave;

    // Connect all Wishbone slave inputs
    slave->wb_clk_i (clk_i);
    slave->wb_rst_i (rst_i);
    slave->wb_stb_i (stb[num_peri_]); // Strobe for this peripheral
    slave->wb_cyc_i (cyc_i);
    slave->wb_cti_i (cti_i);
    slave->wb_bte_i (bte_i);
    slave->wb_we_i (we_i);
    slave->wb_sel_i (sel_i);
    slave->wb_adr_i (adr_i);
    slave->wb_dat_i (dat_i);

    // Connect the slave outputs of this peripherals
    slave->wb_ack_o (ack[num_peri_]);
    slave->wb_rty_o (rty[num_peri_]);
    slave->wb_err_o (err[num_peri_]);
    slave->wb_dat_o (dat[num_peri_]);

    num_peri_++;
}


// **************** MInterconnect ******************

void MInterconnect::InterconnectMethod()
{
    SInterPeriph *per;
    sc_uint<32> adr_i_var;
    int n;

    // Defaults ...
    dat_o = 0;
    err_o = 0;
    rty_o = 0;
    ack_o = 0;

    for (int n = 0; n < num_peri_; ++n)
        stb[n] = 0;

    adr_i_var = adr_i.read ();

    // Read or write access
    if (stb_i == 1 && cyc_i == 1) {
        for (n = 0; n < num_peri_; n++) {
            per = &peripherals_[n];
            // Route outputs accordingly when the peripheral is addressed
            if (adr_i_var >= per->adr && adr_i_var < per->adr+per->size) {
                stb[n] = 1;
                ack_o = ack[n];
                err_o = err[n];
                rty_o = rty[n];
                dat_o = dat[n];
                break;
            }
        }
        // No peripheral for the current address
        if (n == num_peri_) {
            PN_ERRORF (("Trying to access a non valid address: 0x%08x", adr_i_var));
        }
    }
}



