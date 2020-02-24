/*************************************************************************

  This file was copied and modified from the Spike ISA Simulator project:
    https://github.com/riscv/riscv-isa-sim

Copyright (c) 2010-2017, The Regents of the University of California
(Regents).  All Rights Reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
1. Redistributions of source code must retain the above copyright
   notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright
   notice, this list of conditions and the following disclaimer in the
   documentation and/or other materials provided with the distribution.
3. Neither the name of the Regents nor the
   names of its contributors may be used to endorse or promote products
   derived from this software without specific prior written permission.

IN NO EVENT SHALL REGENTS BE LIABLE TO ANY PARTY FOR DIRECT, INDIRECT,
SPECIAL, INCIDENTAL, OR CONSEQUENTIAL DAMAGES, INCLUDING LOST PROFITS, ARISING
OUT OF THE USE OF THIS SOFTWARE AND ITS DOCUMENTATION, EVEN IF REGENTS HAS
BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

REGENTS SPECIFICALLY DISCLAIMS ANY WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
PURPOSE. THE SOFTWARE AND ACCOMPANYING DOCUMENTATION, IF ANY, PROVIDED
HEREUNDER IS PROVIDED "AS IS". REGENTS HAS NO OBLIGATION TO PROVIDE
MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.

 *************************************************************************/

#ifndef JTAG_DTM_H
#define JTAG_DTM_H

#include <stdint.h>
#include <systemc.h>

#include "base.h"

#define DTM_ADDR_WIDTH 6
#define DTM_IR_WIDTH 5

typedef enum {
    TEST_LOGIC_RESET,
    RUN_TEST_IDLE,
    SELECT_DR_SCAN,
    CAPTURE_DR,
    SHIFT_DR,
    EXIT1_DR,
    PAUSE_DR,
    EXIT2_DR,
    UPDATE_DR,
    SELECT_IR_SCAN,
    CAPTURE_IR,
    SHIFT_IR,
    EXIT1_IR,
    PAUSE_IR,
    EXIT2_IR,
    UPDATE_IR
} jtag_state_t;

typedef enum {
    IDCODE = 0x1,
    DTMCS = 0x10,
    DMI = 0x11,
} jtag_regs_t;

// Only include this if we are not synthesizing AND simulating
#if !defined(__SYNTHESIS__) && defined(SIMBUILD)
class MDtm;

class jtag_dtm_t {
    static const unsigned idcode = 0xdeadbeef;

    public:
    jtag_dtm_t (MDtm *dtm, unsigned required_rti_cycles);
    void reset ();

    void set_pins (bool tck, bool tms, bool tdi);

    bool tdo () const { return _tdo; }

    jtag_state_t state () const { return _state; }

    private:
    MDtm *dtm;
    // The number of Run-Test/Idle cycles required before a DMI access is
    // complete.
    unsigned required_rti_cycles;
    bool _tck, _tms, _tdi, _tdo;
    uint32_t ir;
    const unsigned ir_length = 5;
    uint64_t dr;
    unsigned dr_length;

    // abits must come before dtmcontrol so it can easily be used in the
    // constructor.
    const unsigned abits = 6;
    uint32_t dtmcontrol;
    uint64_t dmi;
    // Number of Run-Test/Idle cycles needed before we call this access
    // complete.
    unsigned rti_remaining;
    bool busy_stuck;

    jtag_state_t _state;

    void capture_dr ();
    void update_dr ();
};
#endif

// **************** MDtm *************
SC_MODULE (MDtm) {
public:
    // Ports JTAG (Don't do anything during simulation, just for routing the toplevel JTAG signals)
    sc_in<bool> tck;
    sc_in<bool> tms;
    sc_in<bool> tdi;
    sc_out<bool> tdo;

    // Ports DMI Master
    //  sc_in_clk           clk;
    sc_in<bool> reset;

    sc_out<sc_uint<DTM_ADDR_WIDTH> > dmi_adr; // address output
    sc_out<sc_uint<32> > dmi_dat_o; // output data
    sc_in<sc_uint<32> > dmi_dat_i; // input data
    sc_out<bool> dmi_rd, dmi_wr;

    // Constructor...
    SC_CTOR (MDtm) {
        SC_METHOD (JTAGMethod);
            sensitive << tck.pos ();
        SC_METHOD (OutputMethod);
            sensitive << dr << dmi_op << ir << state;
    }

    // Functions...
    void Trace (sc_trace_file * tf, int levels = 1);
#ifndef __SYNTESIS__
    bool dmi_write (TWord adr, TWord val);
    bool dmi_read (TWord adr, uint64_t * val);
#endif

    // Processes...
    //  void DMIMethod ();
    void OutputMethod ();
    void JTAGMethod ();

protected:
    // Simulation members...
#ifndef __SYNTHESIS__
    sc_uint<DTM_ADDR_WIDTH> adr;
    sc_uint<32> val;
    uint64_t *val_out;
    bool write;
    bool read, dmi_rd_last[3];
#endif

    // Synthesis registers ...
    sc_signal<sc_uint<34 + DTM_ADDR_WIDTH> > dmi, dr;

    sc_signal<sc_uint<4> > state;
    sc_signal<sc_uint<DTM_IR_WIDTH> > ir;
    sc_signal<bool> dmi_length;

    sc_signal<bool> dmi_op;
};


#endif
