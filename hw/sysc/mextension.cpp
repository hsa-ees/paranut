/*************************************************************************

  This file is part of the ParaNut project.

  Copyright (C) 2019 Alexander Bahle <alexander.bahle@hs-augsburg.de>
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


#include "mextension.h"


#ifndef __SYNTHESIS__
void MMExtension::Trace (sc_trace_file *tf, int level) {
    if (!tf || pn_trace_verbose) printf ("\nSignals of Module \"%s\":\n", name ());

    // Ports ...
    PN_TRACE (tf, clk);
    PN_TRACE (tf, m_enable);
    PN_TRACE (tf, d_enable);
    PN_TRACE (tf, m_result);
    PN_TRACE (tf, d_result);
    PN_TRACE (tf, m_valid);
    PN_TRACE (tf, d_valid);
    PN_TRACE (tf, md_func);
    PN_TRACE (tf, op_a);
    PN_TRACE (tf, op_b);

    // MUL signals ...
    PN_TRACE (tf, sign);
    PN_TRACE (tf, a1b1);
    PN_TRACE (tf, a1b2);
    PN_TRACE (tf, a2b1);
    PN_TRACE (tf, a2b2);

    // DIV/REM signals ...
    PN_TRACE (tf, remd);
    PN_TRACE (tf, remd_reg);
    PN_TRACE (tf, quot);
    PN_TRACE (tf, quot_reg);
    PN_TRACE (tf, state);
}
#endif // __SYNTHESIS__

void MMExtension::MulMethod () {
    sc_uint<32> op_a_val, op_b_val;
    sc_uint<2> func = 0;
    bool sign_a, sign_b;

    func = md_func.read ();
    op_a_val = op_a.read ();
    op_b_val = op_b.read ();
    // A is only unsigned for MULHU)
    sign_a = (op_a_val[31] & !(func[1] & func[0]));
    // B is unsigned for MULHSU/MULHU
    sign_b = (op_b_val[31] & !func[1]);

    // Calculate 2's complement of A and B
    if (sign_a) op_a_val = sc_uint<32> ((~op_a_val) + 1);

    if (sign_b) op_b_val = sc_uint<32> ((~op_b_val) + 1);

    // Four 16 Bit multiplications
    // A2A1 * B2B1 =
    // -------------
    //         A1B1
    //     A1B2
    //     A2B1
    // A2B2
    // -------------
    // Added in MulAddMethod for HLS reasons (If added in the same Method HLS removes necessary bits)
    a1b1 = sc_uint<32> ((__uint32_t) (op_a_val (15, 0)) * (__uint32_t) (op_b_val (15, 0)));
    a1b2 = sc_uint<32> ((__uint32_t) (op_a_val (15, 0)) * (__uint32_t) (op_b_val (31, 16)));
    a2b1 = sc_uint<32> ((__uint32_t) (op_a_val (31, 16)) * (__uint32_t) (op_b_val (15, 0)));
    a2b2 = sc_uint<32> ((__uint32_t) (op_a_val (31, 16)) * (__uint32_t) (op_b_val (31, 16)));

    // Determine sign of result:
    sign = sign_a ^ sign_b;

    // Finished
    m_valid = m_enable.read ();
}

void MMExtension::MulAddMethod () {
    sc_uint<64> result;
    sc_uint<2> func;

    func = md_func.read ();

    // Add the four multiplication resulsts with appropriate offset
    //   -------------
    //           A1B1
    //       A1B20000
    //       A2B10000
    //   A2B200000000
    // + ------------
    // = (A * B)
    result = sc_uint<64> ((__uint64_t) (a1b1_reg.read ()) + sc_uint<64> ((a1b2_reg.read (), sc_uint<16> (0))) +
                          sc_uint<64> ((a2b1_reg.read (), sc_uint<16> (0))) +
                          sc_uint<64> ((a2b2_reg.read (), sc_uint<32> (0))));

    // Calculate 2's complement of result based on sign determined in MulMethod
    if (sign) result = sc_uint<64> ((~result) + 1);

    // Write back upper or lower 32 bits
    m_result = func == afMul ? result (31, 0) : result (63, 32);
}

void MMExtension::DivTransitionMethod () {
    // Register method
    if (reset) {
        state = 0;
        quot_reg = 0;
        remd_reg = 0;

        // Use this clocked Method for m_result register
        a1b1_reg = 0;
        a1b2_reg = 0;
        a2b1_reg = 0;
        a2b2_reg = 0;
    } else {
        if (d_enable == 1 && state.read () == 0) {
            // Start
            state = (state.read () (31, 0), sc_uint<1> (1));
        } else {
            // Shift
            state = (state.read () (31, 0), sc_uint<1> (0));
        }

        // Quotient and remainder registers
        quot_reg = quot.read ();
        remd_reg = remd.read ();

        // Use this clocked Method for m_result register
        a1b1_reg = a1b1.read ();
        a1b2_reg = a1b2.read ();
        a2b1_reg = a2b1.read ();
        a2b2_reg = a2b2.read ();
    }
}

void MMExtension::DivCombMethod () {
    sc_uint<33> state_val, cal;
    sc_uint<32> remd_mux, quot_mux, comb, dividend, divisor, quot_val, remd_val, op_a_val, op_b_val,
    out_quot, out_remd;
    sc_uint<2> func;

    // Read input signals/ports
    op_a_val = op_a.read ();
    op_b_val = op_b.read ();
    state_val = state.read ();
    func = md_func.read ();
    quot_val = quot_reg.read ();
    remd_val = remd_reg.read ();

    // Calc two's complement for signed input
    dividend = op_a_val[31] & !func[0] ? sc_uint<32> ((~op_a_val) + 1) : op_a_val;
    divisor = op_b_val[31] & !func[0] ? sc_uint<32> ((~op_b_val) + 1) : op_b_val;

    // Muxes for initial calculation (state_val[0] == 1)
    quot_mux = state_val[0] ? dividend : quot_val;
    remd_mux = state_val[0] ? sc_uint<32> (0) : remd_val;

    // Subtract
    comb = (remd_mux (30, 0), quot_mux[31]);
    cal = (sc_uint<1> (0), comb) - (sc_uint<1> (0), divisor);

    // Write output signals
    quot = (quot_mux (30, 0), !cal[32]);
    remd = cal[32] ? comb : cal (31, 0);

    // Write output ports
    d_valid = state_val[31];

    // Output Mux
    if (op_b_val == 0) {
        out_quot = -1;
        out_remd = op_a_val;
    } else if (op_a_val == 0x80000000 && op_b_val == 0xffffffff && !func[0]) {
        // Signed overflow:
        out_quot = op_a_val;
        out_remd = 0;
    } else {
        out_quot = (op_a_val[31] ^ op_b_val[31]) & !func[0] ? sc_uint<32> ((~quot_val) + 1) : quot_val;
        out_remd = op_a_val[31] & !func[0] ? sc_uint<32> ((~remd_val) + 1) : remd_val;
    }

    // REM/U or DIV/U
    if (func[1]) {
        d_result = out_remd;
    } else {
        d_result = out_quot;
    }
}
