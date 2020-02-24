/*************************************************************************

  This file is part of the ParaNut project.

  Copyright (C) 2019 Alexander Bahle <alexander.bahle@hs-augsburg.de>
      Hochschule Augsburg, University of Applied Sciences

  Description:
    This is a SystemC model of the the hardware multiplier/divider
    embedded in the execution unit (EXU) of the ParaNut.
    It is only necessary if the RISC-V M extension is enabled (config.h).

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

#ifndef MEXTENSION_H
#define MEXTENSION_H

#ifndef __SYNTHESIS__
// Vivado HLS 2017.2 throws weird errors if the same header is included in more than one file
#include "base.h"
#endif
#include <systemc.h>

// **************** MExtension Function Codes *************
typedef enum {
    // MExtension
    afMul,
    afDiv = 0,
    afMulh,
    afDivu = 1,
    afMulhsu,
    afRem = 2,
    afMulhu,
    afRemu = 3,
} EMExtFunc;


// **************** MMExtension ************************

SC_MODULE (MMExtension) {
public:
    // Ports ...
    sc_in<bool> clk, reset;

    sc_in<bool> m_enable, d_enable;
    sc_in<sc_uint<2> > md_func;
    sc_in<sc_uint<32> > op_a, op_b;

    sc_out<bool> m_valid, d_valid;
    sc_out<sc_uint<32> > m_result, d_result;

    // Constructor ...
    SC_HAS_PROCESS (MMExtension);
    MMExtension (sc_module_name name) : sc_module (name) {
        SC_METHOD (MulMethod);
            sensitive << op_a << op_b << m_enable << md_func;
        SC_METHOD (MulAddMethod);
            sensitive << a1b1 << a1b2 << a2b1 << a2b2 << md_func;
        SC_METHOD (DivTransitionMethod);
            sensitive << clk.pos ();
        SC_METHOD (DivCombMethod);
            sensitive << op_a << op_b << d_enable << md_func << remd_reg << quot_reg << state;
    }

    // Functions ...
    void Trace (sc_trace_file * tf, int levels = 1);

    // Processes ...
    void MulMethod ();
    void MulAddMethod ();
    void DivCombMethod ();
    void DivTransitionMethod ();

protected:
    // MUL signals ...
    sc_signal<sc_uint<32> > a1b1, a1b2, a2b1, a2b2;
    sc_signal<bool> sign;
    sc_signal<sc_uint<32> > m_result_next;

    // DIV/REM signals & registers ...
    sc_signal<sc_uint<32> > remd, quot, remd_reg, quot_reg;
    sc_signal<sc_uint<33> > state;
};

#endif // MEXTENSION_H
