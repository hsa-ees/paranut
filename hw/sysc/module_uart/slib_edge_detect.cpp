/*************************************************************************

  This file is part of the ParaNut project.

  Copyright (C) 2022-2023 Elias Schuler <elias.schuler@hs-augsburg.de>
                2022-2023 Lukas Bauer <lukas.bauer@hs-ausgsburg.de>
      Hochschule Augsburg, University of Applied Sciences

    Description : This file is based on the work of Sebastian Witt
                  Signal edge detect

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

#include "slib_edge_detect.h"



#ifndef __SYNTHESIS__
void SlibEdgeDetect::Trace(sc_trace_file *tf, int level){
    if (!tf || pn_trace_verbose) printf ("\nSignals of Module \"%s\":\n", name ());

    //PN_TRACE
    PN_TRACE(tf, clk_i);
    PN_TRACE(tf, rst_i);
    PN_TRACE(tf, d_i);
    PN_TRACE(tf, re_o);
    PN_TRACE(tf, fe_o);
    PN_TRACE(tf, idd);
}
#endif

void SlibEdgeDetect::OutputMethod(){

    re_o = (idd.read() == 0 && d_i.read() == 1) ? 1 : 0;
    fe_o = (idd.read() == 1 && d_i.read() == 0) ? 1 : 0;
}


void SlibEdgeDetect::EdgeDetectMethod(){

    idd = 0;

    wait();

    while (true)
    {
        idd = d_i.read();
        wait();
    }

}

