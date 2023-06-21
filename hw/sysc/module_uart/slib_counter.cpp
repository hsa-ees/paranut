/*************************************************************************

  This file is part of the ParaNut project.

  Copyright (C) 2022-2023 Elias Schuler <elias.schuler@hs-augsburg.de>
                2022-2023 Lukas Bauer <lukas.bauer@hs-ausgsburg.de>
      Hochschule Augsburg, University of Applied Sciences

    Description : This file is based on the work of Sebastian Witt
                  Counter

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

#include "slib_counter.h"



#ifndef __SYNTHESIS__
void SlibCounter::Trace(sc_trace_file *tf, int level){
    if (!tf || pn_trace_verbose) printf ("\nSignals of Module \"%s\":\n", name ());

    //PN_TRACE
    PN_TRACE(tf, clk_i);
    PN_TRACE(tf, rst_i);
    PN_TRACE(tf, clear_i);
    PN_TRACE(tf, load_i);
    PN_TRACE(tf, enable_i);
    PN_TRACE(tf, down_i);
    PN_TRACE(tf, d_i);
    PN_TRACE(tf, q_o);
    PN_TRACE(tf, overflow_o);

    PN_TRACE(tf, counter);
}
#endif

void SlibCounter::OutputMethod(){
    sc_uint<SLIB_COUNTER_WIDTH+1> counter_internal = counter.read();
    overflow_o = counter_internal[SLIB_COUNTER_WIDTH];
    q_o = counter_internal(SLIB_COUNTER_WIDTH-1, 0);
}


void SlibCounter::CounterMethod(){

    counter = 0;

    wait();

    while (true)
    {

        if(clear_i.read() == 1)
        {
            counter = 0;
        } else if(load_i.read() == 1){
            counter = (sc_uint<1> (0),
                       d_i.read());
        } else if (enable_i.read() == 1){
            if(down_i.read() == 0){
                counter = counter.read() + 1;
            }else{
                counter = counter.read() - 1;
            }
        }
        sc_uint<SLIB_COUNTER_WIDTH+1> counter_internal = counter.read();
        if(counter_internal[SLIB_COUNTER_WIDTH] == 1){
            counter_internal[SLIB_COUNTER_WIDTH] = 0;
            counter = counter_internal;
        }
        wait();
    }


}

