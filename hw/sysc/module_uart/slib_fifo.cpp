/*************************************************************************

  This file is part of the ParaNut project.

  Copyright (C) 2022-2023 Elias Schuler <elias.schuler@hs-augsburg.de>
                2022-2023 Lukas Bauer <lukas.bauer@hs-ausgsburg.de>
      Hochschule Augsburg, University of Applied Sciences

    Description : This file is based on the work of Sebastian Witt
                  FIFO (Width 8)

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

#include "slib_fifo.h"



#ifndef __SYNTHESIS__
void SlibFifo::Trace(sc_trace_file *tf, int level){
    if (!tf || pn_trace_verbose) printf ("\nSignals of Module \"%s\":\n", name ());

    //PN_TRACE
    PN_TRACE(tf, clk_i);
    PN_TRACE(tf, rst_i);
    PN_TRACE(tf, clear_i);
    PN_TRACE(tf, write_i);
    PN_TRACE(tf, read_i);
    PN_TRACE(tf, d_i);
    PN_TRACE(tf, q_o);
    PN_TRACE(tf, empty_o);
    PN_TRACE(tf, full_o);
    PN_TRACE(tf, usage_o);
    PN_TRACE(tf, empty);
    PN_TRACE(tf, full);
    PN_TRACE(tf, WRAddr);
    PN_TRACE(tf, RDAddr);
    PN_TRACE(tf, Usage);
    PN_TRACE_BUS(tf, FifoMem, (1 << SLIB_FIFO_SIZE_E));

}
#endif

void SlibFifo::OutputMethod(){

    sc_uint<SLIB_FIFO_SIZE_E + 1> RDAddr_internal = RDAddr.read();
    sc_uint<SLIB_FIFO_SIZE_E + 1> WRAddr_internal = WRAddr.read();


    // if difference between RDAddr and WRAddr is the greatest set full flag
    if(RDAddr_internal(SLIB_FIFO_SIZE_E - 1 , 0) == WRAddr_internal(SLIB_FIFO_SIZE_E - 1, 0) &&
       RDAddr_internal[SLIB_FIFO_SIZE_E]        != WRAddr_internal[SLIB_FIFO_SIZE_E]){

        full = 1;

    }else{

        full = 0;

    }



    empty_o = empty.read();
    full_o = full.read();
    usage_o = Usage.read();

}


void SlibFifo::AdrMethod(){

    WRAddr = 0;
    RDAddr = 0;
    empty = 1;

    wait();

    while(true){

        if((write_i.read() == 1) && (full.read() == 0)){
            WRAddr = WRAddr.read() + 1;
        }

        if((read_i.read() == 1) && (empty.read() == 0)){
            RDAddr = RDAddr.read() + 1;
        }

        if(clear_i.read() == 1){
            WRAddr = 0;
            RDAddr = 0;
        }

        if (RDAddr.read() == WRAddr.read()){

        empty = 1;

        }else{

        empty = 0;

        }

        wait();
    }

}

void SlibFifo::MemMethod(){

    sc_uint<SLIB_FIFO_SIZE_E + 1> RDAddr_internal = 0;
    sc_uint<SLIB_FIFO_SIZE_E + 1> WRAddr_internal = 0;

    wait();

    while(true){

        RDAddr_internal = RDAddr.read();
        WRAddr_internal = WRAddr.read();

        if(write_i.read() == 1 && full.read() == 0){

            FifoMem[WRAddr_internal(SLIB_FIFO_SIZE_E-1, 0)] = d_i.read();

        }

        q_o = FifoMem[RDAddr_internal(SLIB_FIFO_SIZE_E-1, 0)].read();

        wait();
    }

}


void SlibFifo::UsageMethod(){


    Usage = 0;

    wait();

    while(true){

        if(clear_i.read() == 1){

            Usage = 0;

        }else{

            if(read_i.read() == 0 && write_i.read() == 1 && full.read() == 0){

                Usage = Usage.read() + 1;

            }

            if(read_i.read() == 1 && write_i.read() == 0 && empty.read() == 0){

                Usage = Usage.read() - 1;

            }
        }
        wait();
    }
}

