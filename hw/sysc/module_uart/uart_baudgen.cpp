/*************************************************************************

  This file is part of the ParaNut project.

  Copyright (C) 2022-2023 Elias Schuler <elias.schuler@hs-augsburg.de>
                2022-2023 Lukas Bauer <lukas.bauer@hs-ausgsburg.de>
      Hochschule Augsburg, University of Applied Sciences

    Description : This file is based on the work of Sebastian Witt
                  UART Baudgen Generator

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

#include "uart_baudgen.h"



#ifndef __SYNTHESIS__
void UartBaudgen::Trace(sc_trace_file *tf, int level){
    if (!tf || pn_trace_verbose) printf ("\nSignals of Module \"%s\":\n", name ());

    //PN_TRACE
    PN_TRACE(tf, clk_i);
    PN_TRACE(tf, rst_i);
    PN_TRACE(tf, ce_i);
    PN_TRACE(tf, clear_i);
    PN_TRACE(tf, divider_i);
    PN_TRACE(tf, baudtick_o);
    PN_TRACE(tf, counter);
}
#endif

void UartBaudgen::BaudgenMethod(){

    counter = 0;
    baudtick_o = 0;

    wait();

    while(true){

        if(clear_i.read() == 1){

            counter = 0;

        }else if(ce_i.read() == 1){

            counter = counter.read() + 1;

        }

        baudtick_o = 0;

        if(counter.read() == divider_i.read()){

            counter = 0;
            baudtick_o = 1;
        }

        wait();

    }

}

