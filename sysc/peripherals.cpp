/*************************************************************************

  This file is part of the ParaNut project.

  Copyright (C) 2010-2019 Alexander Bahle <alexander.bahle@hs-augsburg.de>
                          Gundolf Kiefer <gundolf.kiefer@hs-augsburg.de>
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


#include "peripherals.h"
#include "dm.h"

#define WRITE_DELAY 1
#define READ_DELAY 1

// TBD: Allow real zero-delays (for testing purposes)


void MPeripherals::MainThread (void) {
    TWord adr, sel;
#if CFG_MEMU_BUSIF_WIDTH == 64
    TDWord val;
#else
    TWord val;
#endif
    int n;

    rty_o = 0;
    err_o = 0;

    while (true) {
        wait ();
        if (stb_i == 1 && cyc_i == 1) {
            if (we_i == 1) {

                // Write transfer...
                adr = adr_i.read ();
                val = dat_i.read ();
                sel = sel_i.read ();

                if (memory->IsAdressed (adr)) {
//                    INFOF (("MEMORY: Write (%08x, %016llx) [%x]", adr, val, sel));
                    if (sel == 0xff)
                        memory->WriteDWord (adr, val);
                    else if (sel == 0xf)
                        memory->WriteWord (adr, (TWord)val);
                    else
                        for (n = 0; n < CFG_MEMU_BUSIF_WIDTH/8; n++)
#if PN_BIG_ENDIAN == 1
                            if (sel & (1 << n))
                                memory->WriteByte (adr + n, (val >> (24 - 8 * n)) & 0xff);
#else
                            if (sel & (1 << n)) memory->WriteByte (adr + n, (val >> (8 * n)) & 0xff);
#endif
                    if (WRITE_DELAY > 0) wait (WRITE_DELAY);
                    ack_o = 1;
                } else if ((adr & 0xffff000) == DBG_ADDRESS) {
                    // Write  access to DBGU
                    // Do nothing
                } else {
                    // err_o = 1;
                    WARNINGF (("Write access to non-existing address: %08x - ignoring", adr));
                    ack_o = 1;
                }
                wait ();
            } else {

                // Read transfer...
                adr = adr_i.read ();
                sel = sel_i.read ();
                val = 0;

                if (memory->IsAdressed (adr)) {
                    if (sel == 0xff)
                        val = memory->ReadDWord (adr);
                    else if (sel == 0xf0)
                        val = (TDWord)memory->ReadWord (adr+4) << 32;
                    else if (sel == 0xf)
                        val = memory->ReadWord (adr);
                    else
                        for (n = 0; n < CFG_MEMU_BUSIF_WIDTH/8; n++)
#if PN_BIG_ENDIAN == 1
                            if (sel & (1 << n)) val |= memory->ReadByte (adr + n) << (24 - 8 * n);
#else
                            if (sel & (1 << n)) val |= (TDWord)memory->ReadByte (adr + n) << (8 * n);
#endif
//                    INFOF (("MEMORY: Read (%08x, %016llx) [%x]", adr, val, sel));
                    if (READ_DELAY > 0) wait (READ_DELAY);
                    dat_o = val;
                    ack_o = 1;
                } else if ((adr & 0xffff000) == DBG_ADDRESS) {
                    // Read access to DM
                    // Do nothing
                } else {
                    // err_o = 1;
                    WARNINGF (("Read access to non-existing address: %08x - returning all-one", adr));
                    dat_o = 0xffffffff;
                    ack_o = 1;
                }
                wait ();
            } // if !(we_i == 1)
        } // if (stb_i == 1 && cyc_i == 1)
        ack_o = 0;
    }
}

