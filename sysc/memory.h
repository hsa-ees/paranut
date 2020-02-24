/*************************************************************************

  This file is part of the ParaNut project.

  Copyright (C) 2010-2020 Alexander Bahle <alexander.bahle@hs-augsburg.de>
                          Gundolf Kiefer <gundolf.kiefer@hs-augsburg.de>
      Hochschule Augsburg, University of Applied Sciences

  Description:
    This file defines the class 'CMemory', which simulates a memory
    environment for the testbench of ParaNut.

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


#ifndef _MEMORY_
#define _MEMORY_

#include "base.h"
#include "config.h"

#include <vector>

#define LABEL_LEN 80


extern class CMemory *mainMemory;


class CLabel {
public:
    CLabel (TWord _adr, const char *_name);

    TWord adr;
    char name[LABEL_LEN + 1];
};


class CMemory {
public:
    CMemory () { Init (0, CFG_NUT_MEM_SIZE); }
    CMemory (TWord base, TWord size) { Init (base, size); }

    void Init (TWord base, TWord size);

    bool IsAdressed (TWord adr) { return adr >= base_ && adr < base_ + size_; }

    // Read & Write...
    TByte ReadByte (TWord adr) { return data_[adr - base_]; }
#if PN_BIG_ENDIAN == 1
    THalfWord ReadHalfWord (TWord adr) {
        return ((THalfWord)data[adr - base] << 8) + data[adr - base + 1];
    }
    TWord ReadWord (TWord adr) {
        return ((TWord)data[adr - base] << 24) + ((TWord)data[adr - base + 1] << 16) +
               ((TWord)data[adr - base + 2] << 8) + ((TWord)data[adr - base + 3]);
    }
#else
    THalfWord ReadHalfWord (TWord adr) { return *((THalfWord *)(&data_[adr - base_])); }
    TWord ReadWord (TWord adr) { return *(TWord *)(&data_[adr - base_]); }
    TDWord ReadDWord (TWord adr) {
        return *(TDWord *)&data_[adr - base_];
//        return (*(TWord *)&data_[adr - base_] | (TDWord)((*(TWord *)&data_[adr - base_ + 4])) << 32);
    }
#endif

    void WriteByte (TWord adr, TByte val) { data_[adr - base_] = val; }
#if PN_BIG_ENDIAN == 1
    void WriteHalfWord (TWord adr, THalfWord val) {
        data[adr - base] = (TByte) (val >> 8);
        data[adr - base + 1] = (TByte)val;
    }
    void WriteWord (TWord adr, TWord val) {
        data[adr - base] = (TByte) (val >> 24);
        data[adr - base + 1] = (TByte) (val >> 16);
        data[adr - base + 2] = (TByte) (val >> 8);
        data[adr - base + 3] = (TByte)val;
    }
#else
    void WriteHalfWord (TWord adr, THalfWord val) {
        THalfWord *dst = (THalfWord *)&data_[adr - base_];
        *dst = val;
    }
    void WriteWord (TWord adr, TWord val) {
        TWord *dst = (TWord *)&data_[adr - base_];
        *dst = val;
    }
    void WriteDWord (TWord adr, TDWord val) {
        TWord *dst = (TWord *)&data_[adr - base_];
        *dst = val & 0xffffffff;
//        INFOF (("Write DWORD (0x%08x)0x%016llx: (%p)0x%08x == 0x%08x  ", adr, val, dst, val & 0xffffffff,  *dst ));
        *(dst+1) = (TDWord)(val >> 32);
//        INFOF (("Write DWORD (0x%08x)0x%016llx: (%p)0x%08x == 0x%08x  ",adr, val, dst+1, (val & ~0xffffffff) >> 32,  *(dst+1)));
    }
#endif

    // Read ELF...
    bool ReadFile (char *fileName, bool dumpVHDL);

    // Labels...
    int FindLabel (TWord adr);

    // Dumping...
    char *GetDumpStr (TWord adr);
    char *GetDumpStrVHDL (TWord adr);
    void Dump (TWord adr0 = 0, TWord adr1 = 0xffffffff);
    void DumpVHDL (char *filename, unsigned size);
    void DumpSignature (char *filename);

    TWord tdata_adr;
    TWord tohost_adr;

protected:
    TByte *data_;
    int base_, size_;

    std::vector<CLabel> label_list_;

    bool sig_found_;
    TWord sig_adr_, sig_adr_end_;
};


#endif
