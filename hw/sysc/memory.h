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

#include "paranut-peripheral.h"

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
    ~CMemory ();

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
//        PN_INFOF (("Write DWORD (0x%08x)0x%016llx: (%p)0x%08x == 0x%08x  ", adr, val, dst, val & 0xffffffff,  *dst ));
        *(dst+1) = (TDWord)(val >> 32);
//        PN_INFOF (("Write DWORD (0x%08x)0x%016llx: (%p)0x%08x == 0x%08x  ",adr, val, dst+1, (val & ~0xffffffff) >> 32,  *(dst+1)));
    }
#endif

    // Read ELF...
    bool ReadFile (const char *filename, const bool dumpVHDL);

    // Labels...
    int FindLabel (TWord adr);

    // Dumping...
    char *GetDumpStr (TWord adr);
    char *GetDumpStrVHDL (TWord adr);
    void Dump (TWord adr0 = 0, TWord adr1 = 0xffffffff);
    void DumpVHDL (const char *filename, unsigned size);
    void DumpSignature (const char *filename);

    TWord tdata_adr;
    TWord tohost_adr;

protected:
    TByte *data_;
    TWord base_, size_;

    std::vector<CLabel> label_list_;

    bool sig_found_;
    TWord sig_adr_, sig_adr_end_;
};

#ifndef __SYNTHESIS__

/// @addtogroup peripherals 
/// @{

/// @brief Simulation ready configurable Wishbone slave memory.
///
/// This class defines a memory with a Wishbone slave interface and all other necessary functionality
/// to add it to a *ParaNut* systems Wishbone interconnect (MInterconnect).
///
/// It is not synthesizable but rather focusses on functionality to control its behaviour and data 
/// during SystemC simulation. 
///
/// Derived from MPeripheral and CMemory.
///
class MWBMemory : public MPeripheral, public CMemory {
public:
    // Only need a Wishbone slave interface inherited from MPeripheral...

    SC_HAS_PROCESS(MWBMemory); // Necessary when not using SC_MODULE macro
    
    /// @brief Create a Wishbone slave memory.
    ///
    /// Reserves memory of size, sets it to 0 and sets internal base address to base by calling 
    /// CMemory() constructor. Sets defaults for the simulation timing members (see SetDelays()) and
    /// initialize the SC_THREAD for MainThread(). 
    /// @param name is the module name. 
    /// @param base is the memory base address in the ParaNut system. 
    /// @param size is the memory size in byte. 
    MWBMemory(const sc_module_name& name, TWord base, TWord size) : MPeripheral(name), CMemory(base, size),
            wr_setup(5), rd_setup(5), wr_delay (0), rd_delay (0) {
        SC_THREAD (MainThread); // Using non synthesizable SC_THREAD to implement wr/rd setup & burst delays
            sensitive << wb_clk_i.pos();
            reset_signal_is (wb_rst_i, true);
    }

    // Functions...
    /// @brief Set read and write setup and delay clock cycles.
    ///
    /// Set internal variables for read and write setup clocks (time from request to first 
    /// acknowledge) and delay clocks for bursts (time between last and next acknowledge). 
    /// @note The delay is currently disabled and not applied during bursts!
    /// @param rd_setup is the amount of clocks between request and first acknowledge during read. 
    /// @param rd_delay is the amount of clocks between last and next acknowledge during burst read. 
    /// @param wr_setup is the amount of clocks between request and first acknowledge during write. 
    /// @param wr_delay is the amount of clocks between last and next acknowledge during burst write. 
    void SetDelays (uint rd_setup, uint rd_delay, uint wr_setup, uint wr_delay);


    // Processes...
    /// @brief Main SC_THREAD implements the Wishbone slave interface.
    ///
    /// Implements the Wishbone slave to CMemory interace. Is a SC_THREAD for easier setup and delay
    /// realisation.
    void MainThread ();

private:
    // Helper Functions...
    TDWord ReadMemory (TWord adr, TWord sel);
    void WriteMemory (TWord adr, TWord sel, TDWord val);

    // Setup cycles:
    uint wr_setup;
    uint rd_setup;
    // Burst delay:
    uint wr_delay;
    uint rd_delay;
};

/// @} // @addtogroup peripherals 

#endif

#endif
