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
    CLabel (uint32_t _adr, const char *_name);

    uint32_t adr;
    char name[LABEL_LEN + 1];
};


class CMemory {
public:
    CMemory () { Init (0, CFG_NUT_MEM_SIZE); }
    CMemory (uint32_t base, uint32_t size) { Init (base, size); }
    ~CMemory ();

    void Init (uint32_t base, uint32_t size);

    bool IsAdressed (uint32_t adr) { return adr >= base_ && adr < base_ + size_; }

    // Read & Write...
    uint8_t ReadByte (uint32_t adr) { return data_[adr - base_]; }
#if PN_BIG_ENDIAN == 1
    uint16_t ReadHalfWord (uint32_t adr) {
        return ((uint16_t)data[adr - base] << 8) + data[adr - base + 1];
    }
    uint32_t ReadWord (uint32_t adr) {
        return ((uint32_t)data[adr - base] << 24) + ((uint32_t)data[adr - base + 1] << 16) +
               ((uint32_t)data[adr - base + 2] << 8) + ((uint32_t)data[adr - base + 3]);
    }
#else
    uint16_t ReadHalfWord (uint32_t adr) { return *((uint16_t *)(&data_[adr - base_])); }
    uint32_t ReadWord (uint32_t adr) { return *(uint32_t *)(&data_[adr - base_]); }
    uint64_t ReadDWord (uint32_t adr) {
        return *(uint64_t *)&data_[adr - base_];
//        return (*(uint32_t *)&data_[adr - base_] | (uint64_t)((*(uint32_t *)&data_[adr - base_ + 4])) << 32);
    }
#endif

    void WriteByte (uint32_t adr, uint8_t val) { data_[adr - base_] = val; }
#if PN_BIG_ENDIAN == 1
    void WriteHalfWord (uint32_t adr, uint16_t val) {
        data[adr - base] = (uint8_t) (val >> 8);
        data[adr - base + 1] = (uint8_t)val;
    }
    void WriteWord (uint32_t adr, uint32_t val) {
        data[adr - base] = (uint8_t) (val >> 24);
        data[adr - base + 1] = (uint8_t) (val >> 16);
        data[adr - base + 2] = (uint8_t) (val >> 8);
        data[adr - base + 3] = (uint8_t)val;
    }
#else
    void WriteHalfWord (uint32_t adr, uint16_t val) {
        uint16_t *dst = (uint16_t *)&data_[adr - base_];
        *dst = val;
        // PN_INFOF (("Write WriteHalfWord (0x%08x)0x%016llx: (%p)0x%08x == 0x%08x  ", adr, val, dst, val & 0xffffffff,  *dst ));
        
    }
    void WriteWord (uint32_t adr, uint32_t val) {
        uint32_t *dst = (uint32_t *)&data_[adr - base_];
        *dst = val;
        // PN_INFOF (("Write WriteWord (0x%08x)0x%016llx: (%p)0x%08x == 0x%08x  ", adr, val, dst, val & 0xffffffff,  *dst ));
        
    }
    void WriteDWord (uint32_t adr, uint64_t val) {
        uint32_t *dst = (uint32_t *)&data_[adr - base_];
        *dst = val & 0xffffffff;
        // PN_INFOF (("Write DWORD (0x%08x)0x%016llx: (%p)0x%08x == 0x%08x  ", adr, val, dst, val & 0xffffffff,  *dst ));
        *(dst+1) = (uint64_t)(val >> 32);
//        PN_INFOF (("Write DWORD (0x%08x)0x%016llx: (%p)0x%08x == 0x%08x  ",adr, val, dst+1, (val & ~0xffffffff) >> 32,  *(dst+1)));
    }
#endif

    // Read ELF...
    bool ReadFile (const char *filename, const bool dumpVHDL);

    // Labels...
    int FindLabel (uint32_t adr);

    // Dumping...
    char *GetDumpStr (uint32_t adr);
    char *GetDumpStrVHDL (uint32_t adr);
    void Dump (uint32_t adr0 = 0, uint32_t adr1 = 0xffffffff);
    void DumpVHDL (const char *filename, unsigned size);
    void DumpSignature (const char *filename);

    uint32_t tdata_adr;
    uint32_t tohost_adr;

protected:
    uint8_t*data_;
    uint32_t base_, size_;

    std::vector<CLabel> label_list_;

    bool sig_found_;
    uint32_t sig_adr_, sig_adr_end_;
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
    MWBMemory(const sc_module_name& name, uint32_t base, uint32_t size) : MPeripheral(name), CMemory(base, size),
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
    uint64_t ReadMemory (uint32_t adr, uint32_t sel);
    void WriteMemory (uint32_t adr, uint32_t sel, uint64_t val);

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
