/*************************************************************************

  This file is part of the ParaNut project.

  Copyright (C) 2019-2022 Alexander Bahle <alexander.bahle@hs-augsburg.de>
                          Christian H. Meyer <christian.meyer@hs-augsburg.de>
      Hochschule Augsburg, University of Applied Sciences

  Description:
    This is a SystemC model of a Debug Module compatible with the
    RISC-V External Debug Support Version 0.13

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


#pragma once

#include "paranut-config.h"

#define EX_ID_LENGTH 5

typedef enum {
    perfInstret,
    perfALU,
    perfLoad,
    perfStore,
    perfJump,
    perfSystem,
    perfLoadStoreWait,
} EPerfCount;

typedef enum {
    User = 0,
    Supervisor = 1,
    Machine = 3,
} EPrivMode;

typedef enum {
    off = 0, // All off
    initial, // None dirty or clean, some on
    clean, // None dirty, some clean
    dirty, // Some dirty
} EXS;

// **************** Exception Codes *************
typedef enum {
    /*** Interrupts: ***/   
    UserSoftwareInterrupt = 0,
    SupervisorSoftwareInterrupt = 1,
    MachineSoftwareInterrupt = 3,
    UserTimerInterrupt = 4,
    SupervisorTimerInterrupt = 5,
    MachineTimerInterrupt = 7,
    UserExternalInterrupt = 8,
    SupervisorExternalInterrupt = 9,
    MachineExternalInterrupt = 11,
    
    /*** Exceptions: ***/
    InstructionAddressMisaligned = 0,
    // InstructionAccessFault,                 // no access faults from IFU
    IllegalInstruction = 2,
    Breakpoint = 3,
    LoadAddressMisaligned = 4,
    // LoadAccessFault,                        // no access faults from LSU
    StoreAddressMisaligned = 6,
    // StoreAccessFault,                       // no access faults from LSU
    ECallU = 8,
    ECallS = 9,
    // 10 Reserved
    ECallM = 11,
    InstructionPageFault = 12,
    LoadPageFault = 13,
    // 14 Reserved
    StorePageFault = 15,
    // >= 16 Reserved
    CoPUException = 16
} EExceptions;