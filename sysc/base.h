/************************************************************************
*
  This file is part of the ParaNut project.

  Copyright (C) 2010-2020 Alexander Bahle <alexander.bahle@hs-augsburg.de>
                          Gundolf Kiefer <gundolf.kiefer@hs-augsburg.de>
      Hochschule Augsburg, University of Applied Sciences

  Description:
    This module contains various types, constants and helper functions
    for the SystemC model of ParaNut.

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


#ifndef _BASE_
#define _BASE_

#include <systemc.h>

#ifndef SIMBUILD
#ifdef __SYNTHESIS__
#warning "__SYNTHESIS__ is set!"
#else
#warning "__SYNTHESIS__ is not set!"
#endif
#endif

// Static Configuration...

#define PN_BIG_ENDIAN 0


// Dynamic Configuration...

extern int cfg_vcd_level; // VCD trace level (0 = no VCD file)
extern int cfg_insn_trace; // if set, simulation info is printed with each instruction
extern int cfg_disable_cache; // if set, caching is disabled by the EXUs, independent of the ICE/DCE flag


// Basic types and constants...

#define KB 1024
#define MB (1024 * 1024)
#define GB (1024 * 1024 * 1024)

typedef unsigned char TByte;
typedef unsigned THalfWord;
typedef unsigned TWord;
typedef unsigned long long TDWord;

#define MIN(A, B) ((A) < (B) ? (A) : (B))
#define MAX(A, B) ((A) > (B) ? (A) : (B))

#define XLEN 32

// SystemC tracing...

extern bool trace_verbose;

#ifndef __SYNTHESIS__
char *GetTraceName (sc_object *obj, const char *name, int dim, int arg1, int arg2);
#else
char *GetTraceName (...) {
    return "\0";
}
#endif

#define TRACE(TF, OBJ)                                                                \
    {                                                                                 \
        if (TF) sc_trace (TF, OBJ, GetTraceName (&(OBJ), #OBJ, 0, 0, 0));             \
        if (!TF || trace_verbose) cout << "  " #OBJ " = '" << (OBJ).name () << "'\n"; \
    }


#define TRACE_BUS(TF, OBJ, N_MAX)                                                     \
    {                                                                                 \
        for (int n = 0; n < N_MAX; n++) {                                             \
            if (TF) sc_trace (TF, (OBJ)[n], GetTraceName (&(OBJ)[n], #OBJ, 1, n, 0)); \
            if (!TF || trace_verbose)                                                 \
                cout << "  " #OBJ "[" << n << "] = '" << (OBJ)[n].name () << "'\n";   \
        }                                                                             \
    }

#define TRACE_BUS_BUS(TF, OBJ, N_MAX, K_MAX)                                                            \
    {                                                                                                   \
        for (int n = 0; n < N_MAX; n++)                                                                 \
            for (int k = 0; k < K_MAX; k++) {                                                           \
                if (TF) sc_trace (TF, (OBJ)[n][k], GetTraceName (&(OBJ)[n][k], #OBJ, 2, n, k));         \
                if (!TF || trace_verbose)                                                               \
                    cout << "  " #OBJ "[" << n << "][" << k << "] = '" << (OBJ)[n][k].name () << "'\n"; \
            }                                                                                           \
    }


#define PRINT(OBJ) TRACE (NULL, OBJ)
#define PRINT_BUS(OBJ, N_MAX) TRACE_BUS (NULL, OBJ, N_MAX)
#define PRINT_BUS_BUS(OBJ, N_MAX, K_MAX) TRACE_BUS (NULL, OBJ, N_MAX, K_MAX)


// **************** Testbench helpers ***********


extern sc_trace_file *trace_file;

char *TbPrintf (const char *format, ...);
void TbAssert (bool cond, const char *msg, const char *filename, const int line);
void TbInfo (const char *msg, const char *filename, const int line);
void TbWarning (const char *msg, const char *filename, const int line);
void TbError (const char *msg, const char *filename, const int line);


/*
char *TbStringF (const char *format, ...) {
}
*/

#ifndef __SYNTHESIS__
#define ASSERT(COND) TbAssert (COND, NULL, __FILE__, __LINE__)
#define ASSERTF(COND, FMT) TbAssert (COND, TbPrintf FMT, __FILE__, __LINE__)
#define ASSERTM(COND, MSG) TbAssert (COND, MSG, __FILE__, __LINE__)

#define INFO(MSG) TbInfo (MSG, __FILE__, __LINE__)
#define INFOF(FMT) TbInfo (TbPrintf FMT, __FILE__, __LINE__)

#define WARNING(MSG) TbWarning (MSG, __FILE__, __LINE__)
#define WARNINGF(FMT) TbWarning (TbPrintf FMT, __FILE__, __LINE__)

#define ERROR(MSG) TbError (MSG, __FILE__, __LINE__)
#define ERRORF(FMT) TbError (TbPrintf FMT, __FILE__, __LINE__)
#else
#define ASSERT(COND)
#define ASSERTF(COND, FMT)
#define ASSERTM(COND, MSG)

#define INFO(MSG)
#define INFOF(FMT)

#define WARNING(MSG)
#define WARNINGF(FMT)

#define ERROR(MSG)
#define ERRORF(FMT)
#endif


// Functions...

char *DisAss (TWord insn); // disassemble RISC-V instruction; return string valid until next call to this function


// **************** Performance measuring *****************

class CEventDef {
    public:
    const char *name;
    bool is_timed;
};


class CPerfMon {
    public:
    CPerfMon () { Init (0, NULL); }
    CPerfMon (int events, CEventDef *ev_tab) { Init (events, ev_tab); }
    ~CPerfMon () { Done (); }

    void Init (int events, CEventDef *ev_tab);
    void Done ();

    void Reset ();
    void Count (int ev_no);

    void Display (const char *name = NULL);

    protected:
    int events_;
    CEventDef *ev_tab_;
    int *count_tab_;
    double *time_tab_, *min_tab_, *max_tab_;

    double last_stamp_;
    int last_no_;
};


// ***** CPerfMonCPU *****

typedef enum { EV_ALU = 0, EV_LOAD, EV_STORE, EV_JUMP, EV_OTHER } EEventsCPU;

#ifndef __SYNTHESIS__
class CPerfMonCPU : public CPerfMon {
    public:
    CPerfMonCPU () { Init (); }
    void Init ();

    void Count (EEventsCPU ev_no) { CPerfMon::Count ((int)ev_no); }
};
#else
class CPerfMonCPU {
    public:
    CPerfMonCPU () {}

    void Display () { /* nothing */
    }
    void Count (EEventsCPU ev_no) { /* nothing */
    }
};
#endif


#endif
