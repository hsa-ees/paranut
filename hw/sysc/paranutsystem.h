/*************************************************************************

  This file is part of the ParaNut project.

  Copyright (C) 2020 Alexander Bahle <alexander.bahle@hs-augsburg.de>
      Hochschule Augsburg, University of Applied Sciences

  Description:
    This is the top-level SystemC model of a complete ParaNut system including
    the ParaNut itself, a Wishbone interconnect and the main memory.

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

#ifndef _PARANUTSYSTEM_H
#define _PARANUTSYSTEM_H


/// @file
/// @brief The MParaNutSystem class for simulation.


#include "base.h"
#include "paranut-config.h"
#include "nut.h"
#include "memory.h"
#include "interconnect.h"
#include "remote_bitbang.h"
#include "jtag_dtm.h"


/// @defgroup simulator Simulator
/// @brief ParaNut simulation system and control.
/// 
/// This module groups the simulation system and it's control functions.
///
/// A minimal SystemC main program to simulate a ParaNut system could look like this:
/// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.cpp}
/// #include <systemc.h>
/// #include <paranut-sim.h>
/// 
/// int sc_main (int argc, char *argv[]) {
///   MParaNutSystem paranut (argc, argv);
///   paranut.Run ();  
///   return 0;
/// }
/// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
/// @{


/// @brief Complete ParaNut System ready for simulation.
///
/// This system contains a ParaNut processeor (MParanut) connected as master to a Wishbone
/// interconnect (MInterconnect) which in turn is connected to the Main Memory (MWBMemory).
///
/// The system is Elaborated by executing the Constructor with the necessary command line parameters
/// (see Usage()).
///
/// You can expand the system by adding Wishbone slave peripherals (see MPeripheral for inspiration)
/// using the AddSlave() function.
///
/// Simulation is started by calling the Run() function.
///
class MParaNutSystem {
public:

    // TBD: Make signals private? Do we have a use case for leaving these public?
    // **************** Signals *********************
    /// @name Wishbone signals...
    /// @{
    sc_signal<bool> clk, reset;
    sc_signal<bool> wb_stb, wb_cyc, wb_we, wb_ack, wb_err, wb_rty;
    sc_signal<sc_uint<3> > wb_cti;
    sc_signal<sc_uint<2> > wb_bte;
    sc_signal<sc_uint<CFG_MEMU_BUSIF_WIDTH/8> > wb_sel;
    sc_signal<sc_uint<32> > wb_adr;
    sc_signal<sc_uint<CFG_MEMU_BUSIF_WIDTH> > wb_dat_w, wb_dat_r;
    /// @}
    /// @name Other signals (Interrupt, JTAG, etc.)...
    /// @{
    // Interrupt:
    sc_signal<bool> ex_int[CFG_NUT_EX_INT];
    // JTAG
    sc_signal<bool> tck, tms, tdi, tdo;
    ///@}

    /// @name Constructor / Destructor...
    /// @{
    /// @ingroup simulator

    ////////////////////////////////////////////////////////////////////////////////////////////////
    /// @ingroup simulator 
    /// @brief Create and elablorate the System.
    ///
    /// Parses the supplied argc and argv to configure the simulation and elaborates the System
    /// containing a ParaNut (MParanut), Wishbone interconnect (MInterconnect) and the Main Memory
    /// (MWBMemory).
    /// @param argc is the number of arguments in argv.
    /// @param argv is the array of arguments that gets parsed.
    ////////////////////////////////////////////////////////////////////////////////////////////////
    MParaNutSystem(const int argc, char *argv[]);
    ////////////////////////////////////////////////////////////////////////////////////////////////
    /// @ingroup simulator 
    /// @brief Destructor destroying all runtime objects.
    ////////////////////////////////////////////////////////////////////////////////////////////////
    ~MParaNutSystem();
    /// @}


    // Functions...
    /// @name System Construction...
    /// @{
    
    ////////////////////////////////////////////////////////////////////////////////////////////////
    /// @ingroup simulator 
    /// @brief Add a slave peripheral to the ParaNut System.
    ///
    /// The provided slave peripheral will be added to the Wishbone interconnect at start_adr and
    /// will occupie size amount of bytes.
    ///
    /// The interconnect checks for address space collisions and will stop execution on error.
    ///
    /// NOTE: Calling with a non MPeripheral derived or compatible slave value will likely lead to
    /// unwanted behaviour or a segmentation fault!
    ///
    /// @param start_adr is the I/O base address.
    /// @param size is the number of bytes occupied by the slave.
    /// @param slave is a pointer to a MPeripheral derived or compatible module.
    ////////////////////////////////////////////////////////////////////////////////////////////////
    void AddSlave(TWord start_adr, size_t size, MPeripheral *slave) {interconnect->AddSlave (start_adr, size, slave);}

    ////////////////////////////////////////////////////////////////////////////////////////////////
    /// @ingroup simulator 
    /// @brief Add a slave port to the ParaNut Systems internal interrupt input signals.
    ///
    /// The provided port will be bound to the interupt input at the given index.
    /// @param index is the index of the interrupt input (must be < @ref CFG_NUT_EX_INT).
    /// @param port is the pointer of a sc_out<bool> port that will be bound.
    ////////////////////////////////////////////////////////////////////////////////////////////////
    void AddInterrupt(TWord index, sc_out<bool> *port);
    /// @}

    /// @name Execution Control...
    /// @{

    ////////////////////////////////////////////////////////////////////////////////////////////////
    /// @ingroup simulator 
    /// @brief Start the SystemC simulation and wait for the ParaNut to HALT.
    ///
    /// Sets up and starts the debugging interfaces if the interactive debug flag is set (-d).
    ///
    /// Prints performance statistics (-p), memory content befor and after completion (-m) and dumps
    /// signature section contents after completion (-s)
    ////////////////////////////////////////////////////////////////////////////////////////////////
    void Run ();
    /// @}

    /// @name Support Functions...
    /// @{

    ////////////////////////////////////////////////////////////////////////////////////////////////
    /// @ingroup simulator 
    /// @brief Print usage of the command line options to stdout.
    ////////////////////////////////////////////////////////////////////////////////////////////////
    void Usage ();
    ////////////////////////////////////////////////////////////////////////////////////////////////
    /// @ingroup simulator 
    /// @brief Print the current configuration to stderr.
    ///
    /// Uses the @ref PS_PRINTCAT and @ref PS_PRINTCONF macros.
    ////////////////////////////////////////////////////////////////////////////////////////////////
    void PrintConfig ();
    /// @}

private:
    MWBMemory *mmemory;
    MInterconnect *interconnect;
    MParanut *paranut;
    sc_trace_file *tf;      // trace file

    // Simulation debugging:
    jtag_dtm_t *jtag_dtm;
    remote_bitbang_t *rbb;

    // Configuration options
    const char *cfg_elf_filename;
    int cfg_dump_from, cfg_dump_to;
    int cfg_perf_lvl = 0;
    bool cfg_dump_VHDL, cfg_dump_signature;


    void RunCycles (const int n = 1);
    void SimHost ();
};

/// @} // @defgroup simulator Simulator

#endif // _PARANUTSYSTEM_H
