/*************************************************************************

  This file is part of the ParaNut project.

  Copyright (C) 2020 Alexander Bahle <alexander.bahle@hs-augsburg.de>
      Hochschule Augsburg, University of Applied Sciences

  Description:
    This is a SystemC model of a periphery with a Wishbone slave interface that
    is compatible with the Interconnect defined in "interconnect.h".

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

#ifndef _PARANUT_PERIPHERAL_
#define _PARANUT_PERIPHERAL_


/// @file
/// @brief The MPeripheral class containing the interface for Wishbone slave peripherals.


#include <systemc.h>

#include "base.h"
#include "paranut-config.h"


/// @brief Wishbone data port size in bit. 
///
/// Defines the Wishbone data in/out port size (width) in bit. Default is to use the 
/// configuration value @ref CFG_MEMU_BUSIF_WIDTH.
/// @param 32 - 32 bit data width.
/// @param 64 - 64 bit data width.
#define WB_PORT_SIZE CFG_MEMU_BUSIF_WIDTH   


/// @defgroup peripherals Peripherals
/// @brief Wishbone slave peripherals for simulation and hardware.
///
/// This module contains all Wishbone peripherals which can be added to the MParaNutSystem by adding
/// them to the MInterconnect (@ref MParaNutSystem::AddSlave()) and the MPeripheral base class containing the Wishbone 
/// slave interface for peripherals.
/// @{

/// @brief Class containing the interface for Wishbone slave peripherals.
///
/// Defines the Wishbone input and output ports necessary to connect a peripheral to the 
/// simulation Wishbone interconnect (MInterconnect)
///
/// Has no other functionality (no Methods or Threads).
///
/// Can be used as base class for for simulation only peripherals (see MWBMemory for example). 
///
class MPeripheral : public ::sc_core::sc_module {
public:

    /// @name Wishbone slave ports...
    /// @{
    sc_in_clk           wb_clk_i;      ///< WB Clock input.
    sc_in<bool>         wb_rst_i;      ///< WB Reset input.

    sc_in<bool>         wb_stb_i;      ///< WB Strobe input
    sc_in<bool>         wb_cyc_i;      ///< WB cycle valid input.
    sc_in<bool>         wb_we_i;       ///< WB write enable intput.
    sc_in<sc_uint<3> >  wb_cti_i;      ///< WB cycle type identifier (optional, for registered feedback).
    sc_in<sc_uint<2> >  wb_bte_i;      ///< WB burst type extension (optional, for registered feedback).
    sc_in<sc_uint<WB_PORT_SIZE/8> > wb_sel_i; ///< WB byte select inputs.
    sc_out<bool>        wb_ack_o;      ///< WB normal termination.
    sc_out<bool>        wb_err_o;      ///< WB termination w/ error (optional).
    sc_out<bool>        wb_rty_o;      ///< WB termination w/ retry (optional).

    sc_in<sc_uint<32> >             wb_adr_i;  ///< WB address bus inputs.
    sc_in<sc_uint<WB_PORT_SIZE> >   wb_dat_i;  ///< WB input data bus.
    sc_out<sc_uint<WB_PORT_SIZE> >  wb_dat_o;  ///< WB output data bus.
    /// @} // @name Wishbone slave ports...
    
    /// @brief Constructor.
    ///
    /// Does nothing except for naming all the ports for easier elaboration time debugging.
    /// @param name is the module name. 
    MPeripheral (const sc_module_name &name) : sc_module (name),
        wb_clk_i ("wb_clk_i"), wb_rst_i ("wb_rst_i"),
        wb_stb_i ("wb_stb_i"), wb_cyc_i ("wb_cyc_i"), wb_we_i ("wb_we_i"), wb_cti_i ("wb_cti_i"),
        wb_bte_i ("wb_bte_i"), wb_sel_i ("wb_sel_i"),
        wb_ack_o ("wb_ack_o"), wb_err_o ("wb_err_o"), wb_rty_o ("wb_rty_o"),
        wb_adr_i ("wb_adr_i"), wb_dat_i ("wb_dat_i"), wb_dat_o ("wb_dat_o")
    {
    }

    /// @brief Trace function definition.
    ///
    /// Does nothing, but when overridden in a child should add internal signals/registers and ports
    /// to the trace file tf. The trace level is only necessary for peripherals with submodules. 
    /// @note A Trace function must be present in every peripheral that is connected to 
    /// the simulation Wishbone interconnect (MInterconnect).
    /// @param tf is the sc_trace_file to where to add everything. 
    /// @param level is the trace level (for submodules). 
    #ifndef __SYNTHESIS__
      void Trace (sc_trace_file * tf, int level = 1);
    #endif

};

/// @} // @defgroup peripherals Peripherals

#endif // _PARANUT_PERIPHERAL_
