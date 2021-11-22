/*************************************************************************

  This file was copied and modified from the Spike ISA Simulator project:
    https://github.com/riscv/riscv-isa-sim

Copyright (c) 2010-2017, The Regents of the University of California
(Regents).  All Rights Reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
1. Redistributions of source code must retain the above copyright
   notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright
   notice, this list of conditions and the following disclaimer in the
   documentation and/or other materials provided with the distribution.
3. Neither the name of the Regents nor the
   names of its contributors may be used to endorse or promote products
   derived from this software without specific prior written permission.

IN NO EVENT SHALL REGENTS BE LIABLE TO ANY PARTY FOR DIRECT, INDIRECT,
SPECIAL, INCIDENTAL, OR CONSEQUENTIAL DAMAGES, INCLUDING LOST PROFITS, ARISING
OUT OF THE USE OF THIS SOFTWARE AND ITS DOCUMENTATION, EVEN IF REGENTS HAS
BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

REGENTS SPECIFICALLY DISCLAIMS ANY WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
PURPOSE. THE SOFTWARE AND ACCOMPANYING DOCUMENTATION, IF ANY, PROVIDED
HEREUNDER IS PROVIDED "AS IS". REGENTS HAS NO OBLIGATION TO PROVIDE
MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.

 *************************************************************************/

#include <stdio.h>

#include "jtag_dtm.h"
//#define SIMBUILD
//#define __SYNTHESIS__

// Two code versions, one for synthesis and one for simulation
#if !defined(__SYNTHESIS__) && defined(SIMBUILD)
// Workaround for jtag testbench (test synthesis, not sim module
#if defined(JTAG_TB_SIM)
  #define USE 0
#else
  // Use simulation source
  #define USE 1
#endif
#else
// Use synthesis source
#define USE 0
#endif


#if USE == 1 // Simulation source?

#if 0
#  define D(x) x
#else
#  define D(x)
#endif

enum {
  IR_IDCODE=1,
  IR_DTMCONTROL=0x10,
  IR_DBUS=0x11,
  IR_RESET=0x1c
};

#define DTMCONTROL_VERSION      0xf
#define DTMCONTROL_ABITS        (0x3f << 4)
#define DTMCONTROL_DBUSSTAT     (3<<10)
#define DTMCONTROL_IDLE         (7<<12)
#define DTMCONTROL_DBUSRESET    (1<<16)

#define DMI_OP                 3
#define DMI_DATA               (0xffffffffLL<<2)
#define DMI_ADDRESS            ((1LL<<(abits+34)) - (1LL<<34))

#define DMI_OP_STATUS_SUCCESS	0
#define DMI_OP_STATUS_RESERVED	1
#define DMI_OP_STATUS_FAILED	2
#define DMI_OP_STATUS_BUSY	3

#define DMI_OP_NOP	        0
#define DMI_OP_READ	        1
#define DMI_OP_WRITE	        2
#define DMI_OP_RESERVED	        3

#define DTM_DTMCS_ABITS_OFFSET 4

#define get_field(reg, mask) (((reg) & (mask)) / ((mask) & ~((mask) << 1)))
#define set_field(reg, mask, val) (((reg) & ~(mask)) | (((val) * ((mask) & ~((mask) << 1))) & (mask)))

jtag_dtm_t::jtag_dtm_t(MDtm *dtm, unsigned required_rti_cycles) :
  dtm(dtm), required_rti_cycles(required_rti_cycles),
  _tck(false), _tms(false), _tdi(false), _tdo(false),
  dtmcontrol((4<<12) | (abits << DTM_DTMCS_ABITS_OFFSET) | 1),
  dmi(DMI_OP_STATUS_SUCCESS),
  _state(TEST_LOGIC_RESET)
{
}

void jtag_dtm_t::reset() {
  _state = TEST_LOGIC_RESET;
  busy_stuck = false;
  rti_remaining = 0;
  dmi = 0;
}

void jtag_dtm_t::set_pins(bool tck, bool tms, bool tdi) {
  const jtag_state_t next[16][2] = {
    /* TEST_LOGIC_RESET */    { RUN_TEST_IDLE, TEST_LOGIC_RESET },
    /* RUN_TEST_IDLE */       { RUN_TEST_IDLE, SELECT_DR_SCAN },
    /* SELECT_DR_SCAN */      { CAPTURE_DR, SELECT_IR_SCAN },
    /* CAPTURE_DR */          { SHIFT_DR, EXIT1_DR },
    /* SHIFT_DR */            { SHIFT_DR, EXIT1_DR },
    /* EXIT1_DR */            { PAUSE_DR, UPDATE_DR },
    /* PAUSE_DR */            { PAUSE_DR, EXIT2_DR },
    /* EXIT2_DR */            { SHIFT_DR, UPDATE_DR },
    /* UPDATE_DR */           { RUN_TEST_IDLE, SELECT_DR_SCAN },
    /* SELECT_IR_SCAN */      { CAPTURE_IR, TEST_LOGIC_RESET },
    /* CAPTURE_IR */          { SHIFT_IR, EXIT1_IR },
    /* SHIFT_IR */            { SHIFT_IR, EXIT1_IR },
    /* EXIT1_IR */            { PAUSE_IR, UPDATE_IR },
    /* PAUSE_IR */            { PAUSE_IR, EXIT2_IR },
    /* EXIT2_IR */            { SHIFT_IR, UPDATE_IR },
    /* UPDATE_IR */           { RUN_TEST_IDLE, SELECT_DR_SCAN }
  };

  if (!_tck && tck) {
    // Positive clock edge.

    switch (_state) {
      case SHIFT_DR:
        dr >>= 1;
        dr |= (uint64_t) _tdi << (dr_length-1);
        break;
      case SHIFT_IR:
        ir >>= 1;
        ir |= _tdi << (ir_length-1);
        break;
      default:
        break;
    }

    _state = next[_state][_tms];

    switch (_state) {
      case RUN_TEST_IDLE:
        if (rti_remaining > 0)
          rti_remaining--;
        break;
      case TEST_LOGIC_RESET:
        ir = IR_IDCODE;
        break;
      case CAPTURE_DR:
        capture_dr();
        break;
      case SHIFT_DR:
        _tdo = dr & 1;
        break;
      case UPDATE_DR:
        update_dr();
        break;
      case CAPTURE_IR:
        break;
      case SHIFT_IR:
        _tdo = ir & 1;
        break;
      case UPDATE_IR:
        if (ir == IR_RESET) {
           // Make a reset happen
//           reset();
        }
        break;
      default:
        break;
    }
  }

  D(fprintf(stderr, "state=%2d, tdi=%d, tdo=%d, tms=%d, tck=%d, ir=0x%02x, "
        "dr=0x%lx\n",
        _state, _tdi, _tdo, _tms, _tck, ir, dr));

  _tck = tck;
  _tms = tms;
  _tdi = tdi;
}

void jtag_dtm_t::capture_dr()
{
  switch (ir) {
    case IR_IDCODE:
      dr = idcode;
      dr_length = 32;
      break;
    case IR_DTMCONTROL:
      dr = dtmcontrol;
      dr_length = 32;
      break;
    case IR_DBUS:
      dr = dmi;
      dr_length = abits + 34;
      break;
    default:
      D(fprintf(stderr, "Unsupported IR: 0x%x\n", ir));
      break;
  }
  D(fprintf(stderr, "Capture DR; IR=0x%x, DR=0x%lx (%d bits)\n",
        ir, dr, dr_length));
}

void jtag_dtm_t::update_dr()
{
  D(fprintf(stderr, "Update DR; IR=0x%x, DR=0x%lx (%d bits)\n",
        ir, dr, dr_length));
  if (ir == IR_DTMCONTROL) {
    if (dr & DTMCONTROL_DBUSRESET)
      reset();
  } else if (ir == IR_DBUS) {
    if (rti_remaining > 0 || busy_stuck) {
        dmi = DMI_OP_STATUS_BUSY;
        busy_stuck = true;
    } else {
      unsigned op = get_field(dr, DMI_OP);
      uint32_t data = get_field(dr, DMI_DATA);
      unsigned address = get_field(dr, DMI_ADDRESS);

      dmi = dr;

      bool success = true;
      if (op == DMI_OP_READ) {
        if (dtm->dmi_read(address, &dmi)) {
          D(fprintf(stderr, "dmi_read=(0x%x) 0x%lx\n",address, dmi));
//          dmi = set_field(dmi, DMI_DATA, value);
        } else {
          success = false;
        }
      } else if (op == DMI_OP_WRITE) {
        D(fprintf(stderr, "dmi_write=(0x%x) 0x%lx\n",address, data));
        success = dtm->dmi_write(address, data);
      }

      if (success) {
        dmi = set_field(dmi, DMI_OP, DMI_OP_STATUS_SUCCESS);
      } else {
        dmi = set_field(dmi, DMI_OP, DMI_OP_STATUS_FAILED);
      }
      D(fprintf(stderr, "dmi=0x%lx\n", dmi));

      rti_remaining = required_rti_cycles;
    }
  }
}

void MDtm::Trace (sc_trace_file *tf, int level) {
    if (!tf || pn_trace_verbose) printf ("\nSignals of Module \"%s\":\n", name ());

    // Ports...
    PN_TRACE (tf, reset);

    PN_TRACE (tf, tck);
    PN_TRACE (tf, tms);
    PN_TRACE (tf, tdi);
    PN_TRACE (tf, tdo);

    PN_TRACE (tf, dmi_adr);
    PN_TRACE (tf, dmi_dat_o);
    PN_TRACE (tf, dmi_dat_i);
    PN_TRACE (tf, dmi_rd);
    PN_TRACE (tf, dmi_wr);

    // Signals
    PN_TRACE (tf, dmi);
    PN_TRACE (tf, dr);
    PN_TRACE (tf, ir);
    PN_TRACE (tf, dmi_length);
    PN_TRACE (tf, dmi_op);
    PN_TRACE (tf, state);
}

void MDtm::JTAGMethod () {
    // Reset
    if (reset) {
        adr = 0;
        val = 0;
        write = 0;
        val_out = nullptr;
    }

    // Read value if we set dmi_rd was active for last cycle
    if (dmi_rd_last[2] == true && val_out != NULL) {
        *val_out = set_field (*val_out, DMI_DATA, (TWord)dmi_dat_i.read ());
    }
    dmi = dmi_dat_i.read ();

    // Remember current dmi_rd value
    dmi_rd_last[2] = dmi_rd_last[1];
    dmi_rd_last[1] = dmi_rd_last[0];
    dmi_rd_last[0] = dmi_rd;

    // Set output registers
    dmi_adr = adr;
    dmi_dat_o = val;
    dmi_rd = read;
    dmi_wr = write;


    // Set read and write to 0 for next cycle
    if (read) read = !read;
    if (write) write = !write;
}

void MDtm::OutputMethod () {
    // NOTHING
}

bool MDtm::dmi_write (TWord adr, TWord val) {
    // Only continue if neither read nor write are set
    if (!read && !write) {
        this->adr = adr;
        this->val = val;

        read = 0;
        write = 1;

        return true;
    } else {
        return false;
    }
}

bool MDtm::dmi_read (TWord adr, uint64_t *val) {
    // Only continue if neither read nor write are set
    if (!read && !write) {
        this->adr = adr;
        this->val = 0;

        read = 1;
        write = 0;

        val_out = val;

        return true;
    } else {
        return false;
    }
}
#else // USE != 1 - Synthesis source

#ifndef __SYNTHESIS__
void MDtm::Trace (sc_trace_file *tf, int level) {
    if (!tf || pn_trace_verbose) printf ("\nSignals of Module \"%s\":\n", name ());

    // Ports...
    PN_TRACE (tf, reset);

    PN_TRACE (tf, tck);
    PN_TRACE (tf, tms);
    PN_TRACE (tf, tdi);
    PN_TRACE (tf, tdo);

    PN_TRACE (tf, dmi_adr);
    PN_TRACE (tf, dmi_dat_o);
    PN_TRACE (tf, dmi_dat_i);
    PN_TRACE (tf, dmi_rd);
    PN_TRACE (tf, dmi_wr);

    // Signals
    PN_TRACE (tf, dmi);
    PN_TRACE (tf, dr);
    PN_TRACE (tf, ir);
    PN_TRACE (tf, dmi_length);
    PN_TRACE (tf, dmi_op);
    PN_TRACE (tf, state);
}
#endif

void MDtm::JTAGMethod () {

    sc_uint<34 + DTM_ADDR_WIDTH> dr_var;
    sc_uint<DTM_IR_WIDTH> ir_var;

    dr_var = dr.read ();
    ir_var = ir.read ();

    if (dmi_op)
        dmi = (dr_var (33 + DTM_ADDR_WIDTH, 34), (sc_uint<32>)dmi_dat_i.read (), sc_uint<2> (0));

    dmi_op = 0;

    switch (state.read ()) {
    case TEST_LOGIC_RESET:
        ir = IDCODE;
        state = tms ? TEST_LOGIC_RESET : RUN_TEST_IDLE;
        break;
    case RUN_TEST_IDLE:
        state = tms ? SELECT_DR_SCAN : RUN_TEST_IDLE;
        break;
    case SELECT_DR_SCAN:
        state = tms ? SELECT_IR_SCAN : CAPTURE_DR;
        break;
    case CAPTURE_DR:
        switch (ir_var) {
        case IDCODE:
            dr = 0xdeadbeef;
            dmi_length = 0;
            break;
        case DTMCS:
            dr = (4 << 12) | (DTM_ADDR_WIDTH << 4) | 1;
            dmi_length = 0;
            break;
        case DMI:
            dr = dmi.read ();
            dmi_length = 1;
            break;
        default:
            PN_ERRORF (("JTAG_DTM PN_ERROR: Unsupported IR: 0x%x\n", ir_var.value()));
            break;
        }
        state = tms ? EXIT1_DR : SHIFT_DR;
        break;
    case SHIFT_DR:
        // Two shift variants for normal and dmi register
        //        tdo = dr_var[0];
        if (dmi_length)
            dr = (tdi.read (), dr_var (33 + DTM_ADDR_WIDTH, 1));
        else
            dr = (sc_uint<DTM_ADDR_WIDTH - 1> (0), tdi.read (), dr_var (31, 1));
        state = tms ? EXIT1_DR : SHIFT_DR;
        break;
    case EXIT1_DR:
        state = tms ? UPDATE_DR : PAUSE_DR;
        break;
    case PAUSE_DR:
        state = tms ? EXIT2_DR : PAUSE_DR;
        break;
    case EXIT2_DR:
        state = tms ? UPDATE_DR : SHIFT_DR;
        break;
    case UPDATE_DR:
        state = tms ? SELECT_DR_SCAN : RUN_TEST_IDLE;
        if (ir_var == DMI) {
            //            dmi = (dr_var(33+DTM_ADDR_WIDTH,34), (sc_uint<32>)dmi_dat_i.read(), sc_uint<2>(0));
            dmi_op = 1;
        } else if (ir_var == DTMCS && dr_var[16]) {
            state = TEST_LOGIC_RESET;
        }
        break;
    case SELECT_IR_SCAN:
        state = tms ? TEST_LOGIC_RESET : CAPTURE_IR;
        break;
    case CAPTURE_IR:
        state = tms ? EXIT1_IR : SHIFT_IR;
        break;
    case SHIFT_IR:
        // tdo = ir_var[0];
        ir = (tdi.read (), ir_var (DTM_IR_WIDTH - 1, 1));
        state = tms ? EXIT1_IR : SHIFT_IR;
        break;
    case EXIT1_IR:
        state = tms ? UPDATE_IR : PAUSE_IR;
        break;
    case PAUSE_IR:
        state = tms ? EXIT2_IR : PAUSE_IR;
        break;
    case EXIT2_IR:
        state = tms ? UPDATE_IR : SHIFT_IR;
        break;
    case UPDATE_IR:
        state = tms ? SELECT_DR_SCAN : RUN_TEST_IDLE;
        break;
    default:
        PN_ERRORF (("JTAG_DTM Error: DTM is in unknown state %d!", (uint8_t)state.read ()));
        break;
    }

    // Reset...
    if (reset) {
        state = TEST_LOGIC_RESET;
        ir = IDCODE;
        dr = 0xdeadbeef;
        dmi = 0x0;
        dmi_length = 0;
        dmi_op = 0;
    }
}

void MDtm::OutputMethod () {
    sc_uint<34 + DTM_ADDR_WIDTH> dr_var;
    sc_uint<DTM_IR_WIDTH> ir_var;

    dr_var = dr.read ();
    ir_var = ir.read ();

    tdo = state.read () == SHIFT_IR ? ir_var[0] : dr_var[0];

    dmi_dat_o = dr_var (33, 2);
    dmi_adr = dr_var (33 + DTM_ADDR_WIDTH, 34);

    if (dmi_op) {
        dmi_rd = dr_var[0] & !dr_var[1];
        dmi_wr = dr_var[1] & !dr_var[0];
    } else {
        dmi_rd = 0;
        dmi_wr = 0;
    }
}

#endif // USE == 1
