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

#ifndef REMOTE_BITBANG_H
#define REMOTE_BITBANG_H

#include <stdint.h>

#include "jtag_dtm.h"

class remote_bitbang_t
{
public:
  // Create a new server, listening for connections from localhost on the given
  // port.
  remote_bitbang_t(uint16_t port, jtag_dtm_t *tap);

  // Do a bit of work.
  void tick();

private:
  jtag_dtm_t *tap;

  int socket_fd;
  int client_fd;

  static const ssize_t buf_size = 64 * 1024;
  char recv_buf[buf_size];
  ssize_t recv_start, recv_end;

  // Check for a client connecting, and accept if there is one.
  void accept();
  // Execute any commands the client has for us.
  void execute_commands();
};

#endif
