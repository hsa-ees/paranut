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

#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#ifndef AF_INET
#include <sys/socket.h>
#endif
#ifndef INADDR_ANY
#include <netinet/in.h>
#endif

#include <algorithm>
#include <cassert>
#include <cstdio>

#include "remote_bitbang.h"
#include "base.h"

#if 0
#  define D(x) x
#else
#  define D(x)
#endif

/////////// remote_bitbang_t

remote_bitbang_t::remote_bitbang_t(uint16_t port, jtag_dtm_t *tap) :
  tap(tap),
  socket_fd(0),
  client_fd(0),
  recv_start(0),
  recv_end(0)
{
  socket_fd = socket(AF_INET, SOCK_STREAM, 0);
  if (socket_fd == -1) {
    PN_WARNINGF (("remote_bitbang failed to make socket: %s (%d)",
        strerror(errno), errno));
    abort();
  }

  fcntl(socket_fd, F_SETFL, O_NONBLOCK);
  int reuseaddr = 1;
  if (setsockopt(socket_fd, SOL_SOCKET, SO_REUSEADDR, &reuseaddr,
        sizeof(int)) == -1) {
    PN_WARNINGF (("remote_bitbang failed setsockopt: %s (%d)",
        strerror(errno), errno));
    abort();
  }

  struct sockaddr_in addr;
  memset(&addr, 0, sizeof(addr));
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = INADDR_ANY;
  addr.sin_port = htons(port);

  if (bind(socket_fd, (struct sockaddr *) &addr, sizeof(addr)) == -1) {
    PN_WARNINGF (("remote_bitbang failed to bind socket: %s (%d)",
        strerror(errno), errno));
    abort();
  }

  if (listen(socket_fd, 1) == -1) {
    PN_WARNINGF (("remote_bitbang failed to listen on socket: %s (%d)",
        strerror(errno), errno));
    abort();
  }

  socklen_t addrlen = sizeof(addr);
  if (getsockname(socket_fd, (struct sockaddr *) &addr, &addrlen) == -1) {
    PN_WARNINGF (("remote_bitbang getsockname failed: %s (%d)",
        strerror(errno), errno));
    abort();
  }

  PN_INFOF (("Listening for remote bitbang connection on port %d.",
      ntohs(addr.sin_port)));
  fflush(stdout);
}

void remote_bitbang_t::accept()
{
  client_fd = ::accept(socket_fd, NULL, NULL);
  if (client_fd == -1) {
    if (errno == EAGAIN) {
      // No client waiting to connect right now.
    } else {
      PN_WARNINGF (("remote_bitbang failed to accept on socket: %s (%d)", strerror(errno),
          errno));
      abort();
    }
  } else {
    fcntl(client_fd, F_SETFL, O_NONBLOCK);
  }
}

void remote_bitbang_t::tick()
{
  if (client_fd > 0) {
    execute_commands();
  } else {
    this->accept();
  }
}

void remote_bitbang_t::execute_commands()
{
  static char send_buf[buf_size];
  unsigned total_processed = 0;
  bool quit = false;
  bool in_rti = tap->state() == RUN_TEST_IDLE;
  bool entered_rti = false;
  while (1) {
    if (recv_start < recv_end) {
      unsigned send_offset = 0;
      while (recv_start < recv_end) {
        uint8_t command = recv_buf[recv_start];

        switch (command) {
          case 'B': /* PN_INFO ("*BLINK*"); */ break;
          case 'b': /* PN_INFO ("_______"); */ break;
          case 'r': tap->reset(); break;
          case '0': tap->set_pins(0, 0, 0); break;
          case '1': tap->set_pins(0, 0, 1); break;
          case '2': tap->set_pins(0, 1, 0); break;
          case '3': tap->set_pins(0, 1, 1); break;
          case '4': tap->set_pins(1, 0, 0); break;
          case '5': tap->set_pins(1, 0, 1); break;
          case '6': tap->set_pins(1, 1, 0); break;
          case '7': tap->set_pins(1, 1, 1); break;
          case 'R': send_buf[send_offset++] = tap->tdo() ? '1' : '0'; break;
          case 'Q': quit = true; break;
          default:
                    PN_WARNINGF (("remote_bitbang got unsupported command '%c'",
                        command));
        }
        recv_start++;
        total_processed++;
        if (!in_rti && tap->state() == RUN_TEST_IDLE) {
          entered_rti = true;
          break;
        }
        in_rti = false;
      }
      unsigned sent = 0;
      while (sent < send_offset) {
        ssize_t bytes = write(client_fd, send_buf + sent, send_offset);
        if (bytes == -1) {
          PN_WARNINGF (("remote_bitbang failed to write to socket: %s (%d)", strerror(errno), errno));
          abort();
        }
        sent += bytes;
      }
    }

    if (total_processed > buf_size || quit || entered_rti) {
      // Don't go forever, because that could starve the main simulation.
      break;
    }

    recv_start = 0;
    recv_end = read(client_fd, recv_buf, buf_size);

    if (recv_end == -1) {
      if (errno == EAGAIN) {
        break;
      } else {
        PN_WARNINGF (("remote_bitbang failed to read on socket: %s (%d)",
            strerror(errno), errno));
        abort();
      }
    }

    if (quit) {
      PN_INFO ("Remote Bitbang received 'Q'");
    }

    if (recv_end == 0 || quit) {
      // The remote disconnected.
      PN_WARNING ("Received nothing. Quitting.");
      close(client_fd);
      client_fd = 0;
      break;
    }
  }
}
