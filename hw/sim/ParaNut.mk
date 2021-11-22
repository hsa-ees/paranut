#  -----------------------------------------------------------------------------
#
#  This file is part of the ParaNut project.
#
#  Copyright (C) 2020 Gundolf Kiefer <gundolf.kiefer@hs-augsburg.de>
#      Hochschule Augsburg, University of Applied Sciences
#
#  --------------------- LICENSE -----------------------------------------------
#  Redistribution and use in source and binary forms, with or without modification,
#  are permitted provided that the following conditions are met:
#
#  1. Redistributions of source code must retain the above copyright notice, this
#     list of conditions and the following disclaimer.
#
#  2. Redistributions in binary form must reproduce the above copyright notice,
#     this list of conditions and the following disclaimer in the documentation and/or
#     other materials provided with the distribution.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
#  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
#  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
#  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
#  ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
#  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
#  ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
#  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
#  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#  -----------------------------------------------------------------------------


# Identify the ParaNut directory ...
PARANUT := $(dir $(lastword $(MAKEFILE_LIST)))..

# Compiler/linker flags ...
PARANUT_CFLAGS := -I$(PARANUT)/sysc -DSIMBUILD
PARANUT_LDFLAGS := -L$(PARANUT)/sysc -lparanutsim

PARANUT_DEPEND := $(PARANUT)/sysc/libparanutsim.a

# Helper targets to update or clean libraries and headers ...
.PHONY: paranut-update paranut-clean

paranut-update:
	$(MAKE) -C $(PARANUT)/sysc libparanutsim.a

.PHONY: paranut-clean
paranut-clean:
	$(MAKE) -C $(PARANUT)/sysc clean
