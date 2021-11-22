#!/bin/bash
#  -----------------------------------------------------------------------------
#
#  This file is part of the ParaNut project.
#
#  Copyright (C) 2020-2021 Michael Schaeferling <michael.schaeferling@hs-augsburg.de>
#                          Gundolf Kiefer <gundolf.kiefer@hs-augsburg.de>
#      Hochschule Augsburg, University of Applied Sciences
#
#  Description:
#    This file creates a hex value from git describe for use in MIMPID CSR
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

UNKNOWN_VERSION=0x000001
GITVERSION=$(git describe --tags --long --dirty='*')
if [[ $GITVERSION =~ v[0-9]*\.+[0-9]*-[0-9]*-([a-z0-9]*)\*? ]]; then
  GITVERSION=${GITVERSION#*v} 
  SUBSTR1=${GITVERSION#*.}
  MAJOR=${GITVERSION%."$SUBSTR1"}
  SUBSTR2=${SUBSTR1#*-}
  MINOR=${SUBSTR1%-"$SUBSTR2"}

  SUBSTR3=${SUBSTR2#*-}
  REVISION=${SUBSTR2%-"$SUBSTR3"}

  DIRTY=0
  if [[ $SUBSTR3 =~ "*" ]]; then
    DIRTY=1
  fi
  MAJOR=$(((MAJOR & 0xff) << 24))
  MINOR=$(((MINOR & 0xff) << 16))
  REVISION=$(((REVISION & 0x7fff) << 1))
  DIRTY=$((DIRTY & 1)) || DIRTY=1
  VERSION=$((MAJOR | MINOR | REVISION | DIRTY))
else
  VERSION=$UNKNOWN_VERSION  
fi
  echo $( printf "0x%x" $VERSION )