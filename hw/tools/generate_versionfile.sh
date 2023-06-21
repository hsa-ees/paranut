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
#    This file creates a file containing the ParaNut version in different formats.
#    How the version is detremained may be read about in 
#    /doc-src/codingconvenstions.txt
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

# PARAMETERS
PARANUT=../..                        # root directory of installation
VERSION_FILE=version.env             # default target file
VERSION_FILE_TMP=version.env.new     # default temporary target file
UNKNOWN_VERSION=0x000001             # Version to be used if no generation possible
CFG_NUT_MIMPID=UNKNOWN_VERSION       # will be changed by tagToHex
                      

### FUNCTIONS

# Help
help(){
  echo "Call: generate_versionfile.sh -g"
  echo "Parameters:"
  echo "  -h           print this help"
  echo "  -t <FILE>    specify target file"
  echo "  -p <PARANUT> ParaNut root dir"
  echo "  -g           generate (if not given nothing happens)"
  exit 0
}

# Function for generating Version Hex Value for CFG_NUT_MIMPID (HW-Register)
tagToHex(){
  arg1=$1
  if [[ $arg1 =~ ^v([0-9]{1,3})\.([0-9]{1,3})-[a-z]*?-?([0-9]{1,3})-([0-9a-z]{6,12})(\*)?$ ]]; then
    SEDSTRING=$(echo ${arg1} | sed -E 's/v([0-9]{1,3})\.([0-9]{1,3})-[a-z]*?-?([0-9]{1,3})-([0-9a-z]{6,8})(\*)?/\1\n\2\n\3\n\5/g')
    MAJOR=$(echo "$SEDSTRING" | sed -n '1p')
    MINOR=$(echo "$SEDSTRING" | sed -n '2p')
    REVISION=$(echo "$SEDSTRING" | sed -n '3p')
    DIRTYSTAR=$(echo "$SEDSTRING" | sed -n '4p')

    DIRTY=0
    if [[ "$DIRTYSTAR" =~ "*" ]]; then
      DIRTY=1
    fi
    MAJOR=$(((MAJOR & 0xff) << 24))
    MINOR=$(((MINOR & 0xff) << 16))
    REVISION=$(((REVISION & 0x7fff) << 1))
    DIRTY=$((DIRTY & 1)) || DIRTY=1
    CFG_NUT_MIMPID=$((MAJOR | MINOR | REVISION | DIRTY))
  else
    echo >&2 "  git-tag-version not suitable for HW-revision"
    CFG_NUT_MIMPID=$UNKNOWN_VERSION  
  fi
    echo >&2 "  using $CFG_NUT_MIMPID for CFG_NUT_MIMPID"
}

# BEGIN MAIN EXECUTION 

# get command line flags
GENERATE=0
while getopts ht:p:g flag
do
    case "${flag}" in
        t) 
          VERSION_FILE=${OPTARG}
          VERSION_FILE_TMP=${OPTARG}".new";;
        h) 
          help;;
        p) 
          PARANUT=${OPTARG};;
        g)
          GENERATE=1;;
    esac
done

# check if -g is set
if [ $GENERATE == 0 ]; then
  help
fi


# Do the actual work
echo "Updating version file ..."

# if paranut root is git
  if [ -e $PARANUT/.git ]; then
    GITVERSION=$(git describe --tags --long --dirty='' --abbrev=8 2> /dev/null)
    RETVAL=$?
    GITVERSION=${GITVERSION}$([[ -n $(git status -s . -- ':!doc') ]] && echo '*')
    if [ $RETVAL -eq 0 ]; then
      echo "  Using git-tag-version: "$GITVERSION
      tagToHex "$GITVERSION"
      echo "# Git describe" > $VERSION_FILE_TMP;
      echo "PN_GITVERSION="$GITVERSION >> $VERSION_FILE_TMP;
      echo "# MIMPID Version Value" >> $VERSION_FILE_TMP;
      echo "CFG_NUT_MIMPID="$CFG_NUT_MIMPID >> $VERSION_FILE_TMP;
      diff -q $VERSION_FILE $VERSION_FILE_TMP > /dev/null 2>&1 || mv $VERSION_FILE_TMP $VERSION_FILE;
      rm -f $VERSION_FILE_TMP;
    fi
    exit 0
  fi

# else
  if [ -e $VERSION_FILE ]; then
    echo "  No git-tag-version, preserving old version: "$(grep PN_GITVERSION $VERSION_FILE | sed 's#^.*=##' )
  else
    echo "  No git-tag-version, using unknown version: 0.0.0*"
    echo "# Git describe" > $VERSION_FILE
    echo "PN_GITVERSION=0.0-0*" >> $VERSION_FILE
    echo "# MIMPID Version Value" >> $VERSION_FILE
    echo "CFG_NUT_MIMPID=0x000001" >> $VERSION_FILE
  fi
  exit 0
# END MAIN EXECUTION 
