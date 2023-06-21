/*************************************************************************

  This file is part of the ParaNut project.

  Copyright (C)  2023 Abdurrahman Celep <abdurrahman.celep1@hs-augsburg.de>
      Hochschule Augsburg, University of Applied Sciences

  Description:
     The wrapper for libparanut.

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

#![allow(non_upper_case_globals)]
#![allow(non_camel_case_types)]
#![allow(non_snake_case)]
#![allow(improper_ctypes)]
#![no_std]
#![no_main]

include!(concat!(env!("OUT_DIR"), "/bindings.rs"));

pub fn parallel<F>(cpus: usize, f: F)
where
    F: Fn(usize) + Send + Sync,
{
    unsafe {
        let cpuid = pn_begin_threaded(cpus as i32) as usize;
        f(cpuid);
        pn_end_threaded();
    }
}

pub fn numcores () -> usize{
    unsafe {
        pn_numcores() as usize
    }
}

pub fn spinlock<F>(mut lock: _pn_spinlock, f: F)
where
    F: Fn() + Send + Sync,
{
    unsafe {
        pn_spinlock_lock(&mut lock);
        f();
        pn_spinlock_unlock(&mut lock);
    }
}

