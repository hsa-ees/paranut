/*************************************************************************

  This file is part of the ParaNut project.

  Copyright (C)  2023 Abdurrahman Celep <abdurrahman.celep1@hs-augsburg.de>
      Hochschule Augsburg, University of Applied Sciences

  Description:
     Demo application showcasing the print output.´
     This application requires the libparanut crate´.

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

#![no_std]
#![no_main]
#![allow(non_camel_case_types)]
#![allow(unused_variables)]

use core::panic::PanicInfo;
use libc_print::std_name::println;
use bindgen_paranut_sys as paranut;

#[no_mangle]
pub extern "C" fn main() -> i32 {

    println!("\nHello, Paranut!");
    println!("\nThe code was programmed with rust.");
    println!("\nIt also uses C libraries like libparanut.");
    println!("\nThe code was programmed with rust.");
    println!("\nThe code was programmed with rust.");
    println!("\nAt the moment {} CPUs are active.", paranut::numcores());

    for index in 0..9 {
        println!("\nthe Cargo Environment");
    }

    // Exit with a return status of 0.
    return 0;
}

#[panic_handler]
fn panic(_panic: &PanicInfo<'_>) -> ! {
    loop {}
}
