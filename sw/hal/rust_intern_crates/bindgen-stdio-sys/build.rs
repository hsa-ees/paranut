/*************************************************************************

  This file is part of the ParaNut project.

  Copyright (C)  2023 Abdurrahman Celep <abdurrahman.celep1@hs-augsburg.de>
      Hochschule Augsburg, University of Applied Sciences

  Description:
     Demo application showcasing the CoPU Linked Mode (Mode 1).
     This application requires the libparanut.

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

extern crate bindgen;

use std::env;
use std::ffi::OsString;
use std::path::PathBuf;

fn main() {
    println!("cargo:rustc-cfg=riscv");
    println!("cargo:rustc-cfg=riscv32");

    let key = "RISCV_HOME";
    let riscv_home;
    match env::var_os(key) {
        Some(val) => riscv_home = val.into_string().unwrap(),
        None => {
            riscv_home = OsString::from("{key} is not defiend in the enviroment.")
                .into_string()
                .unwrap()
        }
    }

    let bindings = bindgen::Builder::default()
        .ctypes_prefix("core::ffi")
        .use_core()
        .clang_arg(String::from("--sysroot=") + riscv_home.as_str() + "/riscv64-unknown-elf/")
        .clang_arg(String::from("--prefix=") + riscv_home.as_str())
        .clang_arg("--target=riscv32-unknown-none-elf")
        .header("wrapper.h")
        .generate()
        .expect("Unable to generate bindings");

    let out_path = PathBuf::from(env::var("OUT_DIR").unwrap());
    bindings
        .write_to_file(out_path.join("bindings.rs"))
        .expect("Couldn't write bindings!");
}
