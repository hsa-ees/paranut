/*************************************************************************

  This file is part of the ParaNut project.

  Copyright (C)  2023 Abdurrahman Celep <abdurrahman.celep1@hs-augsburg.de>
      Hochschule Augsburg, University of Applied Sciences

  Description:
     Demo application showcasing the CoPU Parallel Mode (Mode 2).
     This application requires the libparanut crate.

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
#![allow(unused_assignments)]
#[allow(non_snake_case)]
#[allow(non_upper_case_globals)]


use bindgen_paranut_sys as paranut;
use libc_print::std_name::println;
use paranut_allocator as allocator;
use paranut_allocator::Spinlock;

extern crate alloc;
use alloc::string::{String, ToString};
use alloc::vec;
use alloc::vec::Vec;
use alloc::fmt::write;

extern crate encoding;

use core::panic::PanicInfo;
use core::ffi::{c_int, c_char};

use core::arch::asm;
/* TYPEDEFS */
#[derive(Clone)]
pub struct histogram_t {
    data: Vec<u32>,
    data_length: usize,
    hist: Vec<u32>,
    bitmask: u32,
}

pub const BITMASK: u32 = 0xFF;
pub const DATA_SIZE: u32 = 3000;

static mut GLOBAL_VAL: u32 = 0;

pub fn parallel_func(cpuid: u32) -> String {
    let mut val: u32 = 0;
    let mut s = String::new();

    for i in 0..DATA_SIZE {
        val += i;
        val *= 3;
        val /= 5;
        val <<= 2;
        val >>= 5;
    }

    val += cpuid;

    write(
        &mut s,
        format_args!(
            "Result of CPU #{} running thread #{}: {}",
            cpuid, cpuid, val
        ),
    )
    .unwrap();

    s
}

pub fn atomic_increment(adr: *const u32, increment_val: u32) -> u32 {
    let mut lr_val: u32;

    unsafe{
        asm!("mv {lr_val}, a4", lr_val = out(reg) lr_val);
    }

    loop {
        unsafe {
            asm! ("lr.w {lr_val}, ({adr})",
            "add {lr_val}, {lr_val}, {increment_val}",
            "sc.w {lr_val}, {lr_val}, ({adr})",  
                lr_val = inout(reg) lr_val,
                adr = in(reg) adr,
                increment_val = in(reg) increment_val);
        }
        if lr_val == 0 {
            break;
        }
    }

    0
}

pub fn histogram_fuc(arg: *mut histogram_t) {
    let result = arg;
    let mut shifts = 0;
    
    unsafe {
        let bitmask = (*result).bitmask;
        let histogram = &mut (*result).hist;
        let data = &mut (*result).data;
        let data_length = (*result).data_length;

        // Determine number of right shifts for given bitmask
        while !(((bitmask >> shifts) & 1) != 0)  {
            shifts += 1;
        }
        if shifts > 31 {
            return; // ERROR
        }
        
        //Calculate histogram
        let mut val = 0;
        
        data.iter().take(data_length).all(|data_value|{
            val = ((data_value & bitmask) >> shifts) - 1 ;

            if histogram .len() < ((val + 1)as usize){
                histogram .push(0);   
            }

            histogram[val as usize] += 1;  
            true
        });
    }    
}

pub fn atomic_histogram_func(arg: *mut histogram_t) {
    let result = arg;
    let mut shifts = 0;
    
    unsafe {
        let bitmask = (*result).bitmask;
        let histogram = &mut (*result).hist;
        let data = &mut (*result).data;
        let data_length = (*result).data_length; 

        // Determine number of right shifts for given bitmask
        while !(((bitmask >> shifts) & 1) != 0)  {
            shifts += 1;
        }
        if shifts > 31 {
            return; // ERROR
        }
        
        //Calculate histogram
        let mut val = 0;
        
        data.iter().take(data_length).all(|data_value|{
            val = ((data_value & bitmask) >> shifts) - 1 ;

            if histogram .len() < ((val + 1)as usize){
                histogram .push(0);   
            }

            atomic_increment(&mut (histogram[val as usize]), 1);   
            true
        });
    }    
}
   
extern "C" {
    fn printf(format: *const c_char,  ...) -> c_int;
}

#[no_mangle]
pub extern "C" fn main() -> c_int {
    
    allocator::init_heap();

    unsafe {
        let cpus = encoding::read_csr!("0xCD0");
        let mut start: u32;
        let mut end: u32;
        let mut seq_time: u32;

        let _cpuid: u32;
        let mut block_size = DATA_SIZE as usize / cpus;        
        let mut thread_results = Spinlock::new(vec!["".to_string(); cpus]);

        let clks_per_msec: usize = encoding::read_csr!(0xFC6)/1000;

        let mut histogram_spin = Spinlock::new(histogram_t {data: vec![0], data_length: 0, hist: vec![0], bitmask: 0});
        let mut histogram = vec![histogram_t {data: vec![0], data_length: 0, hist: vec![0], bitmask: 0}; cpus];
        let mut histogram1: Vec<u32>  = Vec::new();
        let mut histogram2: Vec<u32> = Vec::new();
        let mut data: Vec<u32> = Vec::new();

        //Init
        paranut::pn_exception_init();
        paranut::pn_cache_init();

        println!("\nWelcome to the ParaNut mode 2 demo\n----------------------------------\n");
        println!("\nCurrent system has {} CPUs\n", cpus);
     
        // ------------------------------------------------------------------------------
        // Test independent parallel execution (no A-Extension required)
        //			
        // ------------------------------------------------------------------------------
        println!("\nTest independent task on all CPUs\n----------------------------------\n");
 
        paranut::parallel(cpus, |_cpuid| {
            let result = parallel_func(_cpuid as u32);
            let mut guard = thread_results.lock();  
            guard[_cpuid] = result; 
        });

        for value in thread_results.inner() {
            println!("{}\n", value);
        }
 
    
        // ------------------------------------------------------------------------------
        // Test A-Extension:
        //			High contention on single address
        // ------------------------------------------------------------------------------
        println!("\nTest A-Extension: High contention on single address\n----------------------------------\n");
        // Get base time for simple add operation on CePU without synchronization
        // ------------------------------------------------------------------------------
        println!("Get base time for simple add operation on CePU without synchronization...");
        GLOBAL_VAL = 0;
        start = encoding::read_csr!("0xB00") as u32;

        for _index in 0..DATA_SIZE {
            GLOBAL_VAL+=1;
        }

        end = encoding::read_csr!("0xB00") as u32;
        seq_time = end - start;
        println!("Add:\t\t {} |\t {} ms \n", seq_time, seq_time as f64/ clks_per_msec as f64);

        // Check Value
        if GLOBAL_VAL != DATA_SIZE {
            println!("ERROR: Global value = {} != {}", GLOBAL_VAL, DATA_SIZE);
        }
    
        // Synchronized add on all CPUs
        // ------------------------------------------------------------------------------
        println!("Synchronized add on all CPUS...");
        GLOBAL_VAL = 0;
        let global_val_g = Spinlock::new(0 as u32);
        start = encoding::read_csr!("0xB00") as u32;

        paranut::parallel(cpus, |_cpuid| {
            for _index in 0..(DATA_SIZE/(cpus as u32)) {
                let mut guard = global_val_g.lock();
                *guard = GLOBAL_VAL;
                GLOBAL_VAL += 1; 
            }
        });

        end = encoding::read_csr!("0xB00") as u32;
        seq_time = end - start;
        println!("Sync add:\t\t {} |\t {} ms \n", seq_time, seq_time as f64/ clks_per_msec as f64);

        // Check Value
        if GLOBAL_VAL != DATA_SIZE {
            println!("ERROR: Global value = {} != {}", GLOBAL_VAL, DATA_SIZE);
        }
        
        
        println!("Atomic add on all CPUS...");
        GLOBAL_VAL = 0;
        start = encoding::read_csr!("0xB00") as u32;
        
        paranut::parallel(cpus, |_cpuid| {
            for _index in 0..(DATA_SIZE/(cpus as u32)) {
                atomic_increment(&mut GLOBAL_VAL, 1);                
            }
        });

        end = encoding::read_csr!("0xB00") as u32;
        seq_time = end - start;
        println!("Atomic add:\t\t {} |\t {} ms \n",  seq_time, seq_time as f64/ clks_per_msec as f64);

        // Check Value
        if GLOBAL_VAL != DATA_SIZE {
            println!("ERROR: Global value = {} != {}", GLOBAL_VAL, DATA_SIZE);
        } 
     
 
        // ------------------------------------------------------------------------------
        // Test A-Extension: 
        //			Histogram test (contention spread accross different addresses)
        // ------------------------------------------------------------------------------
        println!("\nHistogram test (contention spread accross different addresses)\n----------------------------------\n\n");
        // Initialize data:
        // ------------------------------------------------------------------------------
        println!("Initializing data for Histogram test...\n");

        for _index in 0..(DATA_SIZE/4) {
            data.push(1 as u32);
            data.push(2 as u32);
            data.push(3 as u32);
            data.push(4 as u32);
        }

        // Get base time for single threaded histogram function without synchronization
        // ------------------------------------------------------------------------------
        println!("\nGet base time for single threaded histogram function without synchronization...\n");
        histogram[0].data = data.clone();
        histogram[0].data_length = DATA_SIZE as usize;
        histogram[0].bitmask = 0xFF; 

        start = encoding::read_csr!("0xB00") as u32;
        histogram_fuc(&mut histogram[0]);
        end = encoding::read_csr!("0xB00") as u32;
        seq_time = end - start;

        histogram1 = (histogram[0].hist).to_vec();

        println!("Histogram:\t\t {} |\t {} ms \n",  seq_time, seq_time as f64/ clks_per_msec as f64);
        
        // Synchronized histogram on all CPUs (repeat for all possible block sizes)
        // ------------------------------------------------------------------------------
        for cur_cpus in (2..(cpus + 1)).rev(){
            
            // Determine block size
            block_size = DATA_SIZE as usize / cur_cpus;

            // Run test on x CPUs
            println!("\nParallel histogram test ({} blocks of size: {})...\n", cur_cpus, block_size);
    
            start = encoding::read_csr!("0xB00") as u32;
            
            for index in 0..histogram_spin.inner().hist.len() {
                histogram_spin.inner().hist[index] = 0;
            }

            //data
            let mut data_temp = data.clone();
            data_temp.rotate_right(1);

            fn map(vector: Vec<u32>, value: usize) -> Vec<u32> {
                let mut a = vector.clone();
                a.rotate_right(value.try_into().unwrap());
                a
            }
            paranut::parallel(cur_cpus, |_cpuid| {
                let mut guard = histogram_spin.lock();
                
                guard.data = map(data.clone(), block_size*_cpuid);
                guard.data_length = block_size;
                guard.bitmask = 0xFF;

                atomic_histogram_func(&mut *guard);
            });

            histogram2 = histogram_spin.inner().hist.to_vec();
                    
            end = encoding::read_csr!("0xB00") as u32;
            seq_time = end - start;
            println!("Parallel histogram:\t\t {} |\t {} ms \n",  seq_time, seq_time as f64/ clks_per_msec as f64);
            
            if histogram1 != histogram2 {
                println!("ERROR: Values of histogram1 and histogram2 differ: {:?} != {:?} \n", histogram1,histogram2);
            }
        }

    printf(
            b"\n\nEnd of ParaNut mode 2 demo\n----------------------------------\n\n\0"
            as *const u8 as *const i8,
    )
    

    }
}

#[panic_handler]
fn panic(_panic: &PanicInfo<'_>) -> ! {
    loop {}
}
