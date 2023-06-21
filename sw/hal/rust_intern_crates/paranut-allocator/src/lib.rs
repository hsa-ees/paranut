/*************************************************************************

  This file is part of the ParaNut project.

  Copyright (C)  2023 Dario Bartussek <dario.bartussek@hs-augsburg.de> & Abdurrahman Celep <abdurrahman.celep1@hs-augsburg.de>
      Hochschule Augsburg, University of Applied Sciences

  Description:
     The Rust Allocator for Rust.

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
#![allow(non_camel_case_types)]
#![allow(unused_variables)]
#![allow(unused_imports)]

extern crate linked_list_allocator;

extern crate alloc;
use linked_list_allocator::Heap;
use alloc::alloc::GlobalAlloc;
use alloc::alloc::Layout;
use core::ptr::NonNull;


extern "C" {
    static mut __heap_start: u8; 
    static mut __heap_end: u8;
}

#[global_allocator]
static HEAP_ALLOCATOR: LockedHeap = LockedHeap::empty();
pub struct LockedHeap(Spinlock<Heap>);

impl LockedHeap {
    pub const fn empty() -> LockedHeap {
        LockedHeap(Spinlock::new(Heap::empty()))
    }
}

impl Deref for LockedHeap {
    type Target = Spinlock<Heap>;

    fn deref(&self) -> &Spinlock<Heap> {
        &self.0
    }
}

unsafe impl GlobalAlloc for LockedHeap {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        self.0
            .lock()
            .allocate_first_fit(layout)
            .ok()
            .map_or(core::ptr::null_mut(), |allocation| allocation.as_ptr())
    }

    unsafe fn dealloc(&self, ptr: *mut u8, layout: Layout) {
        self.0
            .lock()
            .deallocate(NonNull::new_unchecked(ptr), layout)
    }
}


use core::arch::asm;
use core::cell::UnsafeCell;
use core::ops::{Deref, DerefMut};
use core::panic::PanicInfo;

/// https://five-embeddev.com/riscv-isa-manual/latest/a.html
pub unsafe fn cmp_exchange(cell: *const u32, current: u32, new: u32) -> Result<u32, u32> {
    let result: u32;
    let value: u32;

    asm!(
        "1:", // cas
        "lr.w {value}, ({cell})",
        "bne {value}, {current}, 2f",
        "sc.w {value}, {new}, ({cell})",
        "bnez {value}, 1b",
        "li {result}, 0",
        "j 3f",

        "2:", // fail
        "li {result}, 1",
        "j 3f",

        "3:", // end

        cell = in(reg) cell,
        current = in(reg) current,
        new = in(reg) new,
        result = out(reg) result,
        value = out(reg) value,
    );

    if result != 0 {
        Err(value)
    } else {
        Ok(new)
    }
}

pub struct Spinlock<T> {
    lock: u32,
    value: UnsafeCell<T>,
}
pub struct SpinlockGuard<'lt, T>(&'lt Spinlock<T>);

unsafe impl<T> Send for Spinlock<T> {}
unsafe impl<T> Sync for Spinlock<T> {}

impl<T> Spinlock<T> {
    pub const fn new(value: T) -> Self {
        Self {
            lock: 0,
            value: UnsafeCell::new(value),
        }
    }

    unsafe fn lock_inner(&self) {
        while cmp_exchange(&self.lock as *const _, 0, 1).is_err() {}
    }
    unsafe fn unlock_inner(&self) {
        assert!(cmp_exchange(&self.lock as *const _, 1, 0).is_ok());
    }

    pub fn lock(&self) -> SpinlockGuard<T> {
        unsafe {
            self.lock_inner();
            SpinlockGuard(self)
        }
    }

    pub fn inner(&mut self) -> &mut T{
        self.value.get_mut()
    }
}

impl<'lt, T> Deref for SpinlockGuard<'lt, T> {
    type Target = T;

    fn deref(&self) -> &Self::Target {
        unsafe {
            &*self.0.value.get()
        }
    }
}


impl<'lt, T> DerefMut for SpinlockGuard<'lt, T> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        unsafe {
            &mut *self.0.value.get()
        }
    }
}

impl<'lt, T> Drop for SpinlockGuard<'lt, T> {
    fn drop(&mut self) {
        unsafe {self.0.unlock_inner()};
    }
}


pub fn init_heap(){
    unsafe {
        let heap_start = &mut __heap_start as *mut u8;
        let heap_size = &__heap_end as *const u8 as usize - &__heap_start as *const u8 as usize;

        HEAP_ALLOCATOR.lock().init(heap_start, heap_size);
    }
}
