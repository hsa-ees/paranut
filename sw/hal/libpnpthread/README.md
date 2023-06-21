# PThreads for ParaNut
This module contains an implementation of a PThreads compatible API for the ParaNut.
## Preparation
First, please execute steps 1 through 4 of the README in this repo's root (if you want you may adopt your system's mode 2 cores during this). Assuming your console is in the repo root, then execute the following:
```
cd sw/libparanut
make install INSTALL_DIR=INSTALL
cd ../pthreads
make install
```
For a first example test have a look into subdirectory "pn_pthread_example". It contains a README with instructions on running it.

## Running on Hardware 
The generated binaries can be run on a hardware system by following the steps explained in the root-README.

## Example Programs
* A simple starting point is the example contained in the subdirectory pn_pthread_example. It also includes instructions on how to run.
* https://resources.oreilly.com/examples/9781565921153/blob/master/examples.tar.gz -> many programs, some of which make use of unimplemented features
# Documentation
Currently implemented functions are documented with doxygen.
To create the documentation run ```make doxygen``` inside the *sw/pthreads* folder. You can then open the documentation from *sw/pthreads/Documentation/html/index.html*
## Known working system:
Compiled with: riscv64-unknown-elf-gcc (SiFive GCC 8.3.0-2020.04.1) 8.3.0
Synthesized with: Xilinx Vivado 2017.2
# Known shortcomings
## missing functionalities:
* pthread_attr_setdetachstate
* pthread_attr_setinheritsched
* pthread_attr_setschedparam
* pthread_attr_setschedpolicy
* pthread_attr_setstacksize
* pthread_barrier_t
* pthread_barrier_init
* pthread_barrier_wait
* pthread_detach
* pthread_getspecific
* pthread_key_create
* pthread_key_t
* pthread_mutexattr_getprotocol
* pthread_mutexattr_setprioceiling
* pthread_mutexattr_setprotocol
* pthread_mutexattr_setpshared
* pthread_setcancelstate
* pthread_setcanceltype
* pthread_setspecific
* pthread_testcancel

## No scheduling/prioritizing options
Currently only the native fifo queueing scheme is supported


##### Contributers and Contact
Felix Wagner <felix.wagner1@hs-augsburg.de>
