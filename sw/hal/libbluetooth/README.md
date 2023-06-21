# Bluetooth for ParaNut
This module contains a library for the use of the Bluetooth BM70 Module with the ParaNut
## Preparation
First, please execute steps 1 through 4 of the README in this repo's root (if you want you may adopt your system's mode 2 cores during this). Assuming your console is in the repo root, then execute the following:
```
cd sw/lib_bluetooth_api
make install
```
For a first example test have a look into subdirectory "libbluetooth_example". It contains a README with instructions on running it.

## Running on Hardware 
The generated binaries can be run on a hardware system by following the steps explained in the root-README.

## Example Programs
* A simple starting point is the example contained in the subdirectory libbluetooth_example. It also includes instructions on how to run.
# Documentation
Further Information can also be found in the README inside the subdirectory */libbluetooth_example*
Currently implemented functions are documented with doxygen.
To create the documentation run ```make doxygen``` inside the *sw/lib_bluetooth_api* folder. You can then open the documentation from *sw/lib_bluetooth_apitrue/Documentation/html/index.html*




