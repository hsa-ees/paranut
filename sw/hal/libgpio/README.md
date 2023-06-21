# GPIO for ParaNut
This module contains a library for the use of the GPIO Module for the ParaNut
## Preparation
First, please execute steps 1 through 4 of the README in this repo's root (if you want you may adopt your system's mode 2 cores during this). Assuming your console is in the repo root, then execute the following:
```
cd sw/lib_gpio_api
make install
```
For a first example test have a look into subdirectory "libgpio_example". It contains a README with instructions on running it.

## Running on Hardware 
The generated binaries can be run on a hardware system by following the steps explained in the root-README.

## Example Programs
* A simple starting point is the example contained in the subdirectory libgpio_example. It also includes instructions on how to run.
# Documentation
Further Information can also be found in the README inside the subdirectory */libgpio_example*
Currently implemented functions are documented with doxygen.
To create the documentation run ```make doxygen``` inside the *sw/lib_gpio_api* folder. You can then open the documentation from *sw/lib_gpio_api/Documentation/html/index.html*




