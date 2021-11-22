# Coremark Testbench
This Coremark Test is derived from https://github.com/eembc/coremark (v1.0).
Please see the Coremark github page for further usage information.

# Usage
The test can be run in simulation as well as on hardware. 
It is however recommended to run the test on hardware, 
since a Test in the simulation may take a lot of time

## Run in simulation
In the root of the paranut git run:
	´´´make build-sim´´´
In the coremark_v1.0 folder run:
	´´´make run´´´


## Run on Hardware
Follow the instructions in the root of the paranut repository on how to 
build the Hardware. Then in the /systems/refdesign directory run:
´´´
make clean-software
make software SOFTWARE_SRC=coremark_v1.0
make run
´´´