# Testing pthread for ParaNut
## Preparation
Tests are intended to be executed in the simulation environment. So let's make shure all that is up to date. You are expected to have pulled to most recent commit from *pn_pthread_testing*
First, please execute steps 1 through 4 of the README in this repo's root (if you want you may adopt your systems mode 2 cores during this). If you are able to run *hello_newlib* you are ready to compile pthreads:
Assuming your console is in the repo root:
```
make install
cd sw/pthreads
make install
cd pn_pthread_example
make sim
```
Expected Output of the last statement (without simulation info):
```
(INFO): 0 s, paranutsystem.cpp:310:   Starting SystemC simulation...
numcores 4; cap2: 15
 0. Hello World from 2 !
 1. Hello World from 1 !
 2. Hello World from 2 !
 3. Hello World from 1 !
 8. Hello World from 3 !
 6. Hello World from 1 !
 4. Hello World from 2 !
 9. Hello World from 3 !
 5. Hello World from 2 !
 7. Hello World from 1 !
(INFO):      6465480 ns, paranutsystem.cpp:353:   CePU has reached HALT.
(INFO):      6465880 ns, paranutsystem.cpp:358:   SystemC Simulation completed.
```
## Testing on Hardware
If your simulation takes ages (mine often does). You can speed things up significantly by running it on real Hardware. For the setup copy the files *firmware.elf*, *system.bit* and *system.hdf* into the pn_test folder. run ```source <Vivado-dir>/settings64.sh``` and ```source <paranut-git>/settings.sh```. Then run ```make flash-bit```. Any prints in your software should now be visible on console. (This system does only have mode 2 CoPU, in the demo program, any threads assigned to other cores are never executed!)
After the initial ```make flash-bit``` you may use ```make flash``` for subsequent flashes.

## More Example Programs
* https://resources.oreilly.com/examples/9781565921153/blob/master/examples.tar.gz -> many programs, some of which make use of unimplemented features
* https://www.bogotobogo.com/cplusplus/multithreading_pthread.php -> tutorial with many example snippets
*  https://www.uni-muenster.de/AMM/num/Vorlesungen/NumForHPC_WS1213/Material/pthreads-article.pdf -> pdf containing 4 examples

