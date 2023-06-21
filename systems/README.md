# Systems Directory
## What is a ParaNut System
A ParaNut system describes a specific ParaNut configuration. In this it defines 
hardware options like the number and type of co-processors or extension modules.
(For a full list of the config-options see [config.mk](../config.mk) or the 
[ParaNut manual](../doc/paranut-manual.pdf))

### A Note on Software
Each Software in the ../sw directory contains a reference to the System it is 
meant to be executed on (defined by PN_SYSTEM in the makefile). For some software 
this is crucial (e.g. linux), other software is relatively independent of the 
hardware configuration (e.g. hello_newlib)

## Structure
This directory contains multiple predefined ParaNut Systems:
* refdesign (general purpose configuration, the baseline)
* linux (providing priviledged modes and pagining for linux support)
* bluetooth (provides uart and gpios for communication with a bluetooth module)

