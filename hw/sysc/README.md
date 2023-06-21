# SYSTEMC DIRECTORY

This directory contains hardware modules for the Paranut.

For simulation, each module is compiled into its own library and then linked 
with the simulator binary contained in one of the /systems directories.
High Level Synthesis can either be done on a module level, creating a single .v/.sv 
containing all submodules of the module. Or it may be done from the system's 
sysc directory to create a single file describing the whole ParaNut System.