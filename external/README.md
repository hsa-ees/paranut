# Directory Info
External repositories that are relevant for both
software and hardware reside here.

## Subdirectories
The files located in the subdirectory "patches" are
used to alter the git submodules if necessary.

## Dependent Applications
Currently, three ParaNut applications depend on external 
repositories.
### SDL Demo
Information on this application may be found in the README.md contained in
*sw/sdl_demo*.
 
### Refdesign Vears
This extension of the basic hardware refdesign adds VGA output capabilities
to the basic system configuration. It is located in *systems/refdesign_vears*.
# Important
Do not delete this folder,
it's being used by SDL Demo and refdesign_vears
for checking out asterics for it's VEARS module.

### Linux kernel
The Linux kernel application is found at *sw/linux* and places different external software
repositories in this folder: The Linux kernel itself, OpenSBI and a compiler toolchain.
