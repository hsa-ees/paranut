#File: core_portme.mak

# Parameters to genreate .bin
CROSS_COMPILE ?= riscv64-unknown-elf
OBJDUMP := $(CROSS_COMPILE)-objdump
OBJCOPY := $(CROSS_COMPILE)-objcopy


# Flag: OUTFLAG
#	Use this flag to define how to to get an executable (e.g -o)
OUTFLAG= -o
# Flag: CC
#	Use this flag to define compiler to use
CC = riscv64-unknown-elf-gcc

# Configuration options
CFG_CLKS_PER_SEC ?= 25000000
CFG_NUT_CPU_CORES_LD ?= 1
CFG_MARCH ?= rv32im

PTHREAD_LIB_DIR = ../pthreads/INSTALL
PTHREAD_LIB_INC_DIR = $(PTHREAD_LIB_DIR)/include
PTHREAD_LIB_LIB_DIR = $(PTHREAD_LIB_DIR)/lib
PN_LIB_DIR = ../libparanut/INSTALL
PN_LIB_LIB_DIR = $(PN_LIB_DIR)/lib
PN_LIB_INC_DIR = $(PN_LIB_DIR)/include
# Flag: CFLAGS
#	Use this flag to define compiler options. Note, you can add compiler options from the command line using XCFLAGS="other flags"
PTHREAD_CFLAGS = -lpnpthread -L$(PTHREAD_LIB_LIB_DIR) -I$(PTHREAD_LIB_INC_DIR) -lparanut -L$(PN_LIB_LIB_DIR) -I$(PN_LIB_INC_DIR) 
PORT_CFLAGS = -O3 -mcmodel=medany -static -march=$(CFG_MARCH) -mabi=ilp32 -nostartfiles -D_CLKS_PER_SEC=$(CFG_CLKS_PER_SEC) -DCFG_NUT_CPU_CORES_LD=$(CFG_NUT_CPU_CORES_LD) -I$(RISCV_COMMON_DIR) $(RISCV_COMMON_DIR)/startup.S $(RISCV_COMMON_DIR)/syscalls.c 
# FLAGS_STR = "$(PORT_CFLAGS) $(PTHREAD_CFLAGS) $(XCFLAGS) $(XLFLAGS) $(LFLAGS_END)"
FLAGS_STR = "$(PORT_CFLAGS) $(XCFLAGS) $(XLFLAGS) $(LFLAGS_END)"
# CFLAGS = $(PORT_CFLAGS) $(PTHREAD_CFLAGS) -I$(PORT_DIR) -I. -DFLAGS_STR=\"$(FLAGS_STR)\"
CFLAGS = $(PORT_CFLAGS) -I$(PORT_DIR) -I. -DFLAGS_STR=\"$(FLAGS_STR)\"
#Flag: LFLAGS_END
#	Define any libraries needed for linking or other flags that should come at the end of the link line (e.g. linker scripts). 
#	Note: On certain platforms, the default clock_gettime implementation is supported but requires linking of librt.
# LFLAGS_END += -T $(RISCV_COMMON_DIR)/paranut.ld -lpnpthread  -L$(PTHREAD_LIB_LIB_DIR) -I$(PTHREAD_LIB_INC_DIR) -lparanut -L$(PN_LIB_LIB_DIR) -I$(PN_LIB_INC_DIR)
 LFLAGS_END += -T $(RISCV_COMMON_DIR)/paranut.ld -L$(PN_LIB_LIB_DIR) -I$(PN_LIB_INC_DIR)  
# Flag: PORT_SRCS
# Port specific source files can be added here
PORT_SRCS = $(PORT_DIR)/core_portme.c
# Flag: LOAD
#	Define this flag if you need to load to a target, as in a cross compile environment.

# Flag: RUN
#	Define this flag if running does not consist of simple invocation of the binary.
#	In a cross compile environment, you need to define this.

#For flashing and using a tera term macro, you could use
#LOAD = flash ADDR 
#RUN =  ttpmacro coremark.ttl

#For copying to target and executing via SSH connection, you could use
#LOAD = scp $(OUTFILE)  user@target:~
#RUN = ssh user@target -c  

#For native compilation and execution
#LOAD = echo Loading done
LOAD = echo "Loading done"
RUN =  $(PN_SIM_BIN)

OEXT = .o
EXE = 

# Flag: SEPARATE_COMPILE
# Define if you need to separate compilation from link stage. 
# In this case, you also need to define below how to create an object file, and how to link.
ifdef SEPARATE_COMPILE

LD		= riscv32-unknown-elf-gcc
OBJOUT 	= -o
LFLAGS 	= 
OFLAG 	= -o
COUT 	= -c
# Flag: PORT_OBJS
# Port specific object files can be added here
PORT_OBJS = $(PORT_DIR)/core_portme$(OEXT)
PORT_CLEAN = *$(OEXT)

$(OPATH)%$(OEXT) : %.c
	$(CC) $(CFLAGS) $(XCFLAGS) $(COUT) $< $(OBJOUT) $@
	
endif

# Target: port_prebuild
# Generate any files that are needed before actual build starts.
# E.g. generate profile guidance files. Sample PGO generation for gcc enabled with PGO=1
#  - First, check if PGO was defined on the command line, if so, need to add -fprofile-use to compile line.
#  - Second, if PGO reference has not yet been generated, add a step to the prebuild that will build a profile-generate version and run it.
#  Note - Using REBUILD=1 
#
# Use make PGO=1 to invoke this sample processing.

ifdef PGO
 ifeq (,$(findstring $(PGO),gen))
  PGO_STAGE=build_pgo_gcc
  CFLAGS+=-fprofile-use
 endif
 PORT_CLEAN+=*.gcda *.gcno gmon.out
endif

.PHONY: port_prebuild
port_prebuild: $(PGO_STAGE)

.PHONY: build_pgo_gcc
build_pgo_gcc:
	$(MAKE) PGO=gen XCFLAGS="$(XCFLAGS) -fprofile-generate -DTOTAL_DATA_SIZE=1200" ITERATIONS=10 gen_pgo_data REBUILD=1
	
# Target: port_postbuild
# Generate any files that are needed after actual build end.
# E.g. change format to srec, bin, zip in order to be able to load into flash
.PHONY: port_postbuild
port_postbuild:

# Target: port_postrun
# 	Do platform specific after run stuff. 
#	E.g. reset the board, backup the logfiles etc.
.PHONY: port_postrun
port_postrun:

# Target: port_prerun
# 	Do platform specific after run stuff. 
#	E.g. reset the board, backup the logfiles etc.
.PHONY: port_prerun
port_prerun:

# Target: port_postload
# 	Do platform specific after load stuff. 
#	E.g. reset the reset power to the flash eraser
.PHONY: port_postload
port_postload:

# Target: port_preload
# 	Do platform specific before load stuff. 
#	E.g. reset the reset power to the flash eraser
.PHONY: port_preload
port_preload:

# FLAG: OPATH
# Path to the output folder. Default - current folder.
OPATH = ./
MKDIR = mkdir -p

# FLAG: PERL
# Define perl executable to calculate the geomean if running separate.
PERL=/usr/bin/perl

.PHONY: flash flash-bit
flash: bin
	pn-flash -c -p $(OUTNAME).bin $(PN_SYSTEM_HDF) $(PN_FIRMWARE_ELF)

flash-bit: bin
	pn-flash -c -b $(PN_SYSTEM_BIT) -p $(OUTNAME).bin $(PN_SYSTEM_HDF) $(PN_FIRMWARE_ELF)

.PHONY: bin
bin: $(OUTNAME).bin
$(OUTNAME).bin: link
	$(OBJCOPY) -S -O binary $(OUTNAME) $(OUTNAME)$@

