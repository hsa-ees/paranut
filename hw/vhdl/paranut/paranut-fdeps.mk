# List all dependent libraries and entity implementations for component instantiations
paranut_lib.o:
paranut.o: memu_lib.o types.o
mparanut.o: paranut_config.o paranut.o types.o memu.o memu_lib.o text_io.o txt_util.o exu.o ifu.o lsu.o dbg.o mlsu_simple.o

ifu.o: paranut_config.o types.o memu_lib.o histogram.o
mifu.o: paranut_config.o ifu.o types.o memu_lib.o paranut_lib.o


lsu.o: paranut_config.o types.o memu_lib.o
mlsu.o: paranut_config.o lsu.o types.o memu_lib.o paranut_lib.o
mlsu_simple.o: paranut_config.o lsu.o types.o memu_lib.o paranut_lib.o

mem_inferred.o: paranut_lib.o types.o


exu.o: types.o ifu.o lsu.o histogram.o dbg.o intc.o
mexu.o: paranut_config.o exu.o types.o ifu.o lsu.o intc.o paranut_lib.o text_io.o txt_util.o tb_monitor.o mhistogram.o histogram.o

dbg.o: types.o  paranut_config.o memu_lib.o 
mdbg.o: paranut_config.o memu_lib.o paranut_lib.o text_io.o txt_util.o dbg.o

intc.o: types.o  paranut_config.o 
mintc.o: paranut_config.o paranut_lib.o intc.o

lfsr.o: types.o
memu_lib.o: paranut_config.o types.o paranut_lib.o histogram.o
memu.o: memu_lib.o histogram.o
mmemu.o: memu.o paranut_config.o types.o memu_lib.o paranut_lib.o text_io.o txt_util.o

mtagram.o: types.o paranut_config.o memu_lib.o mem_tech.o paranut_lib.o lfsr.o
mbankram.o: paranut_config.o types.o memu_lib.o mem_tech.o
mbusif.o: paranut_config.o types.o memu_lib.o paranut_lib.o histogram.o
mreadport.o: paranut_config.o types.o memu_lib.o paranut_lib.o histogram.o
mwriteport.o: paranut_config.o types.o memu_lib.o paranut_lib.o histogram.o
marbiter.o: paranut_config.o types.o memu_lib.o paranut_lib.o lfsr.o

histogram.o: types.o
mhistogram.o: types.o histogram.o mem_tech.o
