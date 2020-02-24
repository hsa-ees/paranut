include ../../paranut/paranut-fdeps.mk
# Testbench file dependencies
$(ENTITY_NAME)_tb.o: types.o exu.o lsu.o ifu.o memu_lib.o tb_monitor.o 
rwports.o: paranut_config.o types.o memu_lib.o paranut_lib.o prog_mem.o

tb_monitor.o: paranut_config.o types.o
orbis32_disas.o: types.o text_io.o orbis32.o tb_monitor.o paranut_lib.o
