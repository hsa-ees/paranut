include ../../paranut/paranut-fdeps.mk
# Testbench file dependencies
$(ENTITY_NAME)_tb.o: paranut_config.o types.o paranut_lib.o memu.o memu_lib.o peripherals.o tb_monitor.o text_io.o
peripherals.o: types.o
wb_memory.o: paranut_config.o types.o paranut_lib.o text_io.o txt_util.o
tb_monitor.o: paranut_config.o types.o
