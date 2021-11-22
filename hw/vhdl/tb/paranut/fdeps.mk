include ../../paranut/paranut-fdeps.mk
include ../../counter/counter-fdeps.mk
# testbench file dependencies
$(ENTITY_NAME)_tb.o: types.o paranut.o peripherals.o prog_mem.o tb_monitor.o 

peripherals.o: types.o
wb_memory.o: types.o paranut_lib.o text_io.o txt_util.o
prog_mem.o: types.o
wb_uart.o: types.o paranut_lib.o text_io.o txt_util.o
wb_counter_wrapper.o: types.o paranut_lib.o counter_pkg.o

text_io.o:
tb_monitor.o: types.o
orbis32_disas.o: types.o text_io.o orbis32.o tb_monitor.o paranut_lib.o
uart_transmitter.o: tb_monitor.o
