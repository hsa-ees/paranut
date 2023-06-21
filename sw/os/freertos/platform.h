#pragma once

// @todo: das wurde in die paranut_uart.h ausgelagert, diese file kann weg
#define UART_BASE		0xE0001000
#define UART_SR (volatile char*)(UART_BASE+0x0000002C)
#define UART_FIFO (volatile char*)(UART_BASE + 0x00000030)
#define UART_TXFULL 	0x00000010
#define UART_RXEMPTY 	0x00000002