/*
 * FreeRTOS+IO V1.0.1 (C) 2012 Real Time Engineers ltd.
 *
 * FreeRTOS+IO is an add-on component to FreeRTOS.  It is not, in itself, part 
 * of the FreeRTOS kernel.  FreeRTOS+IO is licensed separately from FreeRTOS, 
 * and uses a different license to FreeRTOS.  FreeRTOS+IO uses a dual license
 * model, information on which is provided below:
 *
 * - Open source licensing -
 * FreeRTOS+IO is a free download and may be used, modified and distributed
 * without charge provided the user adheres to version two of the GNU General
 * Public license (GPL) and does not remove the copyright notice or this text.
 * The GPL V2 text is available on the gnu.org web site, and on the following
 * URL: http://www.FreeRTOS.org/gpl-2.0.txt
 *
 * - Commercial licensing -
 * Businesses and individuals who wish to incorporate FreeRTOS+IO into
 * proprietary software for redistribution in any form must first obtain a low
 * cost commercial license - and in-so-doing support the maintenance, support
 * and further development of the FreeRTOS+IO product.  Commercial licenses can
 * be obtained from http://shop.freertos.org and do not require any source files
 * to be changed.
 *
 * FreeRTOS+IO is distributed in the hope that it will be useful.  You cannot
 * use FreeRTOS+IO unless you agree that you use the software 'as is'.
 * FreeRTOS+IO is provided WITHOUT ANY WARRANTY; without even the implied
 * warranties of NON-INFRINGEMENT, MERCHANTABILITY or FITNESS FOR A PARTICULAR
 * PURPOSE. Real Time Engineers Ltd. disclaims all conditions and terms, be they
 * implied, expressed, or statutory.
 *
 * 1 tab == 4 spaces!
 *
 * http://www.FreeRTOS.org
 * http://www.FreeRTOS.org/FreeRTOS-Plus
 *
 */

/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "portmacro.h"

/* IO library includes. */
#include "FreeRTOS_IO.h"
#include "IOUtils_Common.h"
#include "FreeRTOS_uart.h"
//#include "platform.h"
//#include "paranut_uart.h"

/* The bits in the FIFOLVL register that represent the Tx Fifo level. */
#define uartTX_FIFO_LEVEL_MASK		( 0xf00UL )

/* The TEMT bit in the line status register. */
#define uartTX_BUSY_MASK			( 1UL << 6UL )

#define boardNUM_UARTS 2

/*------------------ Paranut functions to access memory bus ----------------------*/
static void paranut_Out8(uint32_t addr, uint8_t value) {
	register uint8_t *store_ptr asm ("x6") = (uint8_t*)addr;
	*store_ptr = value;
	asm volatile (".word (0x300B | ((6) << 15))\n");
}

static void paranut_Out32(uint32_t addr, uint32_t value) {
	register uint32_t *store_ptr asm ("x6") = (uint32_t*)addr;
	*store_ptr = value;
	asm volatile (".word (0x300B | ((6) << 15))\n");
}

static uint32_t paranut_In32(uint32_t addr) {
	register uint32_t *store_ptr asm ("x6") = (uint32_t*)addr;
	asm volatile (".word (0x100B | ((6) << 15))\n");
	return *store_ptr;
}

static uint8_t paranut_In8(uint32_t addr) {
	register uint8_t *store_ptr asm ("x6") = (uint8_t*)addr;
	asm volatile (".word (0x100B | ((6) << 15))\n");
	return *store_ptr;
}

/*------------------ Paranut functions to access UART module ----------------------*/
void UART_SendByte(uint32_t base_addr, uint8_t data) {
    while (paranut_In32(base_addr + XUARTPS_SR_OFFSET) & SR_TXFULL) {
        // wait for FIFO
    }

    paranut_Out8(base_addr + XUARTPS_FIFO_OFFSET, data);
    return;
}

uint32_t UART_Send(uint32_t base_addr, uint8_t *txbuf, uint32_t buflen) {
    // blocking mode
	int bSent = 0;
	while(buflen) {
        UART_SendByte(base_addr, txbuf[bSent]);
        buflen--;
		bSent++;
    }
	return bSent;
}
/*-----------------------------------------------------------*/

uint8_t UART_ReceiveByte(uint32_t base_addr) {
	while (paranut_In32(base_addr + XUARTPS_SR_OFFSET) & SR_RXEMPTY) {
        // wait for FIFO
    }

    return paranut_In8(base_addr + XUARTPS_FIFO_OFFSET);   
}

uint32_t UART_Receive(uint32_t base_addr, uint8_t *rxbuf, uint32_t buflen) {
	// blocking mode
	int bRecv = 0;
	while(buflen) {
        rxbuf[bRecv] = UART_ReceiveByte(base_addr);
        buflen--;
		bRecv++;
    }
	return bRecv;
}
/*-----------------------------------------------------------*/

portBASE_TYPE FreeRTOS_UART_init(uint32_t base_addr) {
	
	portBASE_TYPE xReturn = pdPASS;
	// Confiure UART PINS
	// TODO:

	/* Set up the default UART configuration. */
	// TODO:

	/* Enable the FIFO. */
	// TODO:

	/* Enable Tx. */
	// TODO:
	/*
	HIER DIE ZYBO UART INITIALISIERUNG

	if () {

		xReturn = pdFAIL;
	}

	*/


	UARTn_register_t uart;

	// init all uart registers
	uart.CR = 0x114;
	uart.MR = 0x20;
	uart.IER = 0;
    uart.IDR = 0;
    // ro
	uart.IMR = 0;
	// wtc; was 0xf1a
    uart.ISR = 0x0;
    uart.BAUDGEN = 0x7c;
    uart.RXTOUT = 0;
    uart.RXWM = 0x20;
    uart.MODEMCR = 0x0;
    uart.MODEMSR = 0xfb;
	// ro
    uart.SR = 0;
    uart.FIFO = 0x0;
    uart.divider_reg0 = 0x06;
    uart.Flow_delay_reg0 = 0x0;
    uart.Tx_FIFO_trigger_level0 = 0x20;

	// write the uart hardware registers
	for (uint32_t i = 0; i < (sizeof(UARTn_register_t) / sizeof(uint32_t)); i++) {
		paranut_Out32(base_addr + (i * sizeof(uint32_t)), ((uint32_t*)&uart)[i]);	
	}

	return xReturn;	
}
/*-----------------------------------------------------------*/

portBASE_TYPE FreeRTOS_UART_DeInit(uint32_t base_addr) {
	// DEINIT @todo:
}
/*-----------------------------------------------------------*/

portBASE_TYPE FreeRTOS_UART_open( Peripheral_Control_t * const pxPeripheralControl )
{

uint32_t uart_base_address = ( uint32_t ) diGET_PERIPHERAL_BASE_ADDRESS( pxPeripheralControl );	// base address of the UART module
portBASE_TYPE xReturn;
const uint8_t cPeripheralNumber = diGET_PERIPHERAL_NUMBER( pxPeripheralControl );

	/* Sanity check the peripheral number. */
	if( cPeripheralNumber < boardNUM_UARTS )
	{
		pxPeripheralControl->read = FreeRTOS_UART_read;
		pxPeripheralControl->write = FreeRTOS_UART_write;
		pxPeripheralControl->ioctl = FreeRTOS_UART_ioctl;

		/* Setup the pins for the UART being used. */
		taskENTER_CRITICAL();
		{
			FreeRTOS_UART_init(uart_base_address);
		}
		taskEXIT_CRITICAL();

		xReturn = pdPASS;
	}
	else
	{
		xReturn = pdFAIL;
	}

	return xReturn;
}

/*-----------------------------------------------------------*/

size_t FreeRTOS_UART_write( Peripheral_Descriptor_t const pxPeripheral, const void *pvBuffer, const size_t xBytes )
{
Peripheral_Control_t * const pxPeripheralControl = ( Peripheral_Control_t * const ) pxPeripheral;
size_t xReturn = 0U;
uint32_t uart_base_address = ( uint32_t ) diGET_PERIPHERAL_BASE_ADDRESS( ( ( Peripheral_Control_t * const ) pxPeripheral ) );
int8_t cPeripheralNumber;

	configASSERT( diGET_TX_TRANSFER_STRUCT( pxPeripheralControl ) == NULL );

	#if ioconfigUSE_UART_POLLED_TX == 1
	{
		/* No FreeRTOS objects exist to allow transmission without blocking
		the	task, so just send out by polling.  No semaphore or queue is
		used here, so the application must ensure only one task attempts to
		make a polling write at a time. */
		// TODO:
		//xReturn = UART_Send( pxUART, ( uint8_t * ) pvBuffer, ( size_t ) xBytes, BLOCKING );

		/* The UART is set to polling mode, so may as well poll the busy bit
		too.  Change to interrupt driven mode to avoid wasting CPU time here. */
		// TODO:
		//while( UART_CheckBusy( pxUART ) != RESET );

		xReturn = UART_Send(uart_base_address, ( uint8_t * ) pvBuffer, ( size_t ) xBytes);
	}
	#endif /* ioconfigUSE_UART_POLLED_TX */


	return xReturn;
}
/*-----------------------------------------------------------*/

size_t FreeRTOS_UART_read( Peripheral_Descriptor_t const pxPeripheral, void * const pvBuffer, const size_t xBytes )
{
Peripheral_Control_t * const pxPeripheralControl = ( Peripheral_Control_t * const ) pxPeripheral;
size_t xReturn = 0U;
uint32_t uart_base_address = ( uint32_t ) diGET_PERIPHERAL_BASE_ADDRESS( ( ( Peripheral_Control_t * const ) pxPeripheral ) );
	configASSERT(diGET_RX_TRANSFER_STRUCT( pxPeripheralControl ) == NULL);
	#if ioconfigUSE_UART_POLLED_RX == 1
	{
		/* No FreeRTOS objects exist to allow reception without blocking
		the task, so just receive by polling.  No semaphore or queue is
		used here, so the application must ensure only one task attempts
		to make a polling read at a time. */
		// TODO:
		//xReturn = UART_Receive( pxUART, pvBuffer, xBytes, NONE_BLOCKING );
		xReturn = UART_Receive(uart_base_address, pvBuffer, xBytes);
	}
	#endif /* ioconfigUSE_UART_POLLED_RX */
	

	return xReturn;
}
/*-----------------------------------------------------------*/

static inline size_t prvFillFifoFromBuffer( void * const pxUART, uint8_t **ppucBuffer, const size_t xTotalBytes )
{
size_t xBytesSent = 0U;

	/* This function is only used by zero copy transmissions, so mutual
	exclusion is already taken care of by the fact that a task must first
	obtain a semaphore before initiating a zero copy transfer.  The semaphore
	is part of the zero copy structure, not part of the application. */
	// TODO: WHILE FIFO NOT FULL, WRITE

	return xBytesSent;
}
/*-----------------------------------------------------------*/

portBASE_TYPE FreeRTOS_UART_ioctl( Peripheral_Descriptor_t pxPeripheral, uint32_t ulRequest, void *pvValue )
{
Peripheral_Control_t * const pxPeripheralControl = ( Peripheral_Control_t * const ) pxPeripheral;
uint32_t ulValue = ( uint32_t ) pvValue;
const int8_t cPeripheralNumber = diGET_PERIPHERAL_NUMBER( ( ( Peripheral_Control_t * const ) pxPeripheral ) );
void * pxUART = ( void * ) diGET_PERIPHERAL_BASE_ADDRESS( ( ( Peripheral_Control_t * const ) pxPeripheral ) );
portBASE_TYPE xReturn = pdPASS;

	/* Sanity check the array index. */
	configASSERT( cPeripheralNumber < ( int8_t ) ( sizeof( xIRQ ) / sizeof( IRQn_Type ) ) );

	taskENTER_CRITICAL();
	{
		switch( ulRequest )
		{
			case ioctlUSE_INTERRUPTS :

				xReturn = pdFAIL;
				break;


			case ioctlSET_SPEED :

				/* Set up the default UART configuration. */
				xReturn = pdFAIL;
				break;


			case ioctlSET_INTERRUPT_PRIORITY :

				/* The ISR uses ISR safe FreeRTOS API functions, so the priority
				being set must be lower than (ie numerically larger than)
				configMAX_LIBRARY_INTERRUPT_PRIORITY. */
				xReturn = pdFAIL;
				break;


			default :

				xReturn = pdFAIL;
				break;
		}
	}
	taskEXIT_CRITICAL();

	return xReturn;
}
/*-----------------------------------------------------------*/

