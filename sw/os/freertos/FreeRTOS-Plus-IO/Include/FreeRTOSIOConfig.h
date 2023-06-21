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

#ifndef FREERTOS_IO_CONFIG_H
#define FREERTOS_IO_CONFIG_H

/* Global transfer mode settings. */
#define ioconfigUSE_ZERO_COPY_TX							1
#define ioconfigUSE_TX_CHAR_QUEUE  							1
#define ioconfigUSE_CIRCULAR_BUFFER_RX 						1
#define ioconfigUSE_RX_CHAR_QUEUE 							1

/* universal asynchronous receiver transmitter */
#define ioconfigINCLUDE_UART								1
	#define ioconfigUSE_UART_POLLED_TX						1
	#define ioconfigUSE_UART_POLLED_RX						1
	#define ioconfigUSE_UART_ZERO_COPY_TX					0
	#define ioconfigUSE_UART_TX_CHAR_QUEUE					0
	#define ioconfigUSE_UART_CIRCULAR_BUFFER_RX				0
	#define ioconfigUSE_UART_RX_CHAR_QUEUE					0

/* Synchronous Serial Port */
#define ioconfigINCLUDE_SSP									0
	#define ioconfigUSE_SSP_POLLED_TX						1
	#define ioconfigUSE_SSP_POLLED_RX						1
	#define ioconfigUSE_SSP_ZERO_COPY_TX					1
	#define ioconfigUSE_SSP_CIRCULAR_BUFFER_RX				1
	#define ioconfigUSE_SSP_RX_CHAR_QUEUE					1
	#define ioconfigUSE_SSP_TX_CHAR_QUEUE					1

/* inter-integradec circuit */
#define ioconfigINCLUDE_I2C									0
	#define ioconfigUSE_I2C_POLLED_TX						0
	#define ioconfigUSE_I2C_POLLED_RX						0
	#define ioconfigUSE_I2C_ZERO_COPY_TX					0
	#define ioconfigUSE_I2C_CIRCULAR_BUFFER_RX				0
	#define ioconfigUSE_I2C_TX_CHAR_QUEUE					0




/* Sanity check configuration.  Do not edit below this line. */
#if ( ioconfigINCLUDE_UART == 1 ) && ( ioconfigUSE_UART_ZERO_COPY_TX == 1 ) && ( ioconfigUSE_ZERO_COPY_TX != 1 )
	#error ioconfigUSE_ZERO_COPY_TX must also be set to 1 if ioconfigUSE_UART_ZERO_COPY_TX is set to 1
#endif

#if ( ioconfigINCLUDE_UART == 1 ) && ( ioconfigUSE_UART_TX_CHAR_QUEUE == 1 ) && ( ioconfigUSE_TX_CHAR_QUEUE != 1 )
	#error ioconfigUSE_TX_CHAR_QUEUE must also be set to 1 if ioconfigUSE_UART_TX_CHAR_QUEUE is set to 1
#endif

#if ( ioconfigINCLUDE_UART == 1 ) && ( ioconfigUSE_UART_CIRCULAR_BUFFER_RX == 1 ) && ( ioconfigUSE_CIRCULAR_BUFFER_RX != 1 )
	#error ioconfigUSE_CIRCULAR_BUFFER_RX must also be set to 1 if ioconfigUSE_UART_CIRCULAR_BUFFER_RX is set to 1
#endif

#if ( ioconfigINCLUDE_UART == 1 ) && ( ioconfigUSE_UART_RX_CHAR_QUEUE == 1 ) && ( ioconfigUSE_RX_CHAR_QUEUE != 1 )
	#error ioconfigUSE_RX_CHAR_QUEUE must also be set to 1 if ioconfigUSE_UART_RX_CHAR_QUEUE is set to 1
#endif

#if ( ioconfigUSE_SSP == 1 ) && ( ioconfigUSE_SSP_ZERO_COPY_TX == 1 ) && ( ioconfigUSE_ZERO_COPY_TX != 1 )
	#error ioconfigUSE_ZERO_COPY_TX must also be set to 1 if ioconfigUSE_SSP_ZERO_COPY_TX is set to 1
#endif

#if ( ioconfigINCLUDE_SSP == 1 ) && ( ioconfigUSE_SSP_CIRCULAR_BUFFER_RX == 1 ) && ( ioconfigUSE_CIRCULAR_BUFFER_RX != 1 )
	#error ioconfigUSE_CIRCULAR_BUFFER_RX must also be set to 1 if ioconfigUSE_SSP_CIRCULAR_BUFFER_RX is set to 1
#endif

#if ( ioconfigINCLUDE_SSP == 1 ) && ( ioconfigUSE_SSP_TX_CHAR_QUEUE == 1 ) && ( ioconfigUSE_TX_CHAR_QUEUE != 1 )
	#error ioconfigUSE_TX_CHAR_QUEUE must also be set to 1 if ioconfigUSE_SSP_TX_CHAR_QUEUE is set to 1
#endif

#if ( ioconfigINCLUDE_SSP == 1 ) && ( ioconfigUSE_SSP_RX_CHAR_QUEUE == 1 ) && ( ioconfigUSE_RX_CHAR_QUEUE != 1 )
	#error ioconfigUSE_RX_CHAR_QUEUE must also be set to 1 if ioconfigUSE_SSP_RX_CHAR_QUEUE is set to 1
#endif

#if ( ioconfigUSE_I2C == 1 ) && ( ioconfigUSE_I2C_ZERO_COPY_TX == 1 ) && ( ioconfigUSE_ZERO_COPY_TX != 1 )
	#error ioconfigUSE_ZERO_COPY_TX must also be set to 1 if ioconfigUSE_I2C_ZERO_COPY_TX is set to 1
#endif

#if ( ioconfigINCLUDE_I2C == 1 ) && ( ioconfigUSE_I2C_TX_CHAR_QUEUE == 1 ) && ( ioconfigUSE_TX_CHAR_QUEUE != 1 )
	#error ioconfigUSE_TX_CHAR_QUEUE must also be set to 1 if ioconfigUSE_I2C_TX_CHAR_QUEUE is set to 1
#endif

#if ( ioconfigINCLUDE_I2C == 1 ) && ( ioconfigUSE_I2C_CIRCULAR_BUFFER_RX == 1 ) && ( ioconfigUSE_CIRCULAR_BUFFER_RX != 1 )
	#error ioconfigUSE_CIRCULAR_BUFFER_RX must also be set to 1 if ioconfigUSE_I2C_CIRCULAR_BUFFER_RX is set to 1
#endif

#endif /* FREERTOS_IO_CONFIG_H */


