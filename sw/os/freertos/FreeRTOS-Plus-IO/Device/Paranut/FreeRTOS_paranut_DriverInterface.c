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

/* FreeRTOS IO library includes. */
#include "FreeRTOS_DriverInterface.h"
#include "FreeRTOS_uart.h"
#include "paranut-base-board.h"

portBASE_TYPE vFreeRTOS_paranut_PopulateFunctionPointers( const Peripheral_Types_t ePeripheralType, Peripheral_Control_t * const pxPeripheralControl )
{
portBASE_TYPE xReturn = pdFALSE;

	switch( ePeripheralType )
	{
		/* Open the peripheral. */
		case eUART_TYPE	:

			#if ioconfigINCLUDE_UART == 1
			{
				xReturn = FreeRTOS_UART_open( pxPeripheralControl );
			}
			#endif /* ioconfigINCLUDE_UART */
			break;


		case eSSP_TYPE :

			#if ioconfigINCLUDE_SSP == 1
			{
				xReturn = FreeRTOS_SSP_open( pxPeripheralControl );
			}
			#endif /* ioconfigINCLUDE_SSP */
			break;
			

		case eI2C_TYPE :
		
			#if ioconfigINCLUDE_I2C == 1
			{
				xReturn = FreeRTOS_I2C_open( pxPeripheralControl );
			}
			#endif /* ioconfigINCLUDE_I2C */
			break;


		default :
		
			/* Nothing to do here.  xReturn is already set to pdFALSE. */
			configASSERT( xReturn );
			break;
	}
	
	/* Just to prevent compiler warnings should FreeRTOSIOConfig.h be
	configured to exclude the above FreeRTOS_nnn_open() calls. */
	( void ) pxPeripheralControl;

	return xReturn;
} 
