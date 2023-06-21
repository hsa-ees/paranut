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

/* Standard includes. */
#include <string.h>

/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

/* FreeRTOS IO library includes. */
#include "FreeRTOS_IO.h"
#include "IOUtils_Common.h"

/*-----------------------------------------------------------*/

/* Holds the list of peripherals that are available to the FreeRTOS+IO
interface.  boardAVAILABLE_DEVICED_LIST is defined in FreeRTOS_IO_BSP.h, and is
specific to a hardware platform. */
static const Available_Peripherals_t xAvailablePeripherals[] = boardAVAILABLE_DEVICES_LIST;

/*-----------------------------------------------------------*/

/* See the function prototype definition for documentation information. */
Peripheral_Descriptor_t FreeRTOS_open( const int8_t *pcPath, const uint32_t ulFlags )
{
portBASE_TYPE xIndex, xInitialiseResult;
const portBASE_TYPE xNumberOfPeripherals = sizeof( xAvailablePeripherals ) / sizeof( Available_Peripherals_t );
int8_t cPeripheralNumber;
Peripheral_Control_t *pxPeripheralControl = NULL;

	/* The flags exist to maintain a standard looking interface, but are not
	(yet) used. */
	( void ) ulFlags;

	/* Search for the peripheral in the list of peripherals for the board being
	used. */
	for( xIndex = 0; xIndex < xNumberOfPeripherals; xIndex++ )
	{
		if( strcmp( ( const char * const ) pcPath, ( const char * const ) xAvailablePeripherals[ xIndex ].pcPath ) == 0 )
		{
			/* pcPath is a valid path, search no further. */
			break;
		}
	}

	if( xIndex < xNumberOfPeripherals )
	{
		/* pcPath was a valid path.  Extract the peripheral number.  The
		peripheral number cannot appear in the middle of a string, so must be
		followed by the end of string character. */
		while( ( ( *( pcPath + 1 ) ) != '/' ) && ( ( *( pcPath + 1 ) ) != 0x00 ) )
		{
			pcPath++;
			while( ( *pcPath < '0' ) || ( *pcPath > '9' ) )
			{
				pcPath++;
			}
		}

		/* Convert the number from its ASCII representation. */
		cPeripheralNumber = *pcPath - '0';

		/* Create the peripheral control structure used by FreeRTOS+IO to
		access the peripheral.  This is also used as the handle to the
		peripheral. */
		pxPeripheralControl = pvPortMalloc( sizeof( Peripheral_Control_t ) );
		if( pxPeripheralControl != NULL )
		{
			/* Initialise the common parts of the control structure. */
			pxPeripheralControl->pxTxControl = NULL;
			pxPeripheralControl->pxRxControl = NULL;
			pxPeripheralControl->pxDevice = &( xAvailablePeripherals[ xIndex ] );
			pxPeripheralControl->cPeripheralNumber = cPeripheralNumber;

			/* Initialise the peripheral specific parts of the control
			structure, and call the peripheral specific open function. */
			xInitialiseResult = boardFreeRTOS_PopulateFunctionPointers( xAvailablePeripherals[ xIndex ].xPeripheralType, pxPeripheralControl );
									
			if( xInitialiseResult != pdPASS )
			{
				/* Something went wrong.  Free up resources and return NULL. */
				vPortFree( pxPeripheralControl );
				pxPeripheralControl = NULL;
			}
		}
	}

	return ( Peripheral_Descriptor_t ) pxPeripheralControl;
}
/*-----------------------------------------------------------*/

/* See the function prototype definition for documentation information. */
portBASE_TYPE FreeRTOS_ioctl( Peripheral_Descriptor_t const xPeripheral, uint32_t ulRequest, void *pvValue )
{
Peripheral_Control_t *pxPeripheralControl = ( Peripheral_Control_t * ) xPeripheral;
portBASE_TYPE xReturn = pdFAIL, xCommandIsDeviceSpecific = pdFALSE;

	configASSERT( pxPeripheralControl );

	/* Handle any non peripheral specific commands here. */
	switch( ulRequest )
	{
		case ioctlUSE_ZERO_COPY_TX	:

			#if ioconfigUSE_ZERO_COPY_TX == 1
			{
				/* The peripheral is going to use a	Zero_Copy_Tx_State_t
				structure for transmission.  This allows characters to be
				transmitted directly from the buffer supplied to the
				FreeRTOS_write() function. */
				xReturn = xIOUtilsConfigureZeroCopyTx( pxPeripheralControl );

				/* Zero copy Tx buffers can only be used when interrupts are
				also used.  Enabling interrupts is a peripheral specific
				operation. */
				ulRequest = ioctlUSE_INTERRUPTS;
				pvValue = ( void * ) pdTRUE;
				xCommandIsDeviceSpecific = pdTRUE;
			}
			#endif /* ioconfigUSE_ZERO_COPY_TX */
			break;


		case ioctlUSE_CIRCULAR_BUFFER_RX :

			#if ioconfigUSE_CIRCULAR_BUFFER_RX == 1
			{
				/* The peripheral is going to use a Circular_Buffer_Rx_State_t
				structure for reception.  This places received characters into
				a buffer, then allows a pointer to the buffer to be returned
				directly in a read function. */
				xReturn = xIOUtilsConfigureCircularBufferRx( pxPeripheralControl, ( portBASE_TYPE ) pvValue );

				/* Circular buffer Rx can only be used when interrupts are also
				used.  Enabling interrupts is a peripheral specific operation. */
				ulRequest = ioctlUSE_INTERRUPTS;
				pvValue = ( void * ) pdTRUE;
				xCommandIsDeviceSpecific = pdTRUE;
			}
			#endif /* ioconfigUSE_CIRCULAR_BUFFER_RX */
			break;


		case ioctlUSE_CHARACTER_QUEUE_TX	:
		case ioctlUSE_CHARACTER_QUEUE_RX	:

			#if ( ioconfigUSE_TX_CHAR_QUEUE == 1 ) || ( ioconfigUSE_RX_CHAR_QUEUE == 1 )
			{
				/* The peripheral is going to use a character by character
				queue to transmit or receive data.  This is an easy and
				convenient method, but inefficient for systems that have a
				high throughput. */
				xReturn = xIOUtilsConfigureTransferQueue( pxPeripheralControl, ulRequest, ( portUBASE_TYPE ) pvValue );

				/* Character queues can only be used when interrupts are also
				used.  Enabling interrupts is a device specific operation. */
				ulRequest = ioctlUSE_INTERRUPTS;
				pvValue = ( void * ) pdTRUE;
				xCommandIsDeviceSpecific = pdTRUE;
			}
			#endif /* ( ioconfigUSE_TX_CHAR_QUEUE == 1 ) || ( ioconfigUSE_RX_CHAR_QUEUE == 1 ) */
			break;


		case ioctlSET_TX_TIMEOUT 	:

			if( pxPeripheralControl->pxTxControl->ucType == ioctlUSE_CHARACTER_QUEUE_TX )
			{
				#if ( ioconfigUSE_TX_CHAR_QUEUE == 1 )
				{
					xIOUtilsSetTxQueueTimeout( pxPeripheralControl, ( portTickType ) pvValue );
					xReturn = pdPASS;
				}
				#endif /* ( ioconfigUSE_TX_CHAR_QUEUE == 1 ) */
			}
			else
			{
				/* There is nothing to do here as xReturn is already pdFAIL. */
			}
			break;


		case ioctlSET_RX_TIMEOUT	:

			if( pxPeripheralControl->pxRxControl->ucType == ioctlUSE_CIRCULAR_BUFFER_RX )
			{
				#if ioconfigUSE_CIRCULAR_BUFFER_RX == 1
				{
					vIOUtilsSetCircularBufferTimeout( pxPeripheralControl, ( portTickType ) pvValue );
					xReturn = pdPASS;
				}
				#endif /* ioctlUSE_CIRCUAL_BUFFER_RX */
			}
			else if( pxPeripheralControl->pxRxControl->ucType == ioctlUSE_CHARACTER_QUEUE_RX )
			{
				#if ioconfigUSE_RX_CHAR_QUEUE == 1
				{
					vIOUtilsSetRxQueueTimeout( pxPeripheralControl, ( portTickType ) pvValue );
					xReturn = pdPASS;
				}
				#endif /* ioconfigUSE_RX_CHAR_QUEUE */
			}
			else
			{
				/* Nothing to do here as xReturn is already pdFAIL. */
			}
			break;


		/* These two cases are intentionally together as their functionality is
		very similar. */
		case ioctlOBTAIN_WRITE_MUTEX :
		case ioctlWAIT_PREVIOUS_WRITE_COMPLETE :

			xReturn = pdTRUE;

			if( pxPeripheralControl->pxTxControl->ucType == ioctlUSE_ZERO_COPY_TX )
			{
				#if ioconfigUSE_ZERO_COPY_TX == 1
				{
					/* The write mutex should be obtained prior to attempting a
					zero copy Tx operation.  Obtaining the write mutex also shows
					no transmissions are currently in progress, so provides a
					useful method of waiting until a write has completed. */
					xReturn = xIOUtilsGetZeroCopyWriteMutex( pxPeripheralControl, ulRequest, ( portTickType ) pvValue );
				}
				#endif /* #if ioconfigUSE_ZERO_COPY_TX == 1 */
			}
			else if( pxPeripheralControl->pxTxControl->ucType == ioctlUSE_CHARACTER_QUEUE_TX )
			{
				#if ioconfigUSE_RX_CHAR_QUEUE == 1
				{
					xReturn = xIOUtilsWaitTxQueueEmpty( pxPeripheralControl, ( portTickType ) pvValue );
				}
				#endif /* ioconfigUSE_RX_CHAR_QUEUE */
			}
			else
			{
				/* Nothing to do here as xReturn is already set to pdTRUE.  It
				has to be set to pdTRUE before this if/else construct in case
				the configuration is such that code is conditionally compiled
				out (which would be an error anyway. */
			}
			break;


		case ioctlRELEASE_WRITE_MUTEX :

			xReturn = pdTRUE;

			#if ioconfigUSE_ZERO_COPY_TX == 1
			{
				if( pxPeripheralControl->pxTxControl->ucType == ioctlUSE_ZERO_COPY_TX )
				{
						/* Give back the write mutex, if it is held. */
						xReturn = xIOUtilsReleaseZeroCopyWriteMutex( pxPeripheralControl );
				}
			}
			#endif /* ioconfigUSE_ZERO_COPY_TX */
			break;


		case ioctlCLEAR_RX_BUFFER :

			if( pxPeripheralControl->pxRxControl->ucType == ioctlUSE_CIRCULAR_BUFFER_RX )
			{
				#if ioconfigUSE_CIRCULAR_BUFFER_RX == 1
				{
					vIOUtilsClearRxCircularBuffer( pxPeripheralControl );
					xReturn = pdPASS;
				}
				#endif /* ioconfigUSE_CIRCULAR_BUFFER_RX */
			}
			else if( pxPeripheralControl->pxRxControl->ucType == ioctlUSE_CHARACTER_QUEUE_RX )
			{
				#if ioconfigUSE_RX_CHAR_QUEUE == 1
				{
					xIOUtilsClearRxCharQueue( pxPeripheralControl );
					xReturn = pdPASS;
				}
				#endif /* ioconfigUSE_RX_CHAR_QUEUE */
			}
			else
			{
				/* Nothing to do here as xReturn is already set to pdFAIL; */
			}
			break;


		default :

			xCommandIsDeviceSpecific = pdTRUE;
			xReturn = pdPASS;
			break;
	}

	/* Handle any device specific commands. */
	if( ( xCommandIsDeviceSpecific == pdTRUE ) && ( xReturn != pdFAIL ) )
	{
		xReturn = pxPeripheralControl->ioctl( pxPeripheralControl, ulRequest, pvValue );
	}

	return xReturn;
}
/*-----------------------------------------------------------*/



