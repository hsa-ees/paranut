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

#ifndef IOUTILS_ZERO_COPY_TX_H
#define IOUTILS_ZERO_COPY_TX_H

/* The transfer structure used when a zero copy method is used for
transmission. */
typedef struct xZERO_COPY_TX_STATE
{
	xSemaphoreHandle xWriteAccessMutex; /* Mutex used to indicate the end of transmission, meaning the buffer being transmitted is free for other use, and a new Tx can start if desired. */
	uint8_t *pucBufferStart;			/* The start of the buffer to be transmitted. */
	uint16_t usBufferLength;			/* The length of the buffer to be transmitted. */
} Zero_Copy_Tx_State_t;

/*
 * Zero copy Tx access macros.
 */
#define iouitlsTX_CHARS_FROM_ZERO_COPY_BUFFER_FROM_ISR( pxTransferControl, xCondition, xTransmitFunction, xHigherPriorityTaskWoken )	\
{																															\
Zero_Copy_Tx_State_t * const pxZeroCopyState = ( Zero_Copy_Tx_State_t * ) ( pxTransferControl )->pvTransferState;		\
uint8_t ucChar;																												\
																															\
	if( pxZeroCopyState->usBufferLength != 0 )																				\
	{																														\
		while( ( xCondition ) )																								\
		{																													\
			if( pxZeroCopyState->pucBufferStart == NULL )																	\
			{																												\
				/* Required for peripherals such as SPI and SSP where a Tx is												\
				required in order to receive characters. */																	\
				ucChar = 0xffU;																								\
			}																												\
			else																											\
			{																												\
				ucChar = pxZeroCopyState->pucBufferStart[ 0 ];																\
				( pxZeroCopyState->pucBufferStart )++;																		\
			}																												\
			( xTransmitFunction );																							\
			pxZeroCopyState->usBufferLength--;																				\
																															\
			if( pxZeroCopyState->usBufferLength == 0U )																		\
			{																												\
				/* There are no more characters to transmit.  'Give' the													\
				semaphore to unblock any tasks that are waiting to get access												\
				to the Tx buffer. */																						\
				xSemaphoreGiveFromISR( pxZeroCopyState->xWriteAccessMutex, &( xHigherPriorityTaskWoken ) );					\
				break;																										\
			}																												\
		}																													\
	}																														\
}

#define iouitlsTX_SINGLE_CHAR_FROM_ZERO_COPY_BUFFER_FROM_ISR( pxTransferControl, xTransmitFunction, xCharSent )			\
{																															\
Zero_Copy_Tx_State_t * const pxZeroCopyState = ( Zero_Copy_Tx_State_t * ) ( pxTransferControl )->pvTransferState;		\
uint8_t ucChar;																												\
																															\
	if( pxZeroCopyState->usBufferLength != 0 )																				\
	{																														\
		xCharSent = pdTRUE;																									\
		ucChar = pxZeroCopyState->pucBufferStart[ 0 ];																		\
		( xTransmitFunction );																								\
		( pxZeroCopyState->pucBufferStart )++;																				\
		pxZeroCopyState->usBufferLength--;																					\
	}																														\
	else																													\
	{																														\
		xCharSent = pdFALSE;																								\
	}																														\
}

#define ioutilsGIVE_ZERO_COPY_MUTEX( pxTransferControl, xHigherPriorityTaskWoken )											\
{																															\
Zero_Copy_Tx_State_t * const pxZeroCopyState = ( Zero_Copy_Tx_State_t * ) ( pxTransferControl )->pvTransferState;			\
																															\
	xSemaphoreGiveFromISR( pxZeroCopyState->xWriteAccessMutex, &( xHigherPriorityTaskWoken ) );								\
}

#define ioutilsINITIATE_ZERO_COPY_TX( pxPeripheralControl, xDisablePeripheral, xEnablePeripheral, xTxFunction, pvBuffer, xBytes )	\
{																															\
size_t xBytesSentToFifo;																									\
Zero_Copy_Tx_State_t * pxZeroCopyState = pxPeripheralControl->pxTxControl->pvTransferState;									\
																															\
	if( xSemaphoreGetMutexHolder( pxZeroCopyState->xWriteAccessMutex ) == xTaskGetCurrentTaskHandle() )						\
	{																														\
		/* Disable Tx to prevent race conditions with the interrupt. */														\
		( xDisablePeripheral );																								\
																															\
		/* Sends as many bytes as possible to the peripheral. */															\
		xBytesSentToFifo = ( xTxFunction );																					\
																															\
		if( xBytesSentToFifo < xBytes )																						\
		{																													\
			/* Configure the Tx structure to send whatever has not already been												\
			sent to the FIFO. */																							\
			pxZeroCopyState->pucBufferStart = ( uint8_t * ) ( pvBuffer );													\
			pxZeroCopyState->usBufferLength = ( uint16_t ) ( ( xBytes ) - xBytesSentToFifo );								\
			( xEnablePeripheral );																							\
		}																													\
		else																												\
		{																													\
			/* All the bytes fitted in the FIFO. */																			\
			pxZeroCopyState->usBufferLength = ( uint16_t ) 0U;																\
			( xEnablePeripheral );																							\
			xSemaphoreGive( pxZeroCopyState->xWriteAccessMutex );															\
		}																													\
	}																														\
	else																													\
	{																														\
		( xBytes ) = 0U;																										\
	}																														\
}

#define ioutilsDEFAULT_ZERO_COPY_TX_MUTEX_BLOCK_TIME ( 500U / portTICK_RATE_MS )

/* Prototypes of functions that are for internal use only. */
portBASE_TYPE xIOUtilsConfigureZeroCopyTx( Peripheral_Control_t * const pxPeripheralControl );
portBASE_TYPE xIOUtilsGetZeroCopyWriteMutex( Peripheral_Control_t * const pxPeripheralControl, uint32_t ulRequest, portTickType const xMaxWaitTime );
portBASE_TYPE xIOUtilsReleaseZeroCopyWriteMutex( Peripheral_Control_t * const pxPeripheralControl );

#endif /* IOUTILS_ZERO_COPY_TX_H */




