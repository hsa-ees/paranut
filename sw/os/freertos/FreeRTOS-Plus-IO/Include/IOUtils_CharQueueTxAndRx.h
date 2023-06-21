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

#ifndef IOUTILS_TXANDRX_QUEUES_H
#define IOUTILS_TXANDRX_QUEUES_H

/* The transfer structure used when a queue is used to send or receive
characters. */
typedef struct xCHAR_BY_CHAR_QUEUE_STATE
{
	xQueueHandle xQueue;		/* A pointer to the queue itself. */
	portTickType xBlockTime;	/* The amount of time a task should be held in the Blocked state (not using CPU time) to wait for data to become available if a read attempt is made on an empty queue, or for space to become available when a write attempt is made on a full queue. */
	uint16_t usErrorState;		/* Currently just set to pdFALSE or pdTRUE if a buffer overrun has not/has occurred respectively. */
} Character_Queue_State_t;

/* Transfer type casts from peripheral structs. */
#define prvTX_CHAR_QUEUE_STATE( pxPeripheralControl ) ( ( Character_Queue_State_t * ) ( pxPeripheralControl )->pxTxControl->pvTransferState )
#define prvRX_CHAR_QUEUE_STATE( pxPeripheralControl ) ( ( Character_Queue_State_t * ) ( pxPeripheralControl )->pxRxControl->pvTransferState )

/*-----------------------------------------------------------*/

/*
 * Char queue Tx macros.
 */
#define ioutilsTX_CHARS_FROM_QUEUE_FROM_ISR( pxTransferControl, xCondition, xTransmitFunction, xHigherPriorityTaskWoken ) \
{																															\
Character_Queue_State_t *pxCharQueueState = ( Character_Queue_State_t * ) ( ( pxTransferControl )->pvTransferState );	\
uint8_t ucChar;																												\
																															\
	while( ( xCondition ) )																									\
	{																														\
		if( xQueueReceiveFromISR( pxCharQueueState->xQueue, &ucChar, &xHigherPriorityTaskWoken ) == pdTRUE )				\
		{																													\
			( xTransmitFunction );																							\
		}																													\
		else																												\
		{																													\
			break;																											\
		}																													\
	}																														\
}
/*-----------------------------------------------------------*/

#define ioutilsBLOCKING_SEND_CHARS_TO_TX_QUEUE( pxPeripheralControl, pxPeripheralNotBusy, xPeripheralWrite, pucBuffer, xTotalBytes, xBytesSent )	\
{																															\
portTickType xTicksToWait;																									\
uint8_t ucChar;																												\
xTimeOutType xTimeOut;																										\
Character_Queue_State_t *pxTxState = prvTX_CHAR_QUEUE_STATE( pxPeripheralControl );											\
xQueueHandle xQueue;																										\
																															\
	xTicksToWait = pxTxState->xBlockTime;																					\
	xQueue = pxTxState->xQueue;																								\
	vTaskSetTimeOutState( &xTimeOut );																						\
																															\
	for( ( xBytesSent ) = 0U; ( xBytesSent ) < ( xTotalBytes ); ( xBytesSent )++ )											\
	{																														\
		if( xQueueSend( xQueue, &( pucBuffer[ ( xBytesSent ) ] ), xTicksToWait ) != pdPASS )								\
		{																													\
			break;																											\
		}																													\
		else																												\
		{																													\
			/* If this task is low priority it is possible that it only														\
			gets intermittent CPU time, and therefore possible that the														\
			peripheral interrupt has drained the Tx queue before this point is												\
			reached.  If this is the case, and the queue contains more														\
			data, then force the peripheral ISR to be re-entered. */														\
			if( ( pxPeripheralNotBusy ) )																					\
			{																												\
				if( xQueueReceive( xQueue, &ucChar, xTicksToWait ) == pdPASS )												\
				{																											\
					( xPeripheralWrite );																					\
				}																											\
			}																												\
		}																													\
																															\
		if( xTaskCheckForTimeOut( &xTimeOut, &xTicksToWait ) != pdFALSE )													\
		{																													\
			/* Time out has expired. */																						\
			break;																											\
		}																													\
	}																														\
}
/*-----------------------------------------------------------*/

#define ioutilsFILL_FIFO_FROM_TX_QUEUE( pxPeripheralControl, xDisablePeripheral, xEnablePeripheral, ulFifoDepth, xFifoNotFull, xPeripheralWrite )		\
{																																	\
uint8_t ucChar;																														\
Character_Queue_State_t *pxTxState = prvTX_CHAR_QUEUE_STATE( pxPeripheralControl );													\
uint32_t ulByte;																													\
																																	\
	( xDisablePeripheral );																											\
	for( ulByte = 0; ulByte < ulFifoDepth; ulByte++ )																				\
	{																																\
		if( ( xFifoNotFull ) )																										\
		{																															\
			if( xQueueReceive( pxTxState->xQueue, &ucChar, 0 ) == pdPASS )															\
			{																														\
				( xPeripheralWrite );																								\
			}																														\
		}																															\
		else																														\
		{																															\
			break;																													\
		}																															\
	}																																\
	( xEnablePeripheral );																											\
}
/*-----------------------------------------------------------*/

/*
 * Char queue Rx macros
 */
#define ioutilsRX_CHARS_INTO_QUEUE_FROM_ISR( pxTransferControl, xCondition, xReceiveFunction, ulReceived, xHigherPriorityTaskWoken )	\
{																															\
Character_Queue_State_t *pxCharQueueState = ( Character_Queue_State_t * ) ( ( pxTransferControl )->pvTransferState );	\
uint8_t ucChar;																												\
																															\
	while( ( xCondition ) )																									\
	{																														\
		ucChar = ( xReceiveFunction );																						\
		if( xQueueSendFromISR( pxCharQueueState->xQueue, &ucChar, &( xHigherPriorityTaskWoken ) ) != pdPASS )				\
		{																													\
			pxCharQueueState->usErrorState = pdTRUE;																		\
		}																													\
		else																												\
		{																													\
			ulReceived++;																									\
		}																													\
	}																														\
}
/*-----------------------------------------------------------*/


/* Prototypes of functions that are for internal use only. */
void xIOUtilsSetTxQueueTimeout( Peripheral_Control_t * const pxPeripheralControl, const portTickType xMaxWaitTime );
void vIOUtilsSetRxQueueTimeout( Peripheral_Control_t * const pxPeripheralControl, const portTickType xMaxWaitTime );
portBASE_TYPE xIOUtilsConfigureTransferQueue( Peripheral_Control_t * const pxPeripheralControl, const uint32_t ulRequest, const unsigned int ulQueueLength );
size_t xIOUtilsReceiveCharsFromRxQueue( Peripheral_Control_t * const pxPeripheralControl, uint8_t * const pucBuffer, const size_t xTotalBytes );
size_t xIOUtilsSendCharsToTxQueue( Peripheral_Control_t * const pxPeripheralControl, uint8_t * const pucBuffer, const size_t xTotalBytes );
portBASE_TYPE xIOUtilsWaitTxQueueEmpty( Peripheral_Control_t * const pxPeripheralControl, const portTickType xMaxWaitTime );
void xIOUtilsClearRxCharQueue( Peripheral_Control_t *pxPeripheralControl );

#endif /* IOUTILS_TXANDRX_QUEUES_H */




