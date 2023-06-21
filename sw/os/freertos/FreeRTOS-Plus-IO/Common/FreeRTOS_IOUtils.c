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
#include "string.h"

/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"

/* Device specific library includes. */
#include "FreeRTOS_DriverInterface.h"
#include "IOUtils_Common.h"

/*-----------------------------------------------------------*/

void vIOUtilsCreateTransferControlStructure( Transfer_Control_t **ppxTransferControl )
{
Transfer_Control_t *pxTransferControl = *ppxTransferControl;

	/* Does the transfer control structure already exist? */
	if( pxTransferControl == NULL )
	{
		/* The transfer control structure does not exist.  Create it. */
		*ppxTransferControl = ( Transfer_Control_t * ) pvPortMalloc( sizeof( Transfer_Control_t ) );
	}
	else
	{
		/* The transfer control structure does already exist, so there is
		no need to create it, however the state structure it points to is being
		changed, so delete the existing state structure, and anything it might
		contain. */
		switch( pxTransferControl->ucType )
		{
			case ioctlUSE_ZERO_COPY_TX :

				#if ioconfigUSE_ZERO_COPY_TX == 1
				{
					Zero_Copy_Tx_State_t *pxZeroCopyState;

					/* In this case, the pvTransferState member points to a zero
					copy state structure, which in turn contains a mutex that needs
					to be deleted. */
					pxZeroCopyState = ( Zero_Copy_Tx_State_t * ) ( pxTransferControl->pvTransferState );
					vSemaphoreDelete( pxZeroCopyState->xWriteAccessMutex );
					vPortFree( pxZeroCopyState );
				}
				#endif /* ioconfigUSE_ZERO_COPY_TX */
				break;


			case ioctlUSE_CHARACTER_QUEUE_TX	:
			case ioctlUSE_CHARACTER_QUEUE_RX	:

				#if ( ioconfigUSE_TX_CHAR_QUEUE == 1 ) || ( ioconfigUSE_RX_CHAR_QUEUE == 1 )
				{
					Character_Queue_State_t *pxCharQueueState;

					/* In this case the pvTrasactionState member points to a character
					queue state structure, which in turn contains a queue that needs
					to be deleted. */
					pxCharQueueState = ( Character_Queue_State_t * ) ( pxTransferControl->pvTransferState );
					vQueueDelete( pxCharQueueState->xQueue );
					vPortFree( pxCharQueueState );
				}
				#endif /* ( ioconfigUSE_TX_CHAR_QUEUE == 1 ) || ( ioconfigUSE_RX_CHAR_QUEUE == 1 ) */
				break;


			case ioctlUSE_CIRCULAR_BUFFER_RX	:

				#if ioconfigUSE_CIRCULAR_BUFFER_RX == 1
				{
					Circular_Buffer_Rx_State_t *pxCircularBufferState;

					/* In this case, the pvTransferState member points to a
					circular buffer structure, which in turn contains a semaphore
					and a buffer, both of which need to be deleted. */
					pxCircularBufferState = ( Circular_Buffer_Rx_State_t * ) ( pxTransferControl->pvTransferState );
					vSemaphoreDelete( pxCircularBufferState->xNewDataSemaphore );
					vPortFree( ( void * ) ( pxCircularBufferState->pucBufferStart ) );
					vPortFree( pxCircularBufferState );
				}
				#endif /* ioconfigUSE_CIRCULAR_BUFFER_RX */
				break;


			case ioctlUSE_POLLED_TX	:

				/* Default assumes no specific kernel objects are being used. */
				vPortFree( pxTransferControl->pvTransferState );
				break;


			default	:

				/* To get here a transfer structure must have existed, but
				with a valid pvTransferState member.  This can happen when a
				peripheral is being polled, and both the Tx and Rx transfer
				structures use the same state structure (for example, in an NXP
				SSP/SPI driver where data cannot be received without data also
				being transmitted).  There is nothing to do here. */
				break;
		}

		pxTransferControl->pvTransferState = NULL;
	}
}















