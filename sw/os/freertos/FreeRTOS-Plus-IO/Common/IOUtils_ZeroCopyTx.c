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

portBASE_TYPE xIOUtilsConfigureZeroCopyTx( Peripheral_Control_t * const pxPeripheralControl )
{
portBASE_TYPE xReturn = pdFAIL;
Zero_Copy_Tx_State_t *pxZeroCopyState;

	/* A peripheral is going to use a Zero_Copy_Tx_State_t structure to control
	transmission. */
	vIOUtilsCreateTransferControlStructure( &( pxPeripheralControl->pxTxControl ) );
	configASSERT( pxPeripheralControl->pxTxControl );

	if( pxPeripheralControl->pxTxControl != NULL )
	{
		/* Create the necessary structure. */
		pxZeroCopyState = pvPortMalloc( sizeof( Zero_Copy_Tx_State_t ) );

		if( pxZeroCopyState != NULL )
		{
			pxZeroCopyState->xWriteAccessMutex = NULL;

			/* The structure just created contains a Semaphore handle.  Create
			the mutex type semaphore too. */
			pxZeroCopyState->xWriteAccessMutex = xSemaphoreCreateMutex();
			pxZeroCopyState->usBufferLength = 0U;

			if( pxZeroCopyState->xWriteAccessMutex != NULL )
			{
				/* The semaphore was created correctly.  Fill in the private
				data structure. */
				pxPeripheralControl->pxTxControl->pvTransferState = ( void * ) pxZeroCopyState;
				pxPeripheralControl->pxTxControl->ucType = ioctlUSE_ZERO_COPY_TX;
				xReturn = pdPASS;
			}
			else
			{
				/* The semaphore was not created successfully, free the
				Zero_Copy_Tx_State_t structure and just return an error. */
				vPortFree( pxZeroCopyState );
				pxZeroCopyState = NULL;
			}
		}

		if( pxZeroCopyState == NULL )
		{
			/* The Tx structure, or a member it contains,  could not be created,
			so the Tx control structure (which should point to it) should also
			be deleted. */
			vPortFree( pxPeripheralControl->pxTxControl );
			pxPeripheralControl->pxTxControl = NULL;
		}
	}

	return xReturn;
}
/*-----------------------------------------------------------*/

portBASE_TYPE xIOUtilsGetZeroCopyWriteMutex( Peripheral_Control_t * const pxPeripheralControl, uint32_t ulRequest, const portTickType xMaxWaitTime )
{
portBASE_TYPE xReturn;
Transfer_Control_t *pxTransferControlState = pxPeripheralControl->pxTxControl;
Zero_Copy_Tx_State_t *pxZeroCopyState;

	configASSERT( pxTransferControlState );

	pxZeroCopyState = ( Zero_Copy_Tx_State_t * ) ( pxTransferControlState->pvTransferState );
	xReturn = xSemaphoreTake( pxZeroCopyState->xWriteAccessMutex, xMaxWaitTime );

	if( ( xReturn == pdPASS ) && ( ulRequest == ioctlWAIT_PREVIOUS_WRITE_COMPLETE ) )
	{
		/* The function was only called to wait until the current transmission
		was complete, so the task should not retain the mutex. */
		xSemaphoreGive( pxZeroCopyState->xWriteAccessMutex );
	}

	return xReturn;
}
/*-----------------------------------------------------------*/

portBASE_TYPE xIOUtilsReleaseZeroCopyWriteMutex( Peripheral_Control_t *pxPeripheralControl )
{
portBASE_TYPE xReturn;
Transfer_Control_t *pxTransferControlState = pxPeripheralControl->pxTxControl;
Zero_Copy_Tx_State_t *pxZeroCopyState;

	configASSERT( pxTransferControlState );

	pxZeroCopyState = ( Zero_Copy_Tx_State_t * ) ( pxTransferControlState->pvTransferState );
	xReturn = xSemaphoreGive( pxZeroCopyState->xWriteAccessMutex );
	return xReturn;
}
/*-----------------------------------------------------------*/













