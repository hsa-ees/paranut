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

#ifndef FREERTOS_DRIVER_INTERFACE_H
#define FREERTOS_DRIVER_INTERFACE_H

/* Standard includes. */
#include <stdint.h>

/* FreeRTOS+IO includes. */
#include "FreeRTOSIOConfig.h"

/* The peripherals the IO library can interface to. */
typedef enum
{
	eUART_TYPE = 0,
	eSSP_TYPE,
	eI2C_TYPE
} Peripheral_Types_t;

/* The structure that defines the peripherals that are available for use on
any particular supported board. */
typedef struct xAVAILABLE_DEVICES
{
	const int8_t * const pcPath;				/* Text name of the peripheral.  For example, "/UART0/", or "/SPI2/". */
	const Peripheral_Types_t xPeripheralType;	/* The type of the peripheral, as defined by the Peripheral_Types_t enum. */
	const void *pvBaseAddress;					/* The base address of the peripheral in the microcontroller memory map. */
} Available_Peripherals_t;


/* The base transfer control structure, that points to the actual, IO method
specific structure. */
typedef struct xTRANSFER_CONTROL
{
	void *pvTransferState;	/* Pointer to a structure that has the correct members in accordance with the ucType member. */
	uint8_t ucType;				/* The value of the ioctl command used to set up the type of IO method. */
} Transfer_Control_t;


/* Peripheral handles are void * for data hiding purposes. */
typedef const void * Peripheral_Descriptor_t;

/* Types that define valid read(), write() and ioctl() functions. */
typedef size_t ( *Peripheral_write_Function_t )( Peripheral_Descriptor_t const pxPeripheral, const void *pvBuffer, const size_t xBytes );
typedef size_t ( *Peripheral_read_Function_t )( Peripheral_Descriptor_t const pxPeripheral, void * const pvBuffer, const size_t xBytes );
typedef portBASE_TYPE ( *Peripheral_ioctl_Function_t )( Peripheral_Descriptor_t const pxPeripheral, uint32_t ulRequest, void *pvValue );

/* Structure containing control information for an open peripheral. */
typedef struct xPERIPHREAL_CONTROL
{
	Peripheral_write_Function_t write;			/* The function used to write to the peripheral. */
	Peripheral_read_Function_t read;			/* The function used to read from the peripheral. */
	Peripheral_ioctl_Function_t ioctl;			/* The function used for ioctl access to the peripheral. */
	Transfer_Control_t *pxTxControl;			/* Pointer to the transfer control structure used to manage transmissions through the peripheral. */
	Transfer_Control_t *pxRxControl;			/* Pointer to the transfer control structure used to manage receptions from the peripheral. */
	const Available_Peripherals_t *pxDevice;	/* Pointer to the structure that defines the name and base address of the open peripheral. */
	int8_t cPeripheralNumber;					/* Where more than one peripheral of the same kind is available, this holds the number of the peripheral this structure is used to control. */
} Peripheral_Control_t;


/*
 * Commands that can be sent to peripherals using the ioctl() function.
 */

 /* Set transfer mode ioctl() requests. */
#define ioctlUSE_POLLED_TX					0
#define ioctlUSE_POLLED_RX					1
#define ioctlUSE_ZERO_COPY_TX				2
#define ioctlUSE_CHARACTER_QUEUE_TX			3
#define ioctlUSE_CHARACTER_QUEUE_RX			4
#define ioctlUSE_CIRCULAR_BUFFER_RX			5

/* Transfer mode related ioctl() requests. */
#define ioctlOBTAIN_WRITE_MUTEX				10
#define ioctlWAIT_PREVIOUS_WRITE_COMPLETE	11
#define ioctlRELEASE_WRITE_MUTEX			12
#define ioctlSET_TX_TIMEOUT					13
#define ioctlSET_RX_TIMEOUT					14
#define ioctlCLEAR_RX_BUFFER				15

/* Generic peripheral ioctl requests. */
#define ioctlSET_SPEED						100
#define ioctlSET_INTERRUPT_PRIORITY			102

/* SPI specific ioctl requests. */
#define ioctlSET_SPI_DATA_BITS				200
#define ioctlSET_SPI_CLOCK_PHASE			201
#define ioctlSET_SPI_CLOCK_POLARITY			202
#define ioctlSET_SPI_MODE					203

/* I2C specific ioctl requests. */
#define ioctlSET_I2C_SLAVE_ADDRESS			300

/* Private ioctl requests. */
#define ioctlUSE_INTERRUPTS					101

/*
 * Peripheral control structure access macros.
 */
#define diGET_PERIPHERAL_BASE_ADDRESS( pxPeripheralControl ) ( ( pxPeripheralControl )->pxDevice->pvBaseAddress )
#define diGET_PERIPHERAL_NUMBER( pxPeripheralControl ) ( ( pxPeripheralControl )->cPeripheralNumber )
#define diGET_TX_TRANSFER_TYPE( pxPeripheralControl ) ( ( pxPeripheralControl )->pxTxControl->ucType )
#define diGET_RX_TRANSFER_TYPE( pxPeripheralControl ) ( ( pxPeripheralControl )->pxRxControl->ucType )
#define diGET_TX_TRANSFER_STRUCT( pxPeripheralControl ) ( ( pxPeripheralControl )->pxTxControl )
#define diGET_RX_TRANSFER_STRUCT( pxPeripheralControl ) ( ( pxPeripheralControl )->pxRxControl )
#define diGET_TX_TRANSFER_STATE( pxPeripheralControl ) ( ( pxPeripheralControl )->pxTxControl->pvTransferState )
#define diGET_RX_TRANSFER_STATE( pxPeripheralControl ) ( ( pxPeripheralControl )->pxRxControl->pvTransferState )
#define diGET_TRANSFER_TYPE_FROM_CONTROL_STRUCT( pxTransferControl ) pxTransferControl->ucType

/*
 * Function prototypes.
 */
Peripheral_Descriptor_t FreeRTOS_open( const int8_t *pcPath, const uint32_t ulFlags );
portBASE_TYPE FreeRTOS_ioctl( Peripheral_Descriptor_t const xPeripheral, uint32_t ulRequest, void *pvValue );

/*
 * Macros for the functions that are really macros to keep the call depth down
 * and the efficiency up.
 */
#define FreeRTOS_write( xPeripheral, pvBuffer, xBytes ) 	( ( Peripheral_Control_t * ) xPeripheral )->write( ( ( Peripheral_Control_t * ) xPeripheral ), ( pvBuffer ), ( xBytes ) )
#define FreeRTOS_read( xPeripheral, pvBuffer, xBytes ) 		( ( Peripheral_Control_t * ) xPeripheral )->read( ( ( Peripheral_Control_t * ) xPeripheral ), ( pvBuffer ), ( xBytes ) )


#endif




