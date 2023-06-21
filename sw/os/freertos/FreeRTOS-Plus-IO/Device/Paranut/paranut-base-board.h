#ifndef PARANUT_BASE_BOARD
#define PARANUT_BASE_BOARD 
//#include "platform.h"
#include "paranut_uart.h"

#define boardAVAILABLE_DEVICES_LIST												\
{																				\
	{ ( const int8_t * const ) "/UART0/", eUART_TYPE, ( void * ) BASE_ADDRESS_UART0 },	\
	{ ( const int8_t * const ) "/UART1/", eUART_TYPE, ( void * ) BASE_ADDRESS_UART1 }	\
}

/*******************************************************************************
 * Map the FreeRTOS+IO interface to the LPC17xx specific functions.
 ******************************************************************************/
portBASE_TYPE vFreeRTOS_paranut_PopulateFunctionPointers( const Peripheral_Types_t ePeripheralType, Peripheral_Control_t * const pxPeripheralControl ); 
#define boardFreeRTOS_PopulateFunctionPointers vFreeRTOS_paranut_PopulateFunctionPointers

#endif /* PARANUT_BASE_BOARD */