#pragma once

/* FreeRTOS includes. */
#include "FreeRTOS.h"



/* Public Functions ----------------------------------------------------------- */
/** @defgroup UART_Public_Functions UART Public Functions
 * @{
 */
/* UART Init/DeInit functions --------------------------------------------------*/
portBASE_TYPE FreeRTOS_UART_init(uint32_t base_addr); 
portBASE_TYPE FreeRTOS_UART_DeInit(uint32_t base_addr);

/*********************************************************************//**
 * base addrees defines for Macro defines for UARTn modules 
 **********************************************************************/
#define BASE_ADDRESS_UART0      0xE0000000
#define BASE_ADDRESS_UART1      0xE0001000

/*********************************************************************//**
 * register offset defines for UARTn module (register map: page 1774 Zynq-7000 reference manual)
 **********************************************************************/
#define XUARTPS_CR_OFFSET       0x00000000
#define XUARTPS_MR_OFFSET       0x00000004
#define XUARTPS_IER_OFFSET      0x00000008
#define XUARTPS_IDR_OFFSET      0x0000000C
#define XUARTPS_IMR_OFFSET      0x00000010
#define XUARTPS_ISR_OFFSET      0x00000014
#define XUARTPS_BAUDGEN_OFFSET  0x00000018
#define XUARTPS_RXTOUT_OFFSET   0x0000001C
#define XUARTPS_RXWM_OFFSET     0x00000020
#define XUARTPS_MODEMCR_OFFSET  0x00000024
#define XUARTPS_MODEMSR_OFFSET  0x00000028
#define XUARTPS_SR_OFFSET       0x0000002C
#define XUARTPS_FIFO_OFFSET     0x00000030
#define Baud_rate_divider_reg   0x00000034
#define Flow_delay_reg          0x00000038
#define Tx_FIFO_trigger_level   0x00000044

typedef struct {
    uint32_t CR;                     // UART Control,                   offset: 0x00, reset value: 0x128
    uint32_t MR;                     // UART Mode,                      offset: 0x04, reset value: 0x000
    uint32_t IER;                    // Interrupt Enable,               offset: 0x08, reset value: 0x000
    uint32_t IDR;                    // Interrupt Disable,              offset: 0x0C, reset value: 0x000
    uint32_t IMR;                    // Interrupt Mask,                 offset: 0x10, reset value: 0x000
    uint32_t ISR;                    // Channel Interrupt Status,       offset: 0x14, reset value: 0x000
    uint32_t BAUDGEN;                // Baud Rate Generator,            offset: 0x18, reset value: 0x28B
    uint32_t RXTOUT;                 // Receiver Timeout,               offset: 0x1C, reset value: 0x000
    uint32_t RXWM;                   // Receiver FIFO Trigger Level,    offset: 0x20, reset value: 0x020
    uint32_t MODEMCR;                // Modem Control,                  offset: 0x24, reset value: 0x000
    uint32_t MODEMSR;                // Modem Status,                   offset: 0x28, reset value: -
    uint32_t SR;                     // Channel Status,                 offset: 0x2C, reset value: 0x000
    uint32_t FIFO;                   // Transmit and Receive FIFO,      offset: 0x30, reset value: 0x000
    uint32_t divider_reg0;           // Baud Rate Divider,              offset: 0x34, reset value: 0x00F
    uint32_t Flow_delay_reg0;        // Flow Control Delay,             offset: 0x38, reset value: 0x000 
    uint32_t Tx_FIFO_trigger_level0; // Transmitter FIFO Trigger Level, offset: 0x44, reset value: 0x020 
} UARTn_register_t;

/*********************************************************************//**
 * defines for bitmasks for UARTn module (only necessary ones)
 **********************************************************************/
#define SR_TXFULL 	    (1ULL << 4)
#define SR_TXEMPTY 	    (1ULL << 3)
#define SR_RXFULL 	    (1ULL << 2)
#define SR_RXEMPTY 	    (1ULL << 1)