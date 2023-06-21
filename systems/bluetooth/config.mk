# Enables the Uart hardware module to be synthesized
CFG_UART_ENABLE ?= true

# Enables the GPIO hardware module to be synthesized
CFG_GPIO_ENABLE ?= true

# A amount of 32 can be enabled, but the constraint file for the hardware needs to be set up manually
CFG_GPIO_AMOUNT ?= 5