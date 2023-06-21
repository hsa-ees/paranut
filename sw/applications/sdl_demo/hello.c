#include "FreeRTOS.h"
#include "task.h"
#include "SDL.h"
#include "gfx_hello_world.h"
#include "vears.h"

extern int pong(void);

/** @brief External Interrupt Handler used by FreeRTOS.
 * 
 * In case an interrupt has occured, FreeRTOS will jump to this function.
 * This happens by defining the portasmHANDLE_INTERRUPT to this function in the Makefile.
 */
void external_interrupt_handler( unsigned int cause ) {
    
}

/** @brief Main function called by startup code */
int main() {
    // Prints a "Hello World" to the serial console via "tohost"/"fromhost" interface
    // This is done by newlib calling the "_write" function in syscalls.c
    puts("Hello World\n");
    
    // Rest of setup in here (see pong.c file)
    pong();
    
    // Finally, start the scheduler
    vTaskStartScheduler();
} 
