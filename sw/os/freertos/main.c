#include "FreeRTOS.h"
#include "task.h"
#include "platform.h"

#include "semphr.h"
#include "FreeRTOS_DriverInterface.h"
#include "paranut.h"

#include <stdio.h>

// This UART implementation requires that ParaNut has access to the system bus
#define USE_FREERTOS_UART 0
#define USE_HEAP5_SCHEME 0
#define USE_CUSTOM_PUTS 0

// Declaration of puts to avoid inclusion of header
extern int puts(const char *str);

#if USE_CUSTOM_PUTS
// Same here, defined in syscalls.c
extern int _write(int file, char *ptr, int len);

// Example for puts without newlib
// doesn't automatically print a new line
int puts_custom(const char *str) {
    size_t length = 0;
    for(const char *c=&str[0];*c!='\0';c++) {
        length++;
    }
    // Our _write wont touch str
    _write(0, (char *)str, length);
}

#define puts puts_custom
#endif

/** @brief External Interrupt Handler used by FreeRTOS.
 * 
 * In case an interrupt has occured, FreeRTOS will jump to this function.
 * This happens by defining the portasmHANDLE_INTERRUPT to this function in the Makefile.
 */
void external_interrupt_handler( unsigned int cause ) {

}

static Peripheral_Descriptor_t uart_device_ctl;

static inline int set_a0(int content) {
	//register uint32_t *store_ptr asm ("t0") = (uint32_t*)content;
	asm volatile ( "mv a0, %0\n" : : "r" (content) : "a0" );
	return content;
}

// Used for testing if a0 was correctly restored between task-switches
static int a0_array[CFG_NUT_CPU_CORES];

void task1_entry(void *pvParamaters) {
    const char msg_task_1[] = "Hello from task 1\n\r";
    for(;;) {
#if USE_FREERTOS_UART == 1
        FreeRTOS_write(uart_device_ctl, msg_task_1, sizeof(msg_task_1)-1);
#else
        puts(msg_task_1);
#endif
        vTaskDelay(2);
    }
}

void task2_entry(void *pvParamaters) {
    // Entering the trap handler is (currently) not allowed while setting up CoPUs
    taskENTER_CRITICAL();
    int cid = PN_BEGIN_LINKED(CFG_NUT_CPU_CORES);
    taskEXIT_CRITICAL();
    for(;;) {
#if USE_FREERTOS_UART == 1
        FreeRTOS_write(uart_device_ctl, msg_task_2, sizeof(msg_task_2)-1);
#else
        // Assembly trickery to try avoid usage of s0 for array
        asm volatile("move a3, %0\n\t"
                    :
                    : "r" (a0_array));
        puts("Hello from Task 2\n\r");
        set_a0(0x11 * (cid+3));
        // Purposefully waste cycles to let the scheduler hit
        for(unsigned int i = 0; i < 128; i++) {
            asm volatile ( "mv t0, t0\n" );
        }
        unsigned int a0_val = 0;
        asm volatile ( "mv %0, a0\necall" : "=r" (a0_val) );
        asm volatile ( "mv a0, %0\n" : : "r" (a0_val) : "a0" );
        a0_array[cid] = a0_val;
        for(unsigned int i = 0; i < CFG_NUT_CPU_CORES; i++) {
            if(a0_array[i] != 0x11 * (i+3)) {
                puts("BUG: a0 got modified by task-switch");
                printf("\n\rExpected: %x \tgot: %x\n\r@: %x\n\r", 0x11 * (i+3), a0_array[i], &a0_array[i]);
            }
        }
#endif
    }
    taskENTER_CRITICAL();
    int end_threaded = pn_end_linked();
    taskEXIT_CRITICAL();
}

#if USE_HEAP5_SCHEME
static char usable_heap5_region[configTOTAL_HEAP_SIZE];

const HeapRegion_t xHeapRegions[] = {
    { (uint8_t *)usable_heap5_region, configTOTAL_HEAP_SIZE },
    { NULL, 0 }
};
#endif

int main() {
#if USE_HEAP5_SCHEME
    vPortDefineHeapRegions(xHeapRegions);
#endif
    TaskHandle_t xHandle;
    BaseType_t xReturned;

    //task2_entry(NULL);
#if USE_FREERTOS_UART == 1
    uart_device_ctl = FreeRTOS_open("/UART1/", 0);
    if(uart_device_ctl == NULL) {
        puts("Failed to open UART1\n");
        return 0;
    }
    puts("Successfully opened UART1\n");
#endif

    // Create two simple tasks
    xReturned = xTaskCreate(task1_entry, "Task1", configMINIMAL_STACK_SIZE*3, (void*)1, tskIDLE_PRIORITY+2, &xHandle);
    if(xReturned == pdPASS) {
        //puts("Successfully created TASK1\n");
    }

    xReturned = xTaskCreate(task2_entry, "Task2", configMINIMAL_STACK_SIZE*3, (void*)1, tskIDLE_PRIORITY+1, &xHandle);
    if(xReturned == pdPASS) {
        //puts("Successfully created TASK2\n");
    }

    // Give control to the scheduler
    vTaskStartScheduler();
    puts("WARNING: Scheduler returned!");
} 
