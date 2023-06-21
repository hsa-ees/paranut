#include "FreeRTOS.h"
#include "task.h"
#include "platform.h"

#include "semphr.h"
#include "paranut.h"

// Declaration of puts to avoid inclusion of header
extern int puts(const char *str);

/** @brief External Interrupt Handler used by FreeRTOS.
 * 
 * In case an interrupt has occured, FreeRTOS will jump to this function.
 * This happens by defining the portasmHANDLE_INTERRUPT to this function in the Makefile.
 */
void external_interrupt_handler( unsigned int cause ) {

}

void task1_entry(void *pvParamaters) {
    const char msg_task_1[] = "!!Hello from task 1!!\n\r";
    for(;;) {
        puts(msg_task_1);
        vTaskDelay(2);
    }
}

void task2_entry(void *pvParamaters) {    
    for(;;) {
        // Entering the trap handler is (currently) not allowed while setting up CoPUs
        taskENTER_CRITICAL();
        int cid = PN_BEGIN_LINKED(CFG_NUT_CPU_CORES);
        taskEXIT_CRITICAL();
        for(;;) {
            puts("Hello from task 2\n\r");
        }
        taskENTER_CRITICAL();
        int end_threaded = pn_end_linked();
        taskEXIT_CRITICAL();
    }
}

int main() {
    TaskHandle_t xHandle;
    BaseType_t xReturned;

    // Create two simple tasks
    xReturned = xTaskCreate(task1_entry, "Task1", configMINIMAL_STACK_SIZE*3, (void*)1, tskIDLE_PRIORITY+2, &xHandle);
    xReturned = xTaskCreate(task2_entry, "Task2", configMINIMAL_STACK_SIZE*3, (void*)1, tskIDLE_PRIORITY+1, &xHandle);

    // Give control to the scheduler
    vTaskStartScheduler();
    puts("WARNING: Scheduler returned!");
} 
