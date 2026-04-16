/*
 * FreeRTOS task API shim for emulator.
 */
#ifndef INC_TASK_H
#define INC_TASK_H

#include "FreeRTOS.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Task handle is an opaque pointer to our shim's internal structure */
typedef void *TaskHandle_t;

/* Notify actions */
typedef enum {
    eNoAction = 0,
    eSetBits,
    eIncrement,
    eSetValueWithOverwrite,
    eSetValueWithoutOverwrite
} eNotifyAction;

/*
 * Task notification API.
 * These are implemented in freertos_shim.cpp using condition variables.
 */
BaseType_t xTaskNotify(TaskHandle_t xTaskToNotify, uint32_t ulValue, eNotifyAction eAction);
BaseType_t xTaskNotifyFromISR(TaskHandle_t xTaskToNotify, uint32_t ulValue, eNotifyAction eAction, BaseType_t *pxHigherPriorityTaskWoken);
uint32_t ulTaskNotifyTake(BaseType_t xClearCountOnExit, TickType_t xTicksToWait);

/* Get current task handle (returns the calling thread's task handle) */
TaskHandle_t xTaskGetCurrentTaskHandle(void);

#ifdef __cplusplus
}
#endif

#endif /* INC_TASK_H */
