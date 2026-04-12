/*
 * FreeRTOS kernel shim for emulator.
 * Provides base types used throughout the Modbus library.
 */
#ifndef INC_FREERTOS_H
#define INC_FREERTOS_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef int32_t  BaseType_t;
typedef uint32_t UBaseType_t;
typedef uint32_t TickType_t;

#define pdFALSE  ((BaseType_t)0)
#define pdTRUE   ((BaseType_t)1)
#define pdPASS   pdTRUE
#define pdFAIL   pdFALSE

#define portMAX_DELAY ((TickType_t)0xFFFFFFFFUL)

#define configTICK_RATE_HZ 1000U

/* portYIELD_FROM_ISR - no-op in emulator (no interrupt priority to manage) */
#define portYIELD_FROM_ISR(xHigherPriorityTaskWoken) (void)(xHigherPriorityTaskWoken)

#ifdef __cplusplus
}
#endif

#endif /* INC_FREERTOS_H */
