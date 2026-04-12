/*
 * FreeRTOS software timer API shim for emulator.
 */
#ifndef INC_TIMERS_H
#define INC_TIMERS_H

#include "FreeRTOS.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef void *TimerHandle_t;
typedef void *xTimerHandle;  /* Legacy name used by Modbus library */

typedef void (*TimerCallbackFunction_t)(TimerHandle_t *pxTimer);

TimerHandle_t xTimerCreate(
    const char *pcTimerName,
    TickType_t xTimerPeriodInTicks,
    UBaseType_t uxAutoReload,
    void *pvTimerID,
    TimerCallbackFunction_t pxCallbackFunction
);

BaseType_t xTimerStart(TimerHandle_t xTimer, TickType_t xBlockTime);
BaseType_t xTimerStop(TimerHandle_t xTimer, TickType_t xBlockTime);
BaseType_t xTimerReset(TimerHandle_t xTimer, TickType_t xBlockTime);
BaseType_t xTimerResetFromISR(TimerHandle_t xTimer, BaseType_t *pxHigherPriorityTaskWoken);

#ifdef __cplusplus
}
#endif

#endif /* INC_TIMERS_H */
