/*
 * FreeRTOS semaphore API shim for emulator.
 */
#ifndef INC_SEMPHR_H
#define INC_SEMPHR_H

#include "FreeRTOS.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef void *SemaphoreHandle_t;

SemaphoreHandle_t xSemaphoreCreateBinary(void);
SemaphoreHandle_t xSemaphoreCreateCounting(UBaseType_t uxMaxCount, UBaseType_t uxInitialCount);
BaseType_t xSemaphoreTake(SemaphoreHandle_t xSemaphore, TickType_t xBlockTime);
BaseType_t xSemaphoreGive(SemaphoreHandle_t xSemaphore);

#ifdef __cplusplus
}
#endif

#endif /* INC_SEMPHR_H */
