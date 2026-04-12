/*
 * FreeRTOS queue API shim for emulator.
 */
#ifndef INC_QUEUE_H
#define INC_QUEUE_H

#include "FreeRTOS.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef void *QueueHandle_t;

QueueHandle_t xQueueCreate(UBaseType_t uxQueueLength, UBaseType_t uxItemSize);
BaseType_t xQueueSendToBack(QueueHandle_t xQueue, const void *pvItemToQueue, TickType_t xTicksToWait);
BaseType_t xQueueSendToFront(QueueHandle_t xQueue, const void *pvItemToQueue, TickType_t xTicksToWait);
BaseType_t xQueueReceive(QueueHandle_t xQueue, void *pvBuffer, TickType_t xTicksToWait);
BaseType_t xQueueReset(QueueHandle_t xQueue);

#ifdef __cplusplus
}
#endif

#endif /* INC_QUEUE_H */
