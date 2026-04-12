/*
 * CMSIS-RTOS v2 API shim for emulator.
 */
#ifndef __CMSIS_OS2_H
#define __CMSIS_OS2_H

#include <stdint.h>
#include "FreeRTOS.h"
#include "task.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Priority levels */
typedef enum {
    osPriorityNone          = 0,
    osPriorityIdle          = 1,
    osPriorityLow           = 8,
    osPriorityLow1          = 8+1,
    osPriorityBelowNormal   = 16,
    osPriorityNormal        = 24,
    osPriorityAboveNormal   = 32,
    osPriorityHigh          = 40,
    osPriorityRealtime      = 48,
    osPriorityISR           = 56,
    osPriorityError         = -1,
    osPriorityReserved      = 0x7FFFFFFF
} osPriority_t;

/* Status codes */
typedef enum {
    osOK                    =  0,
    osError                 = -1,
    osErrorTimeout          = -2,
    osErrorResource         = -3,
    osErrorParameter        = -4,
    osErrorNoMemory         = -5,
    osErrorISR              = -6,
    osStatusReserved        = 0x7FFFFFFF
} osStatus_t;

/* Thread */
typedef void *osThreadId_t;

typedef struct {
    const char *name;
    uint32_t    attr_bits;
    void       *cb_mem;
    uint32_t    cb_size;
    void       *stack_mem;
    uint32_t    stack_size;
    osPriority_t priority;
    uint32_t     tz_module;
    uint32_t     reserved;
} osThreadAttr_t;

typedef void (*osThreadFunc_t)(void *argument);

osThreadId_t osThreadNew(osThreadFunc_t func, void *argument, const osThreadAttr_t *attr);
osThreadId_t osThreadGetId(void);

/* Delay */
osStatus_t osDelay(uint32_t ticks);

/* Message Queue */
typedef void *osMessageQueueId_t;

typedef struct {
    const char *name;
    uint32_t    attr_bits;
    void       *cb_mem;
    uint32_t    cb_size;
    void       *mq_mem;
    uint32_t    mq_size;
} osMessageQueueAttr_t;

osMessageQueueId_t osMessageQueueNew(uint32_t msg_count, uint32_t msg_size, const osMessageQueueAttr_t *attr);

/* Semaphore */
typedef void *osSemaphoreId_t;

typedef struct {
    const char *name;
    uint32_t    attr_bits;
    void       *cb_mem;
    uint32_t    cb_size;
} osSemaphoreAttr_t;

osSemaphoreId_t osSemaphoreNew(uint32_t max_count, uint32_t initial_count, const osSemaphoreAttr_t *attr);

/* Kernel */
osStatus_t osKernelInitialize(void);
osStatus_t osKernelStart(void);

#ifdef __cplusplus
}
#endif

#endif /* __CMSIS_OS2_H */
