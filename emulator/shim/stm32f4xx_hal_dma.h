/*
 * HAL DMA shim for emulator.
 * Minimal stubs - DMA is not used in USART_HW mode.
 */
#ifndef __STM32F4xx_HAL_DMA_H
#define __STM32F4xx_HAL_DMA_H

#include "stm32f4xx.h"
#include "stm32f4xx_hal_def.h"

#ifdef __cplusplus
extern "C" {
#endif

#define DMA_IT_HT 0x00000008U

typedef struct {
    DMA_Stream_TypeDef *Instance;
} DMA_HandleTypeDef;

/* Macro used in UARTCallback.c to disable half-transfer interrupt */
#define __HAL_DMA_DISABLE_IT(__HANDLE__, __INTERRUPT__) \
    do { (void)(__HANDLE__); (void)(__INTERRUPT__); } while(0)

#ifdef __cplusplus
}
#endif

#endif /* __STM32F4xx_HAL_DMA_H */
