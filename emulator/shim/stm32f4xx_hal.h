/*
 * Main HAL header shim for emulator.
 * This file is what the firmware includes as "stm32f4xx_hal.h".
 * It pulls in all the sub-module shim headers.
 */
#ifndef __STM32F4xx_HAL_H
#define __STM32F4xx_HAL_H

#include <stdint.h>
#include <stdbool.h>

#include "stm32f4xx_hal_conf.h"
#include "stm32f4xx_hal_def.h"
#include "stm32f4xx.h"
#include "stm32f4xx_hal_gpio.h"
#include "stm32f4xx_hal_dma.h"
#include "stm32f4xx_hal_tim.h"
#include "stm32f4xx_hal_uart.h"
#include "stm32f4xx_hal_rcc.h"
#include "stm32f4xx_hal_cortex.h"

#ifdef __cplusplus
extern "C" {
#endif

/* HAL Init/Tick functions */
HAL_StatusTypeDef HAL_Init(void);
void HAL_IncTick(void);
uint32_t HAL_GetTick(void);

#ifdef __cplusplus
}
#endif

#endif /* __STM32F4xx_HAL_H */
