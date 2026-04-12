/*
 * HAL Cortex shim for emulator.
 */
#ifndef __STM32F4xx_HAL_CORTEX_H
#define __STM32F4xx_HAL_CORTEX_H

#include "stm32f4xx_hal_def.h"
#include "core_cm4.h"

#ifdef __cplusplus
extern "C" {
#endif

/* All NVIC functions are already defined in core_cm4.h */

static inline uint32_t HAL_SYSTICK_Config(uint32_t TicksNumb) { (void)TicksNumb; return 0; }

#ifdef __cplusplus
}
#endif

#endif /* __STM32F4xx_HAL_CORTEX_H */
