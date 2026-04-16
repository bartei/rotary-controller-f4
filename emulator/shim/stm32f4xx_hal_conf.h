/*
 * HAL configuration shim for emulator.
 */
#ifndef __STM32F4xx_HAL_CONF_H
#define __STM32F4xx_HAL_CONF_H

#define HAL_MODULE_ENABLED
#define HAL_GPIO_MODULE_ENABLED
#define HAL_TIM_MODULE_ENABLED
#define HAL_UART_MODULE_ENABLED
#define HAL_DMA_MODULE_ENABLED
#define HAL_RCC_MODULE_ENABLED
#define HAL_CORTEX_MODULE_ENABLED

/* Oscillator values (not used in emulator, but referenced by some HAL macros) */
#define HSE_VALUE    8000000U
#define HSI_VALUE    16000000U
#define LSE_VALUE    32768U
#define LSI_VALUE    32000U

/* Tick frequency */
#define HAL_TICK_FREQ_1KHZ 1U

#endif /* __STM32F4xx_HAL_CONF_H */
