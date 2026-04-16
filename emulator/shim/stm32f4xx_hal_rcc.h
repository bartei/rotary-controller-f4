/*
 * HAL RCC shim for emulator. All clock enable macros are no-ops.
 */
#ifndef __STM32F4xx_HAL_RCC_H
#define __STM32F4xx_HAL_RCC_H

#include "stm32f4xx_hal_def.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Clock enable macros - no-ops in emulator */
#define __HAL_RCC_GPIOA_CLK_ENABLE()  do { } while(0)
#define __HAL_RCC_GPIOB_CLK_ENABLE()  do { } while(0)
#define __HAL_RCC_GPIOC_CLK_ENABLE()  do { } while(0)
#define __HAL_RCC_GPIOD_CLK_ENABLE()  do { } while(0)
#define __HAL_RCC_TIM1_CLK_ENABLE()   do { } while(0)
#define __HAL_RCC_TIM2_CLK_ENABLE()   do { } while(0)
#define __HAL_RCC_TIM3_CLK_ENABLE()   do { } while(0)
#define __HAL_RCC_TIM4_CLK_ENABLE()   do { } while(0)
#define __HAL_RCC_TIM9_CLK_ENABLE()   do { } while(0)
#define __HAL_RCC_TIM11_CLK_ENABLE()  do { } while(0)
#define __HAL_RCC_USART1_CLK_ENABLE() do { } while(0)
#define __HAL_RCC_PWR_CLK_ENABLE()    do { } while(0)
#define __HAL_RCC_DMA1_CLK_ENABLE()   do { } while(0)
#define __HAL_RCC_DMA2_CLK_ENABLE()   do { } while(0)

/* RCC oscillator and clock config types (for main.c SystemClock_Config - we stub these) */
typedef struct {
    uint32_t OscillatorType;
    uint32_t HSEState;
    uint32_t HSIState;
    uint32_t LSEState;
    uint32_t LSIState;
    struct {
        uint32_t PLLState;
        uint32_t PLLSource;
        uint32_t PLLM;
        uint32_t PLLN;
        uint32_t PLLP;
        uint32_t PLLQ;
    } PLL;
} RCC_OscInitTypeDef;

typedef struct {
    uint32_t ClockType;
    uint32_t SYSCLKSource;
    uint32_t AHBCLKDivider;
    uint32_t APB1CLKDivider;
    uint32_t APB2CLKDivider;
} RCC_ClkInitTypeDef;

/* RCC constants */
#define RCC_OSCILLATORTYPE_HSE  0x01U
#define RCC_HSE_ON              0x01U
#define RCC_PLL_ON              0x02U
#define RCC_PLLSOURCE_HSE      0x00400000U
#define RCC_PLLP_DIV2           0x00000002U
#define RCC_CLOCKTYPE_HCLK     0x02U
#define RCC_CLOCKTYPE_SYSCLK   0x01U
#define RCC_CLOCKTYPE_PCLK1    0x04U
#define RCC_CLOCKTYPE_PCLK2    0x08U
#define RCC_SYSCLKSOURCE_PLLCLK 0x08U
#define RCC_SYSCLK_DIV1        0x00U
#define RCC_HCLK_DIV1          0x00U
#define RCC_HCLK_DIV2          0x00001000U

#define FLASH_LATENCY_3        0x03U
#define PWR_REGULATOR_VOLTAGE_SCALE1 0x0000C000U

#define __HAL_PWR_VOLTAGESCALING_CONFIG(__REGULATOR__) do { (void)(__REGULATOR__); } while(0)

HAL_StatusTypeDef HAL_RCC_OscConfig(RCC_OscInitTypeDef *RCC_OscInitStruct);
HAL_StatusTypeDef HAL_RCC_ClockConfig(RCC_ClkInitTypeDef *RCC_ClkInitStruct, uint32_t FLatency);

#ifdef __cplusplus
}
#endif

#endif /* __STM32F4xx_HAL_RCC_H */
