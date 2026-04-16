/*
 * HAL TIM shim for emulator.
 */
#ifndef __STM32F4xx_HAL_TIM_H
#define __STM32F4xx_HAL_TIM_H

#include "stm32f4xx.h"
#include "stm32f4xx_hal_def.h"

#ifdef __cplusplus
extern "C" {
#endif

/* TIM Handle */
typedef struct {
    TIM_TypeDef *Instance;
    struct {
        uint32_t Prescaler;
        uint32_t CounterMode;
        uint32_t Period;
        uint32_t ClockDivision;
        uint32_t RepetitionCounter;
        uint32_t AutoReloadPreload;
    } Init;
} TIM_HandleTypeDef;

/* Encoder Init (flat fields matching real HAL) */
typedef struct {
    uint32_t EncoderMode;
    uint32_t IC1Polarity;
    uint32_t IC1Selection;
    uint32_t IC1Prescaler;
    uint32_t IC1Filter;
    uint32_t IC2Polarity;
    uint32_t IC2Selection;
    uint32_t IC2Prescaler;
    uint32_t IC2Filter;
} TIM_Encoder_InitTypeDef;

/* Master Config */
typedef struct {
    uint32_t MasterOutputTrigger;
    uint32_t MasterSlaveMode;
} TIM_MasterConfigTypeDef;

/* Timer constants */
#define TIM_CHANNEL_ALL       0x0000U
#define TIM_COUNTERMODE_UP    0x0000U
#define TIM_CLOCKDIVISION_DIV1 0x0000U
#define TIM_AUTORELOAD_PRELOAD_DISABLE 0x0000U
#define TIM_ENCODERMODE_TI12  0x0003U
#define TIM_ICPOLARITY_RISING 0x0000U
#define TIM_ICSELECTION_DIRECTTI 0x0001U
#define TIM_ICPSC_DIV1        0x0000U
#define TIM_TRGO_ENABLE       0x0040U
#define TIM_TRGO_RESET        0x0000U
#define TIM_MASTERSLAVEMODE_DISABLE 0x0000U

/* Counter access macro - the critical path for reading encoder values */
#define __HAL_TIM_GET_COUNTER(__HANDLE__) ((__HANDLE__)->Instance->CNT)

/* Function prototypes */
HAL_StatusTypeDef HAL_TIM_Encoder_Init(TIM_HandleTypeDef *htim, TIM_Encoder_InitTypeDef *sConfig);
HAL_StatusTypeDef HAL_TIM_Encoder_Start(TIM_HandleTypeDef *htim, uint32_t Channel);
HAL_StatusTypeDef HAL_TIM_Base_Start_IT(TIM_HandleTypeDef *htim);
HAL_StatusTypeDef HAL_TIMEx_MasterConfigSynchronization(TIM_HandleTypeDef *htim, TIM_MasterConfigTypeDef *sMasterConfig);
void HAL_TIM_IRQHandler(TIM_HandleTypeDef *htim);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);

#ifdef __cplusplus
}
#endif

#endif /* __STM32F4xx_HAL_TIM_H */
