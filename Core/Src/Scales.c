//
// Created by stefano on 12/30/22.
//
#include "Scales.h"

HAL_StatusTypeDef initScaleTimer(TIM_HandleTypeDef * timHandle)
{
  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  timHandle->Init.Prescaler = 0;
  timHandle->Init.CounterMode = TIM_COUNTERMODE_UP;
  timHandle->Init.Period = 65535;
  timHandle->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  timHandle->Init.RepetitionCounter = 0;
  timHandle->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_ENABLE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;

  HAL_StatusTypeDef result = HAL_TIM_Encoder_Init(timHandle, &sConfig);
  if (result != HAL_OK) {
    return result;
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;

  result = HAL_TIMEx_MasterConfigSynchronization(timHandle, &sMasterConfig);
  return result;
}
