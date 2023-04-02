//
// Created by stefano on 12/30/22.
//
#include "Scales.h"

void initScaleTimer(TIM_HandleTypeDef * timHandle)
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
}


/**
 *
 * @param data scales data structure
 * This method updates the current position for all the encoders configured in the system
 */
void updateScales(scales_t * data) {
  for (int i = 0; i < SCALES_COUNT; i++) {
    data->scalePosition[i].encoderPrevious = data->scalePosition[i].encoderCurrent;
    data->scalePosition[i].encoderCurrent = __HAL_TIM_GET_COUNTER(data->scaleTimer[i]);
    data->scalePosition[i].encoderDelta = data->scalePosition[i].encoderCurrent - data->scalePosition[i].encoderPrevious;
    data->scalePosition[i].positionPrevious = data->scalePosition[i].positionCurrent;
    data->scalePosition[i].positionCurrent += data->scalePosition[i].encoderDelta;
  }
}

/**
 *
 * @param data Scales Structure
 * This method inits and starts all the timers for the encoders position detection.
 */
void startScalesTimers(scales_t * data) {
  for (int j = 0; j < SCALES_COUNT; ++j) {
    initScaleTimer(data->scaleTimer[j]);
    HAL_TIM_Encoder_Start(data->scaleTimer[j], TIM_CHANNEL_ALL);
  }
}
