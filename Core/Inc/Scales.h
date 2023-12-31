//
// Created by stefano on 12/30/22.
//

#ifndef ROTARY_CONTROLLER_F4_SCALES_H
#define ROTARY_CONTROLLER_F4_SCALES_H
#include "stm32f4xx_hal.h"

#define SCALES_COUNT 4

HAL_StatusTypeDef initScaleTimer(TIM_HandleTypeDef * timHandle);
#endif //ROTARY_CONTROLLER_F4_SCALES_H
