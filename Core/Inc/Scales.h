//
// Created by stefano on 12/30/22.
//

#ifndef ROTARY_CONTROLLER_F4_SCALES_H
#define ROTARY_CONTROLLER_F4_SCALES_H
#include "stm32f4xx_hal.h"

#define SCALES_COUNT 4

typedef struct {
    uint16_t encoderPrevious, encoderCurrent;
    int16_t encoderDelta;
    int32_t positionPrevious, positionCurrent;
} scalePosition_t;

typedef struct {
    TIM_HandleTypeDef * scaleTimer[SCALES_COUNT];
    scalePosition_t scalePosition[SCALES_COUNT];
} scales_t;

void initScaleTimer(TIM_HandleTypeDef * timHandle);
void updateScales(scales_t data);
void startScalesTimers(scales_t data);
#endif //ROTARY_CONTROLLER_F4_SCALES_H
