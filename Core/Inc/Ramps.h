/**
 * Copyright © 2022 <Stefano Bertelli>
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software
 * and associated documentation files (the “Software”), to deal in the Software without restriction,
 * including without limitation the rights to use, copy, modify, merge, publish, distribute,
 * sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies or substantial
 * portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT
 * LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */
#ifndef THIRD_PARTY_RAMPS_H_
#define THIRD_PARTY_RAMPS_H_

#include <stdint.h>
#include "stm32f4xx_hal.h"
#include "cmsis_os2.h"
#include "Modbus.h"
#include "Scales.h"

#define MODBUS_ADDRESS 17
#define RAMPS_CLOCK_FREQUENCY 1000000
#define DIR_PIN GPIO_PIN_7
#define DIR_GPIO_PORT GPIOA


typedef enum
{
    MODE_HALT = 0,
    MODE_INDEX = 10,
    MODE_INDEX_INIT = 11,
    MODE_SYNCHRO = 20,
    MODE_SYNCHRO_INIT = 21,
    MODE_JOG = 30,
    MODE_JOG_FW = 31,
    MODE_JOG_BW = 32,
    MODE_SET_ENCODER = 40,
    MODE_SYNCHRO_BAD_RATIO = 101,
} ramps_mode_t ;

typedef struct {
    ramps_mode_t mode;
    int32_t currentPosition;
    int32_t finalPosition;
    int16_t unused_1;
    int32_t encoderPosition;
    uint16_t encoderPresetIndex;
    int32_t encoderPresetValue;
    int32_t unused_4;
    float maxSpeed;
    float minSpeed;
    float currentSpeed;
    float acceleration;
    int32_t stepRatioNum;
    int32_t stepRatioDen;
    float unused_5;
    int32_t synRatioNum;
    int32_t synRatioDen;
    int32_t synOffset;
    uint16_t synScaleIndex;
    int32_t scalesPosition[SCALES_COUNT];
} rampsSharedData_t;

// This following structure is used only internally and it's not shared with the modbus data
// The values stored here are used by the adapted bresenham algorithm used to interpolate the
// Y axis with the X axis, where X is the encoder position returned by the master axis and the
// y is the controlled motion axis.
typedef struct {
    uint16_t encoderPrevious, encoderCurrent;
    int16_t encoderDelta;
    int32_t positionPrevious, positionCurrent;
    int32_t yi;
    int32_t D;
    int8_t direction;
} rampsSyncData_t;

typedef struct {
    float floatAccelInterval;
    float stepRatio;
    int32_t currentStep;
    int32_t totalSteps;
    int32_t decelSteps;
} rampsIndexData_t;

typedef struct {
    // Modbus shared data
    rampsSharedData_t shared;
    rampsSyncData_t syncData;
    rampsIndexData_t indexData;

    // Scales Data
    scales_t scales;

    // STM32 Related
    TIM_HandleTypeDef * motorTimer;
    TIM_HandleTypeDef * synTimer;
    UART_HandleTypeDef * modbusUart;

    GPIO_TypeDef * directionPinPort;
    uint16_t directionPin;

    GPIO_TypeDef * enablePinPort;
    uint16_t enablePin;

    // FreeRTOS related
    osThreadId_t TaskRampsHandle;
} rampsHandler_t;

void RampsStart(rampsHandler_t * rampsData);
void RampsMotionIsr(rampsHandler_t * data);
void SyncMotionIsr(rampsHandler_t * data);
void SyncMotionInit(rampsHandler_t * data);
void startRampsTask(rampsHandler_t * rampsData);
void RampsTask(void *argument);

#endif
