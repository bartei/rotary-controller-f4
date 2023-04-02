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
#define DIR_PIN GPIO_PIN_8
#define DIR_GPIO_PORT GPIOB
#define ENA_PIN GPIO_PIN_9
#define ENA_GPIO_PORT GPIOB

typedef struct
{
    unsigned enable: 1;
    unsigned rq_set_encoder: 1;
    unsigned rq_synchro_mode: 1;
    unsigned rq_index_mode: 1;
} ramps_control_flags ;

typedef struct
{
  unsigned sta_ready : 1;
  unsigned sta_forward :1;
  unsigned sta_reverse :1;
  unsigned error_bad_ratio: 1;
  unsigned ack_set_encoder : 1;
  unsigned ack_synchro_mode : 1;
  unsigned ack_index_mode : 1;
  unsigned ack_jog_mode : 1;
  unsigned index_mode : 1;
  unsigned synchro_mode : 1;
} ramps_status_flags;

typedef struct {
    ramps_control_flags control; // 0
    ramps_status_flags status; // 2
    int32_t currentPosition; // 4
    int32_t finalPosition; // 6
    int32_t unused_8; // 8
    uint16_t encoderPresetIndex; // 10
    int32_t encoderPresetValue; // 12
    int32_t unused_14; // 14
    float maxSpeed; // 16
    float minSpeed; // 18
    float currentSpeed; // 20
    float acceleration; // 22
    int32_t stepRatioNum; // 24
    int32_t stepRatioDen; // 26
    float unused_28; // 28
    int32_t synRatioNum; // 30
    int32_t synRatioDen; // 32
    int32_t synOffset; // 34
    uint16_t synScaleIndex; // 36
    int32_t scalesPosition[SCALES_COUNT]; // 38
} rampsSharedData_t;

// This following structure is used only internally and it's not shared with the modbus data
// The values stored here are used by the adapted bresenham algorithm used to interpolate the
// Y axis with the X axis, where X is the encoder position returned by the master axis and the
// y is the controlled motion axis.
typedef struct {
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
