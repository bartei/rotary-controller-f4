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

#define STEP_PIN GPIO_PIN_0
#define STEP_GPIO_PORT GPIOA

#define DIR_PIN GPIO_PIN_14
#define DIR_GPIO_PORT GPIOB

#define ENA_PIN GPIO_PIN_15
#define ENA_GPIO_PORT GPIOB
#define ENA_DELAY_MS 500

#define USR_LED_Pin GPIO_PIN_12
#define USR_LED_GPIO_Port GPIOB


typedef struct {
  int32_t scaledDelta;
  int32_t error;
  int32_t oldPosition;
} deltaPosError_t;

typedef enum : uint16_t {
  linear = 0,
  spindle = 1
} input_mode_t;

typedef struct {
  TIM_HandleTypeDef *timerHandle;
  uint16_t encoderPrevious;
  uint16_t encoderCurrent;
  int32_t ratioNum;
  int32_t ratioDen;
  int32_t maxValue;
  int32_t minValue;
  int32_t position;
  int32_t speed;
  int32_t error;
  int32_t syncRatioNum, syncRatioDen;
  bool syncMotion;
  input_mode_t mode;
} input_t;

typedef struct {
  float minSpeed;
  float maxSpeed;
  float currentSpeed;
  float acceleration;
  float absoluteOffset;
  float indexOffset;
  float unused_1;
  float desiredPosition;
  float currentPosition;
  int32_t currentSteps;
  int32_t desiredSteps;
  int32_t ratioNum;
  int32_t ratioDen;
  int32_t unused_2;
  int32_t unused_3;
  float unused_4;
  float estimatedSpeed;
  float allowedError;
} servo_t;

typedef struct {
  int32_t divisions;
  int32_t index;
} index_t;

typedef struct {
  float servoCurrent;
  float servoDesired;
  float servoSpeed;
  int32_t scaleCurrent[SCALES_COUNT];
  int32_t scaleSpeed[SCALES_COUNT];
  uint32_t cycles;
} fastData_t;

typedef struct {
  uint32_t execution_interval;
  uint32_t execution_interval_previous;
  uint32_t execution_interval_current;
  uint32_t execution_cycles;
  index_t index;
  servo_t servo;
  input_t scales[SCALES_COUNT];
  fastData_t fastData;
} rampsSharedData_t;


typedef struct {
  // Modbus shared data
  rampsSharedData_t shared;

  // STM32 Related
  TIM_HandleTypeDef *synchroRefreshTimer;
  UART_HandleTypeDef *modbusUart;

  deltaPosError_t scalesDeltaPos[SCALES_COUNT];
  deltaPosError_t scalesSyncDeltaPos[SCALES_COUNT];
  deltaPosError_t scalesSyncDeltaPosSteps[SCALES_COUNT];
  deltaPosError_t scalesSpeed[SCALES_COUNT];

  deltaPosError_t servoSpeed;

  deltaPosError_t indexDeltaPos;

} rampsHandler_t;

extern modbusHandler_t RampsModbusData;

void RampsStart(rampsHandler_t *rampsData);

void SynchroRefreshTimerIsr(rampsHandler_t *data);

_Noreturn void updateSpeedTask(void *argument);

_Noreturn void userLedTask(__attribute__((unused)) void *argument);
#endif