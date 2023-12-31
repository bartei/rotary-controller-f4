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
 * LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */
#include <math.h>
#include "Ramps.h"
#include "Scales.h"


// This variable is the handler for the modbus communication
modbusHandler_t RampsModbusData;


void configureOutputPin(GPIO_TypeDef *Port, uint16_t Pin) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(Port, &GPIO_InitStruct);
}


void RampsStart(rampsHandler_t *rampsData) {
  rampsData->shared.index.divisions = 24;
  rampsData->shared.index.index = 0;

  rampsData->shared.servo.ratioDen = 360;
  rampsData->shared.servo.ratioNum = 400;
  rampsData->shared.servo.minSpeed = 1;
  rampsData->shared.servo.maxSpeed = 720;
  rampsData->shared.servo.acceleration = 120;
  rampsData->shared.servo.maxValue = 360;
  rampsData->shared.servo.minValue = 0;
  rampsData->shared.servo.allowedError = 0.01;

  rampsData->shared.scales[0].ratioNum = 1000;
  rampsData->shared.scales[0].ratioDen = 200;
  rampsData->shared.scales[1].ratioNum = 1000;
  rampsData->shared.scales[1].ratioDen = 200;
  rampsData->shared.scales[2].ratioNum = 1000;
  rampsData->shared.scales[2].ratioDen = 200;
  rampsData->shared.scales[3].ratioNum = 1000;
  rampsData->shared.scales[3].ratioDen = 200;

  // Configure Pins
  configureOutputPin(DIR_GPIO_PORT, DIR_PIN);
  configureOutputPin(ENA_GPIO_PORT, ENA_PIN);

  // Initialize and start encoder timer
  for (int j = 0; j < SCALES_COUNT; ++j) {
    initScaleTimer(rampsData->shared.scales[j].timerHandle);
    HAL_TIM_Encoder_Start(rampsData->shared.scales[j].timerHandle, TIM_CHANNEL_ALL);
  }

  // Configure the timer settings for the pwm generation, will be used as one pulse
  __HAL_TIM_SET_AUTORELOAD(rampsData->motorPwmTimer, 100);
  __HAL_TIM_SET_COMPARE(rampsData->motorPwmTimer, TIM_CHANNEL_1, 50);

  // Start synchro interrupt
  HAL_TIM_Base_Start_IT(rampsData->synchroRefreshTimer);

  // Enable debug cycle counter
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  // Start Modbus
  RampsModbusData.uModbusType = MB_SLAVE;
  RampsModbusData.port = rampsData->modbusUart;
  RampsModbusData.u8id = MODBUS_ADDRESS;
  RampsModbusData.u16timeOut = 1000;
  RampsModbusData.EN_Port = NULL;
  RampsModbusData.u16regs = (uint16_t *) (&rampsData->shared);
  RampsModbusData.u16regsize = sizeof(rampsData->shared) / sizeof(uint16_t);
  RampsModbusData.xTypeHW = USART_HW;
  ModbusInit(&RampsModbusData);
  ModbusStart(&RampsModbusData);
}


/**
 * Call this method from the interrupt service routine associated with the pwm generation timer
 * used to control the stepper shared steps generation
 * @param rampsTimer handle reference to the ramps generation time, the same as the calling isr
 * @param data the data structure holding all the rotary controller data
 */
void MotorPwmTimerISR(rampsHandler_t *data) {
  // Controller is in index mode
  if (HAL_GPIO_ReadPin(DIR_GPIO_PORT, DIR_PIN) == GPIO_PIN_SET) {
    data->shared.servo.currentSteps++;
  } else {
    data->shared.servo.currentSteps--;
  }
  HAL_TIM_PWM_Stop_IT(data->motorPwmTimer, TIM_CHANNEL_1);
}


void SynchroRefreshTimerIsr(rampsHandler_t *data) {
  rampsSharedData_t *shared = &(data->shared);
  uint32_t start = DWT->CYCCNT;
  shared->execution_interval_previous = shared->execution_interval_current;
  shared->execution_interval_current = DWT->CYCCNT;
  shared->execution_interval = shared->execution_interval_current - shared->execution_interval_previous;
  float interval = 1.0f / (float)shared->execution_interval;

  float syncPositionAccumulator = 0;

  // Update the scales
  for (int i = 0; i < SCALES_COUNT; i++) {
    shared->scales[i].encoderPrevious = shared->scales[i].encoderCurrent;
    shared->scales[i].encoderCurrent = __HAL_TIM_GET_COUNTER(data->shared.scales[i].timerHandle);
    int16_t distValue, distError;
    distValue = (int16_t) (shared->scales[i].encoderCurrent - shared->scales[i].encoderPrevious) *
                shared->scales[i].ratioNum /
                shared->scales[i].ratioDen;
    distError = (int16_t) (shared->scales[i].encoderCurrent - shared->scales[i].encoderPrevious) *
                shared->scales[i].ratioNum %
                shared->scales[i].ratioDen;

    shared->scales[i].position += distValue;
    shared->scales[i].error += distError;
    if (shared->scales[i].error >= shared->scales[i].ratioDen) {
      shared->scales[i].position++;
      shared->scales[i].error -= shared->scales[i].ratioDen;
    }
    if (-shared->scales[i].error >= shared->scales[i].ratioDen) {
      shared->scales[i].position--;
      shared->scales[i].error += shared->scales[i].ratioDen;
    }

    if (shared->scales[i].syncMotion) {
      syncPositionAccumulator += (float)shared->scales[i].position
        * (float)shared->scales[i].syncRatioNum
        / (float)shared->scales[i].syncRatioDen;
    }
  }

  shared->servo.syncOffset = syncPositionAccumulator;

  // Indexing position calculation
  shared->servo.indexOffset = (float)shared->servo.maxValue / (float)shared->index.divisions *  (float) (shared->index.index);
  shared->servo.desiredPosition = shared->servo.indexOffset + shared->servo.absoluteOffset + shared->servo.syncOffset;
  shared->servo.allowedError = (float)shared->servo.ratioNum/(float)shared->servo.ratioDen/2;

  float distanceToGo = fabsf(shared->servo.desiredPosition - shared->servo.currentPosition);
  float time = (shared->servo.currentSpeed - shared->servo.minSpeed) / shared->servo.acceleration;
  float space = (shared->servo.acceleration * time * time) / 2;

  if (shared->servo.desiredPosition > shared->servo.currentPosition) {
    // Start moving if we need to
    if (shared->servo.currentSpeed == 0) {
      shared->servo.currentSpeed = shared->servo.minSpeed;
    } else if (shared->servo.currentSpeed < shared->servo.maxSpeed && distanceToGo > space) {
      shared->servo.currentSpeed += shared->servo.acceleration * interval;
      if (shared->servo.currentSpeed > shared->servo.maxSpeed) {
        shared->servo.currentSpeed = shared->servo.maxSpeed;
      }
    }
    else if (distanceToGo <= space && shared->servo.currentSpeed > shared->servo.minSpeed) {
      shared->servo.currentSpeed -= shared->servo.acceleration * interval;
      if (shared->servo.currentSpeed < shared->servo.minSpeed) {
        shared->servo.currentSpeed = shared->servo.minSpeed;
      }
    }
    else if (shared->servo.currentSpeed > shared->servo.maxSpeed) {
      shared->servo.currentSpeed -= shared->servo.acceleration * interval;
    }
  }

  if (shared->servo.desiredPosition < shared->servo.currentPosition) {
    if (shared->servo.currentSpeed == 0) {
      shared->servo.currentSpeed = -shared->servo.minSpeed;
    } else if (-shared->servo.currentSpeed < shared->servo.maxSpeed && distanceToGo > space) {
      shared->servo.currentSpeed -= shared->servo.acceleration * interval;
      if (-shared->servo.currentSpeed > shared->servo.maxSpeed) {
        shared->servo.currentSpeed = -shared->servo.maxSpeed;
      }
    }
    else if (distanceToGo <= space && -shared->servo.currentSpeed > shared->servo.minSpeed) {
      shared->servo.currentSpeed += shared->servo.acceleration * interval;
      if (-shared->servo.currentSpeed < shared->servo.minSpeed) {
        shared->servo.currentSpeed = -shared->servo.minSpeed;
      }
    }
    else if (-shared->servo.currentSpeed > shared->servo.maxSpeed) {
      shared->servo.currentSpeed += shared->servo.acceleration * interval;
    }
  }

  // Update current position - This has the side effect of going super fast mode with sync motion, might be desired
  // TODO: Needs more investigation
  shared->servo.currentPosition += shared->servo.currentSpeed * interval;
  if (fabsf(shared->servo.currentPosition - shared->servo.desiredPosition) < shared->servo.allowedError) {
    shared->servo.currentPosition = shared->servo.desiredPosition;
  }

  // Update fast access variables for display refresh
  shared->fastData.cycles = shared->execution_cycles;
  shared->fastData.servoCurrent = shared->servo.currentPosition;
  shared->fastData.servoDesired = shared->servo.desiredPosition;
  for (char i = 0; i < SCALES_COUNT; i++) {
    shared->fastData.scaleCurrent[i] = shared->scales[i].position;
  }

  shared->servo.desiredSteps = (int32_t) (shared->servo.currentPosition
                                          * (float) shared->servo.ratioNum
                                          / (float) shared->servo.ratioDen);

  if (shared->servo.desiredSteps > shared->servo.currentSteps) {
    HAL_GPIO_WritePin(DIR_GPIO_PORT, DIR_PIN, GPIO_PIN_SET);
    HAL_TIM_PWM_Start_IT(data->motorPwmTimer, TIM_CHANNEL_1);
  }
  if (shared->servo.desiredSteps < shared->servo.currentSteps) {
    HAL_GPIO_WritePin(DIR_GPIO_PORT, DIR_PIN, GPIO_PIN_RESET);
    HAL_TIM_PWM_Start_IT(data->motorPwmTimer, TIM_CHANNEL_1);
  }

  shared->execution_cycles = DWT->CYCCNT - start;
}
