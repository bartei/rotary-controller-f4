/**
 * Copyright © 2024 <Stefano Bertelli>
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
#include <stdlib.h>
#include "Ramps.h"
#include "Scales.h"


// This variable is the handler for the modbus communication
modbusHandler_t RampsModbusData;

bool servoEnabled = false;
uint16_t servoCycles = 0;
uint16_t servoCyclesCounter = 0;

typedef struct {
  bool syncMotionEnabled;
  int32_t fixDecimalsMultiplier;
  int32_t position;
  int32_t error;
  int32_t oldPosition;
  float syncPosition;
} scalesPrivate_t;

const int32_t fixDecimalsMultiplier = 1000;
scalesPrivate_t scalesPrivate[SCALES_COUNT];


//osThreadId_t userLedTaskHandler;
const osThreadAttr_t ledTaskAttributes = {
  .name = "UpdateLedTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,

};

//osThreadId_t updateSpeedTaskHandler;
const osThreadAttr_t speedTaskAttributes = {
  .name = "updateSpeedTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

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
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(Port, &GPIO_InitStruct);
}


void RampsStart(rampsHandler_t *rampsData) {
  rampsData->shared.index.divisions = 24;
  rampsData->shared.index.index = 0;

  rampsData->shared.servo.ratioDen = 360;
  rampsData->shared.servo.ratioNum = 400;
  rampsData->shared.servo.minSpeed = 0;
  rampsData->shared.servo.maxSpeed = 720;
  rampsData->shared.servo.acceleration = 120;
  rampsData->shared.servo.maxValue = 360;
  rampsData->shared.servo.minValue = 0;
  rampsData->shared.servo.allowedError = 0.01f;

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
  configureOutputPin(STEP_GPIO_PORT, STEP_PIN);

  // Configure tasks
  osThreadNew(userLedTask, rampsData, &ledTaskAttributes);
  osThreadNew(updateSpeedTask, rampsData, &speedTaskAttributes);

  // Initialize and start encoder timer, reset the sync flags
  for (int j = 0; j < SCALES_COUNT; ++j) {
    initScaleTimer(rampsData->shared.scales[j].timerHandle);
    HAL_TIM_Encoder_Start(rampsData->shared.scales[j].timerHandle, TIM_CHANNEL_ALL);

    scalesPrivate[j].syncMotionEnabled = false;
  }

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

void SynchroRefreshTimerIsr(rampsHandler_t *data) {
  rampsSharedData_t *shared = &(data->shared);
  uint32_t start = DWT->CYCCNT;
  shared->execution_interval_previous = shared->execution_interval_current;
  shared->execution_interval_current = DWT->CYCCNT;
  shared->execution_interval = shared->execution_interval_current - shared->execution_interval_previous;
  float interval = (float)shared->execution_interval / 100000000.0f;

  float syncPositionAccumulator = 0;

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

    // Syncro motion scaling
      int16_t syncDistValue = (shared->scales[i].position - scalesPrivate[i].oldPosition) *
                              shared->scales[i].syncRatioNum /
                              shared->scales[i].syncRatioDen;
      int16_t syncDistError = (shared->scales[i].position - scalesPrivate[i].oldPosition) *
                              shared->scales[i].syncRatioNum %
                              shared->scales[i].syncRatioDen;
      scalesPrivate[i].oldPosition = shared->scales[i].position;

      scalesPrivate[i].position += syncDistValue;
      scalesPrivate[i].error += syncDistError;
      if (scalesPrivate[i].error >= shared->scales[i].syncRatioDen) {
        scalesPrivate[i].position++;
        scalesPrivate[i].error -= shared->scales[i].syncRatioDen;
      }
      if (-scalesPrivate[i].error >= shared->scales[i].syncRatioDen) {
        scalesPrivate[i].position--;
        scalesPrivate[i].error += shared->scales[i].syncRatioDen;
      }

      scalesPrivate[i].syncPosition = (float)scalesPrivate[i].position / 1000.0f;

    // Trigger for auto offset update when enabling sync mode on an axis
    if (!scalesPrivate[i].syncMotionEnabled && shared->scales[i].syncMotion) {
      scalesPrivate[i].position = 0;
      scalesPrivate[i].syncPosition = 0;
      scalesPrivate[i].syncMotionEnabled = true;
      shared->servo.absoluteOffset -= scalesPrivate[i].syncPosition;
      shared->servo.currentPosition -= scalesPrivate[i].syncPosition;
    }

    // Remove auto offset update when disabling sync mode on an axis
    if (scalesPrivate[i].syncMotionEnabled && !shared->scales[i].syncMotion) {
      scalesPrivate[i].syncMotionEnabled = false;
      shared->servo.absoluteOffset += scalesPrivate[i].syncPosition;
      shared->servo.currentPosition += scalesPrivate[i].syncPosition;
    }

    if (shared->scales[i].syncMotion) {
    syncPositionAccumulator += scalesPrivate[i].syncPosition;
    }
    shared->fastData.scaleCurrent[i] = shared->scales[i].position;
  }

  shared->servo.syncOffset = syncPositionAccumulator;

  // Normalize absolute offset
  shared->servo.absoluteOffset = fmodf(shared->servo.absoluteOffset, (float)shared->servo.ratioDen);
  if (shared->servo.absoluteOffset < 0) shared->servo.absoluteOffset += (float)shared->servo.ratioDen;

  // Indexing position calculation
  shared->servo.indexOffset = (float)shared->servo.maxValue / (float)shared->index.divisions *  (float)shared->index.index;

  shared->servo.desiredPosition = fmodf(shared->servo.indexOffset + shared->servo.absoluteOffset, (float)shared->servo.ratioDen);
  if (shared->servo.desiredPosition < 0) shared->servo.desiredPosition += (float)shared->servo.ratioDen;


  while (shared->servo.currentPosition >= (float)shared->servo.ratioDen) {
    shared->servo.currentPosition = shared->servo.currentPosition - (float)shared->servo.ratioDen;
    shared->servo.desiredPosition = shared->servo.desiredPosition - (float)shared->servo.ratioDen;
    shared->servo.currentSteps = shared->servo.currentSteps - shared->servo.ratioNum;
  }
  while (shared->servo.currentPosition < 0) {
    shared->servo.currentPosition = shared->servo.currentPosition + (float)shared->servo.ratioDen;
    shared->servo.desiredPosition = shared->servo.desiredPosition + (float)shared->servo.ratioDen;
    shared->servo.currentSteps = shared->servo.currentSteps + shared->servo.ratioNum;
  }

  while (shared->servo.currentPosition >= (float)shared->servo.ratioDen) {
    shared->servo.currentPosition = shared->servo.currentPosition - (float)shared->servo.ratioDen;
    shared->servo.desiredPosition = shared->servo.desiredPosition - (float)shared->servo.ratioDen;
    shared->servo.currentSteps = shared->servo.currentSteps - shared->servo.ratioNum;
  }


  float distanceToGo = fabsf(shared->servo.currentPosition - shared->servo.desiredPosition);
  bool invert = false;
  if (distanceToGo > ((float)shared->servo.ratioDen/2.0f)) {
    distanceToGo = (float)shared->servo.ratioDen - distanceToGo;
    invert = true;
  }

  bool goForward = (shared->servo.desiredPosition - shared->servo.currentPosition) > 0;
  bool goReverse = (shared->servo.desiredPosition - shared->servo.currentPosition) < 0;

  if (invert) {
    goForward = !goForward;
    goReverse = !goReverse;
  }

  shared->servo.allowedError = ((float)shared->servo.ratioDen / (float)shared->servo.ratioNum);

  float time = (shared->servo.currentSpeed) / shared->servo.acceleration;
  float space = (shared->servo.acceleration *  time * time) / 2;

  if (servoEnabled) {
    if (goForward) {
      // Start moving if we need to
      if (shared->servo.currentSpeed == 0) {
        shared->servo.currentSpeed += shared->servo.acceleration * interval;
      } else if (shared->servo.currentSpeed < shared->servo.maxSpeed && distanceToGo > space) {
        shared->servo.currentSpeed += shared->servo.acceleration * interval;
        if (shared->servo.currentSpeed > shared->servo.maxSpeed) {
          shared->servo.currentSpeed = shared->servo.maxSpeed;
        }
      } else if (distanceToGo <= space && shared->servo.currentSpeed > shared->servo.minSpeed) {
        shared->servo.currentSpeed -= shared->servo.acceleration * interval;
        if (shared->servo.currentSpeed < shared->servo.minSpeed) {
          shared->servo.currentSpeed = shared->servo.minSpeed;
        }
      } else if (shared->servo.currentSpeed > shared->servo.maxSpeed) {
        shared->servo.currentSpeed -= shared->servo.acceleration * interval;
      }
    } else if (goReverse) {
      if (shared->servo.currentSpeed == 0) {
        shared->servo.currentSpeed -= shared->servo.acceleration * interval;
      } else if (-shared->servo.currentSpeed < shared->servo.maxSpeed && distanceToGo > space) {
        shared->servo.currentSpeed -= shared->servo.acceleration * interval;
        if (-shared->servo.currentSpeed > shared->servo.maxSpeed) {
          shared->servo.currentSpeed = -shared->servo.maxSpeed;
        }
      } else if (distanceToGo <= space && -shared->servo.currentSpeed > shared->servo.minSpeed) {
        shared->servo.currentSpeed += shared->servo.acceleration * interval;
        if (-shared->servo.currentSpeed < shared->servo.minSpeed) {
          shared->servo.currentSpeed = -shared->servo.minSpeed;
        }
      } else if (-shared->servo.currentSpeed > shared->servo.maxSpeed) {
        shared->servo.currentSpeed += shared->servo.acceleration * interval;
      }
    }

    shared->servo.currentPosition += shared->servo.currentSpeed * interval;
  }

  // Target reached when we are within the allowed error
  if (fabsf(shared->servo.currentPosition - shared->servo.desiredPosition) <= shared->servo.allowedError) {
    shared->servo.currentPosition = shared->servo.desiredPosition;
    shared->servo.currentSpeed = 0;
  }

  // Update fast access variables for display refresh
  shared->fastData.cycles = shared->execution_cycles;
  shared->fastData.servoCurrent = shared->servo.currentPosition + shared->servo.syncOffset;
  shared->fastData.servoDesired = shared->servo.desiredPosition + shared->servo.syncOffset;

  shared->servo.desiredSteps = (int32_t) ((shared->servo.currentPosition + shared->servo.syncOffset)
                                          * (float) shared->servo.ratioNum
                                          / (float) shared->servo.ratioDen);

  if (servoCyclesCounter > 0) {
    servoCyclesCounter--;
  }

  if (servoCyclesCounter < servoCycles/2) {
    HAL_GPIO_WritePin(STEP_GPIO_PORT, STEP_PIN, GPIO_PIN_RESET);
  }

  // generate pulses to reach desired position with the motor
  if (servoEnabled && servoCyclesCounter == 0) {
    if (shared->servo.desiredSteps > shared->servo.currentSteps) {
      HAL_GPIO_WritePin(DIR_GPIO_PORT, DIR_PIN, GPIO_PIN_SET);
      HAL_GPIO_WritePin(STEP_GPIO_PORT, STEP_PIN, GPIO_PIN_SET);
      servoCyclesCounter = servoCycles;
      shared->servo.currentSteps++;
    }
    if (shared->servo.desiredSteps < shared->servo.currentSteps) {
      HAL_GPIO_WritePin(DIR_GPIO_PORT, DIR_PIN, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(STEP_GPIO_PORT, STEP_PIN, GPIO_PIN_SET);
      servoCyclesCounter = servoCycles;
      shared->servo.currentSteps--;
    }
  }

  shared->execution_cycles = DWT->CYCCNT - start;
}

_Noreturn void userLedTask(__attribute__((unused)) void *argument) {
  uint16_t oldInCnt = 0;

  for(;;)
  {
    osDelay(50);
    if (oldInCnt != RampsModbusData.u16InCnt) {
      oldInCnt = RampsModbusData.u16InCnt;
      HAL_GPIO_TogglePin(USR_LED_GPIO_Port, USR_LED_Pin);
      HAL_GPIO_WritePin(USR_LED_GPIO_Port, USR_LED_Pin, GPIO_PIN_RESET);
      osDelay(50);
      HAL_GPIO_WritePin(USR_LED_GPIO_Port, USR_LED_Pin, GPIO_PIN_SET);
    }
  }

}

_Noreturn void updateSpeedTask(void *argument) {
  rampsHandler_t * rampsData = (rampsHandler_t *)argument;
  int32_t oldPosition = 0;
  uint32_t deltaPosition;

  for(;;)
  {
    // Update the current speed
    osDelay(50);
    deltaPosition = abs(oldPosition - (int32_t)rampsData->shared.servo.currentSteps);
    oldPosition = rampsData->shared.servo.currentSteps;
    rampsData->shared.fastData.servoSpeed = (float)deltaPosition / 0.05f / (float)rampsData->shared.servo.ratioNum * (float)rampsData->shared.servo.ratioDen;
    rampsData->shared.servo.estimatedSpeed = rampsData->shared.fastData.servoSpeed;

    // If maximum speed has been changed, update the motor timer accordingly
    float clock_freq = 100000000.0f / ((float)rampsData->synchroRefreshTimer->Init.Prescaler + 1) / (float)(rampsData->synchroRefreshTimer->Init.Period + 1);
    float motor_max_freq = (rampsData->shared.servo.maxSpeed *  1.1f * (float)rampsData->shared.servo.ratioNum / (float)rampsData->shared.servo.ratioDen);
    float newPeriod = floorf(clock_freq/motor_max_freq);
    if (newPeriod > (float)UINT16_MAX) {
      newPeriod = 65535;
    }
    if (newPeriod < 2) {
      newPeriod = 2;
    }
    servoCycles = (uint16_t)newPeriod;

    // Handle Enable signal for motion control
    bool anySyncMotionEnabled = false;
    for (int i = 0; i < SCALES_COUNT; i++) {
      anySyncMotionEnabled = anySyncMotionEnabled || scalesPrivate[i].syncMotionEnabled;
    }
    if (rampsData->shared.servo.desiredPosition != rampsData->shared.servo.currentPosition || anySyncMotionEnabled) {
      if (!servoEnabled) {
        HAL_GPIO_WritePin(ENA_GPIO_PORT, ENA_PIN, GPIO_PIN_SET);
        osDelay(100);
        servoEnabled = true;
      }
    } else {
        HAL_GPIO_WritePin(ENA_GPIO_PORT, ENA_PIN, GPIO_PIN_RESET);
        servoEnabled = false;
    }

  }
}
