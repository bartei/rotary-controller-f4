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
#include "Ramps.h"
#include "Scales.h"


// This variable is the handler for the modbus communication
modbusHandler_t RampsModbusData;

uint16_t servoCycles = 0;
uint16_t servoCyclesCounter = 0;


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

//osThreadId_t updateServoEnableTaskHandler;
const osThreadAttr_t servoEnableTaskAttributes = {
.name = "servoEnableTask",
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
  rampsData->shared.servo.maxSpeed = 720;
  rampsData->shared.servo.acceleration = 120;

  for (int i = 0; i < SCALES_COUNT; i++) {
    rampsData->shared.scales[i].syncRatioNum = 1;
    rampsData->shared.scales[i].syncRatioDen = 100;
  }

  // Configure Pins
  configureOutputPin(DIR_GPIO_PORT, DIR_PIN);
  configureOutputPin(ENA_GPIO_PORT, ENA_PIN);
  configureOutputPin(STEP_GPIO_PORT, STEP_PIN);
  configureOutputPin(SPARE_1_GPIO_PORT, SPARE_1_PIN);
  configureOutputPin(SPARE_2_GPIO_PORT, SPARE_2_PIN);
  configureOutputPin(SPARE_3_GPIO_PORT, SPARE_3_PIN);

  // Configure tasks
  osThreadNew(userLedTask, rampsData, &ledTaskAttributes);
  osThreadNew(updateSpeedTask, rampsData, &speedTaskAttributes);
  osThreadNew(servoEnableTask, rampsData, &servoEnableTaskAttributes);


  // Initialize and start encoder timer, reset the sync flags
  for (int j = 0; j < SCALES_COUNT; ++j) {
    initScaleTimer(rampsData->shared.scales[j].timerHandle);
    HAL_TIM_Encoder_Start(rampsData->shared.scales[j].timerHandle, TIM_CHANNEL_ALL);
  }

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

  // Start synchro interrupt
  HAL_TIM_Base_Start_IT(rampsData->synchroRefreshTimer);
}

static inline void
deltaPositionAndError(int32_t currentValue, int32_t ratioNum, int32_t ratioDen, deltaPosError_t *data) {
  int32_t startValue = (int16_t) (currentValue - data->oldPosition) * ratioNum + data->error;
  data->oldPosition = currentValue;
  data->scaledDelta = (int32_t) (startValue / ratioDen);
  data->error = (int32_t) (startValue % ratioDen);
}


static inline void updateIndexingPosition(rampsHandler_t *data) {
  rampsSharedData_t *shared = &(data->shared);
  float interval = (float) shared->executionInterval / 100000000.0f;
  float stopDistance = (shared->servo.currentSpeed * shared->servo.currentSpeed / shared->servo.acceleration) / 2;

  // Accelerate Pos
  if (shared->servo.direction > 0) {
    if ((float)shared->servo.direction > stopDistance && shared->servo.currentSpeed < shared->servo.maxSpeed) {
      shared->servo.currentSpeed += shared->servo.acceleration * interval;
      // max speed
      if (shared->servo.currentSpeed > shared->servo.maxSpeed) {
        shared->servo.currentSpeed = shared->servo.maxSpeed;
      }
    }

    // Decelerate Pos
    if ((float)shared->servo.direction < stopDistance) {
      shared->servo.currentSpeed -= shared->servo.acceleration * interval;
      if (shared->servo.currentSpeed < 0) {
        shared->servo.currentSpeed = 0;
      }
    }
  }

  if (shared->servo.direction < 0) {
    // Accelerate Neg
    if (-(float)shared->servo.direction > stopDistance && -shared->servo.currentSpeed < shared->servo.maxSpeed) {
      shared->servo.currentSpeed -= shared->servo.acceleration * interval;
      if (-shared->servo.currentSpeed > shared->servo.maxSpeed) {
        shared->servo.currentSpeed = -shared->servo.maxSpeed;
      }
    }

    // Decelerate Neg
    if (-(float)shared->servo.direction < stopDistance) {
      shared->servo.currentSpeed += shared->servo.acceleration * interval;
      if (shared->servo.currentSpeed > 0) {
        shared->servo.currentSpeed = 0;
      }
    }
  }

  if (shared->servo.direction == 0) {
    // Security measure, end of travel
    shared->servo.currentSpeed = 0;
  } else {
    int32_t positionIncrement = ((int32_t)((float)shared->servo.currentSpeed * (float)shared->executionInterval + (float)data->rampsDeltaPos.error) / 100000000);
    data->rampsDeltaPos.error = ((int32_t)((float)shared->servo.currentSpeed * (float)shared->executionInterval + (float)data->rampsDeltaPos.error) % 100000000);
    shared->servo.desiredSteps += positionIncrement;
    shared->servo.direction -= positionIncrement;
  }

}


void SynchroRefreshTimerIsr(rampsHandler_t *data) {
  HAL_GPIO_TogglePin(SPARE_1_GPIO_PORT, SPARE_1_PIN);
  HAL_GPIO_WritePin(SPARE_2_GPIO_PORT, SPARE_1_PIN, GPIO_PIN_SET);
  rampsSharedData_t *shared = &(data->shared);
  uint32_t start = DWT->CYCCNT;
  shared->executionIntervalPrevious = shared->executionIntervalCurrent;
  shared->executionIntervalCurrent = DWT->CYCCNT;
  shared->executionInterval = shared->executionIntervalCurrent - shared->executionIntervalPrevious;
  shared->fastData.executionInterval = shared->executionInterval;

  HAL_GPIO_WritePin(STEP_GPIO_PORT, STEP_PIN, GPIO_PIN_RESET);

  for (int i = 0; i < SCALES_COUNT; i++) {
    data->scalesDeltaPos[i].oldPosition = data->scalesDeltaPos[i].position;
    data->scalesDeltaPos[i].position = __HAL_TIM_GET_COUNTER(data->shared.scales[i].timerHandle);
    data->scalesDeltaPos[i].delta = (int16_t) (data->scalesDeltaPos[i].position - data->scalesDeltaPos[i].oldPosition);
    shared->scales[i].position += data->scalesDeltaPos[i].delta;

    // calculate delta for sync ratio configured for the current scale
    if (shared->scales[i].syncEnable != 0) {
      deltaPositionAndError(
        shared->scales[i].position,
        shared->scales[i].syncRatioNum,
        shared->scales[i].syncRatioDen,
        &data->scalesSyncDeltaPos[i]
      );
      shared->servo.desiredSteps += data->scalesSyncDeltaPos[i].scaledDelta;
    }

    // Update fastData current position
    shared->fastData.scaleCurrent[i] = shared->scales[i].position;
  }

  updateIndexingPosition(data);

  if (shared->fastData.servoEnable != 0 && servoCyclesCounter == 0) {
    int32_t change = shared->servo.desiredSteps - shared->servo.currentSteps;
    // generate pulses to reach desired position with the motor
    if (change > 0) {
      HAL_GPIO_WritePin(DIR_GPIO_PORT, DIR_PIN, GPIO_PIN_SET);
      HAL_GPIO_WritePin(STEP_GPIO_PORT, STEP_PIN, GPIO_PIN_SET);
      shared->servo.currentSteps++;
    } else if (change < 0) {
      HAL_GPIO_WritePin(DIR_GPIO_PORT, DIR_PIN, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(STEP_GPIO_PORT, STEP_PIN, GPIO_PIN_SET);
      shared->servo.currentSteps--;
    }
  }

  servoCyclesCounter = (servoCyclesCounter + 1) % servoCycles;

  shared->executionCycles = DWT->CYCCNT - start;
  HAL_GPIO_WritePin(SPARE_2_GPIO_PORT, SPARE_1_PIN, GPIO_PIN_RESET);
}

_Noreturn void userLedTask(__attribute__((unused)) void *argument) {
  uint16_t oldInCnt = 0;

  for (;;) {
    osDelay(50);
    if (oldInCnt != RampsModbusData.u16InCnt) {
      oldInCnt = RampsModbusData.u16InCnt;
      HAL_GPIO_TogglePin(USR_LED_GPIO_Port, USR_LED_Pin);
      HAL_GPIO_WritePin(USR_LED_GPIO_Port, USR_LED_Pin, GPIO_PIN_RESET);
      osDelay(25);
      HAL_GPIO_WritePin(USR_LED_GPIO_Port, USR_LED_Pin, GPIO_PIN_SET);
    }
  }

}

const int32_t updateSpeedTaskTicks = 50;

_Noreturn void updateSpeedTask(void *argument) {
  rampsHandler_t *rampsData = (rampsHandler_t *) argument;

  for (;;) {

    // Update the current speed
    osDelay(updateSpeedTaskTicks);

    // Update fast access variables
    rampsData->shared.fastData.cycles = rampsData->shared.executionCycles;
    rampsData->shared.fastData.servoCurrent = rampsData->shared.servo.currentSteps;
    rampsData->shared.fastData.servoDesired = rampsData->shared.servo.desiredSteps;
    rampsData->shared.fastData.servoSpeed = rampsData->shared.servo.currentSpeed;

    // If maximum speed has been changed, update the motor timer accordingly
    float clock_freq = 100000000.0f / ((float) rampsData->synchroRefreshTimer->Init.Prescaler + 1) /
                       (float) (rampsData->synchroRefreshTimer->Init.Period + 1);

    // Clamping value for max speed to the maximum allowed by the current timer refresh rate from the sync routine
    if (rampsData->shared.servo.maxSpeed > 50000) {
      rampsData->shared.servo.maxSpeed = 50000;
    }

    float newPeriod = floorf(clock_freq / rampsData->shared.servo.maxSpeed);
    if (newPeriod > (float) UINT16_MAX) {
      newPeriod = 65535;
    }
    if (newPeriod < 1) {
      newPeriod = 1;
    }
    servoCycles = (uint16_t) newPeriod;

    for (int i = 0; i < SCALES_COUNT; i++) {
      // Update scale/spindle speed value
      deltaPositionAndError(
      rampsData->shared.scales[i].position,
      updateSpeedTaskTicks,
      HAL_GetTickFreq(),
      &rampsData->scalesSpeed[i]
      );
      rampsData->shared.scales[i].speed = rampsData->scalesSpeed[i].scaledDelta;
      rampsData->shared.fastData.scaleSpeed[i] = rampsData->scalesSpeed[i].scaledDelta;
    }
  }
}

_Noreturn void servoEnableTask(void *argument) {
  rampsHandler_t *rampsData = (rampsHandler_t *) argument;
  rampsSharedData_t *shared = (rampsSharedData_t *) &rampsData->shared;

  for (;;) {
    osDelay(100);

    bool anySyncMotionEnabled = false;
    for (int i = 0; i < SCALES_COUNT; i++) {
      anySyncMotionEnabled = anySyncMotionEnabled || (shared->scales[i].syncEnable != 0);
    }

    if (anySyncMotionEnabled) rampsData->shared.fastData.servoEnable = 1;

    if (shared->fastData.servoEnable != 0) HAL_GPIO_WritePin(ENA_GPIO_PORT, ENA_PIN, GPIO_PIN_RESET);
    if (shared->fastData.servoEnable == 0) HAL_GPIO_WritePin(ENA_GPIO_PORT, ENA_PIN, GPIO_PIN_SET);
  }
}