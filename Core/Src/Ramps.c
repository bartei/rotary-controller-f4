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

bool servoEnabled = false;
uint16_t servoCycles = 0;
uint16_t servoCyclesCounter = 0;

typedef struct {
  int32_t syncPosition;
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
//  rampsData->shared.servo.maxValue = 360;
//  rampsData->shared.servo.minValue = 0;
  rampsData->shared.servo.allowedError = 0;

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
  configureOutputPin(SPARE_1_GPIO_PORT, SPARE_1_PIN);
  configureOutputPin(SPARE_2_GPIO_PORT, SPARE_2_PIN);
  configureOutputPin(SPARE_3_GPIO_PORT, SPARE_3_PIN);

  // Configure synchro variables
  rampsData->synchroRefreshFrequency = 100000000.0f / ((float)rampsData->synchroRefreshTimer->Init.Prescaler + 1) / (float)(rampsData->synchroRefreshTimer->Init.Period + 1);

  // Configure tasks
  osThreadNew(userLedTask, rampsData, &ledTaskAttributes);
  osThreadNew(updateSpeedTask, rampsData, &speedTaskAttributes);

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
deltaPositionAndError(int32_t currentValue, int32_t ratioNum, int32_t ratioDen, deltaPosError_t * data) {
  int32_t startValue = (int16_t )(currentValue - data->oldPosition) * ratioNum + data->error;
  data->oldPosition = currentValue;
  data->scaledDelta = (int32_t)(startValue / ratioDen);
  data->error = (int32_t)(startValue % ratioDen);
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
  float interval = (float)shared->executionInterval / 100000000.0f;

  for (int i = 0; i < SCALES_COUNT; i++) {
    deltaPositionAndError(__HAL_TIM_GET_COUNTER(data->shared.scales[i].timerHandle), shared->scales[i].ratioNum, shared->scales[i].ratioDen, &data->scalesDeltaPos[i]);
    shared->scales[i].position += data->scalesDeltaPos[i].scaledDelta;

    // calculate delta for sync ratio configured for the current scale
    deltaPositionAndError(shared->scales[i].position, shared->scales[i].syncRatioNum, shared->scales[i].syncRatioDen, &data->scalesSyncDeltaPos[i]);
    scalesPrivate[i].syncPosition += data->scalesSyncDeltaPos[i].scaledDelta;
    // calculate delta steps for the servo sync position
    deltaPositionAndError(scalesPrivate[i].syncPosition, shared->servo.ratioNum, shared->servo.ratioDen * 1000, &data->scalesSyncDeltaPosSteps[i]);

    // Update the desired steps and clear the accumulator for this scale if enabled
    if (shared->scales[i].syncMotion) {
      shared->servo.desiredSteps += data->scalesSyncDeltaPosSteps[i].scaledDelta;
    }

    if (shared->scales[i].mode == spindle) {
      if (shared->scales[i].position >= (shared->scales[i].ratioNum * 1000)) {
        shared->scales[i].position -= (shared->scales[i].ratioNum * 1000);
        data->scalesSyncDeltaPos[i].oldPosition -= (shared->scales[i].ratioNum * 1000);
        data->scalesSpeed[i].oldPosition -= (shared->scales[i].ratioNum * 1000);
      }
      if (shared->scales[i].position < 0) {
        shared->scales[i].position += (shared->scales[i].ratioNum * 1000);
        data->scalesSyncDeltaPos[i].oldPosition += (shared->scales[i].ratioNum * 1000);
        data->scalesSpeed[i].oldPosition += (shared->scales[i].ratioNum * 1000);
      }
    }

    // Update fastData current position
    shared->fastData.scaleCurrent[i] = shared->scales[i].position;
  }

  // Normalize absolute offset
  shared->servo.absoluteOffset = fmodf(shared->servo.absoluteOffset, (float)shared->servo.ratioDen);
  if (shared->servo.absoluteOffset < 0) shared->servo.absoluteOffset += (float)shared->servo.ratioDen;

  // Indexing position calculation
  shared->servo.indexOffset = (float)shared->servo.ratioDen / (float)shared->index.divisions * (float)shared->index.index;

  shared->servo.desiredPosition = fmodf(shared->servo.indexOffset + shared->servo.absoluteOffset, (float)shared->servo.ratioDen);
  if (shared->servo.desiredPosition < 0) shared->servo.desiredPosition += (float)shared->servo.ratioDen;

  while (shared->servo.currentPosition >= (float)shared->servo.ratioDen) {
    data->indexDeltaPos.oldPosition -= shared->servo.ratioDen * 10000;
    shared->servo.currentPosition -= (float)shared->servo.ratioDen;
    shared->servo.desiredPosition -= (float)shared->servo.ratioDen;
  }
  while (shared->servo.currentPosition < 0) {
    data->indexDeltaPos.oldPosition += shared->servo.ratioDen * 10000;
    shared->servo.currentPosition += (float)shared->servo.ratioDen;
    shared->servo.desiredPosition += (float)shared->servo.ratioDen;
  }

  int direction = 0;
  float signedDistance = (shared->servo.desiredPosition - shared->servo.currentPosition);
  float distanceToGo = fabsf(signedDistance);

  if (signedDistance > 0) {
    direction = +1;
  }
  if (signedDistance < 0) {
    direction = -1;
  }

  if (distanceToGo > ((float)shared->servo.ratioDen/2)) {
    distanceToGo = (float)shared->servo.ratioDen - distanceToGo;
    direction = -direction;
  }

  float space = (shared->servo.currentSpeed * shared->servo.currentSpeed) / (2 * shared->servo.acceleration);

  if (servoEnabled) {
    if (direction != 0) {
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
    }

    shared->servo.currentPosition += shared->servo.currentSpeed * interval * (float)direction;
  }

  // Target reached when we are within the allowed error
  if (fabsf(shared->servo.currentPosition - shared->servo.desiredPosition) <= shared->servo.allowedError) {
    shared->servo.currentPosition = shared->servo.desiredPosition;
    shared->servo.currentSpeed = 0;
  }

  deltaPositionAndError((int32_t)(shared->servo.currentPosition*10000.0f), shared->servo.ratioNum, shared->servo.ratioDen * 10000, &data->indexDeltaPos);
  shared->servo.desiredSteps += data->indexDeltaPos.scaledDelta;

  // Update fast access variables for display refresh
  shared->fastData.cycles = shared->executionCycles;
  shared->fastData.servoCurrent = (float)shared->servo.currentSteps / ((float)shared->servo.ratioNum / (float)shared->servo.ratioDen);
  shared->fastData.servoDesired = (float)shared->servo.desiredSteps / ((float)shared->servo.ratioNum / (float)shared->servo.ratioDen);

  if (shared->servo.currentSteps >= shared->servo.ratioNum) {
    shared->servo.currentSteps -= shared->servo.ratioNum;
    shared->servo.desiredSteps -= shared->servo.ratioNum;
  }

  if (shared->servo.currentSteps < 0) {
    shared->servo.currentSteps += shared->servo.ratioNum;
    shared->servo.desiredSteps += shared->servo.ratioNum;
  }

  if (servoCyclesCounter > 0) {
    servoCyclesCounter--;
  }
  if (servoCyclesCounter < servoCycles/2) {
    HAL_GPIO_WritePin(STEP_GPIO_PORT, STEP_PIN, GPIO_PIN_RESET);
  }

  if (servoEnabled && servoCyclesCounter == 0) {
    // generate pulses to reach desired position with the motor
    if (shared->servo.desiredSteps > shared->servo.currentSteps) {
      HAL_GPIO_WritePin(DIR_GPIO_PORT, DIR_PIN, GPIO_PIN_SET);
      HAL_GPIO_WritePin(STEP_GPIO_PORT, STEP_PIN, GPIO_PIN_SET);
      servoCyclesCounter = servoCycles;
      shared->servo.currentSteps++;
    }
    else if (shared->servo.desiredSteps < shared->servo.currentSteps) {
      HAL_GPIO_WritePin(DIR_GPIO_PORT, DIR_PIN, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(STEP_GPIO_PORT, STEP_PIN, GPIO_PIN_SET);
      servoCyclesCounter = servoCycles;
      shared->servo.currentSteps--;
    }
  }

  shared->executionCycles = DWT->CYCCNT - start;
  HAL_GPIO_WritePin(SPARE_2_GPIO_PORT, SPARE_1_PIN, GPIO_PIN_RESET);
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
      osDelay(25);
      HAL_GPIO_WritePin(USR_LED_GPIO_Port, USR_LED_Pin, GPIO_PIN_SET);
    }
  }

}

const int32_t updateSpeedTaskTicks = 50;
_Noreturn void updateSpeedTask(void *argument) {
  rampsHandler_t * rampsData = (rampsHandler_t *)argument;
  int32_t oldPosition = 0;
  uint32_t deltaPosition;

  for(;;)
  {
    // Update allowed error
    rampsData->shared.servo.allowedError = (
      (float)rampsData->shared.servo.ratioDen /
      (float)rampsData->shared.servo.ratioNum
    ) / 2.0f;

    // Update the current speed
    osDelay(updateSpeedTaskTicks);

    deltaPositionAndError((int32_t)(rampsData->shared.fastData.servoCurrent * 1000) , updateSpeedTaskTicks, HAL_GetTickFreq(), &rampsData->servoSpeed);

    rampsData->shared.fastData.servoSpeed = (float)(rampsData->servoSpeed.scaledDelta) / 1000.0f;
    rampsData->shared.servo.estimatedSpeed = rampsData->shared.fastData.servoSpeed;

    // If maximum speed has been changed, update the motor timer accordingly
    if (rampsData->shadowServoRatioNum != rampsData->shared.servo.ratioNum || rampsData->shadowServoRatioDen != rampsData->shared.servo.ratioDen) {
      rampsData->shadowServoRatioNum = rampsData->shared.servo.ratioNum;
      rampsData->shadowServoRatioDen = rampsData->shared.servo.ratioDen;
      float motor_max_freq = (rampsData->shared.servo.maxSpeed *  1.1f * (float)rampsData->shared.servo.ratioNum / (float)rampsData->shared.servo.ratioDen);
      float newPeriod = floorf(rampsData->synchroRefreshFrequency/motor_max_freq);
      if (newPeriod > (float)UINT16_MAX) {
        newPeriod = 65535;
      }
      if (newPeriod < 2) {
        newPeriod = 2;
      }
      servoCycles = (uint16_t)newPeriod;
    }

    bool anySyncMotionEnabled = false;
    for (int i = 0; i < SCALES_COUNT; i++) {
      // Handle Enable signal for motion control
      anySyncMotionEnabled = anySyncMotionEnabled || rampsData->shared.scales[i].syncMotion;

      // Update scale/spindle speed value
      deltaPositionAndError(rampsData->shared.scales[i].position, updateSpeedTaskTicks, HAL_GetTickFreq(), &rampsData->scalesSpeed[i]);
      rampsData->shared.scales[i].speed = rampsData->scalesSpeed[i].scaledDelta;
      rampsData->shared.fastData.scaleSpeed[i] = rampsData->scalesSpeed[i].scaledDelta;
    }

    if (rampsData->shared.servo.desiredPosition != rampsData->shared.servo.currentPosition || anySyncMotionEnabled) {
      if (!servoEnabled) {
        HAL_GPIO_WritePin(ENA_GPIO_PORT, ENA_PIN, GPIO_PIN_RESET);
        osDelay(ENA_DELAY_MS);
        servoEnabled = true;
      }
    } else {
        HAL_GPIO_WritePin(ENA_GPIO_PORT, ENA_PIN, GPIO_PIN_SET);
        servoEnabled = false;
    }

  }
}
