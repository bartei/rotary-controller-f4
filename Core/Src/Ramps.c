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
  for (int i = 0; i < SERVO_COUNT; i++) {
    rampsData->servo[i].maxSpeed = 720;
    rampsData->servo[i].acceleration = 120;

    configureOutputPin(rampsData->servo[i].dirPort, rampsData->servo[i].dirPin);
    configureOutputPin(rampsData->servo[i].stepPort, rampsData->servo[i].stepPin);
  }

  for (int i = 0; i < SCALES_COUNT; i++) {
    rampsData->scales[i].syncRatioNum = 1;
    rampsData->scales[i].syncRatioDen = 100;
  }

  // Configure Pins
  configureOutputPin(rampsData->enaPort, rampsData->enaPin);

  // Configure tasks
  osThreadNew(userLedTask, rampsData, &ledTaskAttributes);
  osThreadNew(updateSpeedTask, rampsData, &speedTaskAttributes);
  osThreadNew(servoEnableTask, rampsData, &servoEnableTaskAttributes);

  // Initialize and start encoder timer, reset the sync flags
  for (int j = 0; j < SCALES_COUNT; ++j) {
    initScaleTimer(rampsData->scales[j].timerHandle);
    HAL_TIM_Encoder_Start(rampsData->scales[j].timerHandle, TIM_CHANNEL_ALL);
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
  RampsModbusData.u16regs = (uint16_t *) (rampsData);
  RampsModbusData.u16regsize = sizeof(&rampsData) / sizeof(uint16_t);
  RampsModbusData.xTypeHW = USART_HW;
  ModbusInit(&RampsModbusData);
  ModbusStart(&RampsModbusData);

  // Start synchro interrupt
  HAL_TIM_Base_Start_IT(rampsData->synchroRefreshTimer);
}

static inline void deltaPositionAndError(
        int32_t currentValue,
        int32_t ratioNum,
        int32_t ratioDen,
        deltaPosError_t *data) {
  int32_t startValue = (int16_t) (currentValue - data->oldPosition) * ratioNum + data->error;
  data->oldPosition = currentValue;
  data->scaledDelta = (int32_t) (startValue / ratioDen);
  data->error = (int32_t) (startValue % ratioDen);
}

static inline void updateIndexingPosition(rampsHandler_t *data) {
  float interval = (float) data->executionInterval / 100000000.0f;

  for (int i = 0; i < SERVO_COUNT; i++) {
    float stopDistance = (data->servo[i].currentSpeed * data->servo[i].currentSpeed / data->servo[i].acceleration) / 2;

    // Accelerate Pos
    if (data->servo[i].stepsToGo > 0) {
      if ((float)data->servo[i].stepsToGo > stopDistance && data->servo[i].currentSpeed < data->servo[i].maxSpeed) {
        data->servo[i].currentSpeed += data->servo[i].acceleration * interval;
        // max speed
        if (data->servo[i].currentSpeed > data->servo[i].maxSpeed) {
          data->servo[i].currentSpeed = data->servo[i].maxSpeed;
        }
      }

      // Decelerate Pos
      if ((float)data->servo[i].stepsToGo < stopDistance) {
        data->servo[i].currentSpeed -= data->servo[i].acceleration * interval;
        if (data->servo[i].currentSpeed < 0) {
          data->servo[i].currentSpeed = 0;
        }
      }
    }

    if (data->servo[i].stepsToGo < 0) {
      // Accelerate Neg
      if (-(float)data->servo[i].stepsToGo > stopDistance && -data->servo[i].currentSpeed < data->servo[i].maxSpeed) {
        data->servo[i].currentSpeed -= data->servo[i].acceleration * interval;
        if (-data->servo[i].currentSpeed > data->servo[i].maxSpeed) {
          data->servo[i].currentSpeed = -data->servo[i].maxSpeed;
        }
      }

      // Decelerate Neg
      if (-(float)data->servo[i].stepsToGo < stopDistance) {
        data->servo[i].currentSpeed += data->servo[i].acceleration * interval;
        if (data->servo[i].currentSpeed > 0) {
          data->servo[i].currentSpeed = 0;
        }
      }
    }

    if (data->servo[i].stepsToGo == 0) {
      // Security measure, end of travel
      data->servo[i].currentSpeed = 0;
    } else {
      int32_t positionIncrement = ((int32_t)((float)data->servo[i].currentSpeed * (float)data->executionInterval + (float)data->rampsDeltaPos.error) / 100000000);
      data->rampsDeltaPos.error = ((int32_t)((float)data->servo[i].currentSpeed * (float)data->executionInterval + (float)data->rampsDeltaPos.error) % 100000000);
      data->servo[i].desiredSteps += positionIncrement;
      data->servo[i].stepsToGo -= positionIncrement;
      data->fastData.stepsToGo = data->servo[i].stepsToGo;
    }

  }

}

static inline void updateJogPosition(rampsHandler_t *data) {
  float interval = (float) data->executionInterval / 100000000.0f;

  for (int i = 0; i < SERVO_COUNT; i++) {
    // Start Pos
    if (data->servo[i].jogSpeed > 0) {
      if (data->servo[i].currentSpeed < data->servo[i].jogSpeed) {
        data->servo[i].currentSpeed += data->servo[i].acceleration * interval;
        // max speed
        if (data->servo[i].currentSpeed > data->servo[i].jogSpeed) {
          data->servo[i].currentSpeed = data->servo[i].jogSpeed;
        }
      }
    }

    // Start Neg
    if (data->servo[i].jogSpeed < 0) {
      if (data->servo[i].currentSpeed > data->servo[i].jogSpeed) {
        data->servo[i].currentSpeed -= data->servo[i].acceleration * interval;
        if (data->servo[i].currentSpeed < data->servo[i].jogSpeed) {
          data->servo[i].currentSpeed = data->servo[i].jogSpeed;
        }
      }
    }

    // Stop Pos/Neg
    if (data->servo[i].currentSpeed > 0) {
      if (data->servo[i].currentSpeed > data->servo[i].jogSpeed) {
        data->servo[i].currentSpeed -= data->servo[i].acceleration * interval;
        if (data->servo[i].currentSpeed < 0) data->servo[i].currentSpeed = 0;
      }
    }

    if (data->servo[i].currentSpeed < 0) {
      if (data->servo[i].currentSpeed < data->servo[i].jogSpeed) {
        data->servo[i].currentSpeed += data->servo[i].acceleration * interval;
        if (data->servo[i].currentSpeed > 0) data->servo[i].currentSpeed = 0;
      }
    }

    int32_t positionIncrement = ((int32_t)((float)data->servo[i].currentSpeed * (float)data->executionInterval + (float)data->rampsDeltaPos.error) / 100000000);
    data->rampsDeltaPos.error = ((int32_t)((float)data->servo[i].currentSpeed * (float)data->executionInterval + (float)data->rampsDeltaPos.error) % 100000000);
    data->servo[i].desiredSteps += positionIncrement;
  }

}

void SynchroRefreshTimerIsr(rampsHandler_t *data) {
  uint32_t start = DWT->CYCCNT;
  data->executionIntervalPrevious = data->executionIntervalCurrent;
  data->executionIntervalCurrent = DWT->CYCCNT;
  data->executionInterval = data->executionIntervalCurrent - data->executionIntervalPrevious;
  data->fastData.executionInterval = data->executionInterval;

  // Reset the step pin as soon as possible
  for (int i = 0; i < SERVO_COUNT; i++) {
    data->servo[i].stepPort->BSRR = (uint32_t)data->servo[i].stepPin << 16;
  }

  for (int i = 0; i < SCALES_COUNT; i++) {
    data->scalesDeltaPos[i].oldPosition = data->scalesDeltaPos[i].position;
    data->scalesDeltaPos[i].position = __HAL_TIM_GET_COUNTER(data->scales[i].timerHandle);
    data->scalesDeltaPos[i].delta = (int16_t) (data->scalesDeltaPos[i].position - data->scalesDeltaPos[i].oldPosition);
    data->scales[i].position += data->scalesDeltaPos[i].delta;
    data->fastData.scaleCurrent[i] = data->scales[i].position;

// TODO: This has to go into the servo sync function not here and the parameters are now different
//    // calculate delta for sync ratio configured for the current scale
//    deltaPositionAndError(
//      data->scales[i].position,
//      data->scales[i].syncRatioNum,
//      data->scales[i].syncRatioDen,
//      &data->scalesSyncDeltaPos[i]
//    );
//
//    // request motion only if sync is enabled
//    if (data->scales[i].syncEnable != 0) {
//      data->servo.desiredSteps += data->scalesSyncDeltaPos[i].scaledDelta;
//    }

    // Update fastData current position
  }

  if (data->fastData.servoMode == 1) updateIndexingPosition(data);
  if (data->fastData.servoMode == 2) updateJogPosition(data);

  if (data->fastData.servoMode != 0 && servoCyclesCounter == 0) {
    for (int i = 0; i < SERVO_COUNT; i++) {
      int32_t change = (int32_t)(data->servo[i].desiredSteps) - (int32_t)data->servo[i].currentSteps;
      if (change > 0) {
        data->servo[i].currentDirection = 1;
        data->servo[i].dirPort->BSRR = data->servo[i].dirPin;
      }
      if (change < 0) {
        data->servo[i].currentDirection = -1;
        data->servo[i].dirPort->BSRR = data->servo[i].dirPin << 16;
      }
      if (data->servo[i].currentDirection == data->servo[i].previousDirection && change != 0) {
        data->servo[i].stepPort->BSRR = data->servo[i].stepPin << 16;
        data->servo[i].currentSteps += data->servo[i].currentDirection;
        data->servo[i].previousDirection = data->servo[i].currentDirection;
      }

    }

  }

  servoCyclesCounter = (servoCyclesCounter + 1) % servoCycles;
  data->executionCycles = DWT->CYCCNT - start;
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
    rampsData->fastData.cycles = rampsData->executionCycles;
//    rampsData->fastData.servoCurrent = rampsData->servo.currentSteps;
//    rampsData->fastData.servoDesired = rampsData->servo.desiredSteps;

    // If maximum speed has been changed, update the motor timer accordingly
//    float clock_freq = 100000000.0f / ((float) rampsData->synchroRefreshTimer->Init.Prescaler + 1) /
//                       (float) (rampsData->synchroRefreshTimer->Init.Period + 1);

    // Clamping value for max speed to the maximum allowed by the current timer refresh rate from the sync routine
    for (int i = 0; i < SERVO_COUNT; i++) {
      if (rampsData->servo[i].maxSpeed > 100000) {
        rampsData->servo[i].maxSpeed = 100000;
      }
    }

//    float newPeriod = floorf(clock_freq / rampsData->servo.maxSpeed);
//    if (newPeriod > (float) UINT16_MAX) {
//      newPeriod = 65535;
//    }
//    if (newPeriod < 1) {
//      newPeriod = 0;
//    }
//    servoCycles = (uint16_t) newPeriod;
    // TODO: Perhaps consider using the max value from all the speeds from all the servos?
    // TODO: Perhaps move this variable into the rampsdata structure so it can be monitored from ui
    servoCycles = 0;

    for (int i = 0; i < SCALES_COUNT; i++) {
      // Update scale/spindle speed value
      deltaPositionAndError(
        rampsData->scales[i].position,
        updateSpeedTaskTicks,
        HAL_GetTickFreq(),
        &rampsData->scalesSpeed[i]
      );
      rampsData->scales[i].speed = rampsData->scalesSpeed[i].scaledDelta;
      rampsData->fastData.scaleSpeed[i] = rampsData->scalesSpeed[i].scaledDelta;
    }
  }
}

_Noreturn void servoEnableTask(void *argument) {
  rampsHandler_t *rampsData = (rampsHandler_t *) argument;
  uint32_t previousPosition = 0;

  for (;;) {
    osDelay(100);

    bool anySyncMotionEnabled = false;
    for (int i = 0; i < SCALES_COUNT; i++) {
      anySyncMotionEnabled = anySyncMotionEnabled || (rampsData->scales[i].syncEnable != 0);
    }

    if (anySyncMotionEnabled && rampsData->fastData.servoMode != 2)
      rampsData->fastData.servoMode = 1;

//    rampsData->shared.fastData.servoSpeed = (float)(int32_t)(rampsData->shared.servo.currentSteps - previousPosition) * 10;
//    previousPosition = rampsData->shared.servo.currentSteps;

    if (rampsData->fastData.servoMode != 0)
      rampsData->enaPort->BSRR = rampsData->enaPin << 16;
//      HAL_GPIO_WritePin(shared->enaPort, shared->enaPin, GPIO_PIN_RESET);

    if (rampsData->fastData.servoMode == 0)
      rampsData->enaPort->BSRR = rampsData->enaPin;
//      HAL_GPIO_WritePin(ENA_GPIO_PORT, ENA_PIN, GPIO_PIN_SET);
  }
}