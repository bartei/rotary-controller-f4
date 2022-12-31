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
#include <stdlib.h>
#include "Ramps.h"
#include "Scales.h"


const osThreadAttr_t taskRampsAttributes = {
        .name = "taskRamps",
        .stack_size = 128 * 4,
        .priority = (osPriority_t) osPriorityNormal,
};

// This variable is the handler for the modbus communication
modbusHandler_t RampsModbusData;

void RampsStart(rampsHandler_t * rampsData) {
  rampsData->shared.acceleration = 10;
  rampsData->shared.maxSpeed = 10000;
  rampsData->shared.minSpeed = 100;

  // Initialize and start encoder timer
  startScalesTimers(rampsData->scales);

  // Start synchro interrupt
  HAL_TIM_Base_Start_IT(rampsData->synTimer);

  RampsModbusData.uModbusType = MB_SLAVE;
  RampsModbusData.port = rampsData->modbusUart;
  RampsModbusData.u8id = MODBUS_ADDRESS;
  RampsModbusData.u16timeOut = 1000;
  RampsModbusData.EN_Port = NULL;
  RampsModbusData.u16regs = (uint16_t *)(&rampsData->shared);
  RampsModbusData.u16regsize = sizeof(rampsData->shared) / sizeof(uint16_t);
  RampsModbusData.xTypeHW = USART_HW;
  ModbusInit(&RampsModbusData);
  ModbusStart(&RampsModbusData);

  startRampsTask(rampsData);
}

/**
 * This method implements the logic to generate a single pulse when the controlled axis is
 * configured in synchro mode.
 * When operating in this mode the timer is stopped immediately after reaching the
 * requested number of steps
 * @param data Reference to the ramps handler data structure
 */
void motorSynchroModeIsr(rampsHandler_t * data) {
  data->shared.currentPosition += data->syncData.direction;

  // When the final position is reached we stop the timer
  if (data->shared.currentPosition == data->shared.finalPosition) {
    HAL_TIM_PWM_Stop_IT(data->motorTimer, TIM_CHANNEL_1);
  }
}

/**
 * This function has to be called from the ISR associated with the timer used to generate
 * the pwm signal for the shared step pulse train.
 * This method is used to generate the acc/dec ramps when running in index mode
 * @param data Reference to the ramps handler data structure
 */
void motorIndexModeIsr(rampsHandler_t * data) {
  // Stop the timer and exit if mode is 0
   rampsSharedData_t * shared = &data->shared;
   rampsIndexData_t * indexData = &data->indexData;

  if (shared->mode == MODE_HALT) {
    HAL_TIM_PWM_Stop_IT(data->motorTimer, TIM_CHANNEL_1);
    return;
  }

  // Handle the initialization of the motion when step is 0
  if (indexData->currentStep == 0) {
    indexData->floatAccelInterval = shared->acceleration;
    shared->currentSpeed = shared->minSpeed;
    indexData->stepRatio =
            (float )shared->stepRatioNum /
            (float )shared->stepRatioDen;
    indexData->decelSteps = 0;
  }

  // Handle acceleration phase
  if (shared->currentSpeed < shared->maxSpeed && (indexData->currentStep < indexData->totalSteps / 2) ) {
    shared->currentSpeed = shared->currentSpeed + shared->acceleration;
    indexData->floatAccelInterval = (float)RAMPS_CLOCK_FREQUENCY * indexData->stepRatio / shared->currentSpeed;

    if (shared->currentSpeed > shared->maxSpeed) {
      shared->currentSpeed = shared->maxSpeed;
    }
  } else if (indexData->decelSteps == 0) {
    // Store the count of steps it took to accelerate, so it can be used to define when to start
    // decelerating without doing further calculations.
    indexData->decelSteps = indexData->currentStep;
  }

  // Handle deceleration phase
  if (
          shared->currentSpeed > shared->minSpeed &&
          (indexData->currentStep > indexData->totalSteps / 2) &&
          (indexData->currentStep > (indexData->totalSteps - indexData->decelSteps))
          ) {
    shared->currentSpeed = shared->currentSpeed - (float) shared->acceleration;
    indexData->floatAccelInterval = (float)RAMPS_CLOCK_FREQUENCY * indexData->stepRatio / shared->currentSpeed;
  }

  // Configure the timer preload and the pwm duty cycle to 50%
  if (indexData->floatAccelInterval > 65535) {
    __HAL_TIM_SET_AUTORELOAD(data->motorTimer, 65535);
    __HAL_TIM_SET_COMPARE(data->motorTimer, TIM_CHANNEL_1, 10);
  } else {
    __HAL_TIM_SET_AUTORELOAD(data->motorTimer, (uint16_t)indexData->floatAccelInterval);
    __HAL_TIM_SET_COMPARE(data->motorTimer, TIM_CHANNEL_1, 10);
  }

  // Increment the current step
  indexData->currentStep++;

  if (shared->currentPosition < shared->finalPosition) {
    shared->currentPosition = shared->currentPosition + 1;
  }
  if (shared->currentPosition > shared->finalPosition) {
    shared->currentPosition = shared->currentPosition - 1;
  }

  // Motion path finished, go back to halt mode
  if (indexData->currentStep == indexData->totalSteps) {
    shared->mode = MODE_HALT;
    HAL_TIM_PWM_Stop_IT(data->motorTimer, TIM_CHANNEL_1);
  }
}

/**
 * Call this method from the interrupt service routine associated with the pwm generation timer
 * used to control the stepper shared steps generation
 * @param rampsTimer handle reference to the ramps generation time, the same as the calling isr
 * @param data the data structure holding all the rotary controller data
 */
void RampsMotionIsr(rampsHandler_t * data) {
  // Controller is in index mode
  if (data->shared.mode == MODE_INDEX) {
    motorIndexModeIsr(data);
  } else if (data->shared.mode == MODE_SYNCHRO) {
    motorSynchroModeIsr(data);
  }
}

/**
 * This function initializes the data and resources for the operation of the controlled axis so that it moves
 * in sync with an encoder reference.
 * @param delta_x used to set the ratio of the motion, same as the numerator in the ratio
 * @param delta_y used to set the ratio of the motion, same as the denominator in the ratio
 * @param ramps_htim reference to the timer used to generate the stepper shared steps
 * @param data reference to the data structure holding the controlled axis data
 */
void SyncMotionInit(rampsHandler_t * data) {
  rampsSharedData_t * shared = &data->shared;

  // Verify the ratio to be acceptable, return and set error otherwise
  if (shared->synRatioNum == 0 ||
      shared->synRatioDen == 0 ||
      shared->synRatioDen > shared->synRatioNum)
  {
    shared->mode = MODE_SYNCHRO_BAD_RATIO;
    return;
  }

  // Configure the values for the bresenham interpolation
  if (shared->synRatioDen < 0) { data->syncData.yi = -1; }
  else { data->syncData.yi = 1; }

  data->syncData.D = 2 * (shared->synRatioDen - shared->synRatioNum);

  // Configure the timer settings for the pwm generation, will be used as one pulse
  __HAL_TIM_SET_AUTORELOAD(data->motorTimer, 100);
  __HAL_TIM_SET_COMPARE(data->motorTimer, TIM_CHANNEL_1, 10);

  // Ensure the current and final positions are equal
  shared->currentPosition = shared->finalPosition;
  shared->mode = MODE_SYNCHRO;
}

/**
 * This function has to be called from a simple timer interrupt routine
 * happening at regular intervals which shall match the maximum frequency
 * supported by either the controller speed or the stepper shared controller
 * Starting timer interval frequency set to 50Khz
 * @param data Reference to the ramps handler data structure
 */
void SyncMotionIsr(rampsHandler_t * data) {
  rampsSharedData_t * shared = &(data->shared);
  rampsSyncData_t * sync_data = &(data->syncData);

  updateScales(data->scales);
  shared->encoderPosition = data->scales.scalePosition[shared->synScaleIndex].positionCurrent;

  // Skip Conditions
  if (shared->mode != MODE_SYNCHRO || shared->finalPosition != shared->currentPosition) {
    return;
  }

  if (sync_data->positionPrevious < sync_data->positionCurrent)
  {
    HAL_GPIO_WritePin(DIR_GPIO_PORT, DIR_PIN, GPIO_PIN_SET);
    sync_data->direction = +1;
    for (int32_t x = sync_data->positionPrevious; x < sync_data->positionCurrent; ++x) {
      if (sync_data->D > 0) {
        // Error greater than 0, step forward the controlled axis
        shared->finalPosition += sync_data->yi;
        sync_data->D = sync_data->D + (2 * (shared->synRatioDen - shared->synRatioNum));
      } else {
        sync_data->D = sync_data->D + 2 * shared->synRatioDen;
      }
    }
  }

  else if (sync_data->positionPrevious > sync_data->positionCurrent)
  {
    HAL_GPIO_WritePin(DIR_GPIO_PORT, DIR_PIN, GPIO_PIN_RESET);
    sync_data->direction = -1;
    for (int32_t x = sync_data->positionPrevious; x > sync_data->positionCurrent; --x) {
      if (sync_data->D < 0) {
        // Error greater than 0, step forward the controlled axis
        shared->finalPosition -= sync_data->yi;
        sync_data->D = sync_data->D - (2 * (shared->synRatioDen - shared->synRatioNum));
      } else {
        sync_data->D = sync_data->D - 2 * shared->synRatioDen;
      }
    }
  }
  else if (shared->finalPosition != shared->currentPosition) {
    HAL_TIM_PWM_Start_IT(data->motorTimer, TIM_CHANNEL_1);
  }
}

/**
 * This method is used to initialize the RTOS task responsible for controlling the ramps
 * ramp generator.
 */
void startRampsTask(rampsHandler_t * rampsData)
{
  rampsData->TaskRampsHandle = osThreadNew(RampsTask, rampsData, &taskRampsAttributes);
}

/**
 * This is the FreeRTOS task invoked to handle the general low priority task responsible
 * for the management of all the ramps system operation.
 * @param argument Reference to the ramps handler data structure
 */
void RampsTask(void *argument)
{
  rampsHandler_t * data = (rampsHandler_t *)argument;
  rampsSharedData_t * shared = &data->shared;
  rampsIndexData_t * indexData = &data->indexData;
  rampsSyncData_t * syncData = &data->syncData;

  uint16_t ledTicks = 0;
  for(;;)
  {
    osDelay(50);

    // Refresh scales position reporting in the modbus shared data
    for (int i = 0; i < SCALES_COUNT; ++i) {
      data->shared.scalesPosition[i] = data->scales.scalePosition[i].positionCurrent;
    }

    // Handle Status Led Heartbeat
    ledTicks = (ledTicks + 1) % 10;
    if (ledTicks == 0) {
      HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    }

    // Handle sync mode request
    if (shared->mode == MODE_SYNCHRO_INIT) {
      SyncMotionInit(data);
      shared->mode = MODE_SYNCHRO; // Set proper mode
    }

    // Handle request to set encoder count value
    if (shared->mode == MODE_SET_ENCODER) {
      // Reset everything and configure the provided value
      int scaleIndex = data->shared.encoderPresetIndex;
      // Counter reset
      __HAL_TIM_SET_COUNTER(data->scales.scaleTimer[scaleIndex], 0);
      syncData->encoderPrevious = 0;

      // Sync data struct reset
      data->scales.scalePosition[scaleIndex].positionCurrent = shared->encoderPresetValue;

      // Shared data struct reset
      shared->scalesPosition[scaleIndex] = shared->encoderPresetValue;
      shared->mode = MODE_HALT; // Set proper mode
    }

    // Handle index mode request
    if (shared->mode == MODE_INDEX_INIT) {
      // Provided final destination is the same as current, go back to halt
      if (shared->finalPosition == shared->currentPosition) {
        shared->mode = MODE_HALT;
      }

      // Set Direction based on destination
      if (shared->finalPosition > shared->currentPosition) {
        HAL_GPIO_WritePin(data->directionPinPort, data->directionPin, 1);
      } else {
        HAL_GPIO_WritePin(data->directionPinPort, data->directionPin, 0);
      }

      // Set mode
      // Reset counters used for ramps generation, will have to improve this
      shared->mode = MODE_INDEX;
      indexData->currentStep = 0;
      indexData->totalSteps = abs(shared->finalPosition - shared->currentPosition);

      // Start the timer responsible for the ramps generation
      HAL_TIM_PWM_Start_IT(data->motorTimer, TIM_CHANNEL_1);
    }

    if (shared->mode == MODE_INDEX && shared->finalPosition == shared->currentPosition) {
      shared->mode = MODE_HALT;
      HAL_TIM_PWM_Stop_IT(data->motorTimer, TIM_CHANNEL_1);
    }
  }
}