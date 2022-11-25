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
#include "ramps.h"

const osThreadAttr_t taskRampsAttributes = {
        .name = "taskLed",
        .stack_size = 128 * 4,
        .priority = (osPriority_t) osPriorityNormal,
};

// This variable is the handler for the modbus communication
modbusHandler_t RampsModbusData;

void EncoderTimerInit(TIM_HandleTypeDef * encoder_htim)
{
  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

//  encoder_htim.Instance = encoder_timer;
  encoder_htim->Init.Prescaler = 0;
  encoder_htim->Init.CounterMode = TIM_COUNTERMODE_UP;
  encoder_htim->Init.Period = 65535;
  encoder_htim->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  encoder_htim->Init.RepetitionCounter = 0;
  encoder_htim->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_ENABLE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
}

void RampsStart(rampsHandler_t * rampsData) {
  rampsData->sharedData.acceleration = 10;
  rampsData->sharedData.maxSpeed = 10000;
  rampsData->sharedData.minSpeed = 100;

  // Initialize and start encoder timer
  HAL_TIM_Encoder_Start(rampsData->encoderTimer, TIM_CHANNEL_ALL);
  HAL_TIM_Base_Start_IT(rampsData->synTimer);

  RampsModbusData.uModbusType = MB_SLAVE;
  RampsModbusData.port = rampsData->modbusUart;
  RampsModbusData.u8id = MODBUS_ADDRESS;
  RampsModbusData.u16timeOut = 1000;
  RampsModbusData.EN_Port = NULL;
  RampsModbusData.u16regs = (uint16_t *)(&rampsData->sharedData);
  RampsModbusData.u16regsize = sizeof(rampsData->sharedData) / sizeof(uint16_t);
  RampsModbusData.xTypeHW = USART_HW;
  ModbusInit(&RampsModbusData);
  ModbusStart(&RampsModbusData);

  RampsTaskInit(rampsData);
}

/**
 * This method implements the logic to generate a single pulse when the controlled axis is
 * configured in synchro mode.
 * When operating in this mode the timer is stopped immediately after reaching the
 * requested number of steps
 * @param rampsData Reference to the ramps handler data structure
 */
void RampsSynchroModeIsr(rampsHandler_t * rampsData) {
  rampsData->sharedData.currentPosition += rampsData->syncData.direction;

  // When the final position is reached we stop the timer
  if (rampsData->sharedData.currentPosition == rampsData->sharedData.finalPosition) {
    HAL_TIM_PWM_Stop_IT(rampsData->motorTimer, TIM_CHANNEL_1);
  }
}

/**
 * This function has to be called from the ISR associated with the timer used to generate
 * the pwm signal for the motor step pulse train.
 * This method is used to generate the acc/dec ramps when running in index mode
 * @param rampsData Reference to the ramps handler data structure
 */
void RampsIndexModeIsr(rampsHandler_t * rampsData) {
  // Stop the timer and exit if mode is 0
   rampsSharedData_t * shared = &rampsData->sharedData;
   rampsIndexData_t * data = &rampsData->indexData;

  if (shared->mode == MODE_HALT) {
    HAL_TIM_PWM_Stop_IT(rampsData->motorTimer, TIM_CHANNEL_1);
    return;
  }

  // Handle the initialization of the motion when step is 0
  if (data->currentStep == 0) {
    data->floatAccelInterval = shared->acceleration;
    shared->currentSpeed = shared->minSpeed;
    data->stepRatio =
            (float )shared->stepRatioNum /
            (float )shared->stepRatioDen;
    data->decelSteps = 0;
  }

  // Handle acceleration phase
  if (shared->currentSpeed < shared->maxSpeed && (data->currentStep < data->totalSteps / 2) ) {
    shared->currentSpeed = shared->currentSpeed + shared->acceleration;
    data->floatAccelInterval = (float)RAMPS_CLOCK_FREQUENCY * data->stepRatio / shared->currentSpeed;

    if (shared->currentSpeed > shared->maxSpeed) {
      shared->currentSpeed = shared->maxSpeed;
    }
  } else if (data->decelSteps == 0) {
    // Store the count of steps it took to accelerate, so it can be used to define when to start
    // decelerating without doing further calculations.
    data->decelSteps = data->currentStep;
  }

  // Handle deceleration phase
  if (
          shared->currentSpeed > shared->minSpeed &&
          (data->currentStep > data->totalSteps / 2) &&
          (data->currentStep > (data->totalSteps - data->decelSteps))
          ) {
    shared->currentSpeed = shared->currentSpeed - (float) shared->acceleration;
    data->floatAccelInterval = (float)RAMPS_CLOCK_FREQUENCY * data->stepRatio / shared->currentSpeed;
  }

  // Configure the timer preload and the pwm duty cycle to 50%
  if (data->floatAccelInterval > 65535) {
    __HAL_TIM_SET_AUTORELOAD(rampsData->motorTimer, 65535);
    __HAL_TIM_SET_COMPARE(rampsData->motorTimer, TIM_CHANNEL_1, 10);
  } else {
    __HAL_TIM_SET_AUTORELOAD(rampsData->motorTimer, (uint16_t)data->floatAccelInterval);
    __HAL_TIM_SET_COMPARE(rampsData->motorTimer, TIM_CHANNEL_1, 10);
  }

  // Increment the current step
  data->currentStep++;

  if (shared->currentPosition < shared->finalPosition) {
    shared->currentPosition = shared->currentPosition + 1;
  }
  if (shared->currentPosition > shared->finalPosition) {
    shared->currentPosition = shared->currentPosition - 1;
  }

  // Motion path finished, go back to halt mode
  if (data->currentStep == data->totalSteps) {
    shared->mode = MODE_HALT;
    HAL_TIM_PWM_Stop_IT(rampsData->motorTimer, TIM_CHANNEL_1);
  }
}

/**
 * Call this method from the interrupt service routine associated with the pwm generation timer
 * used to control the stepper motor steps generation
 * @param rampsTimer handle reference to the ramps generation time, the same as the calling isr
 * @param data the data structure holding all the rotary controller data
 */
void RampsMotionIsr(rampsHandler_t * rampsData) {
  // Controller is in index mode
  if (rampsData->sharedData.mode == MODE_INDEX) {
    RampsIndexModeIsr(rampsData);
  } else if (rampsData->sharedData.mode == MODE_SYNCHRO) {
    RampsSynchroModeIsr(rampsData);
  }
}

/**
 * This function initializes the data and resources for the operation of the controlled axis so that it moves
 * in sync with an encoder reference.
 * @param delta_x used to set the ratio of the motion, same as the numerator in the ratio
 * @param delta_y used to set the ratio of the motion, same as the denominator in the ratio
 * @param ramps_htim reference to the timer used to generate the stepper motor steps
 * @param data reference to the data structure holding the controlled axis data
 */
void SyncMotionInit(rampsHandler_t * rampsData) {
  rampsSharedData_t * shared = &rampsData->sharedData;

  // Verify the ratio to be acceptable, return and set error otherwise
  if (shared->synRatioNum == 0 ||
      shared->synRatioDen == 0 ||
      shared->synRatioDen > shared->synRatioNum)
  {
    shared->mode = MODE_SYNCHRO_BAD_RATIO;
    return;
  }

  // Configure the values for the bresenham interpolation
  if (shared->synRatioDen < 0) { rampsData->syncData.yi = -1; }
  else { rampsData->syncData.yi = 1; }

  rampsData->syncData.D = 2 * (shared->synRatioDen - shared->synRatioNum);

  // Configure the timer settings for the pwm generation, will be used as one pulse
  __HAL_TIM_SET_AUTORELOAD(rampsData->motorTimer, 100);
  __HAL_TIM_SET_COMPARE(rampsData->motorTimer, TIM_CHANNEL_1, 10);

  // Ensure the current and final positions are equal
  shared->currentPosition = shared->finalPosition;
  shared->mode = MODE_SYNCHRO;
}

/**
 * This function has to be called from a simple timer interrupt routine
 * happening at regular intervals which shall match the maximum frequency
 * supported by either the controller speed or the stepper motor controller
 * Starting timer interval frequency set to 50Khz
 * @param rampsData Reference to the ramps handler data structure
 */
void SyncMotionIsr(rampsHandler_t * rampsData) {
  rampsSharedData_t * data = &(rampsData->sharedData);
  rampsSyncData_t * sync_data = &(rampsData->syncData);

  // Handle encoder overflow
  sync_data->encoderPrevious = sync_data->encoderCurrent;
  sync_data->encoderCurrent = __HAL_TIM_GET_COUNTER(rampsData->encoderTimer);
  sync_data->encoderDelta = sync_data->encoderCurrent - sync_data->encoderPrevious;

  // Encoder position update into the data structure
  sync_data->positionPrevious = sync_data->positionCurrent;
  sync_data->positionCurrent += sync_data->encoderDelta;
  data->encoderPosition = sync_data->positionCurrent;

  // Skip Conditions
  if (data->mode != MODE_SYNCHRO || data->finalPosition != data->currentPosition) {
    return;
  }

  if (sync_data->positionPrevious < sync_data->positionCurrent)
  {
    HAL_GPIO_WritePin(DIR_GPIO_PORT, DIR_PIN, GPIO_PIN_SET);
    sync_data->direction = +1;
    for (int32_t x = sync_data->positionPrevious; x < sync_data->positionCurrent; ++x) {
      if (sync_data->D > 0) {
        // Error greater than 0, step forward the controlled axis
        data->finalPosition += sync_data->yi;
        sync_data->D = sync_data->D + (2 * (data->synRatioDen - data->synRatioNum));
      } else {
        sync_data->D = sync_data->D + 2 * data->synRatioDen;
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
        data->finalPosition -= sync_data->yi;
        sync_data->D = sync_data->D - (2 * (data->synRatioDen - data->synRatioNum));
      } else {
        sync_data->D = sync_data->D - 2 * data->synRatioDen;
      }
    }
  }

  // Now that the final position
  if (data->finalPosition != data->currentPosition) {
    HAL_TIM_PWM_Start_IT(rampsData->motorTimer, TIM_CHANNEL_1);
  }
}

/**
 * This method is used to initialize the RTOS task responsible for controlling the ramps
 * ramp generator.
 */
void RampsTaskInit(rampsHandler_t * rampsData)
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
  rampsHandler_t * rampsData = (rampsHandler_t *)argument;
  rampsSharedData_t * shared = &rampsData->sharedData;
  rampsIndexData_t * indexData = &rampsData->indexData;
  rampsSyncData_t * syncData = &rampsData->syncData;

  for(;;)
  {
    osDelay(50);

    // Handle sync mode request
    if (shared->mode == MODE_SYNCHRO_INIT) {
      SyncMotionInit(rampsData);
      shared->mode = MODE_SYNCHRO; // Set proper mode
    }

    // Handle request to set encoder count value
    if (shared->mode == MODE_SET_ENCODER) {
      // Reset everything and configure the provided value

      // Counter reset
      __HAL_TIM_SET_COUNTER(rampsData->encoderTimer, 0);
      syncData->encoderPrevious = 0;

      // Sync data struct reset
      syncData->positionCurrent = shared->encoderPresetValue;
      syncData->positionCurrent = shared->encoderPresetValue;

      // Shared data struct reset
      shared->encoderPosition = syncData->positionCurrent;
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
        HAL_GPIO_WritePin(rampsData->directionPinPort, rampsData->directionPin, 1);
      } else {
        HAL_GPIO_WritePin(rampsData->directionPinPort, rampsData->directionPin, 0);
      }

      // Set mode
      // Reset counters used for ramps generation, will have to improve this
      shared->mode = MODE_INDEX;
      indexData->currentStep = 0;
      indexData->totalSteps = abs(shared->finalPosition - shared->currentPosition);

      // Start the timer responsible for the ramps generation
      HAL_TIM_PWM_Start_IT(rampsData->motorTimer, TIM_CHANNEL_1);
    }

    if (shared->mode == MODE_INDEX && shared->finalPosition == shared->currentPosition) {
      shared->mode = MODE_HALT;
      HAL_TIM_PWM_Stop_IT(rampsData->motorTimer, TIM_CHANNEL_1);
    }
  }
}