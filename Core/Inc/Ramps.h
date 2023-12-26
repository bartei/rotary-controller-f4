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
#define DIR_PIN GPIO_PIN_14
#define DIR_GPIO_PORT GPIOB
#define ENA_PIN GPIO_PIN_15
#define ENA_GPIO_PORT GPIOB

typedef enum
{
    MODE_HALT = 0,
    MODE_SYNCHRO = 20,
    MODE_SYNCHRO_INIT = 21,
    MODE_JOG = 30,
    MODE_JOG_FW = 31,
    MODE_JOG_BW = 32,
    MODE_SET_ENCODER = 40,
    MODE_SYNCHRO_BAD_RATIO = 101,
} ramps_mode_t ;

typedef struct {
    TIM_HandleTypeDef * timerHandle;
    int32_t encoderPrevious;
    int32_t encoderCurrent;
    int32_t ratioNum;
    int32_t ratioDen;
    int32_t maxValue;
    int32_t minValue;
    int32_t position;
    int32_t error;
} input_t;

typedef struct {
    float minSpeed, maxSpeed, currentSpeed, acceleration;
    float absoluteOffset;
    float indexOffset;
    int32_t inputOffset;
    float desiredPosition;
    float currentPosition;
    int32_t currentSteps;
    int32_t desiredSteps;
    int32_t ratioNum;
    int32_t ratioDen;
    int32_t maxValue;
    int32_t minValue;
    int32_t position;
    int32_t error;
} servo_t;

typedef struct {
    int32_t divisions;
    int32_t curIndex;
    int32_t reqIndex;
} index_t;

typedef struct {
    ramps_mode_t mode; // 0
    uint16_t encoderPresetIndex;
    int32_t encoderPresetValue;
    int32_t unused_14;
    index_t index;
    servo_t servo;
    input_t scales[SCALES_COUNT];
} rampsSharedData_t;


typedef struct {
    // Modbus shared data
    rampsSharedData_t shared;

    // STM32 Related
    TIM_HandleTypeDef * motorPwmTimer;
    TIM_HandleTypeDef * synchroRefreshTimer;

    UART_HandleTypeDef * modbusUart;

    GPIO_TypeDef * directionPinPort;
    uint16_t directionPin;

    GPIO_TypeDef * enablePinPort;
    uint16_t enablePin;

    // FreeRTOS related
    osThreadId_t TaskRampsHandle;
} rampsHandler_t;

void RampsStart(rampsHandler_t * rampsData);
void MotorPwmTimerISR(rampsHandler_t * data);
void IndexRefreshTimerISR(rampsHandler_t * data);
void SynchroRefreshTimerIsr(rampsHandler_t * data);
void SyncMotionInit(rampsHandler_t * data);
void StartRampsTask(rampsHandler_t * rampsData);
void RampsTask(void *argument);

#endif
