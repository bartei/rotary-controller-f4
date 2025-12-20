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
#define SERVO_COUNT 3


typedef struct {
    int32_t delta;
    uint32_t oldPosition;
    uint32_t position;
    int32_t scaledDelta;
    int32_t error;
} deltaPosError_t;

typedef struct {
    TIM_HandleTypeDef *timerHandle;
    int32_t position;
    int32_t speed;
    int32_t syncRatioNum, syncRatioDen;
    uint16_t syncEnable;
} input_t;

typedef struct {
    float maxSpeed;
    float currentSpeed;
    float jogSpeed;
    float acceleration;
    int32_t stepsToGo;
    uint32_t destinationSteps;
    uint32_t currentSteps;
    uint32_t desiredSteps;

    int32_t currentDirection;
    int32_t previousDirection;

    GPIO_TypeDef *stepPort;
    uint16_t stepPin;
    GPIO_TypeDef *dirPort;
    uint16_t dirPin;

    bool syncEnable;
    uint32_t syncScaleIndex;
    uint32_t syncRatioNum;
    uint32_t syncRatioDen;

    deltaPosError_t syncDeltaPos;
} servo_t;

typedef struct {
    uint32_t servoCurrent;
    uint32_t servoDesired;
    uint32_t stepsToGo;
    float servoSpeed;
    int32_t scaleCurrent[SCALES_COUNT];
    int32_t scaleSpeed[SCALES_COUNT];
    uint32_t cycles;
    uint32_t executionInterval;
    uint16_t servoMode; // Servo modes: 0=disabled, 1=sync/index, 2=jog
} fastData_t;

typedef struct {
    // Modbus shared data
    uint32_t executionInterval;
    uint32_t executionIntervalPrevious;
    uint32_t executionIntervalCurrent;
    uint32_t executionCycles;

    GPIO_TypeDef *enaPort;
    uint16_t enaPin;

    GPIO_TypeDef *usrLedPort;
    uint16_t usrLedPin;

    servo_t servo[SERVO_COUNT];
    input_t scales[SCALES_COUNT];

    fastData_t fastData;

    // STM32 Related
    TIM_HandleTypeDef *synchroRefreshTimer;
    UART_HandleTypeDef *modbusUart;

    deltaPosError_t scalesDeltaPos[SCALES_COUNT];
    deltaPosError_t scalesSyncDeltaPos[SCALES_COUNT];
    deltaPosError_t scalesSpeed[SCALES_COUNT];
    deltaPosError_t rampsDeltaPos;

} rampsHandler_t;

extern modbusHandler_t RampsModbusData;

void RampsStart(rampsHandler_t *rampsData);

void SynchroRefreshTimerIsr(rampsHandler_t *data);

_Noreturn void updateSpeedTask(void *argument);

_Noreturn void userLedTask(void *argument);

_Noreturn void servoEnableTask(void *argument);

#endif