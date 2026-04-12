/*
 * main.h shim for emulator.
 * Overrides the firmware's Core/Inc/main.h.
 * Provides the same declarations the firmware code expects.
 */
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include "Ramps.h"

void Error_Handler(void);

/* Pin defines that main.h normally provides */
#define SPARE_1_Pin GPIO_PIN_1
#define SPARE_1_GPIO_Port GPIOA
#define USR_LED_Pin GPIO_PIN_12
#define USR_LED_GPIO_Port GPIOB

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
