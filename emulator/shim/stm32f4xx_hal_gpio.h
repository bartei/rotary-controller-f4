/*
 * HAL GPIO shim for emulator.
 */
#ifndef __STM32F4xx_HAL_GPIO_H
#define __STM32F4xx_HAL_GPIO_H

#include "stm32f4xx.h"
#include "stm32f4xx_hal_def.h"

#ifdef __cplusplus
extern "C" {
#endif

/* GPIO Pin definitions */
#define GPIO_PIN_0   ((uint16_t)0x0001)
#define GPIO_PIN_1   ((uint16_t)0x0002)
#define GPIO_PIN_2   ((uint16_t)0x0004)
#define GPIO_PIN_3   ((uint16_t)0x0008)
#define GPIO_PIN_4   ((uint16_t)0x0010)
#define GPIO_PIN_5   ((uint16_t)0x0020)
#define GPIO_PIN_6   ((uint16_t)0x0040)
#define GPIO_PIN_7   ((uint16_t)0x0080)
#define GPIO_PIN_8   ((uint16_t)0x0100)
#define GPIO_PIN_9   ((uint16_t)0x0200)
#define GPIO_PIN_10  ((uint16_t)0x0400)
#define GPIO_PIN_11  ((uint16_t)0x0800)
#define GPIO_PIN_12  ((uint16_t)0x1000)
#define GPIO_PIN_13  ((uint16_t)0x2000)
#define GPIO_PIN_14  ((uint16_t)0x4000)
#define GPIO_PIN_15  ((uint16_t)0x8000)

/* GPIO Pin State */
typedef enum {
    GPIO_PIN_RESET = 0U,
    GPIO_PIN_SET
} GPIO_PinState;

/* GPIO Init Structure */
typedef struct {
    uint32_t Pin;
    uint32_t Mode;
    uint32_t Pull;
    uint32_t Speed;
    uint32_t Alternate;
} GPIO_InitTypeDef;

/* GPIO Mode definitions */
#define GPIO_MODE_INPUT           0x00000000U
#define GPIO_MODE_OUTPUT_PP       0x00000001U
#define GPIO_MODE_OUTPUT_OD       0x00000011U
#define GPIO_MODE_AF_PP           0x00000002U
#define GPIO_MODE_AF_OD           0x00000012U
#define GPIO_MODE_ANALOG          0x00000003U

/* GPIO Pull definitions */
#define GPIO_NOPULL               0x00000000U
#define GPIO_PULLUP               0x00000001U
#define GPIO_PULLDOWN             0x00000002U

/* GPIO Speed definitions */
#define GPIO_SPEED_FREQ_LOW       0x00000000U
#define GPIO_SPEED_FREQ_MEDIUM    0x00000001U
#define GPIO_SPEED_FREQ_HIGH      0x00000002U
#define GPIO_SPEED_FREQ_VERY_HIGH 0x00000003U

/* Function prototypes */
void HAL_GPIO_Init(GPIO_TypeDef *GPIOx, GPIO_InitTypeDef *GPIO_Init);
void HAL_GPIO_WritePin(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState);
void HAL_GPIO_TogglePin(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);

#ifdef __cplusplus
}
#endif

#endif /* __STM32F4xx_HAL_GPIO_H */
