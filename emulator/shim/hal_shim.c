/*
 * HAL shim implementation for emulator.
 *
 * Provides minimal working implementations of STM32 HAL functions
 * that route through the emulator's hardware state (emulator_state.h).
 */

#include "stm32f4xx_hal.h"
#include "emulator_state.h"
#include <string.h>

/* ========================================================================
 * Hardware register instances (emulated)
 * ======================================================================== */

/* GPIO port instances */
GPIO_TypeDef emu_gpioa = {0};
GPIO_TypeDef emu_gpiob = {0};
GPIO_TypeDef emu_gpioc = {0};

/* USART instance */
USART_TypeDef emu_usart1 = {0};

/* RCC instance */
RCC_TypeDef emu_rcc = {0};

/* DWT / CoreDebug instances */
DWT_Type emu_dwt = {0};
CoreDebug_Type emu_coreDebug = {0};

/* Timer peripheral instances (one per encoder + one for ISR + one for systick) */
static TIM_TypeDef emu_tim1_regs = {0};
static TIM_TypeDef emu_tim2_regs = {0};
static TIM_TypeDef emu_tim3_regs = {0};
static TIM_TypeDef emu_tim4_regs = {0};
static TIM_TypeDef emu_tim9_regs = {0};
static TIM_TypeDef emu_tim11_regs = {0};

/* Timer handle instances (firmware externs these) */
TIM_HandleTypeDef htim1  = { .Instance = &emu_tim1_regs };
TIM_HandleTypeDef htim2  = { .Instance = &emu_tim2_regs };
TIM_HandleTypeDef htim3  = { .Instance = &emu_tim3_regs };
TIM_HandleTypeDef htim4  = { .Instance = &emu_tim4_regs };
TIM_HandleTypeDef htim9  = { .Instance = &emu_tim9_regs, .Init = { .Prescaler = 99, .Period = 9 } };
TIM_HandleTypeDef htim11 = { .Instance = &emu_tim11_regs };

/* UART handle instance */
UART_HandleTypeDef huart1 = { .Instance = &emu_usart1 };

/* ========================================================================
 * Global emulator hardware state
 * ======================================================================== */

EmulatorHardwareState emu_hw = {0};

/* ========================================================================
 * HAL Init / Tick
 * ======================================================================== */

HAL_StatusTypeDef HAL_Init(void) {
    return HAL_OK;
}

static volatile uint32_t uwTick = 0;

void HAL_IncTick(void) {
    uwTick++;
    emu_hw.hal_tick = uwTick;
}

uint32_t HAL_GetTick(void) {
    return uwTick;
}

void Error_Handler(void) {
    /* In emulator, just spin (caller made a mistake) */
    while(1) {}
}

/* ========================================================================
 * GPIO
 * ======================================================================== */

void HAL_GPIO_Init(GPIO_TypeDef *GPIOx, GPIO_InitTypeDef *GPIO_Init) {
    (void)GPIOx;
    (void)GPIO_Init;
    /* No-op: emulator doesn't need real GPIO configuration */
}

void HAL_GPIO_WritePin(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState) {
    int state = (PinState == GPIO_PIN_SET) ? 1 : 0;

    /* Track step/dir/ena pins for physics model */
    if (GPIOx == GPIOA) {
        if (GPIO_Pin == GPIO_PIN_0) {  /* STEP_PIN */
            emu_hw.step_pin = state;
        }
        if (GPIO_Pin == GPIO_PIN_3) {  /* SPARE_2_PIN */
            emu_hw.spare2_pin = state;
        }
    } else if (GPIOx == GPIOB) {
        if (GPIO_Pin == GPIO_PIN_14) {  /* DIR_PIN */
            emu_hw.dir_pin = state;
        }
        if (GPIO_Pin == GPIO_PIN_15) {  /* ENA_PIN */
            emu_hw.ena_pin = state;
        }
    }

    /* Forward to callback if registered */
    if (emu_hw.on_gpio_write) {
        emu_hw.on_gpio_write(GPIOx, GPIO_Pin, state);
    }
}

void HAL_GPIO_TogglePin(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin) {
    /* Read ODR, toggle the bit, write back */
    if (GPIOx->ODR & GPIO_Pin) {
        GPIOx->ODR &= ~GPIO_Pin;
        HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET);
    } else {
        GPIOx->ODR |= GPIO_Pin;
        HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_SET);
    }
}

GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin) {
    return (GPIOx->IDR & GPIO_Pin) ? GPIO_PIN_SET : GPIO_PIN_RESET;
}

/* ========================================================================
 * Timer
 * ======================================================================== */

HAL_StatusTypeDef HAL_TIM_Encoder_Init(TIM_HandleTypeDef *htim, TIM_Encoder_InitTypeDef *sConfig) {
    (void)htim;
    (void)sConfig;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_TIM_Encoder_Start(TIM_HandleTypeDef *htim, uint32_t Channel) {
    (void)htim;
    (void)Channel;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_TIM_Base_Start_IT(TIM_HandleTypeDef *htim) {
    (void)htim;
    /* The ISR is driven by the emulator's ISR thread, not by hardware interrupts */
    return HAL_OK;
}

HAL_StatusTypeDef HAL_TIMEx_MasterConfigSynchronization(TIM_HandleTypeDef *htim,
                                                          TIM_MasterConfigTypeDef *sMasterConfig) {
    (void)htim;
    (void)sMasterConfig;
    return HAL_OK;
}

void HAL_TIM_IRQHandler(TIM_HandleTypeDef *htim) {
    (void)htim;
    /* Not used in emulator — we call SynchroRefreshTimerIsr directly */
}

__attribute__((weak)) void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    (void)htim;
}

/*
 * Update the virtual timer counter values from the physics model.
 * Called by the physics thread before each ISR tick.
 * Index mapping: 0→TIM1, 1→TIM2, 2→TIM3, 3→TIM4
 */
void emu_update_timer_counters(void) {
    emu_tim1_regs.CNT = emu_hw.scale_counters[0];
    emu_tim2_regs.CNT = emu_hw.scale_counters[1];
    emu_tim3_regs.CNT = emu_hw.scale_counters[2];
    emu_tim4_regs.CNT = emu_hw.scale_counters[3];
}

/* ========================================================================
 * UART
 *
 * The firmware uses USART_HW mode (not DMA), so the flow is:
 * RX: HAL_UART_Receive_IT arms → transport injects byte → HAL_UART_RxCpltCallback
 * TX: HAL_UART_Transmit_IT sends → callback → HAL_UART_TxCpltCallback
 * ======================================================================== */

HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size) {
    huart->pRxBuffPtr = pData;
    huart->RxXferSize = Size;
    huart->RxXferCount = Size;
    huart->RxState = HAL_UART_STATE_BUSY_RX;

    /* Tell the transport layer we're ready for a byte */
    emu_hw.uart_rx_buf = pData;
    emu_hw.uart_rx_size = Size;
    emu_hw.uart_rx_armed = 1;

    return HAL_OK;
}

HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *huart, const uint8_t *pData, uint16_t Size) {
    /* Route bytes to transport layer */
    if (emu_hw.on_uart_tx) {
        emu_hw.on_uart_tx(pData, Size);
    }

    /* Set TC flag so the busy-wait in sendTxBuffer completes */
    huart->Instance->SR |= USART_SR_TC;
    huart->gState = HAL_UART_STATE_READY;

    /* Notify the Modbus task that TX is complete */
    HAL_UART_TxCpltCallback(huart);

    return HAL_OK;
}

HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *huart, const uint8_t *pData, uint16_t Size) {
    /* Same as Transmit_IT for the emulator */
    return HAL_UART_Transmit_IT(huart, pData, Size);
}

HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size) {
    (void)huart; (void)pData; (void)Size;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_UART_DMAStop(UART_HandleTypeDef *huart) {
    (void)huart;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_UART_AbortReceive_IT(UART_HandleTypeDef *huart) {
    huart->RxState = HAL_UART_STATE_READY;
    emu_hw.uart_rx_armed = 0;
    return HAL_OK;
}

uint32_t HAL_UART_GetState(UART_HandleTypeDef *huart) {
    (void)huart;
    return HAL_UART_STATE_READY;
}

void HAL_UART_IRQHandler(UART_HandleTypeDef *huart) {
    (void)huart;
}

HAL_StatusTypeDef HAL_HalfDuplex_EnableTransmitter(UART_HandleTypeDef *huart) {
    (void)huart;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_HalfDuplex_EnableReceiver(UART_HandleTypeDef *huart) {
    (void)huart;
    return HAL_OK;
}

/* ========================================================================
 * RCC (no-ops)
 * ======================================================================== */

HAL_StatusTypeDef HAL_RCC_OscConfig(RCC_OscInitTypeDef *RCC_OscInitStruct) {
    (void)RCC_OscInitStruct;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_RCC_ClockConfig(RCC_ClkInitTypeDef *RCC_ClkInitStruct, uint32_t FLatency) {
    (void)RCC_ClkInitStruct; (void)FLatency;
    return HAL_OK;
}

/* ========================================================================
 * Event Log
 * ======================================================================== */

#include <stdio.h>
#include <stdarg.h>
#include <time.h>

void emu_log_event(const char *fmt, ...) {
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    struct tm tm;
    localtime_r(&ts.tv_sec, &tm);

    int idx = emu_hw.event_log_head % 256;
    int off = snprintf(emu_hw.event_log[idx], 128, "%02d:%02d:%02d.%03ld ",
                       tm.tm_hour, tm.tm_min, tm.tm_sec, ts.tv_nsec / 1000000);

    va_list ap;
    va_start(ap, fmt);
    vsnprintf(emu_hw.event_log[idx] + off, 128 - off, fmt, ap);
    va_end(ap);

    emu_hw.event_log_head++;
    if (emu_hw.event_log_count < 256) emu_hw.event_log_count++;
}
