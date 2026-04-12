/*
 * HAL UART shim for emulator.
 */
#ifndef __STM32F4xx_HAL_UART_H
#define __STM32F4xx_HAL_UART_H

#include "stm32f4xx.h"
#include "stm32f4xx_hal_def.h"
#include "stm32f4xx_hal_dma.h"

#ifdef __cplusplus
extern "C" {
#endif

/* UART State */
#define HAL_UART_STATE_RESET     0x00000000U
#define HAL_UART_STATE_READY     0x00000020U
#define HAL_UART_STATE_BUSY      0x00000024U
#define HAL_UART_STATE_BUSY_TX   0x00000021U
#define HAL_UART_STATE_BUSY_RX   0x00000022U
#define HAL_UART_STATE_BUSY_TX_RX 0x00000023U

/* UART Handle */
typedef struct {
    USART_TypeDef *Instance;
    struct {
        uint32_t BaudRate;
        uint32_t WordLength;
        uint32_t StopBits;
        uint32_t Parity;
        uint32_t Mode;
        uint32_t HwFlowCtl;
        uint32_t OverSampling;
    } Init;
    uint8_t *pTxBuffPtr;
    uint16_t TxXferSize;
    volatile uint16_t TxXferCount;
    uint8_t *pRxBuffPtr;
    uint16_t RxXferSize;
    volatile uint16_t RxXferCount;
    DMA_HandleTypeDef *hdmatx;
    DMA_HandleTypeDef *hdmarx;
    volatile uint32_t gState;
    volatile uint32_t RxState;
    volatile uint32_t ErrorCode;
} UART_HandleTypeDef;

/* Function prototypes */
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *huart, const uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *huart, const uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_UART_DMAStop(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_UART_AbortReceive_IT(UART_HandleTypeDef *huart);
uint32_t HAL_UART_GetState(UART_HandleTypeDef *huart);
void HAL_UART_IRQHandler(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_HalfDuplex_EnableTransmitter(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_HalfDuplex_EnableReceiver(UART_HandleTypeDef *huart);

/* Callbacks (implemented in firmware UARTCallback.c) */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart);
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size);

#ifdef __cplusplus
}
#endif

#endif /* __STM32F4xx_HAL_UART_H */
