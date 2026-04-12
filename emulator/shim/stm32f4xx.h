/*
 * STM32F4xx device header shim for emulator.
 * Provides minimal register structure stubs and includes core_cm4.
 */
#ifndef __STM32F4xx_H
#define __STM32F4xx_H

#include <stdint.h>
#include "core_cm4.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Undef termios macros that clash with register field names.
 * This is safe because the emulator never uses termios CR1/CR2/CR3
 * from this header's context — transport.cpp includes termios.h
 * before this header gets pulled in via stm32f4xx_hal.h.
 */
#undef CR1
#undef CR2
#undef CR3

/* Minimal USART register structure (only fields accessed by Modbus.c) */
typedef struct {
    volatile uint32_t SR;    /* Status register */
    volatile uint32_t DR;    /* Data register */
    volatile uint32_t BRR;   /* Baud rate register */
    volatile uint32_t CR1;
    volatile uint32_t CR2;
    volatile uint32_t CR3;
    volatile uint32_t GTPR;
} USART_TypeDef;

#define USART_SR_TC  (1U << 6)  /* Transmission Complete flag */
#define USART_SR_TXE (1U << 7)  /* Transmit Data Register Empty */

/* GPIO port structure */
typedef struct {
    volatile uint32_t MODER;
    volatile uint32_t OTYPER;
    volatile uint32_t OSPEEDR;
    volatile uint32_t PUPDR;
    volatile uint32_t IDR;
    volatile uint32_t ODR;
    volatile uint32_t BSRR;
    volatile uint32_t LCKR;
    volatile uint32_t AFR[2];
} GPIO_TypeDef;

/* Timer counter register structure (only fields accessed via macros) */
typedef struct {
    volatile uint32_t CR1;
    volatile uint32_t CR2;
    volatile uint32_t SMCR;
    volatile uint32_t DIER;
    volatile uint32_t SR;
    volatile uint32_t EGR;
    volatile uint32_t CCMR1;
    volatile uint32_t CCMR2;
    volatile uint32_t CCER;
    volatile uint32_t CNT;   /* Counter value - this is what __HAL_TIM_GET_COUNTER reads */
    volatile uint32_t PSC;
    volatile uint32_t ARR;
    volatile uint32_t RCR;
    volatile uint32_t CCR1;
    volatile uint32_t CCR2;
    volatile uint32_t CCR3;
    volatile uint32_t CCR4;
} TIM_TypeDef;

/* DMA stream structure (minimal) */
typedef struct {
    volatile uint32_t CR;
    volatile uint32_t NDTR;
    volatile uint32_t PAR;
    volatile uint32_t M0AR;
    volatile uint32_t M1AR;
    volatile uint32_t FCR;
} DMA_Stream_TypeDef;

/* RCC structure (minimal stub) */
typedef struct {
    volatile uint32_t CR;
    volatile uint32_t PLLCFGR;
    volatile uint32_t CFGR;
    volatile uint32_t CIR;
    volatile uint32_t AHB1RSTR;
    volatile uint32_t AHB2RSTR;
    volatile uint32_t RESERVED0[2];
    volatile uint32_t APB1RSTR;
    volatile uint32_t APB2RSTR;
    volatile uint32_t RESERVED1[2];
    volatile uint32_t AHB1ENR;
} RCC_TypeDef;

/* GPIO port instances */
extern GPIO_TypeDef emu_gpioa, emu_gpiob, emu_gpioc;
#define GPIOA (&emu_gpioa)
#define GPIOB (&emu_gpiob)
#define GPIOC (&emu_gpioc)

/* RCC instance */
extern RCC_TypeDef emu_rcc;
#define RCC (&emu_rcc)

/* USART instances */
extern USART_TypeDef emu_usart1;

/* STM32F411xE define (expected by HAL headers) */
#ifndef STM32F411xE
#define STM32F411xE
#endif

#ifdef __cplusplus
}
#endif

#endif /* __STM32F4xx_H */
