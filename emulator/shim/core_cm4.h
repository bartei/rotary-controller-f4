/*
 * Cortex-M4 core register shim for emulator.
 * Provides DWT cycle counter and CoreDebug stubs.
 */
#ifndef __CORE_CM4_H
#define __CORE_CM4_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* DWT (Data Watchpoint and Trace) */
typedef struct {
    volatile uint32_t CTRL;
    uint32_t RESERVED0[5];
    volatile uint32_t CYCCNT;
} DWT_Type;

extern DWT_Type emu_dwt;
#define DWT (&emu_dwt)

#define DWT_CTRL_CYCCNTENA_Msk (1UL << 0)

/* CoreDebug */
typedef struct {
    volatile uint32_t DEMCR;
} CoreDebug_Type;

extern CoreDebug_Type emu_coreDebug;
#define CoreDebug (&emu_coreDebug)

#define CoreDebug_DEMCR_TRCENA_Msk (1UL << 24)

/* NVIC stubs */
typedef enum {
    NonMaskableInt_IRQn   = -14,
    HardFault_IRQn        = -13,
    MemoryManagement_IRQn = -12,
    BusFault_IRQn         = -11,
    UsageFault_IRQn       = -10,
    SVCall_IRQn           = -5,
    PendSV_IRQn           = -2,
    SysTick_IRQn          = -1,
    TIM1_BRK_TIM9_IRQn   = 24,
    TIM1_UP_TIM10_IRQn    = 25,
    USART1_IRQn           = 37,
} IRQn_Type;

static inline void __enable_irq(void) {}
static inline void __disable_irq(void) {}
static inline void NVIC_SetPriority(IRQn_Type IRQn, uint32_t priority) { (void)IRQn; (void)priority; }
static inline void NVIC_EnableIRQ(IRQn_Type IRQn) { (void)IRQn; }
static inline void HAL_NVIC_SetPriority(IRQn_Type IRQn, uint32_t PreemptPriority, uint32_t SubPriority) {
    (void)IRQn; (void)PreemptPriority; (void)SubPriority;
}
static inline void HAL_NVIC_EnableIRQ(IRQn_Type IRQn) { (void)IRQn; }

#ifdef __cplusplus
}
#endif

#endif /* __CORE_CM4_H */
