/*
 * HAL definition shim for emulator.
 * Provides base types: HAL_StatusTypeDef, HAL_LockTypeDef, etc.
 */
#ifndef __STM32F4xx_HAL_DEF_H
#define __STM32F4xx_HAL_DEF_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    HAL_OK       = 0x00U,
    HAL_ERROR    = 0x01U,
    HAL_BUSY     = 0x02U,
    HAL_TIMEOUT  = 0x03U
} HAL_StatusTypeDef;

typedef enum {
    HAL_UNLOCKED = 0x00U,
    HAL_LOCKED   = 0x01U
} HAL_LockTypeDef;

#define UNUSED(X) (void)(X)

#define __weak   __attribute__((weak))
#define __packed __attribute__((__packed__))

/* C11 _Noreturn is not valid in C++; map to the C++ attribute */
#ifdef __cplusplus
#ifndef _Noreturn
#define _Noreturn [[noreturn]]
#endif
#endif

#ifdef __cplusplus
}
#endif

#endif /* __STM32F4xx_HAL_DEF_H */
