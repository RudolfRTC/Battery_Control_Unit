/**
 * @file    timestamp.h
 * @brief   Timestamp and timing utilities
 * @author  Battery Control Unit Development Team
 * @date    2026-01-09
 * @version 1.0.0
 *
 * @note    MISRA C:2012 compliant
 * @note    Provides microsecond and millisecond timing
 *
 * @copyright Copyright (c) 2026
 */

#ifndef TIMESTAMP_H
#define TIMESTAMP_H

#ifdef __cplusplus
extern "C" {
#endif

/*============================================================================*/
/* INCLUDES                                                                   */
/*============================================================================*/
#include "app_types.h"
#include <stdint.h>
#include <stdbool.h>

/*============================================================================*/
/* PUBLIC FUNCTION PROTOTYPES                                                 */
/*============================================================================*/
/* Note: Timestamp_t, Timestamp_Init, Timestamp_Diff are defined in app_types.h */

/**
 * @brief Initialize timestamp module
 * @return Status code
 */
Status_t Timestamp_ModuleInit(void);

/**
 * @brief Get current millisecond count
 * @return Milliseconds since system start
 */
uint32_t Timestamp_GetMillis(void);

/**
 * @brief Get current microsecond count
 * @return Microseconds since system start
 */
uint64_t Timestamp_GetMicros(void);

/**
 * @brief SysTick interrupt handler (call from SysTick_Handler)
 */
void Timestamp_SysTick_Handler(void);

/**
 * @brief Check if a timeout has elapsed
 * @param[in] start_ms Start time in milliseconds
 * @param[in] timeout_ms Timeout duration in milliseconds
 * @return true if timeout elapsed, false otherwise
 */
bool Timestamp_IsElapsed(uint32_t start_ms, uint32_t timeout_ms);

/**
 * @brief Delay for specified milliseconds (blocking)
 * @param[in] ms Milliseconds to delay
 */
void Timestamp_DelayMs(uint32_t ms);

#ifdef __cplusplus
}
#endif

#endif /* TIMESTAMP_H */

/*============================================================================*/
/* END OF FILE                                                                */
/*============================================================================*/
