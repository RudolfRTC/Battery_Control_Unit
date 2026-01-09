/**
 * @file    timestamp.h
 * @brief   System timestamp and timing utilities
 * @author  Battery Control Unit Development Team
 * @date    2026-01-09
 * @version 1.0.0
 *
 * @note    MISRA C:2012 compliant
 * @note    Uses SysTick timer for 1ms resolution
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

/*============================================================================*/
/* FUNCTION PROTOTYPES                                                        */
/*============================================================================*/

/**
 * @brief Initialize timestamp module
 * @return STATUS_OK on success
 * @note  Configures SysTick for 1ms interrupts
 */
Status_t Timestamp_Init(void);

/**
 * @brief Get current timestamp in milliseconds
 * @return Current timestamp (ms since startup)
 * @note  32-bit counter wraps after ~49.7 days
 */
uint32_t Timestamp_GetMillis(void);

/**
 * @brief Get current timestamp in microseconds
 * @return Current timestamp (us since startup)
 * @note  Uses DWT cycle counter for high resolution
 */
uint64_t Timestamp_GetMicros(void);

/**
 * @brief Get elapsed time since reference timestamp
 * @param[in] reference_ms Reference timestamp in milliseconds
 * @return Elapsed time in milliseconds
 * @note  Handles 32-bit wraparound correctly
 */
uint32_t Timestamp_GetElapsed(uint32_t reference_ms);

/**
 * @brief Check if timeout has occurred
 * @param[in] start_ms    Start timestamp in milliseconds
 * @param[in] timeout_ms  Timeout duration in milliseconds
 * @return true if timeout occurred, false otherwise
 */
bool Timestamp_IsTimeout(uint32_t start_ms, uint32_t timeout_ms);

/**
 * @brief Blocking delay in milliseconds
 * @param[in] delay_ms Delay duration in milliseconds
 * @note  Uses WFI (Wait For Interrupt) for power efficiency
 */
void Timestamp_DelayMs(uint32_t delay_ms);

/**
 * @brief Blocking delay in microseconds
 * @param[in] delay_us Delay duration in microseconds
 * @note  Busy-wait loop, accurate for short delays
 */
void Timestamp_DelayUs(uint32_t delay_us);

/**
 * @brief SysTick interrupt handler (called every 1ms)
 * @note  Must be called from SysTick_Handler() ISR
 */
void Timestamp_SysTick_Handler(void);

/**
 * @brief Convert timestamp to time structure
 * @param[in]  timestamp_ms Timestamp in milliseconds
 * @param[out] pHours       Hours component
 * @param[out] pMinutes     Minutes component
 * @param[out] pSeconds     Seconds component
 * @param[out] pMillis      Milliseconds component
 * @return STATUS_OK on success
 */
Status_t Timestamp_ToTime(uint32_t timestamp_ms, uint32_t *pHours,
                          uint32_t *pMinutes, uint32_t *pSeconds,
                          uint32_t *pMillis);

/**
 * @brief Get CPU cycle count (for profiling)
 * @return Current CPU cycle count
 * @note  Uses DWT CYCCNT register
 */
uint32_t Timestamp_GetCycles(void);

/**
 * @brief Convert cycles to microseconds
 * @param[in] cycles Number of CPU cycles
 * @return Time in microseconds
 */
uint32_t Timestamp_CyclesToMicros(uint32_t cycles);

/**
 * @brief Convert cycles to nanoseconds
 * @param[in] cycles Number of CPU cycles
 * @return Time in nanoseconds
 */
uint32_t Timestamp_CyclesToNanos(uint32_t cycles);

#ifdef __cplusplus
}
#endif

#endif /* TIMESTAMP_H */
