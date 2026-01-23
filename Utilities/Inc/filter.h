/**
 * @file    filter.h
 * @brief   Digital filter utilities
 * @author  Battery Control Unit Development Team
 * @date    2026-01-09
 * @version 1.0.0
 *
 * @note    MISRA C:2012 compliant
 *
 * @copyright Copyright (c) 2026
 */

#ifndef FILTER_H
#define FILTER_H

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
/* CONSTANTS                                                                  */
/*============================================================================*/

#define FILTER_MA_MAX_SIZE      (32U)   /**< Maximum moving average window size */

/*============================================================================*/
/* TYPE DEFINITIONS                                                           */
/*============================================================================*/

/**
 * @brief Moving average filter structure
 */
typedef struct {
    int32_t buffer[FILTER_MA_MAX_SIZE];  /**< Sample buffer */
    uint8_t size;                         /**< Window size */
    uint8_t index;                        /**< Current index */
    uint8_t count;                        /**< Number of samples */
    int32_t sum;                          /**< Running sum */
} Filter_MovingAverage_t;

/**
 * @brief IIR (low-pass) filter structure
 */
typedef struct {
    int32_t output;       /**< Current output value */
    uint8_t alpha;        /**< Filter coefficient (0-100) */
    uint8_t alphaComplement; /**< 100 - alpha */
    bool initialized;     /**< First sample flag */
} Filter_IIR_t;

/**
 * @brief Debounce filter structure
 */
typedef struct {
    bool state;           /**< Current debounced state */
    bool lastRaw;         /**< Last raw input */
    uint32_t counter;     /**< Debounce counter */
    uint32_t threshold;   /**< Debounce threshold (ms) */
    uint32_t samplePeriod; /**< Sample period (ms) */
} Filter_Debounce_t;

/*============================================================================*/
/* PUBLIC FUNCTION PROTOTYPES                                                 */
/*============================================================================*/

/**
 * @brief Initialize moving average filter
 * @param[out] pFilter Pointer to filter structure
 * @param[in] size Window size (max FILTER_MA_MAX_SIZE)
 * @return Status code
 */
Status_t Filter_MA_Init(Filter_MovingAverage_t *pFilter, uint8_t size);

/**
 * @brief Update moving average filter with new sample
 * @param[in,out] pFilter Pointer to filter structure
 * @param[in] sample New sample value
 * @return Filtered output value
 */
int32_t Filter_MA_Update(Filter_MovingAverage_t *pFilter, int32_t sample);

/**
 * @brief Reset moving average filter
 * @param[out] pFilter Pointer to filter structure
 */
void Filter_MA_Reset(Filter_MovingAverage_t *pFilter);

/**
 * @brief Initialize IIR low-pass filter
 * @param[out] pFilter Pointer to filter structure
 * @param[in] alpha Filter coefficient (0-100, higher = less filtering)
 * @param[in] scale Scale factor (typically 100)
 * @return Status code
 */
Status_t Filter_IIR_Init(Filter_IIR_t *pFilter, uint8_t alpha, uint8_t scale);

/**
 * @brief Update IIR filter with new sample
 * @param[in,out] pFilter Pointer to filter structure
 * @param[in] sample New sample value
 * @return Filtered output value
 */
int32_t Filter_IIR_Update(Filter_IIR_t *pFilter, int32_t sample);

/**
 * @brief Reset IIR filter
 * @param[out] pFilter Pointer to filter structure
 */
void Filter_IIR_Reset(Filter_IIR_t *pFilter);

/**
 * @brief Initialize debounce filter
 * @param[out] pFilter Pointer to filter structure
 * @param[in] threshold_ms Debounce time in milliseconds
 * @param[in] samplePeriod_ms Sample period in milliseconds
 * @return Status code
 */
Status_t Filter_Debounce_Init(Filter_Debounce_t *pFilter, uint32_t threshold_ms, uint32_t samplePeriod_ms);

/**
 * @brief Update debounce filter with new sample
 * @param[in,out] pFilter Pointer to filter structure
 * @param[in] rawState Raw input state
 * @return Debounced output state
 */
bool Filter_Debounce_Update(Filter_Debounce_t *pFilter, bool rawState);

/**
 * @brief Reset debounce filter
 * @param[out] pFilter Pointer to filter structure
 */
void Filter_Debounce_Reset(Filter_Debounce_t *pFilter);

#ifdef __cplusplus
}
#endif

#endif /* FILTER_H */

/*============================================================================*/
/* END OF FILE                                                                */
/*============================================================================*/
