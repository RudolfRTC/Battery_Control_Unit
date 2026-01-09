/**
 * @file    filter.h
 * @brief   Digital signal filtering utilities (moving average, IIR, median)
 * @author  Battery Control Unit Development Team
 * @date    2026-01-09
 * @version 1.0.0
 *
 * @note    MISRA C:2012 compliant
 * @note    Fixed-point arithmetic for deterministic performance
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

/*============================================================================*/
/* CONSTANTS                                                                  */
/*============================================================================*/

/** @brief Maximum filter buffer sizes */
#define FILTER_MA_MAX_SIZE      (32U)   /**< Max moving average size */
#define FILTER_MEDIAN_MAX_SIZE  (15U)   /**< Max median filter size (odd) */

/*============================================================================*/
/* TYPE DEFINITIONS                                                           */
/*============================================================================*/

/**
 * @brief Moving average filter structure
 */
typedef struct {
    int32_t  buffer[FILTER_MA_MAX_SIZE];  /**< Circular buffer */
    int64_t  sum;                          /**< Running sum */
    uint8_t  size;                         /**< Filter size (window) */
    uint8_t  index;                        /**< Current index */
    uint8_t  count;                        /**< Number of samples */
    bool     initialized;                  /**< Initialization flag */
} Filter_MovingAverage_t;

/**
 * @brief IIR (Infinite Impulse Response) filter structure
 * @note  First-order low-pass filter: y[n] = alpha * x[n] + (1-alpha) * y[n-1]
 */
typedef struct {
    int32_t  output;       /**< Previous output (y[n-1]) */
    uint16_t alpha;        /**< Filter coefficient (0-65535, scaled by 65536) */
    bool     initialized;  /**< Initialization flag */
} Filter_IIR_t;

/**
 * @brief Median filter structure
 */
typedef struct {
    int32_t  buffer[FILTER_MEDIAN_MAX_SIZE];  /**< Circular buffer */
    int32_t  sorted[FILTER_MEDIAN_MAX_SIZE];  /**< Sorted buffer */
    uint8_t  size;                             /**< Filter size (must be odd) */
    uint8_t  index;                            /**< Current index */
    uint8_t  count;                            /**< Number of samples */
    bool     initialized;                      /**< Initialization flag */
} Filter_Median_t;

/**
 * @brief Debounce filter for digital signals
 */
typedef struct {
    bool     state;            /**< Current stable state */
    bool     raw_state;        /**< Raw input state */
    uint16_t counter;          /**< Debounce counter */
    uint16_t threshold;        /**< Debounce threshold (samples) */
    bool     initialized;      /**< Initialization flag */
} Filter_Debounce_t;

/*============================================================================*/
/* FUNCTION PROTOTYPES - MOVING AVERAGE                                       */
/*============================================================================*/

/**
 * @brief Initialize moving average filter
 * @param[out] pFilter Pointer to filter structure
 * @param[in]  size    Window size (1 to FILTER_MA_MAX_SIZE)
 * @return STATUS_OK on success, error code otherwise
 */
Status_t Filter_MA_Init(Filter_MovingAverage_t *pFilter, uint8_t size);

/**
 * @brief Update moving average filter with new sample
 * @param[in,out] pFilter Pointer to filter structure
 * @param[in]     sample  New input sample
 * @return Filtered output value
 */
int32_t Filter_MA_Update(Filter_MovingAverage_t *pFilter, int32_t sample);

/**
 * @brief Reset moving average filter
 * @param[in,out] pFilter Pointer to filter structure
 * @return STATUS_OK on success
 */
Status_t Filter_MA_Reset(Filter_MovingAverage_t *pFilter);

/**
 * @brief Get current moving average output
 * @param[in] pFilter Pointer to filter structure
 * @return Current filtered output
 */
int32_t Filter_MA_GetOutput(const Filter_MovingAverage_t *pFilter);

/*============================================================================*/
/* FUNCTION PROTOTYPES - IIR LOW-PASS FILTER                                  */
/*============================================================================*/

/**
 * @brief Initialize IIR low-pass filter
 * @param[out] pFilter Pointer to filter structure
 * @param[in]  cutoff_hz Cutoff frequency in Hz
 * @param[in]  sample_rate_hz Sampling rate in Hz
 * @return STATUS_OK on success, error code otherwise
 * @note  Alpha calculated from cutoff frequency
 */
Status_t Filter_IIR_Init(Filter_IIR_t *pFilter, uint16_t cutoff_hz,
                         uint16_t sample_rate_hz);

/**
 * @brief Initialize IIR filter with explicit alpha
 * @param[out] pFilter Pointer to filter structure
 * @param[in]  alpha   Filter coefficient (0-65535, scaled by 65536)
 * @return STATUS_OK on success, error code otherwise
 * @note  alpha = 65536 means no filtering (pass-through)
 *        alpha = 32768 means 50% weighting
 *        alpha = 16384 means 25% weighting (more smoothing)
 */
Status_t Filter_IIR_InitWithAlpha(Filter_IIR_t *pFilter, uint16_t alpha);

/**
 * @brief Update IIR filter with new sample
 * @param[in,out] pFilter Pointer to filter structure
 * @param[in]     sample  New input sample
 * @return Filtered output value
 */
int32_t Filter_IIR_Update(Filter_IIR_t *pFilter, int32_t sample);

/**
 * @brief Reset IIR filter
 * @param[in,out] pFilter Pointer to filter structure
 * @return STATUS_OK on success
 */
Status_t Filter_IIR_Reset(Filter_IIR_t *pFilter);

/*============================================================================*/
/* FUNCTION PROTOTYPES - MEDIAN FILTER                                        */
/*============================================================================*/

/**
 * @brief Initialize median filter
 * @param[out] pFilter Pointer to filter structure
 * @param[in]  size    Window size (3, 5, 7, 9, 11, 13, or 15)
 * @return STATUS_OK on success, error code otherwise
 * @note  Size must be odd number
 */
Status_t Filter_Median_Init(Filter_Median_t *pFilter, uint8_t size);

/**
 * @brief Update median filter with new sample
 * @param[in,out] pFilter Pointer to filter structure
 * @param[in]     sample  New input sample
 * @return Filtered output value (median of window)
 */
int32_t Filter_Median_Update(Filter_Median_t *pFilter, int32_t sample);

/**
 * @brief Reset median filter
 * @param[in,out] pFilter Pointer to filter structure
 * @return STATUS_OK on success
 */
Status_t Filter_Median_Reset(Filter_Median_t *pFilter);

/*============================================================================*/
/* FUNCTION PROTOTYPES - DEBOUNCE FILTER                                      */
/*============================================================================*/

/**
 * @brief Initialize debounce filter
 * @param[out] pFilter      Pointer to filter structure
 * @param[in]  threshold_ms Debounce time in milliseconds
 * @param[in]  sample_rate_hz Sampling rate in Hz
 * @return STATUS_OK on success, error code otherwise
 */
Status_t Filter_Debounce_Init(Filter_Debounce_t *pFilter, uint16_t threshold_ms,
                               uint16_t sample_rate_hz);

/**
 * @brief Update debounce filter with new sample
 * @param[in,out] pFilter    Pointer to filter structure
 * @param[in]     raw_sample Raw digital input (true/false)
 * @return Debounced output state (true/false)
 */
bool Filter_Debounce_Update(Filter_Debounce_t *pFilter, bool raw_sample);

/**
 * @brief Reset debounce filter
 * @param[in,out] pFilter Pointer to filter structure
 * @param[in]     initial_state Initial state to set
 * @return STATUS_OK on success
 */
Status_t Filter_Debounce_Reset(Filter_Debounce_t *pFilter, bool initial_state);

/**
 * @brief Get current debounced state
 * @param[in] pFilter Pointer to filter structure
 * @return Current stable state
 */
bool Filter_Debounce_GetState(const Filter_Debounce_t *pFilter);

/*============================================================================*/
/* UTILITY FUNCTIONS                                                          */
/*============================================================================*/

/**
 * @brief Calculate simple moving average (no state)
 * @param[in] pData Pointer to data array
 * @param[in] length Array length
 * @return Average value
 */
int32_t Filter_Util_Average(const int32_t *pData, uint8_t length);

/**
 * @brief Calculate median value (no state)
 * @param[in] pData Pointer to data array
 * @param[in] length Array length (must be odd)
 * @return Median value
 * @note  Input array is modified (sorted)
 */
int32_t Filter_Util_Median(int32_t *pData, uint8_t length);

/**
 * @brief Clamp value to range
 * @param[in] value Value to clamp
 * @param[in] min   Minimum value
 * @param[in] max   Maximum value
 * @return Clamped value
 */
int32_t Filter_Util_Clamp(int32_t value, int32_t min, int32_t max);

#ifdef __cplusplus
}
#endif

#endif /* FILTER_H */
