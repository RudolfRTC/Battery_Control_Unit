/**
 * @file    filter.c
 * @brief   Digital signal filtering utilities implementation
 * @author  Battery Control Unit Development Team
 * @date    2026-01-09
 * @version 1.0.0
 *
 * @note    MISRA C:2012 compliant
 * @note    Fixed-point arithmetic for deterministic performance
 *
 * @copyright Copyright (c) 2026
 */

/*============================================================================*/
/* INCLUDES                                                                   */
/*============================================================================*/
#include "filter.h"
#include <string.h>

/*============================================================================*/
/* PRIVATE FUNCTION PROTOTYPES                                                */
/*============================================================================*/

static void filter_insertion_sort(int32_t *pArray, uint8_t length);

/*============================================================================*/
/* PUBLIC FUNCTIONS - MOVING AVERAGE                                          */
/*============================================================================*/

/**
 * @brief Initialize moving average filter
 */
Status_t Filter_MA_Init(Filter_MovingAverage_t *pFilter, uint8_t size)
{
    Status_t status = STATUS_ERROR_PARAM;

    if ((pFilter != NULL) && (size > 0U) && (size <= FILTER_MA_MAX_SIZE))
    {
        uint8_t i;

        /* Initialize buffer to zero */
        for (i = 0U; i < FILTER_MA_MAX_SIZE; i++)
        {
            pFilter->buffer[i] = 0;
        }

        pFilter->sum = 0;
        pFilter->size = size;
        pFilter->index = 0U;
        pFilter->count = 0U;
        pFilter->initialized = true;

        status = STATUS_OK;
    }

    return status;
}

/**
 * @brief Update moving average filter
 */
int32_t Filter_MA_Update(Filter_MovingAverage_t *pFilter, int32_t sample)
{
    int32_t output = sample;

    if ((pFilter != NULL) && pFilter->initialized)
    {
        int32_t old_value;

        /* Get old value at current index */
        old_value = pFilter->buffer[pFilter->index];

        /* Store new sample */
        pFilter->buffer[pFilter->index] = sample;

        /* Update sum */
        pFilter->sum = pFilter->sum - old_value + sample;

        /* Increment index with wraparound */
        pFilter->index = (pFilter->index + 1U) % pFilter->size;

        /* Update count (saturate at size) */
        if (pFilter->count < pFilter->size)
        {
            pFilter->count++;
        }

        /* Calculate average */
        if (pFilter->count > 0U)
        {
            output = (int32_t)(pFilter->sum / (int64_t)pFilter->count);
        }
    }

    return output;
}

/**
 * @brief Reset moving average filter
 */
Status_t Filter_MA_Reset(Filter_MovingAverage_t *pFilter)
{
    Status_t status = STATUS_ERROR_PARAM;

    if ((pFilter != NULL) && pFilter->initialized)
    {
        uint8_t i;

        for (i = 0U; i < pFilter->size; i++)
        {
            pFilter->buffer[i] = 0;
        }

        pFilter->sum = 0;
        pFilter->index = 0U;
        pFilter->count = 0U;

        status = STATUS_OK;
    }

    return status;
}

/**
 * @brief Get current moving average output
 */
int32_t Filter_MA_GetOutput(const Filter_MovingAverage_t *pFilter)
{
    int32_t output = 0;

    if ((pFilter != NULL) && pFilter->initialized && (pFilter->count > 0U))
    {
        output = (int32_t)(pFilter->sum / (int64_t)pFilter->count);
    }

    return output;
}

/*============================================================================*/
/* PUBLIC FUNCTIONS - IIR LOW-PASS FILTER                                     */
/*============================================================================*/

/**
 * @brief Initialize IIR low-pass filter
 */
Status_t Filter_IIR_Init(Filter_IIR_t *pFilter, uint16_t cutoff_hz,
                         uint16_t sample_rate_hz)
{
    Status_t status = STATUS_ERROR_PARAM;

    if ((pFilter != NULL) && (cutoff_hz > 0U) && (sample_rate_hz > 0U))
    {
        uint16_t alpha;
        float rc;
        float dt;
        float alpha_float;

        /* Calculate RC time constant and sample period */
        rc = 1.0f / (2.0f * 3.14159f * (float)cutoff_hz);
        dt = 1.0f / (float)sample_rate_hz;

        /* Calculate alpha: alpha = dt / (rc + dt) */
        alpha_float = dt / (rc + dt);

        /* Scale to 16-bit fixed-point (0-65535) */
        alpha = (uint16_t)(alpha_float * 65536.0f);

        /* Initialize with calculated alpha */
        status = Filter_IIR_InitWithAlpha(pFilter, alpha);
    }

    return status;
}

/**
 * @brief Initialize IIR filter with explicit alpha
 */
Status_t Filter_IIR_InitWithAlpha(Filter_IIR_t *pFilter, uint16_t alpha)
{
    Status_t status = STATUS_ERROR_PARAM;

    if (pFilter != NULL)
    {
        pFilter->output = 0;
        pFilter->alpha = alpha;
        pFilter->initialized = true;

        status = STATUS_OK;
    }

    return status;
}

/**
 * @brief Update IIR filter
 */
int32_t Filter_IIR_Update(Filter_IIR_t *pFilter, int32_t sample)
{
    int32_t output = sample;

    if ((pFilter != NULL) && pFilter->initialized)
    {
        int64_t temp;

        /* y[n] = alpha * x[n] + (1-alpha) * y[n-1] */
        /* Using fixed-point: alpha scaled by 65536 */
        temp = ((int64_t)pFilter->alpha * (int64_t)sample) +
               (((int64_t)65536 - (int64_t)pFilter->alpha) * (int64_t)pFilter->output);

        /* Scale back down */
        output = (int32_t)(temp >> 16);

        /* Store output for next iteration */
        pFilter->output = output;
    }

    return output;
}

/**
 * @brief Reset IIR filter
 */
Status_t Filter_IIR_Reset(Filter_IIR_t *pFilter)
{
    Status_t status = STATUS_ERROR_PARAM;

    if ((pFilter != NULL) && pFilter->initialized)
    {
        pFilter->output = 0;
        status = STATUS_OK;
    }

    return status;
}

/*============================================================================*/
/* PUBLIC FUNCTIONS - MEDIAN FILTER                                           */
/*============================================================================*/

/**
 * @brief Initialize median filter
 */
Status_t Filter_Median_Init(Filter_Median_t *pFilter, uint8_t size)
{
    Status_t status = STATUS_ERROR_PARAM;

    /* Size must be odd and within limits */
    if ((pFilter != NULL) && (size > 0U) && (size <= FILTER_MEDIAN_MAX_SIZE) &&
        ((size % 2U) == 1U))
    {
        uint8_t i;

        /* Initialize buffers to zero */
        for (i = 0U; i < FILTER_MEDIAN_MAX_SIZE; i++)
        {
            pFilter->buffer[i] = 0;
            pFilter->sorted[i] = 0;
        }

        pFilter->size = size;
        pFilter->index = 0U;
        pFilter->count = 0U;
        pFilter->initialized = true;

        status = STATUS_OK;
    }

    return status;
}

/**
 * @brief Update median filter
 */
int32_t Filter_Median_Update(Filter_Median_t *pFilter, int32_t sample)
{
    int32_t output = sample;

    if ((pFilter != NULL) && pFilter->initialized)
    {
        uint8_t i;
        uint8_t median_index;

        /* Store new sample */
        pFilter->buffer[pFilter->index] = sample;

        /* Increment index with wraparound */
        pFilter->index = (pFilter->index + 1U) % pFilter->size;

        /* Update count (saturate at size) */
        if (pFilter->count < pFilter->size)
        {
            pFilter->count++;
        }

        /* Copy buffer to sorted array */
        for (i = 0U; i < pFilter->count; i++)
        {
            pFilter->sorted[i] = pFilter->buffer[i];
        }

        /* Sort the array */
        filter_insertion_sort(pFilter->sorted, pFilter->count);

        /* Get median (middle element) */
        median_index = pFilter->count / 2U;
        output = pFilter->sorted[median_index];
    }

    return output;
}

/**
 * @brief Reset median filter
 */
Status_t Filter_Median_Reset(Filter_Median_t *pFilter)
{
    Status_t status = STATUS_ERROR_PARAM;

    if ((pFilter != NULL) && pFilter->initialized)
    {
        uint8_t i;

        for (i = 0U; i < pFilter->size; i++)
        {
            pFilter->buffer[i] = 0;
            pFilter->sorted[i] = 0;
        }

        pFilter->index = 0U;
        pFilter->count = 0U;

        status = STATUS_OK;
    }

    return status;
}

/*============================================================================*/
/* PUBLIC FUNCTIONS - DEBOUNCE FILTER                                         */
/*============================================================================*/

/**
 * @brief Initialize debounce filter
 */
Status_t Filter_Debounce_Init(Filter_Debounce_t *pFilter, uint16_t threshold_ms,
                               uint16_t sample_rate_hz)
{
    Status_t status = STATUS_ERROR_PARAM;

    if ((pFilter != NULL) && (sample_rate_hz > 0U))
    {
        /* Calculate threshold in number of samples */
        uint32_t threshold_samples = ((uint32_t)threshold_ms * (uint32_t)sample_rate_hz) / 1000U;

        /* Limit to uint16_t range */
        if (threshold_samples > 65535U)
        {
            threshold_samples = 65535U;
        }

        pFilter->state = false;
        pFilter->raw_state = false;
        pFilter->counter = 0U;
        pFilter->threshold = (uint16_t)threshold_samples;
        pFilter->initialized = true;

        status = STATUS_OK;
    }

    return status;
}

/**
 * @brief Update debounce filter
 */
bool Filter_Debounce_Update(Filter_Debounce_t *pFilter, bool raw_sample)
{
    bool output = false;

    if ((pFilter != NULL) && pFilter->initialized)
    {
        pFilter->raw_state = raw_sample;

        if (raw_sample == pFilter->state)
        {
            /* Same state - reset counter */
            pFilter->counter = 0U;
        }
        else
        {
            /* Different state - increment counter */
            pFilter->counter++;

            /* Check if threshold reached */
            if (pFilter->counter >= pFilter->threshold)
            {
                /* State transition confirmed */
                pFilter->state = raw_sample;
                pFilter->counter = 0U;
            }
        }

        output = pFilter->state;
    }

    return output;
}

/**
 * @brief Reset debounce filter
 */
Status_t Filter_Debounce_Reset(Filter_Debounce_t *pFilter, bool initial_state)
{
    Status_t status = STATUS_ERROR_PARAM;

    if ((pFilter != NULL) && pFilter->initialized)
    {
        pFilter->state = initial_state;
        pFilter->raw_state = initial_state;
        pFilter->counter = 0U;

        status = STATUS_OK;
    }

    return status;
}

/**
 * @brief Get current debounced state
 */
bool Filter_Debounce_GetState(const Filter_Debounce_t *pFilter)
{
    bool state = false;

    if ((pFilter != NULL) && pFilter->initialized)
    {
        state = pFilter->state;
    }

    return state;
}

/*============================================================================*/
/* UTILITY FUNCTIONS                                                          */
/*============================================================================*/

/**
 * @brief Calculate simple moving average
 */
int32_t Filter_Util_Average(const int32_t *pData, uint8_t length)
{
    int32_t average = 0;

    if ((pData != NULL) && (length > 0U))
    {
        int64_t sum = 0;
        uint8_t i;

        for (i = 0U; i < length; i++)
        {
            sum += (int64_t)pData[i];
        }

        average = (int32_t)(sum / (int64_t)length);
    }

    return average;
}

/**
 * @brief Calculate median value
 */
int32_t Filter_Util_Median(int32_t *pData, uint8_t length)
{
    int32_t median = 0;

    if ((pData != NULL) && (length > 0U))
    {
        uint8_t median_index;

        /* Sort array */
        filter_insertion_sort(pData, length);

        /* Get median (middle element) */
        median_index = length / 2U;
        median = pData[median_index];
    }

    return median;
}

/**
 * @brief Clamp value to range
 */
int32_t Filter_Util_Clamp(int32_t value, int32_t min, int32_t max)
{
    int32_t clamped = value;

    if (clamped < min)
    {
        clamped = min;
    }
    else if (clamped > max)
    {
        clamped = max;
    }
    else
    {
        /* Value within range - no change needed */
    }

    return clamped;
}

/*============================================================================*/
/* PRIVATE FUNCTIONS                                                          */
/*============================================================================*/

/**
 * @brief Insertion sort for small arrays
 * @param[in,out] pArray Pointer to array to sort
 * @param[in]     length Array length
 */
static void filter_insertion_sort(int32_t *pArray, uint8_t length)
{
    uint8_t i;
    uint8_t j;
    int32_t key;

    for (i = 1U; i < length; i++)
    {
        key = pArray[i];
        j = i;

        while ((j > 0U) && (pArray[j - 1U] > key))
        {
            pArray[j] = pArray[j - 1U];
            j--;
        }

        pArray[j] = key;
    }
}

/*============================================================================*/
/* END OF FILE                                                                */
/*============================================================================*/
