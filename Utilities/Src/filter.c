/**
 * @file    filter.c
 * @brief   Digital filter utilities implementation
 * @author  Battery Control Unit Development Team
 * @date    2026-01-09
 * @version 1.0.0
 *
 * @note    MISRA C:2012 compliant
 *
 * @copyright Copyright (c) 2026
 */

/*============================================================================*/
/* INCLUDES                                                                   */
/*============================================================================*/
#include "filter.h"
#include <string.h>

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
        (void)memset(pFilter, 0, sizeof(Filter_MovingAverage_t));
        pFilter->size = size;
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

    if (pFilter != NULL)
    {
        /* Remove oldest sample from sum if buffer is full */
        if (pFilter->count >= pFilter->size)
        {
            pFilter->sum -= pFilter->buffer[pFilter->index];
        }
        else
        {
            pFilter->count++;
        }

        /* Add new sample */
        pFilter->buffer[pFilter->index] = sample;
        pFilter->sum += sample;

        /* Update index */
        pFilter->index++;
        if (pFilter->index >= pFilter->size)
        {
            pFilter->index = 0U;
        }

        /* Calculate average */
        output = pFilter->sum / (int32_t)pFilter->count;
    }

    return output;
}

/**
 * @brief Reset moving average filter
 */
void Filter_MA_Reset(Filter_MovingAverage_t *pFilter)
{
    if (pFilter != NULL)
    {
        uint8_t size = pFilter->size;
        (void)memset(pFilter, 0, sizeof(Filter_MovingAverage_t));
        pFilter->size = size;
    }
}

/*============================================================================*/
/* PUBLIC FUNCTIONS - IIR FILTER                                              */
/*============================================================================*/

/**
 * @brief Initialize IIR filter
 */
Status_t Filter_IIR_Init(Filter_IIR_t *pFilter, uint8_t alpha, uint8_t scale)
{
    Status_t status = STATUS_ERROR_PARAM;

    if ((pFilter != NULL) && (alpha <= scale) && (scale > 0U))
    {
        pFilter->output = 0;
        pFilter->alpha = alpha;
        pFilter->alphaComplement = scale - alpha;
        pFilter->initialized = false;
        status = STATUS_OK;
    }

    return status;
}

/**
 * @brief Update IIR filter
 * Output = (alpha * sample + (scale - alpha) * output) / scale
 */
int32_t Filter_IIR_Update(Filter_IIR_t *pFilter, int32_t sample)
{
    int32_t output = sample;

    if (pFilter != NULL)
    {
        if (!pFilter->initialized)
        {
            /* First sample - initialize output */
            pFilter->output = sample;
            pFilter->initialized = true;
        }
        else
        {
            /* IIR filter calculation */
            int32_t scale = (int32_t)pFilter->alpha + (int32_t)pFilter->alphaComplement;
            pFilter->output = ((int32_t)pFilter->alpha * sample +
                              (int32_t)pFilter->alphaComplement * pFilter->output) / scale;
        }
        output = pFilter->output;
    }

    return output;
}

/**
 * @brief Reset IIR filter
 */
void Filter_IIR_Reset(Filter_IIR_t *pFilter)
{
    if (pFilter != NULL)
    {
        pFilter->output = 0;
        pFilter->initialized = false;
    }
}

/*============================================================================*/
/* PUBLIC FUNCTIONS - DEBOUNCE FILTER                                         */
/*============================================================================*/

/**
 * @brief Initialize debounce filter
 */
Status_t Filter_Debounce_Init(Filter_Debounce_t *pFilter, uint32_t threshold_ms, uint32_t samplePeriod_ms)
{
    Status_t status = STATUS_ERROR_PARAM;

    if ((pFilter != NULL) && (samplePeriod_ms > 0U))
    {
        pFilter->state = false;
        pFilter->lastRaw = false;
        pFilter->counter = 0U;
        pFilter->threshold = threshold_ms;
        pFilter->samplePeriod = samplePeriod_ms;
        status = STATUS_OK;
    }

    return status;
}

/**
 * @brief Update debounce filter
 */
bool Filter_Debounce_Update(Filter_Debounce_t *pFilter, bool rawState)
{
    bool output = false;

    if (pFilter != NULL)
    {
        if (rawState == pFilter->lastRaw)
        {
            /* Same state - increment counter */
            pFilter->counter += pFilter->samplePeriod;

            if (pFilter->counter >= pFilter->threshold)
            {
                /* Debounce complete - update state */
                pFilter->state = rawState;
                pFilter->counter = pFilter->threshold; /* Prevent overflow */
            }
        }
        else
        {
            /* State changed - reset counter */
            pFilter->counter = 0U;
            pFilter->lastRaw = rawState;
        }

        output = pFilter->state;
    }

    return output;
}

/**
 * @brief Reset debounce filter
 */
void Filter_Debounce_Reset(Filter_Debounce_t *pFilter)
{
    if (pFilter != NULL)
    {
        pFilter->state = false;
        pFilter->lastRaw = false;
        pFilter->counter = 0U;
    }
}

/*============================================================================*/
/* END OF FILE                                                                */
/*============================================================================*/
