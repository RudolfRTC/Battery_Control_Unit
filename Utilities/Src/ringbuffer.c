/**
 * @file    ringbuffer.c
 * @brief   Ring buffer implementation
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
#include "ringbuffer.h"
#include <string.h>

/*============================================================================*/
/* PUBLIC FUNCTIONS                                                           */
/*============================================================================*/

/**
 * @brief Initialize ring buffer
 */
Status_t RingBuffer_Init(RingBuffer_t *pRB, uint8_t *pBuffer, uint32_t size)
{
    Status_t status = STATUS_ERROR_PARAM;

    if ((pRB != NULL) && (pBuffer != NULL) && (size > 0U))
    {
        pRB->pBuffer = pBuffer;
        pRB->size = size;
        pRB->head = 0U;
        pRB->tail = 0U;
        status = STATUS_OK;
    }

    return status;
}

/**
 * @brief Write data to ring buffer
 */
Status_t RingBuffer_Write(RingBuffer_t *pRB, const uint8_t *pData, uint32_t length)
{
    Status_t status = STATUS_ERROR_PARAM;

    if ((pRB != NULL) && (pData != NULL) && (length > 0U))
    {
        uint32_t free = RingBuffer_Free(pRB);

        if (length <= free)
        {
            uint32_t i;
            for (i = 0U; i < length; i++)
            {
                pRB->pBuffer[pRB->head] = pData[i];
                pRB->head = (pRB->head + 1U) % pRB->size;
            }
            status = STATUS_OK;
        }
        else
        {
            status = STATUS_ERROR_OVERFLOW;
        }
    }

    return status;
}

/**
 * @brief Read data from ring buffer
 */
Status_t RingBuffer_Read(RingBuffer_t *pRB, uint8_t *pData, uint32_t length)
{
    Status_t status = STATUS_ERROR_PARAM;

    if ((pRB != NULL) && (pData != NULL) && (length > 0U))
    {
        uint32_t available = RingBuffer_Available(pRB);

        if (length <= available)
        {
            uint32_t i;
            for (i = 0U; i < length; i++)
            {
                pData[i] = pRB->pBuffer[pRB->tail];
                pRB->tail = (pRB->tail + 1U) % pRB->size;
            }
            status = STATUS_OK;
        }
        else
        {
            status = STATUS_ERROR_UNDERFLOW;
        }
    }

    return status;
}

/**
 * @brief Peek data without removing
 */
Status_t RingBuffer_Peek(const RingBuffer_t *pRB, uint8_t *pData, uint32_t length)
{
    Status_t status = STATUS_ERROR_PARAM;

    if ((pRB != NULL) && (pData != NULL) && (length > 0U))
    {
        uint32_t available = RingBuffer_Available(pRB);

        if (length <= available)
        {
            uint32_t i;
            uint32_t idx = pRB->tail;
            for (i = 0U; i < length; i++)
            {
                pData[i] = pRB->pBuffer[idx];
                idx = (idx + 1U) % pRB->size;
            }
            status = STATUS_OK;
        }
        else
        {
            status = STATUS_ERROR_UNDERFLOW;
        }
    }

    return status;
}

/**
 * @brief Get available bytes
 */
uint32_t RingBuffer_Available(const RingBuffer_t *pRB)
{
    uint32_t available = 0U;

    if (pRB != NULL)
    {
        if (pRB->head >= pRB->tail)
        {
            available = pRB->head - pRB->tail;
        }
        else
        {
            available = pRB->size - pRB->tail + pRB->head;
        }
    }

    return available;
}

/**
 * @brief Get free space
 */
uint32_t RingBuffer_Free(const RingBuffer_t *pRB)
{
    uint32_t free = 0U;

    if (pRB != NULL)
    {
        free = pRB->size - RingBuffer_Available(pRB) - 1U;
    }

    return free;
}

/**
 * @brief Check if empty
 */
bool RingBuffer_IsEmpty(const RingBuffer_t *pRB)
{
    bool empty = true;

    if (pRB != NULL)
    {
        empty = (pRB->head == pRB->tail);
    }

    return empty;
}

/**
 * @brief Check if full
 */
bool RingBuffer_IsFull(const RingBuffer_t *pRB)
{
    bool full = false;

    if (pRB != NULL)
    {
        full = (((pRB->head + 1U) % pRB->size) == pRB->tail);
    }

    return full;
}

/**
 * @brief Clear buffer
 */
void RingBuffer_Clear(RingBuffer_t *pRB)
{
    if (pRB != NULL)
    {
        pRB->head = 0U;
        pRB->tail = 0U;
    }
}

/*============================================================================*/
/* END OF FILE                                                                */
/*============================================================================*/
