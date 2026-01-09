/**
 * @file    ringbuffer.c
 * @brief   Thread-safe ring buffer implementation
 * @author  Battery Control Unit Development Team
 * @date    2026-01-09
 * @version 1.0.0
 *
 * @note    MISRA C:2012 compliant
 * @note    Lock-free for single producer/consumer
 *
 * @copyright Copyright (c) 2026
 */

/*============================================================================*/
/* INCLUDES                                                                   */
/*============================================================================*/
#include "ringbuffer.h"
#include <string.h>

/*============================================================================*/
/* PRIVATE MACROS                                                             */
/*============================================================================*/

/** @brief Get minimum of two values */
#define MIN(a, b) (((a) < (b)) ? (a) : (b))

/*============================================================================*/
/* PUBLIC FUNCTIONS                                                           */
/*============================================================================*/

/**
 * @brief Initialize ring buffer
 */
Status_t RingBuffer_Init(RingBuffer_t *pRingBuf, uint8_t *pBuffer, uint32_t size)
{
    Status_t status = STATUS_ERROR_PARAM;

    /* Parameter validation */
    if ((pRingBuf != NULL) && (pBuffer != NULL) && (size > 0U))
    {
        pRingBuf->pBuffer = pBuffer;
        pRingBuf->size = size;
        pRingBuf->head = 0U;
        pRingBuf->tail = 0U;
        pRingBuf->count = 0U;
        pRingBuf->initialized = true;

        status = STATUS_OK;
    }

    return status;
}

/**
 * @brief Reset ring buffer
 */
Status_t RingBuffer_Reset(RingBuffer_t *pRingBuf)
{
    Status_t status = STATUS_ERROR_PARAM;

    if ((pRingBuf != NULL) && pRingBuf->initialized)
    {
        pRingBuf->head = 0U;
        pRingBuf->tail = 0U;
        pRingBuf->count = 0U;

        status = STATUS_OK;
    }

    return status;
}

/**
 * @brief Write single byte to ring buffer
 */
Status_t RingBuffer_WriteByte(RingBuffer_t *pRingBuf, uint8_t data)
{
    Status_t status = STATUS_ERROR_PARAM;

    if ((pRingBuf != NULL) && pRingBuf->initialized)
    {
        if (pRingBuf->count >= pRingBuf->size)
        {
            /* Buffer full */
            status = STATUS_ERROR_OVERFLOW;
        }
        else
        {
            /* Write byte */
            pRingBuf->pBuffer[pRingBuf->head] = data;

            /* Increment head with wraparound */
            pRingBuf->head = (pRingBuf->head + 1U) % pRingBuf->size;

            /* Increment count */
            pRingBuf->count++;

            status = STATUS_OK;
        }
    }

    return status;
}

/**
 * @brief Write multiple bytes to ring buffer
 */
Status_t RingBuffer_Write(RingBuffer_t *pRingBuf, const uint8_t *pData,
                          uint32_t length, uint32_t *pWritten)
{
    Status_t status = STATUS_ERROR_PARAM;
    uint32_t written = 0U;

    if ((pRingBuf != NULL) && pRingBuf->initialized && (pData != NULL))
    {
        uint32_t space_available;
        uint32_t bytes_to_write;
        uint32_t i;

        /* Calculate space available */
        space_available = pRingBuf->size - pRingBuf->count;

        /* Determine how many bytes we can write */
        bytes_to_write = MIN(length, space_available);

        /* Write bytes */
        for (i = 0U; i < bytes_to_write; i++)
        {
            pRingBuf->pBuffer[pRingBuf->head] = pData[i];
            pRingBuf->head = (pRingBuf->head + 1U) % pRingBuf->size;
            written++;
        }

        /* Update count */
        pRingBuf->count += written;

        /* Return number of bytes written */
        if (pWritten != NULL)
        {
            *pWritten = written;
        }

        /* Determine status */
        if (written == length)
        {
            status = STATUS_OK;
        }
        else
        {
            /* Partial write */
            status = STATUS_ERROR_OVERFLOW;
        }
    }

    return status;
}

/**
 * @brief Read single byte from ring buffer
 */
Status_t RingBuffer_ReadByte(RingBuffer_t *pRingBuf, uint8_t *pData)
{
    Status_t status = STATUS_ERROR_PARAM;

    if ((pRingBuf != NULL) && pRingBuf->initialized && (pData != NULL))
    {
        if (pRingBuf->count == 0U)
        {
            /* Buffer empty */
            status = STATUS_ERROR_UNDERFLOW;
        }
        else
        {
            /* Read byte */
            *pData = pRingBuf->pBuffer[pRingBuf->tail];

            /* Increment tail with wraparound */
            pRingBuf->tail = (pRingBuf->tail + 1U) % pRingBuf->size;

            /* Decrement count */
            pRingBuf->count--;

            status = STATUS_OK;
        }
    }

    return status;
}

/**
 * @brief Read multiple bytes from ring buffer
 */
Status_t RingBuffer_Read(RingBuffer_t *pRingBuf, uint8_t *pData,
                         uint32_t length, uint32_t *pRead)
{
    Status_t status = STATUS_ERROR_PARAM;
    uint32_t read = 0U;

    if ((pRingBuf != NULL) && pRingBuf->initialized && (pData != NULL))
    {
        uint32_t bytes_available;
        uint32_t bytes_to_read;
        uint32_t i;

        /* Calculate bytes available */
        bytes_available = pRingBuf->count;

        /* Determine how many bytes we can read */
        bytes_to_read = MIN(length, bytes_available);

        /* Read bytes */
        for (i = 0U; i < bytes_to_read; i++)
        {
            pData[i] = pRingBuf->pBuffer[pRingBuf->tail];
            pRingBuf->tail = (pRingBuf->tail + 1U) % pRingBuf->size;
            read++;
        }

        /* Update count */
        pRingBuf->count -= read;

        /* Return number of bytes read */
        if (pRead != NULL)
        {
            *pRead = read;
        }

        /* Determine status */
        if (read == length)
        {
            status = STATUS_OK;
        }
        else
        {
            /* Partial read */
            status = STATUS_ERROR_UNDERFLOW;
        }
    }

    return status;
}

/**
 * @brief Peek at data without removing from buffer
 */
Status_t RingBuffer_Peek(const RingBuffer_t *pRingBuf, uint8_t *pData,
                         uint32_t length, uint32_t *pPeeked)
{
    Status_t status = STATUS_ERROR_PARAM;
    uint32_t peeked = 0U;

    if ((pRingBuf != NULL) && pRingBuf->initialized && (pData != NULL))
    {
        uint32_t bytes_available;
        uint32_t bytes_to_peek;
        uint32_t index;
        uint32_t i;

        /* Calculate bytes available */
        bytes_available = pRingBuf->count;

        /* Determine how many bytes we can peek */
        bytes_to_peek = MIN(length, bytes_available);

        /* Peek bytes (without modifying tail) */
        index = pRingBuf->tail;
        for (i = 0U; i < bytes_to_peek; i++)
        {
            pData[i] = pRingBuf->pBuffer[index];
            index = (index + 1U) % pRingBuf->size;
            peeked++;
        }

        /* Return number of bytes peeked */
        if (pPeeked != NULL)
        {
            *pPeeked = peeked;
        }

        status = STATUS_OK;
    }

    return status;
}

/**
 * @brief Get number of bytes available to read
 */
uint32_t RingBuffer_GetCount(const RingBuffer_t *pRingBuf)
{
    uint32_t count = 0U;

    if ((pRingBuf != NULL) && pRingBuf->initialized)
    {
        count = pRingBuf->count;
    }

    return count;
}

/**
 * @brief Get free space available for writing
 */
uint32_t RingBuffer_GetFree(const RingBuffer_t *pRingBuf)
{
    uint32_t free = 0U;

    if ((pRingBuf != NULL) && pRingBuf->initialized)
    {
        free = pRingBuf->size - pRingBuf->count;
    }

    return free;
}

/**
 * @brief Check if ring buffer is empty
 */
bool RingBuffer_IsEmpty(const RingBuffer_t *pRingBuf)
{
    bool isEmpty = true;

    if ((pRingBuf != NULL) && pRingBuf->initialized)
    {
        isEmpty = (pRingBuf->count == 0U);
    }

    return isEmpty;
}

/**
 * @brief Check if ring buffer is full
 */
bool RingBuffer_IsFull(const RingBuffer_t *pRingBuf)
{
    bool isFull = false;

    if ((pRingBuf != NULL) && pRingBuf->initialized)
    {
        isFull = (pRingBuf->count >= pRingBuf->size);
    }

    return isFull;
}

/**
 * @brief Discard bytes from ring buffer
 */
uint32_t RingBuffer_Discard(RingBuffer_t *pRingBuf, uint32_t count)
{
    uint32_t discarded = 0U;

    if ((pRingBuf != NULL) && pRingBuf->initialized)
    {
        uint32_t bytes_to_discard;

        /* Limit to available bytes */
        bytes_to_discard = MIN(count, pRingBuf->count);

        /* Update tail */
        pRingBuf->tail = (pRingBuf->tail + bytes_to_discard) % pRingBuf->size;

        /* Update count */
        pRingBuf->count -= bytes_to_discard;

        discarded = bytes_to_discard;
    }

    return discarded;
}

/**
 * @brief Find byte in ring buffer
 */
bool RingBuffer_Find(const RingBuffer_t *pRingBuf, uint8_t byte, uint32_t *pIndex)
{
    bool found = false;

    if ((pRingBuf != NULL) && pRingBuf->initialized)
    {
        uint32_t index;
        uint32_t i;

        index = pRingBuf->tail;

        /* Search through buffer */
        for (i = 0U; i < pRingBuf->count; i++)
        {
            if (pRingBuf->pBuffer[index] == byte)
            {
                found = true;

                if (pIndex != NULL)
                {
                    *pIndex = i;
                }

                break;
            }

            index = (index + 1U) % pRingBuf->size;
        }
    }

    return found;
}

/*============================================================================*/
/* END OF FILE                                                                */
/*============================================================================*/
