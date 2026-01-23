/**
 * @file    ringbuffer.h
 * @brief   Ring buffer (circular buffer) utilities
 * @author  Battery Control Unit Development Team
 * @date    2026-01-09
 * @version 1.0.0
 *
 * @note    MISRA C:2012 compliant
 * @note    Thread-safe for single producer / single consumer
 *
 * @copyright Copyright (c) 2026
 */

#ifndef RINGBUFFER_H
#define RINGBUFFER_H

#ifdef __cplusplus
extern "C" {
#endif

/*============================================================================*/
/* INCLUDES                                                                   */
/*============================================================================*/
#include "app_types.h"
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

/*============================================================================*/
/* TYPE DEFINITIONS                                                           */
/*============================================================================*/

/**
 * @brief Ring buffer structure
 */
typedef struct {
    uint8_t *pBuffer;      /**< Pointer to buffer memory */
    uint32_t size;         /**< Total buffer size */
    volatile uint32_t head; /**< Write index */
    volatile uint32_t tail; /**< Read index */
} RingBuffer_t;

/*============================================================================*/
/* PUBLIC FUNCTION PROTOTYPES                                                 */
/*============================================================================*/

/**
 * @brief Initialize ring buffer
 * @param[out] pRB Pointer to ring buffer structure
 * @param[in] pBuffer Pointer to buffer memory
 * @param[in] size Buffer size in bytes
 * @return Status code
 */
Status_t RingBuffer_Init(RingBuffer_t *pRB, uint8_t *pBuffer, uint32_t size);

/**
 * @brief Write data to ring buffer
 * @param[in,out] pRB Pointer to ring buffer
 * @param[in] pData Data to write
 * @param[in] length Data length
 * @return Status code (STATUS_OK or STATUS_ERROR_FULL)
 */
Status_t RingBuffer_Write(RingBuffer_t *pRB, const uint8_t *pData, uint32_t length);

/**
 * @brief Read data from ring buffer
 * @param[in,out] pRB Pointer to ring buffer
 * @param[out] pData Buffer to read into
 * @param[in] length Number of bytes to read
 * @return Status code (STATUS_OK or STATUS_ERROR_EMPTY)
 */
Status_t RingBuffer_Read(RingBuffer_t *pRB, uint8_t *pData, uint32_t length);

/**
 * @brief Peek data without removing from buffer
 * @param[in] pRB Pointer to ring buffer
 * @param[out] pData Buffer to read into
 * @param[in] length Number of bytes to peek
 * @return Status code
 */
Status_t RingBuffer_Peek(const RingBuffer_t *pRB, uint8_t *pData, uint32_t length);

/**
 * @brief Get number of bytes available to read
 * @param[in] pRB Pointer to ring buffer
 * @return Number of bytes available
 */
uint32_t RingBuffer_Available(const RingBuffer_t *pRB);

/**
 * @brief Get free space in buffer
 * @param[in] pRB Pointer to ring buffer
 * @return Number of free bytes
 */
uint32_t RingBuffer_Free(const RingBuffer_t *pRB);

/**
 * @brief Check if buffer is empty
 * @param[in] pRB Pointer to ring buffer
 * @return true if empty
 */
bool RingBuffer_IsEmpty(const RingBuffer_t *pRB);

/**
 * @brief Check if buffer is full
 * @param[in] pRB Pointer to ring buffer
 * @return true if full
 */
bool RingBuffer_IsFull(const RingBuffer_t *pRB);

/**
 * @brief Clear/reset buffer
 * @param[out] pRB Pointer to ring buffer
 */
void RingBuffer_Clear(RingBuffer_t *pRB);

#ifdef __cplusplus
}
#endif

#endif /* RINGBUFFER_H */

/*============================================================================*/
/* END OF FILE                                                                */
/*============================================================================*/
