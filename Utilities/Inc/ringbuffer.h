/**
 * @file    ringbuffer.h
 * @brief   Thread-safe ring buffer (circular buffer) implementation
 * @author  Battery Control Unit Development Team
 * @date    2026-01-09
 * @version 1.0.0
 *
 * @note    MISRA C:2012 compliant
 * @note    Lock-free implementation for single producer/consumer
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

/*============================================================================*/
/* TYPE DEFINITIONS                                                           */
/*============================================================================*/

/**
 * @brief Ring buffer structure
 */
typedef struct {
    uint8_t  *pBuffer;      /**< Pointer to buffer memory */
    uint32_t size;          /**< Buffer size in bytes */
    uint32_t head;          /**< Write index (producer) */
    uint32_t tail;          /**< Read index (consumer) */
    uint32_t count;         /**< Number of bytes in buffer */
    bool     initialized;   /**< Initialization flag */
} RingBuffer_t;

/*============================================================================*/
/* FUNCTION PROTOTYPES                                                        */
/*============================================================================*/

/**
 * @brief Initialize ring buffer
 * @param[out] pRingBuf Pointer to ring buffer structure
 * @param[in]  pBuffer  Pointer to buffer memory
 * @param[in]  size     Buffer size in bytes
 * @return STATUS_OK on success, error code otherwise
 * @note  Buffer memory must be provided by caller
 */
Status_t RingBuffer_Init(RingBuffer_t *pRingBuf, uint8_t *pBuffer, uint32_t size);

/**
 * @brief Reset ring buffer (clear all data)
 * @param[in,out] pRingBuf Pointer to ring buffer structure
 * @return STATUS_OK on success
 */
Status_t RingBuffer_Reset(RingBuffer_t *pRingBuf);

/**
 * @brief Write single byte to ring buffer
 * @param[in,out] pRingBuf Pointer to ring buffer structure
 * @param[in]     data     Byte to write
 * @return STATUS_OK on success, STATUS_ERROR_OVERFLOW if buffer full
 */
Status_t RingBuffer_WriteByte(RingBuffer_t *pRingBuf, uint8_t data);

/**
 * @brief Write multiple bytes to ring buffer
 * @param[in,out] pRingBuf   Pointer to ring buffer structure
 * @param[in]     pData      Pointer to source data
 * @param[in]     length     Number of bytes to write
 * @param[out]    pWritten   Number of bytes actually written (can be NULL)
 * @return STATUS_OK if all bytes written, STATUS_ERROR_OVERFLOW if partial write
 */
Status_t RingBuffer_Write(RingBuffer_t *pRingBuf, const uint8_t *pData,
                          uint32_t length, uint32_t *pWritten);

/**
 * @brief Read single byte from ring buffer
 * @param[in,out] pRingBuf Pointer to ring buffer structure
 * @param[out]    pData    Pointer to store read byte
 * @return STATUS_OK on success, STATUS_ERROR_UNDERFLOW if buffer empty
 */
Status_t RingBuffer_ReadByte(RingBuffer_t *pRingBuf, uint8_t *pData);

/**
 * @brief Read multiple bytes from ring buffer
 * @param[in,out] pRingBuf Pointer to ring buffer structure
 * @param[out]    pData    Pointer to destination buffer
 * @param[in]     length   Maximum number of bytes to read
 * @param[out]    pRead    Number of bytes actually read (can be NULL)
 * @return STATUS_OK if requested bytes read, STATUS_ERROR_UNDERFLOW if less available
 */
Status_t RingBuffer_Read(RingBuffer_t *pRingBuf, uint8_t *pData,
                         uint32_t length, uint32_t *pRead);

/**
 * @brief Peek at data without removing from buffer
 * @param[in]  pRingBuf Pointer to ring buffer structure
 * @param[out] pData    Pointer to destination buffer
 * @param[in]  length   Number of bytes to peek
 * @param[out] pPeeked  Number of bytes actually peeked (can be NULL)
 * @return STATUS_OK on success
 * @note  Data remains in buffer after peek
 */
Status_t RingBuffer_Peek(const RingBuffer_t *pRingBuf, uint8_t *pData,
                         uint32_t length, uint32_t *pPeeked);

/**
 * @brief Get number of bytes available to read
 * @param[in] pRingBuf Pointer to ring buffer structure
 * @return Number of bytes available
 */
uint32_t RingBuffer_GetCount(const RingBuffer_t *pRingBuf);

/**
 * @brief Get free space available for writing
 * @param[in] pRingBuf Pointer to ring buffer structure
 * @return Number of bytes free
 */
uint32_t RingBuffer_GetFree(const RingBuffer_t *pRingBuf);

/**
 * @brief Check if ring buffer is empty
 * @param[in] pRingBuf Pointer to ring buffer structure
 * @return true if empty, false otherwise
 */
bool RingBuffer_IsEmpty(const RingBuffer_t *pRingBuf);

/**
 * @brief Check if ring buffer is full
 * @param[in] pRingBuf Pointer to ring buffer structure
 * @return true if full, false otherwise
 */
bool RingBuffer_IsFull(const RingBuffer_t *pRingBuf);

/**
 * @brief Discard bytes from ring buffer
 * @param[in,out] pRingBuf Pointer to ring buffer structure
 * @param[in]     count    Number of bytes to discard
 * @return Number of bytes actually discarded
 */
uint32_t RingBuffer_Discard(RingBuffer_t *pRingBuf, uint32_t count);

/**
 * @brief Find byte in ring buffer
 * @param[in]  pRingBuf Pointer to ring buffer structure
 * @param[in]  byte     Byte to search for
 * @param[out] pIndex   Index of byte if found (can be NULL)
 * @return true if byte found, false otherwise
 */
bool RingBuffer_Find(const RingBuffer_t *pRingBuf, uint8_t byte, uint32_t *pIndex);

#ifdef __cplusplus
}
#endif

#endif /* RINGBUFFER_H */
