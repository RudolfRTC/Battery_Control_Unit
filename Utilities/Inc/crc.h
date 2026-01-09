/**
 * @file    crc.h
 * @brief   CRC calculation utilities (CRC-16, CRC-32)
 * @author  Battery Control Unit Development Team
 * @date    2026-01-09
 * @version 1.0.0
 *
 * @note    MISRA C:2012 compliant
 * @note    Hardware CRC peripheral used when available
 *
 * @copyright Copyright (c) 2026
 */

#ifndef CRC_H
#define CRC_H

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

/** @brief CRC-16 polynomial (CCITT) */
#define CRC16_POLY_CCITT    (0x1021U)

/** @brief CRC-32 polynomial (IEEE 802.3) */
#define CRC32_POLY_IEEE     (0x04C11DB7UL)

/** @brief Initial CRC values */
#define CRC16_INIT_VALUE    (0xFFFFU)
#define CRC32_INIT_VALUE    (0xFFFFFFFFUL)

/*============================================================================*/
/* FUNCTION PROTOTYPES                                                        */
/*============================================================================*/

/**
 * @brief Initialize CRC module (configure hardware if available)
 * @return STATUS_OK on success, error code otherwise
 */
Status_t CRC_Init(void);

/**
 * @brief Calculate CRC-16 (CCITT)
 * @param[in] pData Pointer to data buffer
 * @param[in] length Data length in bytes
 * @return Calculated CRC-16 value
 * @note  NULL pointer check performed
 */
uint16_t CRC_Calculate16(const uint8_t *pData, uint32_t length);

/**
 * @brief Calculate CRC-32 (IEEE 802.3)
 * @param[in] pData Pointer to data buffer
 * @param[in] length Data length in bytes
 * @return Calculated CRC-32 value
 * @note  Uses hardware CRC peripheral if available
 */
uint32_t CRC_Calculate32(const uint8_t *pData, uint32_t length);

/**
 * @brief Calculate CRC-32 using hardware peripheral
 * @param[in] pData Pointer to data buffer
 * @param[in] length Data length in bytes
 * @return Calculated CRC-32 value
 * @note  Faster than software implementation
 */
uint32_t CRC_Calculate32_Hardware(const uint8_t *pData, uint32_t length);

/**
 * @brief Verify CRC-16 checksum
 * @param[in] pData Pointer to data buffer (includes CRC at end)
 * @param[in] length Total length including CRC bytes
 * @return true if CRC is valid, false otherwise
 */
bool CRC_Verify16(const uint8_t *pData, uint32_t length);

/**
 * @brief Verify CRC-32 checksum
 * @param[in] pData Pointer to data buffer (includes CRC at end)
 * @param[in] length Total length including CRC bytes
 * @return true if CRC is valid, false otherwise
 */
bool CRC_Verify32(const uint8_t *pData, uint32_t length);

/**
 * @brief Append CRC-16 to data buffer
 * @param[in,out] pData Pointer to data buffer (must have space for CRC)
 * @param[in] length Data length (excluding CRC)
 * @return STATUS_OK on success
 */
Status_t CRC_Append16(uint8_t *pData, uint32_t length);

/**
 * @brief Append CRC-32 to data buffer
 * @param[in,out] pData Pointer to data buffer (must have space for CRC)
 * @param[in] length Data length (excluding CRC)
 * @return STATUS_OK on success
 */
Status_t CRC_Append32(uint8_t *pData, uint32_t length);

#ifdef __cplusplus
}
#endif

#endif /* CRC_H */
