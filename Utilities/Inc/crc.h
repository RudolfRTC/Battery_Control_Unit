/**
 * @file    crc.h
 * @brief   CRC calculation utilities
 * @author  Battery Control Unit Development Team
 * @date    2026-01-09
 * @version 1.0.0
 *
 * @note    MISRA C:2012 compliant
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
#include <stdint.h>
#include <stddef.h>

/*============================================================================*/
/* PUBLIC FUNCTION PROTOTYPES                                                 */
/*============================================================================*/

/**
 * @brief Calculate CRC-32 checksum
 * @param[in] pData Pointer to data buffer
 * @param[in] length Data length in bytes
 * @return CRC-32 checksum
 */
uint32_t CRC_Calculate32(const void *pData, uint32_t length);

/**
 * @brief Calculate CRC-32 checksum incrementally
 * @param[in] crc Initial/previous CRC value
 * @param[in] pData Pointer to data buffer
 * @param[in] length Data length in bytes
 * @return Updated CRC-32 checksum
 */
uint32_t CRC_Calculate32_Incremental(uint32_t crc, const void *pData, uint32_t length);

/**
 * @brief Calculate CRC-16 checksum
 * @param[in] pData Pointer to data buffer
 * @param[in] length Data length in bytes
 * @return CRC-16 checksum
 */
uint16_t CRC_Calculate16(const void *pData, uint32_t length);

/**
 * @brief Calculate CRC-8 checksum
 * @param[in] pData Pointer to data buffer
 * @param[in] length Data length in bytes
 * @return CRC-8 checksum
 */
uint8_t CRC_Calculate8(const void *pData, uint32_t length);

#ifdef __cplusplus
}
#endif

#endif /* CRC_H */

/*============================================================================*/
/* END OF FILE                                                                */
/*============================================================================*/
