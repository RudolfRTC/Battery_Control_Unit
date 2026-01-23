/**
 * @file    crc.c
 * @brief   CRC calculation utilities implementation
 * @author  Battery Control Unit Development Team
 * @date    2026-01-09
 * @version 1.0.0
 *
 * @note    MISRA C:2012 compliant
 * @note    Uses standard CRC polynomials
 *
 * @copyright Copyright (c) 2026
 */

/*============================================================================*/
/* INCLUDES                                                                   */
/*============================================================================*/
#include "crc.h"

/*============================================================================*/
/* PRIVATE CONSTANTS                                                          */
/*============================================================================*/

/** @brief CRC-32 polynomial (IEEE 802.3) */
#define CRC32_POLYNOMIAL    (0xEDB88320UL)

/** @brief CRC-32 initial value */
#define CRC32_INIT          (0xFFFFFFFFUL)

/** @brief CRC-16 polynomial (CCITT) */
#define CRC16_POLYNOMIAL    (0x8408U)

/** @brief CRC-16 initial value */
#define CRC16_INIT          (0xFFFFU)

/** @brief CRC-8 polynomial */
#define CRC8_POLYNOMIAL     (0x07U)

/*============================================================================*/
/* PUBLIC FUNCTIONS                                                           */
/*============================================================================*/

/**
 * @brief Calculate CRC-32 checksum
 */
uint32_t CRC_Calculate32(const void *pData, uint32_t length)
{
    return CRC_Calculate32_Incremental(CRC32_INIT, pData, length) ^ CRC32_INIT;
}

/**
 * @brief Calculate CRC-32 incrementally
 */
uint32_t CRC_Calculate32_Incremental(uint32_t crc, const void *pData, uint32_t length)
{
    const uint8_t *pBytes = (const uint8_t *)pData;
    uint32_t i;
    uint8_t j;

    if (pBytes != NULL)
    {
        for (i = 0U; i < length; i++)
        {
            crc ^= (uint32_t)pBytes[i];

            for (j = 0U; j < 8U; j++)
            {
                if ((crc & 1U) != 0U)
                {
                    crc = (crc >> 1U) ^ CRC32_POLYNOMIAL;
                }
                else
                {
                    crc = crc >> 1U;
                }
            }
        }
    }

    return crc;
}

/**
 * @brief Calculate CRC-16 checksum
 */
uint16_t CRC_Calculate16(const void *pData, uint32_t length)
{
    const uint8_t *pBytes = (const uint8_t *)pData;
    uint16_t crc = CRC16_INIT;
    uint32_t i;
    uint8_t j;

    if (pBytes != NULL)
    {
        for (i = 0U; i < length; i++)
        {
            crc ^= (uint16_t)pBytes[i];

            for (j = 0U; j < 8U; j++)
            {
                if ((crc & 1U) != 0U)
                {
                    crc = (crc >> 1U) ^ CRC16_POLYNOMIAL;
                }
                else
                {
                    crc = crc >> 1U;
                }
            }
        }
    }

    return crc ^ CRC16_INIT;
}

/**
 * @brief Calculate CRC-8 checksum
 */
uint8_t CRC_Calculate8(const void *pData, uint32_t length)
{
    const uint8_t *pBytes = (const uint8_t *)pData;
    uint8_t crc = 0U;
    uint32_t i;
    uint8_t j;

    if (pBytes != NULL)
    {
        for (i = 0U; i < length; i++)
        {
            crc ^= pBytes[i];

            for (j = 0U; j < 8U; j++)
            {
                if ((crc & 0x80U) != 0U)
                {
                    crc = (crc << 1U) ^ CRC8_POLYNOMIAL;
                }
                else
                {
                    crc = crc << 1U;
                }
            }
        }
    }

    return crc;
}

/*============================================================================*/
/* END OF FILE                                                                */
/*============================================================================*/
