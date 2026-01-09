/**
 * @file    crc.c
 * @brief   CRC calculation utilities implementation
 * @author  Battery Control Unit Development Team
 * @date    2026-01-09
 * @version 1.0.0
 *
 * @note    MISRA C:2012 compliant
 * @note    Uses hardware CRC peripheral when available
 *
 * @copyright Copyright (c) 2026
 */

/*============================================================================*/
/* INCLUDES                                                                   */
/*============================================================================*/
#include "crc.h"
#include "stm32f4xx_hal.h"
#include <string.h>

/*============================================================================*/
/* PRIVATE CONSTANTS                                                          */
/*============================================================================*/

/** @brief CRC-16 lookup table (256 entries) */
static const uint16_t CRC16_TABLE[256] = {
    0x0000U, 0x1021U, 0x2042U, 0x3063U, 0x4084U, 0x50A5U, 0x60C6U, 0x70E7U,
    0x8108U, 0x9129U, 0xA14AU, 0xB16BU, 0xC18CU, 0xD1ADU, 0xE1CEU, 0xF1EFU,
    0x1231U, 0x0210U, 0x3273U, 0x2252U, 0x52B5U, 0x4294U, 0x72F7U, 0x62D6U,
    0x9339U, 0x8318U, 0xB37BU, 0xA35AU, 0xD3BDU, 0xC39CU, 0xF3FFU, 0xE3DEU,
    0x2462U, 0x3443U, 0x0420U, 0x1401U, 0x64E6U, 0x74C7U, 0x44A4U, 0x5485U,
    0xA56AU, 0xB54BU, 0x8528U, 0x9509U, 0xE5EEU, 0xF5CFU, 0xC5ACU, 0xD58DU,
    0x3653U, 0x2672U, 0x1611U, 0x0630U, 0x76D7U, 0x66F6U, 0x5695U, 0x46B4U,
    0xB75BU, 0xA77AU, 0x9719U, 0x8738U, 0xF7DFU, 0xE7FEU, 0xD79DU, 0xC7BCU,
    0x48C4U, 0x58E5U, 0x6886U, 0x78A7U, 0x0840U, 0x1861U, 0x2802U, 0x3823U,
    0xC9CCU, 0xD9EDU, 0xE98EU, 0xF9AFU, 0x8948U, 0x9969U, 0xA90AU, 0xB92BU,
    0x5AF5U, 0x4AD4U, 0x7AB7U, 0x6A96U, 0x1A71U, 0x0A50U, 0x3A33U, 0x2A12U,
    0xDBFDU, 0xCBDCU, 0xFBBFU, 0xEB9EU, 0x9B79U, 0x8B58U, 0xBB3BU, 0xAB1AU,
    0x6CA6U, 0x7C87U, 0x4CE4U, 0x5CC5U, 0x2C22U, 0x3C03U, 0x0C60U, 0x1C41U,
    0xEDAEU, 0xFD8FU, 0xCDECU, 0xDDCDU, 0xAD2AU, 0xBD0BU, 0x8D68U, 0x9D49U,
    0x7E97U, 0x6EB6U, 0x5ED5U, 0x4EF4U, 0x3E13U, 0x2E32U, 0x1E51U, 0x0E70U,
    0xFF9FU, 0xEFBEU, 0xDFDDU, 0xCFFCU, 0xBF1BU, 0xAF3AU, 0x9F59U, 0x8F78U,
    0x9188U, 0x81A9U, 0xB1CAU, 0xA1EBU, 0xD10CU, 0xC12DU, 0xF14EU, 0xE16FU,
    0x1080U, 0x00A1U, 0x30C2U, 0x20E3U, 0x5004U, 0x4025U, 0x7046U, 0x6067U,
    0x83B9U, 0x9398U, 0xA3FBU, 0xB3DAU, 0xC33DU, 0xD31CU, 0xE37FU, 0xF35EU,
    0x02B1U, 0x1290U, 0x22F3U, 0x32D2U, 0x4235U, 0x5214U, 0x6277U, 0x7256U,
    0xB5EAU, 0xA5CBU, 0x95A8U, 0x8589U, 0xF56EU, 0xE54FU, 0xD52CU, 0xC50DU,
    0x34E2U, 0x24C3U, 0x14A0U, 0x0481U, 0x7466U, 0x6447U, 0x5424U, 0x4405U,
    0xA7DBU, 0xB7FAU, 0x8799U, 0x97B8U, 0xE75FU, 0xF77EU, 0xC71DU, 0xD73CU,
    0x26D3U, 0x36F2U, 0x0691U, 0x16B0U, 0x6657U, 0x7676U, 0x4615U, 0x5634U,
    0xD94CU, 0xC96DU, 0xF90EU, 0xE92FU, 0x99C8U, 0x89E9U, 0xB98AU, 0xA9ABU,
    0x5844U, 0x4865U, 0x7806U, 0x6827U, 0x18C0U, 0x08E1U, 0x3882U, 0x28A3U,
    0xCB7DU, 0xDB5CU, 0xEB3FU, 0xFB1EU, 0x8BF9U, 0x9BD8U, 0xABBBU, 0xBB9AU,
    0x4A75U, 0x5A54U, 0x6A37U, 0x7A16U, 0x0AF1U, 0x1AD0U, 0x2AB3U, 0x3A92U,
    0xFD2EU, 0xED0FU, 0xDD6CU, 0xCD4DU, 0xBDAAU, 0xAD8BU, 0x9DE8U, 0x8DC9U,
    0x7C26U, 0x6C07U, 0x5C64U, 0x4C45U, 0x3CA2U, 0x2C83U, 0x1CE0U, 0x0CC1U,
    0xEF1FU, 0xFF3EU, 0xCF5DU, 0xDF7CU, 0xAF9BU, 0xBFBAU, 0x8FD9U, 0x9FF8U,
    0x6E17U, 0x7E36U, 0x4E55U, 0x5E74U, 0x2E93U, 0x3EB2U, 0x0ED1U, 0x1EF0U
};

/*============================================================================*/
/* PRIVATE VARIABLES                                                          */
/*============================================================================*/

/** @brief Hardware CRC handle */
static CRC_HandleTypeDef hcrc;

/** @brief Initialization flag */
static bool crc_initialized = false;

/*============================================================================*/
/* PRIVATE FUNCTION PROTOTYPES                                                */
/*============================================================================*/

static uint16_t crc_calculate16_software(const uint8_t *pData, uint32_t length);
static uint32_t crc_calculate32_software(const uint8_t *pData, uint32_t length);

/*============================================================================*/
/* PUBLIC FUNCTIONS                                                           */
/*============================================================================*/

/**
 * @brief Initialize CRC module
 */
Status_t CRC_Init(void)
{
    Status_t status = STATUS_OK;

    /* Enable CRC clock */
    __HAL_RCC_CRC_CLK_ENABLE();

    /* Configure hardware CRC */
    hcrc.Instance = CRC;
    hcrc.Init.DefaultPolynomialUse = DEFAULT_POLYNOMIAL_ENABLE;
    hcrc.Init.DefaultInitValueUse = DEFAULT_INIT_VALUE_ENABLE;
    hcrc.Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_NONE;
    hcrc.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_DISABLE;
    hcrc.InputDataFormat = CRC_INPUTDATA_FORMAT_BYTES;

    if (HAL_CRC_Init(&hcrc) != HAL_OK)
    {
        status = STATUS_ERROR_HW_FAULT;
    }
    else
    {
        crc_initialized = true;
    }

    return status;
}

/**
 * @brief Calculate CRC-16 (CCITT)
 */
uint16_t CRC_Calculate16(const uint8_t *pData, uint32_t length)
{
    uint16_t crc = CRC16_INIT_VALUE;

    /* Parameter validation */
    if ((pData == NULL) || (length == 0U))
    {
        crc = 0U;
    }
    else
    {
        crc = crc_calculate16_software(pData, length);
    }

    return crc;
}

/**
 * @brief Calculate CRC-32 (IEEE 802.3)
 */
uint32_t CRC_Calculate32(const uint8_t *pData, uint32_t length)
{
    uint32_t crc;

    /* Parameter validation */
    if ((pData == NULL) || (length == 0U))
    {
        crc = 0U;
    }
    else
    {
        /* Use hardware CRC if initialized */
        if (crc_initialized)
        {
            crc = CRC_Calculate32_Hardware(pData, length);
        }
        else
        {
            crc = crc_calculate32_software(pData, length);
        }
    }

    return crc;
}

/**
 * @brief Calculate CRC-32 using hardware peripheral
 */
uint32_t CRC_Calculate32_Hardware(const uint8_t *pData, uint32_t length)
{
    uint32_t crc = 0U;

    /* Parameter validation */
    if ((pData != NULL) && (length > 0U) && crc_initialized)
    {
        /* Reset CRC calculator */
        __HAL_CRC_DR_RESET(&hcrc);

        /* Calculate CRC using hardware */
        crc = HAL_CRC_Calculate(&hcrc, (uint32_t*)pData, length);
    }

    return crc;
}

/**
 * @brief Verify CRC-16 checksum
 */
bool CRC_Verify16(const uint8_t *pData, uint32_t length)
{
    bool isValid = false;
    uint16_t calculatedCRC;
    uint16_t storedCRC;

    /* Parameter validation */
    if ((pData != NULL) && (length >= 2U))
    {
        /* Calculate CRC over data (excluding last 2 bytes) */
        calculatedCRC = CRC_Calculate16(pData, length - 2U);

        /* Extract stored CRC (big-endian) */
        storedCRC = ((uint16_t)pData[length - 2U] << 8U) |
                    (uint16_t)pData[length - 1U];

        /* Compare */
        isValid = (calculatedCRC == storedCRC);
    }

    return isValid;
}

/**
 * @brief Verify CRC-32 checksum
 */
bool CRC_Verify32(const uint8_t *pData, uint32_t length)
{
    bool isValid = false;
    uint32_t calculatedCRC;
    uint32_t storedCRC;

    /* Parameter validation */
    if ((pData != NULL) && (length >= 4U))
    {
        /* Calculate CRC over data (excluding last 4 bytes) */
        calculatedCRC = CRC_Calculate32(pData, length - 4U);

        /* Extract stored CRC (big-endian) */
        storedCRC = ((uint32_t)pData[length - 4U] << 24U) |
                    ((uint32_t)pData[length - 3U] << 16U) |
                    ((uint32_t)pData[length - 2U] << 8U)  |
                    (uint32_t)pData[length - 1U];

        /* Compare */
        isValid = (calculatedCRC == storedCRC);
    }

    return isValid;
}

/**
 * @brief Append CRC-16 to data buffer
 */
Status_t CRC_Append16(uint8_t *pData, uint32_t length)
{
    Status_t status = STATUS_ERROR_PARAM;
    uint16_t crc;

    /* Parameter validation */
    if (pData != NULL)
    {
        /* Calculate CRC */
        crc = CRC_Calculate16(pData, length);

        /* Append CRC (big-endian) */
        pData[length] = (uint8_t)(crc >> 8U);
        pData[length + 1U] = (uint8_t)(crc & 0xFFU);

        status = STATUS_OK;
    }

    return status;
}

/**
 * @brief Append CRC-32 to data buffer
 */
Status_t CRC_Append32(uint8_t *pData, uint32_t length)
{
    Status_t status = STATUS_ERROR_PARAM;
    uint32_t crc;

    /* Parameter validation */
    if (pData != NULL)
    {
        /* Calculate CRC */
        crc = CRC_Calculate32(pData, length);

        /* Append CRC (big-endian) */
        pData[length] = (uint8_t)(crc >> 24U);
        pData[length + 1U] = (uint8_t)((crc >> 16U) & 0xFFU);
        pData[length + 2U] = (uint8_t)((crc >> 8U) & 0xFFU);
        pData[length + 3U] = (uint8_t)(crc & 0xFFU);

        status = STATUS_OK;
    }

    return status;
}

/*============================================================================*/
/* PRIVATE FUNCTIONS                                                          */
/*============================================================================*/

/**
 * @brief Calculate CRC-16 using software (lookup table)
 * @param[in] pData  Pointer to data buffer
 * @param[in] length Data length in bytes
 * @return Calculated CRC-16 value
 */
static uint16_t crc_calculate16_software(const uint8_t *pData, uint32_t length)
{
    uint16_t crc = CRC16_INIT_VALUE;
    uint32_t i;
    uint8_t index;

    for (i = 0U; i < length; i++)
    {
        index = (uint8_t)((crc >> 8U) ^ pData[i]);
        crc = (crc << 8U) ^ CRC16_TABLE[index];
    }

    return crc;
}

/**
 * @brief Calculate CRC-32 using software algorithm
 * @param[in] pData  Pointer to data buffer
 * @param[in] length Data length in bytes
 * @return Calculated CRC-32 value
 */
static uint32_t crc_calculate32_software(const uint8_t *pData, uint32_t length)
{
    uint32_t crc = CRC32_INIT_VALUE;
    uint32_t i;
    uint32_t j;
    uint8_t byte;

    for (i = 0U; i < length; i++)
    {
        byte = pData[i];
        crc ^= (uint32_t)byte << 24U;

        for (j = 0U; j < 8U; j++)
        {
            if ((crc & 0x80000000UL) != 0U)
            {
                crc = (crc << 1U) ^ CRC32_POLY_IEEE;
            }
            else
            {
                crc = crc << 1U;
            }
        }
    }

    return crc;
}

/*============================================================================*/
/* END OF FILE                                                                */
/*============================================================================*/
