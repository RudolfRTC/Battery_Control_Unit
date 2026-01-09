/**
 * @file    fram_driver.c
 * @brief   FRAM (Ferroelectric RAM) driver implementation
 * @author  Battery Control Unit Development Team
 * @date    2026-01-09
 * @version 1.0.0
 *
 * @note    MISRA C:2012 compliant
 * @note    CY15B256J 32KB FRAM (I2C interface)
 * @note    Unlimited write endurance, instant write
 *
 * @copyright Copyright (c) 2026
 */

/*============================================================================*/
/* INCLUDES                                                                   */
/*============================================================================*/
#include "fram_driver.h"
#include "bsp_i2c.h"
#include "crc.h"
#include <string.h>

/*============================================================================*/
/* PRIVATE CONSTANTS                                                          */
/*============================================================================*/

/** @brief FRAM I2C device address (7-bit) */
#define FRAM_I2C_ADDR           (0x50U)

/** @brief FRAM I2C instance */
#define FRAM_I2C_INSTANCE       (BSP_I2C_INSTANCE_2)

/** @brief FRAM page size (bytes) */
#define FRAM_PAGE_SIZE          (256U)

/** @brief FRAM total size (bytes) */
#define FRAM_TOTAL_SIZE         (32768U)

/** @brief Memory region sizes */
#define FRAM_CONFIG_SIZE        (1024U)
#define FRAM_CALIBRATION_SIZE   (2048U)
#define FRAM_FAULT_LOG_SIZE     (4096U)
#define FRAM_DATA_LOG_SIZE      (8192U)
#define FRAM_USER_SIZE          (16384U)

/** @brief Memory region offsets */
#define FRAM_CONFIG_OFFSET      (0U)
#define FRAM_CALIB_OFFSET       (FRAM_CONFIG_OFFSET + FRAM_CONFIG_SIZE)
#define FRAM_FAULT_LOG_OFFSET   (FRAM_CALIB_OFFSET + FRAM_CALIBRATION_SIZE)
#define FRAM_DATA_LOG_OFFSET    (FRAM_FAULT_LOG_OFFSET + FRAM_FAULT_LOG_SIZE)
#define FRAM_USER_OFFSET        (FRAM_DATA_LOG_OFFSET + FRAM_DATA_LOG_SIZE)

/** @brief Magic number for config validation */
#define FRAM_CONFIG_MAGIC       (0xBCU2026U)

/** @brief FRAM config header structure */
typedef struct {
    uint32_t magic;
    uint32_t version;
    uint32_t length;
    uint32_t crc32;
} FRAM_ConfigHeader_t;

/*============================================================================*/
/* PRIVATE VARIABLES                                                          */
/*============================================================================*/

/** @brief Initialization flag */
static bool fram_initialized = false;

/** @brief Write protect flag */
static bool fram_write_protected = false;

/** @brief Statistics */
static FRAM_Statistics_t fram_stats;

/*============================================================================*/
/* PRIVATE FUNCTION PROTOTYPES                                                */
/*============================================================================*/

static Status_t fram_read_bytes(uint16_t address, uint8_t *pData, uint16_t length);
static Status_t fram_write_bytes(uint16_t address, const uint8_t *pData, uint16_t length);
static bool fram_validate_range(uint16_t address, uint16_t length);

/*============================================================================*/
/* PUBLIC FUNCTIONS                                                           */
/*============================================================================*/

/**
 * @brief Initialize FRAM driver
 */
Status_t FRAM_Init(void)
{
    Status_t status = STATUS_OK;

    if (fram_initialized)
    {
        status = STATUS_ERROR_ALREADY_INIT;
    }
    else
    {
        /* Initialize statistics */
        (void)memset(&fram_stats, 0, sizeof(fram_stats));

        /* Test FRAM connectivity */
        uint8_t testData = 0xAAU;
        status = fram_read_bytes(0U, &testData, 1U);

        if (status == STATUS_OK)
        {
            fram_initialized = true;
            fram_write_protected = false;
        }
    }

    return status;
}

/**
 * @brief Read data from FRAM
 */
Status_t FRAM_Read(uint16_t address, uint8_t *pData, uint16_t length)
{
    Status_t status = STATUS_ERROR_PARAM;

    if ((pData != NULL) && fram_initialized && fram_validate_range(address, length))
    {
        status = fram_read_bytes(address, pData, length);

        if (status == STATUS_OK)
        {
            fram_stats.readCount++;
            fram_stats.bytesRead += (uint32_t)length;
        }
        else
        {
            fram_stats.readErrors++;
        }
    }

    return status;
}

/**
 * @brief Write data to FRAM
 */
Status_t FRAM_Write(uint16_t address, const uint8_t *pData, uint16_t length)
{
    Status_t status = STATUS_ERROR_PARAM;

    if ((pData != NULL) && fram_initialized && fram_validate_range(address, length))
    {
        if (fram_write_protected)
        {
            status = STATUS_ERROR_WRITE_PROTECTED;
            fram_stats.writeErrors++;
        }
        else
        {
            status = fram_write_bytes(address, pData, length);

            if (status == STATUS_OK)
            {
                fram_stats.writeCount++;
                fram_stats.bytesWritten += (uint32_t)length;
            }
            else
            {
                fram_stats.writeErrors++;
            }
        }
    }

    return status;
}

/**
 * @brief Erase FRAM region (fill with 0xFF)
 */
Status_t FRAM_Erase(uint16_t address, uint16_t length)
{
    Status_t status = STATUS_ERROR_PARAM;

    if (fram_initialized && fram_validate_range(address, length))
    {
        if (fram_write_protected)
        {
            status = STATUS_ERROR_WRITE_PROTECTED;
        }
        else
        {
            uint8_t eraseBuffer[FRAM_PAGE_SIZE];
            (void)memset(eraseBuffer, 0xFFU, sizeof(eraseBuffer));

            uint16_t remaining = length;
            uint16_t currentAddr = address;

            status = STATUS_OK;

            while ((remaining > 0U) && (status == STATUS_OK))
            {
                uint16_t chunkSize = (remaining > FRAM_PAGE_SIZE) ? FRAM_PAGE_SIZE : remaining;

                status = fram_write_bytes(currentAddr, eraseBuffer, chunkSize);

                if (status == STATUS_OK)
                {
                    currentAddr += chunkSize;
                    remaining -= chunkSize;
                }
            }
        }
    }

    return status;
}

/**
 * @brief Save configuration to FRAM
 */
Status_t FRAM_SaveConfig(const uint8_t *pConfig, uint16_t length)
{
    Status_t status = STATUS_ERROR_PARAM;

    if ((pConfig != NULL) && (length <= (FRAM_CONFIG_SIZE - sizeof(FRAM_ConfigHeader_t))))
    {
        FRAM_ConfigHeader_t header;

        /* Build header */
        header.magic = FRAM_CONFIG_MAGIC;
        header.version = 1U;
        header.length = (uint32_t)length;
        header.crc32 = CRC_Calculate32(pConfig, (uint32_t)length);

        /* Write header */
        status = FRAM_Write(FRAM_CONFIG_OFFSET, (const uint8_t *)&header, sizeof(header));

        if (status == STATUS_OK)
        {
            /* Write config data */
            status = FRAM_Write(FRAM_CONFIG_OFFSET + sizeof(header), pConfig, length);
        }
    }

    return status;
}

/**
 * @brief Load configuration from FRAM
 */
Status_t FRAM_LoadConfig(uint8_t *pConfig, uint16_t maxLength, uint16_t *pActualLength)
{
    Status_t status = STATUS_ERROR_PARAM;

    if ((pConfig != NULL) && (pActualLength != NULL))
    {
        FRAM_ConfigHeader_t header;

        /* Read header */
        status = FRAM_Read(FRAM_CONFIG_OFFSET, (uint8_t *)&header, sizeof(header));

        if (status == STATUS_OK)
        {
            /* Validate magic */
            if (header.magic != FRAM_CONFIG_MAGIC)
            {
                status = STATUS_ERROR_INVALID_DATA;
            }
            /* Validate length */
            else if ((header.length > maxLength) ||
                     (header.length > (FRAM_CONFIG_SIZE - sizeof(header))))
            {
                status = STATUS_ERROR_OVERFLOW;
            }
            else
            {
                /* Read config data */
                status = FRAM_Read(FRAM_CONFIG_OFFSET + sizeof(header),
                                  pConfig, (uint16_t)header.length);

                if (status == STATUS_OK)
                {
                    /* Verify CRC */
                    uint32_t calcCRC = CRC_Calculate32(pConfig, header.length);

                    if (calcCRC != header.crc32)
                    {
                        status = STATUS_ERROR_CRC;
                    }
                    else
                    {
                        *pActualLength = (uint16_t)header.length;
                    }
                }
            }
        }
    }

    return status;
}

/**
 * @brief Save calibration data to FRAM
 */
Status_t FRAM_SaveCalibration(const uint8_t *pCalibration, uint16_t length)
{
    Status_t status = STATUS_ERROR_PARAM;

    if ((pCalibration != NULL) && (length <= FRAM_CALIBRATION_SIZE))
    {
        status = FRAM_Write(FRAM_CALIB_OFFSET, pCalibration, length);
    }

    return status;
}

/**
 * @brief Load calibration data from FRAM
 */
Status_t FRAM_LoadCalibration(uint8_t *pCalibration, uint16_t length)
{
    Status_t status = STATUS_ERROR_PARAM;

    if ((pCalibration != NULL) && (length <= FRAM_CALIBRATION_SIZE))
    {
        status = FRAM_Read(FRAM_CALIB_OFFSET, pCalibration, length);
    }

    return status;
}

/**
 * @brief Write fault log entry
 */
Status_t FRAM_WriteFaultLog(uint16_t logIndex, const FRAM_FaultLogEntry_t *pEntry)
{
    Status_t status = STATUS_ERROR_PARAM;

    if (pEntry != NULL)
    {
        uint16_t maxEntries = FRAM_FAULT_LOG_SIZE / sizeof(FRAM_FaultLogEntry_t);

        if (logIndex < maxEntries)
        {
            uint16_t address = FRAM_FAULT_LOG_OFFSET +
                              (logIndex * (uint16_t)sizeof(FRAM_FaultLogEntry_t));

            status = FRAM_Write(address, (const uint8_t *)pEntry, sizeof(FRAM_FaultLogEntry_t));
        }
    }

    return status;
}

/**
 * @brief Read fault log entry
 */
Status_t FRAM_ReadFaultLog(uint16_t logIndex, FRAM_FaultLogEntry_t *pEntry)
{
    Status_t status = STATUS_ERROR_PARAM;

    if (pEntry != NULL)
    {
        uint16_t maxEntries = FRAM_FAULT_LOG_SIZE / sizeof(FRAM_FaultLogEntry_t);

        if (logIndex < maxEntries)
        {
            uint16_t address = FRAM_FAULT_LOG_OFFSET +
                              (logIndex * (uint16_t)sizeof(FRAM_FaultLogEntry_t));

            status = FRAM_Read(address, (uint8_t *)pEntry, sizeof(FRAM_FaultLogEntry_t));
        }
    }

    return status;
}

/**
 * @brief Get FRAM statistics
 */
Status_t FRAM_GetStatistics(FRAM_Statistics_t *pStats)
{
    Status_t status = STATUS_ERROR_PARAM;

    if (pStats != NULL)
    {
        *pStats = fram_stats;
        status = STATUS_OK;
    }

    return status;
}

/**
 * @brief Reset FRAM statistics
 */
Status_t FRAM_ResetStatistics(void)
{
    (void)memset(&fram_stats, 0, sizeof(fram_stats));
    return STATUS_OK;
}

/**
 * @brief Set write protection
 */
Status_t FRAM_SetWriteProtection(bool enable)
{
    fram_write_protected = enable;
    return STATUS_OK;
}

/**
 * @brief Test FRAM integrity
 */
Status_t FRAM_TestIntegrity(void)
{
    Status_t status = STATUS_OK;
    uint8_t testPattern[] = {0xAAU, 0x55U, 0xF0U, 0x0FU};
    uint8_t readback[4];

    /* Use last 4 bytes for testing */
    uint16_t testAddr = FRAM_TOTAL_SIZE - 4U;

    /* Save original data */
    uint8_t original[4];
    status = fram_read_bytes(testAddr, original, 4U);

    if (status == STATUS_OK)
    {
        /* Write test pattern */
        status = fram_write_bytes(testAddr, testPattern, 4U);
    }

    if (status == STATUS_OK)
    {
        /* Read back */
        status = fram_read_bytes(testAddr, readback, 4U);
    }

    if (status == STATUS_OK)
    {
        /* Verify */
        if (memcmp(testPattern, readback, 4U) != 0)
        {
            status = STATUS_ERROR;
        }
        else
        {
            /* Restore original data */
            status = fram_write_bytes(testAddr, original, 4U);
        }
    }

    return status;
}

/**
 * @brief De-initialize FRAM driver
 */
Status_t FRAM_DeInit(void)
{
    if (fram_initialized)
    {
        fram_initialized = false;
    }

    return STATUS_OK;
}

/*============================================================================*/
/* PRIVATE FUNCTIONS                                                          */
/*============================================================================*/

/**
 * @brief Read bytes from FRAM via I2C
 */
static Status_t fram_read_bytes(uint16_t address, uint8_t *pData, uint16_t length)
{
    Status_t status = STATUS_ERROR_PARAM;

    if ((pData != NULL) && (length > 0U))
    {
        status = BSP_I2C_Read(FRAM_I2C_INSTANCE, FRAM_I2C_ADDR,
                             address, pData, length, TIMEOUT_I2C_MS);
    }

    return status;
}

/**
 * @brief Write bytes to FRAM via I2C
 */
static Status_t fram_write_bytes(uint16_t address, const uint8_t *pData, uint16_t length)
{
    Status_t status = STATUS_ERROR_PARAM;

    if ((pData != NULL) && (length > 0U))
    {
        status = BSP_I2C_Write(FRAM_I2C_INSTANCE, FRAM_I2C_ADDR,
                              address, pData, length, TIMEOUT_I2C_MS);
    }

    return status;
}

/**
 * @brief Validate address range
 */
static bool fram_validate_range(uint16_t address, uint16_t length)
{
    bool valid = false;

    if ((length > 0U) && ((uint32_t)address + (uint32_t)length <= FRAM_TOTAL_SIZE))
    {
        valid = true;
    }

    return valid;
}

/*============================================================================*/
/* END OF FILE                                                                */
/*============================================================================*/
