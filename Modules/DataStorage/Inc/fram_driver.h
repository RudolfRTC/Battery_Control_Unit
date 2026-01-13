/**
 * @file    fram_driver.h
 * @brief   FRAM (CY15B256J) non-volatile memory driver
 * @author  Battery Control Unit Development Team
 * @date    2026-01-09
 * @version 1.0.0
 *
 * @note    MISRA C:2012 compliant
 * @note    256 Kbit (32 KB) FRAM with I2C interface
 * @note    Unlimited endurance (10^14 read/write cycles)
 *
 * @copyright Copyright (c) 2026
 */

#ifndef FRAM_DRIVER_H
#define FRAM_DRIVER_H

#ifdef __cplusplus
extern "C" {
#endif

/*============================================================================*/
/* INCLUDES                                                                   */
/*============================================================================*/
#include "app_types.h"
#include "app_config.h"

/* Forward declaration to avoid circular dependency */
#ifndef ERROR_CODE_T_DEFINED
typedef uint16_t ErrorCode_t;
#define ERROR_CODE_T_DEFINED
#endif

/*============================================================================*/
/* CONSTANTS                                                                  */
/*============================================================================*/

/** @brief FRAM device configuration */
#define FRAM_I2C_ADDRESS      (0xA0U >> 1)  /**< 7-bit I2C address (0x50) */
#define FRAM_SIZE_BYTES       (32768U)      /**< 32 KB total size */
#define FRAM_PAGE_SIZE        (256U)        /**< Page size for operations */

/** @brief Memory regions (from app_config.h) */
#define FRAM_REGION_CONFIG    FRAM_ADDR_CONFIG_START
#define FRAM_REGION_CAL_IN    FRAM_ADDR_CAL_INPUT_START
#define FRAM_REGION_CAL_OUT   FRAM_ADDR_CAL_OUTPUT_START
#define FRAM_REGION_FAULTS    FRAM_ADDR_FAULT_LOG_START
#define FRAM_REGION_USER      FRAM_ADDR_USER_DATA_START

/** @brief CRC configuration */
#define FRAM_USE_CRC          (1U)   /**< Enable CRC protection */
#define FRAM_CRC_SIZE         (4U)   /**< CRC-32 size in bytes */

/*============================================================================*/
/* TYPE DEFINITIONS                                                           */
/*============================================================================*/

/**
 * @brief FRAM status
 */
typedef enum {
    FRAM_STATUS_OK            = 0x00U,  /**< Operation successful */
    FRAM_STATUS_NOT_INIT      = 0x01U,  /**< Not initialized */
    FRAM_STATUS_BUSY          = 0x02U,  /**< FRAM busy */
    FRAM_STATUS_ERROR_I2C     = 0x03U,  /**< I2C communication error */
    FRAM_STATUS_ERROR_CRC     = 0x04U,  /**< CRC check failed */
    FRAM_STATUS_ERROR_ADDRESS = 0x05U,  /**< Invalid address */
    FRAM_STATUS_ERROR_SIZE    = 0x06U   /**< Invalid size */
} FRAM_Status_t;

/**
 * @brief FRAM statistics
 */
typedef struct {
    uint32_t writeCount;      /**< Total write operations */
    uint32_t readCount;       /**< Total read operations */
    uint32_t bytesWritten;    /**< Total bytes written */
    uint32_t bytesRead;       /**< Total bytes read */
    uint32_t crcErrorCount;   /**< CRC check failures */
    uint32_t i2cErrorCount;   /**< I2C errors */
    uint32_t writeErrors;     /**< Write error count */
    uint32_t readErrors;      /**< Read error count */
} FRAM_Statistics_t;

/**
 * @brief Fault log entry structure for FRAM storage
 */
typedef struct {
    ErrorCode_t errorCode;    /**< Error code */
    uint32_t timestamp_ms;    /**< Timestamp in milliseconds */
    uint32_t param1;          /**< Error parameter 1 */
    uint32_t param2;          /**< Error parameter 2 */
    uint32_t param3;          /**< Error parameter 3 */
    uint16_t crc;             /**< CRC for data integrity */
} FRAM_FaultLogEntry_t;

/*============================================================================*/
/* FUNCTION PROTOTYPES                                                        */
/*============================================================================*/

/**
 * @brief Initialize FRAM driver
 * @return STATUS_OK on success, error code otherwise
 */
Status_t FRAM_Init(void);

/**
 * @brief De-initialize FRAM driver
 * @return STATUS_OK on success
 */
Status_t FRAM_DeInit(void);

/**
 * @brief Write data to FRAM
 * @param[in] address Start address (0-32767)
 * @param[in] pData   Pointer to data buffer
 * @param[in] length  Number of bytes to write
 * @return STATUS_OK on success, error code otherwise
 */
Status_t FRAM_Write(uint16_t address, const uint8_t *pData, uint16_t length);

/**
 * @brief Read data from FRAM
 * @param[in]  address Start address (0-32767)
 * @param[out] pData   Pointer to data buffer
 * @param[in]  length  Number of bytes to read
 * @return STATUS_OK on success, error code otherwise
 */
Status_t FRAM_Read(uint16_t address, uint8_t *pData, uint16_t length);

/**
 * @brief Write data with CRC protection
 * @param[in] address Start address
 * @param[in] pData   Pointer to data buffer
 * @param[in] length  Number of bytes (excluding CRC)
 * @return STATUS_OK on success, error code otherwise
 */
Status_t FRAM_WriteWithCRC(uint16_t address, const uint8_t *pData, uint16_t length);

/**
 * @brief Read data and verify CRC
 * @param[in]  address Start address
 * @param[out] pData   Pointer to data buffer
 * @param[in]  length  Number of bytes (excluding CRC)
 * @return STATUS_OK on success, STATUS_ERROR_CRC if CRC mismatch
 */
Status_t FRAM_ReadWithCRC(uint16_t address, uint8_t *pData, uint16_t length);

/**
 * @brief Fill FRAM region with pattern
 * @param[in] address Start address
 * @param[in] pattern Fill pattern byte
 * @param[in] length  Number of bytes to fill
 * @return STATUS_OK on success, error code otherwise
 */
Status_t FRAM_Fill(uint16_t address, uint8_t pattern, uint16_t length);

/**
 * @brief Clear FRAM region (fill with zeros)
 * @param[in] address Start address
 * @param[in] length  Number of bytes to clear
 * @return STATUS_OK on success
 */
Status_t FRAM_Clear(uint16_t address, uint16_t length);

/**
 * @brief Verify FRAM data against buffer
 * @param[in] address Start address
 * @param[in] pData   Pointer to reference data buffer
 * @param[in] length  Number of bytes to verify
 * @return STATUS_OK if data matches, error code otherwise
 */
Status_t FRAM_Verify(uint16_t address, const uint8_t *pData, uint16_t length);

/**
 * @brief Check if FRAM is ready
 * @param[out] pReady true if ready, false if busy
 * @return STATUS_OK on success
 */
Status_t FRAM_IsReady(bool *pReady);

/**
 * @brief Get FRAM statistics
 * @param[out] pStats Pointer to statistics structure
 * @return STATUS_OK on success
 */
Status_t FRAM_GetStatistics(FRAM_Statistics_t *pStats);

/**
 * @brief Reset FRAM statistics
 * @return STATUS_OK on success
 */
Status_t FRAM_ResetStatistics(void);

/**
 * @brief Perform FRAM self-test
 * @return STATUS_OK if test passed, error code otherwise
 */
Status_t FRAM_SelfTest(void);

/**
 * @brief Write fault log entry to FRAM
 * @param[in] index    Fault log entry index (0 to max entries)
 * @param[in] pEntry   Pointer to fault log entry
 * @return STATUS_OK on success, error code otherwise
 */
Status_t FRAM_WriteFaultLog(uint16_t index, const FRAM_FaultLogEntry_t *pEntry);

/**
 * @brief Read fault log entry from FRAM
 * @param[in]  index    Fault log entry index (0 to max entries)
 * @param[out] pEntry   Pointer to store fault log entry
 * @return STATUS_OK on success, error code otherwise
 */
Status_t FRAM_ReadFaultLog(uint16_t index, FRAM_FaultLogEntry_t *pEntry);

#ifdef __cplusplus
}
#endif

#endif /* FRAM_DRIVER_H */
