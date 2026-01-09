/**
 * @file    bsp_i2c.h
 * @brief   I2C bus abstraction layer for STM32F413
 * @author  Battery Control Unit Development Team
 * @date    2026-01-09
 * @version 1.0.0
 *
 * @note    MISRA C:2012 compliant
 * @note    Supports FRAM (CY15B256J) and TMP1075 temperature sensor
 *
 * @copyright Copyright (c) 2026
 */

#ifndef BSP_I2C_H
#define BSP_I2C_H

#ifdef __cplusplus
extern "C" {
#endif

/*============================================================================*/
/* INCLUDES                                                                   */
/*============================================================================*/
#include "app_types.h"
#include "stm32f4xx_hal.h"

/*============================================================================*/
/* CONSTANTS                                                                  */
/*============================================================================*/

/** @brief I2C instances */
#define BSP_I2C_INSTANCE_2    (1U)  /**< I2C2 instance (main bus) */

/** @brief I2C clock speeds */
#define BSP_I2C_SPEED_STANDARD   (100000UL)  /**< 100 kHz standard mode */
#define BSP_I2C_SPEED_FAST       (400000UL)  /**< 400 kHz fast mode */

/** @brief Default I2C timeout (milliseconds) */
#define BSP_I2C_DEFAULT_TIMEOUT  (50U)

/** @brief Maximum transfer size */
#define BSP_I2C_MAX_TRANSFER_SIZE (256U)

/*============================================================================*/
/* TYPE DEFINITIONS                                                           */
/*============================================================================*/

/**
 * @brief I2C bus state
 */
typedef enum {
    I2C_STATE_UNINITIALIZED = 0x00U,  /**< Not initialized */
    I2C_STATE_READY         = 0x01U,  /**< Ready for operation */
    I2C_STATE_BUSY          = 0x02U,  /**< Busy with transfer */
    I2C_STATE_ERROR         = 0x03U   /**< Error state */
} I2C_State_t;

/**
 * @brief I2C addressing mode
 */
typedef enum {
    I2C_ADDR_7BIT  = 0x00U,  /**< 7-bit addressing */
    I2C_ADDR_10BIT = 0x01U   /**< 10-bit addressing */
} I2C_AddressMode_t;

/**
 * @brief I2C configuration structure
 */
typedef struct {
    uint32_t clockSpeed;       /**< Clock speed in Hz */
    I2C_AddressMode_t addrMode; /**< Addressing mode */
    bool     useDMA;           /**< Use DMA for transfers */
    uint8_t  ownAddress;       /**< Own address (if slave) */
} I2C_Config_t;

/**
 * @brief I2C statistics structure
 */
typedef struct {
    uint32_t txCount;       /**< Total bytes transmitted */
    uint32_t rxCount;       /**< Total bytes received */
    uint32_t errorCount;    /**< Total errors */
    uint32_t nackCount;     /**< NACK count */
    uint32_t timeoutCount;  /**< Timeout count */
    uint32_t busErrors;     /**< Bus error count */
} I2C_Statistics_t;

/*============================================================================*/
/* FUNCTION PROTOTYPES                                                        */
/*============================================================================*/

/**
 * @brief Initialize I2C interface
 * @param[in] instance I2C instance
 * @param[in] pConfig  Pointer to configuration structure
 * @return STATUS_OK on success, error code otherwise
 */
Status_t BSP_I2C_Init(uint8_t instance, const I2C_Config_t *pConfig);

/**
 * @brief De-initialize I2C interface
 * @param[in] instance I2C instance
 * @return STATUS_OK on success
 */
Status_t BSP_I2C_DeInit(uint8_t instance);

/**
 * @brief Write data to I2C device
 * @param[in] instance    I2C instance
 * @param[in] deviceAddr  7-bit device address (not shifted)
 * @param[in] regAddr     Register address (can be 0)
 * @param[in] pData       Pointer to data buffer
 * @param[in] length      Number of bytes to write
 * @param[in] timeout_ms  Timeout in milliseconds
 * @return STATUS_OK on success, error code otherwise
 */
Status_t BSP_I2C_Write(uint8_t instance, uint8_t deviceAddr, uint16_t regAddr,
                       const uint8_t *pData, uint16_t length, uint32_t timeout_ms);

/**
 * @brief Read data from I2C device
 * @param[in]  instance    I2C instance
 * @param[in]  deviceAddr  7-bit device address (not shifted)
 * @param[in]  regAddr     Register address (can be 0)
 * @param[out] pData       Pointer to data buffer
 * @param[in]  length      Number of bytes to read
 * @param[in]  timeout_ms  Timeout in milliseconds
 * @return STATUS_OK on success, error code otherwise
 */
Status_t BSP_I2C_Read(uint8_t instance, uint8_t deviceAddr, uint16_t regAddr,
                      uint8_t *pData, uint16_t length, uint32_t timeout_ms);

/**
 * @brief Write data to I2C device (no register address)
 * @param[in] instance    I2C instance
 * @param[in] deviceAddr  7-bit device address (not shifted)
 * @param[in] pData       Pointer to data buffer
 * @param[in] length      Number of bytes to write
 * @param[in] timeout_ms  Timeout in milliseconds
 * @return STATUS_OK on success, error code otherwise
 */
Status_t BSP_I2C_WriteRaw(uint8_t instance, uint8_t deviceAddr,
                          const uint8_t *pData, uint16_t length,
                          uint32_t timeout_ms);

/**
 * @brief Read data from I2C device (no register address)
 * @param[in]  instance    I2C instance
 * @param[in]  deviceAddr  7-bit device address (not shifted)
 * @param[out] pData       Pointer to data buffer
 * @param[in]  length      Number of bytes to read
 * @param[in]  timeout_ms  Timeout in milliseconds
 * @return STATUS_OK on success, error code otherwise
 */
Status_t BSP_I2C_ReadRaw(uint8_t instance, uint8_t deviceAddr,
                         uint8_t *pData, uint16_t length,
                         uint32_t timeout_ms);

/**
 * @brief Write then read (combined transaction)
 * @param[in]  instance    I2C instance
 * @param[in]  deviceAddr  7-bit device address (not shifted)
 * @param[in]  pWriteData  Pointer to write data buffer
 * @param[in]  writeLength Number of bytes to write
 * @param[out] pReadData   Pointer to read data buffer
 * @param[in]  readLength  Number of bytes to read
 * @param[in]  timeout_ms  Timeout in milliseconds
 * @return STATUS_OK on success, error code otherwise
 */
Status_t BSP_I2C_WriteRead(uint8_t instance, uint8_t deviceAddr,
                           const uint8_t *pWriteData, uint16_t writeLength,
                           uint8_t *pReadData, uint16_t readLength,
                           uint32_t timeout_ms);

/**
 * @brief Check if I2C device is ready
 * @param[in] instance    I2C instance
 * @param[in] deviceAddr  7-bit device address (not shifted)
 * @param[in] trials      Number of trials
 * @param[in] timeout_ms  Timeout in milliseconds
 * @return STATUS_OK if device ready, error code otherwise
 */
Status_t BSP_I2C_IsDeviceReady(uint8_t instance, uint8_t deviceAddr,
                               uint8_t trials, uint32_t timeout_ms);

/**
 * @brief Scan I2C bus for devices
 * @param[in]  instance    I2C instance
 * @param[out] pDevices    Array to store found device addresses
 * @param[in]  maxDevices  Maximum number of devices to find
 * @param[out] pCount      Number of devices found
 * @return STATUS_OK on success
 */
Status_t BSP_I2C_ScanBus(uint8_t instance, uint8_t *pDevices,
                         uint8_t maxDevices, uint8_t *pCount);

/**
 * @brief Get I2C bus state
 * @param[in]  instance I2C instance
 * @param[out] pState   Pointer to store bus state
 * @return STATUS_OK on success
 */
Status_t BSP_I2C_GetState(uint8_t instance, I2C_State_t *pState);

/**
 * @brief Get I2C statistics
 * @param[in]  instance I2C instance
 * @param[out] pStats   Pointer to statistics structure
 * @return STATUS_OK on success
 */
Status_t BSP_I2C_GetStatistics(uint8_t instance, I2C_Statistics_t *pStats);

/**
 * @brief Reset I2C statistics
 * @param[in] instance I2C instance
 * @return STATUS_OK on success
 */
Status_t BSP_I2C_ResetStatistics(uint8_t instance);

/**
 * @brief Reset I2C bus (recover from stuck state)
 * @param[in] instance I2C instance
 * @return STATUS_OK on success
 */
Status_t BSP_I2C_ResetBus(uint8_t instance);

#ifdef __cplusplus
}
#endif

#endif /* BSP_I2C_H */
