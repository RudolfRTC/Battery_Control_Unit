/**
 * @file    bsp_spi.h
 * @brief   SPI bus abstraction layer for STM32F413
 * @author  Battery Control Unit Development Team
 * @date    2026-01-10
 * @version 1.0.0
 *
 * @note    MISRA C:2012 compliant
 * @note    Supports SPI4 for ISO-SPI (LTC6811 battery monitoring)
 *
 * @copyright Copyright (c) 2026
 */

#ifndef BSP_SPI_H
#define BSP_SPI_H

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

/** @brief SPI instances */
#define BSP_SPI_INSTANCE_4    (4U)  /**< SPI4 instance (ISO-SPI) */

/** @brief SPI clock speeds */
#define BSP_SPI_SPEED_LOW     (1000000UL)   /**< 1 MHz */
#define BSP_SPI_SPEED_MEDIUM  (2000000UL)   /**< 2 MHz */
#define BSP_SPI_SPEED_HIGH    (4000000UL)   /**< 4 MHz */

/** @brief Default SPI timeout (milliseconds) */
#define BSP_SPI_DEFAULT_TIMEOUT  (100U)

/** @brief Maximum transfer size */
#define BSP_SPI_MAX_TRANSFER_SIZE (256U)

/*============================================================================*/
/* TYPE DEFINITIONS                                                           */
/*============================================================================*/

/**
 * @brief SPI bus state
 */
typedef enum {
    SPI_STATE_UNINITIALIZED = 0x00U,  /**< Not initialized */
    SPI_STATE_READY         = 0x01U,  /**< Ready for operation */
    SPI_STATE_BUSY          = 0x02U,  /**< Busy with transfer */
    SPI_STATE_ERROR         = 0x03U   /**< Error state */
} SPI_State_t;

/**
 * @brief SPI mode
 */
typedef enum {
    SPI_MODE_0 = 0x00U,  /**< CPOL=0, CPHA=0 */
    SPI_MODE_1 = 0x01U,  /**< CPOL=0, CPHA=1 */
    SPI_MODE_2 = 0x02U,  /**< CPOL=1, CPHA=0 */
    SPI_MODE_3 = 0x03U   /**< CPOL=1, CPHA=1 */
} SPI_Mode_t;

/**
 * @brief SPI bit order
 */
typedef enum {
    SPI_BITORDER_MSB_FIRST = 0x00U,  /**< MSB transmitted first */
    SPI_BITORDER_LSB_FIRST = 0x01U   /**< LSB transmitted first */
} SPI_BitOrder_t;

/**
 * @brief SPI data size
 */
typedef enum {
    BSP_SPI_DATASIZE_8BIT  = 0x00U,  /**< 8-bit data frame */
    BSP_SPI_DATASIZE_16BIT = 0x01U   /**< 16-bit data frame */
} SPI_DataSize_t;

/**
 * @brief SPI configuration structure
 */
typedef struct {
    uint32_t         clockSpeed;   /**< Clock speed in Hz */
    SPI_Mode_t       mode;          /**< SPI mode (0-3) */
    SPI_BitOrder_t   bitOrder;      /**< Bit order */
    SPI_DataSize_t   dataSize;      /**< Data size */
    bool             useDMA;        /**< Use DMA for transfers */
} SPI_Config_t;

/**
 * @brief SPI statistics structure
 */
typedef struct {
    uint32_t txCount;       /**< Total bytes transmitted */
    uint32_t rxCount;       /**< Total bytes received */
    uint32_t errorCount;    /**< Total errors */
    uint32_t timeoutCount;  /**< Timeout count */
} SPI_Statistics_t;

/*============================================================================*/
/* FUNCTION PROTOTYPES                                                        */
/*============================================================================*/

/**
 * @brief Initialize SPI interface
 * @param[in] instance SPI instance (BSP_SPI_INSTANCE_4)
 * @param[in] pConfig  Pointer to configuration structure
 * @return STATUS_OK on success, error code otherwise
 */
Status_t BSP_SPI_Init(uint8_t instance, const SPI_Config_t *pConfig);

/**
 * @brief De-initialize SPI interface
 * @param[in] instance SPI instance
 * @return STATUS_OK on success
 */
Status_t BSP_SPI_DeInit(uint8_t instance);

/**
 * @brief Transmit data over SPI
 * @param[in] instance    SPI instance
 * @param[in] pData       Pointer to data buffer
 * @param[in] length      Number of bytes to transmit
 * @param[in] timeout_ms  Timeout in milliseconds
 * @return STATUS_OK on success, error code otherwise
 */
Status_t BSP_SPI_Transmit(uint8_t instance, const uint8_t *pData,
                          uint16_t length, uint32_t timeout_ms);

/**
 * @brief Receive data over SPI
 * @param[in]  instance    SPI instance
 * @param[out] pData       Pointer to data buffer
 * @param[in]  length      Number of bytes to receive
 * @param[in]  timeout_ms  Timeout in milliseconds
 * @return STATUS_OK on success, error code otherwise
 */
Status_t BSP_SPI_Receive(uint8_t instance, uint8_t *pData,
                         uint16_t length, uint32_t timeout_ms);

/**
 * @brief Transmit and receive data over SPI (full duplex)
 * @param[in]  instance    SPI instance
 * @param[in]  pTxData     Pointer to transmit data buffer
 * @param[out] pRxData     Pointer to receive data buffer
 * @param[in]  length      Number of bytes to transfer
 * @param[in]  timeout_ms  Timeout in milliseconds
 * @return STATUS_OK on success, error code otherwise
 */
Status_t BSP_SPI_TransmitReceive(uint8_t instance, const uint8_t *pTxData,
                                 uint8_t *pRxData, uint16_t length,
                                 uint32_t timeout_ms);

/**
 * @brief Assert SPI chip select (CS low)
 * @param[in] instance SPI instance
 * @return STATUS_OK on success
 */
Status_t BSP_SPI_ChipSelect(uint8_t instance, bool select);

/**
 * @brief Get SPI bus state
 * @param[in]  instance SPI instance
 * @param[out] pState   Pointer to store bus state
 * @return STATUS_OK on success
 */
Status_t BSP_SPI_GetState(uint8_t instance, SPI_State_t *pState);

/**
 * @brief Get SPI statistics
 * @param[in]  instance SPI instance
 * @param[out] pStats   Pointer to statistics structure
 * @return STATUS_OK on success
 */
Status_t BSP_SPI_GetStatistics(uint8_t instance, SPI_Statistics_t *pStats);

/**
 * @brief Reset SPI statistics
 * @param[in] instance SPI instance
 * @return STATUS_OK on success
 */
Status_t BSP_SPI_ResetStatistics(uint8_t instance);

/**
 * @brief Enable/disable ISO-SPI interface
 * @param[in] enable True to enable, false to disable
 * @return STATUS_OK on success
 */
Status_t BSP_SPI_EnableISOSPI(bool enable);

#ifdef __cplusplus
}
#endif

#endif /* BSP_SPI_H */
