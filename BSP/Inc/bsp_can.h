/**
 * @file    bsp_can.h
 * @brief   CAN bus abstraction layer for STM32F413 (dual CAN)
 * @author  Battery Control Unit Development Team
 * @date    2026-01-09
 * @version 1.0.0
 *
 * @note    MISRA C:2012 compliant
 * @note    Supports CAN 2.0B and CAN-FD
 * @note    TCAN3404 transceivers with fault detection
 *
 * @copyright Copyright (c) 2026
 */

#ifndef BSP_CAN_H
#define BSP_CAN_H

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

/** @brief CAN instances */
#define BSP_CAN_INSTANCE_1    (0U)  /**< CAN1 instance */
#define BSP_CAN_INSTANCE_2    (1U)  /**< CAN2 instance */
#define BSP_CAN_INSTANCE_MAX  (2U)  /**< Total CAN instances */

/** @brief CAN bitrate presets */
#define BSP_CAN_BITRATE_125K  (125000UL)   /**< 125 kbit/s */
#define BSP_CAN_BITRATE_250K  (250000UL)   /**< 250 kbit/s */
#define BSP_CAN_BITRATE_500K  (500000UL)   /**< 500 kbit/s */
#define BSP_CAN_BITRATE_1M    (1000000UL)  /**< 1 Mbit/s */

/** @brief CAN message limits */
#define BSP_CAN_MAX_DATA_LEN  (8U)   /**< Max 8 bytes per CAN 2.0B frame */
#define BSP_CAN_TX_MAILBOXES  (3U)   /**< 3 TX mailboxes available */
#define BSP_CAN_RX_FIFO_SIZE  (3U)   /**< 3 messages per RX FIFO */

/*============================================================================*/
/* TYPE DEFINITIONS                                                           */
/*============================================================================*/

/**
 * @brief CAN frame type
 */
typedef enum {
    CAN_FRAME_STANDARD = 0x00U,  /**< 11-bit standard ID */
    CAN_FRAME_EXTENDED = 0x01U   /**< 29-bit extended ID */
} CAN_FrameType_t;

/**
 * @brief CAN frame format
 */
typedef enum {
    CAN_FRAME_DATA   = 0x00U,  /**< Data frame */
    CAN_FRAME_REMOTE = 0x01U   /**< Remote frame (RTR) */
} CAN_FrameFormat_t;

/**
 * @brief CAN bus state
 */
typedef enum {
    CAN_STATE_UNINITIALIZED = 0x00U,  /**< Not initialized */
    CAN_STATE_READY         = 0x01U,  /**< Ready for operation */
    CAN_STATE_BUSY          = 0x02U,  /**< Busy transmitting */
    CAN_STATE_ERROR_ACTIVE  = 0x03U,  /**< Error active */
    CAN_STATE_ERROR_PASSIVE = 0x04U,  /**< Error passive */
    CAN_STATE_BUS_OFF       = 0x05U   /**< Bus-off */
} CAN_BusState_t;

/**
 * @brief CAN operating mode
 */
typedef enum {
    BSP_CAN_MODE_NORMAL     = 0x00U,  /**< Normal operation */
    BSP_CAN_MODE_LOOPBACK   = 0x01U,  /**< Loopback mode (testing) */
    BSP_CAN_MODE_SILENT     = 0x02U,  /**< Silent mode (listen only) */
    BSP_CAN_MODE_SILENT_LOOPBACK = 0x03U  /**< Silent loopback */
} CAN_Mode_t;

/**
 * @brief CAN message structure
 */
typedef struct {
    uint32_t         id;           /**< CAN identifier (11 or 29-bit) */
    CAN_FrameType_t  frameType;    /**< Standard or extended */
    CAN_FrameFormat_t frameFormat; /**< Data or remote */
    uint8_t          dataLength;   /**< Data length (0-8 bytes) */
    uint8_t          data[BSP_CAN_MAX_DATA_LEN];  /**< Message data */
    uint32_t         timestamp_ms; /**< Reception timestamp */
} CAN_Message_t;

/**
 * @brief CAN filter configuration
 */
typedef struct {
    uint32_t id;         /**< Filter ID */
    uint32_t mask;       /**< Filter mask (0 = don't care) */
    uint8_t  fifo;       /**< FIFO assignment (0 or 1) */
    uint8_t  filterBank; /**< Filter bank number (0-13) */
    bool     enabled;    /**< Filter enabled */
} CAN_Filter_t;

/**
 * @brief CAN configuration structure
 */
typedef struct {
    uint32_t     bitrate;      /**< Bitrate in bps */
    CAN_Mode_t   mode;         /**< Operating mode */
    bool         autoRetransmit; /**< Automatic retransmission */
    bool         rxFifo0Overrun; /**< FIFO0 overrun mode */
    bool         rxFifo1Overrun; /**< FIFO1 overrun mode */
    uint8_t      txPriority;   /**< TX priority (0-3) */
} CAN_Config_t;

/**
 * @brief CAN statistics structure
 */
typedef struct {
    uint32_t txCount;           /**< Total transmitted messages */
    uint32_t rxCount;           /**< Total received messages */
    uint32_t txErrorCount;      /**< TX error count */
    uint32_t rxErrorCount;      /**< RX error count */
    uint32_t busOffCount;       /**< Bus-off occurrences */
    uint32_t rxOverrunCount;    /**< RX FIFO overrun count */
    uint32_t txMailboxFullCount;/**< TX mailbox full count */
    uint32_t lastErrorCode;     /**< Last error code */
} CAN_Statistics_t;

/**
 * @brief CAN callback function types
 */
typedef void (*CAN_RxCallback_t)(uint8_t instance, const CAN_Message_t *pMessage);
typedef void (*CAN_TxCallback_t)(uint8_t instance, uint32_t mailbox);
typedef void (*CAN_ErrorCallback_t)(uint8_t instance, uint32_t errorCode);

/*============================================================================*/
/* FUNCTION PROTOTYPES                                                        */
/*============================================================================*/

/**
 * @brief Initialize CAN interface
 * @param[in] instance CAN instance (0=CAN1, 1=CAN2)
 * @param[in] pConfig  Pointer to configuration structure
 * @return STATUS_OK on success, error code otherwise
 */
Status_t BSP_CAN_Init(uint8_t instance, const CAN_Config_t *pConfig);

/**
 * @brief De-initialize CAN interface
 * @param[in] instance CAN instance
 * @return STATUS_OK on success
 */
Status_t BSP_CAN_DeInit(uint8_t instance);

/**
 * @brief Start CAN bus communication
 * @param[in] instance CAN instance
 * @return STATUS_OK on success, error code otherwise
 */
Status_t BSP_CAN_Start(uint8_t instance);

/**
 * @brief Stop CAN bus communication
 * @param[in] instance CAN instance
 * @return STATUS_OK on success
 */
Status_t BSP_CAN_Stop(uint8_t instance);

/**
 * @brief Transmit CAN message
 * @param[in]  instance  CAN instance
 * @param[in]  pMessage  Pointer to message structure
 * @param[out] pMailbox  TX mailbox used (can be NULL)
 * @param[in]  timeout_ms Timeout in milliseconds (0 = no wait)
 * @return STATUS_OK on success, error code otherwise
 */
Status_t BSP_CAN_Transmit(uint8_t instance, const CAN_Message_t *pMessage,
                          uint32_t *pMailbox, uint32_t timeout_ms);

/**
 * @brief Receive CAN message from FIFO
 * @param[in]  instance  CAN instance
 * @param[in]  fifo      FIFO number (0 or 1)
 * @param[out] pMessage  Pointer to store received message
 * @param[in]  timeout_ms Timeout in milliseconds (0 = no wait)
 * @return STATUS_OK on success, error code otherwise
 */
Status_t BSP_CAN_Receive(uint8_t instance, uint8_t fifo,
                         CAN_Message_t *pMessage, uint32_t timeout_ms);

/**
 * @brief Check if CAN message is available in FIFO
 * @param[in]  instance CAN instance
 * @param[in]  fifo     FIFO number (0 or 1)
 * @param[out] pCount   Number of messages pending (can be NULL)
 * @return true if message available, false otherwise
 */
bool BSP_CAN_IsMessageAvailable(uint8_t instance, uint8_t fifo, uint32_t *pCount);

/**
 * @brief Configure CAN filter
 * @param[in] instance CAN instance
 * @param[in] pFilter  Pointer to filter configuration
 * @return STATUS_OK on success, error code otherwise
 */
Status_t BSP_CAN_ConfigureFilter(uint8_t instance, const CAN_Filter_t *pFilter);

/**
 * @brief Disable CAN filter
 * @param[in] instance   CAN instance
 * @param[in] filterBank Filter bank number
 * @return STATUS_OK on success
 */
Status_t BSP_CAN_DisableFilter(uint8_t instance, uint8_t filterBank);

/**
 * @brief Get CAN bus state
 * @param[in]  instance CAN instance
 * @param[out] pState   Pointer to store bus state
 * @return STATUS_OK on success
 */
Status_t BSP_CAN_GetBusState(uint8_t instance, CAN_BusState_t *pState);

/**
 * @brief Get CAN error counters
 * @param[in]  instance    CAN instance
 * @param[out] pTxErrors   TX error counter (can be NULL)
 * @param[out] pRxErrors   RX error counter (can be NULL)
 * @return STATUS_OK on success
 */
Status_t BSP_CAN_GetErrorCounters(uint8_t instance, uint8_t *pTxErrors,
                                  uint8_t *pRxErrors);

/**
 * @brief Get CAN statistics
 * @param[in]  instance CAN instance
 * @param[out] pStats   Pointer to statistics structure
 * @return STATUS_OK on success
 */
Status_t BSP_CAN_GetStatistics(uint8_t instance, CAN_Statistics_t *pStats);

/**
 * @brief Reset CAN statistics
 * @param[in] instance CAN instance
 * @return STATUS_OK on success
 */
Status_t BSP_CAN_ResetStatistics(uint8_t instance);

/**
 * @brief Abort pending CAN transmission
 * @param[in] instance CAN instance
 * @param[in] mailbox  Mailbox to abort (0-2)
 * @return STATUS_OK on success
 */
Status_t BSP_CAN_AbortTransmission(uint8_t instance, uint32_t mailbox);

/**
 * @brief Register RX callback function
 * @param[in] instance CAN instance
 * @param[in] callback Callback function pointer
 * @return STATUS_OK on success
 */
Status_t BSP_CAN_RegisterRxCallback(uint8_t instance, CAN_RxCallback_t callback);

/**
 * @brief Register TX callback function
 * @param[in] instance CAN instance
 * @param[in] callback Callback function pointer
 * @return STATUS_OK on success
 */
Status_t BSP_CAN_RegisterTxCallback(uint8_t instance, CAN_TxCallback_t callback);

/**
 * @brief Register error callback function
 * @param[in] instance CAN instance
 * @param[in] callback Callback function pointer
 * @return STATUS_OK on success
 */
Status_t BSP_CAN_RegisterErrorCallback(uint8_t instance, CAN_ErrorCallback_t callback);

/**
 * @brief Enable/disable CAN transceiver standby mode
 * @param[in] instance CAN instance
 * @param[in] enable   true to enable standby, false to disable
 * @return STATUS_OK on success
 */
Status_t BSP_CAN_SetStandbyMode(uint8_t instance, bool enable);

/**
 * @brief Check CAN transceiver fault status
 * @param[in]  instance CAN instance
 * @param[out] pFault   true if fault detected, false otherwise
 * @return STATUS_OK on success
 */
Status_t BSP_CAN_GetTransceiverFault(uint8_t instance, bool *pFault);

/*============================================================================*/
/* HELPER FUNCTIONS                                                           */
/*============================================================================*/

/**
 * @brief Build standard CAN message (11-bit ID)
 * @param[out] pMessage    Pointer to message structure
 * @param[in]  id          CAN identifier (11-bit)
 * @param[in]  pData       Pointer to data bytes
 * @param[in]  dataLength  Data length (0-8)
 * @return STATUS_OK on success
 */
Status_t BSP_CAN_BuildStandardMessage(CAN_Message_t *pMessage, uint16_t id,
                                      const uint8_t *pData, uint8_t dataLength);

/**
 * @brief Build extended CAN message (29-bit ID)
 * @param[out] pMessage    Pointer to message structure
 * @param[in]  id          CAN identifier (29-bit)
 * @param[in]  pData       Pointer to data bytes
 * @param[in]  dataLength  Data length (0-8)
 * @return STATUS_OK on success
 */
Status_t BSP_CAN_BuildExtendedMessage(CAN_Message_t *pMessage, uint32_t id,
                                      const uint8_t *pData, uint8_t dataLength);

#ifdef __cplusplus
}
#endif

#endif /* BSP_CAN_H */
