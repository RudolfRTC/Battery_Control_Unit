/**
 * @file    power_mgmt.h
 * @brief   Power Management - Ultra Low Power with CAN Wake-up
 * @author  Battery Control Unit Development Team
 * @date    2026-01-24
 * @version 1.0.0
 *
 * @note    MISRA C:2012 compliant
 * @note    ISO 26262 ASIL-B safety requirements
 * @note    Supports STOP mode with CAN wake-up filter
 *
 * @copyright Copyright (c) 2026
 */

#ifndef POWER_MGMT_H
#define POWER_MGMT_H

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

/** @brief CAN wake-up message ID - EXACT match required */
#define PWR_CAN_WAKEUP_MSG_ID       (0x220U)

/** @brief CAN wake-up message magic bytes (for extra verification) */
#define PWR_WAKEUP_MAGIC_BYTE0      (0xA5U)
#define PWR_WAKEUP_MAGIC_BYTE1      (0x5AU)

/** @brief Power mode definitions */
#define PWR_MODE_RUN                (0x00U)  /**< Normal run mode */
#define PWR_MODE_SLEEP              (0x01U)  /**< Sleep mode (CPU halted) */
#define PWR_MODE_STOP               (0x02U)  /**< Stop mode (low power) */
#define PWR_MODE_STANDBY            (0x03U)  /**< Standby mode (lowest power) */

/** @brief Wake-up source flags */
#define PWR_WAKEUP_SRC_CAN          (0x01U)  /**< CAN message wake-up */
#define PWR_WAKEUP_SRC_RTC          (0x02U)  /**< RTC alarm wake-up */
#define PWR_WAKEUP_SRC_EXTI         (0x04U)  /**< External interrupt wake-up */
#define PWR_WAKEUP_SRC_WATCHDOG     (0x08U)  /**< Watchdog refresh wake-up */
#define PWR_WAKEUP_SRC_RESET        (0x10U)  /**< System reset */

/*============================================================================*/
/* TYPE DEFINITIONS                                                           */
/*============================================================================*/

/**
 * @brief Power management state
 */
typedef enum {
    PWR_STATE_ACTIVE     = 0x00U,  /**< Normal active operation */
    PWR_STATE_PREPARING  = 0x01U,  /**< Preparing for low power */
    PWR_STATE_LOW_POWER  = 0x02U,  /**< In low power mode */
    PWR_STATE_WAKING_UP  = 0x03U   /**< Waking up from low power */
} PowerState_t;

/**
 * @brief Low power configuration
 */
typedef struct {
    bool     enableCanWakeup;      /**< Enable CAN wake-up */
    bool     enableRtcWakeup;      /**< Enable RTC periodic wake-up */
    uint32_t rtcWakeupPeriod_ms;   /**< RTC wake-up period in ms */
    bool     disableWatchdog;      /**< Disable watchdog before sleep */
    uint32_t canWakeupId;          /**< CAN ID for wake-up (exact match) */
    uint32_t canWakeupMask;        /**< CAN ID mask (0x7FF = exact 11-bit) */
} PowerConfig_t;

/**
 * @brief Power management status
 */
typedef struct {
    PowerState_t state;            /**< Current power state */
    uint32_t     lowPowerEntries;  /**< Number of low power entries */
    uint32_t     wakeupCount;      /**< Number of wake-ups */
    uint32_t     lastWakeupSource; /**< Last wake-up source flags */
    uint32_t     totalSleepTime_ms;/**< Total time in low power */
    uint32_t     lastSleepDuration_ms; /**< Last sleep duration */
} PowerStatus_t;

/*============================================================================*/
/* FUNCTION PROTOTYPES                                                        */
/*============================================================================*/

/**
 * @brief Initialize power management module
 * @return STATUS_OK on success, error code otherwise
 */
Status_t PowerMgmt_Init(void);

/**
 * @brief Configure low power mode settings
 * @param[in] pConfig Pointer to configuration structure
 * @return STATUS_OK on success, error code otherwise
 */
Status_t PowerMgmt_Configure(const PowerConfig_t *pConfig);

/**
 * @brief Enter low power (STOP) mode
 * @details Configures CAN filter for exact wake-up message match,
 *          puts peripherals in low power state, and enters STOP mode.
 *          MCU will wake up ONLY when specific CAN message is received.
 * @return STATUS_OK on success, error code otherwise
 * @note This function blocks until wake-up occurs
 */
Status_t PowerMgmt_EnterLowPower(void);

/**
 * @brief Request transition to low power mode
 * @details Non-blocking version - sets flag for main loop to process
 * @return STATUS_OK on success
 */
Status_t PowerMgmt_RequestLowPower(void);

/**
 * @brief Check if low power mode is requested
 * @param[out] pRequested true if low power is requested
 * @return STATUS_OK on success
 */
Status_t PowerMgmt_IsLowPowerRequested(bool *pRequested);

/**
 * @brief Clear low power request flag
 * @return STATUS_OK on success
 */
Status_t PowerMgmt_ClearRequest(void);

/**
 * @brief Get current power state
 * @param[out] pState Pointer to store power state
 * @return STATUS_OK on success
 */
Status_t PowerMgmt_GetState(PowerState_t *pState);

/**
 * @brief Get power management status
 * @param[out] pStatus Pointer to store status
 * @return STATUS_OK on success
 */
Status_t PowerMgmt_GetStatus(PowerStatus_t *pStatus);

/**
 * @brief Get last wake-up source
 * @param[out] pSource Pointer to store wake-up source flags
 * @return STATUS_OK on success
 */
Status_t PowerMgmt_GetWakeupSource(uint32_t *pSource);

/**
 * @brief Check if system woke up from low power mode
 * @param[out] pWokeUp true if system woke from low power
 * @return STATUS_OK on success
 */
Status_t PowerMgmt_DidWakeFromLowPower(bool *pWokeUp);

/**
 * @brief Configure CAN wake-up filter for exact message match
 * @param[in] canId CAN message ID to wake up on
 * @param[in] requireMagic If true, also verify magic bytes in data
 * @return STATUS_OK on success
 */
Status_t PowerMgmt_ConfigureCANWakeup(uint32_t canId, bool requireMagic);

/**
 * @brief Restore system after wake-up
 * @details Reconfigures clocks, peripherals, and resumes normal operation
 * @return STATUS_OK on success
 */
Status_t PowerMgmt_RestoreAfterWakeup(void);

/**
 * @brief Prepare peripherals for low power mode
 * @return STATUS_OK on success
 */
Status_t PowerMgmt_PreparePeripherals(void);

/**
 * @brief Callback for CAN wake-up message received
 * @details Called from CAN RX interrupt when wake-up message received
 * @param[in] pData Pointer to received CAN message data
 * @param[in] length Data length
 */
void PowerMgmt_CANWakeupCallback(const uint8_t *pData, uint8_t length);

#ifdef __cplusplus
}
#endif

#endif /* POWER_MGMT_H */

/*============================================================================*/
/* END OF FILE                                                                */
/*============================================================================*/
