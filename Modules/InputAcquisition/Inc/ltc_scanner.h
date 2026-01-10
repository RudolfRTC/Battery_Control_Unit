/**
 * @file    ltc_scanner.h
 * @brief   Non-blocking LTC6811 cell scanner with adaptive period
 * @author  Battery Control Unit Development Team
 * @date    2026-01-10
 * @version 1.0.0
 *
 * @note    MISRA C:2012 compliant
 * @note    State machine implementation for 48 LTC modules via LTC6820
 * @note    Adaptive scanning period based on operating conditions
 *
 * @copyright Copyright (c) 2026
 */

#ifndef LTC_SCANNER_H
#define LTC_SCANNER_H

#ifdef __cplusplus
extern "C" {
#endif

/*============================================================================*/
/* INCLUDES                                                                   */
/*============================================================================*/
#include "app_types.h"
#include "ltc6811.h"
#include "bcu_timing_config.h"
#include <stdint.h>
#include <stdbool.h>

/*============================================================================*/
/* CONSTANTS                                                                  */
/*============================================================================*/

/** @brief Maximum number of LTC6811 devices (48 modules) */
#define LTC_SCANNER_MAX_DEVICES         (48U)

/** @brief Number of cell voltage groups (A, B, C, D) */
#define LTC_SCANNER_CELL_GROUPS         (4U)

/** @brief Maximum number of cells per module */
#define LTC_SCANNER_MAX_CELLS_PER_MOD   (12U)

/*============================================================================*/
/* TYPE DEFINITIONS                                                           */
/*============================================================================*/

/**
 * @brief LTC scanner state machine states
 */
typedef enum {
    LTC_SM_IDLE             = 0x00U,  /**< Idle state */
    LTC_SM_START_CONVERSION = 0x01U,  /**< Start ADC conversion */
    LTC_SM_WAIT_CONVERSION  = 0x02U,  /**< Wait for conversion complete */
    LTC_SM_READ_GROUP_A     = 0x03U,  /**< Read cell group A (cells 1-3) */
    LTC_SM_READ_GROUP_B     = 0x04U,  /**< Read cell group B (cells 4-6) */
    LTC_SM_READ_GROUP_C     = 0x05U,  /**< Read cell group C (cells 7-9) */
    LTC_SM_READ_GROUP_D     = 0x06U,  /**< Read cell group D (cells 10-12) */
    LTC_SM_VALIDATE_CRC     = 0x07U,  /**< Validate PEC/CRC */
    LTC_SM_PUBLISH          = 0x08U,  /**< Publish results */
    LTC_SM_ERROR            = 0x09U   /**< Error state */
} LTCScanner_State_t;

/**
 * @brief LTC scanner operating mode
 */
typedef enum {
    LTC_MODE_NORMAL     = 0x00U,  /**< Normal mode - low update rate */
    LTC_MODE_ACTIVE     = 0x01U,  /**< Active mode - high current/charging */
    LTC_MODE_TRANSIENT  = 0x02U   /**< Transient mode - precharge/switching */
} LTCScanner_Mode_t;

/**
 * @brief LTC scanner configuration
 */
typedef struct {
    uint8_t             num_devices;            /**< Number of LTC6811 devices */
    float               current_threshold_A;    /**< Current threshold for ACTIVE mode */
    float               precharge_voltage_V;    /**< Precharge detection voltage */
    uint32_t            transient_hold_ms;      /**< Transient mode hold time */
    bool                enable_balancing;       /**< Enable cell balancing */
} LTCScanner_Config_t;

/**
 * @brief LTC scanner statistics
 */
typedef struct {
    uint32_t            scan_count;             /**< Total scans completed */
    uint32_t            error_count;            /**< Total errors */
    uint32_t            crc_errors;             /**< CRC/PEC errors */
    uint32_t            timeout_errors;         /**< Timeout errors */
    uint32_t            consecutive_errors;     /**< Consecutive errors */
    uint32_t            last_scan_time_ms;      /**< Last scan duration */
    uint32_t            max_scan_time_ms;       /**< Maximum scan duration */
    LTCScanner_State_t  current_state;          /**< Current state machine state */
    LTCScanner_Mode_t   current_mode;           /**< Current operating mode */
} LTCScanner_Stats_t;

/**
 * @brief LTC cell data structure (published results)
 */
typedef struct {
    uint16_t            cell_voltages_mV[LTC_SCANNER_MAX_DEVICES][LTC_SCANNER_MAX_CELLS_PER_MOD];
    uint8_t             cell_count[LTC_SCANNER_MAX_DEVICES];
    bool                data_valid[LTC_SCANNER_MAX_DEVICES];
    uint32_t            timestamp_ms;
    bool                new_data_available;
} LTCScanner_CellData_t;

/**
 * @brief Event callback function type
 * @param[in] event Event code
 * @param[in] param Event parameter
 */
typedef void (*LTCScanner_EventCallback_t)(uint8_t event, uint32_t param);

/*============================================================================*/
/* FUNCTION PROTOTYPES                                                        */
/*============================================================================*/

/**
 * @brief Initialize LTC scanner
 * @param[in] pConfig Pointer to configuration structure
 * @return STATUS_OK on success, error code otherwise
 */
Status_t LTCScanner_Init(const LTCScanner_Config_t *pConfig);

/**
 * @brief De-initialize LTC scanner
 * @return STATUS_OK on success
 */
Status_t LTCScanner_DeInit(void);

/**
 * @brief Execute one step of state machine (non-blocking)
 * @param[in] now_ms Current time in milliseconds
 * @return STATUS_OK on success, error code otherwise
 * @note Call this from scheduler - executes quickly (<500µs)
 */
Status_t LTCScanner_Task_Step(uint32_t now_ms);

/**
 * @brief Set scanner operating mode
 * @param[in] mode Operating mode
 * @return STATUS_OK on success
 */
Status_t LTCScanner_SetMode(LTCScanner_Mode_t mode);

/**
 * @brief Trigger transient mode (e.g., on contactor event)
 * @param[in] hold_ms Duration to hold transient mode (0 = use default)
 * @return STATUS_OK on success
 */
Status_t LTCScanner_TriggerTransient(uint32_t hold_ms);

/**
 * @brief Get current cell data
 * @param[out] pData Pointer to cell data structure
 * @return STATUS_OK on success
 */
Status_t LTCScanner_GetCellData(LTCScanner_CellData_t *pData);

/**
 * @brief Get scanner statistics
 * @param[out] pStats Pointer to statistics structure
 * @return STATUS_OK on success
 */
Status_t LTCScanner_GetStats(LTCScanner_Stats_t *pStats);

/**
 * @brief Register event callback
 * @param[in] callback Callback function pointer
 * @return STATUS_OK on success
 */
Status_t LTCScanner_RegisterCallback(LTCScanner_EventCallback_t callback);

/**
 * @brief Force immediate scan
 * @return STATUS_OK on success
 */
Status_t LTCScanner_ForceScan(void);

/**
 * @brief Update pack current (for mode selection)
 * @param[in] pack_current_A Pack current in amperes (signed)
 * @return STATUS_OK on success
 */
Status_t LTCScanner_UpdatePackCurrent(float pack_current_A);

/**
 * @brief Set charging mode flag
 * @param[in] charging true if charging, false otherwise
 * @return STATUS_OK on success
 */
Status_t LTCScanner_SetChargingMode(bool charging);

#ifdef __cplusplus
}
#endif

#endif /* LTC_SCANNER_H */
