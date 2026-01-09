/**
 * @file    pm_monitor.h
 * @brief   Power supply monitoring module
 * @author  Battery Control Unit Development Team
 * @date    2026-01-09
 * @version 1.0.0
 *
 * @note    MISRA C:2012 compliant
 * @note    ISO 26262 ASIL-B safety requirements
 * @note    Monitors LM74900, power rails, and fault conditions
 *
 * @copyright Copyright (c) 2026
 */

#ifndef PM_MONITOR_H
#define PM_MONITOR_H

#ifdef __cplusplus
extern "C" {
#endif

/*============================================================================*/
/* INCLUDES                                                                   */
/*============================================================================*/
#include "app_types.h"
#include "app_config.h"

/*============================================================================*/
/* CONSTANTS                                                                  */
/*============================================================================*/

/** @brief Power rail identifiers */
#define PM_RAIL_INPUT_12V     (0U)  /**< 12V input */
#define PM_RAIL_5V            (1U)  /**< 5V rail */
#define PM_RAIL_3V3_DIGITAL   (2U)  /**< 3.3V digital */
#define PM_RAIL_3V3_ANALOG    (3U)  /**< 3.3V analog */
#define PM_RAIL_COUNT         (4U)  /**< Total power rails */

/** @brief Monitoring periods */
#define PM_MONITOR_PERIOD_MS  (10U)  /**< Monitor every 10ms */
#define PM_FAULT_DEBOUNCE_MS  (50U)  /**< Debounce faults for 50ms */

/*============================================================================*/
/* TYPE DEFINITIONS                                                           */
/*============================================================================*/

/**
 * @brief Power rail status
 */
typedef enum {
    PM_RAIL_STATUS_OFF         = 0x00U,  /**< Rail powered off */
    PM_RAIL_STATUS_STARTING    = 0x01U,  /**< Rail starting up */
    PM_RAIL_STATUS_OK          = 0x02U,  /**< Rail voltage OK */
    PM_RAIL_STATUS_UNDERVOLTAGE = 0x03U, /**< Undervoltage detected */
    PM_RAIL_STATUS_OVERVOLTAGE = 0x04U,  /**< Overvoltage detected */
    PM_RAIL_STATUS_FAULT       = 0x05U   /**< Fault condition */
} PM_RailStatus_t;

/**
 * @brief LM74900 status
 */
typedef struct {
    bool     enabled;           /**< LM74900 enabled */
    bool     faultDetected;     /**< Fault signal active */
    bool     reversePolarity;   /**< Reverse polarity detected */
    uint32_t currentMonitor_mA; /**< Current monitor (IMON) reading */
    uint32_t inputVoltage_mV;   /**< Input voltage */
} PM_LM74900_Status_t;

/**
 * @brief Power rail information
 */
typedef struct {
    PM_RailStatus_t status;          /**< Current status */
    Voltage_mV_t    voltage_mV;      /**< Measured voltage */
    Voltage_mV_t    nominalVoltage_mV; /**< Nominal voltage */
    Voltage_mV_t    minVoltage_mV;   /**< Minimum threshold */
    Voltage_mV_t    maxVoltage_mV;   /**< Maximum threshold */
    bool            powerGood;       /**< Power good signal */
    uint32_t        faultCount;      /**< Fault occurrence count */
    uint32_t        lastFaultTime_ms;/**< Last fault timestamp */
} PM_RailInfo_t;

/**
 * @brief Power monitoring data structure
 */
typedef struct {
    PM_LM74900_Status_t lm74900;                /**< LM74900 status */
    PM_RailInfo_t       rails[PM_RAIL_COUNT];   /**< Power rail info */
    bool                systemPowerGood;        /**< Overall system status */
    uint32_t            totalFaults;            /**< Total fault count */
    uint32_t            uptimeSeconds;          /**< System uptime */
} PM_MonitorData_t;

/**
 * @brief Power fault callback function
 */
typedef void (*PM_FaultCallback_t)(uint8_t railId, PM_RailStatus_t status);

/*============================================================================*/
/* FUNCTION PROTOTYPES                                                        */
/*============================================================================*/

/**
 * @brief Initialize power monitoring module
 * @return STATUS_OK on success, error code otherwise
 * @req BCU_REQ_PWR_001 Power monitoring initialization
 */
Status_t PM_Monitor_Init(void);

/**
 * @brief De-initialize power monitoring module
 * @return STATUS_OK on success
 */
Status_t PM_Monitor_DeInit(void);

/**
 * @brief Update power monitoring (call periodically)
 * @return STATUS_OK if all rails OK, error code otherwise
 * @note  Should be called every 10ms from main loop or task
 * @req BCU_REQ_PWR_010 Periodic power monitoring
 */
Status_t PM_Monitor_Update(void);

/**
 * @brief Get LM74900 status
 * @param[out] pStatus Pointer to store status
 * @return STATUS_OK on success
 */
Status_t PM_Monitor_GetLM74900Status(PM_LM74900_Status_t *pStatus);

/**
 * @brief Get power rail status
 * @param[in]  railId   Rail identifier (0-3)
 * @param[out] pStatus  Pointer to store rail status
 * @return STATUS_OK on success, error code otherwise
 */
Status_t PM_Monitor_GetRailStatus(uint8_t railId, PM_RailStatus_t *pStatus);

/**
 * @brief Get power rail voltage
 * @param[in]  railId      Rail identifier (0-3)
 * @param[out] pVoltage_mV Pointer to store voltage in millivolts
 * @return STATUS_OK on success, error code otherwise
 */
Status_t PM_Monitor_GetRailVoltage(uint8_t railId, Voltage_mV_t *pVoltage_mV);

/**
 * @brief Get power rail information (detailed)
 * @param[in]  railId  Rail identifier (0-3)
 * @param[out] pInfo   Pointer to store rail information
 * @return STATUS_OK on success, error code otherwise
 */
Status_t PM_Monitor_GetRailInfo(uint8_t railId, PM_RailInfo_t *pInfo);

/**
 * @brief Get complete monitoring data
 * @param[out] pData Pointer to store monitoring data
 * @return STATUS_OK on success
 */
Status_t PM_Monitor_GetData(PM_MonitorData_t *pData);

/**
 * @brief Check if system power is good
 * @param[out] pPowerGood true if all rails OK, false otherwise
 * @return STATUS_OK on success
 */
Status_t PM_Monitor_IsSystemPowerGood(bool *pPowerGood);

/**
 * @brief Check if specific rail is in specification
 * @param[in]  railId  Rail identifier (0-3)
 * @param[out] pInSpec true if rail within spec, false otherwise
 * @return STATUS_OK on success, error code otherwise
 */
Status_t PM_Monitor_IsRailInSpec(uint8_t railId, bool *pInSpec);

/**
 * @brief Get input current (from IMON)
 * @param[out] pCurrent_mA Pointer to store current in milliamps
 * @return STATUS_OK on success
 */
Status_t PM_Monitor_GetInputCurrent(Current_mA_t *pCurrent_mA);

/**
 * @brief Check if LM74900 fault is active
 * @param[out] pFault true if fault detected, false otherwise
 * @return STATUS_OK on success
 */
Status_t PM_Monitor_IsLM74900Fault(bool *pFault);

/**
 * @brief Clear fault counters
 * @param[in] railId Rail identifier (0-3), or 0xFF for all
 * @return STATUS_OK on success
 */
Status_t PM_Monitor_ClearFaultCounters(uint8_t railId);

/**
 * @brief Register power fault callback
 * @param[in] callback Callback function pointer
 * @return STATUS_OK on success
 * @note  Callback is called when fault is detected
 */
Status_t PM_Monitor_RegisterFaultCallback(PM_FaultCallback_t callback);

/**
 * @brief Get system uptime
 * @param[out] pUptimeSeconds Pointer to store uptime in seconds
 * @return STATUS_OK on success
 */
Status_t PM_Monitor_GetUptime(uint32_t *pUptimeSeconds);

/**
 * @brief Force immediate power rail measurement
 * @param[in] railId Rail identifier (0-3)
 * @return STATUS_OK on success, error code otherwise
 * @note  Useful for testing or diagnostics
 */
Status_t PM_Monitor_ForceMeasurement(uint8_t railId);

/**
 * @brief Enable/disable power rail monitoring
 * @param[in] railId Rail identifier (0-3)
 * @param[in] enable true to enable, false to disable
 * @return STATUS_OK on success, error code otherwise
 */
Status_t PM_Monitor_EnableRailMonitoring(uint8_t railId, bool enable);

/**
 * @brief Set custom voltage thresholds for rail
 * @param[in] railId       Rail identifier (0-3)
 * @param[in] minVoltage_mV Minimum voltage threshold
 * @param[in] maxVoltage_mV Maximum voltage threshold
 * @return STATUS_OK on success, error code otherwise
 */
Status_t PM_Monitor_SetRailThresholds(uint8_t railId,
                                      Voltage_mV_t minVoltage_mV,
                                      Voltage_mV_t maxVoltage_mV);

#ifdef __cplusplus
}
#endif

#endif /* PM_MONITOR_H */
