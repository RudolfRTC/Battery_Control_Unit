/**
 * @file    safety_monitor.h
 * @brief   Safety monitoring module for ISO 26262 ASIL-B compliance
 * @author  Battery Control Unit Development Team
 * @date    2026-01-09
 * @version 1.0.0
 *
 * @note    MISRA C:2012 compliant
 * @note    ISO 26262 ASIL-B safety requirements
 * @note    Comprehensive system health monitoring
 *
 * @copyright Copyright (c) 2026
 */

#ifndef SAFETY_MONITOR_H
#define SAFETY_MONITOR_H

#ifdef __cplusplus
extern "C" {
#endif

/*============================================================================*/
/* INCLUDES                                                                   */
/*============================================================================*/
#include "app_types.h"
#include "app_errors.h"
#include <stdint.h>
#include <stdbool.h>

/*============================================================================*/
/* CONSTANTS AND MACROS                                                       */
/*============================================================================*/

/** @brief Maximum number of safety checks */
#define SAFETY_MAX_CHECKS               (10U)

/** @brief Safety check execution interval (milliseconds) */
#define SAFETY_CHECK_INTERVAL_MS        (10U)

/*============================================================================*/
/* TYPE DEFINITIONS                                                           */
/*============================================================================*/

/** @brief Safety level enumeration */
typedef enum {
    SAFETY_LEVEL_LOW = 0x00U,       /**< Low criticality */
    SAFETY_LEVEL_MEDIUM = 0x01U,    /**< Medium criticality */
    SAFETY_LEVEL_HIGH = 0x02U,      /**< High criticality */
    SAFETY_LEVEL_CRITICAL = 0x03U   /**< Critical - immediate safe state */
} SafetyLevel_t;

/** @brief Safety monitor state */
typedef enum {
    SAFETY_STATE_UNINITIALIZED = 0x00U, /**< Not initialized */
    SAFETY_STATE_NORMAL = 0x01U,        /**< Normal operation */
    SAFETY_STATE_WARNING = 0x02U,       /**< Warning - non-critical issue */
    SAFETY_STATE_DEGRADED = 0x03U,      /**< Degraded operation */
    SAFETY_STATE_SAFE = 0x04U,          /**< Safe state - operation halted */
    SAFETY_STATE_FAULT = 0x05U          /**< Fault detected */
} SafetyMonitor_State_t;

/** @brief Safety monitor configuration */
typedef struct {
    bool watchdogCheckEnable;       /**< Enable watchdog monitoring */
    bool timingCheckEnable;         /**< Enable timing violation check */
    bool stackCheckEnable;          /**< Enable stack overflow check */
    bool ramCheckEnable;            /**< Enable RAM integrity check */
    bool powerCheckEnable;          /**< Enable power supply monitoring */
    bool tempCheckEnable;           /**< Enable temperature monitoring */
    bool commCheckEnable;           /**< Enable communication monitoring */
    uint32_t maxLoopTime_us;        /**< Maximum allowed loop time (microseconds) */
    uint32_t safeStateTimeout_ms;   /**< Timeout before entering safe state */
} SafetyMonitor_Config_t;

/** @brief Safety statistics structure */
typedef struct {
    SafetyMonitor_State_t currentState;  /**< Current safety state */
    uint32_t watchdogViolations;         /**< Watchdog violation count */
    uint32_t timingViolations;           /**< Timing violation count */
    uint32_t stackViolations;            /**< Stack overflow count */
    uint32_t ramViolations;              /**< RAM integrity error count */
    uint32_t powerViolations;            /**< Power supply fault count */
    uint32_t tempViolations;             /**< Temperature fault count */
    uint32_t commViolations;             /**< Communication fault count */
    uint32_t totalViolations;            /**< Total violation count */
    uint32_t safeStateEntryCount;        /**< Number of safe state entries */
    uint32_t lastLoopTime_us;            /**< Last main loop execution time */
    uint32_t maxLoopTime_us;             /**< Maximum loop time recorded */
    uint32_t lastViolationTime_ms;       /**< Timestamp of last violation */
} SafetyMonitor_Statistics_t;

/** @brief Safety fault callback function type */
typedef void (*SafetyMonitor_FaultCallback_t)(SafetyLevel_t level, ErrorCode_t errorCode);

/*============================================================================*/
/* FUNCTION PROTOTYPES                                                        */
/*============================================================================*/

/**
 * @brief Initialize safety monitoring module
 * @return STATUS_OK on success, error code otherwise
 */
Status_t SafetyMonitor_Init(void);

/**
 * @brief Execute safety monitoring checks
 * @return STATUS_OK on success, error code otherwise
 * @note Should be called periodically from main loop
 */
Status_t SafetyMonitor_Execute(void);

/**
 * @brief Get current safety monitor state
 * @param[out] pState Pointer to store current state
 * @return STATUS_OK on success, error code otherwise
 */
Status_t SafetyMonitor_GetState(SafetyMonitor_State_t *pState);

/**
 * @brief Check if system is in safe operating state
 * @return true if safe, false if faults detected
 */
bool SafetyMonitor_IsSafe(void);

/**
 * @brief Update timing statistics
 * @param[in] loopTime_us Main loop execution time in microseconds
 * @return STATUS_OK on success, error code otherwise
 */
Status_t SafetyMonitor_UpdateTiming(uint32_t loopTime_us);

/**
 * @brief Register safety fault callback
 * @param[in] callback Callback function pointer (NULL to disable)
 * @return STATUS_OK on success, error code otherwise
 */
Status_t SafetyMonitor_RegisterFaultCallback(SafetyMonitor_FaultCallback_t callback);

/**
 * @brief Configure safety monitor
 * @param[in] pConfig Pointer to configuration structure
 * @return STATUS_OK on success, error code otherwise
 */
Status_t SafetyMonitor_Configure(const SafetyMonitor_Config_t *pConfig);

/**
 * @brief Reset safety statistics
 * @return STATUS_OK on success, error code otherwise
 */
Status_t SafetyMonitor_ResetStatistics(void);

/**
 * @brief Force system into safe state
 * @return STATUS_OK on success, error code otherwise
 * @note This function immediately halts operation
 */
Status_t SafetyMonitor_ForceSafeState(void);

/**
 * @brief Get safety statistics
 * @param[out] pStats Pointer to store statistics structure
 * @return STATUS_OK on success, error code otherwise
 */
Status_t SafetyMonitor_GetStatistics(SafetyMonitor_Statistics_t *pStats);

/**
 * @brief De-initialize safety monitoring module
 * @return STATUS_OK on success, error code otherwise
 */
Status_t SafetyMonitor_DeInit(void);

#ifdef __cplusplus
}
#endif

#endif /* SAFETY_MONITOR_H */
