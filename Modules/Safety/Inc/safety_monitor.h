/**
 * @file    safety_monitor.h
 * @brief   Safety monitoring module for ISO 26262 ASIL-B compliance
 * @author  Battery Control Unit Development Team
 * @date    2026-01-10
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
#include <stdint.h>
#include <stdbool.h>

/*============================================================================*/
/* CONSTANTS                                                                  */
/*============================================================================*/

/** @brief STM32F413 RAM memory layout */
#define MCU_RAM_START     (0x20000000U)  /**< RAM start address */
#define MCU_RAM_END       (0x20050000U)  /**< RAM end address (320KB) */
#define MCU_RAM_SIZE      (0x00050000U)  /**< Total RAM size (320KB) */

/** @brief Maximum violation count before fault */
#define SAFETY_MAX_VIOLATIONS     (10U)

/*============================================================================*/
/* TYPE DEFINITIONS                                                           */
/*============================================================================*/

/**
 * @brief Safety system state
 */
typedef enum {
    SAFETY_STATE_INIT            = 0x00U,  /**< Initialization state */
    SAFETY_STATE_OK              = 0x01U,  /**< System OK */
    SAFETY_STATE_WARNING         = 0x02U,  /**< Warning level */
    SAFETY_STATE_FAULT_MEDIUM    = 0x03U,  /**< Medium fault */
    SAFETY_STATE_FAULT_HIGH      = 0x04U,  /**< High severity fault */
    SAFETY_STATE_FAULT_CRITICAL  = 0x05U,  /**< Critical fault */
    SAFETY_STATE_SAFE            = 0x06U   /**< Safe state (all outputs off) */
} SafetyState_t;

/**
 * @brief Safety level (fault severity)
 */
typedef enum {
    SAFETY_LEVEL_LOW      = 0x00U,  /**< Low severity */
    SAFETY_LEVEL_MEDIUM   = 0x01U,  /**< Medium severity */
    SAFETY_LEVEL_HIGH     = 0x02U,  /**< High severity */
    SAFETY_LEVEL_CRITICAL = 0x03U   /**< Critical severity */
} SafetyLevel_t;

/**
 * @brief Safety monitor state structure
 */
typedef struct {
    SafetyState_t  systemState;         /**< Current system state */
    SafetyLevel_t  safetyLevel;         /**< Current safety level */
    uint32_t       criticalFaults;      /**< Critical fault count */
    uint32_t       highFaults;          /**< High fault count */
    uint32_t       mediumFaults;        /**< Medium fault count */
    uint32_t       lowFaults;           /**< Low fault count */
    uint32_t       totalViolations;     /**< Total violation count */
    uint32_t       checkCycles;         /**< Number of check cycles */
    uint32_t       lastCheckTime_ms;    /**< Last check timestamp */
    uint32_t       lastLoopTime_us;     /**< Last loop time in microseconds */
    uint32_t       maxLoopTime_us;      /**< Maximum loop time recorded */
    bool           safeStateActive;     /**< Safe state active flag */
} SafetyMonitor_State_t;

/**
 * @brief Safety monitor configuration
 */
typedef struct {
    bool     enableWatchdogCheck;    /**< Enable watchdog check */
    bool     enableTimingCheck;      /**< Enable timing check */
    bool     enableStackCheck;       /**< Enable stack overflow check */
    bool     enableRAMCheck;         /**< Enable RAM integrity check */
    bool     enablePowerCheck;       /**< Enable power supply check */
    bool     enableTempCheck;        /**< Enable temperature check */
    bool     enableCommCheck;        /**< Enable communication check */
    uint32_t maxLoopTime_us;         /**< Maximum allowed loop time (us) */
    uint32_t stackWatermarkValue;    /**< Stack watermark pattern */
} SafetyMonitor_Config_t;

/**
 * @brief Safety fault callback function type
 * @param[in] faultLevel  Fault severity level
 * @param[in] faultSource Fault source identifier (string)
 */
typedef void (*SafetyMonitor_FaultCallback_t)(SafetyLevel_t faultLevel, const char *faultSource);

/*============================================================================*/
/* FUNCTION PROTOTYPES                                                        */
/*============================================================================*/

/**
 * @brief Initialize safety monitor
 * @return STATUS_OK on success, error code otherwise
 */
Status_t SafetyMonitor_Init(void);

/**
 * @brief De-initialize safety monitor
 * @return STATUS_OK on success
 */
Status_t SafetyMonitor_DeInit(void);

/**
 * @brief Execute safety checks (call every 1ms in main loop)
 * @return STATUS_OK on success, error code otherwise
 */
Status_t SafetyMonitor_Execute(void);

/**
 * @brief Get current safety monitor state
 * @param[out] pState Pointer to store state
 * @return STATUS_OK on success, error code otherwise
 */
Status_t SafetyMonitor_GetState(SafetyMonitor_State_t *pState);

/**
 * @brief Check if system is in safe state
 * @return true if safe, false otherwise
 */
bool SafetyMonitor_IsSafe(void);

/**
 * @brief Update timing measurement
 * @param[in] loopTime_us Loop execution time in microseconds
 * @return STATUS_OK on success
 */
Status_t SafetyMonitor_UpdateTiming(uint32_t loopTime_us);

/**
 * @brief Register fault callback function
 * @param[in] callback Callback function pointer
 * @return STATUS_OK on success
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
 * @return STATUS_OK on success
 */
Status_t SafetyMonitor_ResetStatistics(void);

/**
 * @brief Force system into safe state
 * @return STATUS_OK on success
 */
Status_t SafetyMonitor_ForceSafeState(void);

#ifdef __cplusplus
}
#endif

#endif /* SAFETY_MONITOR_H */
