/**
 * @file    app_main.h
 * @brief   Main application header
 * @author  Battery Control Unit Development Team
 * @date    2026-01-09
 * @version 1.0.0
 *
 * @note    MISRA C:2012 compliant
 * @note    ISO 26262 ASIL-B safety requirements
 *
 * @copyright Copyright (c) 2026
 */

#ifndef APP_MAIN_H
#define APP_MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/*============================================================================*/
/* INCLUDES                                                                   */
/*============================================================================*/
#include "app_types.h"
#include "app_config.h"

/*============================================================================*/
/* TYPE DEFINITIONS                                                           */
/*============================================================================*/

/**
 * @brief Application state machine states
 */
typedef enum {
    APP_STATE_INIT         = 0x00U,  /**< Initialization */
    APP_STATE_STARTUP      = 0x01U,  /**< Startup sequence */
    APP_STATE_POWER_ON     = 0x02U,  /**< Powering on systems */
    APP_STATE_RUNNING      = 0x03U,  /**< Normal operation */
    APP_STATE_IDLE         = 0x04U,  /**< Idle mode */
    APP_STATE_SHUTDOWN     = 0x05U,  /**< Shutdown sequence */
    APP_STATE_ERROR        = 0x06U,  /**< Error state */
    APP_STATE_SAFE         = 0x07U   /**< Safe state (all off) */
} AppState_t;

/**
 * @brief Application system status
 */
typedef struct {
    AppState_t  state;             /**< Current state */
    uint32_t    uptime_ms;         /**< Uptime in milliseconds */
    uint32_t    cycleCount;        /**< Main loop cycle count */
    uint32_t    cycleTime_us;      /**< Last cycle time in microseconds */
    uint32_t    maxCycleTime_us;   /**< Maximum cycle time */
    bool        powerGood;         /**< System power good */
    bool        safetyOK;          /**< Safety checks passed */
    uint16_t    activeErrors;      /**< Active error count */
    uint16_t    activeWarnings;    /**< Active warning count */
} AppStatus_t;

/*============================================================================*/
/* FUNCTION PROTOTYPES                                                        */
/*============================================================================*/

/**
 * @brief Initialize application
 * @return STATUS_OK on success, error code otherwise
 */
Status_t App_Init(void);

/**
 * @brief Application main loop
 * @note  This function contains the main state machine
 */
void App_MainLoop(void);

/**
 * @brief Execute fast tasks (1ms period)
 * @note  Safety-critical tasks
 */
void App_FastTasks(void);

/**
 * @brief Execute medium tasks (10ms period)
 * @note  Control and I/O tasks
 */
void App_MediumTasks(void);

/**
 * @brief Execute slow tasks (100ms period)
 * @note  Communication and diagnostics
 */
void App_SlowTasks(void);

/**
 * @brief Get application state
 * @param[out] pState Pointer to store current state
 * @return STATUS_OK on success
 */
Status_t App_GetState(AppState_t *pState);

/**
 * @brief Get application status
 * @param[out] pStatus Pointer to store status
 * @return STATUS_OK on success
 */
Status_t App_GetStatus(AppStatus_t *pStatus);

/**
 * @brief Request state transition
 * @param[in] newState Requested state
 * @return STATUS_OK on success, error code if invalid transition
 */
Status_t App_RequestStateChange(AppState_t newState);

/**
 * @brief Trigger application shutdown
 * @return STATUS_OK on success
 */
Status_t App_Shutdown(void);

/**
 * @brief Enter safe state (emergency)
 * @note  Disables all outputs, sets safe configuration
 */
void App_EnterSafeState(void);

/**
 * @brief Get application version
 * @param[out] pVersion Pointer to version structure
 * @return STATUS_OK on success
 */
Status_t App_GetVersion(Version_t *pVersion);

#ifdef __cplusplus
}
#endif

#endif /* APP_MAIN_H */
