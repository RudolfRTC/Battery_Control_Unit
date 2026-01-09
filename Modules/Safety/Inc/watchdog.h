/**
 * @file    watchdog.h
 * @brief   Dual watchdog (IWDG + WWDG) safety monitor
 * @author  Battery Control Unit Development Team
 * @date    2026-01-09
 * @version 1.0.0
 *
 * @note    MISRA C:2012 compliant
 * @note    ISO 26262 ASIL-B safety requirements
 * @note    Independent watchdog (IWDG) + Window watchdog (WWDG)
 *
 * @copyright Copyright (c) 2026
 */

#ifndef WATCHDOG_H
#define WATCHDOG_H

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

/** @brief Watchdog timeouts */
#define WDG_IWDG_TIMEOUT_MS   (500U)   /**< IWDG timeout: 500ms */
#define WDG_WWDG_TIMEOUT_MS   (100U)   /**< WWDG timeout: 100ms */
#define WDG_REFRESH_PERIOD_MS (50U)    /**< Refresh every 50ms */

/** @brief Window watchdog timing */
#define WDG_WWDG_WINDOW_MIN_MS (30U)   /**< Min refresh time: 30ms */
#define WDG_WWDG_WINDOW_MAX_MS (100U)  /**< Max refresh time: 100ms */

/*============================================================================*/
/* TYPE DEFINITIONS                                                           */
/*============================================================================*/

/**
 * @brief Watchdog type
 */
typedef enum {
    WDG_TYPE_IWDG = 0x00U,  /**< Independent watchdog */
    WDG_TYPE_WWDG = 0x01U   /**< Window watchdog */
} WDG_Type_t;

/**
 * @brief Watchdog status
 */
typedef struct {
    bool     iwdgEnabled;        /**< IWDG enabled */
    bool     wwdgEnabled;        /**< WWDG enabled */
    uint32_t iwdgRefreshCount;   /**< IWDG refresh count */
    uint32_t wwdgRefreshCount;   /**< WWDG refresh count */
    uint32_t missedRefreshCount; /**< Missed refresh count */
    uint32_t lastRefreshTime_ms; /**< Last refresh timestamp */
} WDG_Status_t;

/**
 * @brief Reset cause enumeration
 */
typedef enum {
    RESET_CAUSE_UNKNOWN     = 0x00U,  /**< Unknown */
    RESET_CAUSE_POR         = 0x01U,  /**< Power-on reset */
    RESET_CAUSE_PIN         = 0x02U,  /**< External pin reset */
    RESET_CAUSE_SOFTWARE    = 0x03U,  /**< Software reset */
    RESET_CAUSE_IWDG        = 0x04U,  /**< Independent watchdog */
    RESET_CAUSE_WWDG        = 0x05U,  /**< Window watchdog */
    RESET_CAUSE_LOW_POWER   = 0x06U,  /**< Low-power reset */
    RESET_CAUSE_BROWNOUT    = 0x07U   /**< Brown-out reset */
} ResetCause_t;

/*============================================================================*/
/* FUNCTION PROTOTYPES                                                        */
/*============================================================================*/

/**
 * @brief Initialize watchdog module
 * @return STATUS_OK on success, error code otherwise
 * @req BCU_REQ_SAFE_001 Watchdog initialization
 */
Status_t Watchdog_Init(void);

/**
 * @brief Start independent watchdog (IWDG)
 * @return STATUS_OK on success, error code otherwise
 * @note  IWDG cannot be stopped once started
 */
Status_t Watchdog_StartIWDG(void);

/**
 * @brief Start window watchdog (WWDG)
 * @return STATUS_OK on success, error code otherwise
 */
Status_t Watchdog_StartWWDG(void);

/**
 * @brief Refresh independent watchdog
 * @return STATUS_OK on success, error code otherwise
 * @note  Must be called within timeout period
 * @req BCU_REQ_SAFE_010 Watchdog refresh
 */
Status_t Watchdog_RefreshIWDG(void);

/**
 * @brief Refresh window watchdog
 * @return STATUS_OK on success, error code otherwise
 * @note  Must be called within window period
 */
Status_t Watchdog_RefreshWWDG(void);

/**
 * @brief Refresh all watchdogs
 * @return STATUS_OK on success
 * @note  Convenience function for refreshing both watchdogs
 */
Status_t Watchdog_RefreshAll(void);

/**
 * @brief Get watchdog status
 * @param[out] pStatus Pointer to status structure
 * @return STATUS_OK on success
 */
Status_t Watchdog_GetStatus(WDG_Status_t *pStatus);

/**
 * @brief Get last reset cause
 * @param[out] pCause Pointer to store reset cause
 * @return STATUS_OK on success
 * @req BCU_REQ_SAFE_020 Reset cause detection
 */
Status_t Watchdog_GetResetCause(ResetCause_t *pCause);

/**
 * @brief Clear reset cause flags
 * @return STATUS_OK on success
 */
Status_t Watchdog_ClearResetFlags(void);

/**
 * @brief Check if reset was caused by watchdog
 * @param[out] pWatchdogReset true if watchdog reset, false otherwise
 * @return STATUS_OK on success
 */
Status_t Watchdog_WasWatchdogReset(bool *pWatchdogReset);

/**
 * @brief Get reset cause string
 * @param[in] cause Reset cause
 * @return Pointer to constant string
 */
const char* Watchdog_GetResetCauseString(ResetCause_t cause);

/**
 * @brief Force system reset
 * @note  This function does not return
 */
void Watchdog_ForceReset(void) __attribute__((noreturn));

/**
 * @brief Check if watchdog refresh is needed
 * @param[out] pNeedRefresh true if refresh needed, false otherwise
 * @return STATUS_OK on success
 */
Status_t Watchdog_NeedRefresh(bool *pNeedRefresh);

/**
 * @brief Get time until next refresh required
 * @param[out] pTimeRemaining_ms Time remaining in milliseconds
 * @return STATUS_OK on success
 */
Status_t Watchdog_GetTimeRemaining(uint32_t *pTimeRemaining_ms);

#ifdef __cplusplus
}
#endif

#endif /* WATCHDOG_H */
