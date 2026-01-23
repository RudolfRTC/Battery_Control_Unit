/**
 * @file    watchdog.c
 * @brief   Dual watchdog (IWDG + WWDG) implementation
 * @author  Battery Control Unit Development Team
 * @date    2026-01-09
 * @version 1.0.0
 *
 * @note    MISRA C:2012 compliant
 * @note    ISO 26262 ASIL-B safety requirements
 * @note    Stubbed out when HAL watchdog modules not enabled
 *
 * @copyright Copyright (c) 2026
 */

/*============================================================================*/
/* INCLUDES                                                                   */
/*============================================================================*/
#include "watchdog.h"
#include "app_config.h"
#include "stm32f4xx_hal.h"

/*============================================================================*/
/* PRIVATE VARIABLES                                                          */
/*============================================================================*/

#ifdef HAL_IWDG_MODULE_ENABLED
/** @brief IWDG handle */
static IWDG_HandleTypeDef hiwdg;
#endif

#ifdef HAL_WWDG_MODULE_ENABLED
/** @brief WWDG handle */
static WWDG_HandleTypeDef hwwdg;
#endif

/** @brief Watchdog status */
static WDG_Status_t watchdog_status = {0};

/** @brief Last reset cause */
static ResetCause_t last_reset_cause = RESET_CAUSE_UNKNOWN;

/*============================================================================*/
/* PRIVATE FUNCTION PROTOTYPES                                                */
/*============================================================================*/

static ResetCause_t watchdog_detect_reset_cause(void);

/*============================================================================*/
/* PUBLIC FUNCTIONS                                                           */
/*============================================================================*/

/**
 * @brief Initialize watchdog module
 */
Status_t Watchdog_Init(void)
{
    Status_t status = STATUS_OK;

    /* Detect and store reset cause before clearing flags */
    last_reset_cause = watchdog_detect_reset_cause();

    /* Clear reset flags */
    __HAL_RCC_CLEAR_RESET_FLAGS();

    /* Initialize status */
    watchdog_status.iwdgEnabled = false;
    watchdog_status.wwdgEnabled = false;
    watchdog_status.iwdgRefreshCount = 0U;
    watchdog_status.wwdgRefreshCount = 0U;
    watchdog_status.missedRefreshCount = 0U;
    watchdog_status.lastRefreshTime_ms = HAL_GetTick();

    return status;
}

/**
 * @brief Start independent watchdog (IWDG)
 */
Status_t Watchdog_StartIWDG(void)
{
    Status_t status = STATUS_OK;

#ifdef HAL_IWDG_MODULE_ENABLED
    if (!watchdog_status.iwdgEnabled)
    {
        /* Configure IWDG (500ms timeout) */
        hiwdg.Instance = IWDG;
        hiwdg.Init.Prescaler = IWDG_PRESCALER_64;
        hiwdg.Init.Reload = 250;

        if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
        {
            status = STATUS_ERROR_HW_FAULT;
        }
        else
        {
            watchdog_status.iwdgEnabled = true;
        }
    }
#else
    /* IWDG not available - stub */
    (void)status;
#endif

    return status;
}

/**
 * @brief Start window watchdog (WWDG)
 */
Status_t Watchdog_StartWWDG(void)
{
    Status_t status = STATUS_OK;

#ifdef HAL_WWDG_MODULE_ENABLED
    if (!watchdog_status.wwdgEnabled)
    {
        /* Enable WWDG clock */
        __HAL_RCC_WWDG_CLK_ENABLE();

        /* Configure WWDG */
        hwwdg.Instance = WWDG;
        hwwdg.Init.Prescaler = WWDG_PRESCALER_8;
        hwwdg.Init.Window = 80;
        hwwdg.Init.Counter = 127;
        hwwdg.Init.EWIMode = WWDG_EWI_DISABLE;

        if (HAL_WWDG_Init(&hwwdg) != HAL_OK)
        {
            status = STATUS_ERROR_HW_FAULT;
        }
        else
        {
            watchdog_status.wwdgEnabled = true;
        }
    }
#else
    /* WWDG not available - stub */
    (void)status;
#endif

    return status;
}

/**
 * @brief Refresh independent watchdog
 */
Status_t Watchdog_RefreshIWDG(void)
{
    Status_t status = STATUS_OK;

#ifdef HAL_IWDG_MODULE_ENABLED
    if (watchdog_status.iwdgEnabled)
    {
        HAL_IWDG_Refresh(&hiwdg);
        watchdog_status.iwdgRefreshCount++;
        watchdog_status.lastRefreshTime_ms = HAL_GetTick();
    }
    else
    {
        status = STATUS_ERROR_NOT_INIT;
    }
#else
    /* IWDG not available - stub */
    watchdog_status.lastRefreshTime_ms = HAL_GetTick();
#endif

    return status;
}

/**
 * @brief Refresh window watchdog
 */
Status_t Watchdog_RefreshWWDG(void)
{
    Status_t status = STATUS_OK;

#ifdef HAL_WWDG_MODULE_ENABLED
    if (watchdog_status.wwdgEnabled)
    {
        if (HAL_WWDG_Refresh(&hwwdg) == HAL_OK)
        {
            watchdog_status.wwdgRefreshCount++;
        }
        else
        {
            status = STATUS_ERROR;
        }
    }
    else
    {
        status = STATUS_ERROR_NOT_INIT;
    }
#else
    /* WWDG not available - stub */
    (void)status;
#endif

    return status;
}

/**
 * @brief Refresh all watchdogs
 */
Status_t Watchdog_RefreshAll(void)
{
    Status_t status1 = STATUS_OK;
    Status_t status2 = STATUS_OK;

    status1 = Watchdog_RefreshIWDG();
    status2 = Watchdog_RefreshWWDG();

    return ((status1 == STATUS_OK) && (status2 == STATUS_OK)) ? STATUS_OK : STATUS_ERROR;
}

/**
 * @brief Get watchdog status
 */
Status_t Watchdog_GetStatus(WDG_Status_t *pStatus)
{
    Status_t status = STATUS_ERROR_PARAM;

    if (pStatus != NULL)
    {
        *pStatus = watchdog_status;
        status = STATUS_OK;
    }

    return status;
}

/**
 * @brief Get last reset cause
 */
Status_t Watchdog_GetResetCause(ResetCause_t *pCause)
{
    Status_t status = STATUS_ERROR_PARAM;

    if (pCause != NULL)
    {
        *pCause = last_reset_cause;
        status = STATUS_OK;
    }

    return status;
}

/**
 * @brief Clear reset cause flags
 */
Status_t Watchdog_ClearResetFlags(void)
{
    __HAL_RCC_CLEAR_RESET_FLAGS();
    return STATUS_OK;
}

/**
 * @brief Check if reset was caused by watchdog
 */
Status_t Watchdog_WasWatchdogReset(bool *pWatchdogReset)
{
    Status_t status = STATUS_ERROR_PARAM;

    if (pWatchdogReset != NULL)
    {
        *pWatchdogReset = ((last_reset_cause == RESET_CAUSE_IWDG) ||
                          (last_reset_cause == RESET_CAUSE_WWDG));
        status = STATUS_OK;
    }

    return status;
}

/**
 * @brief Check if IWDG is running
 */
Status_t Watchdog_IsIWDGRunning(bool *pRunning)
{
    Status_t status = STATUS_ERROR_PARAM;

    if (pRunning != NULL)
    {
        *pRunning = watchdog_status.iwdgEnabled;
        status = STATUS_OK;
    }

    return status;
}

/**
 * @brief Check if WWDG is running
 */
Status_t Watchdog_IsWWDGRunning(bool *pRunning)
{
    Status_t status = STATUS_ERROR_PARAM;

    if (pRunning != NULL)
    {
        *pRunning = watchdog_status.wwdgEnabled;
        status = STATUS_OK;
    }

    return status;
}

/**
 * @brief Get reset cause string
 */
const char* Watchdog_GetResetCauseString(ResetCause_t cause)
{
    const char* cause_str;

    switch (cause)
    {
        case RESET_CAUSE_POR:
            cause_str = "Power-On Reset";
            break;
        case RESET_CAUSE_PIN:
            cause_str = "External Pin Reset";
            break;
        case RESET_CAUSE_SOFTWARE:
            cause_str = "Software Reset";
            break;
        case RESET_CAUSE_IWDG:
            cause_str = "Independent Watchdog Reset";
            break;
        case RESET_CAUSE_WWDG:
            cause_str = "Window Watchdog Reset";
            break;
        case RESET_CAUSE_LOW_POWER:
            cause_str = "Low-Power Reset";
            break;
        case RESET_CAUSE_BROWNOUT:
            cause_str = "Brown-Out Reset";
            break;
        default:
            cause_str = "Unknown Reset";
            break;
    }

    return cause_str;
}

/**
 * @brief Force system reset
 */
void Watchdog_ForceReset(void)
{
    /* Disable interrupts */
    __disable_irq();

    /* Perform software reset */
    NVIC_SystemReset();

    /* Never reached */
    while (1) {}
}

/**
 * @brief Check if watchdog refresh is needed
 */
Status_t Watchdog_NeedRefresh(bool *pNeedRefresh)
{
    Status_t status = STATUS_ERROR_PARAM;

    if (pNeedRefresh != NULL)
    {
        uint32_t elapsed_ms;

        elapsed_ms = HAL_GetTick() - watchdog_status.lastRefreshTime_ms;
        *pNeedRefresh = (elapsed_ms >= WDG_REFRESH_PERIOD_MS);

        status = STATUS_OK;
    }

    return status;
}

/**
 * @brief Get time until next refresh required
 */
Status_t Watchdog_GetTimeRemaining(uint32_t *pTimeRemaining_ms)
{
    Status_t status = STATUS_ERROR_PARAM;

    if (pTimeRemaining_ms != NULL)
    {
        uint32_t elapsed_ms;

        elapsed_ms = HAL_GetTick() - watchdog_status.lastRefreshTime_ms;

        if (elapsed_ms >= WDG_IWDG_TIMEOUT_MS)
        {
            *pTimeRemaining_ms = 0U;
        }
        else
        {
            *pTimeRemaining_ms = WDG_IWDG_TIMEOUT_MS - elapsed_ms;
        }

        status = STATUS_OK;
    }

    return status;
}

/*============================================================================*/
/* PRIVATE FUNCTIONS                                                          */
/*============================================================================*/

/**
 * @brief Detect reset cause from RCC flags
 */
static ResetCause_t watchdog_detect_reset_cause(void)
{
    ResetCause_t cause = RESET_CAUSE_UNKNOWN;

    /* Check reset flags in order of priority */
    if (__HAL_RCC_GET_FLAG(RCC_FLAG_LPWRRST) != 0U)
    {
        cause = RESET_CAUSE_LOW_POWER;
    }
    else if (__HAL_RCC_GET_FLAG(RCC_FLAG_WWDGRST) != 0U)
    {
        cause = RESET_CAUSE_WWDG;
    }
    else if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST) != 0U)
    {
        cause = RESET_CAUSE_IWDG;
    }
    else if (__HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST) != 0U)
    {
        cause = RESET_CAUSE_SOFTWARE;
    }
    else if (__HAL_RCC_GET_FLAG(RCC_FLAG_PORRST) != 0U)
    {
        cause = RESET_CAUSE_POR;
    }
    else if (__HAL_RCC_GET_FLAG(RCC_FLAG_PINRST) != 0U)
    {
        cause = RESET_CAUSE_PIN;
    }
    else if (__HAL_RCC_GET_FLAG(RCC_FLAG_BORRST) != 0U)
    {
        cause = RESET_CAUSE_BROWNOUT;
    }
    else
    {
        /* Unknown cause */
        cause = RESET_CAUSE_UNKNOWN;
    }

    return cause;
}

/*============================================================================*/
/* END OF FILE                                                                */
/*============================================================================*/
