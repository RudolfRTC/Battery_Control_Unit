/**
 * @file    app_errors.c
 * @brief   Error handling and DTC management implementation
 * @author  Battery Control Unit Development Team
 * @date    2026-01-09
 * @version 1.0.0
 *
 * @note    MISRA C:2012 compliant
 * @note    ISO 26262 ASIL-B safety requirements
 * @note    Diagnostic Trouble Code (DTC) management
 *
 * @copyright Copyright (c) 2026
 */

/*============================================================================*/
/* INCLUDES                                                                   */
/*============================================================================*/
#include "app_errors.h"
#include "fram_driver.h"
#include "timestamp.h"
#include "bsp_gpio.h"
#include <string.h>

/*============================================================================*/
/* PRIVATE CONSTANTS                                                          */
/*============================================================================*/

/** @brief Maximum number of active DTCs */
#define MAX_ACTIVE_DTCS         (32U)

/** @brief Maximum DTC history entries */
#define MAX_DTC_HISTORY         (64U)

/** @brief DTC confirmation threshold (occurrences before confirmed) */
#define DTC_CONFIRMATION_THRESHOLD  (3U)

/** @brief DTC aging threshold (ms without occurrence before cleared) */
#define DTC_AGING_THRESHOLD_MS  (60000U)  /* 60 seconds */

/*============================================================================*/
/* PRIVATE TYPES                                                              */
/*============================================================================*/

/** @brief DTC entry structure */
typedef struct {
    ErrorCode_t code;
    uint32_t timestamp_ms;
    uint32_t occurrenceCount;
    uint32_t param1;
    uint32_t param2;
    uint32_t param3;
    DTC_Status_t status;
    bool confirmed;
} DTC_Entry_t;

/** @brief Error handler state */
typedef struct {
    DTC_Entry_t activeDTCs[MAX_ACTIVE_DTCS];
    uint32_t activeDTCCount;
    uint32_t totalErrorCount;
    uint32_t criticalErrorCount;
    uint32_t warningCount;
    bool safeStateActive;
    uint32_t lastErrorTime_ms;
} ErrorHandler_State_t;

/*============================================================================*/
/* PRIVATE VARIABLES                                                          */
/*============================================================================*/

/** @brief Error handler state */
static ErrorHandler_State_t error_state;

/** @brief Error callback */
static ErrorCallback_t error_callback = NULL;

/** @brief Initialization flag */
static bool error_handler_initialized = false;

/** @brief Error code names (for debugging) */
static const char* error_names[] = {
    "NONE",
    "WATCHDOG",
    "TIMING_VIOLATION",
    "STACK_OVERFLOW",
    "MEMORY_CORRUPTION",
    "POWER_FAULT",
    "TEMPERATURE_FAULT",
    "CAN_BUS_OFF",
    "SENSOR_FAULT",
    "ACTUATOR_FAULT",
    "CRITICAL_FAULT",
    "MONITOR_FAULT",
    "HARDWARE_FAULT"
};

/*============================================================================*/
/* PRIVATE FUNCTION PROTOTYPES                                                */
/*============================================================================*/

static DTC_Entry_t* find_dtc(ErrorCode_t code);
static DTC_Entry_t* allocate_dtc(void);
static void age_dtcs(void);
static void save_dtc_to_fram(const DTC_Entry_t *pDTC);
static const char* get_error_name(ErrorCode_t code);

/*============================================================================*/
/* PUBLIC FUNCTIONS                                                           */
/*============================================================================*/

/**
 * @brief Initialize error handler
 */
Status_t ErrorHandler_Init(void)
{
    Status_t status = STATUS_OK;

    if (error_handler_initialized)
    {
        status = STATUS_ERROR_ALREADY_INIT;
    }
    else
    {
        /* Initialize state */
        (void)memset(&error_state, 0, sizeof(error_state));

        /* Load DTC history from FRAM */
        /* TODO: Implement DTC history loading */

        error_handler_initialized = true;
    }

    return status;
}

/**
 * @brief Log error with parameters
 */
Status_t ErrorHandler_LogError(ErrorCode_t code, uint32_t param1, uint32_t param2, uint32_t param3)
{
    Status_t status = STATUS_OK;

    if (error_handler_initialized)
    {
        DTC_Entry_t *pDTC = find_dtc(code);

        if (pDTC == NULL)
        {
            /* Allocate new DTC entry */
            pDTC = allocate_dtc();

            if (pDTC != NULL)
            {
                pDTC->code = code;
                pDTC->timestamp_ms = HAL_GetTick();
                pDTC->occurrenceCount = 1U;
                pDTC->param1 = param1;
                pDTC->param2 = param2;
                pDTC->param3 = param3;
                pDTC->status = DTC_STATUS_PENDING;
                pDTC->confirmed = false;

                error_state.activeDTCCount++;
            }
            else
            {
                status = STATUS_ERROR_OVERFLOW;
            }
        }
        else
        {
            /* Update existing DTC */
            pDTC->occurrenceCount++;
            pDTC->timestamp_ms = HAL_GetTick();
            pDTC->param1 = param1;
            pDTC->param2 = param2;
            pDTC->param3 = param3;

            /* Confirm DTC if threshold reached */
            if (pDTC->occurrenceCount >= DTC_CONFIRMATION_THRESHOLD)
            {
                pDTC->confirmed = true;
                pDTC->status = DTC_STATUS_CONFIRMED;
            }
        }

        /* Update statistics */
        error_state.totalErrorCount++;
        error_state.lastErrorTime_ms = HAL_GetTick();

        if ((code >= ERROR_SAFETY_WATCHDOG) && (code <= ERROR_SAFETY_CRITICAL_FAULT))
        {
            error_state.criticalErrorCount++;
        }

        /* Save to FRAM if confirmed */
        if ((pDTC != NULL) && pDTC->confirmed)
        {
            save_dtc_to_fram(pDTC);
        }

        /* Call error callback */
        if (error_callback != NULL)
        {
            error_callback(code, param1, param2, param3);
        }

        /* Enter safe state for critical errors */
        if ((code >= ERROR_SAFETY_WATCHDOG) && (code <= ERROR_SAFETY_CRITICAL_FAULT))
        {
            ErrorHandler_SafeState();
        }
    }

    return status;
}

/**
 * @brief Get DTC information
 */
Status_t ErrorHandler_GetDTC(ErrorCode_t code, DTC_Info_t *pInfo)
{
    Status_t status = STATUS_ERROR_PARAM;

    if ((pInfo != NULL) && error_handler_initialized)
    {
        DTC_Entry_t *pDTC = find_dtc(code);

        if (pDTC != NULL)
        {
            pInfo->code = pDTC->code;
            pInfo->timestamp_ms = pDTC->timestamp_ms;
            pInfo->occurrenceCount = pDTC->occurrenceCount;
            pInfo->param1 = pDTC->param1;
            pInfo->param2 = pDTC->param2;
            pInfo->param3 = pDTC->param3;
            pInfo->status = pDTC->status;
            pInfo->confirmed = pDTC->confirmed;

            status = STATUS_OK;
        }
        else
        {
            status = STATUS_ERROR_NOT_FOUND;
        }
    }

    return status;
}

/**
 * @brief Clear DTC
 */
Status_t ErrorHandler_ClearDTC(ErrorCode_t code)
{
    Status_t status = STATUS_ERROR_NOT_FOUND;

    if (error_handler_initialized)
    {
        for (uint32_t i = 0U; i < MAX_ACTIVE_DTCS; i++)
        {
            if (error_state.activeDTCs[i].code == code)
            {
                /* Mark as cleared */
                error_state.activeDTCs[i].status = DTC_STATUS_CLEARED;
                error_state.activeDTCs[i].confirmed = false;
                status = STATUS_OK;
                break;
            }
        }
    }

    return status;
}

/**
 * @brief Clear all DTCs
 */
Status_t ErrorHandler_ClearAllDTCs(void)
{
    if (error_handler_initialized)
    {
        for (uint32_t i = 0U; i < MAX_ACTIVE_DTCS; i++)
        {
            error_state.activeDTCs[i].status = DTC_STATUS_CLEARED;
            error_state.activeDTCs[i].confirmed = false;
        }

        error_state.activeDTCCount = 0U;
    }

    return STATUS_OK;
}

/**
 * @brief Get active DTC count
 */
Status_t ErrorHandler_GetActiveDTCCount(uint32_t *pCount)
{
    Status_t status = STATUS_ERROR_PARAM;

    if ((pCount != NULL) && error_handler_initialized)
    {
        uint32_t count = 0U;

        for (uint32_t i = 0U; i < MAX_ACTIVE_DTCS; i++)
        {
            if ((error_state.activeDTCs[i].status == DTC_STATUS_PENDING) ||
                (error_state.activeDTCs[i].status == DTC_STATUS_CONFIRMED))
            {
                count++;
            }
        }

        *pCount = count;
        status = STATUS_OK;
    }

    return status;
}

/**
 * @brief Check if DTC is active
 */
bool ErrorHandler_IsDTCActive(ErrorCode_t code)
{
    bool active = false;

    if (error_handler_initialized)
    {
        DTC_Entry_t *pDTC = find_dtc(code);

        if (pDTC != NULL)
        {
            active = (pDTC->status == DTC_STATUS_PENDING) ||
                    (pDTC->status == DTC_STATUS_CONFIRMED);
        }
    }

    return active;
}

/**
 * @brief Update error handler (periodic aging)
 */
Status_t ErrorHandler_Update(void)
{
    if (error_handler_initialized)
    {
        age_dtcs();
    }

    return STATUS_OK;
}

/**
 * @brief Register error callback
 */
Status_t ErrorHandler_RegisterCallback(ErrorCallback_t callback)
{
    error_callback = callback;
    return STATUS_OK;
}

/**
 * @brief Get error statistics
 */
Status_t ErrorHandler_GetStatistics(ErrorStatistics_t *pStats)
{
    Status_t status = STATUS_ERROR_PARAM;

    if ((pStats != NULL) && error_handler_initialized)
    {
        pStats->totalErrorCount = error_state.totalErrorCount;
        pStats->criticalErrorCount = error_state.criticalErrorCount;
        pStats->warningCount = error_state.warningCount;
        pStats->activeDTCCount = error_state.activeDTCCount;
        pStats->lastErrorTime_ms = error_state.lastErrorTime_ms;
        pStats->safeStateActive = error_state.safeStateActive;

        status = STATUS_OK;
    }

    return status;
}

/**
 * @brief Enter safe state (emergency)
 */
void ErrorHandler_SafeState(void)
{
    /* Disable interrupts */
    __disable_irq();

    /* Turn off all outputs */
    /* TODO: BTT6200_SetSafeState(); */

    /* Turn on error LED */
    (void)BSP_GPIO_WritePin(GPIOH, GPIO_PIN_10, GPIO_STATE_HIGH);  /* Error LED */

    /* Set safe state flag */
    error_state.safeStateActive = true;

    /* Log to FRAM */
    FRAM_FaultLogEntry_t faultLog;
    faultLog.errorCode = ERROR_SAFETY_CRITICAL_FAULT;
    faultLog.timestamp_ms = HAL_GetTick();
    faultLog.param1 = 0U;
    faultLog.param2 = 0U;
    faultLog.param3 = 0U;
    (void)FRAM_WriteFaultLog(0U, &faultLog);

    /* Re-enable interrupts */
    __enable_irq();
}

/**
 * @brief Hard fault handler
 */
void ErrorHandler_HardFault(void)
{
    /* Disable interrupts */
    __disable_irq();

    /* Turn on error LED */
    (void)BSP_GPIO_WritePin(GPIOH, GPIO_PIN_10, GPIO_STATE_HIGH);

    /* Log hard fault */
    ErrorHandler_LogError(ERROR_SAFETY_CRITICAL_FAULT, 0U, __LINE__, 0U);

    /* Wait for watchdog reset */
    while (1)
    {
        /* Infinite loop */
    }
}

/**
 * @brief Get error name string
 */
const char* ErrorHandler_GetErrorName(ErrorCode_t code)
{
    return get_error_name(code);
}

/*============================================================================*/
/* PRIVATE FUNCTIONS                                                          */
/*============================================================================*/

/**
 * @brief Find DTC by code
 */
static DTC_Entry_t* find_dtc(ErrorCode_t code)
{
    DTC_Entry_t *pDTC = NULL;

    for (uint32_t i = 0U; i < MAX_ACTIVE_DTCS; i++)
    {
        if ((error_state.activeDTCs[i].code == code) &&
            (error_state.activeDTCs[i].status != DTC_STATUS_CLEARED))
        {
            pDTC = &error_state.activeDTCs[i];
            break;
        }
    }

    return pDTC;
}

/**
 * @brief Allocate new DTC entry
 */
static DTC_Entry_t* allocate_dtc(void)
{
    DTC_Entry_t *pDTC = NULL;

    /* Find free slot */
    for (uint32_t i = 0U; i < MAX_ACTIVE_DTCS; i++)
    {
        if ((error_state.activeDTCs[i].code == ERROR_NONE) ||
            (error_state.activeDTCs[i].status == DTC_STATUS_CLEARED))
        {
            pDTC = &error_state.activeDTCs[i];
            (void)memset(pDTC, 0, sizeof(DTC_Entry_t));
            break;
        }
    }

    return pDTC;
}

/**
 * @brief Age DTCs (clear old pending DTCs)
 */
static void age_dtcs(void)
{
    uint32_t currentTime = HAL_GetTick();

    for (uint32_t i = 0U; i < MAX_ACTIVE_DTCS; i++)
    {
        DTC_Entry_t *pDTC = &error_state.activeDTCs[i];

        if ((pDTC->status == DTC_STATUS_PENDING) && !pDTC->confirmed)
        {
            uint32_t age_ms = currentTime - pDTC->timestamp_ms;

            if (age_ms > DTC_AGING_THRESHOLD_MS)
            {
                pDTC->status = DTC_STATUS_CLEARED;
                pDTC->code = ERROR_NONE;

                if (error_state.activeDTCCount > 0U)
                {
                    error_state.activeDTCCount--;
                }
            }
        }
    }
}

/**
 * @brief Save DTC to FRAM
 */
static void save_dtc_to_fram(const DTC_Entry_t *pDTC)
{
    if (pDTC != NULL)
    {
        FRAM_FaultLogEntry_t faultLog;

        faultLog.errorCode = pDTC->code;
        faultLog.timestamp_ms = pDTC->timestamp_ms;
        faultLog.param1 = pDTC->param1;
        faultLog.param2 = pDTC->param2;
        faultLog.param3 = pDTC->param3;

        /* Write to circular fault log */
        static uint16_t logIndex = 0U;
        (void)FRAM_WriteFaultLog(logIndex, &faultLog);

        logIndex++;
        if (logIndex >= (FRAM_FAULT_LOG_SIZE / sizeof(FRAM_FaultLogEntry_t)))
        {
            logIndex = 0U;
        }
    }
}

/**
 * @brief Get error name string
 */
static const char* get_error_name(ErrorCode_t code)
{
    const char *name = "UNKNOWN";
    uint32_t index = (uint32_t)code;

    if (index < (sizeof(error_names) / sizeof(error_names[0])))
    {
        name = error_names[index];
    }

    return name;
}

/*============================================================================*/
/* END OF FILE                                                                */
/*============================================================================*/
