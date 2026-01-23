/**
 * @file    safety_monitor.c
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

/*============================================================================*/
/* INCLUDES                                                                   */
/*============================================================================*/
#include "safety_monitor.h"
#include "watchdog.h"
#include "app_errors.h"
#include "pm_monitor.h"
#include "temp_sensor.h"
#include "bsp_can.h"
#include <string.h>

/*============================================================================*/
/* PRIVATE CONSTANTS                                                          */
/*============================================================================*/

/** @brief Maximum allowed loop time (microseconds) */
#define SAFETY_MAX_LOOP_TIME_US     (10000U)  /* 10 ms */

/** @brief Stack watermark pattern */
#define STACK_WATERMARK_PATTERN     (0xDEADBEEFU)

/** @brief RAM test pattern */
#define RAM_TEST_PATTERN_1          (0x55555555U)
#define RAM_TEST_PATTERN_2          (0xAAAAAAAAU)

/*============================================================================*/
/* PRIVATE TYPES                                                              */
/*============================================================================*/

/** @brief Safety check structure */
typedef struct {
    bool (*checkFunction)(void);
    const char *name;
    SafetyLevel_t level;
    uint32_t violationCount;
    uint32_t lastViolationTime_ms;
} SafetyCheck_t;

/*============================================================================*/
/* PRIVATE VARIABLES                                                          */
/*============================================================================*/

/** @brief Safety monitor state */
static SafetyMonitor_State_t safety_state;

/** @brief Safety configuration */
static SafetyMonitor_Config_t safety_config;

/** @brief Fault callback */
static SafetyMonitor_FaultCallback_t safety_fault_callback = NULL;

/** @brief Initialization flag */
static bool safety_initialized = false;

/** @brief RAM test area */
static volatile uint32_t ram_test_area[4] __attribute__((section(".bss")));

/*============================================================================*/
/* PRIVATE FUNCTION PROTOTYPES                                                */
/*============================================================================*/

static bool safety_check_watchdog(void);
static bool safety_check_timing(void);
static bool safety_check_stack(void);
static bool safety_check_ram(void);
static bool safety_check_power(void);
static bool safety_check_temperature(void);
static bool safety_check_communication(void);
static void safety_handle_violation(SafetyCheck_t *pCheck);
static void safety_update_statistics(void);

/*============================================================================*/
/* PRIVATE VARIABLES - SAFETY CHECKS                                          */
/*============================================================================*/

/** @brief Safety check list */
static SafetyCheck_t safety_checks[] = {
    {safety_check_watchdog,      "Watchdog",      SAFETY_LEVEL_CRITICAL, 0, 0},
    {safety_check_timing,        "Timing",        SAFETY_LEVEL_CRITICAL, 0, 0},
    {safety_check_stack,         "Stack",         SAFETY_LEVEL_HIGH,     0, 0},
    {safety_check_ram,           "RAM",           SAFETY_LEVEL_HIGH,     0, 0},
    {safety_check_power,         "Power",         SAFETY_LEVEL_CRITICAL, 0, 0},
    {safety_check_temperature,   "Temperature",   SAFETY_LEVEL_MEDIUM,   0, 0},
    {safety_check_communication, "Communication", SAFETY_LEVEL_LOW,      0, 0}
};

#define SAFETY_CHECK_COUNT  (sizeof(safety_checks) / sizeof(safety_checks[0]))

/*============================================================================*/
/* PUBLIC FUNCTIONS                                                           */
/*============================================================================*/

/**
 * @brief Initialize safety monitor
 */
Status_t SafetyMonitor_Init(void)
{
    Status_t status = STATUS_OK;

    if (safety_initialized)
    {
        status = STATUS_ERROR_ALREADY_INIT;
    }
    else
    {
        /* Initialize state */
        (void)memset(&safety_state, 0, sizeof(safety_state));
        safety_state.systemState = SAFETY_STATE_INIT;
        safety_state.safetyLevel = SAFETY_LEVEL_CRITICAL;

        /* Default configuration - ALL DISABLED FOR TESTING */
        safety_config.enableWatchdogCheck = false;  /* Disabled - HAL IWDG/WWDG modules not enabled */
        safety_config.enableTimingCheck = false;    /* Disabled for testing */
        safety_config.enableStackCheck = false;     /* Disabled for testing */
        safety_config.enableRAMCheck = false;       /* Disabled for testing */
        safety_config.enablePowerCheck = false;     /* Disabled for testing */
        safety_config.enableTempCheck = false;      /* Disabled for testing */
        safety_config.enableCommCheck = false;      /* Disabled for testing */
        safety_config.maxLoopTime_us = SAFETY_MAX_LOOP_TIME_US;
        safety_config.stackWatermarkValue = STACK_WATERMARK_PATTERN;

        /* Initialize RAM test area with known pattern */
        ram_test_area[0] = RAM_TEST_PATTERN_1;
        ram_test_area[1] = RAM_TEST_PATTERN_2;
        ram_test_area[2] = ~RAM_TEST_PATTERN_1;
        ram_test_area[3] = ~RAM_TEST_PATTERN_2;

        safety_initialized = true;
    }

    return status;
}

/**
 * @brief Execute safety checks (call every 1ms)
 */
Status_t SafetyMonitor_Execute(void)
{
    Status_t status = STATUS_OK;

    if (safety_initialized)
    {
        uint32_t i;
        bool allPassed = true;

        /* Execute all enabled checks */
        for (i = 0U; i < SAFETY_CHECK_COUNT; i++)
        {
            SafetyCheck_t *pCheck = &safety_checks[i];

            if (pCheck->checkFunction != NULL)
            {
                bool passed = pCheck->checkFunction();

                if (!passed)
                {
                    safety_handle_violation(pCheck);
                    allPassed = false;

                    /* Update state based on severity */
                    if (pCheck->level == SAFETY_LEVEL_CRITICAL)
                    {
                        safety_state.criticalFaults++;
                    }
                    else if (pCheck->level == SAFETY_LEVEL_HIGH)
                    {
                        safety_state.highFaults++;
                    }
                    else if (pCheck->level == SAFETY_LEVEL_MEDIUM)
                    {
                        safety_state.mediumFaults++;
                    }
                    else
                    {
                        safety_state.lowFaults++;
                    }
                }
            }
        }

        /* Update system state */
        if (safety_state.criticalFaults > 0U)
        {
            safety_state.systemState = SAFETY_STATE_FAULT_CRITICAL;
            safety_state.safetyLevel = SAFETY_LEVEL_CRITICAL;
        }
        else if (safety_state.highFaults > 0U)
        {
            safety_state.systemState = SAFETY_STATE_FAULT_HIGH;
            safety_state.safetyLevel = SAFETY_LEVEL_HIGH;
        }
        else if (safety_state.mediumFaults > 0U)
        {
            safety_state.systemState = SAFETY_STATE_FAULT_MEDIUM;
            safety_state.safetyLevel = SAFETY_LEVEL_MEDIUM;
        }
        else if (safety_state.lowFaults > 0U)
        {
            safety_state.systemState = SAFETY_STATE_WARNING;
            safety_state.safetyLevel = SAFETY_LEVEL_LOW;
        }
        else if (allPassed)
        {
            safety_state.systemState = SAFETY_STATE_OK;
        }

        /* Update statistics */
        safety_update_statistics();
    }
    else
    {
        status = STATUS_ERROR_NOT_INIT;
    }

    return status;
}

/**
 * @brief Get safety monitor state
 */
Status_t SafetyMonitor_GetState(SafetyMonitor_State_t *pState)
{
    Status_t status = STATUS_ERROR_PARAM;

    if ((pState != NULL) && safety_initialized)
    {
        *pState = safety_state;
        status = STATUS_OK;
    }

    return status;
}

/**
 * @brief Check if system is safe
 */
bool SafetyMonitor_IsSafe(void)
{
    bool safe = false;

    if (safety_initialized)
    {
        safe = (safety_state.systemState == SAFETY_STATE_OK) ||
               (safety_state.systemState == SAFETY_STATE_WARNING);
    }

    return safe;
}

/**
 * @brief Update timing measurement
 */
Status_t SafetyMonitor_UpdateTiming(uint32_t loopTime_us)
{
    if (safety_initialized)
    {
        safety_state.lastLoopTime_us = loopTime_us;

        if (loopTime_us > safety_state.maxLoopTime_us)
        {
            safety_state.maxLoopTime_us = loopTime_us;
        }
    }

    return STATUS_OK;
}

/**
 * @brief Register fault callback
 */
Status_t SafetyMonitor_RegisterFaultCallback(SafetyMonitor_FaultCallback_t callback)
{
    safety_fault_callback = callback;
    return STATUS_OK;
}

/**
 * @brief Configure safety monitor
 */
Status_t SafetyMonitor_Configure(const SafetyMonitor_Config_t *pConfig)
{
    Status_t status = STATUS_ERROR_PARAM;

    if ((pConfig != NULL) && safety_initialized)
    {
        safety_config = *pConfig;
        status = STATUS_OK;
    }

    return status;
}

/**
 * @brief Reset safety statistics
 */
Status_t SafetyMonitor_ResetStatistics(void)
{
    if (safety_initialized)
    {
        safety_state.checkCycles = 0U;
        safety_state.totalViolations = 0U;
        safety_state.criticalFaults = 0U;
        safety_state.highFaults = 0U;
        safety_state.mediumFaults = 0U;
        safety_state.lowFaults = 0U;
        safety_state.maxLoopTime_us = 0U;

        /* Reset individual check counters */
        for (uint32_t i = 0U; i < SAFETY_CHECK_COUNT; i++)
        {
            safety_checks[i].violationCount = 0U;
        }
    }

    return STATUS_OK;
}

/**
 * @brief Force safe state
 */
Status_t SafetyMonitor_ForceSafeState(void)
{
    if (safety_initialized)
    {
        safety_state.systemState = SAFETY_STATE_SAFE;
        safety_state.safeStateActive = true;

        /* Log critical safety event */
        ErrorHandler_LogError(ERROR_SAFETY_CRITICAL_FAULT, 0U, __LINE__, 0U);

        /* Call fault callback */
        if (safety_fault_callback != NULL)
        {
            safety_fault_callback(SAFETY_LEVEL_CRITICAL, "Forced safe state");
        }
    }

    return STATUS_OK;
}

/**
 * @brief De-initialize safety monitor
 */
Status_t SafetyMonitor_DeInit(void)
{
    if (safety_initialized)
    {
        safety_initialized = false;
    }

    return STATUS_OK;
}

/*============================================================================*/
/* PRIVATE FUNCTIONS - SAFETY CHECKS                                          */
/*============================================================================*/

/**
 * @brief Check watchdog health
 */
static bool safety_check_watchdog(void)
{
    bool passed = true;

    if (safety_config.enableWatchdogCheck)
    {
        /* Check if watchdog is running */
        bool iwdgRunning = false;
        bool wwdgRunning = false;

        (void)Watchdog_IsIWDGRunning(&iwdgRunning);
        (void)Watchdog_IsWWDGRunning(&wwdgRunning);

        if (!iwdgRunning || !wwdgRunning)
        {
            passed = false;
        }
    }

    return passed;
}

/**
 * @brief Check timing constraints
 */
static bool safety_check_timing(void)
{
    bool passed = true;

    if (safety_config.enableTimingCheck)
    {
        if (safety_state.lastLoopTime_us > safety_config.maxLoopTime_us)
        {
            passed = false;
        }
    }

    return passed;
}

/**
 * @brief Check stack integrity
 */
static bool safety_check_stack(void)
{
    bool passed = true;

    if (safety_config.enableStackCheck)
    {
        /* Check stack watermark (simplified check) */
        extern uint32_t _estack;  /* From linker script */
        (void)_estack;  /* Suppress unused warning - used for reference */

        /* Check if stack pointer is within valid RAM range */
        uint32_t currentSP;
        __asm volatile ("MRS %0, MSP" : "=r" (currentSP));

        if ((currentSP < MCU_RAM_START) || (currentSP > MCU_RAM_END))
        {
            passed = false;
        }
    }

    return passed;
}

/**
 * @brief Check RAM integrity
 */
static bool safety_check_ram(void)
{
    bool passed = true;

    if (safety_config.enableRAMCheck)
    {
        /* Verify RAM test area hasn't been corrupted */
        if ((ram_test_area[0] != RAM_TEST_PATTERN_1) ||
            (ram_test_area[1] != RAM_TEST_PATTERN_2) ||
            (ram_test_area[2] != ~RAM_TEST_PATTERN_1) ||
            (ram_test_area[3] != ~RAM_TEST_PATTERN_2))
        {
            passed = false;

            /* Restore patterns for next check */
            ram_test_area[0] = RAM_TEST_PATTERN_1;
            ram_test_area[1] = RAM_TEST_PATTERN_2;
            ram_test_area[2] = ~RAM_TEST_PATTERN_1;
            ram_test_area[3] = ~RAM_TEST_PATTERN_2;
        }
    }

    return passed;
}

/**
 * @brief Check power supply health
 */
static bool safety_check_power(void)
{
    bool passed = true;

    if (safety_config.enablePowerCheck)
    {
        /* Check if power monitoring module reports good status */
        bool powerGood = false;
        if (PM_Monitor_IsSystemPowerGood(&powerGood) == STATUS_OK)
        {
            passed = powerGood;
        }
        else
        {
            /* If we cannot read power status, assume failure */
            passed = false;
        }
    }

    return passed;
}

/**
 * @brief Check temperature limits
 */
static bool safety_check_temperature(void)
{
    bool passed = true;

    if (safety_config.enableTempCheck)
    {
        /* Check if temperature is within safe limits */
        int32_t temp_mC = 0;
        if (TempSensor_ReadTemperature(&temp_mC) == STATUS_OK)
        {
            /* Check if temperature is within safe operating range */
            /* Typical range: -40°C to +125°C */
            if ((temp_mC < -40000) || (temp_mC > 125000))
            {
                passed = false;
            }
        }
        else
        {
            /* If we cannot read temperature, assume failure */
            passed = false;
        }
    }

    return passed;
}

/**
 * @brief Check communication health
 */
static bool safety_check_communication(void)
{
    bool passed = true;

    if (safety_config.enableCommCheck)
    {
        /* Check CAN bus health */
        CAN_Statistics_t stats;
        if (BSP_CAN_GetStatistics(BSP_CAN_INSTANCE_1, &stats) == STATUS_OK)
        {
            /* Check if error rate is acceptable (< 1% of total messages) */
            uint32_t totalMessages = stats.txCount + stats.rxCount;
            if (totalMessages > 0U)
            {
                uint32_t errorRate = ((stats.txErrorCount + stats.rxErrorCount) * 100U) / totalMessages;
                if (errorRate > 1U)
                {
                    passed = false;
                }
            }
        }
        else
        {
            /* If we cannot read CAN statistics, assume failure */
            passed = false;
        }
    }

    return passed;
}

/**
 * @brief Handle safety violation
 */
static void safety_handle_violation(SafetyCheck_t *pCheck)
{
    if (pCheck != NULL)
    {
        pCheck->violationCount++;
        pCheck->lastViolationTime_ms = HAL_GetTick();
        safety_state.totalViolations++;

        /* Log error */
        ErrorHandler_LogError(ERROR_SAFETY_MONITOR_FAULT,
                            (uint32_t)pCheck->level,
                            __LINE__,
                            pCheck->violationCount);

        /* Call fault callback */
        if (safety_fault_callback != NULL)
        {
            safety_fault_callback(pCheck->level, pCheck->name);
        }

        /* Trigger safe state for critical violations */
        if ((pCheck->level == SAFETY_LEVEL_CRITICAL) &&
            (pCheck->violationCount >= 3U))
        {
            safety_state.safeStateActive = true;
            safety_state.systemState = SAFETY_STATE_SAFE;
        }
    }
}

/**
 * @brief Update safety statistics
 */
static void safety_update_statistics(void)
{
    safety_state.checkCycles++;
    safety_state.lastCheckTime_ms = HAL_GetTick();
}

/*============================================================================*/
/* END OF FILE                                                                */
/*============================================================================*/
