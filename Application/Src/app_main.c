/**
 * @file    app_main.c
 * @brief   Main application implementation
 * @author  Battery Control Unit Development Team
 * @date    2026-01-09
 * @version 1.0.0
 *
 * @note    MISRA C:2012 compliant
 * @note    ISO 26262 ASIL-B safety requirements
 *
 * @copyright Copyright (c) 2026
 */

/*============================================================================*/
/* INCLUDES                                                                   */
/*============================================================================*/
#include "app_main.h"
#include "app_config.h"
#include "app_errors.h"
#include "bsp_gpio.h"
#include "bsp_adc.h"
#include "bsp_i2c.h"
#include "watchdog.h"
#include "lem_sensor.h"
#include "timestamp.h"
#include <string.h>

/*============================================================================*/
/* PRIVATE VARIABLES                                                          */
/*============================================================================*/

/** @brief Application state */
static AppState_t app_state = APP_STATE_INIT;

/** @brief Application status */
static AppStatus_t app_status = {0};

/** @brief Fast task timestamp (1ms) */
static uint32_t fast_task_last_ms = 0U;

/** @brief Medium task timestamp (10ms) */
static uint32_t medium_task_last_ms = 0U;

/** @brief Slow task timestamp (100ms) */
static uint32_t slow_task_last_ms = 0U;

/*============================================================================*/
/* PRIVATE FUNCTION PROTOTYPES                                                */
/*============================================================================*/

static Status_t app_init_peripherals(void);
static Status_t app_init_modules(void);
static void app_update_status(void);
static void app_state_machine(void);

/*============================================================================*/
/* PUBLIC FUNCTIONS                                                           */
/*============================================================================*/

/**
 * @brief Initialize application
 */
Status_t App_Init(void)
{
    Status_t status = STATUS_OK;

    /* Initialize HAL */
    HAL_Init();

    /* Configure system clock to 100 MHz */
    SystemClock_Config();

    /* Initialize timestamp module */
    status = Timestamp_Init();

    if (status == STATUS_OK)
    {
        /* Initialize peripherals */
        status = app_init_peripherals();
    }

    if (status == STATUS_OK)
    {
        /* Initialize modules */
        status = app_init_modules();
    }

    if (status == STATUS_OK)
    {
        /* Initialize watchdog */
        status = Watchdog_Init();

        /* Check reset cause */
        ResetCause_t resetCause;
        (void)Watchdog_GetResetCause(&resetCause);

        if ((resetCause == RESET_CAUSE_IWDG) || (resetCause == RESET_CAUSE_WWDG))
        {
            /* Watchdog reset occurred - log error */
            ErrorHandler_LogError(ERROR_SAFETY_WATCHDOG, 0U, __LINE__, 0U);
        }

        /* Start watchdogs */
        (void)Watchdog_StartIWDG();
        (void)Watchdog_StartWWDG();

        /* Initialize status */
        app_status.state = APP_STATE_STARTUP;
        app_status.uptime_ms = 0U;
        app_status.cycleCount = 0U;
        app_status.cycleTime_us = 0U;
        app_status.maxCycleTime_us = 0U;
        app_status.powerGood = false;
        app_status.safetyOK = false;
        app_status.activeErrors = 0U;
        app_status.activeWarnings = 0U;

        app_state = APP_STATE_STARTUP;
    }

    return status;
}

/**
 * @brief Application main loop
 */
void App_MainLoop(void)
{
    uint32_t loop_start_us;
    uint32_t loop_end_us;
    uint32_t loop_time_us;

    while (1)
    {
        /* Measure loop execution time */
        loop_start_us = (uint32_t)Timestamp_GetMicros();

        /* Get current timestamp */
        uint32_t current_ms = Timestamp_GetMillis();

        /* Fast tasks (1ms) - Safety critical */
        if ((current_ms - fast_task_last_ms) >= TASK_PERIOD_SAFETY_MS)
        {
            App_FastTasks();
            fast_task_last_ms = current_ms;
        }

        /* Medium tasks (10ms) - Control and I/O */
        if ((current_ms - medium_task_last_ms) >= TASK_PERIOD_FAST_MS)
        {
            App_MediumTasks();
            medium_task_last_ms = current_ms;
        }

        /* Slow tasks (100ms) - Communication and diagnostics */
        if ((current_ms - slow_task_last_ms) >= TASK_PERIOD_SLOW_MS)
        {
            App_SlowTasks();
            slow_task_last_ms = current_ms;
        }

        /* State machine */
        app_state_machine();

        /* Update status */
        app_update_status();

        /* Measure loop time */
        loop_end_us = (uint32_t)Timestamp_GetMicros();
        loop_time_us = loop_end_us - loop_start_us;

        app_status.cycleTime_us = loop_time_us;
        app_status.cycleCount++;

        if (loop_time_us > app_status.maxCycleTime_us)
        {
            app_status.maxCycleTime_us = loop_time_us;
        }

        /* Check for timing violation */
        if (loop_time_us > (SAFETY_MAX_LOOP_TIME_MS * 1000U))
        {
            ErrorHandler_LogError(ERROR_SAFETY_TIMING_VIOLATION, 0U, __LINE__, loop_time_us);
        }

        /* Sleep until next cycle (power efficient) */
        __WFI();
    }
}

/**
 * @brief Execute fast tasks (1ms period)
 */
void App_FastTasks(void)
{
    /* Safety monitor - highest priority */
    /* TODO: Add safety monitor checks */

    /* Refresh watchdogs */
    (void)Watchdog_RefreshAll();

    /* Read LEM sensors (1 kHz sampling) */
    (void)LEM_Update();
}

/**
 * @brief Execute medium tasks (10ms period)
 */
void App_MediumTasks(void)
{
    /* Sensor acquisition */
    /* TODO: Read digital inputs, BTT6200 current sense */

    /* Output control */
    /* TODO: Update BTT6200 outputs */

    /* Power monitoring */
    /* TODO: Monitor power rails */
}

/**
 * @brief Execute slow tasks (100ms period)
 */
void App_SlowTasks(void)
{
    /* CAN communication */
    /* TODO: Transmit status messages */

    /* Diagnostics */
    /* TODO: Check faults, update DTCs */

    /* Data logging */
    /* TODO: Log to FRAM */

    /* LED indicators */
    static uint32_t led_toggle_count = 0U;
    led_toggle_count++;

    if ((led_toggle_count % 5U) == 0U)  /* Toggle every 500ms */
    {
        (void)BSP_GPIO_TogglePin(GPIOH, GPIO_PIN_9);  /* Status LED */
    }
}

/**
 * @brief Get application state
 */
Status_t App_GetState(AppState_t *pState)
{
    Status_t status = STATUS_ERROR_PARAM;

    if (pState != NULL)
    {
        *pState = app_state;
        status = STATUS_OK;
    }

    return status;
}

/**
 * @brief Get application status
 */
Status_t App_GetStatus(AppStatus_t *pStatus)
{
    Status_t status = STATUS_ERROR_PARAM;

    if (pStatus != NULL)
    {
        *pStatus = app_status;
        status = STATUS_OK;
    }

    return status;
}

/**
 * @brief Request state transition
 */
Status_t App_RequestStateChange(AppState_t newState)
{
    Status_t status = STATUS_OK;

    /* Validate state transition */
    switch (app_state)
    {
        case APP_STATE_INIT:
            if (newState == APP_STATE_STARTUP)
            {
                app_state = newState;
            }
            else
            {
                status = STATUS_ERROR_INVALID_STATE;
            }
            break;

        case APP_STATE_STARTUP:
            if ((newState == APP_STATE_POWER_ON) || (newState == APP_STATE_ERROR))
            {
                app_state = newState;
            }
            else
            {
                status = STATUS_ERROR_INVALID_STATE;
            }
            break;

        case APP_STATE_POWER_ON:
            if ((newState == APP_STATE_RUNNING) || (newState == APP_STATE_ERROR))
            {
                app_state = newState;
            }
            else
            {
                status = STATUS_ERROR_INVALID_STATE;
            }
            break;

        case APP_STATE_RUNNING:
            if ((newState == APP_STATE_SHUTDOWN) || (newState == APP_STATE_ERROR) ||
                (newState == APP_STATE_SAFE))
            {
                app_state = newState;
            }
            else
            {
                status = STATUS_ERROR_INVALID_STATE;
            }
            break;

        default:
            status = STATUS_ERROR_INVALID_STATE;
            break;
    }

    return status;
}

/**
 * @brief Trigger application shutdown
 */
Status_t App_Shutdown(void)
{
    app_state = APP_STATE_SHUTDOWN;
    return STATUS_OK;
}

/**
 * @brief Enter safe state (emergency)
 */
void App_EnterSafeState(void)
{
    /* Disable all outputs */
    /* TODO: BTT6200_SetSafeState(); */

    /* Disable LEM sensors */
    (void)LEM_EnableSupply(false);

    /* Set state */
    app_state = APP_STATE_SAFE;

    /* Log error */
    ErrorHandler_LogError(ERROR_SAFETY_CRITICAL_FAULT, 0U, __LINE__, 0U);
}

/**
 * @brief Get application version
 */
Status_t App_GetVersion(Version_t *pVersion)
{
    Status_t status = STATUS_ERROR_PARAM;

    if (pVersion != NULL)
    {
        pVersion->major = FIRMWARE_VERSION_MAJOR;
        pVersion->minor = FIRMWARE_VERSION_MINOR;
        pVersion->patch = FIRMWARE_VERSION_PATCH;
        pVersion->reserved = 0U;

        status = STATUS_OK;
    }

    return status;
}

/*============================================================================*/
/* PRIVATE FUNCTIONS                                                          */
/*============================================================================*/

/**
 * @brief Initialize peripherals (BSP layer)
 */
static Status_t app_init_peripherals(void)
{
    Status_t status = STATUS_OK;

    /* Initialize GPIO */
    status = BSP_GPIO_Init();

    if (status == STATUS_OK)
    {
        /* Initialize ADC with DMA */
        ADC_Config_t adcConfig = {
            .mode = ADC_MODE_CONTINUOUS,
            .trigger = ADC_TRIGGER_SOFTWARE,
            .useDMA = true,
            .oversampling = true,
            .oversampleRatio = BSP_ADC_OVERSAMPLE_RATIO
        };
        status = BSP_ADC_Init(&adcConfig);
    }

    if (status == STATUS_OK)
    {
        /* Initialize I2C2 for FRAM and temperature sensor */
        I2C_Config_t i2cConfig = {
            .clockSpeed = BSP_I2C_SPEED_STANDARD,
            .addrMode = I2C_ADDR_7BIT,
            .useDMA = false,
            .ownAddress = 0x00U
        };
        status = BSP_I2C_Init(BSP_I2C_INSTANCE_2, &i2cConfig);
    }

    if (status == STATUS_OK)
    {
        /* Start ADC conversions */
        status = BSP_ADC_Start();
    }

    return status;
}

/**
 * @brief Initialize modules
 */
static Status_t app_init_modules(void)
{
    Status_t status = STATUS_OK;

    /* Initialize LEM HOYS sensors */
    status = LEM_Init();

    if (status == STATUS_OK)
    {
        /* Calibrate all LEM sensors at startup */
        /* TODO: Load calibration from FRAM instead */
        /* for (uint8_t i = 0; i < LEM_SENSOR_COUNT; i++)
        {
            (void)LEM_CalibrateSensor(i);
        } */
    }

    /* Initialize other modules */
    /* TODO: Initialize BTT6200, digital inputs, FRAM, etc. */

    return status;
}

/**
 * @brief Update application status
 */
static void app_update_status(void)
{
    app_status.state = app_state;
    app_status.uptime_ms = Timestamp_GetMillis();

    /* Check power good */
    /* TODO: Read power monitoring status */
    app_status.powerGood = true;  /* Placeholder */

    /* Check safety OK */
    app_status.safetyOK = true;  /* Placeholder */
}

/**
 * @brief Application state machine
 */
static void app_state_machine(void)
{
    switch (app_state)
    {
        case APP_STATE_INIT:
            /* Should not reach here */
            break;

        case APP_STATE_STARTUP:
            /* Startup sequence */
            /* TODO: Power sequencing, self-tests */
            (void)App_RequestStateChange(APP_STATE_POWER_ON);
            break;

        case APP_STATE_POWER_ON:
            /* Power on systems */
            /* TODO: Enable power rails, outputs */
            (void)App_RequestStateChange(APP_STATE_RUNNING);
            break;

        case APP_STATE_RUNNING:
            /* Normal operation */
            /* Check for faults */
            if (!app_status.powerGood || !app_status.safetyOK)
            {
                App_EnterSafeState();
            }
            break;

        case APP_STATE_IDLE:
            /* Low power mode */
            break;

        case APP_STATE_SHUTDOWN:
            /* Shutdown sequence */
            /* TODO: Disable outputs, save data */
            break;

        case APP_STATE_ERROR:
            /* Error handling */
            App_EnterSafeState();
            break;

        case APP_STATE_SAFE:
            /* Safe state - wait for reset */
            break;

        default:
            App_EnterSafeState();
            break;
    }
}

/*============================================================================*/
/* END OF FILE                                                                */
/*============================================================================*/
