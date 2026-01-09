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
#include "app_config_mgmt.h"
#include "app_errors.h"
#include "bsp_gpio.h"
#include "bsp_adc.h"
#include "bsp_i2c.h"
#include "bsp_can.h"
#include "watchdog.h"
#include "safety_monitor.h"
#include "self_test.h"
#include "lem_sensor.h"
#include "digital_input.h"
#include "btt6200.h"
#include "pm_monitor.h"
#include "temp_sensor.h"
#include "fram_driver.h"
#include "can_protocol.h"
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
static Status_t app_register_callbacks(void);
static void app_update_status(void);
static void app_state_machine(void);

/* Callback handlers */
static void app_lem_overcurrent_handler(uint8_t sensorId, Current_mA_t current);
static void app_btt_fault_handler(uint8_t channel, BTT6200_FaultType_t faultType);
static void app_power_fault_handler(uint8_t railId, PM_RailStatus_t status);
static void app_temp_alarm_handler(TempSensor_AlarmType_t alarmType, int32_t temp_mC);
static void app_error_handler(ErrorCode_t code, uint32_t p1, uint32_t p2, uint32_t p3);
static void app_safety_fault_handler(SafetyLevel_t level, const char *pName);
static void app_di_state_change_handler(uint8_t inputId, bool state);

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
    (void)SafetyMonitor_Execute();

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
    (void)DI_Update();  /* Digital inputs with debouncing */

    /* Output control */
    (void)BTT6200_Update();  /* BTT6200 diagnostics */

    /* Power monitoring */
    (void)PM_Monitor_Update();  /* Power rails and LM74900 */
}

/**
 * @brief Execute slow tasks (100ms period)
 */
void App_SlowTasks(void)
{
    /* Temperature monitoring */
    int32_t temperature_mC;
    (void)TempSensor_ReadTemperature(&temperature_mC);

    /* CAN communication */
    (void)CANProto_ProcessRxMessages();    /* Process incoming CAN messages */
    (void)CANProto_TransmitPeriodic();     /* Transmit periodic status messages */

    /* Diagnostics */
    (void)ErrorHandler_Update();  /* Age DTCs */

    /* Update timing for safety monitor */
    (void)SafetyMonitor_UpdateTiming(app_status.cycleTime_us);

    /* Data logging */
    /* TODO: Periodic data logging to FRAM */

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

    /* Initialize error handler first */
    status = ErrorHandler_Init();

    if (status == STATUS_OK)
    {
        /* Initialize safety monitor */
        status = SafetyMonitor_Init();
    }

    if (status == STATUS_OK)
    {
        /* Initialize self-test module */
        status = SelfTest_Init();
    }

    if (status == STATUS_OK)
    {
        /* Initialize FRAM storage */
        status = FRAM_Init();
    }

    if (status == STATUS_OK)
    {
        /* Initialize configuration management (loads from FRAM) */
        status = ConfigMgmt_Init();
    }

    if (status == STATUS_OK)
    {
        /* Initialize LEM HOYS sensors */
        status = LEM_Init();

        if (status == STATUS_OK)
        {
            /* Load calibration from configuration */
            SystemConfig_t config;
            if (ConfigMgmt_Load(&config) == STATUS_OK)
            {
                /* Apply LEM calibration from config */
                for (uint8_t i = 0U; i < LEM_SENSOR_COUNT; i++)
                {
                    if (config.lem_calibration[i].valid)
                    {
                        (void)LEM_SetCalibration(i, &config.lem_calibration[i]);
                    }
                    else
                    {
                        /* Perform zero-current calibration */
                        (void)LEM_CalibrateSensor(i);
                    }
                }
            }
        }
    }

    if (status == STATUS_OK)
    {
        /* Initialize BTT6200 output driver */
        status = BTT6200_Init();
    }

    if (status == STATUS_OK)
    {
        /* Initialize digital inputs */
        status = DI_Init();
    }

    if (status == STATUS_OK)
    {
        /* Initialize power monitoring */
        status = PM_Monitor_Init();
    }

    if (status == STATUS_OK)
    {
        /* Initialize temperature sensor */
        status = TempSensor_Init();
    }

    if (status == STATUS_OK)
    {
        /* Initialize CAN bus */
        CAN_Config_t canConfig = {
            .baudrate = 500000U,
            .loopback = false,
            .autoRetransmit = true,
            .autoBusOff = true
        };
        status = BSP_CAN_Init(BSP_CAN_INSTANCE_1, &canConfig);

        /* CAN2 is optional */
        if (status == STATUS_OK)
        {
            (void)BSP_CAN_Init(BSP_CAN_INSTANCE_2, &canConfig);
        }
    }

    if (status == STATUS_OK)
    {
        /* Initialize CAN protocol stack */
        status = CANProto_Init();
    }

    if (status == STATUS_OK)
    {
        /* Register all callbacks */
        status = app_register_callbacks();
    }

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
    (void)PM_Monitor_IsSystemPowerGood(&app_status.powerGood);

    /* Check safety OK */
    app_status.safetyOK = SafetyMonitor_IsSafe();

    /* Get active error count */
    (void)ErrorHandler_GetActiveDTCCount(&app_status.activeErrors);
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
            /* Execute Power-On Self-Test (POST) */
            {
                static bool postExecuted = false;

                if (!postExecuted)
                {
                    POSTResults_t postResults = {0};
                    Status_t postStatus = SelfTest_RunPOST(&postResults);

                    postExecuted = true;

                    if (postStatus != STATUS_OK)
                    {
                        /* POST failed - log critical error */
                        ErrorHandler_LogError(ERROR_SELF_TEST_FAILED, 0U,
                                            postResults.failedTests,
                                            postResults.executionTime);

                        /* Enter safe state if critical tests failed */
                        uint32_t criticalMask = (1U << POST_TEST_RAM) |
                                              (1U << POST_TEST_FLASH) |
                                              (1U << POST_TEST_WATCHDOG);

                        if ((postResults.failedTests & criticalMask) != 0U)
                        {
                            App_EnterSafeState();
                            break;
                        }
                    }
                }

                /* Transition to power-on state */
                (void)App_RequestStateChange(APP_STATE_POWER_ON);
            }
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
/* CALLBACK FUNCTIONS                                                         */
/*============================================================================*/

/**
 * @brief Register all module callbacks
 */
static Status_t app_register_callbacks(void)
{
    /* Register LEM overcurrent callback */
    (void)LEM_RegisterOvercurrentCallback(app_lem_overcurrent_handler);

    /* Register BTT6200 fault callback */
    (void)BTT6200_RegisterFaultCallback(app_btt_fault_handler);

    /* Register power monitoring fault callback */
    (void)PM_Monitor_RegisterFaultCallback(app_power_fault_handler);

    /* Register temperature alarm callback */
    (void)TempSensor_RegisterAlarmCallback(app_temp_alarm_handler);

    /* Register error handler callback */
    (void)ErrorHandler_RegisterCallback(app_error_handler);

    /* Register safety monitor fault callback */
    (void)SafetyMonitor_RegisterFaultCallback(app_safety_fault_handler);

    /* Register digital input callbacks (example for first 5 inputs) */
    for (uint8_t i = 0U; i < 5U; i++)
    {
        (void)DI_RegisterCallback(i, app_di_state_change_handler);
    }

    return STATUS_OK;
}

/**
 * @brief LEM overcurrent callback handler
 */
static void app_lem_overcurrent_handler(uint8_t sensorId, Current_mA_t current)
{
    /* Log overcurrent fault */
    ErrorHandler_LogError(ERROR_SENSOR_FAULT, (uint32_t)sensorId, (uint32_t)current, 0U);

    /* Turn on warning LED */
    (void)BSP_GPIO_WritePin(GPIOH, GPIO_PIN_10, GPIO_STATE_HIGH);
}

/**
 * @brief BTT6200 fault callback handler
 */
static void app_btt_fault_handler(uint8_t channel, BTT6200_FaultType_t faultType)
{
    /* Log actuator fault */
    ErrorHandler_LogError(ERROR_ACTUATOR_FAULT, (uint32_t)channel, (uint32_t)faultType, 0U);

    /* For short circuit, enter safe state */
    if (faultType == BTT6200_FAULT_SHORT_CIRCUIT)
    {
        App_EnterSafeState();
    }
}

/**
 * @brief Power monitoring fault callback handler
 */
static void app_power_fault_handler(uint8_t railId, PM_RailStatus_t status)
{
    /* Log power fault */
    ErrorHandler_LogError(ERROR_POWER_FAULT, (uint32_t)railId, (uint32_t)status, 0U);

    /* For critical rails (12V input, 3V3 digital), enter safe state */
    if ((railId == PM_RAIL_INPUT_12V) || (railId == PM_RAIL_3V3_DIGITAL))
    {
        if ((status == PM_RAIL_STATUS_UNDERVOLTAGE) || (status == PM_RAIL_STATUS_OVERVOLTAGE))
        {
            App_EnterSafeState();
        }
    }
}

/**
 * @brief Temperature alarm callback handler
 */
static void app_temp_alarm_handler(TempSensor_AlarmType_t alarmType, int32_t temp_mC)
{
    /* Log temperature fault */
    ErrorHandler_LogError(ERROR_TEMPERATURE_FAULT, (uint32_t)alarmType, (uint32_t)temp_mC, 0U);

    /* For overtemperature, enter safe state */
    if (alarmType == TEMP_ALARM_HIGH)
    {
        App_EnterSafeState();
    }
}

/**
 * @brief Error handler callback
 */
static void app_error_handler(ErrorCode_t code, uint32_t p1, uint32_t p2, uint32_t p3)
{
    /* Transmit fault message via CAN */
    (void)CANProto_SendFaults();

    /* For critical errors, blink error LED faster */
    if ((code >= ERROR_SAFETY_WATCHDOG) && (code <= ERROR_SAFETY_CRITICAL_FAULT))
    {
        /* Rapid blink */
        for (uint8_t i = 0U; i < 6U; i++)
        {
            (void)BSP_GPIO_TogglePin(GPIOH, GPIO_PIN_10);
            HAL_Delay(50);
        }
    }

    (void)p1;  /* Suppress unused warnings */
    (void)p2;
    (void)p3;
}

/**
 * @brief Safety monitor fault callback handler
 */
static void app_safety_fault_handler(SafetyLevel_t level, const char *pName)
{
    /* Log safety monitor fault */
    ErrorHandler_LogError(ERROR_SAFETY_MONITOR_FAULT, (uint32_t)level, 0U, 0U);

    /* For critical level, enter safe state */
    if (level == SAFETY_LEVEL_CRITICAL)
    {
        App_EnterSafeState();
    }

    (void)pName;  /* Suppress unused warning */
}

/**
 * @brief Digital input state change callback handler
 */
static void app_di_state_change_handler(uint8_t inputId, bool state)
{
    /* Example: Log state changes for important inputs */
    /* This can be used for emergency stop, ignition key, etc. */

    (void)inputId;  /* Suppress unused warnings */
    (void)state;
}

/*============================================================================*/
/* END OF FILE                                                                */
/*============================================================================*/
