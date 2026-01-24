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
#include "lem_sensor.h"
#include "digital_input.h"
#include "btt6200.h"
#include "pm_monitor.h"
#include "temp_sensor.h"
#include "ltc6811.h"
#include "ltc_scanner.h"
#include "fram_driver.h"
#include "data_logger.h"
#include "can_protocol.h"
#include "bcu_scheduler.h"
#include "bcu_timing_config.h"
#include "timestamp.h"
#include "rul.h"
#include <string.h>

/*============================================================================*/
/* EXTERNAL FUNCTION DECLARATIONS                                             */
/*============================================================================*/

/** @brief System clock configuration (defined in main.c) */
extern void SystemClock_Config(void);

/*============================================================================*/
/* PRIVATE VARIABLES                                                          */
/*============================================================================*/

/** @brief Application state (volatile - accessed from callbacks and tasks) */
static volatile AppState_t app_state = APP_STATE_INIT;

/** @brief Application status (volatile - accessed from multiple contexts) */
static volatile AppStatus_t app_status = {0};

/** @brief Fast task timestamp (1ms, volatile - timing critical) */
static volatile uint32_t fast_task_last_ms = 0U;

/** @brief Medium task timestamp (10ms, volatile - timing critical) */
static volatile uint32_t medium_task_last_ms = 0U;

/** @brief Slow task timestamp (100ms, volatile - timing critical) */
static volatile uint32_t slow_task_last_ms = 0U;

/** @brief RUL module context */
static RUL_Context_t rul_context = {0};

/*============================================================================*/
/* PRIVATE FUNCTION PROTOTYPES                                                */
/*============================================================================*/

static Status_t app_init_peripherals(void);
static Status_t app_init_modules(void);
static Status_t app_register_callbacks(void);
static void app_update_status(void);
static void app_state_machine(void);

/* Scheduler job functions */
static void Job_Critical_1ms(uint32_t now_ms);
static void Job_Fast_2ms(uint32_t now_ms);
static void Job_Medium_10ms(uint32_t now_ms);
static void Job_Slow_50ms(uint32_t now_ms);
static void Job_VerySlow_100ms(uint32_t now_ms);
static void Job_CANRx_EveryLoop(uint32_t now_ms);

/* Callback handlers */
static void app_lem_overcurrent_handler(uint8_t sensorId, Current_mA_t current);
static void app_btt_fault_handler(uint8_t channel, BTT6200_FaultType_t faultType);
static void app_power_fault_handler(uint8_t railId, PM_RailStatus_t status);
static void app_temp_alarm_handler(int32_t temp_mC, bool isOverTemp);
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
    status = Timestamp_ModuleInit();

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

        /* Start watchdogs (critical for safety) */
        Status_t wdStatus;
        wdStatus = Watchdog_StartIWDG();
        if (wdStatus != STATUS_OK)
        {
            ErrorHandler_LogError(ERROR_SAFETY_WATCHDOG, 0U, __LINE__, 0U);
        }

        wdStatus = Watchdog_StartWWDG();
        if (wdStatus != STATUS_OK)
        {
            ErrorHandler_LogError(ERROR_SAFETY_WATCHDOG, 1U, __LINE__, 0U);
        }

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
    Status_t status;

    /* Safety monitor - highest priority */
    status = SafetyMonitor_Execute();
    if (status != STATUS_OK)
    {
        ErrorHandler_LogError(ERROR_SAFETY_MONITOR_FAIL, 0U, __LINE__, 0U);
    }

    /* Refresh watchdogs (critical) */
    status = Watchdog_RefreshAll();
    if (status != STATUS_OK)
    {
        ErrorHandler_LogError(ERROR_SAFETY_WATCHDOG, 2U, __LINE__, 0U);
    }

    /* Read LEM sensors (1 kHz sampling) */
    status = LEM_Update();
    if (status != STATUS_OK)
    {
        /* Log but don't halt - sensor errors handled internally */
        ErrorHandler_LogError(ERROR_LEM_COMMUNICATION, 0U, __LINE__, 0U);
    }
}

/**
 * @brief Execute medium tasks (10ms period)
 */
void App_MediumTasks(void)
{
    Status_t status;

    /* Sensor acquisition */
    status = DI_Update();  /* Digital inputs with debouncing */
    if (status != STATUS_OK)
    {
        ErrorHandler_LogError(ERROR_DI_UPDATE_FAIL, 0U, __LINE__, 0U);
    }

    /* Output control */
    status = BTT6200_Update();  /* BTT6200 diagnostics */
    if (status != STATUS_OK)
    {
        ErrorHandler_LogError(ERROR_BTT6200_FAULT, 0U, __LINE__, 0U);
    }

    /* Power monitoring (critical for safety) */
    status = PM_Monitor_Update();  /* Power rails and LM74900 */
    if (status != STATUS_OK)
    {
        ErrorHandler_LogError(ERROR_POWER_MONITOR_FAIL, 0U, __LINE__, 0U);
        /* Power monitoring failure is critical */
        if (!app_status.safetyOK)
        {
            App_EnterSafeState();
        }
    }
}

/**
 * @brief Execute slow tasks (100ms period)
 * @note  DEPRECATED: This function is not called when using Scheduler_Run().
 *        Actual slow task execution happens in Job_VerySlow_100ms().
 *        Kept for API compatibility.
 */
void App_SlowTasks(void)
{
    /* NOTE: This function is legacy code from manual mainloop execution.
     * The scheduler-based execution (Scheduler_Run) uses Job_VerySlow_100ms() instead.
     * Data logging and LED toggling are handled there to avoid duplicate execution. */

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

    /* Data logging moved to Job_VerySlow_100ms() to avoid duplicate logging */

    /* LED indicators moved to Job_VerySlow_100ms() to avoid duplicate toggling */
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
    /* Refresh watchdog to prevent reset during safe state entry */
    (void)Watchdog_RefreshAll();

    /* Disable all outputs */
    (void)BTT6200_SetSafeState();

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
        /* Initialize FRAM storage (optional - may not be populated) */
        Status_t fram_status = FRAM_Init();
        if (fram_status == STATUS_OK)
        {
            /* Initialize configuration management (loads from FRAM) */
            (void)ConfigMgmt_Init();
        }
        else
        {
            /* Log warning but continue - FRAM may not be populated */
            (void)ErrorHandler_LogError(ERROR_FRAM_I2C_FAULT, 0U, 0U, 0U);
        }
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
        /* Initialize temperature sensor (optional - don't fail if not connected) */
        Status_t temp_status = TempSensor_Init();
        if (temp_status != STATUS_OK)
        {
            /* Log warning but continue - temp sensor may not be populated */
            (void)ErrorHandler_LogError(ERROR_DIAG_TEMP_SENSOR, 0U, 0U, 0U);
        }
    }

    /* LTC6811 scanner disabled for testing - causes DMA interrupt loop if not connected
    if (status == STATUS_OK)
    {
        LTCScanner_Config_t ltc_config = {
            .num_devices = 48U,
            .current_threshold_A = CURRENT_ACTIVE_THRESHOLD_A,
            .precharge_voltage_V = PRECHARGE_VOLTAGE_THRESHOLD_V,
            .transient_hold_ms = TRANSIENT_HOLD_MS,
            .enable_balancing = false
        };
        status = LTCScanner_Init(&ltc_config);
    }
    */

    if (status == STATUS_OK)
    {
        /* Initialize CAN bus */
        CAN_Config_t canConfig = {
            .bitrate = 500000U,
            .mode = BSP_CAN_MODE_NORMAL,
            .autoRetransmit = true,
            .rxFifo0Overrun = false,
            .rxFifo1Overrun = false,
            .txPriority = 0U
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
        /* Initialize RUL (Remaining Useful Life) module */
        status = RUL_Init(&rul_context);
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
    bool powerGood = false;
    uint32_t errorCount = 0U;

    app_status.state = app_state;
    app_status.uptime_ms = Timestamp_GetMillis();

    /* Check power good */
    (void)PM_Monitor_IsSystemPowerGood(&powerGood);
    app_status.powerGood = powerGood;

    /* Check safety OK */
    app_status.safetyOK = SafetyMonitor_IsSafe();

    /* Get active error count */
    (void)ErrorHandler_GetActiveDTCCount(&errorCount);
    app_status.activeErrors = (uint16_t)errorCount;
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
        {
            /* Startup sequence */
            /* Verify power is stable */
            bool powerGood = false;
            if (PM_Monitor_IsSystemPowerGood(&powerGood) == STATUS_OK)
            {
                if (powerGood)
                {
                    (void)App_RequestStateChange(APP_STATE_POWER_ON);
                }
            }
            break;
        }

        case APP_STATE_POWER_ON:
            /* Power on systems */
            /* Enable BTT6200 outputs (all off initially) */
            (void)BTT6200_EnableOutputs(true);
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
            /* Save any pending data to FRAM */
            (void)DataLog_LogData();
            /* Disable all outputs safely */
            (void)BTT6200_SetSafeState();
            (void)BTT6200_EnableOutputs(false);
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

    /* Register digital input callbacks for all 20 inputs (relay auxiliary contacts) */
    for (uint8_t i = 0U; i < 20U; i++)
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
static void app_temp_alarm_handler(int32_t temp_mC, bool isOverTemp)
{
    /* Log temperature fault */
    ErrorHandler_LogError(ERROR_TEMPERATURE_FAULT, (uint32_t)isOverTemp, (uint32_t)temp_mC, 0U);

    /* For overtemperature, enter safe state */
    if (isOverTemp)
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

    /* For critical errors, set error LED ON (non-blocking)
     * NOTE: HAL_Delay() was removed because it blocks the system.
     * If called from ISR context, HAL_Delay() would hang indefinitely.
     * The error LED state will be managed by the main loop or watchdog reset. */
    if ((code >= ERROR_SAFETY_WATCHDOG) && (code <= ERROR_SAFETY_CRITICAL_FAULT))
    {
        /* Turn error LED ON to indicate critical fault */
        (void)BSP_GPIO_WritePin(GPIOH, GPIO_PIN_10, GPIO_PIN_SET);
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
 * @details Called when a digital input (relay auxiliary contact) changes state.
 *          Inputs 0-19 correspond to relay auxiliary contacts.
 *          When contact closes (state=true), the corresponding relay is confirmed ON.
 *          When contact opens (state=false), the corresponding relay is confirmed OFF.
 */
static void app_di_state_change_handler(uint8_t inputId, bool state)
{
    /* Log state change for diagnostic purposes */
    /* Input ID maps to relay: DI0->Relay0, DI1->Relay1, etc. */

    if (inputId < 20U)
    {
        /* Get corresponding output state */
        BTT6200_ChannelState_t outputState;
        if (BTT6200_GetChannelState(inputId, &outputState) == STATUS_OK)
        {
            /* Check for mismatch between commanded output and feedback */
            bool outputOn = (outputState == BTT6200_STATE_ON) || (outputState == BTT6200_STATE_PWM);

            if (outputOn && !state)
            {
                /* Output ON but auxiliary contact open - relay may not have closed */
                /* This could indicate relay failure, wiring issue, or timing */
                /* Don't log immediately - debounce handles timing */
            }
            else if (!outputOn && state)
            {
                /* Output OFF but auxiliary contact closed - relay stuck ON */
                ErrorHandler_LogError(ERROR_ACTUATOR_FAULT, (uint32_t)inputId,
                                     (uint32_t)state, (uint32_t)outputState);
            }
        }
    }
}

/*============================================================================*/
/* END OF FILE                                                                */
/*============================================================================*/

/*============================================================================*/
/* SCHEDULER JOB FUNCTIONS                                                    */
/*============================================================================*/

/**
 * @brief Critical 1ms job - Safety and watchdog
 */
static void Job_Critical_1ms(uint32_t now_ms)
{
    uint32_t start_us = Scheduler_WCET_Start("Critical_1ms");

    /* Safety monitor - highest priority */
    Status_t status = SafetyMonitor_Execute();
    if (status != STATUS_OK)
    {
        ErrorHandler_LogError(ERROR_SAFETY_MONITOR_FAIL, 0U, __LINE__, 0U);
    }

    /* Refresh watchdogs */
    status = Watchdog_RefreshAll();
    if (status != STATUS_OK)
    {
        ErrorHandler_LogError(ERROR_SAFETY_WATCHDOG, 2U, __LINE__, 0U);
    }

    /* LTC scanner disabled for testing
    status = LTCScanner_Task_Step(now_ms);
    if (status != STATUS_OK)
    {
    }
    */
    (void)now_ms;

    (void)Scheduler_WCET_Stop("Critical_1ms", start_us, WCET_CRITICAL_MAX_US);
}

/**
 * @brief Fast 2ms job - LEM sensors
 */
static void Job_Fast_2ms(uint32_t now_ms)
{
    (void)now_ms;

    uint32_t start_us = Scheduler_WCET_Start("Fast_2ms");

    /* Read LEM sensors (1kHz sampling via 2ms period) */
    Status_t status = LEM_Update();
    if (status != STATUS_OK)
    {
        ErrorHandler_LogError(ERROR_LEM_COMMUNICATION, 0U, __LINE__, 0U);
    }

    /* LTC scanner disabled for testing
    Current_mA_t pack_current_mA = 0;
    if (LEM_ReadCurrent(0U, &pack_current_mA) == STATUS_OK)
    {
        float pack_current_A = (float)pack_current_mA / 1000.0f;
        (void)LTCScanner_UpdatePackCurrent(pack_current_A);
    }
    */

    (void)Scheduler_WCET_Stop("Fast_2ms", start_us, 0U);
}

/**
 * @brief Medium 10ms job - Digital inputs, power monitor, temp
 */
static void Job_Medium_10ms(uint32_t now_ms)
{
    (void)now_ms;

    uint32_t start_us = Scheduler_WCET_Start("Medium_10ms");

    /* Digital inputs with debouncing */
    Status_t status = DI_Update();
    if (status != STATUS_OK)
    {
        ErrorHandler_LogError(ERROR_DI_UPDATE_FAIL, 0U, __LINE__, 0U);
    }

    /* BTT6200 output diagnostics */
    status = BTT6200_Update();
    if (status != STATUS_OK)
    {
        ErrorHandler_LogError(ERROR_BTT6200_FAULT, 0U, __LINE__, 0U);
    }

    /* Power monitoring */
    status = PM_Monitor_Update();
    if (status != STATUS_OK)
    {
        ErrorHandler_LogError(ERROR_POWER_MONITOR_FAIL, 0U, __LINE__, 0U);
    }

    /* Temperature sensor */
    int32_t temp_mC;
    (void)TempSensor_ReadTemperature(&temp_mC);

    /* CAN TX periodic (status messages) */
    (void)CANProto_TransmitPeriodic();

    (void)Scheduler_WCET_Stop("Medium_10ms", start_us, 0U);
}

/**
 * @brief Slow 50ms job - CAN cell voltages
 */
static void Job_Slow_50ms(uint32_t now_ms)
{
    (void)now_ms;

    uint32_t start_us = Scheduler_WCET_Start("Slow_50ms");

    /* Transmit CAN cell voltage messages */
    /* CANProto would have specific function for this */
    
    /* Update status */
    app_update_status();

    (void)Scheduler_WCET_Stop("Slow_50ms", start_us, 0U);
}

/**
 * @brief Very slow 100ms job - Logging and diagnostics
 */
static void Job_VerySlow_100ms(uint32_t now_ms)
{
    static uint32_t log_counter = 0U;

    uint32_t start_us = Scheduler_WCET_Start("VerySlow_100ms");

    /* Periodic data logging to FRAM (every 10 seconds = 100 × 100ms) */
    log_counter++;
    if ((log_counter % 100U) == 0U)
    {
        (void)DataLog_LogData();
    }

    /* Diagnostics */
    (void)ErrorHandler_Update();  /* Age DTCs */

    /* Update timing for safety monitor */
    (void)SafetyMonitor_UpdateTiming(app_status.cycleTime_us);

    /* State machine */
    app_state_machine();

    /* Update RUL module with battery data (every 100ms) */
    {
        Current_mA_t pack_current_mA = 0;
        Voltage_mV_t pack_voltage_mV = 0;
        Temperature_t temperature = 0;
        Percentage_t soc_percent = 5000U;  /* 50% default, would come from BMS */

        /* Get pack current from LEM sensor */
        if (LEM_ReadCurrent(0U, &pack_current_mA) == STATUS_OK)
        {
            /* Update RUL with current battery data */
            (void)RUL_Update(&rul_context, pack_current_mA, pack_voltage_mV,
                           temperature, soc_percent);
        }

        /* Update operating time */
        (void)RUL_UpdateOperatingTime(&rul_context, 100U);  /* 100ms increment */

        /* Predict and send RUL every 1 second (10 × 100ms) */
        if ((log_counter % 10U) == 0U)
        {
            RUL_Prediction_t prediction;
            RUL_HealthMetrics_t health;

            if (RUL_PredictLife(&rul_context, &prediction) == STATUS_OK)
            {
                /* Get SoH for CAN message */
                if (RUL_GetHealthMetrics(&rul_context, &health) == STATUS_OK)
                {
                    /* Send RUL status via CAN (0x120) */
                    (void)CANProto_SendRULStatus(&prediction, health.soh);
                }
            }
        }
    }

    /* Send timing message every 100ms */
    {
        SchedulerStats_t stats;
        if (Scheduler_GetStats(&stats) == STATUS_OK)
        {
            (void)CANProto_SendTiming(stats.last_loop_time_us, stats.max_loop_time_us);
        }
    }

    /* LED indicators */
    if ((log_counter % 5U) == 0U)  /* Toggle every 500ms */
    {
        (void)BSP_GPIO_TogglePin(GPIOH, GPIO_PIN_9);  /* Status LED */
    }

    (void)Scheduler_WCET_Stop("VerySlow_100ms", start_us, 0U);
}

/**
 * @brief CAN RX processing - every loop (non-blocking)
 */
static void Job_CANRx_EveryLoop(uint32_t now_ms)
{
    (void)now_ms;

    /* Process incoming CAN messages (non-blocking) */
    (void)CANProto_ProcessRxMessages();
}

/**
 * @brief Register all scheduler jobs
 */
Status_t App_RegisterSchedulerJobs(void)
{
    Status_t status;

    /* Register critical 1ms job */
    status = Scheduler_RegisterJob("Critical_1ms", Job_Critical_1ms,
                                   SCHED_PERIOD_CRITICAL_MS, SCHED_PRIORITY_CRITICAL);
    if (status != STATUS_OK) { return status; }

    /* Register fast 2ms job */
    status = Scheduler_RegisterJob("Fast_2ms", Job_Fast_2ms,
                                   SCHED_PERIOD_FAST_MS, SCHED_PRIORITY_FAST);
    if (status != STATUS_OK) { return status; }

    /* Register medium 10ms job */
    status = Scheduler_RegisterJob("Medium_10ms", Job_Medium_10ms,
                                   SCHED_PERIOD_MEDIUM_MS, SCHED_PRIORITY_MEDIUM);
    if (status != STATUS_OK) { return status; }

    /* Register slow 50ms job */
    status = Scheduler_RegisterJob("Slow_50ms", Job_Slow_50ms,
                                   SCHED_PERIOD_SLOW_MS, SCHED_PRIORITY_SLOW);
    if (status != STATUS_OK) { return status; }

    /* Register very slow 100ms job */
    status = Scheduler_RegisterJob("VerySlow_100ms", Job_VerySlow_100ms,
                                   SCHED_PERIOD_VERY_SLOW_MS, SCHED_PRIORITY_VERY_SLOW);
    if (status != STATUS_OK) { return status; }

    /* Register CAN RX processing (every 1ms) */
    status = Scheduler_RegisterJob("CANRx", Job_CANRx_EveryLoop,
                                   CAN_RX_PERIOD_MS, SCHED_PRIORITY_CRITICAL);

    return status;
}
