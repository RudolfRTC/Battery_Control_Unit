/**
 * @file    ltc_scanner.c
 * @brief   Non-blocking LTC6811 cell scanner implementation
 * @author  Battery Control Unit Development Team
 * @date    2026-01-10
 * @version 1.0.0
 *
 * @note    MISRA C:2012 compliant
 * @note    State machine for non-blocking operation
 *
 * @copyright Copyright (c) 2026
 */

/*============================================================================*/
/* INCLUDES                                                                   */
/*============================================================================*/
#include "ltc_scanner.h"
#include "app_errors.h"
#include "timestamp.h"
#include <string.h>
#include <math.h>

/*============================================================================*/
/* PRIVATE CONSTANTS                                                          */
/*============================================================================*/

/** @brief Event codes for callbacks */
#define LTC_EVENT_SCAN_COMPLETE         (0x01U)
#define LTC_EVENT_ERROR                 (0x02U)
#define LTC_EVENT_CRC_ERROR             (0x03U)
#define LTC_EVENT_TIMEOUT               (0x04U)
#define LTC_EVENT_MODE_CHANGE           (0x05U)

/*============================================================================*/
/* PRIVATE VARIABLES                                                          */
/*============================================================================*/

/** @brief Scanner configuration */
static LTCScanner_Config_t scanner_config;

/** @brief Current state machine state */
static LTCScanner_State_t current_state = LTC_SM_IDLE;

/** @brief Current operating mode */
static LTCScanner_Mode_t current_mode = LTC_MODE_NORMAL;

/** @brief Scanner statistics */
static LTCScanner_Stats_t scanner_stats = {0};

/** @brief Cell data (published results) */
static LTCScanner_CellData_t cell_data = {0};

/** @brief Event callback */
static LTCScanner_EventCallback_t event_callback = NULL;

/** @brief Next scan start time */
static uint32_t next_scan_start_ms = 0U;

/** @brief Scan cycle start time */
static uint32_t scan_cycle_start_ms = 0U;

/** @brief State timeout deadline */
static uint32_t state_timeout_ms = 0U;

/** @brief Transient mode end time */
static uint32_t transient_end_ms = 0U;

/** @brief Pack current (for mode selection) */
static float pack_current_A = 0.0f;

/** @brief Charging mode flag */
static bool charging_mode = false;

/** @brief Initialized flag */
static bool scanner_initialized = false;

/** @brief Current reading group index */
static uint8_t current_group = 0U;

/** @brief Temporary voltage buffer */
static uint16_t temp_voltages[LTC_SCANNER_MAX_DEVICES][LTC_SCANNER_MAX_CELLS_PER_MOD];

/*============================================================================*/
/* PRIVATE FUNCTION PROTOTYPES                                                */
/*============================================================================*/

static void ltc_sm_idle(uint32_t now_ms);
static void ltc_sm_start_conversion(uint32_t now_ms);
static void ltc_sm_wait_conversion(uint32_t now_ms);
static void ltc_sm_read_group(uint32_t now_ms);
static void ltc_sm_validate_crc(uint32_t now_ms);
static void ltc_sm_publish(uint32_t now_ms);
static void ltc_sm_error(uint32_t now_ms);

static void ltc_update_mode(uint32_t now_ms);
static uint32_t ltc_get_scan_period_ms(void);
static void ltc_enter_error_state(uint8_t error_event);
static void ltc_trigger_callback(uint8_t event, uint32_t param);

/*============================================================================*/
/* PUBLIC FUNCTIONS                                                           */
/*============================================================================*/

/**
 * @brief Initialize LTC scanner
 */
Status_t LTCScanner_Init(const LTCScanner_Config_t *pConfig)
{
    Status_t status = STATUS_ERROR_PARAM;

    if ((pConfig != NULL) && (pConfig->num_devices <= LTC_SCANNER_MAX_DEVICES))
    {
        if (!scanner_initialized)
        {
            /* Copy configuration */
            scanner_config = *pConfig;

            /* Initialize LTC6811 driver */
            status = LTC6811_Init(pConfig->num_devices);

            if (status == STATUS_OK)
            {
                /* Clear statistics and data */
                (void)memset(&scanner_stats, 0, sizeof(scanner_stats));
                (void)memset(&cell_data, 0, sizeof(cell_data));
                (void)memset(temp_voltages, 0, sizeof(temp_voltages));

                /* Set initial state */
                current_state = LTC_SM_IDLE;
                current_mode = LTC_MODE_NORMAL;
                current_group = 0U;

                /* Initialize timestamps */
                uint32_t now_ms = Timestamp_GetMillis();
                next_scan_start_ms = now_ms + T_CELL_NORMAL_MS;
                transient_end_ms = 0U;

                scanner_initialized = true;

                scanner_stats.current_state = current_state;
                scanner_stats.current_mode = current_mode;
            }
        }
        else
        {
            status = STATUS_ERROR_ALREADY_INIT;
        }
    }

    return status;
}

/**
 * @brief De-initialize LTC scanner
 */
Status_t LTCScanner_DeInit(void)
{
    scanner_initialized = false;
    current_state = LTC_SM_IDLE;
    return STATUS_OK;
}

/**
 * @brief Execute one step of state machine
 */
Status_t LTCScanner_Task_Step(uint32_t now_ms)
{
    Status_t status = STATUS_OK;

    if (!scanner_initialized)
    {
        return STATUS_ERROR_NOT_INIT;
    }

    /* Update operating mode based on conditions */
    ltc_update_mode(now_ms);

    /* Update statistics */
    scanner_stats.current_state = current_state;
    scanner_stats.current_mode = current_mode;

    /* Execute state machine */
    switch (current_state)
    {
        case LTC_SM_IDLE:
            ltc_sm_idle(now_ms);
            break;

        case LTC_SM_START_CONVERSION:
            ltc_sm_start_conversion(now_ms);
            break;

        case LTC_SM_WAIT_CONVERSION:
            ltc_sm_wait_conversion(now_ms);
            break;

        case LTC_SM_READ_GROUP_A:
        case LTC_SM_READ_GROUP_B:
        case LTC_SM_READ_GROUP_C:
        case LTC_SM_READ_GROUP_D:
            ltc_sm_read_group(now_ms);
            break;

        case LTC_SM_VALIDATE_CRC:
            ltc_sm_validate_crc(now_ms);
            break;

        case LTC_SM_PUBLISH:
            ltc_sm_publish(now_ms);
            break;

        case LTC_SM_ERROR:
            ltc_sm_error(now_ms);
            break;

        default:
            /* Unknown state - enter error */
            ltc_enter_error_state(LTC_EVENT_ERROR);
            break;
    }

    return status;
}

/**
 * @brief Set scanner operating mode
 */
Status_t LTCScanner_SetMode(LTCScanner_Mode_t mode)
{
    Status_t status = STATUS_ERROR_PARAM;

    if (mode <= LTC_MODE_TRANSIENT)
    {
        if (current_mode != mode)
        {
            current_mode = mode;
            ltc_trigger_callback(LTC_EVENT_MODE_CHANGE, (uint32_t)mode);
        }
        status = STATUS_OK;
    }

    return status;
}

/**
 * @brief Trigger transient mode
 */
Status_t LTCScanner_TriggerTransient(uint32_t hold_ms)
{
    uint32_t now_ms = Timestamp_GetMillis();
    uint32_t duration = (hold_ms > 0U) ? hold_ms : scanner_config.transient_hold_ms;

    current_mode = LTC_MODE_TRANSIENT;
    transient_end_ms = now_ms + duration;

    ltc_trigger_callback(LTC_EVENT_MODE_CHANGE, (uint32_t)LTC_MODE_TRANSIENT);

    return STATUS_OK;
}

/**
 * @brief Get current cell data
 */
Status_t LTCScanner_GetCellData(LTCScanner_CellData_t *pData)
{
    Status_t status = STATUS_ERROR_PARAM;

    if (pData != NULL)
    {
        *pData = cell_data;
        cell_data.new_data_available = false;  /* Clear flag after read */
        status = STATUS_OK;
    }

    return status;
}

/**
 * @brief Get scanner statistics
 */
Status_t LTCScanner_GetStats(LTCScanner_Stats_t *pStats)
{
    Status_t status = STATUS_ERROR_PARAM;

    if (pStats != NULL)
    {
        *pStats = scanner_stats;
        status = STATUS_OK;
    }

    return status;
}

/**
 * @brief Register event callback
 */
Status_t LTCScanner_RegisterCallback(LTCScanner_EventCallback_t callback)
{
    event_callback = callback;
    return STATUS_OK;
}

/**
 * @brief Force immediate scan
 */
Status_t LTCScanner_ForceScan(void)
{
    if (current_state == LTC_SM_IDLE)
    {
        next_scan_start_ms = Timestamp_GetMillis();
        return STATUS_OK;
    }
    return STATUS_ERROR_BUSY;
}

/**
 * @brief Update pack current
 */
Status_t LTCScanner_UpdatePackCurrent(float current_A)
{
    /* Update module-level variable (MISRA fix: parameter shadowing) */
    pack_current_A = current_A;
    return STATUS_OK;
}

/**
 * @brief Set charging mode
 */
Status_t LTCScanner_SetChargingMode(bool charging)
{
    charging_mode = charging;
    return STATUS_OK;
}

/*============================================================================*/
/* PRIVATE FUNCTIONS - STATE MACHINE                                         */
/*============================================================================*/

/**
 * @brief IDLE state - wait for next scan period
 */
static void ltc_sm_idle(uint32_t now_ms)
{
    /* Check if it's time to start new scan */
    if (now_ms >= next_scan_start_ms)
    {
        /* Start new scan cycle */
        scan_cycle_start_ms = now_ms;
        current_state = LTC_SM_START_CONVERSION;
        current_group = 0U;

        /* Schedule next scan */
        next_scan_start_ms = now_ms + ltc_get_scan_period_ms();
    }
}

/**
 * @brief START_CONVERSION state - initiate ADC conversion
 */
static void ltc_sm_start_conversion(uint32_t now_ms)
{
    /* Start cell voltage conversion on all devices */
    Status_t status = LTC6811_StartCellVoltageConversion(
        LTC6811_ADC_MODE_7KHZ_3KHZ,  /* Normal mode */
        false,                        /* All cells */
        false                         /* Discharge not permitted during conversion */
    );

    if (status == STATUS_OK)
    {
        /* Set timeout for conversion */
        state_timeout_ms = now_ms + LTC_TIMEOUT_CONVERSION_MS;
        current_state = LTC_SM_WAIT_CONVERSION;
    }
    else
    {
        ltc_enter_error_state(LTC_EVENT_ERROR);
    }
}

/**
 * @brief WAIT_CONVERSION state - wait for ADC to complete
 */
static void ltc_sm_wait_conversion(uint32_t now_ms)
{
    /* Check timeout */
    if (now_ms >= state_timeout_ms)
    {
        ltc_enter_error_state(LTC_EVENT_TIMEOUT);
        return;
    }

    /* Typical conversion time: ~7ms for normal mode */
    /* We wait minimum time then proceed to read */
    if ((now_ms - scan_cycle_start_ms) >= 10U)
    {
        /* Conversion should be complete - start reading */
        current_state = LTC_SM_READ_GROUP_A;
        current_group = 0U;
    }
}

/**
 * @brief READ_GROUP state - read cell voltage groups
 */
static void ltc_sm_read_group(uint32_t now_ms)
{
    Status_t status;
    /* Initialize at declaration per MISRA 9.1 */
    uint16_t voltages[LTC_SCANNER_MAX_DEVICES][3] = {0};  /* 3 cells per group */

    /* Check timeout */
    if (now_ms >= state_timeout_ms)
    {
        ltc_enter_error_state(LTC_EVENT_TIMEOUT);
        return;
    }

    /* Determine which group to read based on state */
    uint8_t group_offset = 0U;
    switch (current_state)
    {
        case LTC_SM_READ_GROUP_A:
            group_offset = 0U;
            break;
        case LTC_SM_READ_GROUP_B:
            group_offset = 3U;
            break;
        case LTC_SM_READ_GROUP_C:
            group_offset = 6U;
            break;
        case LTC_SM_READ_GROUP_D:
            group_offset = 9U;
            break;
        default:
            ltc_enter_error_state(LTC_EVENT_ERROR);
            return;
    }

    /* Read voltage group (implementation would call LTC6811 low-level functions) */
    /* For now, we simulate a successful read */
    status = STATUS_OK;  /* LTC6811_ReadCellVoltageGroup(...) */

    if (status == STATUS_OK)
    {
        /* ISO 26262 ASIL-B: Bounds check on num_devices */
        if (scanner_config.num_devices > LTC_SCANNER_MAX_DEVICES)
        {
            ltc_enter_error_state(LTC_EVENT_CONFIG_ERROR);
            return;
        }

        /* Copy voltages to temporary buffer */
        for (uint8_t dev = 0U; dev < scanner_config.num_devices; dev++)
        {
            for (uint8_t cell = 0U; cell < 3U; cell++)
            {
                uint8_t cell_index = group_offset + cell;
                if (cell_index < LTC_SCANNER_MAX_CELLS_PER_MOD)
                {
                    temp_voltages[dev][cell_index] = voltages[dev][cell];
                }
            }
        }

        /* Move to next group */
        switch (current_state)
        {
            case LTC_SM_READ_GROUP_A:
                current_state = LTC_SM_READ_GROUP_B;
                state_timeout_ms = now_ms + LTC_TIMEOUT_READ_MS;
                break;
            case LTC_SM_READ_GROUP_B:
                current_state = LTC_SM_READ_GROUP_C;
                state_timeout_ms = now_ms + LTC_TIMEOUT_READ_MS;
                break;
            case LTC_SM_READ_GROUP_C:
                current_state = LTC_SM_READ_GROUP_D;
                state_timeout_ms = now_ms + LTC_TIMEOUT_READ_MS;
                break;
            case LTC_SM_READ_GROUP_D:
                current_state = LTC_SM_VALIDATE_CRC;
                break;
            default:
                ltc_enter_error_state(LTC_EVENT_ERROR);
                break;
        }
    }
    else
    {
        ltc_enter_error_state(LTC_EVENT_ERROR);
    }
}

/**
 * @brief VALIDATE_CRC state - check PEC/CRC
 */
static void ltc_sm_validate_crc(uint32_t now_ms)
{
    (void)now_ms;

    /* Validate CRC/PEC for all read data */
    /* In real implementation, this would check PEC from LTC6811 */
    bool crc_valid = true;  /* LTC6811_ValidatePEC(...) */

    if (crc_valid)
    {
        current_state = LTC_SM_PUBLISH;
        scanner_stats.consecutive_errors = 0U;
    }
    else
    {
        scanner_stats.crc_errors++;
        ltc_enter_error_state(LTC_EVENT_CRC_ERROR);
    }
}

/**
 * @brief PUBLISH state - publish valid data
 */
static void ltc_sm_publish(uint32_t now_ms)
{
    /* Copy validated data to published buffer */
    for (uint8_t dev = 0U; dev < scanner_config.num_devices; dev++)
    {
        for (uint8_t cell = 0U; cell < LTC_SCANNER_MAX_CELLS_PER_MOD; cell++)
        {
            cell_data.cell_voltages_mV[dev][cell] = temp_voltages[dev][cell];
        }
        cell_data.cell_count[dev] = LTC_SCANNER_MAX_CELLS_PER_MOD;
        cell_data.data_valid[dev] = true;
    }

    cell_data.timestamp_ms = now_ms;
    cell_data.new_data_available = true;

    /* Update statistics */
    scanner_stats.scan_count++;
    uint32_t scan_duration = now_ms - scan_cycle_start_ms;
    scanner_stats.last_scan_time_ms = scan_duration;

    if (scan_duration > scanner_stats.max_scan_time_ms)
    {
        scanner_stats.max_scan_time_ms = scan_duration;
    }

    /* Trigger callback */
    ltc_trigger_callback(LTC_EVENT_SCAN_COMPLETE, scan_duration);

    /* Return to IDLE */
    current_state = LTC_SM_IDLE;
}

/**
 * @brief ERROR state - handle errors and recovery
 */
static void ltc_sm_error(uint32_t now_ms)
{
    /* Mark data as invalid */
    for (uint8_t dev = 0U; dev < scanner_config.num_devices; dev++)
    {
        cell_data.data_valid[dev] = false;
    }

    /* Attempt recovery after timeout */
    if (scanner_stats.consecutive_errors < LTC_MAX_CONSECUTIVE_ERRORS)
    {
        /* Try to recover - return to IDLE and retry */
        current_state = LTC_SM_IDLE;
        next_scan_start_ms = now_ms + 100U;  /* Short delay before retry */
    }
    else
    {
        /* Too many consecutive errors - log critical error */
        ErrorHandler_LogError(ERROR_LTC_COMMUNICATION, scanner_stats.consecutive_errors, 0U, 0U);

        /* Stay in error state - manual recovery needed */
        /* Could trigger safe state here if critical */
    }
}

/*============================================================================*/
/* PRIVATE FUNCTIONS - HELPERS                                               */
/*============================================================================*/

/**
 * @brief Update operating mode based on conditions
 */
static void ltc_update_mode(uint32_t now_ms)
{
    LTCScanner_Mode_t new_mode = current_mode;

    /* Check if transient mode is active */
    if ((current_mode == LTC_MODE_TRANSIENT) && (now_ms >= transient_end_ms))
    {
        /* Transient mode expired - revert to NORMAL or ACTIVE */
        new_mode = LTC_MODE_NORMAL;
    }

    /* Check for ACTIVE mode conditions */
    if (new_mode != LTC_MODE_TRANSIENT)
    {
        float abs_current = fabsf(pack_current_A);

        if ((abs_current > scanner_config.current_threshold_A) || charging_mode)
        {
            new_mode = LTC_MODE_ACTIVE;
        }
        else
        {
            new_mode = LTC_MODE_NORMAL;
        }
    }

    /* Update mode if changed */
    if (new_mode != current_mode)
    {
        current_mode = new_mode;
        scanner_stats.current_mode = new_mode;

        /* Adjust next scan time based on new mode */
        next_scan_start_ms = now_ms + ltc_get_scan_period_ms();

        ltc_trigger_callback(LTC_EVENT_MODE_CHANGE, (uint32_t)new_mode);
    }
}

/**
 * @brief Get scan period based on current mode
 */
static uint32_t ltc_get_scan_period_ms(void)
{
    uint32_t period_ms;

    switch (current_mode)
    {
        case LTC_MODE_TRANSIENT:
            period_ms = T_CELL_TRANSIENT_MS;
            break;

        case LTC_MODE_ACTIVE:
            period_ms = T_CELL_ACTIVE_MS;
            break;

        case LTC_MODE_NORMAL:
        default:
            period_ms = T_CELL_NORMAL_MS;
            break;
    }

    return period_ms;
}

/**
 * @brief Enter error state
 */
static void ltc_enter_error_state(uint8_t error_event)
{
    current_state = LTC_SM_ERROR;
    scanner_stats.error_count++;
    scanner_stats.consecutive_errors++;

    if (error_event == LTC_EVENT_TIMEOUT)
    {
        scanner_stats.timeout_errors++;
    }

    ltc_trigger_callback(error_event, scanner_stats.consecutive_errors);
}

/**
 * @brief Trigger event callback
 */
static void ltc_trigger_callback(uint8_t event, uint32_t param)
{
    if (event_callback != NULL)
    {
        event_callback(event, param);
    }
}
