/**
 * @file    pm_monitor.c
 * @brief   Power supply monitoring module implementation
 * @author  Battery Control Unit Development Team
 * @date    2026-01-09
 * @version 1.0.0
 *
 * @note    MISRA C:2012 compliant
 * @note    ISO 26262 ASIL-B safety requirements
 * @note    Monitors LM74900, power rails, and fault conditions
 *
 * @copyright Copyright (c) 2026
 */

/*============================================================================*/
/* INCLUDES                                                                   */
/*============================================================================*/
#include "pm_monitor.h"
#include "bsp_gpio.h"
#include "bsp_adc.h"
#include "filter.h"
#include <string.h>

/*============================================================================*/
/* PRIVATE CONSTANTS                                                          */
/*============================================================================*/

/** @brief ADC channel for IMON (power current monitor) */
#define PM_IMON_ADC_CHANNEL     (0U)

/** @brief LM74900 IMON sensitivity (mV per A) */
#define PM_IMON_SENSITIVITY     (100U)  /* 100 mV/A typical */

/*============================================================================*/
/* PRIVATE VARIABLES                                                          */
/*============================================================================*/

/** @brief Power monitoring data */
static PM_MonitorData_t pm_data;

/** @brief Power rail filters */
static Filter_IIR_t pm_rail_filters[PM_RAIL_COUNT];

/** @brief Fault callback */
static PM_FaultCallback_t pm_fault_callback = NULL;

/** @brief Initialization flag */
static bool pm_initialized = false;

/** @brief Uptime counter (seconds) */
static uint32_t pm_uptime_s = 0U;

/** @brief Last uptime update timestamp */
static uint32_t pm_last_uptime_update = 0U;

/*============================================================================*/
/* PRIVATE FUNCTION PROTOTYPES                                                */
/*============================================================================*/

static Status_t pm_read_rail_voltage(uint8_t railId, Voltage_mV_t *pVoltage);
static void pm_check_rail_limits(uint8_t railId);
static void pm_update_lm74900_status(void);

/*============================================================================*/
/* PUBLIC FUNCTIONS                                                           */
/*============================================================================*/

/**
 * @brief Initialize power monitoring module
 */
Status_t PM_Monitor_Init(void)
{
    Status_t status = STATUS_OK;

    if (pm_initialized)
    {
        status = STATUS_ERROR_ALREADY_INIT;
    }
    else
    {
        /* Initialize monitoring data */
        (void)memset(&pm_data, 0, sizeof(pm_data));

        /* Configure power rails */
        pm_data.rails[PM_RAIL_INPUT_12V].nominalVoltage_mV = POWER_INPUT_VOLTAGE_NOM_MV;
        pm_data.rails[PM_RAIL_INPUT_12V].minVoltage_mV = POWER_INPUT_VOLTAGE_MIN_MV;
        pm_data.rails[PM_RAIL_INPUT_12V].maxVoltage_mV = POWER_INPUT_VOLTAGE_MAX_MV;

        pm_data.rails[PM_RAIL_5V].nominalVoltage_mV = POWER_5V_RAIL_NOM_MV;
        pm_data.rails[PM_RAIL_5V].minVoltage_mV = POWER_5V_RAIL_MIN_MV;
        pm_data.rails[PM_RAIL_5V].maxVoltage_mV = POWER_5V_RAIL_MAX_MV;

        pm_data.rails[PM_RAIL_3V3_DIGITAL].nominalVoltage_mV = POWER_3V3_DIGITAL_NOM_MV;
        pm_data.rails[PM_RAIL_3V3_DIGITAL].minVoltage_mV = POWER_3V3_DIGITAL_MIN_MV;
        pm_data.rails[PM_RAIL_3V3_DIGITAL].maxVoltage_mV = POWER_3V3_DIGITAL_MAX_MV;

        pm_data.rails[PM_RAIL_3V3_ANALOG].nominalVoltage_mV = POWER_3V3_ANALOG_NOM_MV;
        pm_data.rails[PM_RAIL_3V3_ANALOG].minVoltage_mV = POWER_3V3_ANALOG_MIN_MV;
        pm_data.rails[PM_RAIL_3V3_ANALOG].maxVoltage_mV = POWER_3V3_ANALOG_MAX_MV;

        /* Initialize IIR filters for each rail (10 Hz cutoff, 100 Hz sample rate) */
        for (uint8_t i = 0U; i < PM_RAIL_COUNT; i++)
        {
            (void)Filter_IIR_Init(&pm_rail_filters[i], 10, 100);
            pm_data.rails[i].status = PM_RAIL_STATUS_OFF;
        }

        /* Initialize LM74900 status */
        pm_data.lm74900.enabled = false;
        pm_data.lm74900.faultDetected = false;
        pm_data.lm74900.reversePolarity = false;
        pm_data.lm74900.currentMonitor_mA = 0U;

        pm_initialized = true;
    }

    return status;
}

/**
 * @brief Update power monitoring (call every 10ms)
 */
Status_t PM_Monitor_Update(void)
{
    Status_t status = STATUS_OK;

    if (pm_initialized)
    {
        /* Update LM74900 status */
        pm_update_lm74900_status();

        /* Update all power rails */
        for (uint8_t i = 0U; i < PM_RAIL_COUNT; i++)
        {
            Voltage_mV_t voltage;

            if (pm_read_rail_voltage(i, &voltage) == STATUS_OK)
            {
                /* Apply filter */
                int32_t filtered = Filter_IIR_Update(&pm_rail_filters[i], (int32_t)voltage);
                pm_data.rails[i].voltage_mV = (Voltage_mV_t)filtered;

                /* Check limits */
                pm_check_rail_limits(i);
            }
        }

        /* Update system power good */
        pm_data.systemPowerGood = true;
        for (uint8_t i = 0U; i < PM_RAIL_COUNT; i++)
        {
            if (pm_data.rails[i].status != PM_RAIL_STATUS_OK)
            {
                pm_data.systemPowerGood = false;
                break;
            }
        }

        /* Update uptime */
        uint32_t current_time = HAL_GetTick();
        if ((current_time - pm_last_uptime_update) >= 1000U)
        {
            pm_uptime_s++;
            pm_data.uptimeSeconds = pm_uptime_s;
            pm_last_uptime_update = current_time;
        }
    }
    else
    {
        status = STATUS_ERROR_NOT_INIT;
    }

    return status;
}

/**
 * @brief Get LM74900 status
 */
Status_t PM_Monitor_GetLM74900Status(PM_LM74900_Status_t *pStatus)
{
    Status_t status = STATUS_ERROR_PARAM;

    if ((pStatus != NULL) && pm_initialized)
    {
        *pStatus = pm_data.lm74900;
        status = STATUS_OK;
    }

    return status;
}

/**
 * @brief Get power rail status
 */
Status_t PM_Monitor_GetRailStatus(uint8_t railId, PM_RailStatus_t *pStatus)
{
    Status_t status = STATUS_ERROR_PARAM;

    if ((railId < PM_RAIL_COUNT) && (pStatus != NULL) && pm_initialized)
    {
        *pStatus = pm_data.rails[railId].status;
        status = STATUS_OK;
    }

    return status;
}

/**
 * @brief Get power rail voltage
 */
Status_t PM_Monitor_GetRailVoltage(uint8_t railId, Voltage_mV_t *pVoltage_mV)
{
    Status_t status = STATUS_ERROR_PARAM;

    if ((railId < PM_RAIL_COUNT) && (pVoltage_mV != NULL) && pm_initialized)
    {
        *pVoltage_mV = pm_data.rails[railId].voltage_mV;
        status = STATUS_OK;
    }

    return status;
}

/**
 * @brief Check if system power is good
 */
Status_t PM_Monitor_IsSystemPowerGood(bool *pPowerGood)
{
    Status_t status = STATUS_ERROR_PARAM;

    if ((pPowerGood != NULL) && pm_initialized)
    {
        *pPowerGood = pm_data.systemPowerGood;
        status = STATUS_OK;
    }

    return status;
}

/**
 * @brief Get input current (from IMON)
 */
Status_t PM_Monitor_GetInputCurrent(Current_mA_t *pCurrent_mA)
{
    Status_t status = STATUS_ERROR_PARAM;

    if ((pCurrent_mA != NULL) && pm_initialized)
    {
        *pCurrent_mA = (Current_mA_t)pm_data.lm74900.currentMonitor_mA;
        status = STATUS_OK;
    }

    return status;
}

/**
 * @brief Register power fault callback
 */
Status_t PM_Monitor_RegisterFaultCallback(PM_FaultCallback_t callback)
{
    pm_fault_callback = callback;
    return STATUS_OK;
}

/**
 * @brief Get system uptime
 */
Status_t PM_Monitor_GetUptime(uint32_t *pUptimeSeconds)
{
    Status_t status = STATUS_ERROR_PARAM;

    if (pUptimeSeconds != NULL)
    {
        *pUptimeSeconds = pm_uptime_s;
        status = STATUS_OK;
    }

    return status;
}

/**
 * @brief De-initialize power monitoring
 */
Status_t PM_Monitor_DeInit(void)
{
    if (pm_initialized)
    {
        pm_initialized = false;
    }

    return STATUS_OK;
}

/*============================================================================*/
/* PRIVATE FUNCTIONS                                                          */
/*============================================================================*/

/**
 * @brief Read rail voltage (placeholder - implement based on hardware)
 */
static Status_t pm_read_rail_voltage(uint8_t railId, Voltage_mV_t *pVoltage)
{
    Status_t status = STATUS_OK;

    /* Voltage divider sensing would be implemented here */
    /* For now, use ADC reference as placeholder */
    if (pVoltage != NULL)
    {
        /* Placeholder: Read from specific ADC channels for each rail */
        switch (railId)
        {
            case PM_RAIL_INPUT_12V:
                /* Would read from voltage divider (12V → 3.3V) */
                *pVoltage = POWER_INPUT_VOLTAGE_NOM_MV;  /* Placeholder */
                break;

            case PM_RAIL_5V:
                /* Read 5V_PG signal (PA5) */
                {
                    GPIO_State_t pg_state;
                    (void)BSP_GPIO_ReadPin(GPIOA, GPIO_PIN_5, &pg_state);
                    pm_data.rails[railId].powerGood = (pg_state == GPIO_STATE_HIGH);
                    *pVoltage = pm_data.rails[railId].powerGood ? POWER_5V_RAIL_NOM_MV : 0U;
                }
                break;

            case PM_RAIL_3V3_DIGITAL:
                /* Use MCU internal ADC reference */
                *pVoltage = POWER_3V3_DIGITAL_NOM_MV;
                break;

            case PM_RAIL_3V3_ANALOG:
                /* Read 3V3A_PG signal (PA6) */
                {
                    GPIO_State_t pg_state;
                    (void)BSP_GPIO_ReadPin(GPIOA, GPIO_PIN_6, &pg_state);
                    pm_data.rails[railId].powerGood = (pg_state == GPIO_STATE_HIGH);
                    *pVoltage = pm_data.rails[railId].powerGood ? POWER_3V3_ANALOG_NOM_MV : 0U;
                }
                break;

            default:
                status = STATUS_ERROR_PARAM;
                break;
        }
    }

    return status;
}

/**
 * @brief Check rail voltage limits
 */
static void pm_check_rail_limits(uint8_t railId)
{
    if (railId < PM_RAIL_COUNT)
    {
        PM_RailInfo_t *rail = &pm_data.rails[railId];
        PM_RailStatus_t old_status = rail->status;

        /* Check voltage limits */
        if (rail->voltage_mV < rail->minVoltage_mV)
        {
            rail->status = PM_RAIL_STATUS_UNDERVOLTAGE;
            rail->faultCount++;
            rail->lastFaultTime_ms = HAL_GetTick();
        }
        else if (rail->voltage_mV > rail->maxVoltage_mV)
        {
            rail->status = PM_RAIL_STATUS_OVERVOLTAGE;
            rail->faultCount++;
            rail->lastFaultTime_ms = HAL_GetTick();
        }
        else if (rail->voltage_mV > 0U)
        {
            rail->status = PM_RAIL_STATUS_OK;
        }
        else
        {
            rail->status = PM_RAIL_STATUS_OFF;
        }

        /* Call callback on status change */
        if ((old_status != rail->status) && (pm_fault_callback != NULL))
        {
            pm_fault_callback(railId, rail->status);
        }
    }
}

/**
 * @brief Update LM74900 status
 */
static void pm_update_lm74900_status(void)
{
    /* Read IMON (PA3 - ADC channel 0) */
    uint16_t imon_raw;
    if (BSP_ADC_ReadChannel(PM_IMON_ADC_CHANNEL, &imon_raw, 10) == STATUS_OK)
    {
        /* Convert ADC to voltage */
        uint32_t imon_voltage = BSP_ADC_ToMillivolts(imon_raw);

        /* Convert voltage to current (IMON sensitivity: 100 mV/A) */
        pm_data.lm74900.currentMonitor_mA = (imon_voltage * 1000U) / PM_IMON_SENSITIVITY;
    }

    /* Read FLT signal (PA4) */
    GPIO_State_t flt_state;
    if (BSP_GPIO_ReadPin(GPIOA, GPIO_PIN_4, &flt_state) == STATUS_OK)
    {
        pm_data.lm74900.faultDetected = (flt_state == GPIO_STATE_HIGH);
    }

    pm_data.lm74900.enabled = true;
}

/*============================================================================*/
/* END OF FILE                                                                */
/*============================================================================*/
