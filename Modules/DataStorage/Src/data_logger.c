/**
 * @file    data_logger.c
 * @brief   Data logger implementation
 * @author  Battery Control Unit Development Team
 * @date    2026-01-09
 * @version 1.0.0
 *
 * @note    MISRA C:2012 compliant
 * @note    FRAM-based circular buffer data logging
 *
 * @copyright Copyright (c) 2026
 */

/*============================================================================*/
/* INCLUDES                                                                   */
/*============================================================================*/
#include "data_logger.h"
#include "fram_driver.h"
#include "lem_sensor.h"
#include "pm_monitor.h"
#include "temp_sensor.h"
#include "digital_input.h"
#include "btt6200.h"
#include "app_main.h"
#include "app_errors.h"
#include "timestamp.h"
#include <string.h>

/*============================================================================*/
/* PRIVATE CONSTANTS                                                          */
/*============================================================================*/

/** @brief FRAM data log base address */
#define DATALOG_FRAM_BASE_ADDR  (0x1C00U)  /* From FRAM memory map */

/*============================================================================*/
/* PRIVATE VARIABLES                                                          */
/*============================================================================*/

/** @brief Data logger statistics */
static DataLog_Statistics_t datalog_stats;

/** @brief Current write index (circular buffer) */
static uint32_t datalog_write_index = 0U;

/** @brief Initialization flag */
static bool datalog_initialized = false;

/*============================================================================*/
/* PRIVATE FUNCTION PROTOTYPES                                                */
/*============================================================================*/

static Status_t datalog_build_entry(DataLog_Entry_t *pEntry);
static uint16_t datalog_calculate_address(uint32_t index);

/*============================================================================*/
/* PUBLIC FUNCTIONS                                                           */
/*============================================================================*/

/**
 * @brief Initialize data logger
 */
Status_t DataLog_Init(void)
{
    Status_t status = STATUS_OK;

    if (datalog_initialized)
    {
        status = STATUS_ERROR_ALREADY_INIT;
    }
    else
    {
        /* Clear statistics */
        (void)memset(&datalog_stats, 0, sizeof(datalog_stats));

        /* Initialize write index to 0 */
        datalog_write_index = 0U;
        datalog_stats.currentIndex = 0U;

        datalog_initialized = true;
    }

    return status;
}

/**
 * @brief Log current system data
 */
Status_t DataLog_LogData(void)
{
    Status_t status = STATUS_OK;

    if (datalog_initialized)
    {
        DataLog_Entry_t entry;

        /* Build log entry from current system state */
        status = datalog_build_entry(&entry);

        if (status == STATUS_OK)
        {
            /* Calculate FRAM address for this entry */
            uint16_t addr = datalog_calculate_address(datalog_write_index);

            /* Write to FRAM */
            status = FRAM_Write(addr, (const uint8_t *)&entry, sizeof(DataLog_Entry_t));

            if (status == STATUS_OK)
            {
                /* Update statistics */
                datalog_stats.writeCount++;
                datalog_stats.totalEntries++;

                if (datalog_stats.totalEntries > DATALOG_MAX_ENTRIES)
                {
                    datalog_stats.totalEntries = DATALOG_MAX_ENTRIES;
                    datalog_stats.overruns++;
                }

                /* Increment write index (circular) */
                datalog_write_index++;
                if (datalog_write_index >= DATALOG_MAX_ENTRIES)
                {
                    datalog_write_index = 0U;
                }

                datalog_stats.currentIndex = datalog_write_index;
            }
            else
            {
                datalog_stats.writeErrors++;
            }
        }
    }
    else
    {
        status = STATUS_ERROR_NOT_INIT;
    }

    return status;
}

/**
 * @brief Read log entry by index
 */
Status_t DataLog_ReadEntry(uint32_t index, DataLog_Entry_t *pEntry)
{
    Status_t status = STATUS_ERROR_PARAM;

    if ((index < DATALOG_MAX_ENTRIES) && (pEntry != NULL) && datalog_initialized)
    {
        /* Calculate FRAM address */
        uint16_t addr = datalog_calculate_address(index);

        /* Read from FRAM */
        status = FRAM_Read(addr, (uint8_t *)pEntry, sizeof(DataLog_Entry_t));

        if (status == STATUS_OK)
        {
            datalog_stats.readCount++;
        }
    }

    return status;
}

/**
 * @brief Get latest log entry
 */
Status_t DataLog_GetLatest(DataLog_Entry_t *pEntry)
{
    Status_t status = STATUS_ERROR_PARAM;

    if ((pEntry != NULL) && datalog_initialized)
    {
        if (datalog_stats.totalEntries > 0U)
        {
            /* Calculate previous index (circular) */
            uint32_t latestIndex;

            if (datalog_write_index == 0U)
            {
                latestIndex = DATALOG_MAX_ENTRIES - 1U;
            }
            else
            {
                latestIndex = datalog_write_index - 1U;
            }

            status = DataLog_ReadEntry(latestIndex, pEntry);
        }
        else
        {
            status = STATUS_ERROR_NO_DATA;
        }
    }

    return status;
}

/**
 * @brief Clear all log entries
 */
Status_t DataLog_Clear(void)
{
    Status_t status = STATUS_OK;

    if (datalog_initialized)
    {
        /* Erase FRAM log area */
        status = FRAM_Erase(DATALOG_FRAM_BASE_ADDR, DATALOG_MAX_ENTRIES * DATALOG_ENTRY_SIZE);

        if (status == STATUS_OK)
        {
            /* Reset indices and counters */
            datalog_write_index = 0U;
            datalog_stats.totalEntries = 0U;
            datalog_stats.currentIndex = 0U;
            datalog_stats.writeCount = 0U;
            datalog_stats.readCount = 0U;
            datalog_stats.overruns = 0U;
        }
    }
    else
    {
        status = STATUS_ERROR_NOT_INIT;
    }

    return status;
}

/**
 * @brief Get data logger statistics
 */
Status_t DataLog_GetStatistics(DataLog_Statistics_t *pStats)
{
    Status_t status = STATUS_ERROR_PARAM;

    if ((pStats != NULL) && datalog_initialized)
    {
        *pStats = datalog_stats;
        status = STATUS_OK;
    }

    return status;
}

/**
 * @brief Export log entries
 */
Status_t DataLog_Export(uint8_t *pBuffer, uint32_t bufferSize,
                       uint32_t startIndex, uint32_t count, uint32_t *pExported)
{
    Status_t status = STATUS_ERROR_PARAM;

    if ((pBuffer != NULL) && (pExported != NULL) && datalog_initialized)
    {
        uint32_t exported = 0U;
        uint32_t maxEntries = bufferSize / sizeof(DataLog_Entry_t);

        /* Limit count to buffer size and available entries */
        if (count > maxEntries)
        {
            count = maxEntries;
        }

        if (count > datalog_stats.totalEntries)
        {
            count = datalog_stats.totalEntries;
        }

        /* Export entries */
        for (uint32_t i = 0U; i < count; i++)
        {
            uint32_t index = (startIndex + i) % DATALOG_MAX_ENTRIES;
            DataLog_Entry_t *pEntry = (DataLog_Entry_t *)&pBuffer[i * sizeof(DataLog_Entry_t)];

            if (DataLog_ReadEntry(index, pEntry) != STATUS_OK)
            {
                break;
            }

            exported++;
        }

        *pExported = exported;
        status = STATUS_OK;
    }

    return status;
}

/**
 * @brief De-initialize data logger
 */
Status_t DataLog_DeInit(void)
{
    if (datalog_initialized)
    {
        datalog_initialized = false;
    }

    return STATUS_OK;
}

/*============================================================================*/
/* PRIVATE FUNCTIONS                                                          */
/*============================================================================*/

/**
 * @brief Build log entry from current system state
 */
static Status_t datalog_build_entry(DataLog_Entry_t *pEntry)
{
    Status_t status = STATUS_ERROR_PARAM;

    if (pEntry != NULL)
    {
        /* Clear entry */
        (void)memset(pEntry, 0, sizeof(DataLog_Entry_t));

        /* Timestamp */
        pEntry->timestamp_ms = Timestamp_GetMillis();

        /* LEM current measurements (first 4 channels) */
        for (uint8_t i = 0U; i < 4U; i++)
        {
            Current_mA_t current;
            if (LEM_ReadCurrent(i, &current) == STATUS_OK)
            {
                pEntry->lem_current_mA[i] = current;
            }
        }

        /* Power rail voltages */
        Voltage_mV_t voltage;
        (void)PM_Monitor_GetRailVoltage(PM_RAIL_INPUT_12V, &voltage);
        pEntry->voltage_12v_mV = (uint16_t)voltage;

        (void)PM_Monitor_GetRailVoltage(PM_RAIL_5V, &voltage);
        pEntry->voltage_5v_mV = (uint16_t)voltage;

        (void)PM_Monitor_GetRailVoltage(PM_RAIL_3V3_DIGITAL, &voltage);
        pEntry->voltage_3v3d_mV = (uint16_t)voltage;

        (void)PM_Monitor_GetRailVoltage(PM_RAIL_3V3_ANALOG, &voltage);
        pEntry->voltage_3v3a_mV = (uint16_t)voltage;

        /* Temperature (convert from milliCelsius to deciCelsius) */
        int32_t temp_mC;
        if (TempSensor_ReadTemperature(&temp_mC) == STATUS_OK)
        {
            pEntry->temperature_dC = (int16_t)(temp_mC / 100);  /* mC to dC */
        }

        /* System status */
        AppStatus_t appStatus;
        if (App_GetStatus(&appStatus) == STATUS_OK)
        {
            pEntry->app_state = (uint8_t)appStatus.state;
            pEntry->power_good = appStatus.powerGood ? 1U : 0U;
            pEntry->safety_ok = appStatus.safetyOK ? 1U : 0U;
            pEntry->active_dtcs = (uint8_t)appStatus.activeErrors;
        }

        /* Output states (20 outputs, packed as bitmap) */
        (void)memset(pEntry->output_states, 0, sizeof(pEntry->output_states));
        for (uint8_t i = 0U; i < 20U; i++)
        {
            BTT6200_ChannelState_t channelState;
            if (BTT6200_GetChannelState(i, &channelState) == STATUS_OK)
            {
                /* Set bit if channel is ON or PWM (active states) */
                if ((channelState == BTT6200_STATE_ON) || (channelState == BTT6200_STATE_PWM))
                {
                    uint8_t byteIndex = i / 8U;
                    uint8_t bitIndex = i % 8U;
                    pEntry->output_states[byteIndex] |= (1U << bitIndex);
                }
            }
        }

        /* Input states (20 inputs, packed as bitmap) */
        (void)memset(pEntry->input_states, 0, sizeof(pEntry->input_states));
        for (uint8_t i = 0U; i < 20U; i++)
        {
            bool state;
            if (DI_ReadInput(i, &state) == STATUS_OK)
            {
                if (state)
                {
                    uint8_t byteIndex = i / 8U;
                    uint8_t bitIndex = i % 8U;
                    pEntry->input_states[byteIndex] |= (1U << bitIndex);
                }
            }
        }

        status = STATUS_OK;
    }

    return status;
}

/**
 * @brief Calculate FRAM address for log entry index
 */
static uint16_t datalog_calculate_address(uint32_t index)
{
    uint16_t addr = DATALOG_FRAM_BASE_ADDR + (uint16_t)(index * DATALOG_ENTRY_SIZE);
    return addr;
}

/*============================================================================*/
/* END OF FILE                                                                */
/*============================================================================*/
