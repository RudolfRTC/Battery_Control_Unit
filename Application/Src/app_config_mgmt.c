/**
 * @file    app_config_mgmt.c
 * @brief   Configuration management implementation
 * @author  Battery Control Unit Development Team
 * @date    2026-01-09
 * @version 1.0.0
 *
 * @note    MISRA C:2012 compliant
 * @note    FRAM-based persistent configuration storage
 *
 * @copyright Copyright (c) 2026
 */

/*============================================================================*/
/* INCLUDES                                                                   */
/*============================================================================*/
#include "app_config_mgmt.h"
#include "app_config.h"
#include "fram_driver.h"
#include "crc.h"
#include <string.h>

/*============================================================================*/
/* PRIVATE VARIABLES                                                          */
/*============================================================================*/

/** @brief Current system configuration */
static SystemConfig_t system_config;

/** @brief Initialization flag */
static bool config_initialized = false;

/*============================================================================*/
/* PRIVATE FUNCTION PROTOTYPES                                                */
/*============================================================================*/

static uint32_t config_calculate_crc(const SystemConfig_t *pConfig);

/*============================================================================*/
/* PUBLIC FUNCTIONS                                                           */
/*============================================================================*/

/**
 * @brief Initialize configuration management
 */
Status_t ConfigMgmt_Init(void)
{
    Status_t status = STATUS_OK;

    if (config_initialized)
    {
        status = STATUS_ERROR_ALREADY_INIT;
    }
    else
    {
        /* Try to load configuration from FRAM */
        status = ConfigMgmt_Load(&system_config);

        if (status != STATUS_OK)
        {
            /* Load failed, use defaults */
            status = ConfigMgmt_LoadDefaults(&system_config);

            if (status == STATUS_OK)
            {
                /* Save defaults to FRAM */
                (void)ConfigMgmt_Save(&system_config);
            }
        }

        if (status == STATUS_OK)
        {
            config_initialized = true;
        }
    }

    return status;
}

/**
 * @brief Load configuration from FRAM
 */
Status_t ConfigMgmt_Load(SystemConfig_t *pConfig)
{
    Status_t status = STATUS_ERROR_PARAM;

    if (pConfig != NULL)
    {
        uint16_t actualLength = 0U;

        /* Load from FRAM */
        status = FRAM_LoadConfig((uint8_t *)pConfig, sizeof(SystemConfig_t), &actualLength);

        if (status == STATUS_OK)
        {
            /* Validate configuration */
            status = ConfigMgmt_Validate(pConfig);
        }
    }

    return status;
}

/**
 * @brief Save configuration to FRAM
 */
Status_t ConfigMgmt_Save(const SystemConfig_t *pConfig)
{
    Status_t status = STATUS_ERROR_PARAM;

    if (pConfig != NULL)
    {
        SystemConfig_t configCopy;

        /* Make a copy for CRC calculation */
        (void)memcpy(&configCopy, pConfig, sizeof(SystemConfig_t));

        /* Fill header */
        configCopy.magic = CONFIG_MAGIC;
        configCopy.version = CONFIG_VERSION;
        configCopy.length = sizeof(SystemConfig_t);

        /* Calculate CRC (excluding CRC field itself) */
        configCopy.crc32 = config_calculate_crc(&configCopy);

        /* Save to FRAM */
        status = FRAM_SaveConfig((const uint8_t *)&configCopy, sizeof(SystemConfig_t));
    }

    return status;
}

/**
 * @brief Load default configuration
 */
Status_t ConfigMgmt_LoadDefaults(SystemConfig_t *pConfig)
{
    Status_t status = STATUS_ERROR_PARAM;

    if (pConfig != NULL)
    {
        /* Clear structure */
        (void)memset(pConfig, 0, sizeof(SystemConfig_t));

        /* Set header */
        pConfig->magic = CONFIG_MAGIC;
        pConfig->version = CONFIG_VERSION;
        pConfig->length = sizeof(SystemConfig_t);

        /* LEM sensor calibration defaults */
        for (uint8_t i = 0U; i < LEM_SENSOR_COUNT; i++)
        {
            pConfig->lem_calibration[i].offsetRaw = 0;
            pConfig->lem_calibration[i].gainFactor = 65536;  /* Unity gain (1.0 in Q16.16) */
            pConfig->lem_calibration[i].sensitivity_mV = LEM_SENSITIVITY_MV_PER_A;
            pConfig->lem_calibration[i].valid = false;  /* Requires calibration */
        }

        /* BTT6200 configuration defaults */
        for (uint8_t i = 0U; i < BTT6200_TOTAL_CHANNELS; i++)
        {
            pConfig->btt_config[i].enabled = true;
            pConfig->btt_config[i].currentLimit_mA = BTT6200_MAX_CURRENT_MA;
            pConfig->btt_config[i].diagnosticsEnabled = true;
        }

        /* Digital input configuration defaults */
        for (uint8_t i = 0U; i < 20U; i++)
        {
            pConfig->di_config[i].enabled = true;
            pConfig->di_config[i].activeLow = false;
            pConfig->di_config[i].pullupEnabled = true;
            pConfig->di_config[i].currentMonitoringEnabled = false;
            pConfig->di_config[i].currentLimit_mA = 5000;  /* 5A */
        }

        /* Power management thresholds */
        pConfig->power_thresholds[0].minVoltage_mV = POWER_INPUT_VOLTAGE_MIN_MV;
        pConfig->power_thresholds[0].maxVoltage_mV = POWER_INPUT_VOLTAGE_MAX_MV;

        pConfig->power_thresholds[1].minVoltage_mV = POWER_5V_RAIL_MIN_MV;
        pConfig->power_thresholds[1].maxVoltage_mV = POWER_5V_RAIL_MAX_MV;

        pConfig->power_thresholds[2].minVoltage_mV = POWER_3V3_DIGITAL_MIN_MV;
        pConfig->power_thresholds[2].maxVoltage_mV = POWER_3V3_DIGITAL_MAX_MV;

        pConfig->power_thresholds[3].minVoltage_mV = POWER_3V3_ANALOG_MIN_MV;
        pConfig->power_thresholds[3].maxVoltage_mV = POWER_3V3_ANALOG_MAX_MV;

        /* Temperature limits */
        pConfig->temp_low_limit_mC = -40000;   /* -40°C */
        pConfig->temp_high_limit_mC = 85000;   /* +85°C */

        /* CAN configuration */
        pConfig->can1_baudrate = CAN1_BAUDRATE;
        pConfig->can2_baudrate = CAN2_BAUDRATE;
        pConfig->can2_enabled = true;

        /* Safety configuration */
        pConfig->max_loop_time_us = SAFETY_MAX_LOOP_TIME_MS * 1000U;
        pConfig->watchdog_enabled = true;

        /* Calculate CRC */
        pConfig->crc32 = config_calculate_crc(pConfig);

        status = STATUS_OK;
    }

    return status;
}

/**
 * @brief Factory reset
 */
Status_t ConfigMgmt_FactoryReset(void)
{
    Status_t status = STATUS_OK;
    SystemConfig_t defaultConfig;

    /* Load defaults */
    status = ConfigMgmt_LoadDefaults(&defaultConfig);

    if (status == STATUS_OK)
    {
        /* Save to FRAM */
        status = ConfigMgmt_Save(&defaultConfig);

        if (status == STATUS_OK)
        {
            /* Update current configuration */
            system_config = defaultConfig;
        }
    }

    return status;
}

/**
 * @brief Validate configuration
 */
Status_t ConfigMgmt_Validate(const SystemConfig_t *pConfig)
{
    Status_t status = STATUS_ERROR_PARAM;

    if (pConfig != NULL)
    {
        /* Check magic number */
        if (pConfig->magic != CONFIG_MAGIC)
        {
            status = STATUS_ERROR_INVALID_DATA;
        }
        /* Check version compatibility */
        else if ((pConfig->version >> 8) != (CONFIG_VERSION >> 8))
        {
            status = STATUS_ERROR_INVALID_DATA;
        }
        /* Check length */
        else if (pConfig->length != sizeof(SystemConfig_t))
        {
            status = STATUS_ERROR_INVALID_DATA;
        }
        else
        {
            /* Verify CRC */
            uint32_t calculatedCRC = config_calculate_crc(pConfig);

            if (calculatedCRC == pConfig->crc32)
            {
                status = STATUS_OK;
            }
            else
            {
                status = STATUS_ERROR_CRC;
            }
        }
    }

    return status;
}

/**
 * @brief Apply configuration to system
 */
Status_t ConfigMgmt_Apply(const SystemConfig_t *pConfig)
{
    Status_t status = STATUS_ERROR_PARAM;

    if (pConfig != NULL)
    {
        /* Apply LEM sensor calibration */
        for (uint8_t i = 0U; i < LEM_SENSOR_COUNT; i++)
        {
            if (pConfig->lem_calibration[i].valid)
            {
                (void)LEM_SetCalibration(i, &pConfig->lem_calibration[i]);
            }
        }

        /* Apply digital input configuration */
        for (uint8_t i = 0U; i < 20U; i++)
        {
            (void)DI_ConfigureInput(i, &pConfig->di_config[i]);
        }

        /* Apply temperature limits */
        TempSensor_Config_t tempConfig;
        tempConfig.enabled = true;
        tempConfig.lowLimit_mC = pConfig->temp_low_limit_mC;
        tempConfig.highLimit_mC = pConfig->temp_high_limit_mC;
        tempConfig.hysteresis_mC = 2000;  /* 2°C hysteresis */
        tempConfig.filterSize = 8U;
        (void)TempSensor_Configure(&tempConfig);

        /* Update current configuration */
        system_config = *pConfig;

        status = STATUS_OK;
    }

    return status;
}

/**
 * @brief Export configuration
 */
Status_t ConfigMgmt_Export(const SystemConfig_t *pConfig, uint8_t *pBlob,
                          uint16_t maxSize, uint16_t *pActualSize)
{
    Status_t status = STATUS_ERROR_PARAM;

    if ((pConfig != NULL) && (pBlob != NULL) && (pActualSize != NULL))
    {
        if (maxSize >= sizeof(SystemConfig_t))
        {
            (void)memcpy(pBlob, pConfig, sizeof(SystemConfig_t));
            *pActualSize = sizeof(SystemConfig_t);
            status = STATUS_OK;
        }
        else
        {
            status = STATUS_ERROR_OVERFLOW;
        }
    }

    return status;
}

/**
 * @brief Import configuration
 */
Status_t ConfigMgmt_Import(SystemConfig_t *pConfig, const uint8_t *pBlob, uint16_t size)
{
    Status_t status = STATUS_ERROR_PARAM;

    if ((pConfig != NULL) && (pBlob != NULL))
    {
        if (size >= sizeof(SystemConfig_t))
        {
            (void)memcpy(pConfig, pBlob, sizeof(SystemConfig_t));

            /* Validate imported configuration */
            status = ConfigMgmt_Validate(pConfig);
        }
        else
        {
            status = STATUS_ERROR_INVALID_DATA;
        }
    }

    return status;
}

/*============================================================================*/
/* PRIVATE FUNCTIONS                                                          */
/*============================================================================*/

/**
 * @brief Calculate configuration CRC32
 */
static uint32_t config_calculate_crc(const SystemConfig_t *pConfig)
{
    uint32_t crc = 0U;

    if (pConfig != NULL)
    {
        /* Calculate CRC over entire structure except CRC field */
        const uint8_t *pData = (const uint8_t *)pConfig;
        uint32_t length = sizeof(SystemConfig_t);

        /* Skip CRC field (offset to crc32 field) */
        uint32_t crcOffset = (uint32_t)((const uint8_t *)&pConfig->crc32 - pData);

        /* Calculate CRC of data before CRC field */
        if (crcOffset > 0U)
        {
            crc = CRC_Calculate32(pData, crcOffset);
        }

        /* Calculate CRC of data after CRC field */
        uint32_t remainingOffset = crcOffset + sizeof(pConfig->crc32);
        uint32_t remainingLength = length - remainingOffset;

        if (remainingLength > 0U)
        {
            crc = CRC_Calculate32_Incremental(crc, pData + remainingOffset, remainingLength);
        }
    }

    return crc;
}

/*============================================================================*/
/* END OF FILE                                                                */
/*============================================================================*/
