/**
 * @file    app_config.c
 * @brief   Application configuration runtime utilities
 * @author  Battery Control Unit Development Team
 * @date    2026-01-10
 * @version 1.0.0
 *
 * @note    MISRA C:2012 compliant
 * @note    Configuration is defined in app_config.h
 *
 * @copyright Copyright (c) 2026
 */

/*============================================================================*/
/* INCLUDES                                                                   */
/*============================================================================*/
#include "app_config.h"
#include "app_types.h"

/*============================================================================*/
/* PRIVATE VARIABLES                                                          */
/*============================================================================*/

/** @brief Firmware version structure */
static const Version_t firmware_version = {
    .major = FIRMWARE_VERSION_MAJOR,
    .minor = FIRMWARE_VERSION_MINOR,
    .patch = FIRMWARE_VERSION_PATCH,
    .reserved = 0U
};

/*============================================================================*/
/* PUBLIC FUNCTIONS                                                           */
/*============================================================================*/

/**
 * @brief Get firmware version
 * @param[out] pVersion Pointer to store version information
 * @return STATUS_OK on success, STATUS_ERROR_PARAM if pVersion is NULL
 */
Status_t AppConfig_GetFirmwareVersion(Version_t *pVersion)
{
    Status_t status = STATUS_ERROR_PARAM;

    if (pVersion != NULL)
    {
        *pVersion = firmware_version;
        status = STATUS_OK;
    }

    return status;
}

/**
 * @brief Get firmware version as string
 * @return Pointer to firmware version string
 */
const char* AppConfig_GetVersionString(void)
{
    return FIRMWARE_VERSION_STRING;
}

/**
 * @brief Validate configuration parameters at runtime
 * @return STATUS_OK if all configurations are valid
 */
Status_t AppConfig_Validate(void)
{
    Status_t status = STATUS_OK;

    /* Validate power supply ranges */
    if ((POWER_INPUT_VOLTAGE_MIN_MV >= POWER_INPUT_VOLTAGE_NOM_MV) ||
        (POWER_INPUT_VOLTAGE_NOM_MV >= POWER_INPUT_VOLTAGE_MAX_MV))
    {
        status = STATUS_ERROR_PARAM;
    }

    /* Validate timing parameters */
    if ((WATCHDOG_REFRESH_PERIOD_MS >= WATCHDOG_TIMEOUT_MS) && (status == STATUS_OK))
    {
        status = STATUS_ERROR_PARAM;
    }

    /* Validate task periods */
    if ((TASK_PERIOD_SAFETY_MS > TASK_PERIOD_FAST_MS) && (status == STATUS_OK))
    {
        status = STATUS_ERROR_PARAM;
    }

    /* Validate CAN configuration */
    if ((CAN_BITRATE_KBPS == 0U) && (status == STATUS_OK))
    {
        status = STATUS_ERROR_PARAM;
    }

    /* Validate FRAM address ranges */
    if ((FRAM_ADDR_CONFIG_START + FRAM_ADDR_CONFIG_SIZE > FRAM_SIZE_BYTES) &&
        (status == STATUS_OK))
    {
        status = STATUS_ERROR_PARAM;
    }

    return status;
}

/**
 * @brief Check if a feature is enabled
 * @param[in] featureMask Feature bit mask
 * @return true if enabled, false otherwise
 */
bool AppConfig_IsFeatureEnabled(uint32_t featureMask)
{
    bool enabled = false;

    /* Check individual feature flags */
    switch (featureMask)
    {
        case 0x01U:  /* CAN Bus 1 */
            enabled = (FEATURE_CAN_BUS_1 != 0U);
            break;
        case 0x02U:  /* CAN Bus 2 */
            enabled = (FEATURE_CAN_BUS_2 != 0U);
            break;
        case 0x04U:  /* Isolated SPI */
            enabled = (FEATURE_ISOLATED_SPI != 0U);
            break;
        case 0x08U:  /* FRAM Storage */
            enabled = (FEATURE_FRAM_STORAGE != 0U);
            break;
        case 0x10U:  /* Temperature Monitor */
            enabled = (FEATURE_TEMPERATURE_MONITOR != 0U);
            break;
        case 0x20U:  /* Output PWM */
            enabled = (FEATURE_OUTPUT_PWM != 0U);
            break;
        case 0x40U:  /* Fault Logging */
            enabled = (FEATURE_FAULT_LOGGING != 0U);
            break;
        case 0x80U:  /* Calibration */
            enabled = (FEATURE_CALIBRATION != 0U);
            break;
        default:
            enabled = false;
            break;
    }

    return enabled;
}
