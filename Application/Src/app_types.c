/**
 * @file    app_types.c
 * @brief   Common type utilities and helper functions
 * @author  Battery Control Unit Development Team
 * @date    2026-01-10
 * @version 1.0.0
 *
 * @note    MISRA C:2012 compliant
 * @note    Type definitions are in app_types.h
 *
 * @copyright Copyright (c) 2026
 */

/*============================================================================*/
/* INCLUDES                                                                   */
/*============================================================================*/
#include "app_types.h"
#include <string.h>

/*============================================================================*/
/* PRIVATE CONSTANTS                                                          */
/*============================================================================*/

/** @brief Status code string table */
static const char* const status_strings[] = {
    "OK",                  /* STATUS_OK */
    "ERROR",               /* STATUS_ERROR */
    "TIMEOUT",             /* STATUS_ERROR_TIMEOUT */
    "BUSY",                /* STATUS_ERROR_BUSY */
    "INVALID_PARAM",       /* STATUS_ERROR_PARAM */
    "HW_FAULT",            /* STATUS_ERROR_HW_FAULT */
    "OVERFLOW",            /* STATUS_ERROR_OVERFLOW */
    "UNDERFLOW",           /* STATUS_ERROR_UNDERFLOW */
    "CRC_ERROR",           /* STATUS_ERROR_CRC */
    "NOT_INITIALIZED",     /* STATUS_ERROR_NOT_INIT */
    "ALREADY_INITIALIZED", /* STATUS_ERROR_ALREADY_INIT */
    "NOT_SUPPORTED",       /* STATUS_ERROR_NOT_SUPPORTED */
    "NO_MEMORY",           /* STATUS_ERROR_NO_MEMORY */
    "OUT_OF_RANGE",        /* STATUS_ERROR_RANGE */
    "INVALID_STATE",       /* STATUS_ERROR_INVALID_STATE */
    "SAFETY_VIOLATION"     /* STATUS_ERROR_SAFETY */
};

#define STATUS_STRING_COUNT  (sizeof(status_strings) / sizeof(status_strings[0]))

/*============================================================================*/
/* PUBLIC FUNCTIONS                                                           */
/*============================================================================*/

/**
 * @brief Convert status code to string
 * @param[in] status Status code
 * @return Pointer to status string
 */
const char* Status_ToString(Status_t status)
{
    const char* str = "UNKNOWN";

    if ((uint8_t)status < STATUS_STRING_COUNT)
    {
        str = status_strings[(uint8_t)status];
    }

    return str;
}

/**
 * @brief Check if status indicates success
 * @param[in] status Status code
 * @return true if success, false otherwise
 */
bool Status_IsOK(Status_t status)
{
    return (status == STATUS_OK);
}

/**
 * @brief Check if status indicates error
 * @param[in] status Status code
 * @return true if error, false otherwise
 */
bool Status_IsError(Status_t status)
{
    return (status != STATUS_OK);
}

/**
 * @brief Initialize version structure
 * @param[out] pVersion Pointer to version structure
 * @param[in]  major    Major version number
 * @param[in]  minor    Minor version number
 * @param[in]  patch    Patch version number
 * @return STATUS_OK on success, STATUS_ERROR_PARAM if pVersion is NULL
 */
Status_t Version_Init(Version_t *pVersion, uint8_t major, uint8_t minor, uint8_t patch)
{
    Status_t status = STATUS_ERROR_PARAM;

    if (pVersion != NULL)
    {
        pVersion->major = major;
        pVersion->minor = minor;
        pVersion->patch = patch;
        pVersion->reserved = 0U;
        status = STATUS_OK;
    }

    return status;
}

/**
 * @brief Compare two version structures
 * @param[in] pVersion1 First version
 * @param[in] pVersion2 Second version
 * @return -1 if version1 < version2, 0 if equal, 1 if version1 > version2
 *         Returns 0 if any pointer is NULL
 */
int8_t Version_Compare(const Version_t *pVersion1, const Version_t *pVersion2)
{
    int8_t result = 0;

    if ((pVersion1 != NULL) && (pVersion2 != NULL))
    {
        /* Compare major version */
        if (pVersion1->major < pVersion2->major)
        {
            result = -1;
        }
        else if (pVersion1->major > pVersion2->major)
        {
            result = 1;
        }
        else
        {
            /* Major versions equal, compare minor */
            if (pVersion1->minor < pVersion2->minor)
            {
                result = -1;
            }
            else if (pVersion1->minor > pVersion2->minor)
            {
                result = 1;
            }
            else
            {
                /* Minor versions equal, compare patch */
                if (pVersion1->patch < pVersion2->patch)
                {
                    result = -1;
                }
                else if (pVersion1->patch > pVersion2->patch)
                {
                    result = 1;
                }
                else
                {
                    result = 0;  /* All equal */
                }
            }
        }
    }

    return result;
}

/**
 * @brief Initialize timestamp
 * @param[out] pTimestamp Pointer to timestamp structure
 * @param[in]  ms         Milliseconds value
 * @return STATUS_OK on success, STATUS_ERROR_PARAM if pTimestamp is NULL
 */
Status_t Timestamp_Init(Timestamp_t *pTimestamp, uint32_t ms)
{
    Status_t status = STATUS_ERROR_PARAM;

    if (pTimestamp != NULL)
    {
        pTimestamp->milliseconds = ms;
        status = STATUS_OK;
    }

    return status;
}

/**
 * @brief Calculate time difference between timestamps
 * @param[in] pStart Start timestamp
 * @param[in] pEnd   End timestamp
 * @return Time difference in milliseconds, 0 if any pointer is NULL
 */
uint32_t Timestamp_Diff(const Timestamp_t *pStart, const Timestamp_t *pEnd)
{
    uint32_t diff = 0U;

    if ((pStart != NULL) && (pEnd != NULL))
    {
        if (pEnd->milliseconds >= pStart->milliseconds)
        {
            diff = pEnd->milliseconds - pStart->milliseconds;
        }
        else
        {
            /* Handle overflow (32-bit wraparound) */
            diff = (0xFFFFFFFFU - pStart->milliseconds) + pEnd->milliseconds + 1U;
        }
    }

    return diff;
}

/**
 * @brief Check if value is within range
 * @param[in] value Value to check
 * @param[in] pRange Range structure
 * @return true if within range, false otherwise
 */
bool Range_IsWithin(int32_t value, const Range_t *pRange)
{
    bool within = false;

    if (pRange != NULL)
    {
        within = ((value >= pRange->min) && (value <= pRange->max));
    }

    return within;
}

/**
 * @brief Clamp value to range
 * @param[in] value Value to clamp
 * @param[in] pRange Range structure
 * @return Clamped value, or original value if pRange is NULL
 */
int32_t Range_Clamp(int32_t value, const Range_t *pRange)
{
    int32_t clamped = value;

    if (pRange != NULL)
    {
        if (value < pRange->min)
        {
            clamped = pRange->min;
        }
        else if (value > pRange->max)
        {
            clamped = pRange->max;
        }
        else
        {
            clamped = value;
        }
    }

    return clamped;
}

/**
 * @brief Convert voltage from millivolts to volts (float)
 * @param[in] voltage_mV Voltage in millivolts
 * @return Voltage in volts
 */
float Voltage_mVtoV(Voltage_mV_t voltage_mV)
{
    return ((float)voltage_mV / 1000.0f);
}

/**
 * @brief Convert current from milliamperes to amperes (float)
 * @param[in] current_mA Current in milliamperes
 * @return Current in amperes
 */
float Current_mAtoA(Current_mA_t current_mA)
{
    return ((float)current_mA / 1000.0f);
}

/**
 * @brief Convert temperature to Celsius (float)
 * @param[in] temp Temperature in 0.01°C units
 * @return Temperature in °C
 */
float Temperature_toCelsius(Temperature_t temp)
{
    return ((float)temp / 100.0f);
}

/**
 * @brief Convert power from milliwatts to watts (float)
 * @param[in] power_mW Power in milliwatts
 * @return Power in watts
 */
float Power_mWtoW(Power_mW_t power_mW)
{
    return ((float)power_mW / 1000.0f);
}

/**
 * @brief Convert percentage to float (0.0 - 1.0)
 * @param[in] percentage Percentage in 0.01% units (0-10000)
 * @return Percentage as float (0.0 - 1.0)
 */
float Percentage_toFloat(Percentage_t percentage)
{
    return ((float)percentage / 10000.0f);
}

/**
 * @brief Convert percentage to integer (0-100)
 * @param[in] percentage Percentage in 0.01% units (0-10000)
 * @return Percentage as integer (0-100)
 */
uint8_t Percentage_toUint8(Percentage_t percentage)
{
    return (uint8_t)(percentage / 100U);
}
