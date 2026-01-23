/**
 * @file    temp_sensor.c
 * @brief   TMP1075 temperature sensor driver implementation
 * @author  Battery Control Unit Development Team
 * @date    2026-01-09
 * @version 1.0.0
 *
 * @note    MISRA C:2012 compliant
 * @note    TI TMP1075 I2C temperature sensor
 * @note    ±0.5°C accuracy, 12-bit resolution
 *
 * @copyright Copyright (c) 2026
 */

/*============================================================================*/
/* INCLUDES                                                                   */
/*============================================================================*/
#include "temp_sensor.h"
#include "bsp_i2c.h"
#include "filter.h"
#include "app_config.h"
#include <string.h>

/*============================================================================*/
/* PRIVATE CONSTANTS                                                          */
/*============================================================================*/

/** @brief TMP1075 I2C device address (7-bit) */
#define TMP1075_I2C_ADDR        (0x48U)

/** @brief TMP1075 I2C instance */
#define TMP1075_I2C_INSTANCE    (BSP_I2C_INSTANCE_2)

/** @brief TMP1075 register addresses */
#define TMP1075_REG_TEMP        (0x00U)
#define TMP1075_REG_CONFIG      (0x01U)
#define TMP1075_REG_TLOW        (0x02U)
#define TMP1075_REG_THIGH       (0x03U)

/** @brief TMP1075 config register bits */
#define TMP1075_CFG_SHUTDOWN    (0x0100U)
#define TMP1075_CFG_ONESHOT     (0x8000U)
#define TMP1075_CFG_CONV_12BIT  (0x0060U)

/** @brief Temperature resolution (LSB = 0.0625°C for 12-bit) */
#define TMP1075_RESOLUTION      (625U)  /* 0.0625°C * 10000 = 625 */

/*============================================================================*/
/* PRIVATE VARIABLES                                                          */
/*============================================================================*/

/** @brief Sensor configuration */
static TempSensor_Config_t temp_config;

/** @brief Temperature data */
static TempSensor_Data_t temp_data;

/** @brief Moving average filter */
static Filter_MovingAverage_t temp_filter;

/** @brief Over-temperature callback */
static TempSensor_AlarmCallback_t temp_alarm_callback = NULL;

/** @brief Initialization flag */
static bool temp_initialized = false;

/*============================================================================*/
/* PRIVATE FUNCTION PROTOTYPES                                                */
/*============================================================================*/

static Status_t tmp1075_read_register(uint8_t regAddr, uint16_t *pValue);
static Status_t tmp1075_write_register(uint8_t regAddr, uint16_t value);
static Status_t tmp1075_read_temperature_raw(int16_t *pTempRaw);
static void temp_check_limits(void);

/*============================================================================*/
/* PUBLIC FUNCTIONS                                                           */
/*============================================================================*/

/**
 * @brief Initialize temperature sensor
 */
Status_t TempSensor_Init(void)
{
    Status_t status = STATUS_OK;

    if (temp_initialized)
    {
        status = STATUS_ERROR_ALREADY_INIT;
    }
    else
    {
        /* Initialize configuration */
        temp_config.enabled = true;
        temp_config.lowLimit_mC = -40000;   /* -40°C */
        temp_config.highLimit_mC = 85000;   /* +85°C */
        temp_config.hysteresis_mC = 2000;   /* 2°C */
        temp_config.filterSize = 8U;

        /* Initialize data */
        (void)memset(&temp_data, 0, sizeof(temp_data));
        temp_data.minTemp_mC = INT32_MAX;
        temp_data.maxTemp_mC = INT32_MIN;

        /* Initialize filter */
        (void)Filter_MA_Init(&temp_filter, temp_config.filterSize);

        /* Configure TMP1075 */
        uint16_t configValue = TMP1075_CFG_CONV_12BIT;  /* 12-bit resolution */
        status = tmp1075_write_register(TMP1075_REG_CONFIG, configValue);

        if (status == STATUS_OK)
        {
            /* Set temperature limits (convert mC to TMP1075 register format) */
            int16_t tlowRaw = (int16_t)((temp_config.lowLimit_mC * TMP1075_TEMP_SCALE_FACTOR) / 1000);
            int16_t thighRaw = (int16_t)((temp_config.highLimit_mC * TMP1075_TEMP_SCALE_FACTOR) / 1000);

            (void)tmp1075_write_register(TMP1075_REG_TLOW, (uint16_t)tlowRaw);
            (void)tmp1075_write_register(TMP1075_REG_THIGH, (uint16_t)thighRaw);

            temp_initialized = true;
        }
    }

    return status;
}

/**
 * @brief Read temperature
 */
Status_t TempSensor_ReadTemperature(int32_t *pTemp_mC)
{
    Status_t status = STATUS_ERROR_PARAM;

    if ((pTemp_mC != NULL) && temp_initialized)
    {
        int16_t tempRaw;
        status = tmp1075_read_temperature_raw(&tempRaw);

        if (status == STATUS_OK)
        {
            /* Convert to milliCelsius */
            /* TMP1075: 12-bit, LSB = 0.0625°C */
            /* Raw value is in units of 0.0625°C */
            int32_t temp_mC = ((int32_t)tempRaw * (int32_t)TMP1075_RESOLUTION) / 10;

            /* Apply filter */
            int32_t filtered = Filter_MA_Update(&temp_filter, temp_mC);

            /* Update data */
            temp_data.currentTemp_mC = filtered;
            temp_data.rawTemp_mC = temp_mC;
            temp_data.timestamp_ms = HAL_GetTick();
            temp_data.measurementCount++;

            /* Update min/max */
            if (filtered < temp_data.minTemp_mC)
            {
                temp_data.minTemp_mC = filtered;
            }

            if (filtered > temp_data.maxTemp_mC)
            {
                temp_data.maxTemp_mC = filtered;
            }

            /* Check limits */
            temp_check_limits();

            *pTemp_mC = filtered;
        }
        else
        {
            temp_data.errorCount++;
        }
    }

    return status;
}

/**
 * @brief Get temperature data
 */
Status_t TempSensor_GetData(TempSensor_Data_t *pData)
{
    Status_t status = STATUS_ERROR_PARAM;

    if ((pData != NULL) && temp_initialized)
    {
        *pData = temp_data;
        status = STATUS_OK;
    }

    return status;
}

/**
 * @brief Configure temperature sensor
 */
Status_t TempSensor_Configure(const TempSensor_Config_t *pConfig)
{
    Status_t status = STATUS_ERROR_PARAM;

    if ((pConfig != NULL) && temp_initialized)
    {
        temp_config = *pConfig;

        /* Update hardware limits */
        int16_t tlowRaw = (int16_t)((temp_config.lowLimit_mC * 16) / 1000);
        int16_t thighRaw = (int16_t)((temp_config.highLimit_mC * 16) / 1000);

        status = tmp1075_write_register(TMP1075_REG_TLOW, (uint16_t)tlowRaw);

        if (status == STATUS_OK)
        {
            status = tmp1075_write_register(TMP1075_REG_THIGH, (uint16_t)thighRaw);
        }

        /* Reconfigure filter if size changed */
        if (status == STATUS_OK)
        {
            (void)Filter_MA_Init(&temp_filter, temp_config.filterSize);
        }
    }

    return status;
}

/**
 * @brief Register alarm callback
 */
Status_t TempSensor_RegisterAlarmCallback(TempSensor_AlarmCallback_t callback)
{
    temp_alarm_callback = callback;
    return STATUS_OK;
}

/**
 * @brief Reset temperature statistics
 */
Status_t TempSensor_ResetStatistics(void)
{
    if (temp_initialized)
    {
        temp_data.measurementCount = 0U;
        temp_data.errorCount = 0U;
        temp_data.minTemp_mC = INT32_MAX;
        temp_data.maxTemp_mC = INT32_MIN;
    }

    return STATUS_OK;
}

/**
 * @brief Check if temperature is valid
 */
bool TempSensor_IsValid(void)
{
    bool valid = false;

    if (temp_initialized)
    {
        /* Check if recent measurement (< 5 seconds old) */
        uint32_t age_ms = HAL_GetTick() - temp_data.timestamp_ms;

        if (age_ms < 5000U)
        {
            valid = true;
        }
    }

    return valid;
}

/**
 * @brief De-initialize temperature sensor
 */
Status_t TempSensor_DeInit(void)
{
    if (temp_initialized)
    {
        /* Put sensor in shutdown mode */
        (void)tmp1075_write_register(TMP1075_REG_CONFIG, TMP1075_CFG_SHUTDOWN);
        temp_initialized = false;
    }

    return STATUS_OK;
}

/*============================================================================*/
/* PRIVATE FUNCTIONS                                                          */
/*============================================================================*/

/**
 * @brief Read TMP1075 register
 */
static Status_t tmp1075_read_register(uint8_t regAddr, uint16_t *pValue)
{
    Status_t status = STATUS_ERROR_PARAM;

    if (pValue != NULL)
    {
        uint8_t data[2];

        status = BSP_I2C_Read(TMP1075_I2C_INSTANCE, TMP1075_I2C_ADDR,
                             (uint16_t)regAddr, data, 2U, TIMEOUT_I2C_MS);

        if (status == STATUS_OK)
        {
            /* TMP1075 sends MSB first */
            *pValue = ((uint16_t)data[0] << 8) | (uint16_t)data[1];
        }
    }

    return status;
}

/**
 * @brief Write TMP1075 register
 */
static Status_t tmp1075_write_register(uint8_t regAddr, uint16_t value)
{
    uint8_t data[2];

    /* TMP1075 expects MSB first */
    data[0] = (uint8_t)(value >> 8);
    data[1] = (uint8_t)(value & 0xFFU);

    return BSP_I2C_Write(TMP1075_I2C_INSTANCE, TMP1075_I2C_ADDR,
                        (uint16_t)regAddr, data, 2U, TIMEOUT_I2C_MS);
}

/**
 * @brief Read raw temperature value
 */
static Status_t tmp1075_read_temperature_raw(int16_t *pTempRaw)
{
    Status_t status = STATUS_ERROR_PARAM;

    if (pTempRaw != NULL)
    {
        uint16_t rawValue;
        status = tmp1075_read_register(TMP1075_REG_TEMP, &rawValue);

        if (status == STATUS_OK)
        {
            /* TMP1075 temperature format:
             * 12-bit mode: bits [15:4] contain temperature data
             * Signed value in two's complement
             * LSB = 0.0625°C
             */
            *pTempRaw = (int16_t)(rawValue >> 4);

            /* Sign extend if negative */
            if ((*pTempRaw & 0x0800) != 0)
            {
                *pTempRaw |= 0xF000;
            }
        }
    }

    return status;
}

/**
 * @brief Check temperature limits
 */
static void temp_check_limits(void)
{
    bool alarmTriggered = false;
    bool isOverTemp = false;

    /* Check low limit */
    if (temp_data.currentTemp_mC < temp_config.lowLimit_mC)
    {
        if (!temp_data.underTemp)
        {
            temp_data.underTemp = true;
            alarmTriggered = true;
            isOverTemp = false;
        }
    }
    else if (temp_data.currentTemp_mC > (temp_config.lowLimit_mC + temp_config.hysteresis_mC))
    {
        temp_data.underTemp = false;
    }

    /* Check high limit */
    if (temp_data.currentTemp_mC > temp_config.highLimit_mC)
    {
        if (!temp_data.overTemp)
        {
            temp_data.overTemp = true;
            alarmTriggered = true;
            isOverTemp = true;
        }
    }
    else if (temp_data.currentTemp_mC < (temp_config.highLimit_mC - temp_config.hysteresis_mC))
    {
        temp_data.overTemp = false;
    }

    /* Call alarm callback */
    if (alarmTriggered && (temp_alarm_callback != NULL))
    {
        temp_alarm_callback(temp_data.currentTemp_mC, isOverTemp);
    }
}

/*============================================================================*/
/* END OF FILE                                                                */
/*============================================================================*/
