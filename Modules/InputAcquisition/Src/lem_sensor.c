/**
 * @file    lem_sensor.c
 * @brief   LEM HOYS current sensor driver implementation
 * @author  Battery Control Unit Development Team
 * @date    2026-01-09
 * @version 1.0.0
 *
 * @note    MISRA C:2012 compliant
 * @note    LEM HOYS series Hall-effect current sensors
 * @note    10 sensors with calibration and fault detection
 *
 * @copyright Copyright (c) 2026
 */

/*============================================================================*/
/* INCLUDES                                                                   */
/*============================================================================*/
#include "lem_sensor.h"
#include "bsp_adc.h"
#include "bsp_gpio.h"
#include "filter.h"
#include <string.h>

/*============================================================================*/
/* PRIVATE CONSTANTS                                                          */
/*============================================================================*/

/** @brief ADC channel mapping for LEM sensors */
static const uint8_t LEM_ADC_CHANNELS[LEM_SENSOR_COUNT] = {
    1, 2, 3, 4, 5, 6, 7, 8, 9, 10
};

/*============================================================================*/
/* PRIVATE VARIABLES                                                          */
/*============================================================================*/

/** @brief LEM sensor configurations */
static LEM_Config_t lem_config[LEM_SENSOR_COUNT];

/** @brief LEM sensor measurements */
static LEM_Measurement_t lem_measurements[LEM_SENSOR_COUNT];

/** @brief LEM sensor calibration */
static LEM_Calibration_t lem_calibration[LEM_SENSOR_COUNT];

/** @brief LEM sensor statistics */
static LEM_Statistics_t lem_stats[LEM_SENSOR_COUNT];

/** @brief Moving average filters for each sensor */
static Filter_MovingAverage_t lem_filters[LEM_SENSOR_COUNT];

/** @brief Overcurrent callback */
static LEM_OvercurrentCallback_t lem_overcurrent_callback = NULL;

/** @brief Initialization flag */
static bool lem_initialized = false;

/*============================================================================*/
/* PRIVATE FUNCTION PROTOTYPES                                                */
/*============================================================================*/

static Status_t lem_read_adc(uint8_t sensorId, uint16_t *pRawADC);
static Status_t lem_check_oc_flag(uint8_t sensorId, bool *pOC);

/*============================================================================*/
/* PUBLIC FUNCTIONS                                                           */
/*============================================================================*/

/**
 * @brief Initialize LEM sensor module
 */
Status_t LEM_Init(void)
{
    Status_t status = STATUS_OK;

    if (lem_initialized)
    {
        status = STATUS_ERROR_ALREADY_INIT;
    }
    else
    {
        uint8_t i;

        /* Initialize all sensors with default config */
        for (i = 0U; i < LEM_SENSOR_COUNT; i++)
        {
            lem_config[i].enabled = true;
            lem_config[i].currentLimit_mA = LEM_MAX_CURRENT_A * 1000;
            lem_config[i].autoCalibrate = false;
            lem_config[i].sensitivity_mV = LEM_SENSITIVITY_MV_PER_A;
            lem_config[i].filterSize = 8U;

            /* Initialize calibration with defaults */
            lem_calibration[i].offsetRaw = 0;
            lem_calibration[i].gainFactor = 65536;  /* Unity gain */
            lem_calibration[i].sensitivity_mV = LEM_SENSITIVITY_MV_PER_A;
            lem_calibration[i].valid = false;

            /* Initialize filter */
            (void)Filter_MA_Init(&lem_filters[i], lem_config[i].filterSize);

            /* Reset statistics */
            (void)memset(&lem_stats[i], 0, sizeof(LEM_Statistics_t));
            lem_stats[i].minCurrent_mA = INT32_MAX;
            lem_stats[i].maxCurrent_mA = INT32_MIN;

            /* Clear measurement */
            (void)memset(&lem_measurements[i], 0, sizeof(LEM_Measurement_t));
        }

        /* Enable LEM sensor supply */
        status = LEM_EnableSupply(true);

        if (status == STATUS_OK)
        {
            lem_initialized = true;
        }
    }

    return status;
}

/**
 * @brief Enable LEM sensor supply
 */
Status_t LEM_EnableSupply(bool enable)
{
    Status_t status = STATUS_OK;

    /* Control LEM_SUPPLY pin (PD2) */
    GPIO_State_t state = enable ? GPIO_STATE_HIGH : GPIO_STATE_LOW;
    status = BSP_GPIO_WritePin(GPIOD, GPIO_PIN_2, state);

    if (enable)
    {
        /* Wait for supply to stabilize */
        HAL_Delay(10);
    }

    return status;
}

/**
 * @brief Read current from LEM sensor
 */
Status_t LEM_ReadCurrent(uint8_t sensorId, Current_mA_t *pCurrent_mA)
{
    Status_t status = STATUS_ERROR_PARAM;

    if ((sensorId < LEM_SENSOR_COUNT) && (pCurrent_mA != NULL) && lem_initialized)
    {
        uint16_t rawADC;

        status = lem_read_adc(sensorId, &rawADC);

        if (status == STATUS_OK)
        {
            /* Apply filter */
            int32_t filtered = Filter_MA_Update(&lem_filters[sensorId], (int32_t)rawADC);

            /* Convert to current */
            *pCurrent_mA = LEM_ADCToCurrent((uint16_t)filtered, &lem_calibration[sensorId]);

            /* Update measurement */
            lem_measurements[sensorId].current_mA = *pCurrent_mA;
            lem_measurements[sensorId].rawADC = (uint16_t)filtered;
            lem_measurements[sensorId].voltage_mV = BSP_ADC_ToMillivolts((uint16_t)filtered);
            lem_measurements[sensorId].timestamp_ms = HAL_GetTick();

            /* Update statistics */
            lem_stats[sensorId].measurementCount++;
            lem_stats[sensorId].sumCurrent_mA += (int64_t)*pCurrent_mA;

            if (*pCurrent_mA < lem_stats[sensorId].minCurrent_mA)
            {
                lem_stats[sensorId].minCurrent_mA = *pCurrent_mA;
            }

            if (*pCurrent_mA > lem_stats[sensorId].maxCurrent_mA)
            {
                lem_stats[sensorId].maxCurrent_mA = *pCurrent_mA;
            }

            /* Check overcurrent */
            if ((*pCurrent_mA > lem_config[sensorId].currentLimit_mA) ||
                (*pCurrent_mA < -lem_config[sensorId].currentLimit_mA))
            {
                lem_measurements[sensorId].status = LEM_STATUS_OVERCURRENT;
                lem_stats[sensorId].overcurrentCount++;

                if (lem_overcurrent_callback != NULL)
                {
                    lem_overcurrent_callback(sensorId, *pCurrent_mA);
                }
            }
            else
            {
                lem_measurements[sensorId].status = LEM_STATUS_OK;
            }
        }
    }

    return status;
}

/**
 * @brief Calibrate LEM sensor (zero-current)
 */
Status_t LEM_CalibrateSensor(uint8_t sensorId)
{
    Status_t status = STATUS_ERROR_PARAM;

    if ((sensorId < LEM_SENSOR_COUNT) && lem_initialized)
    {
        uint32_t sum = 0U;
        uint8_t i;
        uint8_t samples = 32U;

        /* Take multiple samples */
        for (i = 0U; i < samples; i++)
        {
            uint16_t rawADC;

            if (lem_read_adc(sensorId, &rawADC) == STATUS_OK)
            {
                sum += (uint32_t)rawADC;
            }

            HAL_Delay(5);
        }

        /* Calculate average offset */
        lem_calibration[sensorId].offsetRaw = (int32_t)(sum / (uint32_t)samples);
        lem_calibration[sensorId].valid = true;

        status = STATUS_OK;
    }

    return status;
}

/**
 * @brief Convert ADC value to current
 */
Current_mA_t LEM_ADCToCurrent(uint16_t rawADC, const LEM_Calibration_t *pCalibration)
{
    Current_mA_t current_mA = 0;

    if ((pCalibration != NULL) && pCalibration->valid)
    {
        /* Apply offset */
        int32_t calibrated = (int32_t)rawADC - pCalibration->offsetRaw;

        /* Convert ADC to voltage */
        int32_t voltage_mV = (calibrated * (int32_t)BSP_ADC_VREF_MV) / (int32_t)BSP_ADC_MAX_VALUE;

        /* Voltage offset from zero (LEM_ZERO_CURRENT_MV) */
        int32_t voltage_offset = voltage_mV - (int32_t)LEM_ZERO_CURRENT_MV;

        /* Convert to current: I = V / sensitivity */
        /* sensitivity in mV/A, result in mA */
        current_mA = (voltage_offset * 1000) / (int32_t)pCalibration->sensitivity_mV;

        /* Apply gain */
        current_mA = (current_mA * (int64_t)pCalibration->gainFactor) >> 16;
    }

    return current_mA;
}

/**
 * @brief Get sensor measurement
 */
Status_t LEM_GetMeasurement(uint8_t sensorId, LEM_Measurement_t *pMeasurement)
{
    Status_t status = STATUS_ERROR_PARAM;

    if ((sensorId < LEM_SENSOR_COUNT) && (pMeasurement != NULL) && lem_initialized)
    {
        *pMeasurement = lem_measurements[sensorId];
        status = STATUS_OK;
    }

    return status;
}

/**
 * @brief Check overcurrent flag
 */
Status_t LEM_CheckOvercurrentFlag(uint8_t sensorId, bool *pOvercurrent)
{
    Status_t status = STATUS_ERROR_PARAM;

    if ((sensorId < LEM_SENSOR_COUNT) && (pOvercurrent != NULL) && lem_initialized)
    {
        status = lem_check_oc_flag(sensorId, pOvercurrent);

        if (status == STATUS_OK)
        {
            lem_measurements[sensorId].overcurrentFlag = *pOvercurrent;
        }
    }

    return status;
}

/**
 * @brief Set calibration data
 */
Status_t LEM_SetCalibration(uint8_t sensorId, const LEM_Calibration_t *pCalibration)
{
    Status_t status = STATUS_ERROR_PARAM;

    if ((sensorId < LEM_SENSOR_COUNT) && (pCalibration != NULL))
    {
        lem_calibration[sensorId] = *pCalibration;
        status = STATUS_OK;
    }

    return status;
}

/**
 * @brief Get calibration data
 */
Status_t LEM_GetCalibration(uint8_t sensorId, LEM_Calibration_t *pCalibration)
{
    Status_t status = STATUS_ERROR_PARAM;

    if ((sensorId < LEM_SENSOR_COUNT) && (pCalibration != NULL))
    {
        *pCalibration = lem_calibration[sensorId];
        status = STATUS_OK;
    }

    return status;
}

/**
 * @brief Get sensor statistics
 */
Status_t LEM_GetStatistics(uint8_t sensorId, LEM_Statistics_t *pStats)
{
    Status_t status = STATUS_ERROR_PARAM;

    if ((sensorId < LEM_SENSOR_COUNT) && (pStats != NULL))
    {
        *pStats = lem_stats[sensorId];
        status = STATUS_OK;
    }

    return status;
}

/**
 * @brief Reset sensor statistics
 */
Status_t LEM_ResetStatistics(uint8_t sensorId)
{
    Status_t status = STATUS_ERROR_PARAM;

    if ((sensorId < LEM_SENSOR_COUNT) || (sensorId == 0xFFU))
    {
        if (sensorId == 0xFFU)
        {
            /* Reset all */
            for (uint8_t i = 0U; i < LEM_SENSOR_COUNT; i++)
            {
                (void)memset(&lem_stats[i], 0, sizeof(LEM_Statistics_t));
                lem_stats[i].minCurrent_mA = INT32_MAX;
                lem_stats[i].maxCurrent_mA = INT32_MIN;
            }
        }
        else
        {
            (void)memset(&lem_stats[sensorId], 0, sizeof(LEM_Statistics_t));
            lem_stats[sensorId].minCurrent_mA = INT32_MAX;
            lem_stats[sensorId].maxCurrent_mA = INT32_MIN;
        }

        status = STATUS_OK;
    }

    return status;
}

/**
 * @brief Register overcurrent callback
 */
Status_t LEM_RegisterOvercurrentCallback(LEM_OvercurrentCallback_t callback)
{
    lem_overcurrent_callback = callback;
    return STATUS_OK;
}

/**
 * @brief Update LEM sensors (call periodically at 1 kHz)
 */
Status_t LEM_Update(void)
{
    Status_t status = STATUS_OK;

    if (lem_initialized)
    {
        uint8_t i;

        for (i = 0U; i < LEM_SENSOR_COUNT; i++)
        {
            if (lem_config[i].enabled)
            {
                Current_mA_t current;
                (void)LEM_ReadCurrent(i, &current);
            }
        }
    }

    return status;
}

/**
 * @brief De-initialize LEM module
 */
Status_t LEM_DeInit(void)
{
    if (lem_initialized)
    {
        (void)LEM_EnableSupply(false);
        lem_initialized = false;
    }

    return STATUS_OK;
}

/*============================================================================*/
/* PRIVATE FUNCTIONS                                                          */
/*============================================================================*/

/**
 * @brief Read ADC value for LEM sensor
 */
static Status_t lem_read_adc(uint8_t sensorId, uint16_t *pRawADC)
{
    Status_t status = STATUS_ERROR_PARAM;

    if ((sensorId < LEM_SENSOR_COUNT) && (pRawADC != NULL))
    {
        uint8_t adcChannel = LEM_ADC_CHANNELS[sensorId];
        status = BSP_ADC_ReadChannel(adcChannel, pRawADC, TIMEOUT_ADC_MS);
    }

    return status;
}

/**
 * @brief Check LEM overcurrent flag GPIO
 */
static Status_t lem_check_oc_flag(uint8_t sensorId, bool *pOC)
{
    Status_t status = STATUS_ERROR_PARAM;

    if ((sensorId < LEM_SENSOR_COUNT) && (pOC != NULL))
    {
        /* Map sensor ID to GPIO pin */
        /* LEM_OC pins: PF1, PF3, PF5, PF7, PF9, PF11, PF13, PF15, PG1, PG3 */
        uint16_t ocPins[] = {
            GPIO_PIN_1, GPIO_PIN_3, GPIO_PIN_5, GPIO_PIN_7, GPIO_PIN_9,
            GPIO_PIN_11, GPIO_PIN_13, GPIO_PIN_15, GPIO_PIN_1, GPIO_PIN_3
        };

        GPIO_TypeDef *port = (sensorId < 8U) ? GPIOF : GPIOG;
        uint16_t pin = ocPins[sensorId];

        GPIO_State_t state;
        status = BSP_GPIO_ReadPin(port, pin, &state);

        if (status == STATUS_OK)
        {
            *pOC = (state == GPIO_STATE_HIGH);
        }
    }

    return status;
}

/*============================================================================*/
/* END OF FILE                                                                */
/*============================================================================*/
