/**
 * @file    temp_sensor.h
 * @brief   TMP1075 temperature sensor driver interface
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

#ifndef TEMP_SENSOR_H
#define TEMP_SENSOR_H

#ifdef __cplusplus
extern "C" {
#endif

/*============================================================================*/
/* INCLUDES                                                                   */
/*============================================================================*/
#include "app_types.h"
#include <stdint.h>
#include <stdbool.h>

/*============================================================================*/
/* CONSTANTS AND MACROS                                                       */
/*============================================================================*/

/** @brief Default temperature warning threshold (°C * 100) */
#define TEMP_WARNING_THRESHOLD_MC   (7000)   /**< 70.0°C */

/** @brief Default temperature critical threshold (°C * 100) */
#define TEMP_CRITICAL_THRESHOLD_MC  (8500)   /**< 85.0°C */

/** @brief Temperature sensor accuracy (±°C * 100) */
#define TEMP_SENSOR_ACCURACY_MC     (50)     /**< ±0.5°C */

/** @brief Temperature reading invalid value */
#define TEMP_INVALID_VALUE          (-32768) /**< Invalid temperature reading */

/*============================================================================*/
/* TYPE DEFINITIONS                                                           */
/*============================================================================*/

/** @brief Temperature sensor configuration */
typedef struct {
    int32_t warningThreshold_mC;    /**< Warning threshold (milli-Celsius) */
    int32_t criticalThreshold_mC;   /**< Critical threshold (milli-Celsius) */
    uint16_t filterSize;            /**< Moving average filter size (0=disabled) */
    bool alarmEnable;               /**< Enable temperature alarm callbacks */
} TempSensor_Config_t;

/** @brief Temperature sensor data */
typedef struct {
    int32_t temperature_mC;         /**< Current temperature (milli-Celsius) */
    int32_t temperatureMin_mC;      /**< Minimum temperature recorded */
    int32_t temperatureMax_mC;      /**< Maximum temperature recorded */
    bool warningActive;             /**< Warning threshold exceeded flag */
    bool criticalActive;            /**< Critical threshold exceeded flag */
    bool sensorFault;               /**< Sensor communication fault flag */
    uint32_t lastReadTime_ms;       /**< Timestamp of last successful read */
    uint32_t readCount;             /**< Number of successful reads */
    uint32_t errorCount;            /**< Number of read errors */
} TempSensor_Data_t;

/** @brief Temperature alarm callback function type */
typedef void (*TempSensor_AlarmCallback_t)(int32_t temperature_mC, bool critical);

/*============================================================================*/
/* FUNCTION PROTOTYPES                                                        */
/*============================================================================*/

/**
 * @brief Initialize temperature sensor module
 * @return STATUS_OK on success, error code otherwise
 */
Status_t TempSensor_Init(void);

/**
 * @brief Read current temperature
 * @param[out] pTemp_mC Pointer to store temperature in milli-Celsius
 * @return STATUS_OK on success, error code otherwise
 */
Status_t TempSensor_ReadTemperature(int32_t *pTemp_mC);

/**
 * @brief Get detailed temperature sensor data
 * @param[out] pData Pointer to store temperature data structure
 * @return STATUS_OK on success, error code otherwise
 */
Status_t TempSensor_GetData(TempSensor_Data_t *pData);

/**
 * @brief Configure temperature sensor
 * @param[in] pConfig Pointer to configuration structure
 * @return STATUS_OK on success, error code otherwise
 */
Status_t TempSensor_Configure(const TempSensor_Config_t *pConfig);

/**
 * @brief Register temperature alarm callback
 * @param[in] callback Callback function pointer (NULL to disable)
 * @return STATUS_OK on success, error code otherwise
 */
Status_t TempSensor_RegisterAlarmCallback(TempSensor_AlarmCallback_t callback);

/**
 * @brief Reset temperature statistics (min/max values)
 * @return STATUS_OK on success, error code otherwise
 */
Status_t TempSensor_ResetStatistics(void);

/**
 * @brief Check if temperature reading is valid
 * @return true if valid, false if sensor fault detected
 */
bool TempSensor_IsValid(void);

/**
 * @brief De-initialize temperature sensor module
 * @return STATUS_OK on success, error code otherwise
 */
Status_t TempSensor_DeInit(void);

#ifdef __cplusplus
}
#endif

#endif /* TEMP_SENSOR_H */
