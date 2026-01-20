/**
 * @file    temp_sensor.h
 * @brief   TMP1075 temperature sensor driver
 * @author  Battery Control Unit Development Team
 * @date    2026-01-10
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
/* CONSTANTS                                                                  */
/*============================================================================*/

/** @brief Default temperature limits */
#define TEMP_SENSOR_MIN_LIMIT_MC   (-40000)  /**< -40°C in milliCelsius */
#define TEMP_SENSOR_MAX_LIMIT_MC   (85000)   /**< +85°C in milliCelsius */

/** @brief TMP1075 conversion constants */
#define TMP1075_TEMP_SCALE_FACTOR  (16U)     /**< LSB to °C conversion factor (2^4) */
#define TMP1075_MC_PER_LSB         (62.5f)   /**< milliCelsius per LSB (1000/16) */

/*============================================================================*/
/* TYPE DEFINITIONS                                                           */
/*============================================================================*/

/**
 * @brief Temperature sensor configuration
 */
typedef struct {
    bool     enabled;           /**< Sensor enable flag */
    int32_t  lowLimit_mC;       /**< Low temperature limit (milliCelsius) */
    int32_t  highLimit_mC;      /**< High temperature limit (milliCelsius) */
    int32_t  hysteresis_mC;     /**< Hysteresis (milliCelsius) */
    uint8_t  filterSize;        /**< Moving average filter size */
} TempSensor_Config_t;

/**
 * @brief Temperature sensor data
 */
typedef struct {
    int32_t  currentTemp_mC;    /**< Current filtered temperature (mC) */
    int32_t  rawTemp_mC;        /**< Raw temperature reading (mC) */
    int32_t  minTemp_mC;        /**< Minimum temperature recorded (mC) */
    int32_t  maxTemp_mC;        /**< Maximum temperature recorded (mC) */
    uint32_t timestamp_ms;      /**< Last measurement timestamp */
    uint32_t measurementCount;  /**< Total measurement count */
    uint32_t errorCount;        /**< Error count */
    bool     overTemp;          /**< Over-temperature flag */
    bool     underTemp;         /**< Under-temperature flag */
} TempSensor_Data_t;

/**
 * @brief Temperature alarm types
 */
typedef enum {
    TEMP_ALARM_NONE     = 0x00U,  /**< No alarm */
    TEMP_ALARM_LOW      = 0x01U,  /**< Under-temperature alarm */
    TEMP_ALARM_HIGH     = 0x02U,  /**< Over-temperature alarm */
    TEMP_ALARM_CRITICAL = 0x03U   /**< Critical temperature alarm */
} TempSensor_AlarmType_t;

/**
 * @brief Temperature alarm callback function type
 * @param[in] alarmType    Type of temperature alarm
 * @param[in] temp_mC      Current temperature in milliCelsius
 */
typedef void (*TempSensor_AlarmCallback_t)(TempSensor_AlarmType_t alarmType, int32_t temp_mC);

/*============================================================================*/
/* FUNCTION PROTOTYPES                                                        */
/*============================================================================*/

/**
 * @brief Initialize temperature sensor
 * @return STATUS_OK on success, error code otherwise
 */
Status_t TempSensor_Init(void);

/**
 * @brief De-initialize temperature sensor
 * @return STATUS_OK on success
 */
Status_t TempSensor_DeInit(void);

/**
 * @brief Read current temperature
 * @param[out] pTemp_mC Pointer to store temperature in milliCelsius
 * @return STATUS_OK on success, error code otherwise
 */
Status_t TempSensor_ReadTemperature(int32_t *pTemp_mC);

/**
 * @brief Get temperature sensor data
 * @param[out] pData Pointer to store temperature data
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
 * @brief Register alarm callback
 * @param[in] callback Callback function pointer
 * @return STATUS_OK on success
 */
Status_t TempSensor_RegisterAlarmCallback(TempSensor_AlarmCallback_t callback);

/**
 * @brief Reset temperature statistics
 * @return STATUS_OK on success
 */
Status_t TempSensor_ResetStatistics(void);

/**
 * @brief Check if temperature reading is valid
 * @return true if valid, false otherwise
 */
bool TempSensor_IsValid(void);

#ifdef __cplusplus
}
#endif

#endif /* TEMP_SENSOR_H */
