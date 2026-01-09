/**
 * @file    lem_sensor.h
 * @brief   LEM current sensor driver (10 channels)
 * @author  Battery Control Unit Development Team
 * @date    2026-01-09
 * @version 1.0.0
 *
 * @note    MISRA C:2012 compliant
 * @note    ISO 26262 ASIL-B safety requirements
 * @note    LEM Hall-effect current sensors with REF1933 precision reference
 *
 * Hardware interface per sensor:
 * - LEM_OUT: Analog output voltage (proportional to current)
 * - LEM_OC: Digital overcurrent flag
 * - REF_LEM: Precision 2.5V reference (REF1933)
 * - LEM_SUPPLY: Supply control via ferrite bead
 *
 * @copyright Copyright (c) 2026
 */

#ifndef LEM_SENSOR_H
#define LEM_SENSOR_H

#ifdef __cplusplus
extern "C" {
#endif

/*============================================================================*/
/* INCLUDES                                                                   */
/*============================================================================*/
#include "app_types.h"
#include "app_config.h"

/*============================================================================*/
/* CONSTANTS                                                                  */
/*============================================================================*/

/** @brief LEM sensor configuration */
#define LEM_SENSOR_COUNT          (10U)    /**< Number of LEM sensors */
#define LEM_SAMPLE_RATE_HZ        (1000U)  /**< 1 kHz sampling rate */

/** @brief LEM electrical characteristics */
#define LEM_REFERENCE_VOLTAGE_MV  (2500U)  /**< 2.5V from REF1933 */
#define LEM_SENSITIVITY_MV_PER_A  (25U)    /**< 25 mV/A (typical) */
#define LEM_ZERO_CURRENT_MV       (2500U)  /**< Output at 0A */
#define LEM_MAX_CURRENT_A         (100)    /**< ±100A typical range */

/** @brief Fault detection thresholds */
#define LEM_MIN_VOLTAGE_MV        (500U)   /**< Min output voltage */
#define LEM_MAX_VOLTAGE_MV        (4500U)  /**< Max output voltage */
#define LEM_OPEN_WIRE_THRESHOLD_MV (100U)  /**< Open wire detection */

/*============================================================================*/
/* TYPE DEFINITIONS                                                           */
/*============================================================================*/

/**
 * @brief LEM sensor status
 */
typedef enum {
    LEM_STATUS_OK              = 0x00U,  /**< Sensor OK */
    LEM_STATUS_OVERCURRENT     = 0x01U,  /**< Overcurrent detected */
    LEM_STATUS_OPEN_WIRE       = 0x02U,  /**< Open wire detected */
    LEM_STATUS_OUT_OF_RANGE    = 0x03U,  /**< Reading out of range */
    LEM_STATUS_CALIBRATION_ERR = 0x04U,  /**< Calibration error */
    LEM_STATUS_REF_FAULT       = 0x05U,  /**< Reference voltage fault */
    LEM_STATUS_FAULT           = 0x06U   /**< Generic fault */
} LEM_SensorStatus_t;

/**
 * @brief LEM sensor calibration data
 */
typedef struct {
    int32_t  offsetRaw;      /**< Zero-current offset (raw ADC) */
    int32_t  gainFactor;     /**< Gain factor (scaled by 65536) */
    uint16_t sensitivity_mV; /**< Sensitivity in mV/A */
    bool     valid;          /**< Calibration data valid */
} LEM_Calibration_t;

/**
 * @brief LEM sensor measurement
 */
typedef struct {
    Current_mA_t        current_mA;     /**< Measured current (mA) */
    Voltage_mV_t        voltage_mV;     /**< LEM output voltage (mV) */
    uint16_t            rawADC;         /**< Raw ADC value */
    bool                overcurrentFlag; /**< LEM_OC flag status */
    LEM_SensorStatus_t  status;         /**< Sensor status */
    uint32_t            timestamp_ms;   /**< Measurement timestamp */
} LEM_Measurement_t;

/**
 * @brief LEM sensor configuration
 */
typedef struct {
    bool     enabled;              /**< Sensor enabled */
    int32_t  currentLimit_mA;      /**< Software current limit */
    bool     autoCalibrate;        /**< Auto-zero at startup */
    uint16_t sensitivity_mV;       /**< Custom sensitivity (0=use default) */
    uint8_t  filterSize;           /**< Moving average filter size */
} LEM_Config_t;

/**
 * @brief LEM sensor statistics
 */
typedef struct {
    uint32_t measurementCount;     /**< Total measurements */
    uint32_t overcurrentCount;     /**< Overcurrent events */
    uint32_t faultCount;           /**< Total faults */
    int32_t  minCurrent_mA;        /**< Minimum recorded current */
    int32_t  maxCurrent_mA;        /**< Maximum recorded current */
    int64_t  sumCurrent_mA;        /**< Sum for average calculation */
} LEM_Statistics_t;

/**
 * @brief LEM overcurrent callback function
 */
typedef void (*LEM_OvercurrentCallback_t)(uint8_t sensorId, Current_mA_t current);

/*============================================================================*/
/* FUNCTION PROTOTYPES                                                        */
/*============================================================================*/

/**
 * @brief Initialize LEM sensor module
 * @return STATUS_OK on success, error code otherwise
 * @req BCU_REQ_LEM_001 LEM sensor initialization
 */
Status_t LEM_Init(void);

/**
 * @brief De-initialize LEM sensor module
 * @return STATUS_OK on success
 */
Status_t LEM_DeInit(void);

/**
 * @brief Configure LEM sensor
 * @param[in] sensorId Sensor number (0-9)
 * @param[in] pConfig  Pointer to configuration structure
 * @return STATUS_OK on success, error code otherwise
 */
Status_t LEM_ConfigureSensor(uint8_t sensorId, const LEM_Config_t *pConfig);

/**
 * @brief Enable LEM sensor supply
 * @param[in] enable true to enable, false to disable
 * @return STATUS_OK on success
 */
Status_t LEM_EnableSupply(bool enable);

/**
 * @brief Read current from LEM sensor
 * @param[in]  sensorId    Sensor number (0-9)
 * @param[out] pCurrent_mA Pointer to store current in milliamps
 * @return STATUS_OK on success, error code otherwise
 * @req BCU_REQ_LEM_010 Current measurement
 */
Status_t LEM_ReadCurrent(uint8_t sensorId, Current_mA_t *pCurrent_mA);

/**
 * @brief Read multiple sensors simultaneously
 * @param[out] pCurrents Array to store currents (10 elements)
 * @return STATUS_OK on success, error code otherwise
 */
Status_t LEM_ReadAllCurrents(Current_mA_t *pCurrents);

/**
 * @brief Get full measurement data
 * @param[in]  sensorId     Sensor number (0-9)
 * @param[out] pMeasurement Pointer to store measurement data
 * @return STATUS_OK on success, error code otherwise
 */
Status_t LEM_GetMeasurement(uint8_t sensorId, LEM_Measurement_t *pMeasurement);

/**
 * @brief Get sensor status
 * @param[in]  sensorId Sensor number (0-9)
 * @param[out] pStatus  Pointer to store sensor status
 * @return STATUS_OK on success, error code otherwise
 */
Status_t LEM_GetSensorStatus(uint8_t sensorId, LEM_SensorStatus_t *pStatus);

/**
 * @brief Check overcurrent flag (LEM_OC pin)
 * @param[in]  sensorId      Sensor number (0-9)
 * @param[out] pOvercurrent  true if overcurrent detected, false otherwise
 * @return STATUS_OK on success, error code otherwise
 */
Status_t LEM_CheckOvercurrentFlag(uint8_t sensorId, bool *pOvercurrent);

/**
 * @brief Calibrate LEM sensor (zero-current calibration)
 * @param[in] sensorId Sensor number (0-9)
 * @return STATUS_OK on success, error code otherwise
 * @note  Must be performed with zero current through sensor
 * @req BCU_REQ_LEM_020 Sensor calibration
 */
Status_t LEM_CalibrateSensor(uint8_t sensorId);

/**
 * @brief Calibrate all LEM sensors
 * @return STATUS_OK on success, error code otherwise
 * @note  Must be performed with zero current through all sensors
 */
Status_t LEM_CalibrateAll(void);

/**
 * @brief Set sensor calibration data
 * @param[in] sensorId     Sensor number (0-9)
 * @param[in] pCalibration Pointer to calibration data
 * @return STATUS_OK on success, error code otherwise
 */
Status_t LEM_SetCalibration(uint8_t sensorId, const LEM_Calibration_t *pCalibration);

/**
 * @brief Get sensor calibration data
 * @param[in]  sensorId     Sensor number (0-9)
 * @param[out] pCalibration Pointer to store calibration data
 * @return STATUS_OK on success, error code otherwise
 */
Status_t LEM_GetCalibration(uint8_t sensorId, LEM_Calibration_t *pCalibration);

/**
 * @brief Load calibration data from FRAM
 * @return STATUS_OK on success, error code otherwise
 */
Status_t LEM_LoadCalibration(void);

/**
 * @brief Save calibration data to FRAM
 * @return STATUS_OK on success, error code otherwise
 */
Status_t LEM_SaveCalibration(void);

/**
 * @brief Verify reference voltage (REF1933)
 * @param[out] pVoltage_mV Reference voltage in millivolts
 * @return STATUS_OK on success, error code otherwise
 */
Status_t LEM_VerifyReference(Voltage_mV_t *pVoltage_mV);

/**
 * @brief Perform sensor self-test
 * @param[in] sensorId Sensor number (0-9)
 * @return STATUS_OK if test passed, error code otherwise
 */
Status_t LEM_SelfTest(uint8_t sensorId);

/**
 * @brief Update LEM sensors (call periodically)
 * @return STATUS_OK on success
 * @note  Performs background sampling and filtering
 * @note  Should be called every 1ms for 1 kHz sampling
 */
Status_t LEM_Update(void);

/**
 * @brief Get sensor statistics
 * @param[in]  sensorId Sensor number (0-9)
 * @param[out] pStats   Pointer to statistics structure
 * @return STATUS_OK on success, error code otherwise
 */
Status_t LEM_GetStatistics(uint8_t sensorId, LEM_Statistics_t *pStats);

/**
 * @brief Reset sensor statistics
 * @param[in] sensorId Sensor number (0-9), or 0xFF for all
 * @return STATUS_OK on success
 */
Status_t LEM_ResetStatistics(uint8_t sensorId);

/**
 * @brief Register overcurrent callback
 * @param[in] callback Callback function pointer
 * @return STATUS_OK on success
 */
Status_t LEM_RegisterOvercurrentCallback(LEM_OvercurrentCallback_t callback);

/**
 * @brief Convert raw ADC value to current
 * @param[in] rawADC       Raw ADC value
 * @param[in] pCalibration Pointer to calibration data
 * @return Current in milliamps
 */
Current_mA_t LEM_ADCToCurrent(uint16_t rawADC, const LEM_Calibration_t *pCalibration);

/**
 * @brief Convert current to expected ADC value
 * @param[in] current_mA   Current in milliamps
 * @param[in] pCalibration Pointer to calibration data
 * @return Expected ADC value
 */
uint16_t LEM_CurrentToADC(Current_mA_t current_mA, const LEM_Calibration_t *pCalibration);

#ifdef __cplusplus
}
#endif

#endif /* LEM_SENSOR_H */
