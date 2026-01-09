/**
 * @file    bsp_adc.h
 * @brief   ADC abstraction layer for STM32F413
 * @author  Battery Control Unit Development Team
 * @date    2026-01-09
 * @version 1.0.0
 *
 * @note    MISRA C:2012 compliant
 * @note    12-bit ADC with DMA support and oversampling
 *
 * @copyright Copyright (c) 2026
 */

#ifndef BSP_ADC_H
#define BSP_ADC_H

#ifdef __cplusplus
extern "C" {
#endif

/*============================================================================*/
/* INCLUDES                                                                   */
/*============================================================================*/
#include "app_types.h"
#include "stm32f4xx_hal.h"

/*============================================================================*/
/* CONSTANTS                                                                  */
/*============================================================================*/

/** @brief ADC resolution */
#define BSP_ADC_RESOLUTION_12BIT  (12U)    /**< 12-bit resolution */
#define BSP_ADC_MAX_VALUE         (4095U)  /**< Maximum ADC value (2^12 - 1) */

/** @brief ADC reference voltage (millivolts) */
#define BSP_ADC_VREF_MV           (3300U)  /**< 3.3V reference */

/** @brief ADC channel assignments */
#define ADC_CH_POWER_IMON         (0U)   /**< LM74900 current monitor */
#define ADC_CH_LEM_OUT_0          (1U)   /**< LEM sensor 0 output */
#define ADC_CH_LEM_OUT_1          (2U)   /**< LEM sensor 1 output */
#define ADC_CH_LEM_OUT_2          (3U)   /**< LEM sensor 2 output */
#define ADC_CH_LEM_OUT_3          (4U)   /**< LEM sensor 3 output */
#define ADC_CH_LEM_OUT_4          (5U)   /**< LEM sensor 4 output */
#define ADC_CH_LEM_OUT_5          (6U)   /**< LEM sensor 5 output */
#define ADC_CH_LEM_OUT_6          (7U)   /**< LEM sensor 6 output */
#define ADC_CH_LEM_OUT_7          (8U)   /**< LEM sensor 7 output */
#define ADC_CH_LEM_OUT_8          (9U)   /**< LEM sensor 8 output */
#define ADC_CH_LEM_OUT_9          (10U)  /**< LEM sensor 9 output */
#define ADC_CH_BTT_IS_0           (11U)  /**< BTT6200 IC0 current sense */
#define ADC_CH_BTT_IS_1           (12U)  /**< BTT6200 IC1 current sense */
#define ADC_CH_BTT_IS_2           (13U)  /**< BTT6200 IC2 current sense */
#define ADC_CH_BTT_IS_3           (14U)  /**< BTT6200 IC3 current sense */
#define ADC_CH_BTT_IS_4           (15U)  /**< BTT6200 IC4 current sense */

/** @brief Internal ADC channels */
#define ADC_CH_VREFINT            (17U)  /**< Internal reference */
#define ADC_CH_TEMPSENSOR         (18U)  /**< Internal temperature */
#define ADC_CH_VBAT               (18U)  /**< Battery voltage (divided by 2) */

/** @brief Total ADC channels */
#define BSP_ADC_CHANNEL_COUNT     (16U)  /**< Total external channels */

/** @brief Oversampling configuration */
#define BSP_ADC_OVERSAMPLE_RATIO  (16U)  /**< 16x oversampling */

/*============================================================================*/
/* TYPE DEFINITIONS                                                           */
/*============================================================================*/

/**
 * @brief ADC sampling time
 */
typedef enum {
    ADC_SAMPLE_3_CYCLES   = 0x00U,  /**< 3 cycles */
    ADC_SAMPLE_15_CYCLES  = 0x01U,  /**< 15 cycles */
    ADC_SAMPLE_28_CYCLES  = 0x02U,  /**< 28 cycles */
    ADC_SAMPLE_56_CYCLES  = 0x03U,  /**< 56 cycles */
    ADC_SAMPLE_84_CYCLES  = 0x04U,  /**< 84 cycles */
    ADC_SAMPLE_112_CYCLES = 0x05U,  /**< 112 cycles */
    ADC_SAMPLE_144_CYCLES = 0x06U,  /**< 144 cycles */
    ADC_SAMPLE_480_CYCLES = 0x07U   /**< 480 cycles (most accurate) */
} ADC_SampleTime_t;

/**
 * @brief ADC conversion mode
 */
typedef enum {
    ADC_MODE_SINGLE       = 0x00U,  /**< Single conversion */
    ADC_MODE_CONTINUOUS   = 0x01U,  /**< Continuous conversion */
    ADC_MODE_SCAN         = 0x02U   /**< Scan multiple channels */
} ADC_Mode_t;

/**
 * @brief ADC trigger source
 */
typedef enum {
    ADC_TRIGGER_SOFTWARE  = 0x00U,  /**< Software trigger */
    ADC_TRIGGER_TIMER     = 0x01U,  /**< Timer trigger */
    ADC_TRIGGER_EXTERNAL  = 0x02U   /**< External trigger */
} ADC_Trigger_t;

/**
 * @brief ADC channel configuration
 */
typedef struct {
    uint8_t          channel;     /**< Channel number (0-18) */
    ADC_SampleTime_t sampleTime;  /**< Sampling time */
    bool             differential; /**< Differential mode */
    uint8_t          rank;        /**< Conversion rank (for scan mode) */
} ADC_ChannelConfig_t;

/**
 * @brief ADC configuration structure
 */
typedef struct {
    ADC_Mode_t    mode;           /**< Conversion mode */
    ADC_Trigger_t trigger;        /**< Trigger source */
    bool          useDMA;         /**< Use DMA for transfers */
    bool          oversampling;   /**< Enable oversampling */
    uint8_t       oversampleRatio;/**< Oversampling ratio (2-256) */
} ADC_Config_t;

/**
 * @brief ADC calibration data
 */
typedef struct {
    int32_t offset;     /**< Offset calibration (raw ADC counts) */
    int32_t gain;       /**< Gain calibration (scaled by 65536) */
    bool    valid;      /**< Calibration data valid */
} ADC_Calibration_t;

/**
 * @brief ADC statistics structure
 */
typedef struct {
    uint32_t conversionCount;  /**< Total conversions */
    uint32_t overrunCount;     /**< Overrun errors */
    uint32_t timeoutCount;     /**< Timeout errors */
    uint32_t dmaErrorCount;    /**< DMA errors */
} ADC_Statistics_t;

/**
 * @brief ADC conversion complete callback
 */
typedef void (*ADC_ConversionCallback_t)(uint8_t channel, uint16_t value);

/*============================================================================*/
/* FUNCTION PROTOTYPES                                                        */
/*============================================================================*/

/**
 * @brief Initialize ADC module
 * @param[in] pConfig Pointer to configuration structure
 * @return STATUS_OK on success, error code otherwise
 */
Status_t BSP_ADC_Init(const ADC_Config_t *pConfig);

/**
 * @brief De-initialize ADC module
 * @return STATUS_OK on success
 */
Status_t BSP_ADC_DeInit(void);

/**
 * @brief Configure ADC channel
 * @param[in] pChannelConfig Pointer to channel configuration
 * @return STATUS_OK on success, error code otherwise
 */
Status_t BSP_ADC_ConfigureChannel(const ADC_ChannelConfig_t *pChannelConfig);

/**
 * @brief Start ADC conversion (single or continuous)
 * @return STATUS_OK on success, error code otherwise
 */
Status_t BSP_ADC_Start(void);

/**
 * @brief Stop ADC conversion
 * @return STATUS_OK on success
 */
Status_t BSP_ADC_Stop(void);

/**
 * @brief Read ADC channel (blocking)
 * @param[in]  channel    Channel number
 * @param[out] pValue     Pointer to store ADC value (0-4095)
 * @param[in]  timeout_ms Timeout in milliseconds
 * @return STATUS_OK on success, error code otherwise
 */
Status_t BSP_ADC_ReadChannel(uint8_t channel, uint16_t *pValue,
                             uint32_t timeout_ms);

/**
 * @brief Read ADC channel with averaging
 * @param[in]  channel    Channel number
 * @param[in]  samples    Number of samples to average
 * @param[out] pValue     Pointer to store averaged value
 * @param[in]  timeout_ms Timeout in milliseconds
 * @return STATUS_OK on success, error code otherwise
 */
Status_t BSP_ADC_ReadChannelAverage(uint8_t channel, uint8_t samples,
                                    uint16_t *pValue, uint32_t timeout_ms);

/**
 * @brief Read multiple ADC channels (DMA)
 * @param[in]  pChannels  Array of channel numbers
 * @param[in]  count      Number of channels
 * @param[out] pValues    Array to store ADC values
 * @param[in]  timeout_ms Timeout in milliseconds
 * @return STATUS_OK on success, error code otherwise
 */
Status_t BSP_ADC_ReadMultiple(const uint8_t *pChannels, uint8_t count,
                              uint16_t *pValues, uint32_t timeout_ms);

/**
 * @brief Convert ADC value to millivolts
 * @param[in] adcValue ADC value (0-4095)
 * @return Voltage in millivolts
 */
uint32_t BSP_ADC_ToMillivolts(uint16_t adcValue);

/**
 * @brief Convert millivolts to ADC value
 * @param[in] millivolts Voltage in millivolts
 * @return ADC value (0-4095)
 */
uint16_t BSP_ADC_FromMillivolts(uint32_t millivolts);

/**
 * @brief Apply calibration to ADC value
 * @param[in] rawValue      Raw ADC value
 * @param[in] pCalibration  Pointer to calibration data
 * @return Calibrated ADC value
 */
int32_t BSP_ADC_ApplyCalibration(uint16_t rawValue,
                                 const ADC_Calibration_t *pCalibration);

/**
 * @brief Set channel calibration data
 * @param[in] channel       Channel number
 * @param[in] pCalibration  Pointer to calibration data
 * @return STATUS_OK on success
 */
Status_t BSP_ADC_SetCalibration(uint8_t channel,
                                const ADC_Calibration_t *pCalibration);

/**
 * @brief Get channel calibration data
 * @param[in]  channel       Channel number
 * @param[out] pCalibration  Pointer to store calibration data
 * @return STATUS_OK on success
 */
Status_t BSP_ADC_GetCalibration(uint8_t channel,
                                ADC_Calibration_t *pCalibration);

/**
 * @brief Perform ADC self-calibration
 * @return STATUS_OK on success, error code otherwise
 */
Status_t BSP_ADC_Calibrate(void);

/**
 * @brief Read internal temperature sensor
 * @param[out] pTemperature Temperature in 0.01°C resolution
 * @return STATUS_OK on success
 */
Status_t BSP_ADC_ReadTemperature(Temperature_t *pTemperature);

/**
 * @brief Read internal reference voltage
 * @param[out] pVoltage_mV Reference voltage in millivolts
 * @return STATUS_OK on success
 */
Status_t BSP_ADC_ReadVref(uint32_t *pVoltage_mV);

/**
 * @brief Read VBAT voltage
 * @param[out] pVoltage_mV Battery voltage in millivolts
 * @return STATUS_OK on success
 */
Status_t BSP_ADC_ReadVbat(uint32_t *pVoltage_mV);

/**
 * @brief Get ADC statistics
 * @param[out] pStats Pointer to statistics structure
 * @return STATUS_OK on success
 */
Status_t BSP_ADC_GetStatistics(ADC_Statistics_t *pStats);

/**
 * @brief Reset ADC statistics
 * @return STATUS_OK on success
 */
Status_t BSP_ADC_ResetStatistics(void);

/**
 * @brief Register conversion complete callback
 * @param[in] callback Callback function pointer
 * @return STATUS_OK on success
 */
Status_t BSP_ADC_RegisterCallback(ADC_ConversionCallback_t callback);

#ifdef __cplusplus
}
#endif

#endif /* BSP_ADC_H */
