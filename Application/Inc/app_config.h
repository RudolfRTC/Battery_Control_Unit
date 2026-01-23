/**
 * @file    app_config.h
 * @brief   Application-wide configuration parameters for BCU firmware
 * @author  Battery Control Unit Development Team
 * @date    2026-01-09
 * @version 1.0.0
 *
 * @note    MISRA C:2012 compliant
 * @note    ISO 26262 ASIL-B safety requirements
 *
 * @copyright Copyright (c) 2026
 */

#ifndef APP_CONFIG_H
#define APP_CONFIG_H

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
/* FIRMWARE VERSION                                                           */
/*============================================================================*/
#define FIRMWARE_VERSION_MAJOR    (1U)
#define FIRMWARE_VERSION_MINOR    (0U)
#define FIRMWARE_VERSION_PATCH    (0U)
#define FIRMWARE_VERSION_STRING   "1.0.0"

/*============================================================================*/
/* MICROCONTROLLER CONFIGURATION                                              */
/*============================================================================*/
#define MCU_CORE_CLOCK_HZ         (100000000UL)  /**< 100 MHz */
#define MCU_FLASH_SIZE_KB         (1536U)        /**< 1.5 MB */
#define MCU_RAM_SIZE_KB           (320U)         /**< 320 KB */
#define MCU_MAX_TEMP_CELSIUS      (85)           /**< Max operating temp */
#define MCU_CRITICAL_TEMP_CELSIUS (105)          /**< Critical shutdown temp */

/*============================================================================*/
/* POWER SUPPLY CONFIGURATION                                                 */
/*============================================================================*/
/** @brief Input voltage specifications (millivolts) */
#define POWER_INPUT_VOLTAGE_NOM_MV    (12000U)    /**< 12V nominal */
#define POWER_INPUT_VOLTAGE_MIN_MV    (9000U)     /**< 9V minimum */
#define POWER_INPUT_VOLTAGE_MAX_MV    (16000U)    /**< 16V maximum */

/** @brief 5V rail specifications (millivolts) */
#define POWER_5V_RAIL_NOM_MV          (5000U)     /**< 5V nominal */
#define POWER_5V_RAIL_MIN_MV          (4750U)     /**< 4.75V minimum */
#define POWER_5V_RAIL_MAX_MV          (5250U)     /**< 5.25V maximum */

/** @brief 3.3V digital rail specifications (millivolts) */
#define POWER_3V3_DIGITAL_NOM_MV      (3300U)     /**< 3.3V nominal */
#define POWER_3V3_DIGITAL_MIN_MV      (3135U)     /**< 3.135V minimum (5%) */
#define POWER_3V3_DIGITAL_MAX_MV      (3465U)     /**< 3.465V maximum (5%) */

/** @brief 3.3V analog rail specifications (millivolts) */
#define POWER_3V3_ANALOG_NOM_MV       (3300U)     /**< 3.3V nominal */
#define POWER_3V3_ANALOG_MIN_MV       (3267U)     /**< 3.267V minimum (1%) */
#define POWER_3V3_ANALOG_MAX_MV       (3333U)     /**< 3.333V maximum (1%) */

/** @brief Power sequencing delays (milliseconds) */
#define POWER_STARTUP_DELAY_MS        (100U)      /**< Initial delay */
#define POWER_5V_SETTLE_TIME_MS       (50U)       /**< 5V rail settle time */
#define POWER_3V3_SETTLE_TIME_MS      (20U)       /**< 3.3V rail settle time */
#define POWER_SHUTDOWN_DELAY_MS       (10U)       /**< Shutdown delay */

/*============================================================================*/
/* TIMING CONFIGURATION                                                       */
/*============================================================================*/
/** @brief Task execution periods (milliseconds) */
#define TASK_PERIOD_SAFETY_MS         (1U)        /**< Safety monitor: 1ms */
#define TASK_PERIOD_FAST_MS           (10U)       /**< Fast tasks: 10ms */
#define TASK_PERIOD_MEDIUM_MS         (50U)       /**< Medium tasks: 50ms */
#define TASK_PERIOD_SLOW_MS           (100U)      /**< Slow tasks: 100ms */

/** @brief Watchdog configuration (milliseconds) */
#define WATCHDOG_TIMEOUT_MS           (500U)      /**< 500ms timeout */
#define WATCHDOG_REFRESH_PERIOD_MS    (100U)      /**< Refresh every 100ms */

/** @brief Timeout values (milliseconds) */
#define TIMEOUT_CAN_TX_MS             (100U)      /**< CAN transmit timeout */
#define TIMEOUT_I2C_MS                (50U)       /**< I2C operation timeout */
#define TIMEOUT_SPI_MS                (20U)       /**< SPI operation timeout */
#define TIMEOUT_ADC_MS                (10U)       /**< ADC conversion timeout */

/*============================================================================*/
/* CAN COMMUNICATION CONFIGURATION                                            */
/*============================================================================*/
/** @brief CAN bus configuration */
#define CAN_BITRATE_KBPS              (500U)      /**< 500 kbit/s */
#define CAN1_BAUDRATE                 (500000U)   /**< CAN1 baudrate 500kbps */
#define CAN2_BAUDRATE                 (500000U)   /**< CAN2 baudrate 500kbps */
#define CAN_TX_MAILBOX_COUNT          (3U)        /**< 3 TX mailboxes */
#define CAN_RX_FIFO_SIZE              (3U)        /**< 3 RX FIFO depth */
#define CAN_FILTER_COUNT              (14U)       /**< 14 filter banks */

/** @brief CAN message periods (milliseconds) */
#define CAN_MSG_PERIOD_STATUS_MS      (100U)      /**< Status message */
#define CAN_MSG_PERIOD_SENSORS_MS     (50U)       /**< Sensor data */
#define CAN_MSG_PERIOD_OUTPUTS_MS     (100U)      /**< Output status */

/** @brief CAN identifiers (11-bit standard) */
#ifndef CAN_ID_BCU_STATUS
#define CAN_ID_BCU_STATUS             (0x100U)    /**< BCU status */
#endif
#ifndef CAN_ID_BCU_SENSORS
#define CAN_ID_BCU_SENSORS            (0x101U)    /**< Sensor data */
#endif
#ifndef CAN_ID_BCU_OUTPUTS
#define CAN_ID_BCU_OUTPUTS            (0x102U)    /**< Output status */
#endif
#ifndef CAN_ID_BCU_FAULTS
#define CAN_ID_BCU_FAULTS             (0x103U)    /**< Fault messages */
#endif
#ifndef CAN_ID_BCU_CONFIG_REQ
#define CAN_ID_BCU_CONFIG_REQ         (0x7E0U)    /**< Config request */
#endif
#ifndef CAN_ID_BCU_CONFIG_RESP
#define CAN_ID_BCU_CONFIG_RESP        (0x7E8U)    /**< Config response */
#endif

/*============================================================================*/
/* INPUT CONFIGURATION                                                        */
/*============================================================================*/
/** @brief Digital input configuration */
#define DIGITAL_INPUT_COUNT           (20U)       /**< 20 digital inputs */
#define DIGITAL_INPUT_SAMPLE_RATE_HZ  (1000U)     /**< 1 kHz sampling */
#define DIGITAL_INPUT_DEBOUNCE_MS     (10U)       /**< 10ms debounce */

/** @brief ACS772 current sensor configuration */
#define ACS772_SENSITIVITY_MV_PER_A   (40U)       /**< 40 mV/A */
#define ACS772_ZERO_CURRENT_MV        (1650U)     /**< VCC/2 at 0A */
#define ACS772_MAX_CURRENT_A          (150)       /**< ±150A range */

/** @brief LEM current sensor configuration */
#define LEM_SENSOR_COUNT              (10U)       /**< 10 LEM sensors */
#define LEM_SAMPLE_RATE_HZ            (1000U)     /**< 1 kHz sampling */
#define LEM_REFERENCE_VOLTAGE_MV      (2500U)     /**< 2.5V from REF1933 */
#define LEM_SENSITIVITY_MV_PER_A      (25U)       /**< 25 mV/A (typical) */

/** @brief Analog input configuration */
#define ANALOG_INPUT_COUNT            (16U)       /**< Total analog channels */
#define ADC_RESOLUTION_BITS           (12U)       /**< 12-bit ADC */
#define ADC_VREF_MV                   (3300U)     /**< 3.3V reference */
#define ADC_OVERSAMPLING_RATIO        (16U)       /**< 16x oversampling */

/*============================================================================*/
/* OUTPUT CONFIGURATION                                                       */
/*============================================================================*/
/** @brief BTT6200 output driver configuration */
#define OUTPUT_CHANNEL_COUNT          (20U)       /**< 20 output channels */
#define BTT6200_IC_COUNT              (5U)        /**< 5 BTT6200 ICs */
#define BTT6200_CHANNELS_PER_IC       (4U)        /**< 4 channels per IC */
#define BTT6200_MAX_CURRENT_MA        (2000U)     /**< 2A per channel */

/** @brief Output PWM configuration */
#define OUTPUT_PWM_FREQUENCY_HZ       (1000U)     /**< 1 kHz PWM */
#define OUTPUT_PWM_RESOLUTION_BITS    (10U)       /**< 10-bit resolution */

/*============================================================================*/
/* DATA STORAGE CONFIGURATION                                                 */
/*============================================================================*/
/** @brief FRAM memory layout */
#define FRAM_I2C_ADDRESS              (0xA0U)     /**< I2C address */
#define FRAM_SIZE_BYTES               (32768U)    /**< 256 Kbit = 32 KB */

/** @brief Memory address ranges */
#define FRAM_ADDR_CONFIG_START        (0x0000U)   /**< Config: 0x0000-0x00FF */
#define FRAM_ADDR_CONFIG_SIZE         (256U)
#define FRAM_ADDR_CAL_INPUT_START     (0x0100U)   /**< Input cal: 0x0100-0x01FF */
#define FRAM_ADDR_CAL_INPUT_SIZE      (256U)
#define FRAM_ADDR_CAL_OUTPUT_START    (0x0200U)   /**< Output cal: 0x0200-0x02FF */
#define FRAM_ADDR_CAL_OUTPUT_SIZE     (256U)
#define FRAM_ADDR_FAULT_LOG_START     (0x0300U)   /**< Fault log: 0x0300-0x0FFF */
#define FRAM_ADDR_FAULT_LOG_SIZE      (3328U)
#define FRAM_ADDR_USER_DATA_START     (0x1000U)   /**< User data: 0x1000-0x7FFF */
#define FRAM_ADDR_USER_DATA_SIZE      (28672U)

/** @brief Fault log configuration */
#define FAULT_LOG_MAX_ENTRIES         (100U)      /**< Max 100 fault entries */

/*============================================================================*/
/* DIAGNOSTICS CONFIGURATION                                                  */
/*============================================================================*/
/** @brief Temperature sensor configuration (TMP1075) */
#define TEMP_SENSOR_I2C_ADDRESS       (0x48U)     /**< I2C address */
#define TEMP_SENSOR_RESOLUTION        (0.0625)    /**< 0.0625°C per LSB */
#define TEMP_WARNING_THRESHOLD_C      (75)        /**< Warning at 75°C */
#define TEMP_CRITICAL_THRESHOLD_C     (85)        /**< Critical at 85°C */
#define TEMP_SHUTDOWN_THRESHOLD_C     (95)        /**< Shutdown at 95°C */

/** @brief Fault counter thresholds */
#define FAULT_COUNTER_MAX             (10U)       /**< Max transient faults */
#define FAULT_COUNTER_RESET_TIME_MS   (60000U)    /**< Reset after 60s */

/*============================================================================*/
/* SAFETY CONFIGURATION (ISO 26262)                                           */
/*============================================================================*/
/** @brief Safety monitoring periods */
#define SAFETY_MONITOR_PERIOD_MS      (1U)        /**< 1ms safety check */
#define SAFETY_PLAUSIBILITY_CHECK_MS  (10U)       /**< Plausibility every 10ms */

/** @brief Safety thresholds */
#define SAFETY_MAX_LOOP_TIME_MS       (20U)       /**< Max main loop time */
#define SAFETY_STACK_MIN_FREE_BYTES   (512U)      /**< Min free stack */

/** @brief Redundancy configuration */
#define SAFETY_ENABLE_DUAL_CHANNEL    (1U)        /**< Enable dual-channel checks */
#define SAFETY_ENABLE_WATCHDOG        (1U)        /**< Enable watchdog */
#define SAFETY_ENABLE_CRC_CHECK       (1U)        /**< Enable CRC checks */

/*============================================================================*/
/* DEBUG CONFIGURATION                                                        */
/*============================================================================*/
/** @brief Debug UART configuration */
#define DEBUG_UART_BAUDRATE           (115200U)   /**< 115200 baud */
#define DEBUG_UART_ENABLE             (1U)        /**< Enable debug output */

/** @brief Log level configuration */
#define LOG_LEVEL_ERROR               (0U)        /**< Errors only */
#define LOG_LEVEL_WARNING             (1U)        /**< Warnings and errors */
#define LOG_LEVEL_INFO                (2U)        /**< Info, warnings, errors */
#define LOG_LEVEL_DEBUG               (3U)        /**< All messages */

#define LOG_LEVEL                     LOG_LEVEL_INFO  /**< Current log level */

/*============================================================================*/
/* FEATURE ENABLES                                                            */
/*============================================================================*/
#define FEATURE_CAN_BUS_1             (1U)        /**< Enable CAN bus 1 */
#define FEATURE_CAN_BUS_2             (1U)        /**< Enable CAN bus 2 */
#define FEATURE_ISOLATED_SPI          (1U)        /**< Enable isolated SPI */
#define FEATURE_FRAM_STORAGE          (1U)        /**< Enable FRAM storage */
#define FEATURE_TEMPERATURE_MONITOR   (1U)        /**< Enable temp monitoring */
#define FEATURE_OUTPUT_PWM            (1U)        /**< Enable PWM outputs */
#define FEATURE_FAULT_LOGGING         (1U)        /**< Enable fault logging */
#define FEATURE_CALIBRATION           (1U)        /**< Enable calibration */

/*============================================================================*/
/* FUNCTION PROTOTYPES                                                        */
/*============================================================================*/

/**
 * @brief Get firmware version
 * @param[out] pVersion Pointer to store version information
 * @return STATUS_OK on success, STATUS_ERROR_PARAM if pVersion is NULL
 */
Status_t AppConfig_GetFirmwareVersion(Version_t *pVersion);

/**
 * @brief Get firmware version as string
 * @return Pointer to firmware version string
 */
const char* AppConfig_GetVersionString(void);

/**
 * @brief Validate configuration parameters at runtime
 * @return STATUS_OK if all configurations are valid
 */
Status_t AppConfig_Validate(void);

/**
 * @brief Check if a feature is enabled
 * @param[in] featureMask Feature bit mask
 * @return true if enabled, false otherwise
 */
bool AppConfig_IsFeatureEnabled(uint32_t featureMask);

#ifdef __cplusplus
}
#endif

#endif /* APP_CONFIG_H */
