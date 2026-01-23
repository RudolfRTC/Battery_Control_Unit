/**
 * @file    app_config_mgmt.h
 * @brief   Configuration management header
 * @author  Battery Control Unit Development Team
 * @date    2026-01-09
 * @version 1.0.0
 *
 * @note    MISRA C:2012 compliant
 * @note    FRAM-based persistent configuration storage
 *
 * @copyright Copyright (c) 2026
 */

#ifndef APP_CONFIG_MGMT_H
#define APP_CONFIG_MGMT_H

#ifdef __cplusplus
extern "C" {
#endif

/*============================================================================*/
/* INCLUDES                                                                   */
/*============================================================================*/
#include "app_types.h"
#include "lem_sensor.h"
#include "btt6200.h"
#include "digital_input.h"

/*============================================================================*/
/* CONSTANTS                                                                  */
/*============================================================================*/

/** @brief Configuration version */
#define CONFIG_VERSION          (0x0100U)  /* v1.0 */

/** @brief Configuration magic number */
#define CONFIG_MAGIC            (0xBC2026U)

/*============================================================================*/
/* TYPES                                                                      */
/*============================================================================*/

/** @brief System configuration structure */
typedef struct {
    /* Header */
    uint32_t magic;
    uint16_t version;
    uint16_t length;
    uint32_t crc32;

    /* LEM sensor calibration (10 channels) */
    LEM_Calibration_t lem_calibration[LEM_SENSOR_COUNT];

    /* BTT6200 configuration (20 channels) */
    struct {
        bool enabled;
        uint32_t currentLimit_mA;
        bool diagnosticsEnabled;
    } btt_config[BTT6200_TOTAL_CHANNELS];

    /* Digital input configuration (20 channels) */
    DI_InputConfig_t di_config[20];

    /* Power management thresholds */
    struct {
        Voltage_mV_t minVoltage_mV;
        Voltage_mV_t maxVoltage_mV;
    } power_thresholds[4];  /* 4 power rails */

    /* Temperature limits */
    int32_t temp_low_limit_mC;
    int32_t temp_high_limit_mC;

    /* CAN configuration */
    uint32_t can1_baudrate;
    uint32_t can2_baudrate;
    bool can2_enabled;

    /* Safety configuration */
    uint32_t max_loop_time_us;
    bool watchdog_enabled;

    /* Reserved for future use */
    uint8_t reserved[128];
} SystemConfig_t;

/*============================================================================*/
/* PUBLIC FUNCTION PROTOTYPES                                                 */
/*============================================================================*/

/**
 * @brief Initialize configuration management
 * @return Status code
 */
Status_t ConfigMgmt_Init(void);

/**
 * @brief Load configuration from FRAM
 * @param[out] pConfig Pointer to configuration structure
 * @return Status code
 */
Status_t ConfigMgmt_Load(SystemConfig_t *pConfig);

/**
 * @brief Save configuration to FRAM
 * @param[in] pConfig Pointer to configuration structure
 * @return Status code
 */
Status_t ConfigMgmt_Save(const SystemConfig_t *pConfig);

/**
 * @brief Load default configuration
 * @param[out] pConfig Pointer to configuration structure
 * @return Status code
 */
Status_t ConfigMgmt_LoadDefaults(SystemConfig_t *pConfig);

/**
 * @brief Factory reset (restore defaults and save to FRAM)
 * @return Status code
 */
Status_t ConfigMgmt_FactoryReset(void);

/**
 * @brief Validate configuration
 * @param[in] pConfig Pointer to configuration structure
 * @return STATUS_OK if valid, error code otherwise
 */
Status_t ConfigMgmt_Validate(const SystemConfig_t *pConfig);

/**
 * @brief Apply configuration to system
 * @param[in] pConfig Pointer to configuration structure
 * @return Status code
 */
Status_t ConfigMgmt_Apply(const SystemConfig_t *pConfig);

/**
 * @brief Export configuration as binary blob
 * @param[in] pConfig Pointer to configuration structure
 * @param[out] pBlob Pointer to output buffer
 * @param[in] maxSize Maximum buffer size
 * @param[out] pActualSize Actual size written
 * @return Status code
 */
Status_t ConfigMgmt_Export(const SystemConfig_t *pConfig, uint8_t *pBlob,
                           uint16_t maxSize, uint16_t *pActualSize);

/**
 * @brief Import configuration from binary blob
 * @param[out] pConfig Pointer to configuration structure
 * @param[in] pBlob Pointer to input buffer
 * @param[in] size Buffer size
 * @return Status code
 */
Status_t ConfigMgmt_Import(SystemConfig_t *pConfig, const uint8_t *pBlob, uint16_t size);

#ifdef __cplusplus
}
#endif

#endif /* APP_CONFIG_MGMT_H */

/*============================================================================*/
/* END OF FILE                                                                */
/*============================================================================*/
