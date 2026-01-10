/**
 * @file    ltc6811.h
 * @brief   LTC6811 Battery Stack Monitor driver
 * @author  Battery Control Unit Development Team
 * @date    2026-01-10
 * @version 1.0.0
 *
 * @note    MISRA C:2012 compliant
 * @note    Supports LTC6811-1 and LTC6811-2 battery monitoring ICs
 * @note    Communication via ISO-SPI (daisy-chain capable)
 *
 * @details LTC6811 Features:
 *          - Monitors up to 12 series-connected battery cells
 *          - Cell voltage measurement accuracy: ±1.2mV
 *          - 16-bit ADC resolution
 *          - 9 GPIO pins for temperature sensing
 *          - Passive cell balancing (up to 100mA per cell)
 *          - ISO-SPI interface (1 Mbps)
 *          - Daisy-chain up to 255 devices
 *
 * @copyright Copyright (c) 2026
 */

#ifndef LTC6811_H
#define LTC6811_H

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

/** @brief Maximum number of cells per LTC6811 */
#define LTC6811_MAX_CELLS         (12U)

/** @brief Maximum number of GPIO pins */
#define LTC6811_MAX_GPIO          (9U)

/** @brief Maximum number of LTC6811 devices in daisy-chain */
#define LTC6811_MAX_DEVICES       (8U)

/** @brief LTC6811 command length */
#define LTC6811_CMD_LEN           (4U)

/** @brief LTC6811 data register size */
#define LTC6811_REG_SIZE          (6U)

/** @brief Wake-up pulse time (microseconds) */
#define LTC6811_TWAKE_US          (500U)

/** @brief Ready time after wake-up (microseconds) */
#define LTC6811_TREADY_US         (10U)

/** @brief Voltage measurement ranges */
#define LTC6811_CELL_VOLTAGE_MIN_MV    (0U)
#define LTC6811_CELL_VOLTAGE_MAX_MV    (5000U)

/** @brief PEC15 (Packet Error Code) constants */
#define LTC6811_PEC15_SEED             (16U)    /**< Initial PEC15 value */
#define LTC6811_PEC15_MULTIPLIER       (2U)     /**< Final PEC15 multiplier */

/*============================================================================*/
/* TYPE DEFINITIONS                                                           */
/*============================================================================*/

/**
 * @brief LTC6811 ADC mode (conversion time vs. noise)
 */
typedef enum {
    LTC6811_ADC_MODE_27KHZ_14KHZ = 0x00U,  /**< Fast mode: 27kHz/14kHz */
    LTC6811_ADC_MODE_7KHZ_3KHZ   = 0x01U,  /**< Normal mode: 7kHz/3kHz */
    LTC6811_ADC_MODE_26HZ_2KHZ   = 0x02U,  /**< Filtered mode: 26Hz/2kHz */
    LTC6811_ADC_MODE_2KHZ_1KHZ   = 0x03U   /**< Slow mode: 2kHz/1kHz (lowest noise) */
} LTC6811_ADCMode_t;

/**
 * @brief LTC6811 discharge permission
 */
typedef enum {
    LTC6811_DISCHARGE_NOT_PERMITTED = 0x00U,  /**< Discharge not permitted */
    LTC6811_DISCHARGE_PERMITTED     = 0x01U   /**< Discharge permitted */
} LTC6811_DischargePermission_t;

/**
 * @brief LTC6811 self-test mode
 */
typedef enum {
    LTC6811_SELF_TEST_MODE_1 = 0x01U,  /**< Self-test mode 1 */
    LTC6811_SELF_TEST_MODE_2 = 0x02U   /**< Self-test mode 2 */
} LTC6811_SelfTestMode_t;

/**
 * @brief LTC6811 GPIO selection for ADC
 */
typedef enum {
    LTC6811_GPIO_ALL     = 0x00U,  /**< All GPIO pins */
    LTC6811_GPIO_1       = 0x01U,  /**< GPIO1 only */
    LTC6811_GPIO_2       = 0x02U,  /**< GPIO2 only */
    LTC6811_GPIO_3       = 0x03U,  /**< GPIO3 only */
    LTC6811_GPIO_4       = 0x04U,  /**< GPIO4 only */
    LTC6811_GPIO_5       = 0x05U,  /**< GPIO5 only */
    LTC6811_GPIO_REF     = 0x06U   /**< Second reference */
} LTC6811_GPIOSelection_t;

/**
 * @brief LTC6811 cell selection for ADC
 */
typedef enum {
    LTC6811_CELL_ALL     = 0x00U,  /**< All cells */
    LTC6811_CELL_1_7     = 0x01U,  /**< Cells 1 and 7 */
    LTC6811_CELL_2_8     = 0x02U,  /**< Cells 2 and 8 */
    LTC6811_CELL_3_9     = 0x03U,  /**< Cells 3 and 9 */
    LTC6811_CELL_4_10    = 0x04U,  /**< Cells 4 and 10 */
    LTC6811_CELL_5_11    = 0x05U,  /**< Cells 5 and 11 */
    LTC6811_CELL_6_12    = 0x06U   /**< Cells 6 and 12 */
} LTC6811_CellSelection_t;

/**
 * @brief LTC6811 configuration structure
 */
typedef struct {
    LTC6811_ADCMode_t adcMode;                          /**< ADC conversion mode */
    bool enableGPIO[LTC6811_MAX_GPIO];                  /**< GPIO enable flags */
    bool enableCellBalancing[LTC6811_MAX_CELLS];        /**< Cell balancing enable */
    LTC6811_DischargePermission_t dischargePermission;  /**< Discharge permission */
    uint16_t dischargeTimeout_min;                      /**< Discharge timeout (minutes, 0-1440) */
    bool enableReferenceOn;                             /**< Reference always on */
    uint16_t underVoltageThreshold_mV;                  /**< Under-voltage threshold */
    uint16_t overVoltageThreshold_mV;                   /**< Over-voltage threshold */
} LTC6811_Config_t;

/**
 * @brief LTC6811 cell voltages data
 */
typedef struct {
    uint16_t cellVoltage_mV[LTC6811_MAX_CELLS];  /**< Cell voltages in mV */
    bool     cellValid[LTC6811_MAX_CELLS];       /**< Cell validity flags */
    uint8_t  numCells;                           /**< Number of valid cells */
} LTC6811_CellVoltages_t;

/**
 * @brief LTC6811 GPIO voltages data (for temperature sensing)
 */
typedef struct {
    uint16_t gpioVoltage_mV[LTC6811_MAX_GPIO];  /**< GPIO voltages in mV */
    bool     gpioValid[LTC6811_MAX_GPIO];       /**< GPIO validity flags */
    uint16_t referenceVoltage_mV;               /**< Reference voltage */
} LTC6811_GPIOVoltages_t;

/**
 * @brief LTC6811 status data
 */
typedef struct {
    uint16_t sumOfCells_mV;       /**< Sum of all cells voltage */
    uint16_t internalDieTemp_C;   /**< Internal die temperature */
    uint16_t analogSupply_mV;     /**< Analog supply voltage */
    uint16_t digitalSupply_mV;    /**< Digital supply voltage */
    uint8_t  revisionCode;        /**< Device revision code */
    bool     thermalShutdown;     /**< Thermal shutdown flag */
    bool     muxFail;             /**< Multiplexer fail flag */
} LTC6811_Status_t;

/**
 * @brief LTC6811 device state
 */
typedef struct {
    LTC6811_Config_t        config;        /**< Configuration */
    LTC6811_CellVoltages_t  cellVoltages;  /**< Cell voltages */
    LTC6811_GPIOVoltages_t  gpioVoltages;  /**< GPIO voltages */
    LTC6811_Status_t        status;        /**< Status registers */
    uint32_t                lastUpdate_ms; /**< Last update timestamp */
    bool                    initialized;   /**< Initialization flag */
} LTC6811_Device_t;

/*============================================================================*/
/* FUNCTION PROTOTYPES                                                        */
/*============================================================================*/

/**
 * @brief Initialize LTC6811 driver
 * @param[in] numDevices Number of LTC6811 devices in daisy-chain (1-8)
 * @return STATUS_OK on success, error code otherwise
 */
Status_t LTC6811_Init(uint8_t numDevices);

/**
 * @brief De-initialize LTC6811 driver
 * @return STATUS_OK on success
 */
Status_t LTC6811_DeInit(void);

/**
 * @brief Wake up LTC6811 from sleep mode
 * @return STATUS_OK on success, error code otherwise
 */
Status_t LTC6811_WakeUp(void);

/**
 * @brief Write configuration to LTC6811 device
 * @param[in] deviceIndex Device index (0 to numDevices-1)
 * @param[in] pConfig     Pointer to configuration structure
 * @return STATUS_OK on success, error code otherwise
 */
Status_t LTC6811_WriteConfig(uint8_t deviceIndex, const LTC6811_Config_t *pConfig);

/**
 * @brief Read configuration from LTC6811 device
 * @param[in]  deviceIndex Device index (0 to numDevices-1)
 * @param[out] pConfig     Pointer to store configuration
 * @return STATUS_OK on success, error code otherwise
 */
Status_t LTC6811_ReadConfig(uint8_t deviceIndex, LTC6811_Config_t *pConfig);

/**
 * @brief Start cell voltage conversion
 * @param[in] adcMode       ADC conversion mode
 * @param[in] cellSelection Cell selection
 * @return STATUS_OK on success, error code otherwise
 */
Status_t LTC6811_StartCellVoltageConversion(LTC6811_ADCMode_t adcMode,
                                            LTC6811_CellSelection_t cellSelection);

/**
 * @brief Read cell voltages from LTC6811 device
 * @param[in]  deviceIndex Device index (0 to numDevices-1)
 * @param[out] pVoltages   Pointer to store cell voltages
 * @return STATUS_OK on success, error code otherwise
 */
Status_t LTC6811_ReadCellVoltages(uint8_t deviceIndex, LTC6811_CellVoltages_t *pVoltages);

/**
 * @brief Start GPIO/aux voltage conversion
 * @param[in] adcMode        ADC conversion mode
 * @param[in] gpioSelection  GPIO selection
 * @return STATUS_OK on success, error code otherwise
 */
Status_t LTC6811_StartGPIOConversion(LTC6811_ADCMode_t adcMode,
                                     LTC6811_GPIOSelection_t gpioSelection);

/**
 * @brief Read GPIO voltages from LTC6811 device
 * @param[in]  deviceIndex Device index (0 to numDevices-1)
 * @param[out] pVoltages   Pointer to store GPIO voltages
 * @return STATUS_OK on success, error code otherwise
 */
Status_t LTC6811_ReadGPIOVoltages(uint8_t deviceIndex, LTC6811_GPIOVoltages_t *pVoltages);

/**
 * @brief Start status group conversion
 * @param[in] adcMode ADC conversion mode
 * @return STATUS_OK on success, error code otherwise
 */
Status_t LTC6811_StartStatusConversion(LTC6811_ADCMode_t adcMode);

/**
 * @brief Read status registers from LTC6811 device
 * @param[in]  deviceIndex Device index (0 to numDevices-1)
 * @param[out] pStatus     Pointer to store status data
 * @return STATUS_OK on success, error code otherwise
 */
Status_t LTC6811_ReadStatus(uint8_t deviceIndex, LTC6811_Status_t *pStatus);

/**
 * @brief Enable/disable cell balancing
 * @param[in] deviceIndex  Device index (0 to numDevices-1)
 * @param[in] cellMask     Bitmask of cells to balance (bit 0 = cell 1, etc.)
 * @return STATUS_OK on success, error code otherwise
 */
Status_t LTC6811_SetCellBalancing(uint8_t deviceIndex, uint16_t cellMask);

/**
 * @brief Clear all cell voltages
 * @return STATUS_OK on success, error code otherwise
 */
Status_t LTC6811_ClearCellVoltages(void);

/**
 * @brief Clear all GPIO voltages
 * @return STATUS_OK on success, error code otherwise
 */
Status_t LTC6811_ClearGPIOVoltages(void);

/**
 * @brief Clear all status registers
 * @return STATUS_OK on success, error code otherwise
 */
Status_t LTC6811_ClearStatus(void);

/**
 * @brief Run self-test on LTC6811
 * @param[in] testMode Self-test mode
 * @return STATUS_OK on success, error code otherwise
 */
Status_t LTC6811_RunSelfTest(LTC6811_SelfTestMode_t testMode);

/**
 * @brief Read ADC overlap flag (diagnostics)
 * @param[in]  deviceIndex Device index (0 to numDevices-1)
 * @param[out] pOverlap    Pointer to store overlap flag
 * @return STATUS_OK on success, error code otherwise
 */
Status_t LTC6811_ReadADCOverlap(uint8_t deviceIndex, bool *pOverlap);

/**
 * @brief Mute/unmute discharge
 * @param[in] mute True to mute, false to unmute
 * @return STATUS_OK on success, error code otherwise
 */
Status_t LTC6811_MuteDischarge(bool mute);

/**
 * @brief Poll ADC conversion status
 * @param[out] pReady Pointer to store ready flag (true when conversion done)
 * @return STATUS_OK on success, error code otherwise
 */
Status_t LTC6811_PollADC(bool *pReady);

/**
 * @brief Get device state
 * @param[in]  deviceIndex Device index (0 to numDevices-1)
 * @param[out] pDevice     Pointer to store device state
 * @return STATUS_OK on success, error code otherwise
 */
Status_t LTC6811_GetDeviceState(uint8_t deviceIndex, LTC6811_Device_t *pDevice);

/**
 * @brief Update all measurements (convenience function)
 * @param[in] deviceIndex Device index (0 to numDevices-1)
 * @return STATUS_OK on success, error code otherwise
 */
Status_t LTC6811_UpdateAllMeasurements(uint8_t deviceIndex);

#ifdef __cplusplus
}
#endif

#endif /* LTC6811_H */
