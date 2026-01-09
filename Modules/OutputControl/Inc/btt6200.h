/**
 * @file    btt6200.h
 * @brief   BTT6200-4ESA quad high-side switch driver
 * @author  Battery Control Unit Development Team
 * @date    2026-01-09
 * @version 1.0.0
 *
 * @note    MISRA C:2012 compliant
 * @note    ISO 26262 ASIL-B safety requirements
 * @note    Controls 5× BTT6200 ICs (20 total outputs)
 *
 * Hardware interface per IC:
 * - DEN: Diagnostic enable (digital output)
 * - DSEL0, DSEL1: Diagnostic channel select (digital outputs)
 * - IN0-IN3: Input control (digital outputs or PWM)
 * - IS: Current sense output (analog input via ADC)
 *
 * @copyright Copyright (c) 2026
 */

#ifndef BTT6200_H
#define BTT6200_H

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

/** @brief BTT6200 device configuration */
#define BTT6200_IC_COUNT          (5U)   /**< Number of BTT6200 ICs */
#define BTT6200_CHANNELS_PER_IC   (4U)   /**< Channels per IC */
#define BTT6200_TOTAL_CHANNELS    (20U)  /**< Total output channels */

/** @brief Output current limits */
#define BTT6200_MAX_CURRENT_MA    (2000U)   /**< Max 2A per channel */
#define BTT6200_WARNING_CURRENT_MA (1800U) /**< Warning at 1.8A */

/** @brief Diagnostic states */
#define BTT6200_DIAG_CH0          (0U)  /**< Diagnose channel 0 */
#define BTT6200_DIAG_CH1          (1U)  /**< Diagnose channel 1 */
#define BTT6200_DIAG_CH2          (2U)  /**< Diagnose channel 2 */
#define BTT6200_DIAG_CH3          (3U)  /**< Diagnose channel 3 */

/** @brief PWM configuration */
#define BTT6200_PWM_FREQUENCY_HZ  (1000U)  /**< 1 kHz PWM */
#define BTT6200_PWM_MAX_DUTY      (1000U)  /**< 100.0% = 1000 */

/*============================================================================*/
/* TYPE DEFINITIONS                                                           */
/*============================================================================*/

/**
 * @brief Output channel state
 */
typedef enum {
    BTT6200_STATE_OFF         = 0x00U,  /**< Output off */
    BTT6200_STATE_ON          = 0x01U,  /**< Output on (full) */
    BTT6200_STATE_PWM         = 0x02U,  /**< Output PWM mode */
    BTT6200_STATE_FAULT       = 0x03U   /**< Output in fault state */
} BTT6200_ChannelState_t;

/**
 * @brief Diagnostic fault types
 */
typedef enum {
    BTT6200_FAULT_NONE            = 0x00U,  /**< No fault */
    BTT6200_FAULT_OPEN_LOAD       = 0x01U,  /**< Open load detected */
    BTT6200_FAULT_SHORT_CIRCUIT   = 0x02U,  /**< Short circuit detected */
    BTT6200_FAULT_OVERCURRENT     = 0x03U,  /**< Overcurrent detected */
    BTT6200_FAULT_OVERTEMPERATURE = 0x04U,  /**< Overtemperature */
    BTT6200_FAULT_UNDERVOLTAGE    = 0x05U,  /**< Supply undervoltage */
    BTT6200_FAULT_COMMUNICATION   = 0x06U   /**< Communication error */
} BTT6200_FaultType_t;

/**
 * @brief Channel diagnostic information
 */
typedef struct {
    BTT6200_FaultType_t faultType;      /**< Current fault type */
    Current_mA_t        current_mA;     /**< Measured current */
    uint16_t            rawADC;         /**< Raw ADC value from IS pin */
    bool                openLoad;       /**< Open load flag */
    bool                shortCircuit;   /**< Short circuit flag */
    bool                overcurrent;    /**< Overcurrent flag */
    uint32_t            faultCount;     /**< Total fault count */
    uint32_t            lastFaultTime_ms; /**< Last fault timestamp */
} BTT6200_ChannelDiag_t;

/**
 * @brief Channel configuration
 */
typedef struct {
    bool     enabled;              /**< Channel enabled */
    uint16_t pwmDutyCycle;         /**< PWM duty cycle (0-1000 = 0-100%) */
    uint16_t currentLimit_mA;      /**< Current limit */
    bool     diagnosticsEnabled;   /**< Enable diagnostics */
    bool     autoRetry;            /**< Auto retry after fault */
} BTT6200_ChannelConfig_t;

/**
 * @brief IC status structure (per BTT6200 device)
 */
typedef struct {
    bool                    initialized;  /**< IC initialized */
    BTT6200_ChannelState_t  state[4];    /**< Channel states */
    BTT6200_ChannelDiag_t   diag[4];     /**< Channel diagnostics */
    uint32_t                faultCount;  /**< IC fault count */
} BTT6200_IC_Status_t;

/**
 * @brief Fault callback function
 */
typedef void (*BTT6200_FaultCallback_t)(uint8_t channel, BTT6200_FaultType_t fault);

/*============================================================================*/
/* FUNCTION PROTOTYPES                                                        */
/*============================================================================*/

/**
 * @brief Initialize BTT6200 driver
 * @return STATUS_OK on success, error code otherwise
 * @req BCU_REQ_OUT_001 Output driver initialization
 */
Status_t BTT6200_Init(void);

/**
 * @brief De-initialize BTT6200 driver
 * @return STATUS_OK on success
 */
Status_t BTT6200_DeInit(void);

/**
 * @brief Turn on output channel (full on)
 * @param[in] channel Output channel (0-19)
 * @return STATUS_OK on success, error code otherwise
 * @req BCU_REQ_OUT_010 Output channel control
 */
Status_t BTT6200_ChannelOn(uint8_t channel);

/**
 * @brief Turn off output channel
 * @param[in] channel Output channel (0-19)
 * @return STATUS_OK on success, error code otherwise
 */
Status_t BTT6200_ChannelOff(uint8_t channel);

/**
 * @brief Set output channel PWM duty cycle
 * @param[in] channel    Output channel (0-19)
 * @param[in] dutyCycle  Duty cycle (0-1000 = 0.0-100.0%)
 * @return STATUS_OK on success, error code otherwise
 * @req BCU_REQ_OUT_020 PWM output control
 */
Status_t BTT6200_ChannelSetPWM(uint8_t channel, uint16_t dutyCycle);

/**
 * @brief Toggle output channel
 * @param[in] channel Output channel (0-19)
 * @return STATUS_OK on success, error code otherwise
 */
Status_t BTT6200_ChannelToggle(uint8_t channel);

/**
 * @brief Get output channel state
 * @param[in]  channel Output channel (0-19)
 * @param[out] pState  Pointer to store channel state
 * @return STATUS_OK on success, error code otherwise
 */
Status_t BTT6200_GetChannelState(uint8_t channel, BTT6200_ChannelState_t *pState);

/**
 * @brief Configure output channel
 * @param[in] channel Output channel (0-19)
 * @param[in] pConfig Pointer to configuration structure
 * @return STATUS_OK on success, error code otherwise
 */
Status_t BTT6200_ConfigureChannel(uint8_t channel,
                                  const BTT6200_ChannelConfig_t *pConfig);

/**
 * @brief Read channel current (via IS pin ADC)
 * @param[in]  channel     Output channel (0-19)
 * @param[out] pCurrent_mA Pointer to store current in milliamps
 * @return STATUS_OK on success, error code otherwise
 */
Status_t BTT6200_ReadChannelCurrent(uint8_t channel, Current_mA_t *pCurrent_mA);

/**
 * @brief Perform channel diagnostics
 * @param[in]  channel Output channel (0-19)
 * @param[out] pDiag   Pointer to store diagnostic information
 * @return STATUS_OK on success, error code otherwise
 * @req BCU_REQ_OUT_030 Output diagnostics
 */
Status_t BTT6200_DiagnoseChannel(uint8_t channel, BTT6200_ChannelDiag_t *pDiag);

/**
 * @brief Perform diagnostics on all channels of an IC
 * @param[in] icNumber IC number (0-4)
 * @return STATUS_OK on success, error code otherwise
 */
Status_t BTT6200_DiagnoseIC(uint8_t icNumber);

/**
 * @brief Clear channel fault
 * @param[in] channel Output channel (0-19)
 * @return STATUS_OK on success, error code otherwise
 */
Status_t BTT6200_ClearFault(uint8_t channel);

/**
 * @brief Set all channels to safe state (all off)
 * @return STATUS_OK on success
 * @req BCU_REQ_OUT_040 Safe state functionality
 */
Status_t BTT6200_SetSafeState(void);

/**
 * @brief Enable output channels (global)
 * @param[in] enable true to enable all outputs, false to disable
 * @return STATUS_OK on success
 */
Status_t BTT6200_EnableOutputs(bool enable);

/**
 * @brief Get IC status
 * @param[in]  icNumber IC number (0-4)
 * @param[out] pStatus  Pointer to store IC status
 * @return STATUS_OK on success, error code otherwise
 */
Status_t BTT6200_GetICStatus(uint8_t icNumber, BTT6200_IC_Status_t *pStatus);

/**
 * @brief Get total fault count across all channels
 * @param[out] pFaultCount Pointer to store fault count
 * @return STATUS_OK on success
 */
Status_t BTT6200_GetTotalFaultCount(uint32_t *pFaultCount);

/**
 * @brief Register fault callback
 * @param[in] callback Callback function pointer
 * @return STATUS_OK on success
 */
Status_t BTT6200_RegisterFaultCallback(BTT6200_FaultCallback_t callback);

/**
 * @brief Update BTT6200 driver (call periodically)
 * @return STATUS_OK on success
 * @note  Performs background diagnostics and fault handling
 * @note  Should be called every 10ms
 */
Status_t BTT6200_Update(void);

/**
 * @brief Convert IS pin ADC value to current
 * @param[in] adcValue Raw ADC value from IS pin
 * @return Current in milliamps
 */
Current_mA_t BTT6200_ADCToCurrent(uint16_t adcValue);

/**
 * @brief Check if channel has active fault
 * @param[in]  channel  Output channel (0-19)
 * @param[out] pHasFault true if fault active, false otherwise
 * @return STATUS_OK on success
 */
Status_t BTT6200_HasFault(uint8_t channel, bool *pHasFault);

/**
 * @brief Set current limit for channel
 * @param[in] channel     Output channel (0-19)
 * @param[in] limit_mA    Current limit in milliamps
 * @return STATUS_OK on success, error code otherwise
 */
Status_t BTT6200_SetCurrentLimit(uint8_t channel, uint16_t limit_mA);

/**
 * @brief Enable/disable auto-retry after fault
 * @param[in] channel    Output channel (0-19)
 * @param[in] autoRetry  true to enable auto-retry, false to disable
 * @return STATUS_OK on success
 */
Status_t BTT6200_SetAutoRetry(uint8_t channel, bool autoRetry);

#ifdef __cplusplus
}
#endif

#endif /* BTT6200_H */
