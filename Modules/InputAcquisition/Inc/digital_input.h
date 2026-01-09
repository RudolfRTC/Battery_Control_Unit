/**
 * @file    digital_input.h
 * @brief   Digital input driver with ACS772 current sensing
 * @author  Battery Control Unit Development Team
 * @date    2026-01-09
 * @version 1.0.0
 *
 * @note    MISRA C:2012 compliant
 * @note    20 digital inputs with debouncing
 * @note    ACS772 current sensing for diagnostics
 *
 * @copyright Copyright (c) 2026
 */

#ifndef DIGITAL_INPUT_H
#define DIGITAL_INPUT_H

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

/** @brief Maximum number of digital inputs */
#define DI_MAX_INPUTS           (20U)

/*============================================================================*/
/* TYPE DEFINITIONS                                                           */
/*============================================================================*/

/** @brief Current measurement type (milliamps) */
typedef int32_t Current_mA_t;

/** @brief Input polarity configuration */
typedef enum {
    DI_POLARITY_NORMAL = 0x00U,   /**< Normal polarity (active high) */
    DI_POLARITY_INVERTED = 0x01U  /**< Inverted polarity (active low) */
} DI_Polarity_t;

/** @brief Input configuration structure */
typedef struct {
    bool enabled;                /**< Input enable flag */
    DI_Polarity_t polarity;      /**< Input polarity */
    bool debounceEnable;         /**< Debounce enable flag */
    uint16_t debounceTime_ms;    /**< Debounce time in milliseconds */
    bool currentSenseEnable;     /**< Current sensing enable */
    Current_mA_t currentThreshold_mA;  /**< Current threshold for fault detection */
} DI_InputConfig_t;

/** @brief Input state structure */
typedef struct {
    bool currentState;           /**< Current logical state */
    bool rawState;               /**< Raw hardware state (before debounce) */
    bool risingEdgeDetected;     /**< Rising edge flag */
    bool fallingEdgeDetected;    /**< Falling edge flag */
    uint32_t stateChangeCount;   /**< Number of state changes */
    uint32_t lastChangeTime_ms;  /**< Timestamp of last state change */
    Current_mA_t current_mA;     /**< Measured current (if enabled) */
    bool openCircuitFault;       /**< Open circuit fault flag */
    bool shortCircuitFault;      /**< Short circuit fault flag */
} DI_InputState_t;

/** @brief State change callback function type */
typedef void (*DI_StateChangeCallback_t)(uint8_t inputId, bool newState);

/*============================================================================*/
/* FUNCTION PROTOTYPES                                                        */
/*============================================================================*/

/**
 * @brief Initialize digital input module
 * @return STATUS_OK on success, error code otherwise
 */
Status_t DI_Init(void);

/**
 * @brief Configure a digital input
 * @param[in] inputId Input identifier (0 to DI_MAX_INPUTS-1)
 * @param[in] pConfig Pointer to configuration structure
 * @return STATUS_OK on success, error code otherwise
 */
Status_t DI_ConfigureInput(uint8_t inputId, const DI_InputConfig_t *pConfig);

/**
 * @brief Read digital input state
 * @param[in] inputId Input identifier (0 to DI_MAX_INPUTS-1)
 * @param[out] pState Pointer to store input state (true = active, false = inactive)
 * @return STATUS_OK on success, error code otherwise
 */
Status_t DI_ReadInput(uint8_t inputId, bool *pState);

/**
 * @brief Get detailed input state information
 * @param[in] inputId Input identifier (0 to DI_MAX_INPUTS-1)
 * @param[out] pState Pointer to store input state structure
 * @return STATUS_OK on success, error code otherwise
 */
Status_t DI_GetInputState(uint8_t inputId, DI_InputState_t *pState);

/**
 * @brief Read input current measurement (if current sensing enabled)
 * @param[in] inputId Input identifier (0 to DI_MAX_INPUTS-1)
 * @param[out] pCurrent_mA Pointer to store current in milliamps
 * @return STATUS_OK on success, error code otherwise
 */
Status_t DI_ReadInputCurrent(uint8_t inputId, Current_mA_t *pCurrent_mA);

/**
 * @brief Get edge detection status
 * @param[in] inputId Input identifier (0 to DI_MAX_INPUTS-1)
 * @param[out] pRisingEdge Pointer to store rising edge flag
 * @param[out] pFallingEdge Pointer to store falling edge flag
 * @return STATUS_OK on success, error code otherwise
 * @note Edge flags are cleared after reading
 */
Status_t DI_GetEdgeDetection(uint8_t inputId, bool *pRisingEdge, bool *pFallingEdge);

/**
 * @brief Register state change callback
 * @param[in] inputId Input identifier (0 to DI_MAX_INPUTS-1)
 * @param[in] callback Callback function pointer (NULL to disable)
 * @return STATUS_OK on success, error code otherwise
 */
Status_t DI_RegisterCallback(uint8_t inputId, DI_StateChangeCallback_t callback);

/**
 * @brief Update digital input module (periodic call from main loop)
 * @return STATUS_OK on success, error code otherwise
 * @note Should be called every 1-10ms for proper debouncing
 */
Status_t DI_Update(void);

/**
 * @brief Reset input statistics
 * @param[in] inputId Input identifier (0 to DI_MAX_INPUTS-1)
 * @return STATUS_OK on success, error code otherwise
 */
Status_t DI_ResetStatistics(uint8_t inputId);

/**
 * @brief De-initialize digital input module
 * @return STATUS_OK on success, error code otherwise
 */
Status_t DI_DeInit(void);

/**
 * @brief Convert ADC value to current measurement
 * @param[in] adcValue ADC value (0-4095 for 12-bit ADC)
 * @return Current in milliamps
 */
Current_mA_t DI_ADCToCurrent(uint16_t adcValue);

#ifdef __cplusplus
}
#endif

#endif /* DIGITAL_INPUT_H */
