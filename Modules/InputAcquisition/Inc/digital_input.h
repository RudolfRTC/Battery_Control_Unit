/**
 * @file    digital_input.h
 * @brief   Digital input driver with ACS772 current sensing
 * @author  Battery Control Unit Development Team
 * @date    2026-01-10
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
/* CONSTANTS                                                                  */
/*============================================================================*/

/** @brief Number of digital inputs */
#define DI_MAX_INPUTS        (20U)

/*============================================================================*/
/* TYPE DEFINITIONS                                                           */
/*============================================================================*/

/**
 * @brief Digital input status
 */
typedef enum {
    DI_STATUS_OK             = 0x00U,  /**< Input OK */
    DI_STATUS_OPEN_CIRCUIT   = 0x01U,  /**< Open circuit detected */
    DI_STATUS_SHORT_CIRCUIT  = 0x02U,  /**< Short circuit detected */
    DI_STATUS_OVERCURRENT    = 0x03U   /**< Overcurrent detected */
} DI_Status_t;

/**
 * @brief Digital input configuration
 */
typedef struct {
    bool        enabled;                   /**< Input enable flag */
    bool        activeLow;                 /**< Active low logic */
    bool        pullupEnabled;             /**< Internal pullup enable */
    bool        currentMonitoringEnabled;  /**< Enable current monitoring */
    Current_mA_t currentLimit_mA;          /**< Current limit (milliamperes) */
} DI_InputConfig_t;

/**
 * @brief Digital input state
 */
typedef struct {
    bool         currentState;       /**< Current debounced state */
    bool         rawState;           /**< Raw GPIO state */
    bool         edgeDetected;       /**< Edge detected this cycle */
    bool         risingEdge;         /**< Rising edge detected */
    bool         fallingEdge;        /**< Falling edge detected */
    Current_mA_t current_mA;         /**< Measured current (mA) */
    DI_Status_t  status;             /**< Diagnostic status */
    uint32_t     lastChangeTime_ms;  /**< Last state change timestamp */
    uint32_t     changeCount;        /**< Total state change count */
} DI_InputState_t;

/**
 * @brief State change callback function type
 * @param[in] inputId Input index (0-19)
 * @param[in] newState New input state
 */
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
 * @brief De-initialize digital input module
 * @return STATUS_OK on success
 */
Status_t DI_DeInit(void);

/**
 * @brief Configure digital input
 * @param[in] inputId Input index (0-19)
 * @param[in] pConfig Pointer to configuration structure
 * @return STATUS_OK on success, error code otherwise
 */
Status_t DI_ConfigureInput(uint8_t inputId, const DI_InputConfig_t *pConfig);

/**
 * @brief Read digital input state
 * @param[in]  inputId Input index (0-19)
 * @param[out] pState  Pointer to store input state
 * @return STATUS_OK on success, error code otherwise
 */
Status_t DI_ReadInput(uint8_t inputId, bool *pState);

/**
 * @brief Get full input state (including diagnostics)
 * @param[in]  inputId Input index (0-19)
 * @param[out] pState  Pointer to store full input state
 * @return STATUS_OK on success, error code otherwise
 */
Status_t DI_GetInputState(uint8_t inputId, DI_InputState_t *pState);

/**
 * @brief Read input current (from ACS772 sensor)
 * @param[in]  inputId     Input index (0-19)
 * @param[out] pCurrent_mA Pointer to store current in milliamperes
 * @return STATUS_OK on success, error code otherwise
 */
Status_t DI_ReadInputCurrent(uint8_t inputId, Current_mA_t *pCurrent_mA);

/**
 * @brief Get edge detection status
 * @param[in]  inputId      Input index (0-19)
 * @param[out] pRisingEdge  Pointer to store rising edge flag
 * @param[out] pFallingEdge Pointer to store falling edge flag
 * @return STATUS_OK on success, error code otherwise
 */
Status_t DI_GetEdgeDetection(uint8_t inputId, bool *pRisingEdge, bool *pFallingEdge);

/**
 * @brief Register state change callback
 * @param[in] inputId  Input index (0-19)
 * @param[in] callback Callback function pointer
 * @return STATUS_OK on success
 */
Status_t DI_RegisterCallback(uint8_t inputId, DI_StateChangeCallback_t callback);

/**
 * @brief Update all digital inputs (periodic task)
 * @return STATUS_OK on success
 */
Status_t DI_Update(void);

/**
 * @brief Reset input statistics
 * @param[in] inputId Input index (0-19)
 * @return STATUS_OK on success
 */
Status_t DI_ResetStatistics(uint8_t inputId);

#ifdef __cplusplus
}
#endif

#endif /* DIGITAL_INPUT_H */
