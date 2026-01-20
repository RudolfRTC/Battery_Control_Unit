/**
 * @file    app_errors.h
 * @brief   Error code definitions and error handling utilities
 * @author  Battery Control Unit Development Team
 * @date    2026-01-09
 * @version 1.0.0
 *
 * @note    MISRA C:2012 compliant
 * @note    ISO 26262 ASIL-B safety requirements
 *
 * @copyright Copyright (c) 2026
 */

#ifndef APP_ERRORS_H
#define APP_ERRORS_H

#ifdef __cplusplus
extern "C" {
#endif

/*============================================================================*/
/* INCLUDES                                                                   */
/*============================================================================*/
#include "app_types.h"

/*============================================================================*/
/* ERROR CODE DEFINITIONS                                                     */
/*============================================================================*/

/**
 * @brief Detailed error codes for system modules
 * @note  Organized by module (bits 15-8) and specific error (bits 7-0)
 *        Format: 0xMMEE where MM=module, EE=error
 */
typedef enum {
    /* Generic errors (0x00xx) */
    ERROR_NONE                      = 0x0000U,  /**< No error */
    ERROR_GENERIC                   = 0x0001U,  /**< Generic error */
    ERROR_NOT_IMPLEMENTED           = 0x0002U,  /**< Feature not implemented */
    ERROR_INVALID_PARAMETER         = 0x0003U,  /**< Invalid parameter */
    ERROR_NULL_POINTER              = 0x0004U,  /**< Null pointer detected */
    ERROR_TIMEOUT                   = 0x0005U,  /**< Operation timeout */
    ERROR_BUSY                      = 0x0006U,  /**< Resource busy */
    ERROR_OUT_OF_RANGE              = 0x0007U,  /**< Value out of range */

    /* Power Management errors (0x01xx) */
    ERROR_POWER_UNDERVOLTAGE        = 0x0101U,  /**< Undervoltage detected */
    ERROR_POWER_OVERVOLTAGE         = 0x0102U,  /**< Overvoltage detected */
    ERROR_POWER_OVERCURRENT         = 0x0103U,  /**< Overcurrent detected */
    ERROR_POWER_SEQUENCING          = 0x0104U,  /**< Power sequencing fault */
    ERROR_POWER_5V_FAULT            = 0x0105U,  /**< 5V rail fault */
    ERROR_POWER_3V3_DIGITAL_FAULT   = 0x0106U,  /**< 3.3V digital fault */
    ERROR_POWER_3V3_ANALOG_FAULT    = 0x0107U,  /**< 3.3V analog fault */
    ERROR_POWER_LM74900_FAULT       = 0x0108U,  /**< LM74900 fault signal */

    /* CAN Communication errors (0x02xx) */
    ERROR_CAN_BUS_OFF               = 0x0201U,  /**< CAN bus-off */
    ERROR_CAN_ERROR_PASSIVE         = 0x0202U,  /**< CAN error passive */
    ERROR_CAN_TX_FAILED             = 0x0203U,  /**< CAN transmit failed */
    ERROR_CAN_RX_OVERFLOW           = 0x0204U,  /**< CAN receive overflow */
    ERROR_CAN_FILTER_CONFIG         = 0x0205U,  /**< CAN filter config error */
    ERROR_CAN_INVALID_MESSAGE       = 0x0206U,  /**< Invalid CAN message */
    ERROR_CAN_TIMEOUT               = 0x0207U,  /**< CAN operation timeout */

    /* Digital Input errors (0x03xx) */
    ERROR_DIN_OPEN_CIRCUIT          = 0x0301U,  /**< Digital input open */
    ERROR_DIN_SHORT_CIRCUIT         = 0x0302U,  /**< Digital input short */
    ERROR_DIN_ACS772_FAULT          = 0x0303U,  /**< ACS772 sensor fault */
    ERROR_DIN_OVERCURRENT           = 0x0304U,  /**< Input overcurrent */
    ERROR_DIN_CALIBRATION           = 0x0305U,  /**< Calibration error */

    /* Analog Input errors (0x04xx) */
    ERROR_AIN_OUT_OF_RANGE          = 0x0401U,  /**< Analog value out of range */
    ERROR_AIN_LEM_FAULT             = 0x0402U,  /**< LEM sensor fault */
    ERROR_AIN_LEM_OVERCURRENT       = 0x0403U,  /**< LEM overcurrent flag */
    ERROR_AIN_OPEN_WIRE             = 0x0404U,  /**< Open wire detected */
    ERROR_AIN_REF_VOLTAGE           = 0x0405U,  /**< Reference voltage fault */
    ERROR_AIN_CALIBRATION           = 0x0406U,  /**< Calibration error */
    ERROR_AIN_ADC_TIMEOUT           = 0x0407U,  /**< ADC conversion timeout */

    /* Output Control errors (0x05xx) */
    ERROR_OUT_OPEN_LOAD             = 0x0501U,  /**< Open load detected */
    ERROR_OUT_SHORT_CIRCUIT         = 0x0502U,  /**< Short circuit detected */
    ERROR_OUT_OVERCURRENT           = 0x0503U,  /**< Output overcurrent */
    ERROR_OUT_OVERTEMPERATURE       = 0x0504U,  /**< Output overtemperature */
    ERROR_OUT_BTT6200_FAULT         = 0x0505U,  /**< BTT6200 fault */
    ERROR_OUT_INVALID_CHANNEL       = 0x0506U,  /**< Invalid channel number */
    ERROR_OUT_CONTROL_TIMEOUT       = 0x0507U,  /**< Control timeout */

    /* Data Storage errors (0x06xx) */
    ERROR_FRAM_I2C_FAULT            = 0x0601U,  /**< FRAM I2C communication */
    ERROR_FRAM_WRITE_FAILED         = 0x0602U,  /**< FRAM write failed */
    ERROR_FRAM_READ_FAILED          = 0x0603U,  /**< FRAM read failed */
    ERROR_FRAM_CRC_MISMATCH         = 0x0604U,  /**< FRAM CRC check failed */
    ERROR_FRAM_ADDRESS_INVALID      = 0x0605U,  /**< Invalid FRAM address */
    ERROR_FRAM_DATA_CORRUPT         = 0x0606U,  /**< Data corruption detected */

    /* Diagnostics errors (0x07xx) */
    ERROR_DIAG_TEMP_SENSOR          = 0x0701U,  /**< Temperature sensor fault */
    ERROR_DIAG_TEMP_WARNING         = 0x0702U,  /**< Temperature warning */
    ERROR_DIAG_TEMP_CRITICAL        = 0x0703U,  /**< Temperature critical */
    ERROR_DIAG_DTC_FULL             = 0x0704U,  /**< DTC storage full */
    ERROR_DIAG_LOG_FULL             = 0x0705U,  /**< Event log full */

    /* Safety errors (0x08xx) - Critical! */
    ERROR_SAFETY_WATCHDOG           = 0x0801U,  /**< Watchdog timeout */
    ERROR_SAFETY_STACK_OVERFLOW     = 0x0802U,  /**< Stack overflow */
    ERROR_SAFETY_PLAUSIBILITY       = 0x0803U,  /**< Plausibility check failed */
    ERROR_SAFETY_TIMING_VIOLATION   = 0x0804U,  /**< Timing constraint violated */
    ERROR_SAFETY_REDUNDANCY         = 0x0805U,  /**< Redundancy check failed */
    ERROR_SAFETY_MEMORY_CORRUPTION  = 0x0806U,  /**< Memory corruption */
    ERROR_SAFETY_CRITICAL_FAULT     = 0x0807U,  /**< Critical safety fault */

    /* BSP/Hardware errors (0x09xx) */
    ERROR_BSP_GPIO_CONFIG           = 0x0901U,  /**< GPIO configuration error */
    ERROR_BSP_I2C_TIMEOUT           = 0x0902U,  /**< I2C timeout */
    ERROR_BSP_I2C_NACK              = 0x0903U,  /**< I2C NACK received */
    ERROR_BSP_SPI_TIMEOUT           = 0x0904U,  /**< SPI timeout */
    ERROR_BSP_ADC_CONFIG            = 0x0905U,  /**< ADC configuration error */
    ERROR_BSP_DMA_ERROR             = 0x0906U,  /**< DMA error */
    ERROR_BSP_CLOCK_CONFIG          = 0x0907U,  /**< Clock configuration error */

    /* Application errors (0x0Axx) */
    ERROR_APP_NOT_INITIALIZED       = 0x0A01U,  /**< Application not initialized */
    ERROR_APP_INVALID_STATE         = 0x0A02U,  /**< Invalid state transition */
    ERROR_APP_CONFIG_INVALID        = 0x0A03U,  /**< Invalid configuration */
    ERROR_APP_TASK_OVERRUN          = 0x0A04U,  /**< Task execution overrun */

    ERROR_MAX                       = 0xFFFFU   /**< Maximum error value */
} ErrorCode_t;

/**
 * @brief Error severity levels
 */
typedef enum {
    SEVERITY_INFO     = 0x00U,  /**< Informational */
    SEVERITY_WARNING  = 0x01U,  /**< Warning - non-critical */
    SEVERITY_ERROR    = 0x02U,  /**< Error - recoverable */
    SEVERITY_CRITICAL = 0x03U   /**< Critical - requires safe state */
} ErrorSeverity_t;

/**
 * @brief DTC (Diagnostic Trouble Code) status
 */
typedef enum {
    DTC_STATUS_PENDING   = 0x00U,  /**< DTC pending confirmation */
    DTC_STATUS_CONFIRMED = 0x01U,  /**< DTC confirmed */
    DTC_STATUS_CLEARED   = 0x02U   /**< DTC cleared */
} DTC_Status_t;

/**
 * @brief DTC information structure (public API)
 */
typedef struct {
    ErrorCode_t code;              /**< Error code */
    uint32_t timestamp_ms;         /**< Timestamp in milliseconds */
    uint32_t occurrenceCount;      /**< Number of occurrences */
    uint32_t param1;               /**< First error parameter */
    uint32_t param2;               /**< Second error parameter */
    uint32_t param3;               /**< Third error parameter */
    DTC_Status_t status;           /**< DTC status */
    bool confirmed;                /**< Confirmation flag */
} DTC_Info_t;

/**
 * @brief Error statistics structure
 */
typedef struct {
    uint32_t totalErrorCount;      /**< Total error count */
    uint32_t criticalErrorCount;   /**< Critical error count */
    uint32_t warningCount;         /**< Warning count */
    uint32_t activeDTCCount;       /**< Active DTC count */
    uint32_t lastErrorTime_ms;     /**< Last error timestamp */
    bool safeStateActive;          /**< Safe state active flag */
} ErrorStatistics_t;

/**
 * @brief Error callback function pointer
 */
typedef void (*ErrorCallback_t)(ErrorCode_t code, uint32_t param1,
                                 uint32_t param2, uint32_t param3);

/**
 * @brief Error record structure for logging
 */
typedef struct {
    ErrorCode_t     code;         /**< Error code */
    ErrorSeverity_t severity;     /**< Error severity */
    uint32_t        timestamp_ms; /**< Timestamp in milliseconds */
    uint16_t        module_id;    /**< Module identifier */
    uint16_t        line_number;  /**< Source line number */
    uint32_t        data;         /**< Additional error data */
} ErrorRecord_t;

/*============================================================================*/
/* ERROR HANDLING MACROS                                                      */
/*============================================================================*/

/**
 * @brief Check status and return on error
 * @param status Status to check
 */
#define CHECK_STATUS(status) \
    do { \
        if ((status) != STATUS_OK) { \
            return (status); \
        } \
    } while (0)

/**
 * @brief Check pointer and return error if NULL
 * @param ptr Pointer to check
 */
#define CHECK_NULL(ptr) \
    do { \
        if ((ptr) == NULL) { \
            return STATUS_ERROR_PARAM; \
        } \
    } while (0)

/**
 * @brief Check range and return error if out of bounds
 * @param val Value to check
 * @param min Minimum allowed value
 * @param max Maximum allowed value
 */
#define CHECK_RANGE(val, min, max) \
    do { \
        if (((val) < (min)) || ((val) > (max))) { \
            return STATUS_ERROR_RANGE; \
        } \
    } while (0)

/**
 * @brief Assert condition (only in debug builds)
 * @param cond Condition to assert
 */
#ifdef DEBUG
#define ASSERT(cond) \
    do { \
        if (!(cond)) { \
            ErrorHandler_Assert(__FILE__, __LINE__); \
        } \
    } while (0)
#else
#define ASSERT(cond) ((void)0)
#endif

/*============================================================================*/
/* FUNCTION PROTOTYPES                                                        */
/*============================================================================*/

/**
 * @brief Get error severity for a given error code
 * @param[in] errorCode Error code to evaluate
 * @return Error severity level
 */
ErrorSeverity_t ErrorHandler_GetSeverity(ErrorCode_t errorCode);

/**
 * @brief Convert error code to status
 * @param[in] errorCode Error code
 * @return Corresponding status code
 */
Status_t ErrorHandler_ErrorCodeToStatus(ErrorCode_t errorCode);

/**
 * @brief Get string description of error code
 * @param[in] errorCode Error code
 * @return Pointer to constant error string
 */
const char* ErrorHandler_GetErrorString(ErrorCode_t errorCode);

/**
 * @brief Get string description of status code
 * @param[in] status Status code
 * @return Pointer to constant status string
 */
const char* ErrorHandler_GetStatusString(Status_t status);

/**
 * @brief Log error with timestamp and context
 * @param[in] code Error code to log
 * @param[in] param1 First error parameter (e.g., module ID)
 * @param[in] param2 Second error parameter (e.g., line number)
 * @param[in] param3 Third error parameter (additional data)
 * @return STATUS_OK on success, error code otherwise
 */
Status_t ErrorHandler_LogError(ErrorCode_t code, uint32_t param1,
                                uint32_t param2, uint32_t param3);

/**
 * @brief Assert handler for debug builds
 * @param[in] file Source file name
 * @param[in] line Line number
 */
void ErrorHandler_Assert(const char *file, uint32_t line);

/**
 * @brief Hard fault handler
 */
void ErrorHandler_HardFault(void);

/**
 * @brief Enter safe state (critical errors)
 */
void ErrorHandler_SafeState(void);

/**
 * @brief Initialize error handler
 * @return STATUS_OK on success, error code otherwise
 */
Status_t ErrorHandler_Init(void);

/**
 * @brief Get DTC information by error code
 * @param[in] code Error code to query
 * @param[out] pInfo Pointer to DTC info structure
 * @return STATUS_OK on success, STATUS_ERROR_NOT_FOUND if not found
 */
Status_t ErrorHandler_GetDTC(ErrorCode_t code, DTC_Info_t *pInfo);

/**
 * @brief Clear DTC by error code
 * @param[in] code Error code to clear
 * @return STATUS_OK on success, error code otherwise
 */
Status_t ErrorHandler_ClearDTC(ErrorCode_t code);

/**
 * @brief Clear all DTCs
 * @return STATUS_OK on success, error code otherwise
 */
Status_t ErrorHandler_ClearAllDTCs(void);

/**
 * @brief Get error statistics
 * @param[out] pStats Pointer to statistics structure
 * @return STATUS_OK on success, error code otherwise
 */
Status_t ErrorHandler_GetStatistics(ErrorStatistics_t *pStats);

/**
 * @brief Register error callback
 * @param[in] callback Callback function pointer
 * @return STATUS_OK on success, error code otherwise
 */
Status_t ErrorHandler_RegisterCallback(ErrorCallback_t callback);

/**
 * @brief Check if DTC is active
 * @param[in] code Error code to check
 * @return true if active, false otherwise
 */
bool ErrorHandler_IsDTCActive(ErrorCode_t code);

#ifdef __cplusplus
}
#endif

#endif /* APP_ERRORS_H */
