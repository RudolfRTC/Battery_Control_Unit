/**
 * @file    app_types.h
 * @brief   Common type definitions for BCU firmware
 * @author  Battery Control Unit Development Team
 * @date    2026-01-09
 * @version 1.0.0
 *
 * @note    MISRA C:2012 compliant
 * @note    ISO 26262 ASIL-B safety requirements
 *
 * @copyright Copyright (c) 2026
 */

#ifndef APP_TYPES_H
#define APP_TYPES_H

#ifdef __cplusplus
extern "C" {
#endif

/*============================================================================*/
/* INCLUDES                                                                   */
/*============================================================================*/
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/*============================================================================*/
/* CONSTANTS AND MACROS                                                       */
/*============================================================================*/

/** @brief Null pointer definition */
#ifndef NULL
#define NULL ((void *)0)
#endif

/** @brief Boolean definitions for clarity */
#define TRUE  (1U)
#define FALSE (0U)

/** @brief Enable/Disable definitions */
#define ENABLE  (1U)
#define DISABLE (0U)

/** @brief Set/Reset definitions */
#define SET   (1U)
#define RESET (0U)

/** @brief Bit manipulation macros */
#define BIT_SET(REG, BIT)     ((REG) |= (BIT))
#define BIT_CLEAR(REG, BIT)   ((REG) &= ~(BIT))
#define BIT_READ(REG, BIT)    ((REG) & (BIT))
#define BIT_TOGGLE(REG, BIT)  ((REG) ^= (BIT))

/** @brief Array size calculation */
#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof((arr)[0]))

/** @brief Min/Max macros */
#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#define MAX(a, b) (((a) > (b)) ? (a) : (b))

/** @brief Unused parameter macro for MISRA compliance */
#define UNUSED(x) ((void)(x))

/*============================================================================*/
/* TYPE DEFINITIONS                                                           */
/*============================================================================*/

/**
 * @brief Standard function return status codes
 * @note  All API functions must return this type
 */
typedef enum {
    STATUS_OK                = 0x00U,  /**< Operation successful */
    STATUS_ERROR             = 0x01U,  /**< Generic error */
    STATUS_ERROR_TIMEOUT     = 0x02U,  /**< Operation timeout */
    STATUS_ERROR_BUSY        = 0x03U,  /**< Resource busy */
    STATUS_ERROR_PARAM       = 0x04U,  /**< Invalid parameter */
    STATUS_ERROR_HW_FAULT    = 0x05U,  /**< Hardware fault detected */
    STATUS_ERROR_OVERFLOW    = 0x06U,  /**< Buffer/counter overflow */
    STATUS_ERROR_UNDERFLOW   = 0x07U,  /**< Buffer/counter underflow */
    STATUS_ERROR_CRC         = 0x08U,  /**< CRC check failed */
    STATUS_ERROR_NOT_INIT    = 0x09U,  /**< Module not initialized */
    STATUS_ERROR_ALREADY_INIT = 0x0AU, /**< Module already initialized */
    STATUS_ERROR_NOT_SUPPORTED = 0x0BU, /**< Feature not supported */
    STATUS_ERROR_NO_MEMORY   = 0x0CU,  /**< Memory allocation failed */
    STATUS_ERROR_RANGE       = 0x0DU,  /**< Value out of range */
    STATUS_ERROR_INVALID_STATE = 0x0EU, /**< Invalid state transition */
    STATUS_ERROR_SAFETY      = 0x0FU   /**< Safety violation detected */
} Status_t;

/**
 * @brief State enumeration for modules
 */
typedef enum {
    STATE_UNINITIALIZED = 0x00U,  /**< Module not initialized */
    STATE_INITIALIZED   = 0x01U,  /**< Module initialized */
    STATE_RUNNING       = 0x02U,  /**< Module running normally */
    STATE_SUSPENDED     = 0x03U,  /**< Module suspended */
    STATE_ERROR         = 0x04U,  /**< Module in error state */
    STATE_SAFE          = 0x05U   /**< Module in safe state */
} ModuleState_t;

/**
 * @brief Priority levels for tasks/events
 */
typedef enum {
    PRIORITY_CRITICAL = 0x00U,  /**< Critical priority (1ms) */
    PRIORITY_HIGH     = 0x01U,  /**< High priority (10ms) */
    PRIORITY_MEDIUM   = 0x02U,  /**< Medium priority (50ms) */
    PRIORITY_LOW      = 0x03U   /**< Low priority (100ms) */
} Priority_t;

/**
 * @brief Timestamp structure (millisecond resolution)
 */
typedef struct {
    uint32_t milliseconds;  /**< Milliseconds since startup */
} Timestamp_t;

/**
 * @brief Version information structure
 */
typedef struct {
    uint8_t major;       /**< Major version number */
    uint8_t minor;       /**< Minor version number */
    uint8_t patch;       /**< Patch version number */
    uint8_t reserved;    /**< Reserved for alignment */
} Version_t;

/**
 * @brief Generic callback function pointer
 */
typedef void (*Callback_t)(void);

/**
 * @brief Error callback function pointer with error code
 */
typedef void (*ErrorCallback_t)(Status_t errorCode);

/**
 * @brief Range validation structure
 */
typedef struct {
    int32_t min;  /**< Minimum allowed value */
    int32_t max;  /**< Maximum allowed value */
} Range_t;

/**
 * @brief Voltage measurement type (millivolts)
 */
typedef uint32_t Voltage_mV_t;

/**
 * @brief Current measurement type (milliamperes)
 */
typedef int32_t Current_mA_t;

/**
 * @brief Temperature measurement type (0.01°C resolution)
 */
typedef int16_t Temperature_t;

/**
 * @brief Power measurement type (milliwatts)
 */
typedef uint32_t Power_mW_t;

/**
 * @brief Percentage type (0.01% resolution, 0-10000 = 0.00-100.00%)
 */
typedef uint16_t Percentage_t;

/*============================================================================*/
/* STATIC ASSERTIONS FOR TYPE SAFETY                                          */
/*============================================================================*/

/* Ensure fixed-width types are correct size */
_Static_assert(sizeof(uint8_t) == 1U, "uint8_t must be 1 byte");
_Static_assert(sizeof(uint16_t) == 2U, "uint16_t must be 2 bytes");
_Static_assert(sizeof(uint32_t) == 4U, "uint32_t must be 4 bytes");
_Static_assert(sizeof(int8_t) == 1U, "int8_t must be 1 byte");
_Static_assert(sizeof(int16_t) == 2U, "int16_t must be 2 bytes");
_Static_assert(sizeof(int32_t) == 4U, "int32_t must be 4 bytes");

/* Ensure bool is correct size */
_Static_assert(sizeof(bool) == 1U, "bool must be 1 byte");

#ifdef __cplusplus
}
#endif

#endif /* APP_TYPES_H */
