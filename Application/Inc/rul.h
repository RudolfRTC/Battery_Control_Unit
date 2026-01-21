/**
 * @file    rul.h
 * @brief   Remaining Useful Life (RUL) prediction module for battery systems
 * @author  Battery Control Unit Development Team
 * @date    2026-01-20
 * @version 1.0.0
 *
 * @note    MISRA C:2012 compliant
 * @note    ISO 26262 ASIL-B safety requirements
 *
 * @copyright Copyright (c) 2026
 */

#ifndef RUL_H
#define RUL_H

#ifdef __cplusplus
extern "C" {
#endif

/*============================================================================*/
/* INCLUDES                                                                   */
/*============================================================================*/
#include "app_types.h"

/*============================================================================*/
/* CONSTANTS AND MACROS                                                       */
/*============================================================================*/

/** @brief Maximum number of temperature history samples */
#define RUL_TEMP_HISTORY_SIZE       (100U)

/** @brief Maximum number of cycle count samples */
#define RUL_CYCLE_HISTORY_SIZE      (50U)

/** @brief End of Life threshold (70% of original capacity) */
#define RUL_EOL_THRESHOLD_PERCENT   (7000U)  /**< 70.00% in 0.01% units */

/** @brief Default nominal capacity (mAh) */
#define RUL_DEFAULT_NOMINAL_CAPACITY (5000U)

/** @brief Maximum allowed SoH percentage */
#define RUL_MAX_SOH_PERCENT         (10000U)  /**< 100.00% */

/** @brief Minimum valid SoH percentage */
#define RUL_MIN_SOH_PERCENT         (0U)      /**< 0.00% */

/** @brief Temperature weight factor for degradation */
#define RUL_TEMP_WEIGHT_FACTOR      (100U)

/** @brief Cycle count weight factor for degradation */
#define RUL_CYCLE_WEIGHT_FACTOR     (50U)

/** @brief Calendar aging weight factor (per day) */
#define RUL_CALENDAR_WEIGHT_FACTOR  (1U)

/** @brief Reference temperature for degradation (25°C in 0.01°C units) */
#define RUL_REFERENCE_TEMP          (2500)

/** @brief High temperature threshold (45°C in 0.01°C units) */
#define RUL_HIGH_TEMP_THRESHOLD     (4500)

/*============================================================================*/
/* TYPE DEFINITIONS                                                           */
/*============================================================================*/

/**
 * @brief RUL module state enumeration
 */
typedef enum {
    RUL_STATE_UNINITIALIZED = 0x00U,  /**< Module not initialized */
    RUL_STATE_INITIALIZED   = 0x01U,  /**< Module initialized */
    RUL_STATE_RUNNING       = 0x02U,  /**< Module running */
    RUL_STATE_ERROR         = 0x03U   /**< Module in error state */
} RUL_State_t;

/**
 * @brief Battery cycle information
 */
typedef struct {
    uint32_t totalCycles;           /**< Total full charge-discharge cycles */
    uint32_t partialCycles;         /**< Partial cycles (accumulated to full) */
    uint32_t deepDischargeCycles;   /**< Deep discharge events (<20% SoC) */
    uint32_t highTempCycles;        /**< Cycles at high temperature */
} RUL_CycleInfo_t;

/**
 * @brief Battery health metrics
 */
typedef struct {
    Percentage_t    soh;                /**< State of Health (0-10000 = 0-100%) */
    uint32_t        currentCapacity_mAh; /**< Current capacity in mAh */
    uint32_t        nominalCapacity_mAh; /**< Nominal (new) capacity in mAh */
    uint32_t        capacityFade_mAh;    /**< Capacity fade in mAh */
    Temperature_t   avgTemperature;      /**< Average operating temperature */
    Temperature_t   maxTemperature;      /**< Maximum recorded temperature */
} RUL_HealthMetrics_t;

/**
 * @brief RUL prediction result
 */
typedef struct {
    uint32_t    remainingCycles;        /**< Estimated remaining full cycles */
    uint32_t    remainingDays;          /**< Estimated remaining calendar days */
    uint32_t    remainingHours;         /**< Estimated remaining operating hours */
    Percentage_t confidence;            /**< Confidence level (0-10000 = 0-100%) */
    bool        isValid;                /**< Prediction validity flag */
} RUL_Prediction_t;

/**
 * @brief Degradation factors
 */
typedef struct {
    float       cyclicDegradation;      /**< Degradation from cycles (% per cycle) */
    float       calendarDegradation;    /**< Degradation from aging (% per day) */
    float       temperatureFactor;      /**< Temperature acceleration factor */
    float       totalDegradationRate;   /**< Combined degradation rate (% per day) */
} RUL_DegradationFactors_t;

/**
 * @brief RUL module configuration
 */
typedef struct {
    uint32_t    nominalCapacity_mAh;    /**< Battery nominal capacity */
    Percentage_t eolThreshold;          /**< End of Life threshold */
    uint16_t    tempHistorySize;        /**< Temperature history buffer size */
    uint16_t    cycleHistorySize;       /**< Cycle history buffer size */
    bool        enableTempCompensation; /**< Enable temperature compensation */
    bool        enableCalendarAging;    /**< Enable calendar aging model */
} RUL_Config_t;

/**
 * @brief RUL module context (internal state)
 */
typedef struct {
    RUL_State_t             state;              /**< Module state */
    RUL_Config_t            config;             /**< Configuration */
    RUL_CycleInfo_t         cycleInfo;          /**< Cycle information */
    RUL_HealthMetrics_t     health;             /**< Health metrics */
    RUL_Prediction_t        prediction;         /**< Latest prediction */
    RUL_DegradationFactors_t degradation;       /**< Degradation factors */

    /* History buffers */
    Temperature_t   tempHistory[RUL_TEMP_HISTORY_SIZE];  /**< Temperature history */
    uint16_t        tempHistoryIndex;                     /**< Current index in temp history */
    uint16_t        tempHistoryCount;                     /**< Number of valid samples */

    /* Timing */
    uint32_t        lastUpdateTime_ms;          /**< Last update timestamp */
    uint32_t        operatingTime_hours;        /**< Total operating time */
    uint32_t        calendarAge_days;           /**< Calendar age since first use */

    /* Statistics */
    uint32_t        updateCount;                /**< Number of updates performed */
    uint32_t        predictionCount;            /**< Number of predictions made */
} RUL_Context_t;

/*============================================================================*/
/* FUNCTION PROTOTYPES                                                        */
/*============================================================================*/

/**
 * @brief Initialize RUL module with default configuration
 * @param[in,out] pContext Pointer to RUL context
 * @return STATUS_OK on success, error code otherwise
 */
Status_t RUL_Init(RUL_Context_t *pContext);

/**
 * @brief Initialize RUL module with custom configuration
 * @param[in,out] pContext Pointer to RUL context
 * @param[in]     pConfig  Pointer to configuration structure
 * @return STATUS_OK on success, error code otherwise
 */
Status_t RUL_InitWithConfig(RUL_Context_t *pContext, const RUL_Config_t *pConfig);

/**
 * @brief Deinitialize RUL module
 * @param[in,out] pContext Pointer to RUL context
 * @return STATUS_OK on success, error code otherwise
 */
Status_t RUL_Deinit(RUL_Context_t *pContext);

/**
 * @brief Update RUL calculations with new battery data
 * @param[in,out] pContext      Pointer to RUL context
 * @param[in]     current_mA    Current measurement (positive = charging)
 * @param[in]     voltage_mV    Voltage measurement
 * @param[in]     temperature   Temperature measurement
 * @param[in]     soc_percent   State of Charge (0-10000 = 0-100%)
 * @return STATUS_OK on success, error code otherwise
 */
Status_t RUL_Update(RUL_Context_t *pContext,
                    Current_mA_t current_mA,
                    Voltage_mV_t voltage_mV,
                    Temperature_t temperature,
                    Percentage_t soc_percent);

/**
 * @brief Calculate and update State of Health (SoH)
 * @param[in,out] pContext           Pointer to RUL context
 * @param[in]     measuredCapacity_mAh Measured current capacity
 * @return STATUS_OK on success, error code otherwise
 */
Status_t RUL_UpdateSoH(RUL_Context_t *pContext, uint32_t measuredCapacity_mAh);

/**
 * @brief Increment cycle counter
 * @param[in,out] pContext    Pointer to RUL context
 * @param[in]     isFullCycle true for full cycle, false for partial
 * @param[in]     minSoC      Minimum SoC reached during cycle
 * @return STATUS_OK on success, error code otherwise
 */
Status_t RUL_IncrementCycle(RUL_Context_t *pContext, bool isFullCycle, Percentage_t minSoC);

/**
 * @brief Predict Remaining Useful Life
 * @param[in,out] pContext    Pointer to RUL context
 * @param[out]    pPrediction Pointer to store prediction result
 * @return STATUS_OK on success, error code otherwise
 */
Status_t RUL_PredictLife(RUL_Context_t *pContext, RUL_Prediction_t *pPrediction);

/**
 * @brief Get current health metrics
 * @param[in]  pContext Pointer to RUL context
 * @param[out] pHealth  Pointer to store health metrics
 * @return STATUS_OK on success, error code otherwise
 */
Status_t RUL_GetHealthMetrics(const RUL_Context_t *pContext, RUL_HealthMetrics_t *pHealth);

/**
 * @brief Get current cycle information
 * @param[in]  pContext   Pointer to RUL context
 * @param[out] pCycleInfo Pointer to store cycle information
 * @return STATUS_OK on success, error code otherwise
 */
Status_t RUL_GetCycleInfo(const RUL_Context_t *pContext, RUL_CycleInfo_t *pCycleInfo);

/**
 * @brief Get degradation factors
 * @param[in]  pContext      Pointer to RUL context
 * @param[out] pDegradation  Pointer to store degradation factors
 * @return STATUS_OK on success, error code otherwise
 */
Status_t RUL_GetDegradationFactors(const RUL_Context_t *pContext,
                                   RUL_DegradationFactors_t *pDegradation);

/**
 * @brief Get module state
 * @param[in]  pContext Pointer to RUL context
 * @param[out] pState   Pointer to store current state
 * @return STATUS_OK on success, error code otherwise
 */
Status_t RUL_GetState(const RUL_Context_t *pContext, RUL_State_t *pState);

/**
 * @brief Reset RUL module (clear all history and statistics)
 * @param[in,out] pContext Pointer to RUL context
 * @return STATUS_OK on success, error code otherwise
 */
Status_t RUL_Reset(RUL_Context_t *pContext);

/**
 * @brief Set nominal capacity (recalibration)
 * @param[in,out] pContext          Pointer to RUL context
 * @param[in]     nominalCapacity_mAh New nominal capacity
 * @return STATUS_OK on success, error code otherwise
 */
Status_t RUL_SetNominalCapacity(RUL_Context_t *pContext, uint32_t nominalCapacity_mAh);

/**
 * @brief Set End of Life threshold
 * @param[in,out] pContext      Pointer to RUL context
 * @param[in]     eolThreshold  New EOL threshold (0-10000 = 0-100%)
 * @return STATUS_OK on success, error code otherwise
 */
Status_t RUL_SetEOLThreshold(RUL_Context_t *pContext, Percentage_t eolThreshold);

/**
 * @brief Update operating time
 * @param[in,out] pContext     Pointer to RUL context
 * @param[in]     deltaTime_ms Time increment in milliseconds
 * @return STATUS_OK on success, error code otherwise
 */
Status_t RUL_UpdateOperatingTime(RUL_Context_t *pContext, uint32_t deltaTime_ms);

#ifdef __cplusplus
}
#endif

#endif /* RUL_H */
