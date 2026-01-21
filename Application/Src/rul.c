/**
 * @file    rul.c
 * @brief   Remaining Useful Life (RUL) prediction module implementation
 * @author  Battery Control Unit Development Team
 * @date    2026-01-20
 * @version 1.0.0
 *
 * @note    MISRA C:2012 compliant
 * @note    ISO 26262 ASIL-B safety requirements
 *
 * @copyright Copyright (c) 2026
 */

/*============================================================================*/
/* INCLUDES                                                                   */
/*============================================================================*/
#include "rul.h"
#include <string.h>
#include <math.h>

/*============================================================================*/
/* PRIVATE CONSTANTS                                                          */
/*============================================================================*/

/** @brief Deep discharge threshold (20% in 0.01% units) */
#define DEEP_DISCHARGE_THRESHOLD    (2000U)

/** @brief Cycle degradation rate (% per 1000 cycles) */
#define CYCLE_DEGRADATION_RATE      (0.05f)

/** @brief Calendar aging rate (% per year at 25°C) */
#define CALENDAR_AGING_RATE         (0.02f)

/** @brief Temperature acceleration factor (per 10°C above reference) */
#define TEMP_ACCELERATION_FACTOR    (1.5f)

/** @brief Minimum confidence level */
#define MIN_CONFIDENCE_LEVEL        (3000U)  /**< 30% */

/** @brief Hours per day */
#define HOURS_PER_DAY              (24U)

/** @brief Milliseconds per hour */
#define MS_PER_HOUR                (3600000U)

/** @brief Days per year */
#define DAYS_PER_YEAR              (365U)

/*============================================================================*/
/* PRIVATE FUNCTION PROTOTYPES                                                */
/*============================================================================*/

static Status_t RUL_ValidateContext(const RUL_Context_t *pContext);
static Status_t RUL_ValidateConfig(const RUL_Config_t *pConfig);
static void RUL_CalculateDegradationFactors(RUL_Context_t *pContext);
static float RUL_CalculateTemperatureFactor(Temperature_t avgTemp);
static float RUL_GetAverageTemperature(const RUL_Context_t *pContext);
static void RUL_AddTemperatureSample(RUL_Context_t *pContext, Temperature_t temp);
static uint32_t RUL_CalculateRemainingCycles(const RUL_Context_t *pContext);
static uint32_t RUL_CalculateRemainingDays(const RUL_Context_t *pContext);
static Percentage_t RUL_CalculateConfidence(const RUL_Context_t *pContext);

/*============================================================================*/
/* PUBLIC FUNCTION IMPLEMENTATIONS                                            */
/*============================================================================*/

/**
 * @brief Initialize RUL module with default configuration
 */
Status_t RUL_Init(RUL_Context_t *pContext)
{
    RUL_Config_t defaultConfig;

    /* Validate input parameter */
    if (pContext == NULL)
    {
        return STATUS_ERROR_PARAM;
    }

    /* Setup default configuration */
    defaultConfig.nominalCapacity_mAh = RUL_DEFAULT_NOMINAL_CAPACITY;
    defaultConfig.eolThreshold = RUL_EOL_THRESHOLD_PERCENT;
    defaultConfig.tempHistorySize = RUL_TEMP_HISTORY_SIZE;
    defaultConfig.cycleHistorySize = RUL_CYCLE_HISTORY_SIZE;
    defaultConfig.enableTempCompensation = true;
    defaultConfig.enableCalendarAging = true;

    return RUL_InitWithConfig(pContext, &defaultConfig);
}

/**
 * @brief Initialize RUL module with custom configuration
 */
Status_t RUL_InitWithConfig(RUL_Context_t *pContext, const RUL_Config_t *pConfig)
{
    Status_t status;

    /* Validate input parameters */
    if ((pContext == NULL) || (pConfig == NULL))
    {
        return STATUS_ERROR_PARAM;
    }

    /* Validate configuration */
    status = RUL_ValidateConfig(pConfig);
    if (status != STATUS_OK)
    {
        return status;
    }

    /* Clear context */
    (void)memset(pContext, 0, sizeof(RUL_Context_t));

    /* Copy configuration */
    (void)memcpy(&pContext->config, pConfig, sizeof(RUL_Config_t));

    /* Initialize health metrics */
    pContext->health.soh = RUL_MAX_SOH_PERCENT;  /* Start at 100% */
    pContext->health.currentCapacity_mAh = pConfig->nominalCapacity_mAh;
    pContext->health.nominalCapacity_mAh = pConfig->nominalCapacity_mAh;
    pContext->health.capacityFade_mAh = 0U;
    pContext->health.avgTemperature = RUL_REFERENCE_TEMP;
    pContext->health.maxTemperature = RUL_REFERENCE_TEMP;

    /* Initialize cycle information */
    pContext->cycleInfo.totalCycles = 0U;
    pContext->cycleInfo.partialCycles = 0U;
    pContext->cycleInfo.deepDischargeCycles = 0U;
    pContext->cycleInfo.highTempCycles = 0U;

    /* Initialize prediction */
    pContext->prediction.remainingCycles = 0U;
    pContext->prediction.remainingDays = 0U;
    pContext->prediction.remainingHours = 0U;
    pContext->prediction.confidence = MIN_CONFIDENCE_LEVEL;
    pContext->prediction.isValid = false;

    /* Initialize degradation factors */
    pContext->degradation.cyclicDegradation = CYCLE_DEGRADATION_RATE;
    pContext->degradation.calendarDegradation = CALENDAR_AGING_RATE / (float)DAYS_PER_YEAR;
    pContext->degradation.temperatureFactor = 1.0f;
    pContext->degradation.totalDegradationRate = 0.0f;

    /* Initialize timing */
    pContext->lastUpdateTime_ms = 0U;
    pContext->operatingTime_hours = 0U;
    pContext->calendarAge_days = 0U;

    /* Initialize statistics */
    pContext->updateCount = 0U;
    pContext->predictionCount = 0U;

    /* Set state */
    pContext->state = RUL_STATE_INITIALIZED;

    return STATUS_OK;
}

/**
 * @brief Deinitialize RUL module
 */
Status_t RUL_Deinit(RUL_Context_t *pContext)
{
    /* Validate input parameter */
    if (pContext == NULL)
    {
        return STATUS_ERROR_PARAM;
    }

    /* Clear context */
    (void)memset(pContext, 0, sizeof(RUL_Context_t));

    /* Set state */
    pContext->state = RUL_STATE_UNINITIALIZED;

    return STATUS_OK;
}

/**
 * @brief Update RUL calculations with new battery data
 */
Status_t RUL_Update(RUL_Context_t *pContext,
                    Current_mA_t current_mA,
                    Voltage_mV_t voltage_mV,
                    Temperature_t temperature,
                    Percentage_t soc_percent)
{
    Status_t status;

    /* Validate context */
    status = RUL_ValidateContext(pContext);
    if (status != STATUS_OK)
    {
        return status;
    }

    /* Add temperature to history */
    RUL_AddTemperatureSample(pContext, temperature);

    /* Update average and max temperature */
    pContext->health.avgTemperature = (Temperature_t)RUL_GetAverageTemperature(pContext);
    if (temperature > pContext->health.maxTemperature)
    {
        pContext->health.maxTemperature = temperature;
    }

    /* Recalculate degradation factors */
    RUL_CalculateDegradationFactors(pContext);

    /* Increment update counter */
    pContext->updateCount++;

    /* Set state to running */
    pContext->state = RUL_STATE_RUNNING;

    UNUSED(current_mA);
    UNUSED(voltage_mV);
    UNUSED(soc_percent);

    return STATUS_OK;
}

/**
 * @brief Calculate and update State of Health (SoH)
 */
Status_t RUL_UpdateSoH(RUL_Context_t *pContext, uint32_t measuredCapacity_mAh)
{
    Status_t status;
    uint32_t nominalCapacity;
    Percentage_t newSoH;

    /* Validate context */
    status = RUL_ValidateContext(pContext);
    if (status != STATUS_OK)
    {
        return status;
    }

    /* Validate measured capacity */
    if (measuredCapacity_mAh == 0U)
    {
        return STATUS_ERROR_PARAM;
    }

    nominalCapacity = pContext->health.nominalCapacity_mAh;

    /* Calculate SoH as percentage of nominal capacity */
    if (measuredCapacity_mAh > nominalCapacity)
    {
        /* Cap at 100% if measured exceeds nominal */
        newSoH = RUL_MAX_SOH_PERCENT;
        pContext->health.currentCapacity_mAh = nominalCapacity;
        pContext->health.capacityFade_mAh = 0U;
    }
    else
    {
        /* Calculate SoH percentage */
        newSoH = (Percentage_t)((measuredCapacity_mAh * (uint32_t)RUL_MAX_SOH_PERCENT) /
                                nominalCapacity);

        /* Update capacity metrics */
        pContext->health.currentCapacity_mAh = measuredCapacity_mAh;
        pContext->health.capacityFade_mAh = nominalCapacity - measuredCapacity_mAh;
    }

    /* Update SoH */
    pContext->health.soh = newSoH;

    return STATUS_OK;
}

/**
 * @brief Increment cycle counter
 */
Status_t RUL_IncrementCycle(RUL_Context_t *pContext, bool isFullCycle, Percentage_t minSoC)
{
    Status_t status;

    /* Validate context */
    status = RUL_ValidateContext(pContext);
    if (status != STATUS_OK)
    {
        return status;
    }

    if (isFullCycle)
    {
        /* Increment full cycle counter */
        pContext->cycleInfo.totalCycles++;

        /* Check for deep discharge */
        if (minSoC < DEEP_DISCHARGE_THRESHOLD)
        {
            pContext->cycleInfo.deepDischargeCycles++;
        }

        /* Check for high temperature cycle */
        if (pContext->health.avgTemperature > RUL_HIGH_TEMP_THRESHOLD)
        {
            pContext->cycleInfo.highTempCycles++;
        }
    }
    else
    {
        /* Increment partial cycle counter */
        pContext->cycleInfo.partialCycles++;
    }

    /* Recalculate degradation factors */
    RUL_CalculateDegradationFactors(pContext);

    return STATUS_OK;
}

/**
 * @brief Predict Remaining Useful Life
 */
Status_t RUL_PredictLife(RUL_Context_t *pContext, RUL_Prediction_t *pPrediction)
{
    Status_t status;
    uint32_t remainingCycles;
    uint32_t remainingDays;
    uint32_t remainingHours;
    Percentage_t confidence;

    /* Validate context and output parameter */
    status = RUL_ValidateContext(pContext);
    if (status != STATUS_OK)
    {
        return status;
    }

    if (pPrediction == NULL)
    {
        return STATUS_ERROR_PARAM;
    }

    /* Check if SoH is already below EOL threshold */
    if (pContext->health.soh <= pContext->config.eolThreshold)
    {
        /* Battery has reached end of life */
        pPrediction->remainingCycles = 0U;
        pPrediction->remainingDays = 0U;
        pPrediction->remainingHours = 0U;
        pPrediction->confidence = RUL_MAX_SOH_PERCENT;  /* 100% confidence at EOL */
        pPrediction->isValid = true;
    }
    else
    {
        /* Calculate predictions */
        remainingCycles = RUL_CalculateRemainingCycles(pContext);
        remainingDays = RUL_CalculateRemainingDays(pContext);
        confidence = RUL_CalculateConfidence(pContext);

        /* Calculate remaining hours based on days */
        remainingHours = remainingDays * HOURS_PER_DAY;

        /* Store prediction */
        pPrediction->remainingCycles = remainingCycles;
        pPrediction->remainingDays = remainingDays;
        pPrediction->remainingHours = remainingHours;
        pPrediction->confidence = confidence;
        pPrediction->isValid = true;
    }

    /* Store prediction in context */
    (void)memcpy(&pContext->prediction, pPrediction, sizeof(RUL_Prediction_t));

    /* Increment prediction counter */
    pContext->predictionCount++;

    return STATUS_OK;
}

/**
 * @brief Get current health metrics
 */
Status_t RUL_GetHealthMetrics(const RUL_Context_t *pContext, RUL_HealthMetrics_t *pHealth)
{
    Status_t status;

    /* Validate context and output parameter */
    status = RUL_ValidateContext(pContext);
    if (status != STATUS_OK)
    {
        return status;
    }

    if (pHealth == NULL)
    {
        return STATUS_ERROR_PARAM;
    }

    /* Copy health metrics */
    (void)memcpy(pHealth, &pContext->health, sizeof(RUL_HealthMetrics_t));

    return STATUS_OK;
}

/**
 * @brief Get current cycle information
 */
Status_t RUL_GetCycleInfo(const RUL_Context_t *pContext, RUL_CycleInfo_t *pCycleInfo)
{
    Status_t status;

    /* Validate context and output parameter */
    status = RUL_ValidateContext(pContext);
    if (status != STATUS_OK)
    {
        return status;
    }

    if (pCycleInfo == NULL)
    {
        return STATUS_ERROR_PARAM;
    }

    /* Copy cycle information */
    (void)memcpy(pCycleInfo, &pContext->cycleInfo, sizeof(RUL_CycleInfo_t));

    return STATUS_OK;
}

/**
 * @brief Get degradation factors
 */
Status_t RUL_GetDegradationFactors(const RUL_Context_t *pContext,
                                   RUL_DegradationFactors_t *pDegradation)
{
    Status_t status;

    /* Validate context and output parameter */
    status = RUL_ValidateContext(pContext);
    if (status != STATUS_OK)
    {
        return status;
    }

    if (pDegradation == NULL)
    {
        return STATUS_ERROR_PARAM;
    }

    /* Copy degradation factors */
    (void)memcpy(pDegradation, &pContext->degradation, sizeof(RUL_DegradationFactors_t));

    return STATUS_OK;
}

/**
 * @brief Get module state
 */
Status_t RUL_GetState(const RUL_Context_t *pContext, RUL_State_t *pState)
{
    /* Validate input parameters */
    if ((pContext == NULL) || (pState == NULL))
    {
        return STATUS_ERROR_PARAM;
    }

    *pState = pContext->state;

    return STATUS_OK;
}

/**
 * @brief Reset RUL module
 */
Status_t RUL_Reset(RUL_Context_t *pContext)
{
    RUL_Config_t savedConfig;
    Status_t status;

    /* Validate context */
    status = RUL_ValidateContext(pContext);
    if (status != STATUS_OK)
    {
        return status;
    }

    /* Save configuration */
    (void)memcpy(&savedConfig, &pContext->config, sizeof(RUL_Config_t));

    /* Re-initialize with saved configuration */
    status = RUL_InitWithConfig(pContext, &savedConfig);

    return status;
}

/**
 * @brief Set nominal capacity
 */
Status_t RUL_SetNominalCapacity(RUL_Context_t *pContext, uint32_t nominalCapacity_mAh)
{
    Status_t status;

    /* Validate context */
    status = RUL_ValidateContext(pContext);
    if (status != STATUS_OK)
    {
        return status;
    }

    /* Validate capacity value */
    if (nominalCapacity_mAh == 0U)
    {
        return STATUS_ERROR_PARAM;
    }

    /* Update configuration and health metrics */
    pContext->config.nominalCapacity_mAh = nominalCapacity_mAh;
    pContext->health.nominalCapacity_mAh = nominalCapacity_mAh;

    /* Recalculate SoH based on current capacity */
    if (pContext->health.currentCapacity_mAh > 0U)
    {
        status = RUL_UpdateSoH(pContext, pContext->health.currentCapacity_mAh);
    }

    return status;
}

/**
 * @brief Set End of Life threshold
 */
Status_t RUL_SetEOLThreshold(RUL_Context_t *pContext, Percentage_t eolThreshold)
{
    Status_t status;

    /* Validate context */
    status = RUL_ValidateContext(pContext);
    if (status != STATUS_OK)
    {
        return status;
    }

    /* Validate threshold (must be between 0% and 100%) */
    if (eolThreshold > RUL_MAX_SOH_PERCENT)
    {
        return STATUS_ERROR_RANGE;
    }

    /* Update threshold */
    pContext->config.eolThreshold = eolThreshold;

    return STATUS_OK;
}

/**
 * @brief Update operating time
 */
Status_t RUL_UpdateOperatingTime(RUL_Context_t *pContext, uint32_t deltaTime_ms)
{
    Status_t status;
    uint32_t newOperatingTime_ms;
    uint32_t newOperatingTime_hours;

    /* Validate context */
    status = RUL_ValidateContext(pContext);
    if (status != STATUS_OK)
    {
        return status;
    }

    /* Calculate new operating time */
    newOperatingTime_ms = pContext->lastUpdateTime_ms + deltaTime_ms;
    newOperatingTime_hours = newOperatingTime_ms / MS_PER_HOUR;

    /* Update operating time */
    pContext->operatingTime_hours = newOperatingTime_hours;
    pContext->lastUpdateTime_ms = newOperatingTime_ms;

    /* Update calendar age (rough estimation) */
    pContext->calendarAge_days = pContext->operatingTime_hours / HOURS_PER_DAY;

    /* Recalculate degradation factors */
    RUL_CalculateDegradationFactors(pContext);

    return STATUS_OK;
}

/*============================================================================*/
/* PRIVATE FUNCTION IMPLEMENTATIONS                                           */
/*============================================================================*/

/**
 * @brief Validate RUL context
 */
static Status_t RUL_ValidateContext(const RUL_Context_t *pContext)
{
    if (pContext == NULL)
    {
        return STATUS_ERROR_PARAM;
    }

    if (pContext->state == RUL_STATE_UNINITIALIZED)
    {
        return STATUS_ERROR_NOT_INIT;
    }

    if (pContext->state == RUL_STATE_ERROR)
    {
        return STATUS_ERROR;
    }

    return STATUS_OK;
}

/**
 * @brief Validate RUL configuration
 */
static Status_t RUL_ValidateConfig(const RUL_Config_t *pConfig)
{
    if (pConfig == NULL)
    {
        return STATUS_ERROR_PARAM;
    }

    /* Validate nominal capacity */
    if (pConfig->nominalCapacity_mAh == 0U)
    {
        return STATUS_ERROR_PARAM;
    }

    /* Validate EOL threshold */
    if (pConfig->eolThreshold > RUL_MAX_SOH_PERCENT)
    {
        return STATUS_ERROR_RANGE;
    }

    /* Validate history sizes */
    if ((pConfig->tempHistorySize == 0U) ||
        (pConfig->tempHistorySize > RUL_TEMP_HISTORY_SIZE))
    {
        return STATUS_ERROR_RANGE;
    }

    if ((pConfig->cycleHistorySize == 0U) ||
        (pConfig->cycleHistorySize > RUL_CYCLE_HISTORY_SIZE))
    {
        return STATUS_ERROR_RANGE;
    }

    return STATUS_OK;
}

/**
 * @brief Calculate degradation factors
 */
static void RUL_CalculateDegradationFactors(RUL_Context_t *pContext)
{
    float cyclicDeg;
    float calendarDeg;
    float tempFactor;
    float totalDeg;

    /* Calculate cyclic degradation (based on total cycles) */
    cyclicDeg = ((float)pContext->cycleInfo.totalCycles * CYCLE_DEGRADATION_RATE) / 1000.0f;

    /* Calculate calendar degradation */
    if (pContext->config.enableCalendarAging)
    {
        calendarDeg = ((float)pContext->calendarAge_days * CALENDAR_AGING_RATE) / (float)DAYS_PER_YEAR;
    }
    else
    {
        calendarDeg = 0.0f;
    }

    /* Calculate temperature acceleration factor */
    if (pContext->config.enableTempCompensation)
    {
        tempFactor = RUL_CalculateTemperatureFactor(pContext->health.avgTemperature);
    }
    else
    {
        tempFactor = 1.0f;
    }

    /* Calculate total degradation rate (% per day) */
    totalDeg = (cyclicDeg + calendarDeg) * tempFactor;

    /* Update degradation factors */
    pContext->degradation.cyclicDegradation = cyclicDeg;
    pContext->degradation.calendarDegradation = calendarDeg;
    pContext->degradation.temperatureFactor = tempFactor;
    pContext->degradation.totalDegradationRate = totalDeg;
}

/**
 * @brief Calculate temperature acceleration factor
 */
static float RUL_CalculateTemperatureFactor(Temperature_t avgTemp)
{
    float tempDiff;
    float factor;

    /* Calculate temperature difference from reference (in °C) */
    tempDiff = ((float)avgTemp - (float)RUL_REFERENCE_TEMP) / 100.0f;

    /* Calculate acceleration factor (exponential for every 10°C) */
    if (tempDiff > 0.0f)
    {
        /* Temperature above reference accelerates degradation */
        factor = powf(TEMP_ACCELERATION_FACTOR, tempDiff / 10.0f);
    }
    else
    {
        /* Temperature at or below reference has minimal effect */
        factor = 1.0f;
    }

    return factor;
}

/**
 * @brief Get average temperature from history
 */
static float RUL_GetAverageTemperature(const RUL_Context_t *pContext)
{
    uint32_t sum = 0U;
    uint16_t count;
    uint16_t i;

    count = pContext->tempHistoryCount;

    if (count == 0U)
    {
        return (float)RUL_REFERENCE_TEMP;
    }

    /* Calculate sum of all temperature samples */
    for (i = 0U; i < count; i++)
    {
        sum += (uint32_t)pContext->tempHistory[i];
    }

    /* Return average */
    return (float)sum / (float)count;
}

/**
 * @brief Add temperature sample to history
 */
static void RUL_AddTemperatureSample(RUL_Context_t *pContext, Temperature_t temp)
{
    uint16_t index;

    index = pContext->tempHistoryIndex;

    /* Add sample to circular buffer */
    pContext->tempHistory[index] = temp;

    /* Increment index (wrap around) */
    index++;
    if (index >= pContext->config.tempHistorySize)
    {
        index = 0U;
    }
    pContext->tempHistoryIndex = index;

    /* Update count (saturate at buffer size) */
    if (pContext->tempHistoryCount < pContext->config.tempHistorySize)
    {
        pContext->tempHistoryCount++;
    }
}

/**
 * @brief Calculate remaining cycles until EOL
 */
static uint32_t RUL_CalculateRemainingCycles(const RUL_Context_t *pContext)
{
    float currentSoH_percent;
    float eolThreshold_percent;
    float remainingSoH_percent;
    float degradationPerCycle;
    uint32_t remainingCycles;

    /* Get current SoH as percentage (0-100) */
    currentSoH_percent = (float)pContext->health.soh / 100.0f;

    /* Get EOL threshold as percentage (0-100) */
    eolThreshold_percent = (float)pContext->config.eolThreshold / 100.0f;

    /* Calculate remaining SoH until EOL */
    remainingSoH_percent = currentSoH_percent - eolThreshold_percent;

    if (remainingSoH_percent <= 0.0f)
    {
        return 0U;
    }

    /* Calculate degradation per cycle */
    degradationPerCycle = pContext->degradation.cyclicDegradation;

    if (degradationPerCycle <= 0.0f)
    {
        /* If no cyclic degradation, estimate based on default rate */
        degradationPerCycle = CYCLE_DEGRADATION_RATE / 1000.0f;
    }

    /* Calculate remaining cycles */
    remainingCycles = (uint32_t)(remainingSoH_percent / degradationPerCycle);

    return remainingCycles;
}

/**
 * @brief Calculate remaining days until EOL
 */
static uint32_t RUL_CalculateRemainingDays(const RUL_Context_t *pContext)
{
    float currentSoH_percent;
    float eolThreshold_percent;
    float remainingSoH_percent;
    float totalDegradationRate;
    uint32_t remainingDays;

    /* Get current SoH as percentage (0-100) */
    currentSoH_percent = (float)pContext->health.soh / 100.0f;

    /* Get EOL threshold as percentage (0-100) */
    eolThreshold_percent = (float)pContext->config.eolThreshold / 100.0f;

    /* Calculate remaining SoH until EOL */
    remainingSoH_percent = currentSoH_percent - eolThreshold_percent;

    if (remainingSoH_percent <= 0.0f)
    {
        return 0U;
    }

    /* Get total degradation rate (% per day) */
    totalDegradationRate = pContext->degradation.totalDegradationRate;

    if (totalDegradationRate <= 0.0f)
    {
        /* If no degradation rate, estimate based on calendar aging only */
        totalDegradationRate = CALENDAR_AGING_RATE / (float)DAYS_PER_YEAR;
    }

    /* Calculate remaining days */
    remainingDays = (uint32_t)(remainingSoH_percent / totalDegradationRate);

    return remainingDays;
}

/**
 * @brief Calculate prediction confidence level
 */
static Percentage_t RUL_CalculateConfidence(const RUL_Context_t *pContext)
{
    Percentage_t confidence;
    uint32_t dataPoints;

    /* Base confidence on amount of collected data */
    dataPoints = pContext->updateCount;

    if (dataPoints == 0U)
    {
        confidence = MIN_CONFIDENCE_LEVEL;  /* 30% minimum */
    }
    else if (dataPoints < 100U)
    {
        /* Low confidence: 30-60% */
        confidence = MIN_CONFIDENCE_LEVEL + ((dataPoints * 3000U) / 100U);
    }
    else if (dataPoints < 1000U)
    {
        /* Medium confidence: 60-80% */
        confidence = 6000U + (((dataPoints - 100U) * 2000U) / 900U);
    }
    else
    {
        /* High confidence: 80-95% */
        confidence = 8000U + (((dataPoints - 1000U) * 1500U) / 9000U);

        /* Cap at 95% (never 100% certainty) */
        if (confidence > 9500U)
        {
            confidence = 9500U;
        }
    }

    return confidence;
}
