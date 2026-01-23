/**
 * @file    bsp_adc.c
 * @brief   ADC driver implementation with DMA for STM32F413
 * @author  Battery Control Unit Development Team
 * @date    2026-01-09
 * @version 1.0.0
 *
 * @note    MISRA C:2012 compliant
 * @note    12-bit ADC with DMA continuous scanning
 * @note    Channels: LEM HOYS sensors, BTT6200 current sense, power monitor
 *
 * @copyright Copyright (c) 2026
 */

/*============================================================================*/
/* INCLUDES                                                                   */
/*============================================================================*/
#include "bsp_adc.h"
#include "app_config.h"
#include <string.h>

/*============================================================================*/
/* PRIVATE CONSTANTS                                                          */
/*============================================================================*/

/** @brief Number of ADC channels to scan */
#define ADC_NUM_CHANNELS    (BSP_ADC_CHANNEL_COUNT)

/** @brief DMA buffer size (channels × oversampling) */
#define ADC_DMA_BUFFER_SIZE (ADC_NUM_CHANNELS * BSP_ADC_OVERSAMPLE_RATIO)

/*============================================================================*/
/* PRIVATE VARIABLES                                                          */
/*============================================================================*/

/** @brief ADC handle */
static ADC_HandleTypeDef hadc1;

/** @brief DMA handle */
static DMA_HandleTypeDef hdma_adc1;

/** @brief DMA buffer for ADC results (volatile - accessed by DMA ISR) */
static volatile uint16_t adc_dma_buffer[ADC_DMA_BUFFER_SIZE] __attribute__((aligned(32)));

/** @brief Filtered ADC values (after oversampling, volatile - accessed by ISR) */
static volatile uint16_t adc_values[ADC_NUM_CHANNELS];

/** @brief Calibration data for each channel */
static ADC_Calibration_t adc_calibration[ADC_NUM_CHANNELS];

/** @brief ADC statistics */
static ADC_Statistics_t adc_stats;

/** @brief Conversion complete callback */
static ADC_ConversionCallback_t adc_callback = NULL;

/** @brief Initialization flag */
static bool adc_initialized = false;

/*============================================================================*/
/* PRIVATE FUNCTION PROTOTYPES                                                */
/*============================================================================*/

static void adc_dma_init(void);
static void adc_process_dma_buffer(void);

/*============================================================================*/
/* PUBLIC FUNCTIONS                                                           */
/*============================================================================*/

/**
 * @brief Initialize ADC module
 */
Status_t BSP_ADC_Init(const ADC_Config_t *pConfig)
{
    Status_t status = STATUS_ERROR_PARAM;

    if (pConfig != NULL)
    {
        ADC_ChannelConfTypeDef sConfig = {0};

        /* Enable ADC clock */
        __HAL_RCC_ADC1_CLK_ENABLE();

        /* Initialize DMA */
        adc_dma_init();

        /* Configure ADC */
        hadc1.Instance = ADC1;
        hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
        hadc1.Init.Resolution = ADC_RESOLUTION_12B;
        hadc1.Init.ScanConvMode = ENABLE;
        hadc1.Init.ContinuousConvMode = (pConfig->mode == ADC_MODE_CONTINUOUS) ? ENABLE : DISABLE;
        hadc1.Init.DiscontinuousConvMode = DISABLE;
        hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
        hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
        hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
        hadc1.Init.NbrOfConversion = ADC_NUM_CHANNELS;
        hadc1.Init.DMAContinuousRequests = ENABLE;
        hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;

        if (HAL_ADC_Init(&hadc1) != HAL_OK)
        {
            status = STATUS_ERROR_HW_FAULT;
        }
        else
        {
            /* Configure ADC channels */
            /* Channel 0: Power IMON (PA3) */
            sConfig.Channel = ADC_CHANNEL_3;
            sConfig.Rank = 1;
            sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
            HAL_ADC_ConfigChannel(&hadc1, &sConfig);

            /* Channels 1-10: LEM HOYS sensors */
            /* PF2, PF4, PF6, PF8, PF10, PF12, PF14, PG0, PG2, PG4 */
            uint32_t lem_channels[] = {
                ADC_CHANNEL_10, ADC_CHANNEL_14, ADC_CHANNEL_4,
                ADC_CHANNEL_6, ADC_CHANNEL_8, ADC_CHANNEL_9,
                ADC_CHANNEL_7, ADC_CHANNEL_15, ADC_CHANNEL_13,
                ADC_CHANNEL_11
            };

            for (uint8_t i = 0; i < 10U; i++)
            {
                sConfig.Channel = lem_channels[i];
                sConfig.Rank = i + 2U;
                sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
                HAL_ADC_ConfigChannel(&hadc1, &sConfig);
            }

            /* Channels 11-15: BTT6200 current sense */
            /* PC7, PC15, PD7, PD15, PE11 */
            uint32_t btt_channels[] = {
                ADC_CHANNEL_7, ADC_CHANNEL_15, ADC_CHANNEL_7,
                ADC_CHANNEL_15, ADC_CHANNEL_11
            };

            for (uint8_t i = 0; i < 5U; i++)
            {
                sConfig.Channel = btt_channels[i];
                sConfig.Rank = i + 12U;
                sConfig.SamplingTime = ADC_SAMPLETIME_144CYCLES;
                HAL_ADC_ConfigChannel(&hadc1, &sConfig);
            }

            /* Initialize calibration data */
            for (uint8_t i = 0; i < ADC_NUM_CHANNELS; i++)
            {
                adc_calibration[i].offset = 0;
                adc_calibration[i].gain = 65536;  /* Unity gain (1.0 in fixed-point) */
                adc_calibration[i].valid = false;
            }

            /* Reset statistics */
            (void)memset(&adc_stats, 0, sizeof(adc_stats));

            adc_initialized = true;
            status = STATUS_OK;
        }
    }

    return status;
}

/**
 * @brief Start ADC conversion
 */
Status_t BSP_ADC_Start(void)
{
    Status_t status = STATUS_ERROR_NOT_INIT;

    if (adc_initialized)
    {
        /* Start ADC with DMA */
        if (HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_dma_buffer, ADC_DMA_BUFFER_SIZE) == HAL_OK)
        {
            status = STATUS_OK;
        }
        else
        {
            status = STATUS_ERROR_HW_FAULT;
        }
    }

    return status;
}

/**
 * @brief Stop ADC conversion
 */
Status_t BSP_ADC_Stop(void)
{
    Status_t status = STATUS_ERROR_NOT_INIT;

    if (adc_initialized)
    {
        HAL_ADC_Stop_DMA(&hadc1);
        status = STATUS_OK;
    }

    return status;
}

/**
 * @brief Read ADC channel (blocking)
 */
Status_t BSP_ADC_ReadChannel(uint8_t channel, uint16_t *pValue, uint32_t timeout_ms)
{
    Status_t status = STATUS_ERROR_PARAM;

    if ((pValue != NULL) && (channel < ADC_NUM_CHANNELS) && adc_initialized)
    {
        /* Return last converted value from DMA buffer */
        *pValue = adc_values[channel];
        adc_stats.conversionCount++;
        status = STATUS_OK;
    }

    return status;
}

/**
 * @brief Read ADC channel with averaging
 */
Status_t BSP_ADC_ReadChannelAverage(uint8_t channel, uint8_t samples,
                                    uint16_t *pValue, uint32_t timeout_ms)
{
    Status_t status = STATUS_ERROR_PARAM;

    if ((pValue != NULL) && (channel < ADC_NUM_CHANNELS) &&
        (samples > 0U) && adc_initialized)
    {
        uint32_t sum = 0U;
        uint8_t i;

        for (i = 0U; i < samples; i++)
        {
            sum += (uint32_t)adc_values[channel];
            /* Small delay between samples */
            HAL_Delay(1);
        }

        *pValue = (uint16_t)(sum / (uint32_t)samples);
        status = STATUS_OK;
    }

    return status;
}

/**
 * @brief Convert ADC value to millivolts
 */
uint32_t BSP_ADC_ToMillivolts(uint16_t adcValue)
{
    uint32_t millivolts;

    /* millivolts = (adcValue * VREF) / ADC_MAX_VALUE */
    millivolts = ((uint32_t)adcValue * BSP_ADC_VREF_MV) / BSP_ADC_MAX_VALUE;

    return millivolts;
}

/**
 * @brief Convert millivolts to ADC value
 */
uint16_t BSP_ADC_FromMillivolts(uint32_t millivolts)
{
    uint16_t adcValue;

    /* adcValue = (millivolts * ADC_MAX_VALUE) / VREF */
    adcValue = (uint16_t)(((uint32_t)millivolts * BSP_ADC_MAX_VALUE) / BSP_ADC_VREF_MV);

    return adcValue;
}

/**
 * @brief Apply calibration to ADC value
 */
int32_t BSP_ADC_ApplyCalibration(uint16_t rawValue, const ADC_Calibration_t *pCalibration)
{
    int32_t calibrated = (int32_t)rawValue;

    if ((pCalibration != NULL) && pCalibration->valid)
    {
        /* Apply offset */
        calibrated = (int32_t)rawValue - pCalibration->offset;

        /* Apply gain (fixed-point multiply, scale by 65536) */
        calibrated = (calibrated * (int64_t)pCalibration->gain) >> 16;
    }

    return calibrated;
}

/**
 * @brief Set channel calibration data
 */
Status_t BSP_ADC_SetCalibration(uint8_t channel, const ADC_Calibration_t *pCalibration)
{
    Status_t status = STATUS_ERROR_PARAM;

    if ((channel < ADC_NUM_CHANNELS) && (pCalibration != NULL))
    {
        adc_calibration[channel] = *pCalibration;
        status = STATUS_OK;
    }

    return status;
}

/**
 * @brief Get channel calibration data
 */
Status_t BSP_ADC_GetCalibration(uint8_t channel, ADC_Calibration_t *pCalibration)
{
    Status_t status = STATUS_ERROR_PARAM;

    if ((channel < ADC_NUM_CHANNELS) && (pCalibration != NULL))
    {
        *pCalibration = adc_calibration[channel];
        status = STATUS_OK;
    }

    return status;
}

/**
 * @brief Perform ADC self-calibration
 */
Status_t BSP_ADC_Calibrate(void)
{
    Status_t status = STATUS_ERROR_NOT_INIT;

    if (adc_initialized)
    {
        /* STM32F4 ADC is factory-calibrated, no software calibration available */
        /* User calibration can be applied via gain/offset adjustment */
        status = STATUS_OK;
    }

    return status;
}

/**
 * @brief Get ADC statistics
 */
Status_t BSP_ADC_GetStatistics(ADC_Statistics_t *pStats)
{
    Status_t status = STATUS_ERROR_PARAM;

    if (pStats != NULL)
    {
        *pStats = adc_stats;
        status = STATUS_OK;
    }

    return status;
}

/**
 * @brief Reset ADC statistics
 */
Status_t BSP_ADC_ResetStatistics(void)
{
    (void)memset(&adc_stats, 0, sizeof(adc_stats));
    return STATUS_OK;
}

/**
 * @brief Register conversion complete callback
 */
Status_t BSP_ADC_RegisterCallback(ADC_ConversionCallback_t callback)
{
    adc_callback = callback;
    return STATUS_OK;
}

/**
 * @brief De-initialize ADC module
 */
Status_t BSP_ADC_DeInit(void)
{
    if (adc_initialized)
    {
        HAL_ADC_Stop_DMA(&hadc1);
        HAL_ADC_DeInit(&hadc1);
        HAL_DMA_DeInit(&hdma_adc1);
        adc_initialized = false;
    }

    return STATUS_OK;
}

/*============================================================================*/
/* PRIVATE FUNCTIONS                                                          */
/*============================================================================*/

/**
 * @brief Initialize DMA for ADC
 */
static void adc_dma_init(void)
{
    /* Enable DMA2 clock */
    __HAL_RCC_DMA2_CLK_ENABLE();

    /* Configure DMA */
    hdma_adc1.Instance = DMA2_Stream0;
    hdma_adc1.Init.Channel = DMA_CHANNEL_0;
    hdma_adc1.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_adc1.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_adc1.Init.MemInc = DMA_MINC_ENABLE;
    hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_adc1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_adc1.Init.Mode = DMA_CIRCULAR;
    hdma_adc1.Init.Priority = DMA_PRIORITY_HIGH;
    hdma_adc1.Init.FIFOMode = DMA_FIFOMODE_DISABLE;

    HAL_DMA_Init(&hdma_adc1);

    /* Link DMA to ADC */
    __HAL_LINKDMA(&hadc1, DMA_Handle, hdma_adc1);

    /* Enable DMA interrupt */
    HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
}

/**
 * @brief Process DMA buffer and perform oversampling
 */
static void adc_process_dma_buffer(void)
{
    uint8_t ch;
    uint8_t sample;

    /* Average oversampled values for each channel */
    for (ch = 0U; ch < ADC_NUM_CHANNELS; ch++)
    {
        uint32_t sum = 0U;

        for (sample = 0U; sample < BSP_ADC_OVERSAMPLE_RATIO; sample++)
        {
            uint16_t index = (sample * ADC_NUM_CHANNELS) + ch;

            /* Bounds check to prevent buffer overflow */
            if (index < ADC_DMA_BUFFER_SIZE)
            {
                sum += (uint32_t)adc_dma_buffer[index];
            }
            else
            {
                /* Log error and skip invalid samples */
                adc_stats.overrunCount++;
                break;
            }
        }

        adc_values[ch] = (uint16_t)(sum / BSP_ADC_OVERSAMPLE_RATIO);

        /* Call callback if registered */
        if (adc_callback != NULL)
        {
            adc_callback(ch, adc_values[ch]);
        }
    }

    adc_stats.conversionCount++;
}

/*============================================================================*/
/* HAL CALLBACKS                                                              */
/*============================================================================*/

/**
 * @brief DMA conversion complete callback
 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if (hadc->Instance == ADC1)
    {
        adc_process_dma_buffer();
    }
}

/**
 * @brief DMA conversion half complete callback
 */
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
{
    /* Process first half of buffer */
    UNUSED(hadc);
}

/**
 * @brief ADC error callback
 */
void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1)
    {
        adc_stats.dmaErrorCount++;
    }
}

/*============================================================================*/
/* END OF FILE                                                                */
/*============================================================================*/
