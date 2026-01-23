/**
 * @file    bsp_spi.c
 * @brief   SPI bus abstraction layer implementation
 * @author  Battery Control Unit Development Team
 * @date    2026-01-10
 * @version 1.0.0
 *
 * @note    MISRA C:2012 compliant
 * @note    Supports SPI4 for ISO-SPI communication
 *
 * @copyright Copyright (c) 2026
 */

/*============================================================================*/
/* INCLUDES                                                                   */
/*============================================================================*/
#include "bsp_spi.h"
#include "bsp_gpio.h"
#include "stm32f4xx_hal.h"
#include <string.h>

/*============================================================================*/
/* PRIVATE CONSTANTS                                                          */
/*============================================================================*/

/** @brief Number of SPI instances */
#define SPI_NUM_INSTANCES (1U)

/*============================================================================*/
/* EXTERNAL VARIABLES                                                         */
/*============================================================================*/

/** @brief SPI4 handle from main.c (STM32CubeMX generated) */
extern SPI_HandleTypeDef hspi4;

/*============================================================================*/
/* PRIVATE VARIABLES                                                          */
/*============================================================================*/

/** @brief SPI statistics */
static SPI_Statistics_t spi_stats[SPI_NUM_INSTANCES];

/** @brief Initialization flags */
static bool spi_initialized[SPI_NUM_INSTANCES] = {false};

/*============================================================================*/
/* PRIVATE FUNCTION PROTOTYPES                                                */
/*============================================================================*/

static Status_t bsp_spi_get_handle(uint8_t instance, SPI_HandleTypeDef **ppHandle);

/*============================================================================*/
/* PUBLIC FUNCTIONS                                                           */
/*============================================================================*/

/**
 * @brief Initialize SPI interface
 */
Status_t BSP_SPI_Init(uint8_t instance, const SPI_Config_t *pConfig)
{
    Status_t status = STATUS_OK;
    SPI_HandleTypeDef *pHandle = NULL;

    /* Parameter validation */
    if (pConfig == NULL)
    {
        status = STATUS_ERROR_PARAM;
    }
    else if (instance != BSP_SPI_INSTANCE_4)
    {
        status = STATUS_ERROR_PARAM;
    }
    else
    {
        /* Get SPI handle */
        status = bsp_spi_get_handle(instance, &pHandle);

        if (status == STATUS_OK)
        {
            /* If already initialized, deinitialize first */
            if (spi_initialized[0])
            {
                (void)HAL_SPI_DeInit(pHandle);
                spi_initialized[0] = false;
            }

            /* Enable SPI4 clock */
            __HAL_RCC_SPI4_CLK_ENABLE();

            /* Configure SPI4 */
            pHandle->Instance = SPI4;
            pHandle->Init.Mode = SPI_MODE_MASTER;
            pHandle->Init.Direction = SPI_DIRECTION_2LINES;

            /* Data size */
            if (pConfig->dataSize == BSP_SPI_DATASIZE_16BIT)
            {
                pHandle->Init.DataSize = SPI_DATASIZE_16BIT;
            }
            else
            {
                pHandle->Init.DataSize = SPI_DATASIZE_8BIT;
            }

            /* Clock polarity and phase */
            switch (pConfig->mode)
            {
                case SPI_MODE_0:
                    pHandle->Init.CLKPolarity = SPI_POLARITY_LOW;
                    pHandle->Init.CLKPhase = SPI_PHASE_1EDGE;
                    break;
                case SPI_MODE_1:
                    pHandle->Init.CLKPolarity = SPI_POLARITY_LOW;
                    pHandle->Init.CLKPhase = SPI_PHASE_2EDGE;
                    break;
                case SPI_MODE_2:
                    pHandle->Init.CLKPolarity = SPI_POLARITY_HIGH;
                    pHandle->Init.CLKPhase = SPI_PHASE_1EDGE;
                    break;
                case SPI_MODE_3:
                    pHandle->Init.CLKPolarity = SPI_POLARITY_HIGH;
                    pHandle->Init.CLKPhase = SPI_PHASE_2EDGE;
                    break;
                default:
                    status = STATUS_ERROR_PARAM;
                    break;
            }

            if (status == STATUS_OK)
            {
                /* Chip select management */
                pHandle->Init.NSS = SPI_NSS_SOFT;

                /* Bit order */
                if (pConfig->bitOrder == SPI_BITORDER_LSB_FIRST)
                {
                    pHandle->Init.FirstBit = SPI_FIRSTBIT_LSB;
                }
                else
                {
                    pHandle->Init.FirstBit = SPI_FIRSTBIT_MSB;
                }

                /* Calculate prescaler based on clock speed */
                /* APB2 = 100 MHz, SPI4 is on APB2 */
                uint32_t apb2_freq = HAL_RCC_GetPCLK2Freq();

                /* Validate clock frequency and speed */
                if ((apb2_freq == 0U) || (pConfig->clockSpeed == 0U))
                {
                    status = STATUS_ERROR_HW_FAULT;
                }

                if (status == STATUS_OK)
                {
                    uint32_t prescaler_value = apb2_freq / pConfig->clockSpeed;

                    if (prescaler_value <= 2U)
                {
                    pHandle->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
                }
                else if (prescaler_value <= 4U)
                {
                    pHandle->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
                }
                else if (prescaler_value <= 8U)
                {
                    pHandle->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
                }
                else if (prescaler_value <= 16U)
                {
                    pHandle->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
                }
                else if (prescaler_value <= 32U)
                {
                    pHandle->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
                }
                else if (prescaler_value <= 64U)
                {
                    pHandle->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
                }
                else if (prescaler_value <= 128U)
                {
                    pHandle->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
                }
                else
                {
                    pHandle->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
                }

                    pHandle->Init.TIMode = SPI_TIMODE_DISABLE;
                    pHandle->Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;

                    /* Initialize SPI */
                    if (HAL_SPI_Init(pHandle) != HAL_OK)
                    {
                        status = STATUS_ERROR_HW_FAULT;
                    }
                    else
                    {
                        /* Reset statistics */
                        (void)memset(&spi_stats[0], 0, sizeof(SPI_Statistics_t));
                        spi_initialized[0] = true;

                        /* Deselect chip select initially */
                        (void)BSP_SPI_ChipSelect(instance, false);
                    }
                }
            }
        }
    }

    return status;
}

/**
 * @brief De-initialize SPI interface
 */
Status_t BSP_SPI_DeInit(uint8_t instance)
{
    Status_t status = STATUS_OK;
    SPI_HandleTypeDef *pHandle = NULL;

    /* Parameter validation */
    if (instance != BSP_SPI_INSTANCE_4)
    {
        status = STATUS_ERROR_PARAM;
    }
    else if (!spi_initialized[0])
    {
        status = STATUS_ERROR_NOT_INIT;
    }
    else
    {
        status = bsp_spi_get_handle(instance, &pHandle);

        if (status == STATUS_OK)
        {
            /* De-initialize SPI */
            if (HAL_SPI_DeInit(pHandle) != HAL_OK)
            {
                status = STATUS_ERROR_HW_FAULT;
            }
            else
            {
                /* Disable clock */
                __HAL_RCC_SPI4_CLK_DISABLE();
                spi_initialized[0] = false;
            }
        }
    }

    return status;
}

/**
 * @brief Transmit data over SPI
 */
Status_t BSP_SPI_Transmit(uint8_t instance, const uint8_t *pData,
                          uint16_t length, uint32_t timeout_ms)
{
    Status_t status = STATUS_OK;
    SPI_HandleTypeDef *pHandle = NULL;
    HAL_StatusTypeDef hal_status;

    /* Parameter validation */
    if ((pData == NULL) || (length == 0U) || (length > BSP_SPI_MAX_TRANSFER_SIZE))
    {
        status = STATUS_ERROR_PARAM;
    }
    else if (instance != BSP_SPI_INSTANCE_4)
    {
        status = STATUS_ERROR_PARAM;
    }
    else if (!spi_initialized[0])
    {
        status = STATUS_ERROR_NOT_INIT;
    }
    else
    {
        status = bsp_spi_get_handle(instance, &pHandle);

        if (status == STATUS_OK)
        {
            hal_status = HAL_SPI_Transmit(pHandle, (uint8_t *)pData, length, timeout_ms);

            if (hal_status == HAL_OK)
            {
                spi_stats[0].txCount += length;
            }
            else if (hal_status == HAL_TIMEOUT)
            {
                spi_stats[0].timeoutCount++;
                status = STATUS_ERROR_TIMEOUT;
            }
            else
            {
                spi_stats[0].errorCount++;
                status = STATUS_ERROR;
            }
        }
    }

    return status;
}

/**
 * @brief Receive data over SPI
 */
Status_t BSP_SPI_Receive(uint8_t instance, uint8_t *pData,
                         uint16_t length, uint32_t timeout_ms)
{
    Status_t status = STATUS_OK;
    SPI_HandleTypeDef *pHandle = NULL;
    HAL_StatusTypeDef hal_status;

    /* Parameter validation */
    if ((pData == NULL) || (length == 0U) || (length > BSP_SPI_MAX_TRANSFER_SIZE))
    {
        status = STATUS_ERROR_PARAM;
    }
    else if (instance != BSP_SPI_INSTANCE_4)
    {
        status = STATUS_ERROR_PARAM;
    }
    else if (!spi_initialized[0])
    {
        status = STATUS_ERROR_NOT_INIT;
    }
    else
    {
        status = bsp_spi_get_handle(instance, &pHandle);

        if (status == STATUS_OK)
        {
            hal_status = HAL_SPI_Receive(pHandle, pData, length, timeout_ms);

            if (hal_status == HAL_OK)
            {
                spi_stats[0].rxCount += length;
            }
            else if (hal_status == HAL_TIMEOUT)
            {
                spi_stats[0].timeoutCount++;
                status = STATUS_ERROR_TIMEOUT;
            }
            else
            {
                spi_stats[0].errorCount++;
                status = STATUS_ERROR;
            }
        }
    }

    return status;
}

/**
 * @brief Transmit and receive data over SPI
 */
Status_t BSP_SPI_TransmitReceive(uint8_t instance, const uint8_t *pTxData,
                                 uint8_t *pRxData, uint16_t length,
                                 uint32_t timeout_ms)
{
    Status_t status = STATUS_OK;
    SPI_HandleTypeDef *pHandle = NULL;
    HAL_StatusTypeDef hal_status;

    /* Parameter validation */
    if ((pTxData == NULL) || (pRxData == NULL) || (length == 0U) ||
        (length > BSP_SPI_MAX_TRANSFER_SIZE))
    {
        status = STATUS_ERROR_PARAM;
    }
    else if (instance != BSP_SPI_INSTANCE_4)
    {
        status = STATUS_ERROR_PARAM;
    }
    else if (!spi_initialized[0])
    {
        status = STATUS_ERROR_NOT_INIT;
    }
    else
    {
        status = bsp_spi_get_handle(instance, &pHandle);

        if (status == STATUS_OK)
        {
            hal_status = HAL_SPI_TransmitReceive(pHandle, (uint8_t *)pTxData,
                                                 pRxData, length, timeout_ms);

            if (hal_status == HAL_OK)
            {
                spi_stats[0].txCount += length;
                spi_stats[0].rxCount += length;
            }
            else if (hal_status == HAL_TIMEOUT)
            {
                spi_stats[0].timeoutCount++;
                status = STATUS_ERROR_TIMEOUT;
            }
            else
            {
                spi_stats[0].errorCount++;
                status = STATUS_ERROR;
            }
        }
    }

    return status;
}

/**
 * @brief Control SPI chip select
 */
Status_t BSP_SPI_ChipSelect(uint8_t instance, bool select)
{
    Status_t status = STATUS_OK;

    /* Parameter validation */
    if (instance != BSP_SPI_INSTANCE_4)
    {
        status = STATUS_ERROR_PARAM;
    }
    else
    {
        /* Control CS pin (PE4) */
        if (select)
        {
            HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_RESET);  /* CS low */
        }
        else
        {
            HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_SET);    /* CS high */
        }
    }

    return status;
}

/**
 * @brief Get SPI bus state
 */
Status_t BSP_SPI_GetState(uint8_t instance, SPI_State_t *pState)
{
    Status_t status = STATUS_OK;
    SPI_HandleTypeDef *pHandle = NULL;

    /* Parameter validation */
    if (pState == NULL)
    {
        status = STATUS_ERROR_PARAM;
    }
    else if (instance != BSP_SPI_INSTANCE_4)
    {
        status = STATUS_ERROR_PARAM;
    }
    else if (!spi_initialized[0])
    {
        *pState = SPI_STATE_UNINITIALIZED;
    }
    else
    {
        status = bsp_spi_get_handle(instance, &pHandle);

        if (status == STATUS_OK)
        {
            HAL_SPI_StateTypeDef hal_state = HAL_SPI_GetState(pHandle);

            switch (hal_state)
            {
                case HAL_SPI_STATE_READY:
                    *pState = SPI_STATE_READY;
                    break;
                case HAL_SPI_STATE_BUSY:
                case HAL_SPI_STATE_BUSY_TX:
                case HAL_SPI_STATE_BUSY_RX:
                case HAL_SPI_STATE_BUSY_TX_RX:
                    *pState = SPI_STATE_BUSY;
                    break;
                case HAL_SPI_STATE_ERROR:
                    *pState = SPI_STATE_ERROR;
                    break;
                default:
                    *pState = SPI_STATE_UNINITIALIZED;
                    break;
            }
        }
    }

    return status;
}

/**
 * @brief Get SPI statistics
 */
Status_t BSP_SPI_GetStatistics(uint8_t instance, SPI_Statistics_t *pStats)
{
    Status_t status = STATUS_OK;

    /* Parameter validation */
    if (pStats == NULL)
    {
        status = STATUS_ERROR_PARAM;
    }
    else if (instance != BSP_SPI_INSTANCE_4)
    {
        status = STATUS_ERROR_PARAM;
    }
    else if (!spi_initialized[0])
    {
        status = STATUS_ERROR_NOT_INIT;
    }
    else
    {
        (void)memcpy(pStats, &spi_stats[0], sizeof(SPI_Statistics_t));
    }

    return status;
}

/**
 * @brief Reset SPI statistics
 */
Status_t BSP_SPI_ResetStatistics(uint8_t instance)
{
    Status_t status = STATUS_OK;

    /* Parameter validation */
    if (instance != BSP_SPI_INSTANCE_4)
    {
        status = STATUS_ERROR_PARAM;
    }
    else if (!spi_initialized[0])
    {
        status = STATUS_ERROR_NOT_INIT;
    }
    else
    {
        (void)memset(&spi_stats[0], 0, sizeof(SPI_Statistics_t));
    }

    return status;
}

/**
 * @brief Enable/disable ISO-SPI interface
 */
Status_t BSP_SPI_EnableISOSPI(bool enable)
{
    /* Control ISOSPI_EN pin (PD6) */
    if (enable)
    {
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, GPIO_PIN_SET);  /* Enable ISO-SPI */
    }
    else
    {
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, GPIO_PIN_RESET); /* Disable ISO-SPI */
    }

    return STATUS_OK;
}

/*============================================================================*/
/* PRIVATE FUNCTIONS                                                          */
/*============================================================================*/

/**
 * @brief Get SPI handle for instance
 */
static Status_t bsp_spi_get_handle(uint8_t instance, SPI_HandleTypeDef **ppHandle)
{
    Status_t status = STATUS_OK;

    if (ppHandle == NULL)
    {
        status = STATUS_ERROR_PARAM;
    }
    else if (instance == BSP_SPI_INSTANCE_4)
    {
        *ppHandle = &hspi4;
    }
    else
    {
        status = STATUS_ERROR_PARAM;
    }

    return status;
}

/* Note: HAL_SPI_MspInit and HAL_SPI_MspDeInit are defined in stm32f4xx_hal_msp.c
 * with DMA configuration. Do not duplicate here to avoid linker errors.
 */
