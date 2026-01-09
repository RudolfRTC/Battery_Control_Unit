/**
 * @file    bsp_i2c.c
 * @brief   I2C driver implementation for STM32F413
 * @author  Battery Control Unit Development Team
 * @date    2026-01-09
 * @version 1.0.0
 *
 * @note    MISRA C:2012 compliant
 * @note    Supports FRAM (CY15B256J) and TMP1075 temperature sensor
 *
 * @copyright Copyright (c) 2026
 */

/*============================================================================*/
/* INCLUDES                                                                   */
/*============================================================================*/
#include "bsp_i2c.h"
#include "app_config.h"
#include <string.h>

/*============================================================================*/
/* PRIVATE VARIABLES                                                          */
/*============================================================================*/

/** @brief I2C handle */
static I2C_HandleTypeDef hi2c2;

/** @brief I2C statistics */
static I2C_Statistics_t i2c_stats;

/** @brief Initialization flag */
static bool i2c_initialized = false;

/*============================================================================*/
/* PRIVATE FUNCTION PROTOTYPES                                                */
/*============================================================================*/

static Status_t i2c_wait_ready(uint32_t timeout_ms);

/*============================================================================*/
/* PUBLIC FUNCTIONS                                                           */
/*============================================================================*/

/**
 * @brief Initialize I2C interface
 */
Status_t BSP_I2C_Init(uint8_t instance, const I2C_Config_t *pConfig)
{
    Status_t status = STATUS_ERROR_PARAM;

    if ((instance == BSP_I2C_INSTANCE_2) && (pConfig != NULL))
    {
        /* Enable I2C clock */
        __HAL_RCC_I2C2_CLK_ENABLE();

        /* Configure I2C */
        hi2c2.Instance = I2C2;
        hi2c2.Init.ClockSpeed = pConfig->clockSpeed;
        hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
        hi2c2.Init.OwnAddress1 = (uint32_t)pConfig->ownAddress << 1;
        hi2c2.Init.AddressingMode = (pConfig->addrMode == I2C_ADDR_7BIT) ?
                                     I2C_ADDRESSINGMODE_7BIT : I2C_ADDRESSINGMODE_10BIT;
        hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
        hi2c2.Init.OwnAddress2 = 0;
        hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
        hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

        if (HAL_I2C_Init(&hi2c2) != HAL_OK)
        {
            status = STATUS_ERROR_HW_FAULT;
        }
        else
        {
            /* Reset statistics */
            (void)memset(&i2c_stats, 0, sizeof(i2c_stats));

            i2c_initialized = true;
            status = STATUS_OK;
        }
    }

    return status;
}

/**
 * @brief Write data to I2C device
 */
Status_t BSP_I2C_Write(uint8_t instance, uint8_t deviceAddr, uint16_t regAddr,
                       const uint8_t *pData, uint16_t length, uint32_t timeout_ms)
{
    Status_t status = STATUS_ERROR_PARAM;

    if ((instance == BSP_I2C_INSTANCE_2) && (pData != NULL) &&
        (length > 0U) && i2c_initialized)
    {
        HAL_StatusTypeDef hal_status;

        /* Wait for bus ready */
        status = i2c_wait_ready(timeout_ms);

        if (status == STATUS_OK)
        {
            /* Write with memory address */
            hal_status = HAL_I2C_Mem_Write(&hi2c2,
                                           (uint16_t)(deviceAddr << 1),
                                           regAddr,
                                           (regAddr > 255U) ? I2C_MEMADD_SIZE_16BIT : I2C_MEMADD_SIZE_8BIT,
                                           (uint8_t*)pData,
                                           length,
                                           timeout_ms);

            if (hal_status == HAL_OK)
            {
                i2c_stats.txCount += length;
                status = STATUS_OK;
            }
            else if (hal_status == HAL_TIMEOUT)
            {
                i2c_stats.timeoutCount++;
                status = STATUS_ERROR_TIMEOUT;
            }
            else
            {
                i2c_stats.errorCount++;
                status = STATUS_ERROR;
            }
        }
    }

    return status;
}

/**
 * @brief Read data from I2C device
 */
Status_t BSP_I2C_Read(uint8_t instance, uint8_t deviceAddr, uint16_t regAddr,
                      uint8_t *pData, uint16_t length, uint32_t timeout_ms)
{
    Status_t status = STATUS_ERROR_PARAM;

    if ((instance == BSP_I2C_INSTANCE_2) && (pData != NULL) &&
        (length > 0U) && i2c_initialized)
    {
        HAL_StatusTypeDef hal_status;

        /* Wait for bus ready */
        status = i2c_wait_ready(timeout_ms);

        if (status == STATUS_OK)
        {
            /* Read with memory address */
            hal_status = HAL_I2C_Mem_Read(&hi2c2,
                                          (uint16_t)(deviceAddr << 1),
                                          regAddr,
                                          (regAddr > 255U) ? I2C_MEMADD_SIZE_16BIT : I2C_MEMADD_SIZE_8BIT,
                                          pData,
                                          length,
                                          timeout_ms);

            if (hal_status == HAL_OK)
            {
                i2c_stats.rxCount += length;
                status = STATUS_OK;
            }
            else if (hal_status == HAL_TIMEOUT)
            {
                i2c_stats.timeoutCount++;
                status = STATUS_ERROR_TIMEOUT;
            }
            else
            {
                i2c_stats.errorCount++;
                status = STATUS_ERROR;
            }
        }
    }

    return status;
}

/**
 * @brief Write data without register address
 */
Status_t BSP_I2C_WriteRaw(uint8_t instance, uint8_t deviceAddr,
                          const uint8_t *pData, uint16_t length,
                          uint32_t timeout_ms)
{
    Status_t status = STATUS_ERROR_PARAM;

    if ((instance == BSP_I2C_INSTANCE_2) && (pData != NULL) &&
        (length > 0U) && i2c_initialized)
    {
        HAL_StatusTypeDef hal_status;

        /* Wait for bus ready */
        status = i2c_wait_ready(timeout_ms);

        if (status == STATUS_OK)
        {
            hal_status = HAL_I2C_Master_Transmit(&hi2c2,
                                                 (uint16_t)(deviceAddr << 1),
                                                 (uint8_t*)pData,
                                                 length,
                                                 timeout_ms);

            if (hal_status == HAL_OK)
            {
                i2c_stats.txCount += length;
                status = STATUS_OK;
            }
            else if (hal_status == HAL_TIMEOUT)
            {
                i2c_stats.timeoutCount++;
                status = STATUS_ERROR_TIMEOUT;
            }
            else
            {
                i2c_stats.errorCount++;
                status = STATUS_ERROR;
            }
        }
    }

    return status;
}

/**
 * @brief Read data without register address
 */
Status_t BSP_I2C_ReadRaw(uint8_t instance, uint8_t deviceAddr,
                         uint8_t *pData, uint16_t length,
                         uint32_t timeout_ms)
{
    Status_t status = STATUS_ERROR_PARAM;

    if ((instance == BSP_I2C_INSTANCE_2) && (pData != NULL) &&
        (length > 0U) && i2c_initialized)
    {
        HAL_StatusTypeDef hal_status;

        /* Wait for bus ready */
        status = i2c_wait_ready(timeout_ms);

        if (status == STATUS_OK)
        {
            hal_status = HAL_I2C_Master_Receive(&hi2c2,
                                                (uint16_t)(deviceAddr << 1),
                                                pData,
                                                length,
                                                timeout_ms);

            if (hal_status == HAL_OK)
            {
                i2c_stats.rxCount += length;
                status = STATUS_OK;
            }
            else if (hal_status == HAL_TIMEOUT)
            {
                i2c_stats.timeoutCount++;
                status = STATUS_ERROR_TIMEOUT;
            }
            else
            {
                i2c_stats.errorCount++;
                status = STATUS_ERROR;
            }
        }
    }

    return status;
}

/**
 * @brief Check if I2C device is ready
 */
Status_t BSP_I2C_IsDeviceReady(uint8_t instance, uint8_t deviceAddr,
                               uint8_t trials, uint32_t timeout_ms)
{
    Status_t status = STATUS_ERROR_PARAM;

    if ((instance == BSP_I2C_INSTANCE_2) && i2c_initialized)
    {
        HAL_StatusTypeDef hal_status;

        hal_status = HAL_I2C_IsDeviceReady(&hi2c2,
                                          (uint16_t)(deviceAddr << 1),
                                          trials,
                                          timeout_ms);

        if (hal_status == HAL_OK)
        {
            status = STATUS_OK;
        }
        else if (hal_status == HAL_TIMEOUT)
        {
            status = STATUS_ERROR_TIMEOUT;
        }
        else
        {
            status = STATUS_ERROR;
        }
    }

    return status;
}

/**
 * @brief Scan I2C bus for devices
 */
Status_t BSP_I2C_ScanBus(uint8_t instance, uint8_t *pDevices,
                         uint8_t maxDevices, uint8_t *pCount)
{
    Status_t status = STATUS_ERROR_PARAM;

    if ((instance == BSP_I2C_INSTANCE_2) && (pDevices != NULL) &&
        (pCount != NULL) && i2c_initialized)
    {
        uint8_t addr;
        uint8_t found = 0U;

        *pCount = 0U;

        /* Scan addresses 0x08 to 0x77 (7-bit addressing) */
        for (addr = 0x08U; (addr < 0x78U) && (found < maxDevices); addr++)
        {
            if (HAL_I2C_IsDeviceReady(&hi2c2, (uint16_t)(addr << 1), 1, 10) == HAL_OK)
            {
                pDevices[found] = addr;
                found++;
            }
        }

        *pCount = found;
        status = STATUS_OK;
    }

    return status;
}

/**
 * @brief Get I2C statistics
 */
Status_t BSP_I2C_GetStatistics(uint8_t instance, I2C_Statistics_t *pStats)
{
    Status_t status = STATUS_ERROR_PARAM;

    if ((instance == BSP_I2C_INSTANCE_2) && (pStats != NULL))
    {
        *pStats = i2c_stats;
        status = STATUS_OK;
    }

    return status;
}

/**
 * @brief Reset I2C statistics
 */
Status_t BSP_I2C_ResetStatistics(uint8_t instance)
{
    Status_t status = STATUS_ERROR_PARAM;

    if (instance == BSP_I2C_INSTANCE_2)
    {
        (void)memset(&i2c_stats, 0, sizeof(i2c_stats));
        status = STATUS_OK;
    }

    return status;
}

/**
 * @brief Reset I2C bus
 */
Status_t BSP_I2C_ResetBus(uint8_t instance)
{
    Status_t status = STATUS_ERROR_PARAM;

    if ((instance == BSP_I2C_INSTANCE_2) && i2c_initialized)
    {
        /* Reset I2C peripheral */
        __HAL_I2C_DISABLE(&hi2c2);
        HAL_Delay(1);
        __HAL_I2C_ENABLE(&hi2c2);

        i2c_stats.busErrors++;
        status = STATUS_OK;
    }

    return status;
}

/**
 * @brief De-initialize I2C interface
 */
Status_t BSP_I2C_DeInit(uint8_t instance)
{
    Status_t status = STATUS_ERROR_PARAM;

    if ((instance == BSP_I2C_INSTANCE_2) && i2c_initialized)
    {
        HAL_I2C_DeInit(&hi2c2);
        i2c_initialized = false;
        status = STATUS_OK;
    }

    return status;
}

/*============================================================================*/
/* PRIVATE FUNCTIONS                                                          */
/*============================================================================*/

/**
 * @brief Wait for I2C bus to be ready
 */
static Status_t i2c_wait_ready(uint32_t timeout_ms)
{
    Status_t status = STATUS_OK;
    uint32_t start_time = HAL_GetTick();

    while (HAL_I2C_GetState(&hi2c2) != HAL_I2C_STATE_READY)
    {
        if ((HAL_GetTick() - start_time) > timeout_ms)
        {
            status = STATUS_ERROR_TIMEOUT;
            break;
        }
    }

    return status;
}

/*============================================================================*/
/* END OF FILE                                                                */
/*============================================================================*/
