/**
 * @file    ltc6811.c
 * @brief   LTC6811 Battery Stack Monitor driver implementation
 * @author  Battery Control Unit Development Team
 * @date    2026-01-10
 * @version 1.0.0
 *
 * @note    MISRA C:2012 compliant
 * @note    Communication via ISO-SPI (SPI4)
 *
 * @copyright Copyright (c) 2026
 */

/*============================================================================*/
/* INCLUDES                                                                   */
/*============================================================================*/
#include "ltc6811.h"
#include "bsp_spi.h"
#include "app_config.h"
#include <string.h>

/*============================================================================*/
/* PRIVATE CONSTANTS                                                          */
/*============================================================================*/

/** @brief LTC6811 Commands */
#define LTC6811_CMD_WRCFGA      (0x0001U)  /**< Write configuration register group A */
#define LTC6811_CMD_WRCFGB      (0x0024U)  /**< Write configuration register group B */
#define LTC6811_CMD_RDCFGA      (0x0002U)  /**< Read configuration register group A */
#define LTC6811_CMD_RDCFGB      (0x0026U)  /**< Read configuration register group B */

#define LTC6811_CMD_RDCVA       (0x0004U)  /**< Read cell voltage register group A */
#define LTC6811_CMD_RDCVB       (0x0006U)  /**< Read cell voltage register group B */
#define LTC6811_CMD_RDCVC       (0x0008U)  /**< Read cell voltage register group C */
#define LTC6811_CMD_RDCVD       (0x000AU)  /**< Read cell voltage register group D */

#define LTC6811_CMD_RDAUXA      (0x000CU)  /**< Read auxiliary register group A */
#define LTC6811_CMD_RDAUXB      (0x000EU)  /**< Read auxiliary register group B */
#define LTC6811_CMD_RDAUXC      (0x000DU)  /**< Read auxiliary register group C */
#define LTC6811_CMD_RDAUXD      (0x000FU)  /**< Read auxiliary register group D */

#define LTC6811_CMD_RDSTATA     (0x0010U)  /**< Read status register group A */
#define LTC6811_CMD_RDSTATB     (0x0012U)  /**< Read status register group B */

#define LTC6811_CMD_ADCV        (0x0260U)  /**< Start cell voltage ADC conversion */
#define LTC6811_CMD_ADAX        (0x0460U)  /**< Start GPIO ADC conversion */
#define LTC6811_CMD_ADSTAT      (0x0468U)  /**< Start status group ADC conversion */

#define LTC6811_CMD_PLADC       (0x0714U)  /**< Poll ADC conversion status */
#define LTC6811_CMD_CLRCELL     (0x0711U)  /**< Clear cell voltage registers */
#define LTC6811_CMD_CLRAUX      (0x0712U)  /**< Clear auxiliary registers */
#define LTC6811_CMD_CLRSTAT     (0x0713U)  /**< Clear status registers */

#define LTC6811_CMD_DIAGN       (0x0715U)  /**< Run diagnostics */
#define LTC6811_CMD_MUTE        (0x0028U)  /**< Mute discharge */
#define LTC6811_CMD_UNMUTE      (0x0029U)  /**< Unmute discharge */

/** @brief PEC (Packet Error Code) polynomial */
#define PEC15_POLY              (0x4599U)

/** @brief Conversion times (microseconds) */
#define CONV_TIME_27KHZ_US      (201U)
#define CONV_TIME_7KHZ_US       (405U)
#define CONV_TIME_26HZ_US       (3000U)
#define CONV_TIME_2KHZ_US       (3300U)

/** @brief Register offsets */
#define REG_GROUP_SIZE          (6U)

/*============================================================================*/
/* PRIVATE VARIABLES                                                          */
/*============================================================================*/

/** @brief Device array */
static LTC6811_Device_t devices[LTC6811_MAX_DEVICES];

/** @brief Number of devices in daisy-chain */
static uint8_t num_devices = 0U;

/** @brief Initialization flag */
static bool ltc6811_initialized = false;

/** @brief PEC lookup table */
static const uint16_t pec15Table[256] = {
    0x0000, 0xC599, 0xCEAB, 0x0B32, 0xD8CF, 0x1D56, 0x1664, 0xD3FD,
    0xF407, 0x319E, 0x3AAC, 0xFF35, 0x2CC8, 0xE951, 0xE263, 0x27FA,
    0xAD97, 0x680E, 0x633C, 0xA6A5, 0x7558, 0xB0C1, 0xBBF3, 0x7E6A,
    0x5990, 0x9C09, 0x973B, 0x52A2, 0x815F, 0x44C6, 0x4FF4, 0x8A6D,
    0x5B2E, 0x9EB7, 0x9585, 0x501C, 0x83E1, 0x4678, 0x4D4A, 0x88D3,
    0xAF29, 0x6AB0, 0x6182, 0xA41B, 0x77E6, 0xB27F, 0xB94D, 0x7CD4,
    0xF6B9, 0x3320, 0x3812, 0xFD8B, 0x2E76, 0xEBEF, 0xE0DD, 0x2544,
    0x02BE, 0xC727, 0xCC15, 0x098C, 0xDA71, 0x1FE8, 0x14DA, 0xD143,
    0xB65C, 0x73C5, 0x78F7, 0xBD6E, 0x6E93, 0xAB0A, 0xA038, 0x65A1,
    0x425B, 0x87C2, 0x8CF0, 0x4969, 0x9A94, 0x5F0D, 0x543F, 0x91A6,
    0x1BCB, 0xDE52, 0xD560, 0x10F9, 0xC304, 0x069D, 0x0DAF, 0xC836,
    0xEFCC, 0x2A55, 0x2167, 0xE4FE, 0x3703, 0xF29A, 0xF9A8, 0x3C31,
    0xED72, 0x28EB, 0x23D9, 0xE640, 0x35BD, 0xF024, 0xFB16, 0x3E8F,
    0x1975, 0xDCEC, 0xD7DE, 0x1247, 0xC1BA, 0x0423, 0x0F11, 0xCA88,
    0x40E5, 0x857C, 0x8E4E, 0x4BD7, 0x982A, 0x5DB3, 0x5681, 0x9318,
    0xB4E2, 0x717B, 0x7A49, 0xBFD0, 0x6C2D, 0xA9B4, 0xA286, 0x671F,
    0xABB4, 0x6E2D, 0x651F, 0xA086, 0x737B, 0xB6E2, 0xBDD0, 0x7849,
    0x5FB3, 0x9A2A, 0x9118, 0x5481, 0x877C, 0x42E5, 0x49D7, 0x8C4E,
    0x0623, 0xC3BA, 0xC888, 0x0D11, 0xDEEC, 0x1B75, 0x1047, 0xD5DE,
    0xF224, 0x37BD, 0x3C8F, 0xF916, 0x2AEB, 0xEF72, 0xE440, 0x21D9,
    0xF09A, 0x3503, 0x3E31, 0xFBA8, 0x2855, 0xEDCC, 0xE6FE, 0x2367,
    0x049D, 0xC104, 0xCA36, 0x0FAF, 0xDC52, 0x19CB, 0x12F9, 0xD760,
    0x5D0D, 0x9894, 0x93A6, 0x563F, 0x85C2, 0x405B, 0x4B69, 0x8EF0,
    0xA90A, 0x6C93, 0x67A1, 0xA238, 0x71C5, 0xB45C, 0xBF6E, 0x7AF7,
    0x1DE8, 0xD871, 0xD343, 0x16DA, 0xC527, 0x00BE, 0x0B8C, 0xCE15,
    0xE9EF, 0x2C76, 0x2744, 0xE2DD, 0x3120, 0xF4B9, 0xFF8B, 0x3A12,
    0xB07F, 0x75E6, 0x7ED4, 0xBB4D, 0x68B0, 0xAD29, 0xA61B, 0x6382,
    0x4478, 0x81E1, 0x8AD3, 0x4F4A, 0x9CB7, 0x592E, 0x521C, 0x9785,
    0x46C6, 0x835F, 0x886D, 0x4DF4, 0x9E09, 0x5B90, 0x50A2, 0x953B,
    0xB2C1, 0x7758, 0x7C6A, 0xB9F3, 0x6A0E, 0xAF97, 0xA4A5, 0x613C,
    0xEB51, 0x2EC8, 0x25FA, 0xE063, 0x339E, 0xF607, 0xFD35, 0x38AC,
    0x1F56, 0xDACF, 0xD1FD, 0x1464, 0xC799, 0x0200, 0x0932, 0xCCAB
};

/*============================================================================*/
/* PRIVATE FUNCTION PROTOTYPES                                                */
/*============================================================================*/

static uint16_t ltc6811_calculate_pec15(const uint8_t *pData, uint8_t length);
static Status_t ltc6811_send_command(uint16_t cmd);
static Status_t ltc6811_write_register(uint16_t cmd, const uint8_t *pData, uint8_t numDevices);
static Status_t ltc6811_read_register(uint16_t cmd, uint8_t *pData, uint8_t numDevices);
static void ltc6811_delay_us(uint32_t us);
static uint16_t ltc6811_code_to_voltage_mv(uint16_t code);
static Status_t ltc6811_config_to_bytes(const LTC6811_Config_t *pConfig, uint8_t *pBytes);
static Status_t ltc6811_bytes_to_config(const uint8_t *pBytes, LTC6811_Config_t *pConfig);

/*============================================================================*/
/* PUBLIC FUNCTIONS                                                           */
/*============================================================================*/

/**
 * @brief Initialize LTC6811 driver
 */
Status_t LTC6811_Init(uint8_t numDevices)
{
    Status_t status = STATUS_OK;
    SPI_Config_t spi_config;

    /* Parameter validation */
    if ((numDevices == 0U) || (numDevices > LTC6811_MAX_DEVICES))
    {
        status = STATUS_ERROR_PARAM;
    }
    else if (ltc6811_initialized)
    {
        status = STATUS_ERROR_ALREADY_INIT;
    }
    else
    {
        /* Clear device array */
        (void)memset(devices, 0, sizeof(devices));
        num_devices = numDevices;

        /* Initialize SPI interface */
        spi_config.clockSpeed = BSP_SPI_SPEED_MEDIUM;  /* 2 MHz for ISO-SPI */
        spi_config.mode = SPI_MODE_3;                   /* CPOL=1, CPHA=1 */
        spi_config.bitOrder = SPI_BITORDER_MSB_FIRST;
        spi_config.dataSize = SPI_DATASIZE_8BIT;
        spi_config.useDMA = false;

        status = BSP_SPI_Init(BSP_SPI_INSTANCE_4, &spi_config);

        if (status == STATUS_OK)
        {
            /* Enable ISO-SPI interface */
            status = BSP_SPI_EnableISOSPI(true);

            if (status == STATUS_OK)
            {
                ltc6811_initialized = true;

                /* Wake up devices */
                status = LTC6811_WakeUp();
            }
        }
    }

    return status;
}

/**
 * @brief De-initialize LTC6811 driver
 */
Status_t LTC6811_DeInit(void)
{
    Status_t status = STATUS_OK;

    if (!ltc6811_initialized)
    {
        status = STATUS_ERROR_NOT_INIT;
    }
    else
    {
        /* Disable ISO-SPI */
        (void)BSP_SPI_EnableISOSPI(false);

        /* De-initialize SPI */
        status = BSP_SPI_DeInit(BSP_SPI_INSTANCE_4);

        if (status == STATUS_OK)
        {
            ltc6811_initialized = false;
            num_devices = 0U;
        }
    }

    return status;
}

/**
 * @brief Wake up LTC6811 from sleep mode
 */
Status_t LTC6811_WakeUp(void)
{
    Status_t status = STATUS_OK;
    uint8_t dummy = 0xFFU;

    if (!ltc6811_initialized)
    {
        status = STATUS_ERROR_NOT_INIT;
    }
    else
    {
        /* Assert CS */
        (void)BSP_SPI_ChipSelect(BSP_SPI_INSTANCE_4, true);

        /* Send dummy byte to wake up */
        status = BSP_SPI_Transmit(BSP_SPI_INSTANCE_4, &dummy, 1U, BSP_SPI_DEFAULT_TIMEOUT);

        /* Deassert CS */
        (void)BSP_SPI_ChipSelect(BSP_SPI_INSTANCE_4, false);

        /* Wait for wake-up time */
        ltc6811_delay_us(LTC6811_TWAKE_US);
    }

    return status;
}

/**
 * @brief Write configuration to LTC6811 device
 */
Status_t LTC6811_WriteConfig(uint8_t deviceIndex, const LTC6811_Config_t *pConfig)
{
    Status_t status = STATUS_OK;
    uint8_t config_bytes[6];

    /* Parameter validation */
    if (pConfig == NULL)
    {
        status = STATUS_ERROR_PARAM;
    }
    else if (deviceIndex >= num_devices)
    {
        status = STATUS_ERROR_PARAM;
    }
    else if (!ltc6811_initialized)
    {
        status = STATUS_ERROR_NOT_INIT;
    }
    else
    {
        /* Convert config to bytes */
        status = ltc6811_config_to_bytes(pConfig, config_bytes);

        if (status == STATUS_OK)
        {
            /* Write configuration register group A */
            status = ltc6811_write_register(LTC6811_CMD_WRCFGA, config_bytes, num_devices);

            if (status == STATUS_OK)
            {
                /* Store configuration */
                (void)memcpy(&devices[deviceIndex].config, pConfig, sizeof(LTC6811_Config_t));
                devices[deviceIndex].initialized = true;
            }
        }
    }

    return status;
}

/**
 * @brief Read configuration from LTC6811 device
 */
Status_t LTC6811_ReadConfig(uint8_t deviceIndex, LTC6811_Config_t *pConfig)
{
    Status_t status = STATUS_OK;
    uint8_t rx_data[64];
    uint8_t *pDeviceData;

    /* Parameter validation */
    if (pConfig == NULL)
    {
        status = STATUS_ERROR_PARAM;
    }
    else if (deviceIndex >= num_devices)
    {
        status = STATUS_ERROR_PARAM;
    }
    else if (!ltc6811_initialized)
    {
        status = STATUS_ERROR_NOT_INIT;
    }
    else
    {
        /* Read configuration register group A */
        status = ltc6811_read_register(LTC6811_CMD_RDCFGA, rx_data, num_devices);

        if (status == STATUS_OK)
        {
            /* Extract data for specific device */
            pDeviceData = &rx_data[deviceIndex * 8U];

            /* Verify PEC */
            uint16_t received_pec = ((uint16_t)pDeviceData[7] << 8) | pDeviceData[6];
            uint16_t calculated_pec = ltc6811_calculate_pec15(pDeviceData, 6U);

            if (received_pec != calculated_pec)
            {
                status = STATUS_ERROR_CRC;
            }
            else
            {
                /* Convert bytes to config */
                status = ltc6811_bytes_to_config(pDeviceData, pConfig);
            }
        }
    }

    return status;
}

/**
 * @brief Start cell voltage conversion
 */
Status_t LTC6811_StartCellVoltageConversion(LTC6811_ADCMode_t adcMode,
                                            LTC6811_CellSelection_t cellSelection)
{
    Status_t status = STATUS_OK;
    uint16_t cmd;

    if (!ltc6811_initialized)
    {
        status = STATUS_ERROR_NOT_INIT;
    }
    else
    {
        /* Build ADCV command */
        cmd = LTC6811_CMD_ADCV | ((uint16_t)adcMode << 7) | ((uint16_t)cellSelection << 0);

        /* Send command */
        status = ltc6811_send_command(cmd);
    }

    return status;
}

/**
 * @brief Read cell voltages from LTC6811 device
 */
Status_t LTC6811_ReadCellVoltages(uint8_t deviceIndex, LTC6811_CellVoltages_t *pVoltages)
{
    Status_t status = STATUS_OK;
    uint8_t rx_data[64];
    uint8_t *pGroupData;
    uint16_t cell_codes[12];
    uint8_t cell_idx;

    /* Parameter validation */
    if (pVoltages == NULL)
    {
        status = STATUS_ERROR_PARAM;
    }
    else if (deviceIndex >= num_devices)
    {
        status = STATUS_ERROR_PARAM;
    }
    else if (!ltc6811_initialized)
    {
        status = STATUS_ERROR_NOT_INIT;
    }
    else
    {
        /* Clear output structure */
        (void)memset(pVoltages, 0, sizeof(LTC6811_CellVoltages_t));

        /* Read cell voltage register groups A, B, C, D */
        /* Group A: Cells 1, 2, 3 */
        status = ltc6811_read_register(LTC6811_CMD_RDCVA, rx_data, num_devices);
        if (status == STATUS_OK)
        {
            pGroupData = &rx_data[deviceIndex * 8U];
            cell_codes[0] = ((uint16_t)pGroupData[1] << 8) | pGroupData[0];
            cell_codes[1] = ((uint16_t)pGroupData[3] << 8) | pGroupData[2];
            cell_codes[2] = ((uint16_t)pGroupData[5] << 8) | pGroupData[4];

            /* Group B: Cells 4, 5, 6 */
            status = ltc6811_read_register(LTC6811_CMD_RDCVB, rx_data, num_devices);
        }
        if (status == STATUS_OK)
        {
            pGroupData = &rx_data[deviceIndex * 8U];
            cell_codes[3] = ((uint16_t)pGroupData[1] << 8) | pGroupData[0];
            cell_codes[4] = ((uint16_t)pGroupData[3] << 8) | pGroupData[2];
            cell_codes[5] = ((uint16_t)pGroupData[5] << 8) | pGroupData[4];

            /* Group C: Cells 7, 8, 9 */
            status = ltc6811_read_register(LTC6811_CMD_RDCVC, rx_data, num_devices);
        }
        if (status == STATUS_OK)
        {
            pGroupData = &rx_data[deviceIndex * 8U];
            cell_codes[6] = ((uint16_t)pGroupData[1] << 8) | pGroupData[0];
            cell_codes[7] = ((uint16_t)pGroupData[3] << 8) | pGroupData[2];
            cell_codes[8] = ((uint16_t)pGroupData[5] << 8) | pGroupData[4];

            /* Group D: Cells 10, 11, 12 */
            status = ltc6811_read_register(LTC6811_CMD_RDCVD, rx_data, num_devices);
        }
        if (status == STATUS_OK)
        {
            pGroupData = &rx_data[deviceIndex * 8U];
            cell_codes[9] = ((uint16_t)pGroupData[1] << 8) | pGroupData[0];
            cell_codes[10] = ((uint16_t)pGroupData[3] << 8) | pGroupData[2];
            cell_codes[11] = ((uint16_t)pGroupData[5] << 8) | pGroupData[4];

            /* Convert codes to voltages */
            for (cell_idx = 0U; cell_idx < LTC6811_MAX_CELLS; cell_idx++)
            {
                pVoltages->cellVoltage_mV[cell_idx] = ltc6811_code_to_voltage_mv(cell_codes[cell_idx]);
                pVoltages->cellValid[cell_idx] = (cell_codes[cell_idx] != 0xFFFFU);
            }

            pVoltages->numCells = LTC6811_MAX_CELLS;

            /* Store in device state */
            (void)memcpy(&devices[deviceIndex].cellVoltages, pVoltages, sizeof(LTC6811_CellVoltages_t));
        }
    }

    return status;
}

/**
 * @brief Start GPIO conversion
 */
Status_t LTC6811_StartGPIOConversion(LTC6811_ADCMode_t adcMode,
                                     LTC6811_GPIOSelection_t gpioSelection)
{
    Status_t status = STATUS_OK;
    uint16_t cmd;

    if (!ltc6811_initialized)
    {
        status = STATUS_ERROR_NOT_INIT;
    }
    else
    {
        /* Build ADAX command */
        cmd = LTC6811_CMD_ADAX | ((uint16_t)adcMode << 7) | ((uint16_t)gpioSelection << 0);

        /* Send command */
        status = ltc6811_send_command(cmd);
    }

    return status;
}

/**
 * @brief Read GPIO voltages from LTC6811 device
 */
Status_t LTC6811_ReadGPIOVoltages(uint8_t deviceIndex, LTC6811_GPIOVoltages_t *pVoltages)
{
    Status_t status = STATUS_OK;
    uint8_t rx_data[64];
    uint8_t *pGroupData;
    uint16_t gpio_codes[9];
    uint8_t gpio_idx;

    /* Parameter validation */
    if (pVoltages == NULL)
    {
        status = STATUS_ERROR_PARAM;
    }
    else if (deviceIndex >= num_devices)
    {
        status = STATUS_ERROR_PARAM;
    }
    else if (!ltc6811_initialized)
    {
        status = STATUS_ERROR_NOT_INIT;
    }
    else
    {
        /* Clear output structure */
        (void)memset(pVoltages, 0, sizeof(LTC6811_GPIOVoltages_t));

        /* Read auxiliary register groups A, B */
        /* Group A: GPIO1, GPIO2, GPIO3 */
        status = ltc6811_read_register(LTC6811_CMD_RDAUXA, rx_data, num_devices);
        if (status == STATUS_OK)
        {
            pGroupData = &rx_data[deviceIndex * 8U];
            gpio_codes[0] = ((uint16_t)pGroupData[1] << 8) | pGroupData[0];
            gpio_codes[1] = ((uint16_t)pGroupData[3] << 8) | pGroupData[2];
            gpio_codes[2] = ((uint16_t)pGroupData[5] << 8) | pGroupData[4];

            /* Group B: GPIO4, GPIO5, REF */
            status = ltc6811_read_register(LTC6811_CMD_RDAUXB, rx_data, num_devices);
        }
        if (status == STATUS_OK)
        {
            pGroupData = &rx_data[deviceIndex * 8U];
            gpio_codes[3] = ((uint16_t)pGroupData[1] << 8) | pGroupData[0];
            gpio_codes[4] = ((uint16_t)pGroupData[3] << 8) | pGroupData[2];
            pVoltages->referenceVoltage_mV = ltc6811_code_to_voltage_mv(
                ((uint16_t)pGroupData[5] << 8) | pGroupData[4]);

            /* Convert codes to voltages */
            for (gpio_idx = 0U; gpio_idx < 5U; gpio_idx++)
            {
                pVoltages->gpioVoltage_mV[gpio_idx] = ltc6811_code_to_voltage_mv(gpio_codes[gpio_idx]);
                pVoltages->gpioValid[gpio_idx] = (gpio_codes[gpio_idx] != 0xFFFFU);
            }

            /* Store in device state */
            (void)memcpy(&devices[deviceIndex].gpioVoltages, pVoltages, sizeof(LTC6811_GPIOVoltages_t));
        }
    }

    return status;
}

/**
 * @brief Start status conversion
 */
Status_t LTC6811_StartStatusConversion(LTC6811_ADCMode_t adcMode)
{
    Status_t status = STATUS_OK;
    uint16_t cmd;

    if (!ltc6811_initialized)
    {
        status = STATUS_ERROR_NOT_INIT;
    }
    else
    {
        /* Build ADSTAT command */
        cmd = LTC6811_CMD_ADSTAT | ((uint16_t)adcMode << 7);

        /* Send command */
        status = ltc6811_send_command(cmd);
    }

    return status;
}

/**
 * @brief Read status registers
 */
Status_t LTC6811_ReadStatus(uint8_t deviceIndex, LTC6811_Status_t *pStatus)
{
    Status_t status = STATUS_OK;
    uint8_t rx_data[64];
    uint8_t *pGroupData;

    /* Parameter validation */
    if (pStatus == NULL)
    {
        status = STATUS_ERROR_PARAM;
    }
    else if (deviceIndex >= num_devices)
    {
        status = STATUS_ERROR_PARAM;
    }
    else if (!ltc6811_initialized)
    {
        status = STATUS_ERROR_NOT_INIT;
    }
    else
    {
        /* Clear output structure */
        (void)memset(pStatus, 0, sizeof(LTC6811_Status_t));

        /* Read status register group A */
        status = ltc6811_read_register(LTC6811_CMD_RDSTATA, rx_data, num_devices);
        if (status == STATUS_OK)
        {
            pGroupData = &rx_data[deviceIndex * 8U];
            pStatus->sumOfCells_mV = ltc6811_code_to_voltage_mv(
                ((uint16_t)pGroupData[1] << 8) | pGroupData[0]) * 20U;
            pStatus->internalDieTemp_C = (uint16_t)((((uint16_t)pGroupData[3] << 8) | pGroupData[2]) / 75U) - 273U;
            pStatus->analogSupply_mV = ltc6811_code_to_voltage_mv(
                ((uint16_t)pGroupData[5] << 8) | pGroupData[4]);

            /* Read status register group B */
            status = ltc6811_read_register(LTC6811_CMD_RDSTATB, rx_data, num_devices);
        }
        if (status == STATUS_OK)
        {
            pGroupData = &rx_data[deviceIndex * 8U];
            pStatus->digitalSupply_mV = ltc6811_code_to_voltage_mv(
                ((uint16_t)pGroupData[1] << 8) | pGroupData[0]);
            pStatus->revisionCode = pGroupData[4] & 0x0FU;
            pStatus->muxFail = ((pGroupData[4] & 0x02U) != 0U);
            pStatus->thermalShutdown = ((pGroupData[4] & 0x04U) != 0U);

            /* Store in device state */
            (void)memcpy(&devices[deviceIndex].status, pStatus, sizeof(LTC6811_Status_t));
        }
    }

    return status;
}

/**
 * @brief Set cell balancing
 */
Status_t LTC6811_SetCellBalancing(uint8_t deviceIndex, uint16_t cellMask)
{
    Status_t status = STATUS_OK;
    LTC6811_Config_t config;
    uint8_t cell_idx;

    /* Parameter validation */
    if (deviceIndex >= num_devices)
    {
        status = STATUS_ERROR_PARAM;
    }
    else if (!ltc6811_initialized)
    {
        status = STATUS_ERROR_NOT_INIT;
    }
    else
    {
        /* Get current configuration */
        (void)memcpy(&config, &devices[deviceIndex].config, sizeof(LTC6811_Config_t));

        /* Update cell balancing flags */
        for (cell_idx = 0U; cell_idx < LTC6811_MAX_CELLS; cell_idx++)
        {
            config.enableCellBalancing[cell_idx] = ((cellMask & (1U << cell_idx)) != 0U);
        }

        /* Write configuration */
        status = LTC6811_WriteConfig(deviceIndex, &config);
    }

    return status;
}

/**
 * @brief Clear cell voltages
 */
Status_t LTC6811_ClearCellVoltages(void)
{
    Status_t status = STATUS_OK;

    if (!ltc6811_initialized)
    {
        status = STATUS_ERROR_NOT_INIT;
    }
    else
    {
        status = ltc6811_send_command(LTC6811_CMD_CLRCELL);
    }

    return status;
}

/**
 * @brief Clear GPIO voltages
 */
Status_t LTC6811_ClearGPIOVoltages(void)
{
    Status_t status = STATUS_OK;

    if (!ltc6811_initialized)
    {
        status = STATUS_ERROR_NOT_INIT;
    }
    else
    {
        status = ltc6811_send_command(LTC6811_CMD_CLRAUX);
    }

    return status;
}

/**
 * @brief Clear status registers
 */
Status_t LTC6811_ClearStatus(void)
{
    Status_t status = STATUS_OK;

    if (!ltc6811_initialized)
    {
        status = STATUS_ERROR_NOT_INIT;
    }
    else
    {
        status = ltc6811_send_command(LTC6811_CMD_CLRSTAT);
    }

    return status;
}

/**
 * @brief Poll ADC conversion status
 */
Status_t LTC6811_PollADC(bool *pReady)
{
    Status_t status = STATUS_OK;
    uint8_t tx_data[4];
    uint8_t rx_data[1];
    uint16_t cmd_pec;

    /* Parameter validation */
    if (pReady == NULL)
    {
        status = STATUS_ERROR_PARAM;
    }
    else if (!ltc6811_initialized)
    {
        status = STATUS_ERROR_NOT_INIT;
    }
    else
    {
        /* Build command */
        tx_data[0] = (uint8_t)(LTC6811_CMD_PLADC >> 8);
        tx_data[1] = (uint8_t)(LTC6811_CMD_PLADC & 0xFFU);
        cmd_pec = ltc6811_calculate_pec15(tx_data, 2U);
        tx_data[2] = (uint8_t)(cmd_pec >> 8);
        tx_data[3] = (uint8_t)(cmd_pec & 0xFFU);

        /* Send command and read response */
        (void)BSP_SPI_ChipSelect(BSP_SPI_INSTANCE_4, true);
        status = BSP_SPI_Transmit(BSP_SPI_INSTANCE_4, tx_data, 4U, BSP_SPI_DEFAULT_TIMEOUT);
        if (status == STATUS_OK)
        {
            status = BSP_SPI_Receive(BSP_SPI_INSTANCE_4, rx_data, 1U, BSP_SPI_DEFAULT_TIMEOUT);
        }
        (void)BSP_SPI_ChipSelect(BSP_SPI_INSTANCE_4, false);

        if (status == STATUS_OK)
        {
            *pReady = (rx_data[0] == 0xFFU);
        }
    }

    return status;
}

/**
 * @brief Get device state
 */
Status_t LTC6811_GetDeviceState(uint8_t deviceIndex, LTC6811_Device_t *pDevice)
{
    Status_t status = STATUS_OK;

    /* Parameter validation */
    if (pDevice == NULL)
    {
        status = STATUS_ERROR_PARAM;
    }
    else if (deviceIndex >= num_devices)
    {
        status = STATUS_ERROR_PARAM;
    }
    else if (!ltc6811_initialized)
    {
        status = STATUS_ERROR_NOT_INIT;
    }
    else
    {
        (void)memcpy(pDevice, &devices[deviceIndex], sizeof(LTC6811_Device_t));
    }

    return status;
}

/**
 * @brief Update all measurements
 */
Status_t LTC6811_UpdateAllMeasurements(uint8_t deviceIndex)
{
    Status_t status = STATUS_OK;
    bool adc_ready = false;
    uint32_t timeout;

    /* Parameter validation */
    if (deviceIndex >= num_devices)
    {
        status = STATUS_ERROR_PARAM;
    }
    else if (!ltc6811_initialized)
    {
        status = STATUS_ERROR_NOT_INIT;
    }
    else
    {
        /* Start cell voltage conversion */
        status = LTC6811_StartCellVoltageConversion(LTC6811_ADC_MODE_7KHZ_3KHZ, LTC6811_CELL_ALL);

        if (status == STATUS_OK)
        {
            /* Wait for conversion */
            ltc6811_delay_us(CONV_TIME_7KHZ_US);

            /* Read cell voltages */
            status = LTC6811_ReadCellVoltages(deviceIndex, &devices[deviceIndex].cellVoltages);
        }

        if (status == STATUS_OK)
        {
            /* Start GPIO conversion */
            status = LTC6811_StartGPIOConversion(LTC6811_ADC_MODE_7KHZ_3KHZ, LTC6811_GPIO_ALL);
        }

        if (status == STATUS_OK)
        {
            /* Wait for conversion */
            ltc6811_delay_us(CONV_TIME_7KHZ_US);

            /* Read GPIO voltages */
            status = LTC6811_ReadGPIOVoltages(deviceIndex, &devices[deviceIndex].gpioVoltages);
        }

        if (status == STATUS_OK)
        {
            /* Start status conversion */
            status = LTC6811_StartStatusConversion(LTC6811_ADC_MODE_7KHZ_3KHZ);
        }

        if (status == STATUS_OK)
        {
            /* Wait for conversion */
            ltc6811_delay_us(CONV_TIME_7KHZ_US);

            /* Read status */
            status = LTC6811_ReadStatus(deviceIndex, &devices[deviceIndex].status);
        }
    }

    return status;
}

/**
 * @brief Run self-test on LTC6811
 */
Status_t LTC6811_RunSelfTest(LTC6811_SelfTestMode_t testMode)
{
    Status_t status = STATUS_OK;
    uint16_t cmd;

    if (!ltc6811_initialized)
    {
        status = STATUS_ERROR_NOT_INIT;
    }
    else
    {
        /* Build DIAGN command with test mode */
        cmd = LTC6811_CMD_DIAGN | ((uint16_t)testMode << 0);

        /* Send diagnostic command */
        status = ltc6811_send_command(cmd);
    }

    return status;
}

/**
 * @brief Read ADC overlap flag (diagnostics)
 */
Status_t LTC6811_ReadADCOverlap(uint8_t deviceIndex, bool *pOverlap)
{
    Status_t status = STATUS_OK;
    LTC6811_Status_t device_status;

    /* Parameter validation */
    if (pOverlap == NULL)
    {
        status = STATUS_ERROR_PARAM;
    }
    else if (deviceIndex >= num_devices)
    {
        status = STATUS_ERROR_PARAM;
    }
    else if (!ltc6811_initialized)
    {
        status = STATUS_ERROR_NOT_INIT;
    }
    else
    {
        /* Read status registers */
        status = LTC6811_ReadStatus(deviceIndex, &device_status);

        if (status == STATUS_OK)
        {
            /* ADC overlap is indicated by mux fail flag */
            *pOverlap = device_status.muxFail;
        }
    }

    return status;
}

/**
 * @brief Mute/unmute discharge
 */
Status_t LTC6811_MuteDischarge(bool mute)
{
    Status_t status = STATUS_OK;

    if (!ltc6811_initialized)
    {
        status = STATUS_ERROR_NOT_INIT;
    }
    else
    {
        if (mute)
        {
            status = ltc6811_send_command(LTC6811_CMD_MUTE);
        }
        else
        {
            status = ltc6811_send_command(LTC6811_CMD_UNMUTE);
        }
    }

    return status;
}

/*============================================================================*/
/* PRIVATE FUNCTIONS                                                          */
/*============================================================================*/

/**
 * @brief Calculate PEC15 (Packet Error Code)
 * @note Algorithm per LTC6811 datasheet section 4.2.2
 */
static uint16_t ltc6811_calculate_pec15(const uint8_t *pData, uint8_t length)
{
    uint16_t pec = LTC6811_PEC15_SEED;  /* PEC seed */
    uint8_t addr;
    uint8_t i;

    for (i = 0U; i < length; i++)
    {
        addr = (uint8_t)(((uint8_t)(pec >> 7)) ^ pData[i]);
        pec = (uint16_t)((pec << 8) ^ pec15Table[addr]);
    }

    return (pec * LTC6811_PEC15_MULTIPLIER);  /* Multiply by 2 for final PEC */
}

/**
 * @brief Send command to LTC6811
 */
static Status_t ltc6811_send_command(uint16_t cmd)
{
    Status_t status = STATUS_OK;
    uint8_t tx_data[4];
    uint16_t cmd_pec;

    /* Build command packet */
    tx_data[0] = (uint8_t)(cmd >> 8);
    tx_data[1] = (uint8_t)(cmd & 0xFFU);
    cmd_pec = ltc6811_calculate_pec15(tx_data, 2U);
    tx_data[2] = (uint8_t)(cmd_pec >> 8);
    tx_data[3] = (uint8_t)(cmd_pec & 0xFFU);

    /* Send command */
    (void)BSP_SPI_ChipSelect(BSP_SPI_INSTANCE_4, true);
    status = BSP_SPI_Transmit(BSP_SPI_INSTANCE_4, tx_data, 4U, BSP_SPI_DEFAULT_TIMEOUT);
    (void)BSP_SPI_ChipSelect(BSP_SPI_INSTANCE_4, false);

    return status;
}

/**
 * @brief Write register to LTC6811
 */
static Status_t ltc6811_write_register(uint16_t cmd, const uint8_t *pData, uint8_t numDevices)
{
    Status_t status = STATUS_OK;
    uint8_t tx_data[4 + (8U * LTC6811_MAX_DEVICES)];
    uint16_t cmd_pec;
    uint16_t data_pec;
    uint8_t i;

    /* Parameter validation */
    if ((pData == NULL) || (numDevices == 0U) || (numDevices > LTC6811_MAX_DEVICES))
    {
        status = STATUS_ERROR_PARAM;
    }
    else
    {
        /* Build command */
        tx_data[0] = (uint8_t)(cmd >> 8);
        tx_data[1] = (uint8_t)(cmd & 0xFFU);
        cmd_pec = ltc6811_calculate_pec15(tx_data, 2U);
        tx_data[2] = (uint8_t)(cmd_pec >> 8);
        tx_data[3] = (uint8_t)(cmd_pec & 0xFFU);

        /* Add data for each device */
        for (i = 0U; i < numDevices; i++)
        {
            (void)memcpy(&tx_data[4U + (i * 8U)], pData, 6U);
            data_pec = ltc6811_calculate_pec15(pData, 6U);
            tx_data[4U + (i * 8U) + 6U] = (uint8_t)(data_pec >> 8);
            tx_data[4U + (i * 8U) + 7U] = (uint8_t)(data_pec & 0xFFU);
        }

        /* Send data */
        (void)BSP_SPI_ChipSelect(BSP_SPI_INSTANCE_4, true);
        status = BSP_SPI_Transmit(BSP_SPI_INSTANCE_4, tx_data, 4U + (numDevices * 8U),
                                  BSP_SPI_DEFAULT_TIMEOUT);
        (void)BSP_SPI_ChipSelect(BSP_SPI_INSTANCE_4, false);
    }

    return status;
}

/**
 * @brief Read register from LTC6811
 */
static Status_t ltc6811_read_register(uint16_t cmd, uint8_t *pData, uint8_t numDevices)
{
    Status_t status = STATUS_OK;
    uint8_t tx_data[4];
    uint16_t cmd_pec;

    /* Parameter validation */
    if ((pData == NULL) || (numDevices == 0U) || (numDevices > LTC6811_MAX_DEVICES))
    {
        status = STATUS_ERROR_PARAM;
    }
    else
    {
        /* Build command */
        tx_data[0] = (uint8_t)(cmd >> 8);
        tx_data[1] = (uint8_t)(cmd & 0xFFU);
        cmd_pec = ltc6811_calculate_pec15(tx_data, 2U);
        tx_data[2] = (uint8_t)(cmd_pec >> 8);
        tx_data[3] = (uint8_t)(cmd_pec & 0xFFU);

        /* Send command and receive data */
        (void)BSP_SPI_ChipSelect(BSP_SPI_INSTANCE_4, true);
        status = BSP_SPI_Transmit(BSP_SPI_INSTANCE_4, tx_data, 4U, BSP_SPI_DEFAULT_TIMEOUT);
        if (status == STATUS_OK)
        {
            status = BSP_SPI_Receive(BSP_SPI_INSTANCE_4, pData, numDevices * 8U,
                                     BSP_SPI_DEFAULT_TIMEOUT);
        }
        (void)BSP_SPI_ChipSelect(BSP_SPI_INSTANCE_4, false);
    }

    return status;
}

/**
 * @brief Delay in microseconds (simple busy wait)
 */
static void ltc6811_delay_us(uint32_t us)
{
    /* Assuming 100 MHz clock, 100 cycles = 1 us */
    volatile uint32_t count = us * 100U;
    while (count > 0U)
    {
        count--;
    }
}

/**
 * @brief Convert ADC code to voltage in mV
 */
static uint16_t ltc6811_code_to_voltage_mv(uint16_t code)
{
    /* LTC6811 resolution: 100µV per LSB
     * ADC code is in units of 100µV (0.1mV)
     * To convert to mV: voltage_mV = code * 0.1mV = code / 10
     * Example: code=10 -> 10 * 100µV = 1000µV = 1.0mV -> 10/10 = 1mV
     */
    return (uint16_t)((uint32_t)code / 10U);
}

/**
 * @brief Convert configuration structure to byte array
 */
static Status_t ltc6811_config_to_bytes(const LTC6811_Config_t *pConfig, uint8_t *pBytes)
{
    Status_t status = STATUS_OK;
    uint16_t uv_val;
    uint16_t ov_val;

    if ((pConfig == NULL) || (pBytes == NULL))
    {
        status = STATUS_ERROR_PARAM;
    }
    else
    {
        /* Clear bytes */
        (void)memset(pBytes, 0, 6U);

        /* CFGR0: GPIO and reference configuration */
        pBytes[0] = 0x00U;
        if (pConfig->enableGPIO[0]) { pBytes[0] |= 0x01U; }
        if (pConfig->enableGPIO[1]) { pBytes[0] |= 0x02U; }
        if (pConfig->enableGPIO[2]) { pBytes[0] |= 0x04U; }
        if (pConfig->enableGPIO[3]) { pBytes[0] |= 0x08U; }
        if (pConfig->enableGPIO[4]) { pBytes[0] |= 0x10U; }
        if (pConfig->enableReferenceOn) { pBytes[0] |= 0x20U; }

        /* CFGR1: Under-voltage threshold */
        uv_val = pConfig->underVoltageThreshold_mV / 16U;  /* 16 * 100uV = 1.6mV per LSB */
        pBytes[1] = (uint8_t)(uv_val & 0xFFU);

        /* CFGR2: Over-voltage threshold */
        ov_val = pConfig->overVoltageThreshold_mV / 16U;
        pBytes[2] = (uint8_t)(ov_val & 0xFFU);

        /* CFGR3: Discharge enable */
        pBytes[3] = 0x00U;
        if (pConfig->enableCellBalancing[0]) { pBytes[3] |= 0x01U; }
        if (pConfig->enableCellBalancing[1]) { pBytes[3] |= 0x02U; }
        if (pConfig->enableCellBalancing[2]) { pBytes[3] |= 0x04U; }
        if (pConfig->enableCellBalancing[3]) { pBytes[3] |= 0x08U; }
        if (pConfig->enableCellBalancing[4]) { pBytes[3] |= 0x10U; }
        if (pConfig->enableCellBalancing[5]) { pBytes[3] |= 0x20U; }
        if (pConfig->enableCellBalancing[6]) { pBytes[3] |= 0x40U; }
        if (pConfig->enableCellBalancing[7]) { pBytes[3] |= 0x80U; }

        /* CFGR4: Discharge enable (continued) */
        pBytes[4] = 0x00U;
        if (pConfig->enableCellBalancing[8]) { pBytes[4] |= 0x01U; }
        if (pConfig->enableCellBalancing[9]) { pBytes[4] |= 0x02U; }
        if (pConfig->enableCellBalancing[10]) { pBytes[4] |= 0x04U; }
        if (pConfig->enableCellBalancing[11]) { pBytes[4] |= 0x08U; }

        /* CFGR5: Discharge timeout */
        pBytes[5] = (uint8_t)((pConfig->dischargeTimeout_min / 10U) & 0x0FU);
        if (pConfig->dischargePermission == LTC6811_DISCHARGE_PERMITTED)
        {
            pBytes[5] |= 0x10U;
        }
    }

    return status;
}

/**
 * @brief Convert byte array to configuration structure
 */
static Status_t ltc6811_bytes_to_config(const uint8_t *pBytes, LTC6811_Config_t *pConfig)
{
    Status_t status = STATUS_OK;

    if ((pBytes == NULL) || (pConfig == NULL))
    {
        status = STATUS_ERROR_PARAM;
    }
    else
    {
        /* Parse GPIO and reference configuration */
        pConfig->enableGPIO[0] = ((pBytes[0] & 0x01U) != 0U);
        pConfig->enableGPIO[1] = ((pBytes[0] & 0x02U) != 0U);
        pConfig->enableGPIO[2] = ((pBytes[0] & 0x04U) != 0U);
        pConfig->enableGPIO[3] = ((pBytes[0] & 0x08U) != 0U);
        pConfig->enableGPIO[4] = ((pBytes[0] & 0x10U) != 0U);
        pConfig->enableReferenceOn = ((pBytes[0] & 0x20U) != 0U);

        /* Parse voltage thresholds */
        pConfig->underVoltageThreshold_mV = (uint16_t)pBytes[1] * 16U;
        pConfig->overVoltageThreshold_mV = (uint16_t)pBytes[2] * 16U;

        /* Parse discharge enable */
        pConfig->enableCellBalancing[0] = ((pBytes[3] & 0x01U) != 0U);
        pConfig->enableCellBalancing[1] = ((pBytes[3] & 0x02U) != 0U);
        pConfig->enableCellBalancing[2] = ((pBytes[3] & 0x04U) != 0U);
        pConfig->enableCellBalancing[3] = ((pBytes[3] & 0x08U) != 0U);
        pConfig->enableCellBalancing[4] = ((pBytes[3] & 0x10U) != 0U);
        pConfig->enableCellBalancing[5] = ((pBytes[3] & 0x20U) != 0U);
        pConfig->enableCellBalancing[6] = ((pBytes[3] & 0x40U) != 0U);
        pConfig->enableCellBalancing[7] = ((pBytes[3] & 0x80U) != 0U);
        pConfig->enableCellBalancing[8] = ((pBytes[4] & 0x01U) != 0U);
        pConfig->enableCellBalancing[9] = ((pBytes[4] & 0x02U) != 0U);
        pConfig->enableCellBalancing[10] = ((pBytes[4] & 0x04U) != 0U);
        pConfig->enableCellBalancing[11] = ((pBytes[4] & 0x08U) != 0U);

        /* Parse discharge timeout and permission */
        pConfig->dischargeTimeout_min = (uint16_t)(pBytes[5] & 0x0FU) * 10U;
        pConfig->dischargePermission = ((pBytes[5] & 0x10U) != 0U) ?
            LTC6811_DISCHARGE_PERMITTED : LTC6811_DISCHARGE_NOT_PERMITTED;

        /* Set ADC mode to default */
        pConfig->adcMode = LTC6811_ADC_MODE_7KHZ_3KHZ;
    }

    return status;
}
