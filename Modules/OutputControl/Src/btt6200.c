/**
 * @file    btt6200.c
 * @brief   BTT6200-4ESA quad high-side switch driver implementation
 * @author  Battery Control Unit Development Team
 * @date    2026-01-09
 * @version 1.0.0
 *
 * @note    MISRA C:2012 compliant
 * @note    Controls 5× BTT6200 ICs (20 total outputs)
 *
 * @copyright Copyright (c) 2026
 */

/*============================================================================*/
/* INCLUDES                                                                   */
/*============================================================================*/
#include "btt6200.h"
#include "bsp_gpio.h"
#include "bsp_adc.h"
#include "app_config.h"
#include <string.h>

/*============================================================================*/
/* PRIVATE CONSTANTS                                                          */
/*============================================================================*/

/** @brief BTT6200 current sense resistor value (ohms) */
#define BTT6200_RSENSE_OHM          (1000U)  /* 1k ohm */

/** @brief BTT6200 current sense ratio (A/V) */
#define BTT6200_SENSE_RATIO         (1200U)  /* 1200:1 */

/*============================================================================*/
/* PRIVATE TYPES                                                              */
/*============================================================================*/

/** @brief BTT6200 IC control pins */
typedef struct {
    GPIO_TypeDef *port;
    uint16_t den_pin;
    uint16_t dsel0_pin;
    uint16_t dsel1_pin;
    uint16_t in0_pin;
    uint16_t in1_pin;
    uint16_t in2_pin;
    uint16_t in3_pin;
    uint8_t  is_adc_channel;
} BTT6200_IC_Pins_t;

/*============================================================================*/
/* PRIVATE VARIABLES                                                          */
/*============================================================================*/

/** @brief IC status for all 5 BTT6200 devices */
static BTT6200_IC_Status_t ic_status[BTT6200_IC_COUNT];

/** @brief Channel configurations */
static BTT6200_ChannelConfig_t channel_config[BTT6200_TOTAL_CHANNELS];

/** @brief Fault callback */
static BTT6200_FaultCallback_t fault_callback = NULL;

/** @brief Initialization flag */
static bool btt6200_initialized = false;

/** @brief Pin mapping for all 5 ICs */
static const BTT6200_IC_Pins_t ic_pins[BTT6200_IC_COUNT] = {
    /* IC0: Port C */
    {GPIOC, GPIO_PIN_0, GPIO_PIN_1, GPIO_PIN_2, GPIO_PIN_3, GPIO_PIN_4, GPIO_PIN_5, GPIO_PIN_6, 11},
    /* IC1: Port C */
    {GPIOC, GPIO_PIN_8, GPIO_PIN_9, GPIO_PIN_10, GPIO_PIN_11, GPIO_PIN_12, GPIO_PIN_13, GPIO_PIN_14, 12},
    /* IC2: Port D */
    {GPIOD, GPIO_PIN_0, GPIO_PIN_1, GPIO_PIN_2, GPIO_PIN_3, GPIO_PIN_4, GPIO_PIN_5, GPIO_PIN_6, 13},
    /* IC3: Port D */
    {GPIOD, GPIO_PIN_8, GPIO_PIN_9, GPIO_PIN_10, GPIO_PIN_11, GPIO_PIN_12, GPIO_PIN_13, GPIO_PIN_14, 14},
    /* IC4: Port E */
    {GPIOE, GPIO_PIN_0, GPIO_PIN_1, GPIO_PIN_3, GPIO_PIN_7, GPIO_PIN_8, GPIO_PIN_9, GPIO_PIN_10, 15}
};

/*============================================================================*/
/* PRIVATE FUNCTION PROTOTYPES                                                */
/*============================================================================*/

static void btt6200_set_input_pins(uint8_t icNum, uint8_t channelMask);
static Status_t btt6200_read_current(uint8_t icNum, Current_mA_t *pCurrent);
static void btt6200_diagnose_channel_internal(uint8_t channel);

/*============================================================================*/
/* PUBLIC FUNCTIONS                                                           */
/*============================================================================*/

/**
 * @brief Initialize BTT6200 driver
 */
Status_t BTT6200_Init(void)
{
    Status_t status = STATUS_OK;

    if (btt6200_initialized)
    {
        status = STATUS_ERROR_ALREADY_INIT;
    }
    else
    {
        uint8_t i;

        /* Initialize all ICs to safe state */
        for (i = 0U; i < BTT6200_IC_COUNT; i++)
        {
            /* Disable diagnostic mode */
            HAL_GPIO_WritePin(ic_pins[i].port, ic_pins[i].den_pin, GPIO_PIN_RESET);

            /* Set all inputs LOW (outputs OFF) */
            btt6200_set_input_pins(i, 0x00U);

            /* Initialize IC status */
            ic_status[i].initialized = true;
            ic_status[i].faultCount = 0U;

            for (uint8_t ch = 0U; ch < 4U; ch++)
            {
                ic_status[i].state[ch] = BTT6200_STATE_OFF;
                (void)memset(&ic_status[i].diag[ch], 0, sizeof(BTT6200_ChannelDiag_t));
            }
        }

        /* Initialize channel configurations */
        for (i = 0U; i < BTT6200_TOTAL_CHANNELS; i++)
        {
            channel_config[i].enabled = false;
            channel_config[i].pwmDutyCycle = 0U;
            channel_config[i].currentLimit_mA = BTT6200_MAX_CURRENT_MA;
            channel_config[i].diagnosticsEnabled = true;
            channel_config[i].autoRetry = false;
        }

        btt6200_initialized = true;
    }

    return status;
}

/**
 * @brief Turn on output channel
 */
Status_t BTT6200_ChannelOn(uint8_t channel)
{
    Status_t status = STATUS_ERROR_PARAM;

    if ((channel < BTT6200_TOTAL_CHANNELS) && btt6200_initialized)
    {
        uint8_t icNum = channel / 4U;
        uint8_t chNum = channel % 4U;

        /* Set channel state */
        ic_status[icNum].state[chNum] = BTT6200_STATE_ON;
        channel_config[channel].enabled = true;
        channel_config[channel].pwmDutyCycle = BTT6200_PWM_MAX_DUTY;

        /* Update input pins */
        uint8_t currentMask = 0U;
        for (uint8_t i = 0U; i < 4U; i++)
        {
            if (ic_status[icNum].state[i] == BTT6200_STATE_ON)
            {
                currentMask |= (1U << i);
            }
        }

        btt6200_set_input_pins(icNum, currentMask);
        status = STATUS_OK;
    }

    return status;
}

/**
 * @brief Turn off output channel
 */
Status_t BTT6200_ChannelOff(uint8_t channel)
{
    Status_t status = STATUS_ERROR_PARAM;

    if ((channel < BTT6200_TOTAL_CHANNELS) && btt6200_initialized)
    {
        uint8_t icNum = channel / 4U;
        uint8_t chNum = channel % 4U;

        /* Set channel state */
        ic_status[icNum].state[chNum] = BTT6200_STATE_OFF;
        channel_config[channel].enabled = false;
        channel_config[channel].pwmDutyCycle = 0U;

        /* Update input pins */
        uint8_t currentMask = 0U;
        for (uint8_t i = 0U; i < 4U; i++)
        {
            if (ic_status[icNum].state[i] == BTT6200_STATE_ON)
            {
                currentMask |= (1U << i);
            }
        }

        btt6200_set_input_pins(icNum, currentMask);
        status = STATUS_OK;
    }

    return status;
}

/**
 * @brief Read channel current
 */
Status_t BTT6200_ReadChannelCurrent(uint8_t channel, Current_mA_t *pCurrent_mA)
{
    Status_t status = STATUS_ERROR_PARAM;

    if ((channel < BTT6200_TOTAL_CHANNELS) && (pCurrent_mA != NULL) && btt6200_initialized)
    {
        uint8_t icNum = channel / 4U;

        /* Read IC current sense */
        Current_mA_t totalCurrent;
        status = btt6200_read_current(icNum, &totalCurrent);

        if (status == STATUS_OK)
        {
            /* Estimate current per channel (total / active channels) */
            uint8_t activeChannels = 0U;
            for (uint8_t i = 0U; i < 4U; i++)
            {
                if (ic_status[icNum].state[i] == BTT6200_STATE_ON)
                {
                    activeChannels++;
                }
            }

            if (activeChannels > 0U)
            {
                *pCurrent_mA = totalCurrent / (int32_t)activeChannels;
            }
            else
            {
                *pCurrent_mA = 0;
            }
        }
    }

    return status;
}

/**
 * @brief Perform channel diagnostics
 */
Status_t BTT6200_DiagnoseChannel(uint8_t channel, BTT6200_ChannelDiag_t *pDiag)
{
    Status_t status = STATUS_ERROR_PARAM;

    if ((channel < BTT6200_TOTAL_CHANNELS) && (pDiag != NULL) && btt6200_initialized)
    {
        uint8_t icNum = channel / 4U;
        uint8_t chNum = channel % 4U;

        btt6200_diagnose_channel_internal(channel);

        *pDiag = ic_status[icNum].diag[chNum];
        status = STATUS_OK;
    }

    return status;
}

/**
 * @brief Set all channels to safe state
 */
Status_t BTT6200_SetSafeState(void)
{
    Status_t status = STATUS_OK;

    if (btt6200_initialized)
    {
        /* Turn off all channels */
        for (uint8_t i = 0U; i < BTT6200_TOTAL_CHANNELS; i++)
        {
            (void)BTT6200_ChannelOff(i);
        }
    }

    return status;
}

/**
 * @brief Update BTT6200 driver (periodic)
 */
Status_t BTT6200_Update(void)
{
    Status_t status = STATUS_OK;

    if (btt6200_initialized)
    {
        /* Perform diagnostics on all enabled channels */
        for (uint8_t ch = 0U; ch < BTT6200_TOTAL_CHANNELS; ch++)
        {
            if (channel_config[ch].enabled && channel_config[ch].diagnosticsEnabled)
            {
                btt6200_diagnose_channel_internal(ch);
            }
        }
    }

    return status;
}

/**
 * @brief Convert ADC to current
 */
Current_mA_t BTT6200_ADCToCurrent(uint16_t adcValue)
{
    Current_mA_t current_mA;

    /* Convert ADC to voltage */
    uint32_t voltage_mV = BSP_ADC_ToMillivolts(adcValue);

    /* Calculate current: I = (V * SENSE_RATIO) / RSENSE */
    current_mA = ((int32_t)voltage_mV * (int32_t)BTT6200_SENSE_RATIO) / (int32_t)BTT6200_RSENSE_OHM;

    return current_mA;
}

/**
 * @brief Register fault callback
 */
Status_t BTT6200_RegisterFaultCallback(BTT6200_FaultCallback_t callback)
{
    fault_callback = callback;
    return STATUS_OK;
}

/**
 * @brief De-initialize BTT6200 driver
 */
Status_t BTT6200_DeInit(void)
{
    if (btt6200_initialized)
    {
        (void)BTT6200_SetSafeState();
        btt6200_initialized = false;
    }

    return STATUS_OK;
}

/*============================================================================*/
/* PRIVATE FUNCTIONS                                                          */
/*============================================================================*/

/**
 * @brief Set input pins for an IC
 */
static void btt6200_set_input_pins(uint8_t icNum, uint8_t channelMask)
{
    if (icNum < BTT6200_IC_COUNT)
    {
        const BTT6200_IC_Pins_t *pins = &ic_pins[icNum];

        HAL_GPIO_WritePin(pins->port, pins->in0_pin,
                         ((channelMask & 0x01U) != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_GPIO_WritePin(pins->port, pins->in1_pin,
                         ((channelMask & 0x02U) != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_GPIO_WritePin(pins->port, pins->in2_pin,
                         ((channelMask & 0x04U) != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_GPIO_WritePin(pins->port, pins->in3_pin,
                         ((channelMask & 0x08U) != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }
}

/**
 * @brief Read current from IC
 */
static Status_t btt6200_read_current(uint8_t icNum, Current_mA_t *pCurrent)
{
    Status_t status = STATUS_ERROR_PARAM;

    if ((icNum < BTT6200_IC_COUNT) && (pCurrent != NULL))
    {
        uint16_t adcValue;
        uint8_t adcChannel = ic_pins[icNum].is_adc_channel;

        status = BSP_ADC_ReadChannel(adcChannel, &adcValue, TIMEOUT_ADC_MS);

        if (status == STATUS_OK)
        {
            *pCurrent = BTT6200_ADCToCurrent(adcValue);
        }
    }

    return status;
}

/**
 * @brief Internal diagnostics
 */
static void btt6200_diagnose_channel_internal(uint8_t channel)
{
    if (channel < BTT6200_TOTAL_CHANNELS)
    {
        uint8_t icNum = channel / 4U;
        uint8_t chNum = channel % 4U;

        /* Read current */
        Current_mA_t current;
        if (btt6200_read_current(icNum, &current) == STATUS_OK)
        {
            ic_status[icNum].diag[chNum].current_mA = current;

            /* Check overcurrent */
            if (current > (int32_t)channel_config[channel].currentLimit_mA)
            {
                ic_status[icNum].diag[chNum].overcurrent = true;
                ic_status[icNum].diag[chNum].faultType = BTT6200_FAULT_OVERCURRENT;
                ic_status[icNum].diag[chNum].faultCount++;

                if (fault_callback != NULL)
                {
                    fault_callback(channel, BTT6200_FAULT_OVERCURRENT);
                }
            }
            else
            {
                ic_status[icNum].diag[chNum].overcurrent = false;
            }

            /* Check open load (current too low when ON) */
            if ((ic_status[icNum].state[chNum] == BTT6200_STATE_ON) && (current < 10))
            {
                ic_status[icNum].diag[chNum].openLoad = true;
                ic_status[icNum].diag[chNum].faultType = BTT6200_FAULT_OPEN_LOAD;
                ic_status[icNum].diag[chNum].faultCount++;

                if (fault_callback != NULL)
                {
                    fault_callback(channel, BTT6200_FAULT_OPEN_LOAD);
                }
            }
            else
            {
                ic_status[icNum].diag[chNum].openLoad = false;
            }

            /* Check short circuit (current too high immediately) */
            if (current > ((int32_t)BTT6200_MAX_CURRENT_MA * 110 / 100))
            {
                ic_status[icNum].diag[chNum].shortCircuit = true;
                ic_status[icNum].diag[chNum].faultType = BTT6200_FAULT_SHORT_CIRCUIT;
                ic_status[icNum].diag[chNum].faultCount++;

                /* Turn off channel immediately */
                (void)BTT6200_ChannelOff(channel);

                if (fault_callback != NULL)
                {
                    fault_callback(channel, BTT6200_FAULT_SHORT_CIRCUIT);
                }
            }
            else
            {
                ic_status[icNum].diag[chNum].shortCircuit = false;
            }

            /* Update timestamp */
            if (ic_status[icNum].diag[chNum].faultType != BTT6200_FAULT_NONE)
            {
                ic_status[icNum].diag[chNum].lastFaultTime_ms = HAL_GetTick();
            }
        }
    }
}

/*============================================================================*/
/* END OF FILE                                                                */
/*============================================================================*/
