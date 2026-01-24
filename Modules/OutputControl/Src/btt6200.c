/**
 * @file    btt6200.c
 * @brief   BTT6200-4ESA quad high-side switch driver implementation
 * @author  Battery Control Unit Development Team
 * @date    2026-01-09
 * @version 1.1.0
 *
 * @note    MISRA C:2012 compliant
 * @note    Controls 5x BTT6200 ICs (20 total outputs)
 *
 * Pin Mapping (from schematic):
 * =============================
 * IC0: DEN=PE14, DSEL0=PE13, DSEL1=PE10, IN0=PB10, IN1=PE15, IN2=PE12, IN3=PE11, IS=PC0 (ADC_CH10)
 * IC1: DEN=PD11, DSEL0=PD10, DSEL1=PB15, IN0=PD13, IN1=PD12, IN2=PD9,  IN3=PD8,  IS=PC1 (ADC_CH11)
 * IC2: DEN=PG4,  DSEL0=PG3,  DSEL1=PD14, IN0=PG6,  IN1=PG5,  IN2=PG2,  IN3=PD15, IS=PC3 (ADC_CH13)
 * IC3: DEN=PC7,  DSEL0=PC8,  DSEL1=PC9,  IN0=PA8,  IN1=PA9,  IN2=PA10, IN3=PA11, IS=PC2 (ADC_CH12)
 * IC4: DEN=PC11, DSEL0=PC12, DSEL1=PD2,  IN0=PA15, IN1=PC10, IN2=PD0,  IN3=PD1,  IS=PC4 (ADC_CH14)
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
#include "timestamp.h"  /* For Timestamp_DelayUs */
#include "main.h"  /* For GPIO pin definitions */
#include <string.h>

/*============================================================================*/
/* PRIVATE CONSTANTS                                                          */
/*============================================================================*/

/** @brief BTT6200 current sense resistor value (ohms) */
#define BTT6200_RSENSE_OHM          (1000U)  /* 1k ohm */

/** @brief BTT6200 current sense ratio (A/V) - k_ILIS from datasheet */
#define BTT6200_SENSE_RATIO         (1200U)  /* 1200:1 */

/** @brief Diagnostic settle time (microseconds) */
#define BTT6200_DIAG_SETTLE_US      (100U)

/** @brief Minimum current to detect load (mA) */
#define BTT6200_OPEN_LOAD_THRESHOLD_MA   (10)

/** @brief Short circuit threshold - 110% of max current */
#define BTT6200_SHORT_CIRCUIT_THRESHOLD_MA  (BTT6200_MAX_CURRENT_MA * 110 / 100)

/*============================================================================*/
/* PRIVATE TYPES                                                              */
/*============================================================================*/

/**
 * @brief Pin definition for a single GPIO
 */
typedef struct {
    GPIO_TypeDef *port;
    uint16_t pin;
} GPIO_Pin_t;

/**
 * @brief BTT6200 IC pin configuration
 * @note Each IC has pins on multiple GPIO ports
 */
typedef struct {
    GPIO_Pin_t den;     /**< Diagnostic Enable */
    GPIO_Pin_t dsel0;   /**< Diagnostic Select 0 */
    GPIO_Pin_t dsel1;   /**< Diagnostic Select 1 */
    GPIO_Pin_t in0;     /**< Channel 0 input */
    GPIO_Pin_t in1;     /**< Channel 1 input */
    GPIO_Pin_t in2;     /**< Channel 2 input */
    GPIO_Pin_t in3;     /**< Channel 3 input */
    uint8_t is_adc_channel; /**< ADC channel for IS pin */
} BTT6200_IC_PinConfig_t;

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

/**
 * @brief Pin mapping for all 5 BTT6200 ICs
 * @note Mapping based on actual schematic connections
 *
 * ADC Channels for IS pins (PC0-PC4):
 * - PC0 = ADC1_IN10
 * - PC1 = ADC1_IN11
 * - PC2 = ADC1_IN12
 * - PC3 = ADC1_IN13
 * - PC4 = ADC1_IN14
 */
static const BTT6200_IC_PinConfig_t ic_pins[BTT6200_IC_COUNT] = {
    /* IC0: DEN=PE14, DSEL0=PE13, DSEL1=PE10, IN0=PB10, IN1=PE15, IN2=PE12, IN3=PE11, IS=PC0 */
    {
        .den   = {GPIOE, GPIO_PIN_14},
        .dsel0 = {GPIOE, GPIO_PIN_13},
        .dsel1 = {GPIOE, GPIO_PIN_10},
        .in0   = {GPIOB, GPIO_PIN_10},
        .in1   = {GPIOE, GPIO_PIN_15},
        .in2   = {GPIOE, GPIO_PIN_12},
        .in3   = {GPIOE, GPIO_PIN_11},
        .is_adc_channel = 10U  /* ADC1_IN10 = PC0 */
    },
    /* IC1: DEN=PD11, DSEL0=PD10, DSEL1=PB15, IN0=PD13, IN1=PD12, IN2=PD9, IN3=PD8, IS=PC1 */
    {
        .den   = {GPIOD, GPIO_PIN_11},
        .dsel0 = {GPIOD, GPIO_PIN_10},
        .dsel1 = {GPIOB, GPIO_PIN_15},
        .in0   = {GPIOD, GPIO_PIN_13},
        .in1   = {GPIOD, GPIO_PIN_12},
        .in2   = {GPIOD, GPIO_PIN_9},
        .in3   = {GPIOD, GPIO_PIN_8},
        .is_adc_channel = 11U  /* ADC1_IN11 = PC1 */
    },
    /* IC2: DEN=PG4, DSEL0=PG3, DSEL1=PD14, IN0=PG6, IN1=PG5, IN2=PG2, IN3=PD15, IS=PC3 */
    {
        .den   = {GPIOG, GPIO_PIN_4},
        .dsel0 = {GPIOG, GPIO_PIN_3},
        .dsel1 = {GPIOD, GPIO_PIN_14},
        .in0   = {GPIOG, GPIO_PIN_6},
        .in1   = {GPIOG, GPIO_PIN_5},
        .in2   = {GPIOG, GPIO_PIN_2},
        .in3   = {GPIOD, GPIO_PIN_15},
        .is_adc_channel = 13U  /* ADC1_IN13 = PC3 */
    },
    /* IC3: DEN=PC7, DSEL0=PC8, DSEL1=PC9, IN0=PA8, IN1=PA9, IN2=PA10, IN3=PA11, IS=PC2 */
    {
        .den   = {GPIOC, GPIO_PIN_7},
        .dsel0 = {GPIOC, GPIO_PIN_8},
        .dsel1 = {GPIOC, GPIO_PIN_9},
        .in0   = {GPIOA, GPIO_PIN_8},
        .in1   = {GPIOA, GPIO_PIN_9},
        .in2   = {GPIOA, GPIO_PIN_10},
        .in3   = {GPIOA, GPIO_PIN_11},
        .is_adc_channel = 12U  /* ADC1_IN12 = PC2 */
    },
    /* IC4: DEN=PC11, DSEL0=PC12, DSEL1=PD2, IN0=PA15, IN1=PC10, IN2=PD0, IN3=PD1, IS=PC4 */
    {
        .den   = {GPIOC, GPIO_PIN_11},
        .dsel0 = {GPIOC, GPIO_PIN_12},
        .dsel1 = {GPIOD, GPIO_PIN_2},
        .in0   = {GPIOA, GPIO_PIN_15},
        .in1   = {GPIOC, GPIO_PIN_10},
        .in2   = {GPIOD, GPIO_PIN_0},
        .in3   = {GPIOD, GPIO_PIN_1},
        .is_adc_channel = 14U  /* ADC1_IN14 = PC4 */
    }
};

/*============================================================================*/
/* PRIVATE FUNCTION PROTOTYPES                                                */
/*============================================================================*/

static void btt6200_write_pin(const GPIO_Pin_t *pPin, GPIO_PinState state);
static GPIO_PinState btt6200_read_pin(const GPIO_Pin_t *pPin);
static void btt6200_set_channel_output(uint8_t icNum, uint8_t channel, bool state);
static void btt6200_set_all_outputs(uint8_t icNum, uint8_t channelMask);
static void btt6200_enable_diagnostics(uint8_t icNum, bool enable);
static void btt6200_select_diag_channel(uint8_t icNum, uint8_t channel);
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
            const BTT6200_IC_PinConfig_t *pPins = &ic_pins[i];

            /* Disable diagnostic mode (DEN = LOW) */
            btt6200_write_pin(&pPins->den, GPIO_PIN_RESET);

            /* Set diagnostic select to channel 0 */
            btt6200_write_pin(&pPins->dsel0, GPIO_PIN_RESET);
            btt6200_write_pin(&pPins->dsel1, GPIO_PIN_RESET);

            /* Set all inputs LOW (outputs OFF) */
            btt6200_set_all_outputs(i, 0x00U);

            /* Initialize IC status */
            ic_status[i].initialized = true;
            ic_status[i].faultCount = 0U;

            for (uint8_t ch = 0U; ch < BTT6200_CHANNELS_PER_IC; ch++)
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
        uint8_t icNum = channel / BTT6200_CHANNELS_PER_IC;
        uint8_t chNum = channel % BTT6200_CHANNELS_PER_IC;

        /* Set channel state */
        ic_status[icNum].state[chNum] = BTT6200_STATE_ON;
        channel_config[channel].enabled = true;
        channel_config[channel].pwmDutyCycle = BTT6200_PWM_MAX_DUTY;

        /* Set the output pin HIGH */
        btt6200_set_channel_output(icNum, chNum, true);

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
        uint8_t icNum = channel / BTT6200_CHANNELS_PER_IC;
        uint8_t chNum = channel % BTT6200_CHANNELS_PER_IC;

        /* Set channel state */
        ic_status[icNum].state[chNum] = BTT6200_STATE_OFF;
        channel_config[channel].enabled = false;
        channel_config[channel].pwmDutyCycle = 0U;

        /* Set the output pin LOW */
        btt6200_set_channel_output(icNum, chNum, false);

        status = STATUS_OK;
    }

    return status;
}

/**
 * @brief Toggle output channel
 */
Status_t BTT6200_ChannelToggle(uint8_t channel)
{
    Status_t status = STATUS_ERROR_PARAM;

    if ((channel < BTT6200_TOTAL_CHANNELS) && btt6200_initialized)
    {
        uint8_t icNum = channel / BTT6200_CHANNELS_PER_IC;
        uint8_t chNum = channel % BTT6200_CHANNELS_PER_IC;

        if (ic_status[icNum].state[chNum] == BTT6200_STATE_ON)
        {
            status = BTT6200_ChannelOff(channel);
        }
        else
        {
            status = BTT6200_ChannelOn(channel);
        }
    }

    return status;
}

/**
 * @brief Set output channel PWM duty cycle
 */
Status_t BTT6200_ChannelSetPWM(uint8_t channel, uint16_t dutyCycle)
{
    Status_t status = STATUS_ERROR_PARAM;

    if ((channel < BTT6200_TOTAL_CHANNELS) && (dutyCycle <= BTT6200_PWM_MAX_DUTY) && btt6200_initialized)
    {
        uint8_t icNum = channel / BTT6200_CHANNELS_PER_IC;
        uint8_t chNum = channel % BTT6200_CHANNELS_PER_IC;

        channel_config[channel].pwmDutyCycle = dutyCycle;

        if (dutyCycle == 0U)
        {
            ic_status[icNum].state[chNum] = BTT6200_STATE_OFF;
            btt6200_set_channel_output(icNum, chNum, false);
        }
        else if (dutyCycle == BTT6200_PWM_MAX_DUTY)
        {
            ic_status[icNum].state[chNum] = BTT6200_STATE_ON;
            btt6200_set_channel_output(icNum, chNum, true);
        }
        else
        {
            ic_status[icNum].state[chNum] = BTT6200_STATE_PWM;
            /* Note: Actual PWM implementation requires timer configuration */
            /* For now, treat as ON if duty > 0 */
            btt6200_set_channel_output(icNum, chNum, true);
        }

        status = STATUS_OK;
    }

    return status;
}

/**
 * @brief Read channel current via IS pin
 */
Status_t BTT6200_ReadChannelCurrent(uint8_t channel, Current_mA_t *pCurrent_mA)
{
    Status_t status = STATUS_ERROR_PARAM;

    if ((channel < BTT6200_TOTAL_CHANNELS) && (pCurrent_mA != NULL) && btt6200_initialized)
    {
        uint8_t icNum = channel / BTT6200_CHANNELS_PER_IC;
        uint8_t chNum = channel % BTT6200_CHANNELS_PER_IC;

        /* Enable diagnostics and select channel */
        btt6200_enable_diagnostics(icNum, true);
        btt6200_select_diag_channel(icNum, chNum);

        /* Delay for diagnostic output to settle (100us per BTT6200 datasheet) */
        Timestamp_DelayUs(BTT6200_DIAG_SETTLE_US);

        /* Read current from IS pin */
        status = btt6200_read_current(icNum, pCurrent_mA);

        /* Disable diagnostics */
        btt6200_enable_diagnostics(icNum, false);
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
        uint8_t icNum = channel / BTT6200_CHANNELS_PER_IC;
        uint8_t chNum = channel % BTT6200_CHANNELS_PER_IC;

        btt6200_diagnose_channel_internal(channel);

        *pDiag = ic_status[icNum].diag[chNum];
        status = STATUS_OK;
    }

    return status;
}

/**
 * @brief Perform diagnostics on all channels of an IC
 */
Status_t BTT6200_DiagnoseIC(uint8_t icNumber)
{
    Status_t status = STATUS_ERROR_PARAM;

    if ((icNumber < BTT6200_IC_COUNT) && btt6200_initialized)
    {
        /* Diagnose all 4 channels */
        for (uint8_t ch = 0U; ch < BTT6200_CHANNELS_PER_IC; ch++)
        {
            uint8_t globalChannel = (icNumber * BTT6200_CHANNELS_PER_IC) + ch;
            btt6200_diagnose_channel_internal(globalChannel);
        }
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
        /* Turn off all outputs immediately */
        for (uint8_t ic = 0U; ic < BTT6200_IC_COUNT; ic++)
        {
            /* Disable diagnostics */
            btt6200_enable_diagnostics(ic, false);

            /* Set all outputs LOW */
            btt6200_set_all_outputs(ic, 0x00U);

            /* Update state */
            for (uint8_t ch = 0U; ch < BTT6200_CHANNELS_PER_IC; ch++)
            {
                ic_status[ic].state[ch] = BTT6200_STATE_OFF;
            }
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
 * @note Formula: I_load = (V_IS * k_ILIS) where k_ILIS = 1200
 *       V_IS = ADC_value * 3.3V / 4096
 *       I_load = V_IS * 1200 (in mA with 1k sense resistor)
 */
Current_mA_t BTT6200_ADCToCurrent(uint16_t adcValue)
{
    Current_mA_t current_mA;

    /* Convert ADC to voltage in millivolts */
    uint32_t voltage_mV = BSP_ADC_ToMillivolts(adcValue);

    /* Calculate current: I = (V * SENSE_RATIO) / RSENSE
     * With 1k resistor and 1200:1 ratio:
     * I_load = V_IS * 1.2 (approximately)
     * For more accuracy: I_load_mA = voltage_mV * 1200 / 1000
     */
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

/**
 * @brief Get output channel state
 */
Status_t BTT6200_GetChannelState(uint8_t channel, BTT6200_ChannelState_t *pState)
{
    Status_t status = STATUS_ERROR_PARAM;

    if ((channel < BTT6200_TOTAL_CHANNELS) && (pState != NULL) && btt6200_initialized)
    {
        uint8_t icNum = channel / BTT6200_CHANNELS_PER_IC;
        uint8_t chNum = channel % BTT6200_CHANNELS_PER_IC;

        *pState = ic_status[icNum].state[chNum];
        status = STATUS_OK;
    }

    return status;
}

/**
 * @brief Enable/disable all outputs globally
 */
Status_t BTT6200_EnableOutputs(bool enable)
{
    Status_t status = STATUS_OK;

    if (btt6200_initialized)
    {
        if (enable)
        {
            /* Re-enable outputs based on stored configuration */
            for (uint8_t ch = 0U; ch < BTT6200_TOTAL_CHANNELS; ch++)
            {
                if (channel_config[ch].enabled)
                {
                    uint8_t icNum = ch / BTT6200_CHANNELS_PER_IC;
                    uint8_t chNum = ch % BTT6200_CHANNELS_PER_IC;

                    ic_status[icNum].state[chNum] = BTT6200_STATE_ON;
                    btt6200_set_channel_output(icNum, chNum, true);
                }
            }
        }
        else
        {
            /* Disable all outputs but keep configuration */
            for (uint8_t ic = 0U; ic < BTT6200_IC_COUNT; ic++)
            {
                btt6200_set_all_outputs(ic, 0x00U);

                for (uint8_t ch = 0U; ch < BTT6200_CHANNELS_PER_IC; ch++)
                {
                    if (ic_status[ic].state[ch] == BTT6200_STATE_ON)
                    {
                        ic_status[ic].state[ch] = BTT6200_STATE_OFF;
                    }
                }
            }
        }
    }
    else
    {
        status = STATUS_ERROR_NOT_INIT;
    }

    return status;
}

/**
 * @brief Configure output channel
 */
Status_t BTT6200_ConfigureChannel(uint8_t channel, const BTT6200_ChannelConfig_t *pConfig)
{
    Status_t status = STATUS_ERROR_PARAM;

    if ((channel < BTT6200_TOTAL_CHANNELS) && (pConfig != NULL) && btt6200_initialized)
    {
        channel_config[channel] = *pConfig;
        status = STATUS_OK;
    }

    return status;
}

/**
 * @brief Get IC status
 */
Status_t BTT6200_GetICStatus(uint8_t icNumber, BTT6200_IC_Status_t *pStatus)
{
    Status_t status = STATUS_ERROR_PARAM;

    if ((icNumber < BTT6200_IC_COUNT) && (pStatus != NULL) && btt6200_initialized)
    {
        *pStatus = ic_status[icNumber];
        status = STATUS_OK;
    }

    return status;
}

/**
 * @brief Get total fault count across all channels
 */
Status_t BTT6200_GetTotalFaultCount(uint32_t *pFaultCount)
{
    Status_t status = STATUS_ERROR_PARAM;

    if ((pFaultCount != NULL) && btt6200_initialized)
    {
        uint32_t total = 0U;

        for (uint8_t ic = 0U; ic < BTT6200_IC_COUNT; ic++)
        {
            total += ic_status[ic].faultCount;
        }

        *pFaultCount = total;
        status = STATUS_OK;
    }

    return status;
}

/**
 * @brief Clear channel fault
 */
Status_t BTT6200_ClearFault(uint8_t channel)
{
    Status_t status = STATUS_ERROR_PARAM;

    if ((channel < BTT6200_TOTAL_CHANNELS) && btt6200_initialized)
    {
        uint8_t icNum = channel / BTT6200_CHANNELS_PER_IC;
        uint8_t chNum = channel % BTT6200_CHANNELS_PER_IC;

        ic_status[icNum].diag[chNum].faultType = BTT6200_FAULT_NONE;
        ic_status[icNum].diag[chNum].openLoad = false;
        ic_status[icNum].diag[chNum].shortCircuit = false;
        ic_status[icNum].diag[chNum].overcurrent = false;

        if (ic_status[icNum].state[chNum] == BTT6200_STATE_FAULT)
        {
            ic_status[icNum].state[chNum] = BTT6200_STATE_OFF;
        }

        status = STATUS_OK;
    }

    return status;
}

/**
 * @brief Check if channel has active fault
 */
Status_t BTT6200_HasFault(uint8_t channel, bool *pHasFault)
{
    Status_t status = STATUS_ERROR_PARAM;

    if ((channel < BTT6200_TOTAL_CHANNELS) && (pHasFault != NULL) && btt6200_initialized)
    {
        uint8_t icNum = channel / BTT6200_CHANNELS_PER_IC;
        uint8_t chNum = channel % BTT6200_CHANNELS_PER_IC;

        *pHasFault = (ic_status[icNum].diag[chNum].faultType != BTT6200_FAULT_NONE);
        status = STATUS_OK;
    }

    return status;
}

/**
 * @brief Set current limit for channel
 */
Status_t BTT6200_SetCurrentLimit(uint8_t channel, uint16_t limit_mA)
{
    Status_t status = STATUS_ERROR_PARAM;

    if ((channel < BTT6200_TOTAL_CHANNELS) && (limit_mA <= BTT6200_MAX_CURRENT_MA) && btt6200_initialized)
    {
        channel_config[channel].currentLimit_mA = limit_mA;
        status = STATUS_OK;
    }

    return status;
}

/**
 * @brief Enable/disable auto-retry after fault
 */
Status_t BTT6200_SetAutoRetry(uint8_t channel, bool autoRetry)
{
    Status_t status = STATUS_ERROR_PARAM;

    if ((channel < BTT6200_TOTAL_CHANNELS) && btt6200_initialized)
    {
        channel_config[channel].autoRetry = autoRetry;
        status = STATUS_OK;
    }

    return status;
}

/*============================================================================*/
/* PRIVATE FUNCTIONS                                                          */
/*============================================================================*/

/**
 * @brief Write to a GPIO pin
 */
static void btt6200_write_pin(const GPIO_Pin_t *pPin, GPIO_PinState state)
{
    if (pPin != NULL)
    {
        HAL_GPIO_WritePin(pPin->port, pPin->pin, state);
    }
}

/**
 * @brief Read from a GPIO pin
 * @note Currently unused - reserved for future diagnostic feedback reading
 * @warning Suppress -Wunused-function warning: function intentionally kept for future use
 */
__attribute__((unused))
static GPIO_PinState btt6200_read_pin(const GPIO_Pin_t *pPin)
{
    GPIO_PinState state = GPIO_PIN_RESET;

    if (pPin != NULL)
    {
        state = HAL_GPIO_ReadPin(pPin->port, pPin->pin);
    }

    return state;
}

/**
 * @brief Set individual channel output
 */
static void btt6200_set_channel_output(uint8_t icNum, uint8_t channel, bool state)
{
    if ((icNum < BTT6200_IC_COUNT) && (channel < BTT6200_CHANNELS_PER_IC))
    {
        const BTT6200_IC_PinConfig_t *pPins = &ic_pins[icNum];
        GPIO_PinState pinState = state ? GPIO_PIN_SET : GPIO_PIN_RESET;

        switch (channel)
        {
            case 0U:
                btt6200_write_pin(&pPins->in0, pinState);
                break;
            case 1U:
                btt6200_write_pin(&pPins->in1, pinState);
                break;
            case 2U:
                btt6200_write_pin(&pPins->in2, pinState);
                break;
            case 3U:
                btt6200_write_pin(&pPins->in3, pinState);
                break;
            default:
                /* Invalid channel - do nothing */
                break;
        }
    }
}

/**
 * @brief Set all outputs for an IC using a bitmask
 */
static void btt6200_set_all_outputs(uint8_t icNum, uint8_t channelMask)
{
    if (icNum < BTT6200_IC_COUNT)
    {
        const BTT6200_IC_PinConfig_t *pPins = &ic_pins[icNum];

        btt6200_write_pin(&pPins->in0, ((channelMask & 0x01U) != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        btt6200_write_pin(&pPins->in1, ((channelMask & 0x02U) != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        btt6200_write_pin(&pPins->in2, ((channelMask & 0x04U) != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        btt6200_write_pin(&pPins->in3, ((channelMask & 0x08U) != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }
}

/**
 * @brief Enable/disable diagnostic mode for an IC
 */
static void btt6200_enable_diagnostics(uint8_t icNum, bool enable)
{
    if (icNum < BTT6200_IC_COUNT)
    {
        const BTT6200_IC_PinConfig_t *pPins = &ic_pins[icNum];
        btt6200_write_pin(&pPins->den, enable ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }
}

/**
 * @brief Select diagnostic channel
 * @param icNum IC number (0-4)
 * @param channel Channel to diagnose (0-3)
 *
 * DSEL1 DSEL0 | Selected Channel
 * -----------+------------------
 *   0    0   | Channel 0
 *   0    1   | Channel 1
 *   1    0   | Channel 2
 *   1    1   | Channel 3
 */
static void btt6200_select_diag_channel(uint8_t icNum, uint8_t channel)
{
    if ((icNum < BTT6200_IC_COUNT) && (channel < BTT6200_CHANNELS_PER_IC))
    {
        const BTT6200_IC_PinConfig_t *pPins = &ic_pins[icNum];

        btt6200_write_pin(&pPins->dsel0, ((channel & 0x01U) != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        btt6200_write_pin(&pPins->dsel1, ((channel & 0x02U) != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }
}

/**
 * @brief Read current from IC's IS pin
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
 * @brief Internal diagnostics for a channel
 */
static void btt6200_diagnose_channel_internal(uint8_t channel)
{
    if (channel < BTT6200_TOTAL_CHANNELS)
    {
        uint8_t icNum = channel / BTT6200_CHANNELS_PER_IC;
        uint8_t chNum = channel % BTT6200_CHANNELS_PER_IC;

        /* Enable diagnostics and select channel */
        btt6200_enable_diagnostics(icNum, true);
        btt6200_select_diag_channel(icNum, chNum);

        /* Small delay for diagnostic output to settle */
        for (volatile uint32_t i = 0U; i < 1000U; i++) { }

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
                ic_status[icNum].faultCount++;

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
            if ((ic_status[icNum].state[chNum] == BTT6200_STATE_ON) &&
                (current < BTT6200_OPEN_LOAD_THRESHOLD_MA))
            {
                ic_status[icNum].diag[chNum].openLoad = true;
                ic_status[icNum].diag[chNum].faultType = BTT6200_FAULT_OPEN_LOAD;
                ic_status[icNum].diag[chNum].faultCount++;
                ic_status[icNum].faultCount++;

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
            if (current > BTT6200_SHORT_CIRCUIT_THRESHOLD_MA)
            {
                ic_status[icNum].diag[chNum].shortCircuit = true;
                ic_status[icNum].diag[chNum].faultType = BTT6200_FAULT_SHORT_CIRCUIT;
                ic_status[icNum].diag[chNum].faultCount++;
                ic_status[icNum].faultCount++;
                ic_status[icNum].state[chNum] = BTT6200_STATE_FAULT;

                /* Turn off channel immediately */
                btt6200_set_channel_output(icNum, chNum, false);

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

        /* Disable diagnostics */
        btt6200_enable_diagnostics(icNum, false);
    }
}

/*============================================================================*/
/* END OF FILE                                                                */
/*============================================================================*/
