/**
 * @file    digital_input.c
 * @brief   Digital input driver with ACS772 current sensing
 * @author  Battery Control Unit Development Team
 * @date    2026-01-09
 * @version 1.0.0
 *
 * @note    MISRA C:2012 compliant
 * @note    20 digital inputs with debouncing
 * @note    ACS772 current sensing for diagnostics
 *
 * @copyright Copyright (c) 2026
 */

/*============================================================================*/
/* INCLUDES                                                                   */
/*============================================================================*/
#include "digital_input.h"
#include "bsp_gpio.h"
#include "bsp_adc.h"
#include "filter.h"
#include <string.h>

/*============================================================================*/
/* PRIVATE CONSTANTS                                                          */
/*============================================================================*/

/** @brief Number of digital inputs */
#define DI_COUNT                (20U)

/** @brief Debounce time (milliseconds) */
#define DI_DEBOUNCE_TIME_MS     (20U)

/** @brief ACS772 sensitivity (mV per A) */
#define ACS772_SENSITIVITY_MV   (40U)  /* 40 mV/A for ACS772-050U */

/** @brief ACS772 zero current voltage (mV) */
#define ACS772_ZERO_CURRENT_MV  (2500U)  /* VCC/2 @ 5V supply */

/** @brief ADC channel mapping for current sense */
static const uint8_t DI_CURRENT_SENSE_ADC_CHANNELS[DI_COUNT] = {
    /* Channels mapped to ACS772 outputs */
    16, 17, 18, 19, 20, 21, 22, 23, 24, 25,
    26, 27, 28, 29, 30, 31, 32, 33, 34, 35
};

/** @brief GPIO pin mapping for digital inputs */
typedef struct {
    GPIO_TypeDef *port;
    uint16_t pin;
} DI_PinMap_t;

/** @brief Digital input pin mapping */
static const DI_PinMap_t di_pin_map[DI_COUNT] = {
    /* Port A inputs */
    {GPIOA, GPIO_PIN_0},  /* DI_0 */
    {GPIOA, GPIO_PIN_1},  /* DI_1 */
    {GPIOA, GPIO_PIN_2},  /* DI_2 */
    {GPIOA, GPIO_PIN_8},  /* DI_3 */
    {GPIOA, GPIO_PIN_9},  /* DI_4 */
    /* Port B inputs */
    {GPIOB, GPIO_PIN_0},  /* DI_5 */
    {GPIOB, GPIO_PIN_1},  /* DI_6 */
    {GPIOB, GPIO_PIN_2},  /* DI_7 */
    {GPIOB, GPIO_PIN_10}, /* DI_8 */
    {GPIOB, GPIO_PIN_11}, /* DI_9 */
    {GPIOB, GPIO_PIN_12}, /* DI_10 */
    {GPIOB, GPIO_PIN_13}, /* DI_11 */
    {GPIOB, GPIO_PIN_14}, /* DI_12 */
    {GPIOB, GPIO_PIN_15}, /* DI_13 */
    /* Port C inputs */
    {GPIOC, GPIO_PIN_7},  /* DI_14 */
    {GPIOC, GPIO_PIN_15}, /* DI_15 */
    /* Port D inputs */
    {GPIOD, GPIO_PIN_7},  /* DI_16 */
    {GPIOD, GPIO_PIN_15}, /* DI_17 */
    /* Port E inputs */
    {GPIOE, GPIO_PIN_2},  /* DI_18 */
    {GPIOE, GPIO_PIN_4}   /* DI_19 */
};

/*============================================================================*/
/* PRIVATE VARIABLES                                                          */
/*============================================================================*/

/** @brief Input configurations */
static DI_InputConfig_t di_config[DI_COUNT];

/** @brief Input states */
static DI_InputState_t di_state[DI_COUNT];

/** @brief Debounce filters */
static Filter_Debounce_t di_debounce[DI_COUNT];

/** @brief State change callbacks */
static DI_StateChangeCallback_t di_callbacks[DI_COUNT];

/** @brief Initialization flag */
static bool di_initialized = false;

/*============================================================================*/
/* PRIVATE FUNCTION PROTOTYPES                                                */
/*============================================================================*/

static Status_t di_read_gpio(uint8_t inputId, bool *pState);
static Status_t di_read_current(uint8_t inputId, Current_mA_t *pCurrent);
static void di_check_diagnostics(uint8_t inputId);

/*============================================================================*/
/* PUBLIC FUNCTIONS                                                           */
/*============================================================================*/

/**
 * @brief Initialize digital input module
 */
Status_t DI_Init(void)
{
    Status_t status = STATUS_OK;

    if (di_initialized)
    {
        status = STATUS_ERROR_ALREADY_INIT;
    }
    else
    {
        uint8_t i;

        /* Initialize all inputs */
        for (i = 0U; i < DI_COUNT; i++)
        {
            /* Default configuration */
            di_config[i].enabled = true;
            di_config[i].activeLow = false;
            di_config[i].pullupEnabled = true;
            di_config[i].currentMonitoringEnabled = false;
            di_config[i].currentLimit_mA = 5000;  /* 5A default */

            /* Initialize state */
            (void)memset(&di_state[i], 0, sizeof(DI_InputState_t));
            di_state[i].status = DI_STATUS_OK;

            /* Initialize debounce filter */
            (void)Filter_Debounce_Init(&di_debounce[i], DI_DEBOUNCE_TIME_MS);

            /* No callback by default */
            di_callbacks[i] = NULL;
        }

        di_initialized = true;
    }

    return status;
}

/**
 * @brief Configure digital input
 */
Status_t DI_ConfigureInput(uint8_t inputId, const DI_InputConfig_t *pConfig)
{
    Status_t status = STATUS_ERROR_PARAM;

    if ((inputId < DI_COUNT) && (pConfig != NULL) && di_initialized)
    {
        di_config[inputId] = *pConfig;
        status = STATUS_OK;
    }

    return status;
}

/**
 * @brief Read digital input state
 */
Status_t DI_ReadInput(uint8_t inputId, bool *pState)
{
    Status_t status = STATUS_ERROR_PARAM;

    if ((inputId < DI_COUNT) && (pState != NULL) && di_initialized)
    {
        if (di_config[inputId].enabled)
        {
            bool rawState;
            status = di_read_gpio(inputId, &rawState);

            if (status == STATUS_OK)
            {
                /* Apply debouncing */
                bool debouncedState = Filter_Debounce_Update(&di_debounce[inputId], rawState);

                /* Apply active-low logic */
                if (di_config[inputId].activeLow)
                {
                    debouncedState = !debouncedState;
                }

                /* Update state */
                bool oldState = di_state[inputId].currentState;
                di_state[inputId].currentState = debouncedState;
                di_state[inputId].rawState = rawState;

                /* Detect edge */
                if (debouncedState != oldState)
                {
                    di_state[inputId].edgeDetected = true;
                    di_state[inputId].risingEdge = debouncedState && !oldState;
                    di_state[inputId].fallingEdge = !debouncedState && oldState;
                    di_state[inputId].lastChangeTime_ms = HAL_GetTick();
                    di_state[inputId].changeCount++;

                    /* Call callback if registered */
                    if (di_callbacks[inputId] != NULL)
                    {
                        di_callbacks[inputId](inputId, debouncedState);
                    }
                }
                else
                {
                    di_state[inputId].edgeDetected = false;
                    di_state[inputId].risingEdge = false;
                    di_state[inputId].fallingEdge = false;
                }

                *pState = debouncedState;
            }
        }
        else
        {
            status = STATUS_ERROR_DISABLED;
        }
    }

    return status;
}

/**
 * @brief Get input state (full status)
 */
Status_t DI_GetInputState(uint8_t inputId, DI_InputState_t *pState)
{
    Status_t status = STATUS_ERROR_PARAM;

    if ((inputId < DI_COUNT) && (pState != NULL) && di_initialized)
    {
        *pState = di_state[inputId];
        status = STATUS_OK;
    }

    return status;
}

/**
 * @brief Read input current (via ACS772)
 */
Status_t DI_ReadInputCurrent(uint8_t inputId, Current_mA_t *pCurrent_mA)
{
    Status_t status = STATUS_ERROR_PARAM;

    if ((inputId < DI_COUNT) && (pCurrent_mA != NULL) && di_initialized)
    {
        if (di_config[inputId].currentMonitoringEnabled)
        {
            status = di_read_current(inputId, pCurrent_mA);

            if (status == STATUS_OK)
            {
                di_state[inputId].current_mA = *pCurrent_mA;
            }
        }
        else
        {
            status = STATUS_ERROR_DISABLED;
        }
    }

    return status;
}

/**
 * @brief Check if input edge was detected
 */
Status_t DI_GetEdgeDetection(uint8_t inputId, bool *pRisingEdge, bool *pFallingEdge)
{
    Status_t status = STATUS_ERROR_PARAM;

    if ((inputId < DI_COUNT) && (pRisingEdge != NULL) && (pFallingEdge != NULL) && di_initialized)
    {
        *pRisingEdge = di_state[inputId].risingEdge;
        *pFallingEdge = di_state[inputId].fallingEdge;
        status = STATUS_OK;
    }

    return status;
}

/**
 * @brief Register state change callback
 */
Status_t DI_RegisterCallback(uint8_t inputId, DI_StateChangeCallback_t callback)
{
    Status_t status = STATUS_ERROR_PARAM;

    if ((inputId < DI_COUNT) && di_initialized)
    {
        di_callbacks[inputId] = callback;
        status = STATUS_OK;
    }

    return status;
}

/**
 * @brief Update all digital inputs (call periodically @ 10ms)
 */
Status_t DI_Update(void)
{
    Status_t status = STATUS_OK;

    if (di_initialized)
    {
        uint8_t i;

        for (i = 0U; i < DI_COUNT; i++)
        {
            if (di_config[i].enabled)
            {
                bool state;
                (void)DI_ReadInput(i, &state);

                /* Perform diagnostics if enabled */
                if (di_config[i].currentMonitoringEnabled)
                {
                    di_check_diagnostics(i);
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
 * @brief Reset input statistics
 */
Status_t DI_ResetStatistics(uint8_t inputId)
{
    Status_t status = STATUS_ERROR_PARAM;

    if ((inputId < DI_COUNT) && di_initialized)
    {
        di_state[inputId].changeCount = 0U;
        di_state[inputId].faultCount = 0U;
        di_state[inputId].lastChangeTime_ms = 0U;
        status = STATUS_OK;
    }

    return status;
}

/**
 * @brief Convert ADC to current (ACS772)
 */
Current_mA_t DI_ADCToCurrent(uint16_t adcValue)
{
    Current_mA_t current_mA;

    /* Convert ADC to voltage */
    uint32_t voltage_mV = BSP_ADC_ToMillivolts(adcValue);

    /* Calculate voltage offset from zero point */
    int32_t voltage_offset = (int32_t)voltage_mV - (int32_t)ACS772_ZERO_CURRENT_MV;

    /* Convert to current: I = V / sensitivity */
    current_mA = (voltage_offset * 1000) / (int32_t)ACS772_SENSITIVITY_MV;

    return current_mA;
}

/**
 * @brief De-initialize digital input module
 */
Status_t DI_DeInit(void)
{
    if (di_initialized)
    {
        di_initialized = false;
    }

    return STATUS_OK;
}

/*============================================================================*/
/* PRIVATE FUNCTIONS                                                          */
/*============================================================================*/

/**
 * @brief Read GPIO pin state
 */
static Status_t di_read_gpio(uint8_t inputId, bool *pState)
{
    Status_t status = STATUS_ERROR_PARAM;

    if ((inputId < DI_COUNT) && (pState != NULL))
    {
        const DI_PinMap_t *pin = &di_pin_map[inputId];
        GPIO_State_t gpioState;

        status = BSP_GPIO_ReadPin(pin->port, pin->pin, &gpioState);

        if (status == STATUS_OK)
        {
            *pState = (gpioState == GPIO_STATE_HIGH);
        }
    }

    return status;
}

/**
 * @brief Read input current via ACS772
 */
static Status_t di_read_current(uint8_t inputId, Current_mA_t *pCurrent)
{
    Status_t status = STATUS_ERROR_PARAM;

    if ((inputId < DI_COUNT) && (pCurrent != NULL))
    {
        uint16_t adcValue;
        uint8_t adcChannel = DI_CURRENT_SENSE_ADC_CHANNELS[inputId];

        status = BSP_ADC_ReadChannel(adcChannel, &adcValue, TIMEOUT_ADC_MS);

        if (status == STATUS_OK)
        {
            *pCurrent = DI_ADCToCurrent(adcValue);
        }
    }

    return status;
}

/**
 * @brief Check input diagnostics
 */
static void di_check_diagnostics(uint8_t inputId)
{
    if (inputId < DI_COUNT)
    {
        Current_mA_t current;

        if (di_read_current(inputId, &current) == STATUS_OK)
        {
            /* Check overcurrent */
            if ((current > di_config[inputId].currentLimit_mA) ||
                (current < -di_config[inputId].currentLimit_mA))
            {
                di_state[inputId].status = DI_STATUS_OVERCURRENT;
                di_state[inputId].faultCount++;
            }
            /* Check open circuit (current too low when active) */
            else if (di_state[inputId].currentState && (current < 10))
            {
                di_state[inputId].status = DI_STATUS_OPEN_CIRCUIT;
                di_state[inputId].faultCount++;
            }
            /* Check short circuit (current too high immediately) */
            else if (current > (di_config[inputId].currentLimit_mA * 110 / 100))
            {
                di_state[inputId].status = DI_STATUS_SHORT_CIRCUIT;
                di_state[inputId].faultCount++;
            }
            else
            {
                di_state[inputId].status = DI_STATUS_OK;
            }
        }
    }
}

/*============================================================================*/
/* END OF FILE                                                                */
/*============================================================================*/
