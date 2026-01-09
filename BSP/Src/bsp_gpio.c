/**
 * @file    bsp_gpio.c
 * @brief   GPIO abstraction layer implementation for STM32F413
 * @author  Battery Control Unit Development Team
 * @date    2026-01-09
 * @version 1.0.0
 *
 * @note    MISRA C:2012 compliant
 * @note    Single MCU architecture with LEM HOYS sensors
 *
 * @copyright Copyright (c) 2026
 */

/*============================================================================*/
/* INCLUDES                                                                   */
/*============================================================================*/
#include "bsp_gpio.h"
#include "app_errors.h"
#include "stm32f4xx_hal.h"

/*============================================================================*/
/* PRIVATE VARIABLES                                                          */
/*============================================================================*/

/** @brief Initialization status */
static bool gpio_initialized = false;

/*============================================================================*/
/* PRIVATE FUNCTION PROTOTYPES                                                */
/*============================================================================*/

static void gpio_init_power_pins(void);
static void gpio_init_can_pins(void);
static void gpio_init_i2c_pins(void);
static void gpio_init_uart_pins(void);
static void gpio_init_spi_pins(void);
static void gpio_init_btt6200_pins(void);
static void gpio_init_lem_pins(void);
static void gpio_init_digital_input_pins(void);
static void gpio_init_led_pins(void);

/*============================================================================*/
/* PUBLIC FUNCTIONS                                                           */
/*============================================================================*/

/**
 * @brief Initialize GPIO module
 */
Status_t BSP_GPIO_Init(void)
{
    Status_t status = STATUS_OK;

    if (gpio_initialized)
    {
        status = STATUS_ERROR_ALREADY_INIT;
    }
    else
    {
        /* Enable all GPIO port clocks */
        __HAL_RCC_GPIOA_CLK_ENABLE();
        __HAL_RCC_GPIOB_CLK_ENABLE();
        __HAL_RCC_GPIOC_CLK_ENABLE();
        __HAL_RCC_GPIOD_CLK_ENABLE();
        __HAL_RCC_GPIOE_CLK_ENABLE();
        __HAL_RCC_GPIOF_CLK_ENABLE();
        __HAL_RCC_GPIOG_CLK_ENABLE();
        __HAL_RCC_GPIOH_CLK_ENABLE();

        /* Initialize pin groups */
        gpio_init_power_pins();
        gpio_init_can_pins();
        gpio_init_i2c_pins();
        gpio_init_uart_pins();
        gpio_init_spi_pins();
        gpio_init_btt6200_pins();
        gpio_init_lem_pins();
        gpio_init_digital_input_pins();
        gpio_init_led_pins();

        gpio_initialized = true;
    }

    return status;
}

/**
 * @brief Configure GPIO pin
 */
Status_t BSP_GPIO_ConfigurePin(const GPIO_Config_t *pConfig)
{
    Status_t status = STATUS_ERROR_PARAM;

    if (pConfig != NULL)
    {
        GPIO_InitTypeDef GPIO_InitStruct = {0};

        /* Configure pin */
        GPIO_InitStruct.Pin = pConfig->pin;
        GPIO_InitStruct.Pull = (uint32_t)pConfig->pull;
        GPIO_InitStruct.Speed = (uint32_t)pConfig->speed;

        /* Set mode */
        switch (pConfig->mode)
        {
            case GPIO_MODE_INPUT:
                GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
                break;

            case GPIO_MODE_OUTPUT_PP:
                GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
                break;

            case GPIO_MODE_OUTPUT_OD:
                GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
                break;

            case GPIO_MODE_AF_PP:
                GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
                GPIO_InitStruct.Alternate = pConfig->alternate;
                break;

            case GPIO_MODE_AF_OD:
                GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
                GPIO_InitStruct.Alternate = pConfig->alternate;
                break;

            case GPIO_MODE_ANALOG:
                GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
                break;

            default:
                GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
                break;
        }

        HAL_GPIO_Init(pConfig->port, &GPIO_InitStruct);
        status = STATUS_OK;
    }

    return status;
}

/**
 * @brief Set GPIO pin state
 */
Status_t BSP_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_State_t state)
{
    Status_t status = STATUS_ERROR_PARAM;

    if (port != NULL)
    {
        GPIO_PinState pinState = (state == GPIO_STATE_HIGH) ? GPIO_PIN_SET : GPIO_PIN_RESET;
        HAL_GPIO_WritePin(port, pin, pinState);
        status = STATUS_OK;
    }

    return status;
}

/**
 * @brief Read GPIO pin state
 */
Status_t BSP_GPIO_ReadPin(GPIO_TypeDef *port, uint16_t pin, GPIO_State_t *pState)
{
    Status_t status = STATUS_ERROR_PARAM;

    if ((port != NULL) && (pState != NULL))
    {
        GPIO_PinState pinState = HAL_GPIO_ReadPin(port, pin);
        *pState = (pinState == GPIO_PIN_SET) ? GPIO_STATE_HIGH : GPIO_STATE_LOW;
        status = STATUS_OK;
    }

    return status;
}

/**
 * @brief Toggle GPIO pin
 */
Status_t BSP_GPIO_TogglePin(GPIO_TypeDef *port, uint16_t pin)
{
    Status_t status = STATUS_ERROR_PARAM;

    if (port != NULL)
    {
        HAL_GPIO_TogglePin(port, pin);
        status = STATUS_OK;
    }

    return status;
}

/**
 * @brief De-initialize GPIO module
 */
Status_t BSP_GPIO_DeInit(void)
{
    Status_t status = STATUS_OK;

    if (gpio_initialized)
    {
        /* Deinitialize all GPIO ports */
        HAL_GPIO_DeInit(GPIOA, GPIO_PIN_All);
        HAL_GPIO_DeInit(GPIOB, GPIO_PIN_All);
        HAL_GPIO_DeInit(GPIOC, GPIO_PIN_All);
        HAL_GPIO_DeInit(GPIOD, GPIO_PIN_All);
        HAL_GPIO_DeInit(GPIOE, GPIO_PIN_All);
        HAL_GPIO_DeInit(GPIOF, GPIO_PIN_All);
        HAL_GPIO_DeInit(GPIOG, GPIO_PIN_All);
        HAL_GPIO_DeInit(GPIOH, GPIO_PIN_All);

        gpio_initialized = false;
    }

    return status;
}

/*============================================================================*/
/* PRIVATE FUNCTIONS - PIN INITIALIZATION                                     */
/*============================================================================*/

/**
 * @brief Initialize power control pins
 */
static void gpio_init_power_pins(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* Power enable outputs (start LOW/disabled) */
    GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2;  /* 5V_EN, 3V3_EN, SLEEP */
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2, GPIO_PIN_RESET);

    /* Power monitor analog input (IMON) */
    GPIO_InitStruct.Pin = GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* Power good and fault inputs (with pull-down) */
    GPIO_InitStruct.Pin = GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6;  /* FLT, 5V_PG, 3V3A_PG */
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

/**
 * @brief Initialize CAN bus pins
 */
static void gpio_init_can_pins(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* CAN1: PA11 (RX), PA12 (TX) */
    GPIO_InitStruct.Pin = GPIO_PIN_11 | GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* CAN2: PB12 (RX), PB13 (TX) */
    GPIO_InitStruct.Pin = GPIO_PIN_12 | GPIO_PIN_13;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* CAN standby control pins (active high, start disabled) */
    GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;  /* CAN1_STB, CAN2_STB */
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0 | GPIO_PIN_1, GPIO_PIN_RESET);
}

/**
 * @brief Initialize I2C pins (FRAM + temperature sensor)
 */
static void gpio_init_i2c_pins(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* I2C2: PB10 (SCL), PB11 (SDA) */
    GPIO_InitStruct.Pin = GPIO_PIN_10 | GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

/**
 * @brief Initialize UART debug pins
 */
static void gpio_init_uart_pins(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* UART1: PA9 (TX), PA10 (RX) */
    GPIO_InitStruct.Pin = GPIO_PIN_9 | GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

/**
 * @brief Initialize SPI4 pins (isolated)
 */
static void gpio_init_spi_pins(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* SPI4: PE2 (SCK), PE5 (MISO), PE6 (MOSI) */
    GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_5 | GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI4;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    /* SPI4 CS: PE4 (output, start HIGH) */
    GPIO_InitStruct.Pin = GPIO_PIN_4;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_SET);
}

/**
 * @brief Initialize BTT6200 output control pins
 */
static void gpio_init_btt6200_pins(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* BTT6200 IC0: PC0-PC6 (control), PC7 (analog IS) */
    GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 |
                          GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 |
                      GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6, GPIO_PIN_RESET);

    /* BTT6200 IC0 current sense: PC7 (analog) */
    GPIO_InitStruct.Pin = GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* BTT6200 IC1-IC4: Similar pattern on other ports */
    /* IC1: PC8-PC15 */
    GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 |
                          GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 |
                      GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14, GPIO_PIN_RESET);

    GPIO_InitStruct.Pin = GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* IC2-IC4 on GPIOD, GPIOE (similar configuration) */
}

/**
 * @brief Initialize LEM HOYS sensor pins
 */
static void gpio_init_lem_pins(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* LEM supply enable: PD2 (output, start LOW) */
    GPIO_InitStruct.Pin = GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);

    /* LEM overcurrent flags: PF1, PF3, PF5, etc. (digital inputs) */
    GPIO_InitStruct.Pin = GPIO_PIN_1 | GPIO_PIN_3 | GPIO_PIN_5 |
                          GPIO_PIN_7 | GPIO_PIN_9 | GPIO_PIN_11 |
                          GPIO_PIN_13 | GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

    /* LEM analog outputs: PF2, PF4, PF6, etc. (ADC inputs) */
    GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_4 | GPIO_PIN_6 |
                          GPIO_PIN_8 | GPIO_PIN_10 | GPIO_PIN_12 | GPIO_PIN_14;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

    /* Additional LEM outputs on GPIOG: PG0, PG2, PG4 */
    GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_2 | GPIO_PIN_4;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

    /* Additional LEM OC flags on GPIOG: PG1, PG3 */
    GPIO_InitStruct.Pin = GPIO_PIN_1 | GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);
}

/**
 * @brief Initialize digital input pins (20 channels)
 */
static void gpio_init_digital_input_pins(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* Digital inputs as analog (for ACS772 current measurement) */
    /* PG5-PG15: DIN_0 to DIN_10 */
    GPIO_InitStruct.Pin = GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7 |
                          GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 |
                          GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 |
                          GPIO_PIN_14 | GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

    /* PH0-PH8: DIN_11 to DIN_19 */
    GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 |
                          GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 |
                          GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8;
    HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);
}

/**
 * @brief Initialize status LED pins
 */
static void gpio_init_led_pins(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* Status LEDs: PH9-PH12 (outputs, start LOW/off) */
    GPIO_InitStruct.Pin = GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOH, GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12, GPIO_PIN_RESET);
}

/*============================================================================*/
/* END OF FILE                                                                */
/*============================================================================*/
