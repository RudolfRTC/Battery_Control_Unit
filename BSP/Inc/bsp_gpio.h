/**
 * @file    bsp_gpio.h
 * @brief   GPIO abstraction layer for STM32F413
 * @author  Battery Control Unit Development Team
 * @date    2026-01-09
 * @version 1.0.0
 *
 * @note    MISRA C:2012 compliant
 * @note    Hardware abstraction for all GPIO pins
 *
 * @copyright Copyright (c) 2026
 */

#ifndef BSP_GPIO_H
#define BSP_GPIO_H

#ifdef __cplusplus
extern "C" {
#endif

/*============================================================================*/
/* INCLUDES                                                                   */
/*============================================================================*/
#include "app_types.h"
#include "stm32f4xx_hal.h"

/*============================================================================*/
/* GPIO PIN DEFINITIONS                                                       */
/*============================================================================*/

/** @brief Power control pins */
#define GPIO_PIN_POWER_5V_EN          GPIOA, GPIO_PIN_0   /**< 5V enable */
#define GPIO_PIN_POWER_3V3_EN         GPIOA, GPIO_PIN_1   /**< 3.3V enable */
#define GPIO_PIN_POWER_SLEEP          GPIOA, GPIO_PIN_2   /**< LM74900 sleep */
#define GPIO_PIN_POWER_IMON           GPIOA, GPIO_PIN_3   /**< Current monitor (analog) */
#define GPIO_PIN_POWER_FLT            GPIOA, GPIO_PIN_4   /**< Fault input */
#define GPIO_PIN_POWER_5V_PG          GPIOA, GPIO_PIN_5   /**< 5V power good */
#define GPIO_PIN_POWER_3V3A_PG        GPIOA, GPIO_PIN_6   /**< 3.3V analog PG */

/** @brief CAN bus pins */
#define GPIO_PIN_CAN1_TX              GPIOA, GPIO_PIN_12  /**< CAN1 TX */
#define GPIO_PIN_CAN1_RX              GPIOA, GPIO_PIN_11  /**< CAN1 RX */
#define GPIO_PIN_CAN1_STB             GPIOB, GPIO_PIN_0   /**< CAN1 standby */
#define GPIO_PIN_CAN2_TX              GPIOB, GPIO_PIN_13  /**< CAN2 TX */
#define GPIO_PIN_CAN2_RX              GPIOB, GPIO_PIN_12  /**< CAN2 RX */
#define GPIO_PIN_CAN2_STB             GPIOB, GPIO_PIN_1   /**< CAN2 standby */

/** @brief SPI4 pins (isolated) */
#define GPIO_PIN_SPI4_SCK             GPIOE, GPIO_PIN_2   /**< SPI4 clock */
#define GPIO_PIN_SPI4_MISO            GPIOE, GPIO_PIN_5   /**< SPI4 MISO */
#define GPIO_PIN_SPI4_MOSI            GPIOE, GPIO_PIN_6   /**< SPI4 MOSI */
#define GPIO_PIN_SPI4_CS              GPIOE, GPIO_PIN_4   /**< SPI4 chip select */

/** @brief I2C2 pins */
#define GPIO_PIN_I2C2_SCL             GPIOB, GPIO_PIN_10  /**< I2C2 clock */
#define GPIO_PIN_I2C2_SDA             GPIOB, GPIO_PIN_11  /**< I2C2 data */

/** @brief UART1 pins (debug) */
#define GPIO_PIN_UART1_TX             GPIOA, GPIO_PIN_9   /**< UART1 TX */
#define GPIO_PIN_UART1_RX             GPIOA, GPIO_PIN_10  /**< UART1 RX */

/** @brief BTT6200 output control pins (5 ICs) */
#define GPIO_PIN_BTT_DEN_0            GPIOC, GPIO_PIN_0   /**< IC0 diagnostic enable */
#define GPIO_PIN_BTT_DSEL0_0          GPIOC, GPIO_PIN_1   /**< IC0 diagnostic select 0 */
#define GPIO_PIN_BTT_DSEL1_0          GPIOC, GPIO_PIN_2   /**< IC0 diagnostic select 1 */
#define GPIO_PIN_BTT_IN0_0            GPIOC, GPIO_PIN_3   /**< IC0 input 0 */
#define GPIO_PIN_BTT_IN1_0            GPIOC, GPIO_PIN_4   /**< IC0 input 1 */
#define GPIO_PIN_BTT_IN2_0            GPIOC, GPIO_PIN_5   /**< IC0 input 2 */
#define GPIO_PIN_BTT_IN3_0            GPIOC, GPIO_PIN_6   /**< IC0 input 3 */
#define GPIO_PIN_BTT_IS_0             GPIOC, GPIO_PIN_7   /**< IC0 current sense (analog) */

#define GPIO_PIN_BTT_DEN_1            GPIOC, GPIO_PIN_8   /**< IC1 diagnostic enable */
#define GPIO_PIN_BTT_DSEL0_1          GPIOC, GPIO_PIN_9   /**< IC1 diagnostic select 0 */
#define GPIO_PIN_BTT_DSEL1_1          GPIOC, GPIO_PIN_10  /**< IC1 diagnostic select 1 */
#define GPIO_PIN_BTT_IN0_1            GPIOC, GPIO_PIN_11  /**< IC1 input 0 */
#define GPIO_PIN_BTT_IN1_1            GPIOC, GPIO_PIN_12  /**< IC1 input 1 */
#define GPIO_PIN_BTT_IN2_1            GPIOC, GPIO_PIN_13  /**< IC1 input 2 */
#define GPIO_PIN_BTT_IN3_1            GPIOC, GPIO_PIN_14  /**< IC1 input 3 */
#define GPIO_PIN_BTT_IS_1             GPIOC, GPIO_PIN_15  /**< IC1 current sense (analog) */

/* IC2, IC3, IC4 similar pattern on GPIOD, GPIOE, GPIOF */

/** @brief LEM current sensor pins */
#define GPIO_PIN_LEM_OC_0             GPIOD, GPIO_PIN_0   /**< LEM 0 overcurrent */
#define GPIO_PIN_LEM_OUT_0            GPIOD, GPIO_PIN_1   /**< LEM 0 output (analog) */
#define GPIO_PIN_LEM_SUPPLY           GPIOD, GPIO_PIN_2   /**< LEM supply control */

/** @brief Digital input pins (20 channels) */
#define GPIO_PIN_DIN_0                GPIOE, GPIO_PIN_0   /**< Digital input 0 */
#define GPIO_PIN_DIN_1                GPIOE, GPIO_PIN_1   /**< Digital input 1 */
/* ... DIN 2-19 follow similar pattern */

/** @brief LED indicators */
#define GPIO_PIN_LED_STATUS           GPIOF, GPIO_PIN_0   /**< Status LED */
#define GPIO_PIN_LED_ERROR            GPIOF, GPIO_PIN_1   /**< Error LED */
#define GPIO_PIN_LED_CAN1             GPIOF, GPIO_PIN_2   /**< CAN1 activity LED */
#define GPIO_PIN_LED_CAN2             GPIOF, GPIO_PIN_3   /**< CAN2 activity LED */

/*============================================================================*/
/* TYPE DEFINITIONS                                                           */
/*============================================================================*/

/**
 * @brief GPIO pin state
 */
typedef enum {
    GPIO_STATE_LOW  = 0U,  /**< Pin low (0V) */
    GPIO_STATE_HIGH = 1U   /**< Pin high (3.3V) */
} GPIO_State_t;

/**
 * @brief GPIO pin mode
 */
typedef enum {
    GPIO_MODE_INPUT        = 0x00U,  /**< Input mode */
    GPIO_MODE_OUTPUT_PP    = 0x01U,  /**< Output push-pull */
    GPIO_MODE_OUTPUT_OD    = 0x02U,  /**< Output open-drain */
    GPIO_MODE_AF_PP        = 0x03U,  /**< Alternate function push-pull */
    GPIO_MODE_AF_OD        = 0x04U,  /**< Alternate function open-drain */
    GPIO_MODE_ANALOG       = 0x05U   /**< Analog mode */
} GPIO_Mode_t;

/**
 * @brief GPIO pull-up/pull-down
 */
typedef enum {
    GPIO_PULL_NONE = 0x00U,  /**< No pull */
    GPIO_PULL_UP   = 0x01U,  /**< Pull-up */
    GPIO_PULL_DOWN = 0x02U   /**< Pull-down */
} GPIO_Pull_t;

/**
 * @brief GPIO speed
 */
typedef enum {
    GPIO_SPEED_LOW    = 0x00U,  /**< Low speed (2 MHz) */
    GPIO_SPEED_MEDIUM = 0x01U,  /**< Medium speed (25 MHz) */
    GPIO_SPEED_HIGH   = 0x02U,  /**< High speed (50 MHz) */
    GPIO_SPEED_VERY_HIGH = 0x03U /**< Very high speed (100 MHz) */
} GPIO_Speed_t;

/**
 * @brief GPIO configuration structure
 */
typedef struct {
    GPIO_TypeDef *port;      /**< GPIO port (GPIOA, GPIOB, etc.) */
    uint16_t     pin;        /**< GPIO pin (GPIO_PIN_0, etc.) */
    GPIO_Mode_t  mode;       /**< Pin mode */
    GPIO_Pull_t  pull;       /**< Pull-up/down */
    GPIO_Speed_t speed;      /**< Output speed */
    uint8_t      alternate;  /**< Alternate function (0-15) */
} GPIO_Config_t;

/*============================================================================*/
/* FUNCTION PROTOTYPES                                                        */
/*============================================================================*/

/**
 * @brief Initialize GPIO module
 * @return STATUS_OK on success
 * @note  Enables all GPIO port clocks
 */
Status_t BSP_GPIO_Init(void);

/**
 * @brief Configure GPIO pin
 * @param[in] pConfig Pointer to configuration structure
 * @return STATUS_OK on success, error code otherwise
 */
Status_t BSP_GPIO_ConfigurePin(const GPIO_Config_t *pConfig);

/**
 * @brief Set GPIO pin state
 * @param[in] port GPIO port
 * @param[in] pin  GPIO pin
 * @param[in] state Pin state (HIGH/LOW)
 * @return STATUS_OK on success
 */
Status_t BSP_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_State_t state);

/**
 * @brief Read GPIO pin state
 * @param[in]  port   GPIO port
 * @param[in]  pin    GPIO pin
 * @param[out] pState Pointer to store pin state
 * @return STATUS_OK on success
 */
Status_t BSP_GPIO_ReadPin(GPIO_TypeDef *port, uint16_t pin, GPIO_State_t *pState);

/**
 * @brief Toggle GPIO pin
 * @param[in] port GPIO port
 * @param[in] pin  GPIO pin
 * @return STATUS_OK on success
 */
Status_t BSP_GPIO_TogglePin(GPIO_TypeDef *port, uint16_t pin);

/**
 * @brief Configure GPIO interrupt (EXTI)
 * @param[in] port     GPIO port
 * @param[in] pin      GPIO pin
 * @param[in] trigger  Trigger mode (rising/falling/both)
 * @param[in] priority Interrupt priority (0-15)
 * @return STATUS_OK on success
 */
Status_t BSP_GPIO_ConfigureInterrupt(GPIO_TypeDef *port, uint16_t pin,
                                     uint32_t trigger, uint8_t priority);

/**
 * @brief Enable GPIO interrupt
 * @param[in] pin GPIO pin
 * @return STATUS_OK on success
 */
Status_t BSP_GPIO_EnableInterrupt(uint16_t pin);

/**
 * @brief Disable GPIO interrupt
 * @param[in] pin GPIO pin
 * @return STATUS_OK on success
 */
Status_t BSP_GPIO_DisableInterrupt(uint16_t pin);

/**
 * @brief Lock GPIO pin configuration
 * @param[in] port GPIO port
 * @param[in] pin  GPIO pin
 * @return STATUS_OK on success
 * @note  Pin configuration cannot be changed until next reset
 */
Status_t BSP_GPIO_LockPin(GPIO_TypeDef *port, uint16_t pin);

/**
 * @brief De-initialize GPIO module
 * @return STATUS_OK on success
 */
Status_t BSP_GPIO_DeInit(void);

/*============================================================================*/
/* INLINE HELPER FUNCTIONS                                                    */
/*============================================================================*/

/**
 * @brief Fast pin set (inline for performance)
 * @param[in] port GPIO port
 * @param[in] pin  GPIO pin
 */
static inline void BSP_GPIO_SetPin(GPIO_TypeDef *port, uint16_t pin)
{
    port->BSRR = pin;
}

/**
 * @brief Fast pin reset (inline for performance)
 * @param[in] port GPIO port
 * @param[in] pin  GPIO pin
 */
static inline void BSP_GPIO_ResetPin(GPIO_TypeDef *port, uint16_t pin)
{
    port->BSRR = (uint32_t)pin << 16U;
}

/**
 * @brief Fast pin read (inline for performance)
 * @param[in] port GPIO port
 * @param[in] pin  GPIO pin
 * @return Pin state (0 or 1)
 */
static inline uint8_t BSP_GPIO_ReadPinFast(GPIO_TypeDef *port, uint16_t pin)
{
    return ((port->IDR & pin) != 0U) ? 1U : 0U;
}

#ifdef __cplusplus
}
#endif

#endif /* BSP_GPIO_H */
