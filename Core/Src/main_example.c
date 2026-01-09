/**
 * @file    main_example.c
 * @brief   Main entry point for BCU firmware (EXAMPLE)
 * @author  Battery Control Unit Development Team
 * @date    2026-01-09
 * @version 1.0.0
 *
 * @note    This file should replace/integrate with existing main.c
 * @note    MISRA C:2012 compliant
 *
 * @copyright Copyright (c) 2026
 */

/*============================================================================*/
/* INCLUDES                                                                   */
/*============================================================================*/
#include "main.h"
#include "app_main.h"
#include "app_config.h"
#include "app_errors.h"
#include "timestamp.h"

/*============================================================================*/
/* EXTERNAL FUNCTIONS (system_stm32f4xx.c)                                    */
/*============================================================================*/
extern void SystemClock_Config(void);

/*============================================================================*/
/* MAIN FUNCTION                                                              */
/*============================================================================*/

/**
 * @brief Main entry point
 * @return Never returns
 */
int main(void)
{
    Status_t status;

    /* Initialize application */
    status = App_Init();

    if (status != STATUS_OK)
    {
        /* Initialization failed - enter safe state */
        ErrorHandler_SafeState();
    }

    /* Start main loop (never returns) */
    App_MainLoop();

    /* Should never reach here */
    return 0;
}

/*============================================================================*/
/* SYSTICK HANDLER                                                            */
/*============================================================================*/

/**
 * @brief SysTick interrupt handler
 * @note  Called every 1ms
 */
void SysTick_Handler(void)
{
    /* HAL tick increment */
    HAL_IncTick();

    /* Timestamp module tick */
    Timestamp_SysTick_Handler();
}

/*============================================================================*/
/* ERROR HANDLERS                                                             */
/*============================================================================*/

/**
 * @brief Hard fault handler
 */
void HardFault_Handler(void)
{
    /* Disable interrupts */
    __disable_irq();

    /* Log error */
    ErrorHandler_HardFault();

    /* Infinite loop */
    while (1)
    {
        /* Wait for watchdog reset */
    }
}

/**
 * @brief Generic error handler
 */
void Error_Handler(void)
{
    /* Disable interrupts */
    __disable_irq();

    /* Enter safe state */
    ErrorHandler_SafeState();

    /* Infinite loop */
    while (1)
    {
        /* Wait for watchdog reset */
    }
}

/*============================================================================*/
/* DMA INTERRUPT HANDLERS (for ADC)                                           */
/*============================================================================*/

/**
 * @brief DMA2 Stream0 interrupt handler (ADC1)
 */
void DMA2_Stream0_IRQHandler(void)
{
    extern DMA_HandleTypeDef hdma_adc1;
    HAL_DMA_IRQHandler(&hdma_adc1);
}

/*============================================================================*/
/* END OF FILE                                                                */
/*============================================================================*/
