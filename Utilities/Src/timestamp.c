/**
 * @file    timestamp.c
 * @brief   Timestamp and timing utilities implementation
 * @author  Battery Control Unit Development Team
 * @date    2026-01-09
 * @version 1.0.0
 *
 * @note    MISRA C:2012 compliant
 * @note    Uses SysTick for millisecond timing and DWT for microseconds
 *
 * @copyright Copyright (c) 2026
 */

/*============================================================================*/
/* INCLUDES                                                                   */
/*============================================================================*/
#include "timestamp.h"
#include "stm32f4xx_hal.h"

/*============================================================================*/
/* PRIVATE VARIABLES                                                          */
/*============================================================================*/

/** @brief Millisecond counter (updated by SysTick) */
static volatile uint32_t timestamp_millis = 0U;

/** @brief Overflow counter for microseconds */
static volatile uint32_t timestamp_micros_overflow = 0U;

/** @brief Module initialized flag */
static bool timestamp_initialized = false;

/*============================================================================*/
/* PUBLIC FUNCTIONS                                                           */
/*============================================================================*/

/**
 * @brief Initialize timestamp module
 */
Status_t Timestamp_ModuleInit(void)
{
    Status_t status = STATUS_OK;

    if (!timestamp_initialized)
    {
        /* Reset counters */
        timestamp_millis = 0U;
        timestamp_micros_overflow = 0U;

        /* Enable DWT cycle counter for microsecond timing */
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
        DWT->CYCCNT = 0U;
        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

        timestamp_initialized = true;
    }

    return status;
}

/**
 * @brief Get current millisecond count
 */
uint32_t Timestamp_GetMillis(void)
{
    return HAL_GetTick();
}

/**
 * @brief Get current microsecond count
 */
uint64_t Timestamp_GetMicros(void)
{
    uint32_t millis;
    uint32_t systick_val;
    uint32_t systick_load;

    /* Disable interrupts to get atomic read */
    __disable_irq();
    millis = HAL_GetTick();
    systick_val = SysTick->VAL;
    systick_load = SysTick->LOAD;
    __enable_irq();

    /* Calculate microseconds:
     * millis * 1000 + ((LOAD - VAL) * 1000) / (LOAD + 1)
     * Assuming 1ms SysTick period
     */
    uint32_t micros_in_tick = ((systick_load - systick_val) * 1000U) / (systick_load + 1U);

    return ((uint64_t)millis * 1000U) + micros_in_tick;
}

/**
 * @brief SysTick interrupt handler
 */
void Timestamp_SysTick_Handler(void)
{
    timestamp_millis++;
}

/**
 * @brief Check if timeout has elapsed
 */
bool Timestamp_IsElapsed(uint32_t start_ms, uint32_t timeout_ms)
{
    uint32_t current_ms = Timestamp_GetMillis();
    uint32_t elapsed;

    if (current_ms >= start_ms)
    {
        elapsed = current_ms - start_ms;
    }
    else
    {
        /* Handle overflow */
        elapsed = (0xFFFFFFFFU - start_ms) + current_ms + 1U;
    }

    return (elapsed >= timeout_ms);
}

/**
 * @brief Blocking delay in milliseconds
 */
void Timestamp_DelayMs(uint32_t ms)
{
    HAL_Delay(ms);
}

/**
 * @brief Blocking delay in microseconds using DWT cycle counter
 * @param us Delay in microseconds (max ~40 seconds at 100MHz)
 * @note Uses DWT CYCCNT register for accurate timing
 */
void Timestamp_DelayUs(uint32_t us)
{
    if (us == 0U)
    {
        return;
    }

    /* Get system core clock frequency */
    uint32_t cycles_per_us = SystemCoreClock / 1000000U;
    uint32_t cycles_to_wait = us * cycles_per_us;

    /* Capture start cycle count */
    uint32_t start_cycles = DWT->CYCCNT;

    /* Wait until the required cycles have elapsed */
    while ((DWT->CYCCNT - start_cycles) < cycles_to_wait)
    {
        /* Spin wait */
    }
}

/*============================================================================*/
/* END OF FILE                                                                */
/*============================================================================*/
