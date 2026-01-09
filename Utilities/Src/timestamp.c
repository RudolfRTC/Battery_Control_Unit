/**
 * @file    timestamp.c
 * @brief   System timestamp and timing utilities implementation
 * @author  Battery Control Unit Development Team
 * @date    2026-01-09
 * @version 1.0.0
 *
 * @note    MISRA C:2012 compliant
 * @note    Uses SysTick timer for 1ms resolution
 *
 * @copyright Copyright (c) 2026
 */

/*============================================================================*/
/* INCLUDES                                                                   */
/*============================================================================*/
#include "timestamp.h"
#include "app_config.h"
#include "stm32f4xx_hal.h"

/*============================================================================*/
/* PRIVATE CONSTANTS                                                          */
/*============================================================================*/

/** @brief CPU frequency in Hz */
#define CPU_FREQ_HZ         (MCU_CORE_CLOCK_HZ)

/** @brief Microseconds per millisecond */
#define US_PER_MS           (1000U)

/** @brief Nanoseconds per cycle at 100 MHz */
#define NS_PER_CYCLE        (10U)

/*============================================================================*/
/* PRIVATE VARIABLES                                                          */
/*============================================================================*/

/** @brief System tick counter (milliseconds) */
static volatile uint32_t system_tick_ms = 0U;

/** @brief Initialization flag */
static bool timestamp_initialized = false;

/*============================================================================*/
/* PRIVATE FUNCTION PROTOTYPES                                                */
/*============================================================================*/

static void timestamp_enable_dwt(void);

/*============================================================================*/
/* PUBLIC FUNCTIONS                                                           */
/*============================================================================*/

/**
 * @brief Initialize timestamp module
 */
Status_t Timestamp_Init(void)
{
    Status_t status = STATUS_OK;

    if (timestamp_initialized)
    {
        status = STATUS_ERROR_ALREADY_INIT;
    }
    else
    {
        /* SysTick is already configured by HAL_Init() for 1ms interrupts */
        /* We just reset our counter */
        system_tick_ms = 0U;

        /* Enable DWT (Data Watchpoint and Trace) for cycle counting */
        timestamp_enable_dwt();

        timestamp_initialized = true;
    }

    return status;
}

/**
 * @brief Get current timestamp in milliseconds
 */
uint32_t Timestamp_GetMillis(void)
{
    uint32_t timestamp_ms;

    /* Read volatile variable (atomic on Cortex-M4) */
    timestamp_ms = system_tick_ms;

    return timestamp_ms;
}

/**
 * @brief Get current timestamp in microseconds
 */
uint64_t Timestamp_GetMicros(void)
{
    uint64_t timestamp_us;
    uint32_t ms;
    uint32_t cycles;
    uint32_t us_fraction;

    /* Disable interrupts for atomic read */
    __disable_irq();

    /* Read millisecond counter */
    ms = system_tick_ms;

    /* Read DWT cycle counter */
    cycles = DWT->CYCCNT;

    /* Re-enable interrupts */
    __enable_irq();

    /* Calculate microseconds from cycles (assuming SysTick reloads every 1ms) */
    us_fraction = (cycles * US_PER_MS) / CPU_FREQ_HZ;

    /* Combine milliseconds and microsecond fraction */
    timestamp_us = ((uint64_t)ms * US_PER_MS) + (uint64_t)us_fraction;

    return timestamp_us;
}

/**
 * @brief Get elapsed time since reference timestamp
 */
uint32_t Timestamp_GetElapsed(uint32_t reference_ms)
{
    uint32_t current_ms;
    uint32_t elapsed_ms;

    current_ms = Timestamp_GetMillis();

    /* Handle 32-bit wraparound correctly */
    if (current_ms >= reference_ms)
    {
        elapsed_ms = current_ms - reference_ms;
    }
    else
    {
        /* Wraparound occurred */
        elapsed_ms = (0xFFFFFFFFUL - reference_ms) + current_ms + 1U;
    }

    return elapsed_ms;
}

/**
 * @brief Check if timeout has occurred
 */
bool Timestamp_IsTimeout(uint32_t start_ms, uint32_t timeout_ms)
{
    bool timeout_occurred;
    uint32_t elapsed_ms;

    elapsed_ms = Timestamp_GetElapsed(start_ms);
    timeout_occurred = (elapsed_ms >= timeout_ms);

    return timeout_occurred;
}

/**
 * @brief Blocking delay in milliseconds
 */
void Timestamp_DelayMs(uint32_t delay_ms)
{
    uint32_t start_ms;

    start_ms = Timestamp_GetMillis();

    while (!Timestamp_IsTimeout(start_ms, delay_ms))
    {
        /* Wait for interrupt (power efficient) */
        __WFI();
    }
}

/**
 * @brief Blocking delay in microseconds
 */
void Timestamp_DelayUs(uint32_t delay_us)
{
    uint32_t start_cycles;
    uint32_t cycles_to_wait;
    uint32_t current_cycles;
    uint32_t elapsed_cycles;

    /* Calculate number of CPU cycles for delay */
    cycles_to_wait = (delay_us * CPU_FREQ_HZ) / 1000000U;

    /* Read start cycle count */
    start_cycles = DWT->CYCCNT;

    /* Busy-wait loop */
    do
    {
        current_cycles = DWT->CYCCNT;

        /* Handle cycle counter wraparound */
        if (current_cycles >= start_cycles)
        {
            elapsed_cycles = current_cycles - start_cycles;
        }
        else
        {
            elapsed_cycles = (0xFFFFFFFFUL - start_cycles) + current_cycles + 1U;
        }

    } while (elapsed_cycles < cycles_to_wait);
}

/**
 * @brief SysTick interrupt handler
 */
void Timestamp_SysTick_Handler(void)
{
    /* Increment millisecond counter */
    system_tick_ms++;
}

/**
 * @brief Convert timestamp to time structure
 */
Status_t Timestamp_ToTime(uint32_t timestamp_ms, uint32_t *pHours,
                          uint32_t *pMinutes, uint32_t *pSeconds,
                          uint32_t *pMillis)
{
    Status_t status = STATUS_ERROR_PARAM;

    /* Parameter validation */
    if ((pHours != NULL) && (pMinutes != NULL) &&
        (pSeconds != NULL) && (pMillis != NULL))
    {
        uint32_t total_seconds;

        /* Extract milliseconds */
        *pMillis = timestamp_ms % 1000U;

        /* Calculate total seconds */
        total_seconds = timestamp_ms / 1000U;

        /* Extract seconds */
        *pSeconds = total_seconds % 60U;

        /* Calculate total minutes */
        total_seconds = total_seconds / 60U;

        /* Extract minutes */
        *pMinutes = total_seconds % 60U;

        /* Calculate hours */
        *pHours = total_seconds / 60U;

        status = STATUS_OK;
    }

    return status;
}

/**
 * @brief Get CPU cycle count
 */
uint32_t Timestamp_GetCycles(void)
{
    uint32_t cycles;

    cycles = DWT->CYCCNT;

    return cycles;
}

/**
 * @brief Convert cycles to microseconds
 */
uint32_t Timestamp_CyclesToMicros(uint32_t cycles)
{
    uint32_t microseconds;

    /* microseconds = cycles / (CPU_FREQ_HZ / 1000000) */
    microseconds = (cycles * 1000000U) / CPU_FREQ_HZ;

    return microseconds;
}

/**
 * @brief Convert cycles to nanoseconds
 */
uint32_t Timestamp_CyclesToNanos(uint32_t cycles)
{
    uint32_t nanoseconds;

    /* nanoseconds = cycles * (1000000000 / CPU_FREQ_HZ) */
    /* At 100 MHz: 10 ns per cycle */
    nanoseconds = cycles * NS_PER_CYCLE;

    return nanoseconds;
}

/*============================================================================*/
/* PRIVATE FUNCTIONS                                                          */
/*============================================================================*/

/**
 * @brief Enable DWT (Data Watchpoint and Trace) for cycle counting
 */
static void timestamp_enable_dwt(void)
{
    /* Enable TRC (Trace) */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

    /* Reset cycle counter */
    DWT->CYCCNT = 0U;

    /* Enable cycle counter */
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

/*============================================================================*/
/* SYSTICK HOOK (called from stm32f4xx_it.c)                                  */
/*============================================================================*/

/**
 * @brief SysTick interrupt handler hook
 * @note  This should be called from SysTick_Handler() in stm32f4xx_it.c:
 *        void SysTick_Handler(void)
 *        {
 *            HAL_IncTick();
 *            Timestamp_SysTick_Handler();
 *        }
 */

/*============================================================================*/
/* END OF FILE                                                                */
/*============================================================================*/
