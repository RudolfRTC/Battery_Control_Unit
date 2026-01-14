/**
 * @file    bcu_scheduler.c
 * @brief   Deterministic superloop scheduler implementation
 * @author  Battery Control Unit Development Team
 * @date    2026-01-10
 * @version 1.0.0
 *
 * @note    MISRA C:2012 compliant
 * @note    Deadline-based scheduling without drift
 *
 * @copyright Copyright (c) 2026
 */

/*============================================================================*/
/* INCLUDES                                                                   */
/*============================================================================*/
#include "bcu_scheduler.h"
#include "app_errors.h"
#include "timestamp.h"
#include <string.h>

/*============================================================================*/
/* GLOBAL VARIABLES                                                           */
/*============================================================================*/

/** @brief Global system tick counter (incremented by SysTick ISR) */
volatile uint32_t g_tick_ms = 0U;

/*============================================================================*/
/* PRIVATE VARIABLES                                                          */
/*============================================================================*/

/** @brief Registered jobs array */
static ScheduledJob_t jobs[SCHED_MAX_JOBS];

/** @brief Number of registered jobs */
static uint8_t job_count = 0U;

/** @brief Scheduler statistics */
static SchedulerStats_t sched_stats = {0};

/** @brief WCET statistics (one per priority level) */
static WCETStats_t wcet_stats[SCHED_PRIORITY_VERY_SLOW + 1U];

/** @brief Consecutive deadline misses counter */
static uint32_t consecutive_deadline_misses = 0U;

/** @brief Scheduler initialized flag */
static bool scheduler_initialized = false;

/*============================================================================*/
/* PRIVATE FUNCTION PROTOTYPES                                                */
/*============================================================================*/

static void scheduler_update_cpu_load(uint32_t loop_time_us);
static void scheduler_check_deadline_violations(void);

/*============================================================================*/
/* PUBLIC FUNCTIONS                                                           */
/*============================================================================*/

/**
 * @brief Initialize scheduler
 */
Status_t Scheduler_Init(void)
{
    Status_t status = STATUS_OK;

    if (!scheduler_initialized)
    {
        /* Clear jobs array */
        (void)memset(jobs, 0, sizeof(jobs));
        job_count = 0U;

        /* Clear statistics */
        (void)memset(&sched_stats, 0, sizeof(sched_stats));
        (void)memset(wcet_stats, 0, sizeof(wcet_stats));

        consecutive_deadline_misses = 0U;

        scheduler_initialized = true;
    }
    else
    {
        status = STATUS_ERROR_ALREADY_INIT;
    }

    return status;
}

/**
 * @brief Register a periodic job
 */
Status_t Scheduler_RegisterJob(const char *name, JobFunc_t func,
                                uint32_t period_ms, uint8_t priority)
{
    Status_t status = STATUS_ERROR_PARAM;

    if ((name != NULL) && (func != NULL) && (period_ms > 0U) && (job_count < SCHED_MAX_JOBS))
    {
        /* Atomic read of g_tick_ms to prevent race with SysTick ISR */
        __disable_irq();
        uint32_t tick_snapshot = g_tick_ms;
        __enable_irq();

        jobs[job_count].name = name;
        jobs[job_count].func = func;
        jobs[job_count].period_ms = period_ms;
        jobs[job_count].next_deadline_ms = tick_snapshot + period_ms;
        jobs[job_count].priority = priority;
        jobs[job_count].enabled = true;

        job_count++;
        status = STATUS_OK;
    }

    return status;
}

/**
 * @brief Enable/disable a job
 */
Status_t Scheduler_EnableJob(const char *name, bool enable)
{
    Status_t status = STATUS_ERROR_NOT_FOUND;

    if (name != NULL)
    {
        for (uint8_t i = 0U; i < job_count; i++)
        {
            if (strcmp(jobs[i].name, name) == 0)
            {
                jobs[i].enabled = enable;
                status = STATUS_OK;
                break;
            }
        }
    }
    else
    {
        status = STATUS_ERROR_PARAM;
    }

    return status;
}

/**
 * @brief Run scheduler superloop
 */
void Scheduler_Run(void)
{
    uint32_t loop_start_us;
    uint32_t loop_end_us;
    uint32_t loop_time_us;

    /* Main superloop - never returns */
    while (1)
    {
        /* Measure loop start time */
        loop_start_us = Timestamp_GetMicros();

        /* Get current tick (atomic read to prevent race with SysTick ISR) */
        __disable_irq();
        uint32_t now_ms = g_tick_ms;
        __enable_irq();

        /* Execute all due jobs (priority order) */
        for (uint8_t priority = 0U; priority <= SCHED_PRIORITY_VERY_SLOW; priority++)
        {
            for (uint8_t i = 0U; i < job_count; i++)
            {
                if ((jobs[i].priority == priority) &&
                    (jobs[i].enabled) &&
                    Scheduler_IsDeadlineDue(now_ms, jobs[i].next_deadline_ms))
                {
                    /* Execute job with WCET measurement */
                    uint32_t job_start_us = Scheduler_WCET_Start(jobs[i].name);

                    jobs[i].func(now_ms);

                    (void)Scheduler_WCET_Stop(jobs[i].name, job_start_us, 0U);

                    /* Update next deadline (no drift) */
                    jobs[i].next_deadline_ms += jobs[i].period_ms;

                    sched_stats.job_executions++;
                }
            }
        }

        /* Measure loop end time */
        loop_end_us = Timestamp_GetMicros();
        loop_time_us = loop_end_us - loop_start_us;

        /* Update statistics */
        sched_stats.loop_count++;
        sched_stats.last_loop_time_us = loop_time_us;

        if (loop_time_us > sched_stats.max_loop_time_us)
        {
            sched_stats.max_loop_time_us = loop_time_us;
        }

        /* Update CPU load */
        scheduler_update_cpu_load(loop_time_us);

        /* Check for deadline violations */
        scheduler_check_deadline_violations();

        /* Idle - can add low-power mode here if needed */
        /* __WFI(); */
    }
}

/**
 * @brief Get scheduler statistics
 */
Status_t Scheduler_GetStats(SchedulerStats_t *pStats)
{
    Status_t status = STATUS_ERROR_PARAM;

    if (pStats != NULL)
    {
        *pStats = sched_stats;
        status = STATUS_OK;
    }

    return status;
}

/**
 * @brief Get WCET statistics
 */
Status_t Scheduler_GetWCET(const char *task_name, WCETStats_t *pStats)
{
    Status_t status = STATUS_ERROR_PARAM;

    if ((task_name != NULL) && (pStats != NULL))
    {
        status = STATUS_ERROR_NOT_FOUND;

        for (uint8_t i = 0U; i <= SCHED_PRIORITY_VERY_SLOW; i++)
        {
            if ((wcet_stats[i].task_name != NULL) &&
                (strcmp(wcet_stats[i].task_name, task_name) == 0))
            {
                *pStats = wcet_stats[i];
                status = STATUS_OK;
                break;
            }
        }
    }

    return status;
}

/**
 * @brief Reset WCET statistics
 */
Status_t Scheduler_ResetWCET(void)
{
    (void)memset(wcet_stats, 0, sizeof(wcet_stats));
    return STATUS_OK;
}

/**
 * @brief Start WCET measurement
 */
uint32_t Scheduler_WCET_Start(const char *task_name)
{
    (void)task_name;  /* Store for matching in Stop */
    return Timestamp_GetMicros();
}

/**
 * @brief Stop WCET measurement
 */
Status_t Scheduler_WCET_Stop(const char *task_name, uint32_t start_us, uint32_t budget_us)
{
    Status_t status = STATUS_OK;
    uint32_t end_us = Timestamp_GetMicros();
    uint32_t elapsed_us = end_us - start_us;

    /* Find or create WCET stats entry */
    for (uint8_t i = 0U; i <= SCHED_PRIORITY_VERY_SLOW; i++)
    {
        if ((wcet_stats[i].task_name == NULL) ||
            (strcmp(wcet_stats[i].task_name, task_name) == 0))
        {
            wcet_stats[i].task_name = task_name;
            wcet_stats[i].last_time_us = elapsed_us;
            wcet_stats[i].count++;

            /* Update max */
            if (elapsed_us > wcet_stats[i].max_time_us)
            {
                wcet_stats[i].max_time_us = elapsed_us;
            }

            /* Update average (running average) */
            /* MISRA 21.4: Division by zero check (defensive programming) */
            if (wcet_stats[i].count > 0U)
            {
                wcet_stats[i].avg_time_us =
                    (wcet_stats[i].avg_time_us * (wcet_stats[i].count - 1U) + elapsed_us) /
                    wcet_stats[i].count;
            }

            /* Check budget overrun */
            if ((budget_us > 0U) && (elapsed_us > budget_us))
            {
                wcet_stats[i].overruns++;
                status = STATUS_ERROR_TIMEOUT;

                /* Log error */
                ErrorHandler_LogError(ERROR_SAFETY_TIMING_VIOLATION,
                                     i, elapsed_us, budget_us);
            }

            break;
        }
    }

    return status;
}

/*============================================================================*/
/* PRIVATE FUNCTIONS                                                          */
/*============================================================================*/

/**
 * @brief Update CPU load percentage
 */
static void scheduler_update_cpu_load(uint32_t loop_time_us)
{
    /* Calculate CPU load as percentage of 1ms period */
    /* CPU load = (loop_time_us / 1000) * 100 */
    uint32_t load = (loop_time_us * 100U) / 1000U;

    if (load > 100U)
    {
        load = 100U;  /* Cap at 100% */
    }

    /* Exponential moving average */
    sched_stats.cpu_load_percent =
        (sched_stats.cpu_load_percent * 9U + load) / 10U;
}

/**
 * @brief Check for deadline violations
 */
static void scheduler_check_deadline_violations(void)
{
    uint32_t now_ms = g_tick_ms;
    bool violation = false;

    /* Check if any critical job missed its deadline */
    for (uint8_t i = 0U; i < job_count; i++)
    {
        if ((jobs[i].priority == SCHED_PRIORITY_CRITICAL) && (jobs[i].enabled))
        {
            /* Check if we're late by more than margin */
            int32_t lateness = (int32_t)(now_ms - jobs[i].next_deadline_ms);
            if (lateness > (int32_t)SCHED_DEADLINE_MARGIN_MS)
            {
                violation = true;
                sched_stats.deadline_misses++;
                break;
            }
        }
    }

    if (violation)
    {
        consecutive_deadline_misses++;

        if (consecutive_deadline_misses >= SCHED_MAX_DEADLINE_MISSES)
        {
            /* Critical: too many deadline misses - trigger safe state */
            ErrorHandler_LogError(ERROR_SAFETY_CRITICAL_FAULT,
                                 consecutive_deadline_misses, 0U, 0U);
            /* App_EnterSafeState() would be called by error handler */
        }
    }
    else
    {
        consecutive_deadline_misses = 0U;
    }
}
