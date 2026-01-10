/**
 * @file    bcu_scheduler.h
 * @brief   Deterministic superloop scheduler for BCU
 * @author  Battery Control Unit Development Team
 * @date    2026-01-10
 * @version 1.0.0
 *
 * @note    MISRA C:2012 compliant
 * @note    Deadline-based scheduling without RTOS
 * @note    Handles tick overflow correctly
 *
 * @copyright Copyright (c) 2026
 */

#ifndef BCU_SCHEDULER_H
#define BCU_SCHEDULER_H

#ifdef __cplusplus
extern "C" {
#endif

/*============================================================================*/
/* INCLUDES                                                                   */
/*============================================================================*/
#include "app_types.h"
#include "bcu_timing_config.h"
#include <stdint.h>
#include <stdbool.h>

/*============================================================================*/
/* CONSTANTS                                                                  */
/*============================================================================*/

/** @brief Maximum number of scheduled jobs */
#define SCHED_MAX_JOBS                  (16U)

/** @brief Job priority levels */
#define SCHED_PRIORITY_CRITICAL         (0U)    /**< 1ms tasks */
#define SCHED_PRIORITY_FAST             (1U)    /**< 2ms tasks */
#define SCHED_PRIORITY_MEDIUM           (2U)    /**< 10ms tasks */
#define SCHED_PRIORITY_SLOW             (3U)    /**< 50ms tasks */
#define SCHED_PRIORITY_VERY_SLOW        (4U)    /**< 100ms tasks */

/*============================================================================*/
/* TYPE DEFINITIONS                                                           */
/*============================================================================*/

/**
 * @brief Job function pointer type
 * @param[in] now_ms Current tick time in milliseconds
 */
typedef void (*JobFunc_t)(uint32_t now_ms);

/**
 * @brief Scheduled job structure
 */
typedef struct {
    const char      *name;              /**< Job name for debugging */
    JobFunc_t       func;               /**< Function to execute */
    uint32_t        period_ms;          /**< Period in milliseconds */
    uint32_t        next_deadline_ms;   /**< Next execution deadline */
    uint8_t         priority;           /**< Job priority level */
    bool            enabled;            /**< Job enabled flag */
} ScheduledJob_t;

/**
 * @brief Scheduler statistics
 */
typedef struct {
    uint32_t        loop_count;         /**< Total loop iterations */
    uint32_t        job_executions;     /**< Total job executions */
    uint32_t        deadline_misses;    /**< Total deadline misses */
    uint32_t        max_loop_time_us;   /**< Maximum loop time (µs) */
    uint32_t        last_loop_time_us;  /**< Last loop time (µs) */
    uint32_t        cpu_load_percent;   /**< CPU load percentage (0-100) */
} SchedulerStats_t;

/**
 * @brief WCET (Worst-Case Execution Time) measurement
 */
typedef struct {
    const char      *task_name;         /**< Task name */
    uint32_t        last_time_us;       /**< Last execution time (µs) */
    uint32_t        max_time_us;        /**< Maximum execution time (µs) */
    uint32_t        avg_time_us;        /**< Average execution time (µs) */
    uint32_t        count;              /**< Execution count */
    uint32_t        overruns;           /**< Deadline overruns */
} WCETStats_t;

/*============================================================================*/
/* GLOBAL VARIABLES (declared in bcu_scheduler.c)                            */
/*============================================================================*/

/** @brief Global system tick counter (incremented by SysTick ISR) */
extern volatile uint32_t g_tick_ms;

/*============================================================================*/
/* FUNCTION PROTOTYPES                                                        */
/*============================================================================*/

/**
 * @brief Initialize scheduler
 * @return STATUS_OK on success
 */
Status_t Scheduler_Init(void);

/**
 * @brief Register a periodic job
 * @param[in] name       Job name (for debugging)
 * @param[in] func       Function pointer to execute
 * @param[in] period_ms  Execution period in milliseconds
 * @param[in] priority   Job priority level
 * @return STATUS_OK on success, error code otherwise
 */
Status_t Scheduler_RegisterJob(const char *name, JobFunc_t func,
                                uint32_t period_ms, uint8_t priority);

/**
 * @brief Enable/disable a job
 * @param[in] name    Job name
 * @param[in] enable  true to enable, false to disable
 * @return STATUS_OK on success
 */
Status_t Scheduler_EnableJob(const char *name, bool enable);

/**
 * @brief Run scheduler superloop (never returns)
 * @note This function contains the main scheduling loop
 */
void Scheduler_Run(void);

/**
 * @brief Get current system tick
 * @return Current tick value in milliseconds
 */
static inline uint32_t Scheduler_GetTick(void)
{
    return g_tick_ms;
}

/**
 * @brief Check if deadline has been reached (handles overflow)
 * @param[in] now_ms      Current time
 * @param[in] deadline_ms Deadline time
 * @return true if now >= deadline (accounting for overflow)
 */
static inline bool Scheduler_IsDeadlineDue(uint32_t now_ms, uint32_t deadline_ms)
{
    /* Handle uint32 overflow by checking signed difference */
    return ((int32_t)(now_ms - deadline_ms)) >= 0;
}

/**
 * @brief Calculate time difference (handles overflow)
 * @param[in] start_ms Start time
 * @param[in] end_ms   End time
 * @return Time difference in milliseconds
 */
static inline uint32_t Scheduler_TimeDiff(uint32_t start_ms, uint32_t end_ms)
{
    return (end_ms - start_ms);
}

/**
 * @brief Get scheduler statistics
 * @param[out] pStats Pointer to statistics structure
 * @return STATUS_OK on success
 */
Status_t Scheduler_GetStats(SchedulerStats_t *pStats);

/**
 * @brief Get WCET statistics for a task
 * @param[in]  task_name Task name
 * @param[out] pStats    Pointer to WCET statistics structure
 * @return STATUS_OK on success
 */
Status_t Scheduler_GetWCET(const char *task_name, WCETStats_t *pStats);

/**
 * @brief Reset WCET statistics
 * @return STATUS_OK on success
 */
Status_t Scheduler_ResetWCET(void);

/**
 * @brief Start WCET measurement for a task
 * @param[in] task_name Task name
 * @return Start timestamp in microseconds
 */
uint32_t Scheduler_WCET_Start(const char *task_name);

/**
 * @brief Stop WCET measurement and update statistics
 * @param[in] task_name   Task name
 * @param[in] start_us    Start timestamp from Scheduler_WCET_Start()
 * @param[in] budget_us   Maximum allowed time (0 = no check)
 * @return STATUS_OK if within budget, STATUS_ERROR_TIMEOUT if exceeded
 */
Status_t Scheduler_WCET_Stop(const char *task_name, uint32_t start_us, uint32_t budget_us);

#ifdef __cplusplus
}
#endif

#endif /* BCU_SCHEDULER_H */
