/**
 * @file    bcu_timing_config.h
 * @brief   BCU timing configuration for deterministic scheduling
 * @author  Battery Control Unit Development Team
 * @date    2026-01-10
 * @version 1.0.0
 *
 * @note    MISRA C:2012 compliant
 * @note    Central timing configuration for scheduler and adaptive scanning
 *
 * @copyright Copyright (c) 2026
 */

#ifndef BCU_TIMING_CONFIG_H
#define BCU_TIMING_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

/*============================================================================*/
/* SCHEDULER TIMING CONFIGURATION                                             */
/*============================================================================*/

/** @brief System tick period (milliseconds) */
#define SYSTICK_PERIOD_MS               (1U)

/** @brief Scheduler periods for different task priorities */
#define SCHED_PERIOD_CRITICAL_MS        (1U)     /**< Critical tasks (1ms) */
#define SCHED_PERIOD_FAST_MS            (2U)     /**< Fast tasks (2ms) */
#define SCHED_PERIOD_MEDIUM_MS          (10U)    /**< Medium tasks (10ms) */
#define SCHED_PERIOD_SLOW_MS            (50U)    /**< Slow tasks (50ms) */
#define SCHED_PERIOD_VERY_SLOW_MS       (100U)   /**< Very slow tasks (100ms) */
#define SCHED_PERIOD_LOGGING_MS         (100U)   /**< Data logging (100ms) */

/** @brief Deadline margin for detecting overruns (milliseconds) */
#define SCHED_DEADLINE_MARGIN_MS        (1U)

/** @brief Maximum consecutive deadline misses before safety action */
#define SCHED_MAX_DEADLINE_MISSES       (5U)

/*============================================================================*/
/* LTC CELL SCANNING TIMING CONFIGURATION                                     */
/*============================================================================*/

/** @brief LTC scan period - NORMAL mode (low activity) */
#define T_CELL_NORMAL_MS                (500U)

/** @brief LTC scan period - ACTIVE mode (high current/charging) */
#define T_CELL_ACTIVE_MS                (200U)

/** @brief LTC scan period - TRANSIENT mode (precharge/switching) */
#define T_CELL_TRANSIENT_MS             (100U)

/** @brief Duration to hold TRANSIENT mode after event (milliseconds) */
#define TRANSIENT_HOLD_MS               (3000U)

/** @brief Current threshold for ACTIVE mode (amperes) */
#define CURRENT_ACTIVE_THRESHOLD_A      (10.0f)

/** @brief Voltage threshold for detecting precharge (volts) */
#define PRECHARGE_VOLTAGE_THRESHOLD_V   (50.0f)

/*============================================================================*/
/* LTC STATE MACHINE TIMEOUTS                                                 */
/*============================================================================*/

/** @brief Timeout for ADC conversion (milliseconds) */
#define LTC_TIMEOUT_CONVERSION_MS       (50U)

/** @brief Timeout for single register read (milliseconds) */
#define LTC_TIMEOUT_READ_MS             (10U)

/** @brief Timeout for complete scan cycle (milliseconds) */
#define LTC_TIMEOUT_SCAN_CYCLE_MS       (200U)

/** @brief Maximum consecutive errors before entering error state */
#define LTC_MAX_CONSECUTIVE_ERRORS      (3U)

/*============================================================================*/
/* SENSOR UPDATE TIMING                                                       */
/*============================================================================*/

/** @brief LEM sensor update period (milliseconds) - target 1kHz */
#define LEM_UPDATE_PERIOD_MS            (1U)

/** @brief Digital input debounce period (milliseconds) */
#define DI_UPDATE_PERIOD_MS             (10U)

/** @brief Power monitor update period (milliseconds) */
#define PM_UPDATE_PERIOD_MS             (10U)

/** @brief Temperature sensor update period (milliseconds) */
#define TEMP_UPDATE_PERIOD_MS           (10U)

/*============================================================================*/
/* CAN COMMUNICATION TIMING                                                   */
/*============================================================================*/

/** @brief CAN RX processing - run every loop (non-blocking) */
#define CAN_RX_PERIOD_MS                (1U)

/** @brief CAN TX periodic messages - status (milliseconds) */
#define CAN_TX_STATUS_PERIOD_MS         (10U)

/** @brief CAN TX periodic messages - cell voltages (milliseconds) */
#define CAN_TX_CELLS_PERIOD_MS          (50U)

/** @brief CAN TX periodic messages - temperatures (milliseconds) */
#define CAN_TX_TEMPS_PERIOD_MS          (100U)

/*============================================================================*/
/* WATCHDOG TIMING                                                            */
/*============================================================================*/

/** @brief Watchdog kick period (milliseconds) - must be < watchdog timeout */
#define WATCHDOG_KICK_PERIOD_MS         (1U)

/** @brief Independent watchdog timeout (milliseconds) */
#define IWDG_TIMEOUT_MS                 (500U)

/** @brief Window watchdog timeout (milliseconds) */
#define WWDG_TIMEOUT_MS                 (100U)

/*============================================================================*/
/* WCET (Worst-Case Execution Time) BUDGETS                                   */
/*============================================================================*/

/** @brief Maximum allowed time for 1ms critical tasks (microseconds) */
#define WCET_CRITICAL_MAX_US            (800U)

/** @brief Maximum allowed time for LTC state machine step (microseconds) */
#define WCET_LTC_STEP_MAX_US            (500U)

/** @brief Maximum allowed time for complete loop iteration (microseconds) */
#define WCET_LOOP_MAX_US                (900U)

/*============================================================================*/
/* FRAM LOGGING CONFIGURATION                                                 */
/*============================================================================*/

/** @brief FRAM write period - low priority (milliseconds) */
#define FRAM_WRITE_PERIOD_MS            (100U)

/** @brief Maximum FRAM write time budget (milliseconds) */
#define FRAM_WRITE_TIMEOUT_MS           (50U)

#ifdef __cplusplus
}
#endif

#endif /* BCU_TIMING_CONFIG_H */
