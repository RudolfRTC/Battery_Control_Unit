/**
 * @file self_test.h
 * @brief Self-Test Routines for Battery Control Unit
 * @details Power-On Self-Test (POST) and periodic runtime self-tests
 *          for ISO 26262 ASIL-B compliance
 *
 * @author Claude Code AI
 * @date 2026-01-09
 * @version 1.0
 *
 * @copyright Copyright (c) 2026
 *
 * ISO 26262 ASIL-B Requirements:
 * - Power-on self-test (POST) before operational mode
 * - Periodic runtime tests for hardware monitoring
 * - Fault detection and logging
 * - Safe state entry on critical failures
 */

#ifndef SELF_TEST_H
#define SELF_TEST_H

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
 * INCLUDES
 ******************************************************************************/
#include <stdint.h>
#include <stdbool.h>
#include "app_types.h"

/*******************************************************************************
 * DEFINITIONS
 ******************************************************************************/

/**
 * @brief Self-test module version
 */
#define SELF_TEST_VERSION_MAJOR    1U
#define SELF_TEST_VERSION_MINOR    0U
#define SELF_TEST_VERSION_PATCH    0U

/**
 * @brief RAM test parameters
 */
#define SELF_TEST_RAM_TEST_SIZE    1024U   /**< Test 1KB of RAM */
#define SELF_TEST_RAM_PATTERN_1    0x55555555U
#define SELF_TEST_RAM_PATTERN_2    0xAAAAAAAAU
#define SELF_TEST_RAM_PATTERN_3    0x00000000U
#define SELF_TEST_RAM_PATTERN_4    0xFFFFFFFFU

/**
 * @brief Flash test parameters
 */
#define SELF_TEST_FLASH_START      0x08000000U  /**< Flash start address */
#define SELF_TEST_FLASH_SIZE       (256U * 1024U) /**< Test first 256KB */

/**
 * @brief Self-test timeout values (milliseconds)
 */
#define SELF_TEST_TIMEOUT_SHORT    100U
#define SELF_TEST_TIMEOUT_MEDIUM   500U
#define SELF_TEST_TIMEOUT_LONG     2000U

/*******************************************************************************
 * TYPES
 ******************************************************************************/

/**
 * @brief Self-test result codes
 */
typedef enum {
    SELF_TEST_RESULT_PASS = 0,           /**< Test passed */
    SELF_TEST_RESULT_FAIL_RAM,           /**< RAM test failed */
    SELF_TEST_RESULT_FAIL_FLASH,         /**< Flash test failed */
    SELF_TEST_RESULT_FAIL_GPIO,          /**< GPIO test failed */
    SELF_TEST_RESULT_FAIL_ADC,           /**< ADC test failed */
    SELF_TEST_RESULT_FAIL_I2C,           /**< I2C test failed */
    SELF_TEST_RESULT_FAIL_CAN,           /**< CAN test failed */
    SELF_TEST_RESULT_FAIL_FRAM,          /**< FRAM test failed */
    SELF_TEST_RESULT_FAIL_LEM,           /**< LEM sensor test failed */
    SELF_TEST_RESULT_FAIL_TEMP,          /**< Temperature sensor test failed */
    SELF_TEST_RESULT_FAIL_BTT6200,       /**< BTT6200 test failed */
    SELF_TEST_RESULT_FAIL_WATCHDOG,      /**< Watchdog test failed */
    SELF_TEST_RESULT_FAIL_TIMEOUT,       /**< Test timeout */
    SELF_TEST_RESULT_NOT_RUN             /**< Test not executed */
} SelfTestResult_t;

/**
 * @brief POST (Power-On Self-Test) test categories
 */
typedef enum {
    POST_TEST_RAM = 0,        /**< RAM integrity test */
    POST_TEST_FLASH,          /**< Flash CRC verification */
    POST_TEST_GPIO,           /**< GPIO configuration check */
    POST_TEST_ADC,            /**< ADC functionality test */
    POST_TEST_I2C,            /**< I2C communication test */
    POST_TEST_CAN,            /**< CAN communication test */
    POST_TEST_FRAM,           /**< FRAM read/write test */
    POST_TEST_LEM,            /**< LEM sensor connectivity */
    POST_TEST_TEMP,           /**< Temperature sensor test */
    POST_TEST_BTT6200,        /**< Output driver test */
    POST_TEST_WATCHDOG,       /**< Watchdog functionality */
    POST_TEST_COUNT           /**< Total number of POST tests */
} POSTTestType_t;

/**
 * @brief Periodic self-test types
 */
typedef enum {
    PERIODIC_TEST_SENSOR_RANGE = 0,  /**< Sensor range validation */
    PERIODIC_TEST_ACTUATOR_LOOP,     /**< Actuator loopback test */
    PERIODIC_TEST_COMM_HEALTH,       /**< Communication health check */
    PERIODIC_TEST_POWER_STABILITY,   /**< Power supply stability */
    PERIODIC_TEST_TIMING,            /**< Timing accuracy check */
    PERIODIC_TEST_COUNT              /**< Total periodic tests */
} PeriodicTestType_t;

/**
 * @brief POST results summary structure
 */
typedef struct {
    SelfTestResult_t results[POST_TEST_COUNT];  /**< Individual test results */
    uint32_t         failedTests;               /**< Bitmask of failed tests */
    uint32_t         executionTime;             /**< Total POST time (ms) */
    uint32_t         timestamp;                 /**< POST execution timestamp */
    bool             allTestsPassed;            /**< Overall pass/fail status */
} POSTResults_t;

/**
 * @brief Periodic test results structure
 */
typedef struct {
    SelfTestResult_t results[PERIODIC_TEST_COUNT]; /**< Test results */
    uint32_t         lastRunTime[PERIODIC_TEST_COUNT]; /**< Last execution time */
    uint32_t         failureCount[PERIODIC_TEST_COUNT]; /**< Failure counters */
    uint32_t         successCount[PERIODIC_TEST_COUNT]; /**< Success counters */
} PeriodicTestResults_t;

/**
 * @brief Self-test statistics
 */
typedef struct {
    uint32_t totalPOSTRuns;        /**< Total POST executions */
    uint32_t totalPOSTFailures;    /**< Total POST failures */
    uint32_t totalPeriodicTests;   /**< Total periodic tests run */
    uint32_t totalPeriodicFailures;/**< Total periodic test failures */
    uint32_t lastPOSTTimestamp;    /**< Last POST timestamp */
    uint32_t lastPeriodicTimestamp;/**< Last periodic test timestamp */
} SelfTestStats_t;

/*******************************************************************************
 * PUBLIC FUNCTION PROTOTYPES
 ******************************************************************************/

/**
 * @brief Initialize self-test module
 * @details Prepares internal structures for self-testing
 * @return STATUS_OK on success, error code otherwise
 */
Status_t SelfTest_Init(void);

/**
 * @brief Execute Power-On Self-Test (POST)
 * @details Runs comprehensive hardware and software tests at startup.
 *          This function should be called before entering operational mode.
 * @param[out] pResults Pointer to structure to store test results
 * @return STATUS_OK if all tests pass, STATUS_ERROR otherwise
 * @note This function may take several seconds to complete
 */
Status_t SelfTest_RunPOST(POSTResults_t *pResults);

/**
 * @brief Execute individual POST test
 * @param[in] testType Type of test to run
 * @return Test result code
 */
SelfTestResult_t SelfTest_RunIndividualPOST(POSTTestType_t testType);

/**
 * @brief RAM integrity test using march algorithm
 * @details Tests RAM for stuck-at faults, address decoding errors,
 *          and coupling faults using march C- algorithm
 * @return SELF_TEST_RESULT_PASS on success
 */
SelfTestResult_t SelfTest_RAMTest(void);

/**
 * @brief Flash CRC verification
 * @details Calculates CRC32 of application code and compares with
 *          stored CRC value
 * @return SELF_TEST_RESULT_PASS if CRC matches
 */
SelfTestResult_t SelfTest_FlashCRCTest(void);

/**
 * @brief GPIO configuration verification
 * @details Verifies that all critical GPIOs are configured correctly
 * @return SELF_TEST_RESULT_PASS if configuration is correct
 */
SelfTestResult_t SelfTest_GPIOTest(void);

/**
 * @brief ADC functionality test
 * @details Tests ADC by reading known references (VREFINT, temperature sensor)
 * @return SELF_TEST_RESULT_PASS if ADC is functioning
 */
SelfTestResult_t SelfTest_ADCTest(void);

/**
 * @brief I2C bus functionality test
 * @details Tests I2C by attempting communication with FRAM
 * @return SELF_TEST_RESULT_PASS if I2C is working
 */
SelfTestResult_t SelfTest_I2CTest(void);

/**
 * @brief CAN bus functionality test
 * @details Tests CAN using loopback mode
 * @return SELF_TEST_RESULT_PASS if CAN is working
 */
SelfTestResult_t SelfTest_CANTest(void);

/**
 * @brief FRAM read/write test
 * @details Tests FRAM by writing and reading test patterns
 * @return SELF_TEST_RESULT_PASS if FRAM is functional
 */
SelfTestResult_t SelfTest_FRAMTest(void);

/**
 * @brief LEM sensor connectivity test
 * @details Verifies LEM sensors are connected and responding
 * @return SELF_TEST_RESULT_PASS if sensors are connected
 */
SelfTestResult_t SelfTest_LEMSensorTest(void);

/**
 * @brief Temperature sensor test
 * @details Tests I2C temperature sensor communication
 * @return SELF_TEST_RESULT_PASS if sensor responds
 */
SelfTestResult_t SelfTest_TempSensorTest(void);

/**
 * @brief BTT6200 output driver test
 * @details Tests output drivers for basic functionality
 * @return SELF_TEST_RESULT_PASS if drivers respond
 */
SelfTestResult_t SelfTest_BTT6200Test(void);

/**
 * @brief Watchdog functionality test
 * @details Verifies watchdog is configured and running
 * @return SELF_TEST_RESULT_PASS if watchdog is active
 */
SelfTestResult_t SelfTest_WatchdogTest(void);

/**
 * @brief Execute periodic runtime self-tests
 * @details Runs periodic tests that can be executed during normal operation
 * @param[in] testType Type of periodic test to run
 * @return Test result code
 */
SelfTestResult_t SelfTest_RunPeriodicTest(PeriodicTestType_t testType);

/**
 * @brief Sensor range validation test
 * @details Checks if sensor readings are within expected ranges
 * @return SELF_TEST_RESULT_PASS if all sensors are in range
 */
SelfTestResult_t SelfTest_SensorRangeTest(void);

/**
 * @brief Actuator loopback test
 * @details Tests output functionality by reading back status
 * @return SELF_TEST_RESULT_PASS if actuators respond correctly
 */
SelfTestResult_t SelfTest_ActuatorLoopbackTest(void);

/**
 * @brief Communication health check
 * @details Verifies CAN communication is active and healthy
 * @return SELF_TEST_RESULT_PASS if communication is healthy
 */
SelfTestResult_t SelfTest_CommHealthTest(void);

/**
 * @brief Power supply stability test
 * @details Checks if power rails are stable
 * @return SELF_TEST_RESULT_PASS if power is stable
 */
SelfTestResult_t SelfTest_PowerStabilityTest(void);

/**
 * @brief Timing accuracy check
 * @details Verifies system timing is accurate
 * @return SELF_TEST_RESULT_PASS if timing is within tolerance
 */
SelfTestResult_t SelfTest_TimingTest(void);

/**
 * @brief Get POST results
 * @param[out] pResults Pointer to store results
 * @return STATUS_OK on success
 */
Status_t SelfTest_GetPOSTResults(POSTResults_t *pResults);

/**
 * @brief Get periodic test results
 * @param[out] pResults Pointer to store results
 * @return STATUS_OK on success
 */
Status_t SelfTest_GetPeriodicResults(PeriodicTestResults_t *pResults);

/**
 * @brief Get self-test statistics
 * @param[out] pStats Pointer to store statistics
 * @return STATUS_OK on success
 */
Status_t SelfTest_GetStatistics(SelfTestStats_t *pStats);

/**
 * @brief Clear self-test statistics
 * @return STATUS_OK on success
 */
Status_t SelfTest_ClearStatistics(void);

/**
 * @brief Log POST results to FRAM
 * @param[in] pResults POST results to log
 * @return STATUS_OK on success
 */
Status_t SelfTest_LogPOSTResults(const POSTResults_t *pResults);

/**
 * @brief Convert test result to string
 * @param[in] result Test result code
 * @return Pointer to result string
 */
const char* SelfTest_ResultToString(SelfTestResult_t result);

/**
 * @brief Convert POST test type to string
 * @param[in] testType Test type
 * @return Pointer to test name string
 */
const char* SelfTest_POSTTypeToString(POSTTestType_t testType);

/**
 * @brief Check if POST has been run successfully
 * @return true if POST passed, false otherwise
 */
bool SelfTest_IsPOSTComplete(void);

/*******************************************************************************
 * INLINE FUNCTIONS
 ******************************************************************************/

/**
 * @brief Get self-test module version
 * @param[out] major Major version number
 * @param[out] minor Minor version number
 * @param[out] patch Patch version number
 */
static inline void SelfTest_GetVersion(uint8_t *major, uint8_t *minor, uint8_t *patch)
{
    if ((major != NULL) && (minor != NULL) && (patch != NULL))
    {
        *major = SELF_TEST_VERSION_MAJOR;
        *minor = SELF_TEST_VERSION_MINOR;
        *patch = SELF_TEST_VERSION_PATCH;
    }
}

#ifdef __cplusplus
}
#endif

#endif /* SELF_TEST_H */
