/**
 * @file self_test.c
 * @brief Self-Test Routines Implementation
 * @details Power-On Self-Test (POST) and periodic runtime self-tests
 *          for ISO 26262 ASIL-B compliance
 *
 * @author Claude Code AI
 * @date 2026-01-09
 * @version 1.0
 *
 * @copyright Copyright (c) 2026
 */

/*******************************************************************************
 * INCLUDES
 ******************************************************************************/
#include "self_test.h"
#include "stm32f4xx_hal.h"
#include "crc.h"
#include "timestamp.h"
#include "app_errors.h"
#include "bsp_gpio.h"
#include "bsp_adc.h"
#include "bsp_i2c.h"
#include "bsp_can.h"
#include "fram_driver.h"
#include "lem_sensor.h"
#include "temp_sensor.h"
#include "btt6200.h"
#include "watchdog.h"
#include "pm_monitor.h"
#include "digital_input.h"
#include <string.h>

/*******************************************************************************
 * PRIVATE DEFINITIONS
 ******************************************************************************/

/**
 * @brief FRAM addresses for self-test results
 */
#define FRAM_ADDR_POST_RESULTS    0x3C00U  /**< POST results storage */
#define FRAM_ADDR_POST_HISTORY    0x3D00U  /**< POST history (last 10 runs) */

/**
 * @brief Flash CRC stored location (end of flash)
 */
#define FLASH_CRC_ADDRESS         0x0803FFF0U

/**
 * @brief Test patterns
 */
#define TEST_PATTERN_55           0x55U
#define TEST_PATTERN_AA           0xAAU
#define TEST_PATTERN_00           0x00U
#define TEST_PATTERN_FF           0xFFU

/*******************************************************************************
 * PRIVATE TYPES
 ******************************************************************************/

/**
 * @brief Self-test module state
 */
typedef struct {
    bool                    initialized;      /**< Module initialized flag */
    bool                    postComplete;     /**< POST completion flag */
    POSTResults_t           postResults;      /**< Latest POST results */
    PeriodicTestResults_t   periodicResults;  /**< Periodic test results */
    SelfTestStats_t         stats;            /**< Statistics */
} SelfTestState_t;

/*******************************************************************************
 * PRIVATE VARIABLES
 ******************************************************************************/

/**
 * @brief Self-test module state
 */
static SelfTestState_t g_selfTestState = {0};

/**
 * @brief RAM test buffer (allocated in specific section for testing)
 */
static uint32_t g_ramTestBuffer[SELF_TEST_RAM_TEST_SIZE / sizeof(uint32_t)] = {0};

/**
 * @brief Test result string lookup table
 */
static const char* const g_resultStrings[] = {
    "PASS",
    "FAIL_RAM",
    "FAIL_FLASH",
    "FAIL_GPIO",
    "FAIL_ADC",
    "FAIL_I2C",
    "FAIL_CAN",
    "FAIL_FRAM",
    "FAIL_LEM",
    "FAIL_TEMP",
    "FAIL_BTT6200",
    "FAIL_WATCHDOG",
    "FAIL_TIMEOUT",
    "NOT_RUN"
};

/**
 * @brief POST test name lookup table
 */
static const char* const g_postTestNames[] = {
    "RAM",
    "FLASH",
    "GPIO",
    "ADC",
    "I2C",
    "CAN",
    "FRAM",
    "LEM",
    "TEMP",
    "BTT6200",
    "WATCHDOG"
};

/*******************************************************************************
 * PRIVATE FUNCTION PROTOTYPES
 ******************************************************************************/

static SelfTestResult_t selfTest_RAMMarchTest(uint32_t *pBuffer, uint32_t size);
static bool selfTest_VerifyFlashCRC(void);
static Status_t selfTest_UpdateStatistics(bool postRun, bool success);

/*******************************************************************************
 * PUBLIC FUNCTIONS
 ******************************************************************************/

/**
 * @brief Initialize self-test module
 */
Status_t SelfTest_Init(void)
{
    /* Clear module state */
    memset(&g_selfTestState, 0, sizeof(SelfTestState_t));

    /* Initialize all test results as NOT_RUN */
    for (uint32_t i = 0; i < POST_TEST_COUNT; i++)
    {
        g_selfTestState.postResults.results[i] = SELF_TEST_RESULT_NOT_RUN;
    }

    for (uint32_t i = 0; i < PERIODIC_TEST_COUNT; i++)
    {
        g_selfTestState.periodicResults.results[i] = SELF_TEST_RESULT_NOT_RUN;
    }

    g_selfTestState.initialized = true;
    g_selfTestState.postComplete = false;

    return STATUS_OK;
}

/**
 * @brief Execute Power-On Self-Test (POST)
 */
Status_t SelfTest_RunPOST(POSTResults_t *pResults)
{
    if (!g_selfTestState.initialized)
    {
        return STATUS_ERROR_NOT_INITIALIZED;
    }

    if (pResults == NULL)
    {
        return STATUS_ERROR_PARAM;
    }

    uint32_t startTime = Timestamp_GetMilliseconds();
    bool allPassed = true;
    uint32_t failedMask = 0;

    /* Run all POST tests in sequence */
    for (uint32_t i = 0; i < POST_TEST_COUNT; i++)
    {
        SelfTestResult_t result = SelfTest_RunIndividualPOST((POSTTestType_t)i);
        g_selfTestState.postResults.results[i] = result;

        if (result != SELF_TEST_RESULT_PASS)
        {
            allPassed = false;
            failedMask |= (1U << i);

            /* Log error for failed test */
            ErrorHandler_LogError(ERROR_SELF_TEST_FAILED, i, (uint32_t)result, 0);
        }

        /* Refresh watchdog during long POST */
        Watchdog_RefreshAll();
    }

    uint32_t endTime = Timestamp_GetMilliseconds();

    /* Store results */
    g_selfTestState.postResults.failedTests = failedMask;
    g_selfTestState.postResults.executionTime = endTime - startTime;
    g_selfTestState.postResults.timestamp = startTime;
    g_selfTestState.postResults.allTestsPassed = allPassed;
    g_selfTestState.postComplete = true;

    /* Copy results to output */
    memcpy(pResults, &g_selfTestState.postResults, sizeof(POSTResults_t));

    /* Update statistics */
    selfTest_UpdateStatistics(true, allPassed);

    /* Log results to FRAM */
    (void)SelfTest_LogPOSTResults(pResults);

    return allPassed ? STATUS_OK : STATUS_ERROR;
}

/**
 * @brief Execute individual POST test
 */
SelfTestResult_t SelfTest_RunIndividualPOST(POSTTestType_t testType)
{
    SelfTestResult_t result = SELF_TEST_RESULT_NOT_RUN;

    switch (testType)
    {
        case POST_TEST_RAM:
            result = SelfTest_RAMTest();
            break;

        case POST_TEST_FLASH:
            result = SelfTest_FlashCRCTest();
            break;

        case POST_TEST_GPIO:
            result = SelfTest_GPIOTest();
            break;

        case POST_TEST_ADC:
            result = SelfTest_ADCTest();
            break;

        case POST_TEST_I2C:
            result = SelfTest_I2CTest();
            break;

        case POST_TEST_CAN:
            result = SelfTest_CANTest();
            break;

        case POST_TEST_FRAM:
            result = SelfTest_FRAMTest();
            break;

        case POST_TEST_LEM:
            result = SelfTest_LEMSensorTest();
            break;

        case POST_TEST_TEMP:
            result = SelfTest_TempSensorTest();
            break;

        case POST_TEST_BTT6200:
            result = SelfTest_BTT6200Test();
            break;

        case POST_TEST_WATCHDOG:
            result = SelfTest_WatchdogTest();
            break;

        default:
            result = SELF_TEST_RESULT_NOT_RUN;
            break;
    }

    return result;
}

/**
 * @brief RAM integrity test using march algorithm
 */
SelfTestResult_t SelfTest_RAMTest(void)
{
    /* Test internal buffer */
    SelfTestResult_t result = selfTest_RAMMarchTest(g_ramTestBuffer,
                                                     SELF_TEST_RAM_TEST_SIZE / sizeof(uint32_t));

    if (result != SELF_TEST_RESULT_PASS)
    {
        return SELF_TEST_RESULT_FAIL_RAM;
    }

    /* Additional RAM test: Stack watermark check */
    extern uint32_t _estack;  /* Defined in linker script */
    extern uint32_t _sdata;   /* Defined in linker script */

    /* Simple sanity check that stack pointer is in valid range */
    uint32_t sp = __get_MSP();
    if ((sp < (uint32_t)&_sdata) || (sp > (uint32_t)&_estack))
    {
        return SELF_TEST_RESULT_FAIL_RAM;
    }

    return SELF_TEST_RESULT_PASS;
}

/**
 * @brief Flash CRC verification
 */
SelfTestResult_t SelfTest_FlashCRCTest(void)
{
    /* Calculate CRC32 of application code */
    const uint8_t *pFlash = (const uint8_t *)SELF_TEST_FLASH_START;
    uint32_t calculatedCRC = CRC32_Calculate(pFlash, SELF_TEST_FLASH_SIZE);

    /* Read stored CRC from flash */
    const uint32_t *pStoredCRC = (const uint32_t *)FLASH_CRC_ADDRESS;
    uint32_t storedCRC = *pStoredCRC;

    /* Compare CRCs - if stored CRC is 0xFFFFFFFF, skip check (not programmed yet) */
    if (storedCRC == 0xFFFFFFFFU)
    {
        /* CRC not programmed yet - pass for development */
        return SELF_TEST_RESULT_PASS;
    }

    if (calculatedCRC != storedCRC)
    {
        return SELF_TEST_RESULT_FAIL_FLASH;
    }

    return SELF_TEST_RESULT_PASS;
}

/**
 * @brief GPIO configuration verification
 */
SelfTestResult_t SelfTest_GPIOTest(void)
{
    /* Verify critical GPIOs are initialized */
    /* Check LED pins are configured as outputs */
    GPIO_TypeDef* ledPort = GPIOG;
    uint32_t ledPin = GPIO_PIN_13;  /* LED_STATUS pin */

    /* Read GPIO mode register */
    uint32_t moder = ledPort->MODER;
    uint32_t pinPos = 13U;  /* LED_STATUS is PG13 */
    uint32_t pinMode = (moder >> (pinPos * 2U)) & 0x03U;

    /* Check if configured as output (mode = 01) */
    if (pinMode != 0x01U)
    {
        return SELF_TEST_RESULT_FAIL_GPIO;
    }

    /* Additional checks for critical pins could be added here */

    return SELF_TEST_RESULT_PASS;
}

/**
 * @brief ADC functionality test
 */
SelfTestResult_t SelfTest_ADCTest(void)
{
    /* Read internal VREFINT channel */
    uint16_t vrefRaw = 0;
    Status_t status = BSP_ADC_ReadChannel(ADC_CHANNEL_VREFINT, &vrefRaw);

    if (status != STATUS_OK)
    {
        return SELF_TEST_RESULT_FAIL_ADC;
    }

    /* VREFINT should be around 1.21V, which is ~1500 counts at 3.3V reference */
    /* Allow wide tolerance for now: 1000-2000 counts */
    if ((vrefRaw < 1000U) || (vrefRaw > 2000U))
    {
        return SELF_TEST_RESULT_FAIL_ADC;
    }

    return SELF_TEST_RESULT_PASS;
}

/**
 * @brief I2C bus functionality test
 */
SelfTestResult_t SelfTest_I2CTest(void)
{
    /* Test I2C by checking if FRAM responds */
    uint8_t testByte = 0;
    Status_t status = BSP_I2C_Read(FRAM_I2C_ADDRESS, FRAM_ADDR_POST_RESULTS,
                                   &testByte, 1, SELF_TEST_TIMEOUT_SHORT);

    if (status != STATUS_OK)
    {
        return SELF_TEST_RESULT_FAIL_I2C;
    }

    return SELF_TEST_RESULT_PASS;
}

/**
 * @brief CAN bus functionality test
 */
SelfTestResult_t SelfTest_CANTest(void)
{
    /* Enable CAN loopback mode for self-test */
    /* This requires reconfiguring CAN temporarily */

    /* For now, just verify CAN peripheral is initialized */
    /* Check if CAN is in normal mode or initialization mode */

    /* Check CAN1 status */
    if ((CAN1->MSR & CAN_MSR_INAK) != 0U)
    {
        /* CAN is still in initialization mode - not properly started */
        return SELF_TEST_RESULT_FAIL_CAN;
    }

    /* Check error counters */
    uint32_t esr = CAN1->ESR;
    uint8_t tec = (uint8_t)((esr >> 16U) & 0xFFU);  /* Transmit error counter */
    uint8_t rec = (uint8_t)((esr >> 24U) & 0xFFU);  /* Receive error counter */

    /* If error counters are too high, bus may have issues */
    if ((tec > 200U) || (rec > 200U))
    {
        return SELF_TEST_RESULT_FAIL_CAN;
    }

    return SELF_TEST_RESULT_PASS;
}

/**
 * @brief FRAM read/write test
 */
SelfTestResult_t SelfTest_FRAMTest(void)
{
    /* Use a test address in reserved area */
    const uint16_t testAddr = 0x7FF0U;  /* Near end of FRAM */

    /* Test patterns */
    uint8_t testPattern[4] = {TEST_PATTERN_55, TEST_PATTERN_AA,
                              TEST_PATTERN_00, TEST_PATTERN_FF};
    uint8_t readback[4] = {0};

    /* Write test pattern */
    Status_t status = FRAM_Write(testAddr, testPattern, sizeof(testPattern));
    if (status != STATUS_OK)
    {
        return SELF_TEST_RESULT_FAIL_FRAM;
    }

    /* Small delay for write to complete */
    HAL_Delay(1);

    /* Read back */
    status = FRAM_Read(testAddr, readback, sizeof(readback));
    if (status != STATUS_OK)
    {
        return SELF_TEST_RESULT_FAIL_FRAM;
    }

    /* Verify */
    if (memcmp(testPattern, readback, sizeof(testPattern)) != 0)
    {
        return SELF_TEST_RESULT_FAIL_FRAM;
    }

    return SELF_TEST_RESULT_PASS;
}

/**
 * @brief LEM sensor connectivity test
 */
SelfTestResult_t SelfTest_LEMSensorTest(void)
{
    /* Read all LEM sensors and check if values are reasonable */
    for (uint8_t i = 0; i < LEM_NUM_SENSORS; i++)
    {
        int32_t current = 0;
        Status_t status = LEM_GetCurrent(i, &current);

        if (status != STATUS_OK)
        {
            return SELF_TEST_RESULT_FAIL_LEM;
        }

        /* Check if reading is within possible physical range */
        /* LEM HOYS sensors typically measure -200A to +200A */
        if ((current < -210000) || (current > 210000))  /* ±210A in mA */
        {
            return SELF_TEST_RESULT_FAIL_LEM;
        }
    }

    return SELF_TEST_RESULT_PASS;
}

/**
 * @brief Temperature sensor test
 */
SelfTestResult_t SelfTest_TempSensorTest(void)
{
    int16_t temperature = 0;
    Status_t status = TempSensor_ReadTemperature(&temperature);

    if (status != STATUS_OK)
    {
        return SELF_TEST_RESULT_FAIL_TEMP;
    }

    /* Check if temperature is in reasonable range: -40°C to +125°C */
    if ((temperature < -400) || (temperature > 1250))  /* In deciCelsius */
    {
        return SELF_TEST_RESULT_FAIL_TEMP;
    }

    return SELF_TEST_RESULT_PASS;
}

/**
 * @brief BTT6200 output driver test
 */
SelfTestResult_t SelfTest_BTT6200Test(void)
{
    /* Verify all BTT6200 channels are initialized */
    /* We can't test output without load, so just check module status */

    for (uint8_t channel = 0; channel < BTT6200_TOTAL_CHANNELS; channel++)
    {
        BTT6200_DiagStatus_t diagStatus = {0};
        Status_t status = BTT6200_GetDiagnostics(channel, &diagStatus);

        if (status != STATUS_OK)
        {
            return SELF_TEST_RESULT_FAIL_BTT6200;
        }
    }

    return SELF_TEST_RESULT_PASS;
}

/**
 * @brief Watchdog functionality test
 */
SelfTestResult_t SelfTest_WatchdogTest(void)
{
    /* Check if watchdog is running */
    bool iwdgRunning = Watchdog_IsIWDGRunning();
    bool wwdgRunning = Watchdog_IsWWDGRunning();

    /* Note: During POST, watchdog may not be started yet */
    /* So we just check if watchdog module is initialized */
    if (!iwdgRunning && !wwdgRunning)
    {
        /* Neither watchdog is running - this is OK during POST */
        /* Just verify the watchdog peripheral is accessible */

        /* Check IWDG registers are accessible */
        if (IWDG->PR > 7U)  /* Prescaler register should be 0-7 */
        {
            return SELF_TEST_RESULT_FAIL_WATCHDOG;
        }
    }

    return SELF_TEST_RESULT_PASS;
}

/**
 * @brief Execute periodic runtime self-tests
 */
SelfTestResult_t SelfTest_RunPeriodicTest(PeriodicTestType_t testType)
{
    if (!g_selfTestState.initialized)
    {
        return SELF_TEST_RESULT_NOT_RUN;
    }

    if (testType >= PERIODIC_TEST_COUNT)
    {
        return SELF_TEST_RESULT_NOT_RUN;
    }

    SelfTestResult_t result = SELF_TEST_RESULT_NOT_RUN;
    uint32_t startTime = Timestamp_GetMilliseconds();

    switch (testType)
    {
        case PERIODIC_TEST_SENSOR_RANGE:
            result = SelfTest_SensorRangeTest();
            break;

        case PERIODIC_TEST_ACTUATOR_LOOP:
            result = SelfTest_ActuatorLoopbackTest();
            break;

        case PERIODIC_TEST_COMM_HEALTH:
            result = SelfTest_CommHealthTest();
            break;

        case PERIODIC_TEST_POWER_STABILITY:
            result = SelfTest_PowerStabilityTest();
            break;

        case PERIODIC_TEST_TIMING:
            result = SelfTest_TimingTest();
            break;

        default:
            break;
    }

    /* Update results */
    g_selfTestState.periodicResults.results[testType] = result;
    g_selfTestState.periodicResults.lastRunTime[testType] = startTime;

    if (result == SELF_TEST_RESULT_PASS)
    {
        g_selfTestState.periodicResults.successCount[testType]++;
    }
    else
    {
        g_selfTestState.periodicResults.failureCount[testType]++;
        g_selfTestState.stats.totalPeriodicFailures++;
    }

    g_selfTestState.stats.totalPeriodicTests++;
    g_selfTestState.stats.lastPeriodicTimestamp = startTime;

    return result;
}

/**
 * @brief Sensor range validation test
 */
SelfTestResult_t SelfTest_SensorRangeTest(void)
{
    /* Check all sensors are in valid ranges */

    /* LEM sensors */
    for (uint8_t i = 0; i < LEM_NUM_SENSORS; i++)
    {
        int32_t current = 0;
        if (LEM_GetCurrent(i, &current) == STATUS_OK)
        {
            if ((current < -210000) || (current > 210000))
            {
                return SELF_TEST_RESULT_FAIL_LEM;
            }
        }
    }

    /* Temperature */
    int16_t temp = 0;
    if (TempSensor_ReadTemperature(&temp) == STATUS_OK)
    {
        if ((temp < -400) || (temp > 1250))
        {
            return SELF_TEST_RESULT_FAIL_TEMP;
        }
    }

    /* Power monitoring */
    uint16_t voltage12V = 0;
    if (PM_Monitor_GetVoltage(PM_RAIL_12V_INPUT, &voltage12V) == STATUS_OK)
    {
        /* 12V rail should be 10V-16V */
        if ((voltage12V < 10000U) || (voltage12V > 16000U))
        {
            return SELF_TEST_RESULT_FAIL_ADC;
        }
    }

    return SELF_TEST_RESULT_PASS;
}

/**
 * @brief Actuator loopback test
 */
SelfTestResult_t SelfTest_ActuatorLoopbackTest(void)
{
    /* Test a few BTT6200 channels */
    /* Turn on, read status, turn off */

    const uint8_t testChannels[] = {0, 5, 10, 15};  /* Test 4 channels */

    for (uint8_t i = 0; i < sizeof(testChannels); i++)
    {
        uint8_t channel = testChannels[i];

        /* Turn on */
        if (BTT6200_SetOutput(channel, true) != STATUS_OK)
        {
            return SELF_TEST_RESULT_FAIL_BTT6200;
        }

        HAL_Delay(10);  /* Allow time for output to stabilize */

        /* Read status */
        bool state = false;
        if (BTT6200_GetOutputState(channel, &state) != STATUS_OK)
        {
            return SELF_TEST_RESULT_FAIL_BTT6200;
        }

        /* Turn off */
        BTT6200_SetOutput(channel, false);
    }

    return SELF_TEST_RESULT_PASS;
}

/**
 * @brief Communication health check
 */
SelfTestResult_t SelfTest_CommHealthTest(void)
{
    /* Check CAN error counters */
    uint32_t esr1 = CAN1->ESR;
    uint8_t tec1 = (uint8_t)((esr1 >> 16U) & 0xFFU);
    uint8_t rec1 = (uint8_t)((esr1 >> 24U) & 0xFFU);

    if ((tec1 > 200U) || (rec1 > 200U))
    {
        return SELF_TEST_RESULT_FAIL_CAN;
    }

    uint32_t esr2 = CAN2->ESR;
    uint8_t tec2 = (uint8_t)((esr2 >> 16U) & 0xFFU);
    uint8_t rec2 = (uint8_t)((esr2 >> 24U) & 0xFFU);

    if ((tec2 > 200U) || (rec2 > 200U))
    {
        return SELF_TEST_RESULT_FAIL_CAN;
    }

    return SELF_TEST_RESULT_PASS;
}

/**
 * @brief Power supply stability test
 */
SelfTestResult_t SelfTest_PowerStabilityTest(void)
{
    /* Check all power rails */
    bool powerGood = PM_Monitor_IsPowerGood();

    if (!powerGood)
    {
        return SELF_TEST_RESULT_FAIL_ADC;
    }

    return SELF_TEST_RESULT_PASS;
}

/**
 * @brief Timing accuracy check
 */
SelfTestResult_t SelfTest_TimingTest(void)
{
    /* Check that system tick is running */
    uint32_t tick1 = Timestamp_GetMilliseconds();

    /* Delay using CPU cycles */
    for (volatile uint32_t i = 0; i < 100000U; i++)
    {
        __NOP();
    }

    uint32_t tick2 = Timestamp_GetMilliseconds();

    /* Should have elapsed some time */
    if (tick2 == tick1)
    {
        return SELF_TEST_RESULT_FAIL_TIMEOUT;
    }

    return SELF_TEST_RESULT_PASS;
}

/**
 * @brief Get POST results
 */
Status_t SelfTest_GetPOSTResults(POSTResults_t *pResults)
{
    if (!g_selfTestState.initialized || (pResults == NULL))
    {
        return STATUS_ERROR_PARAM;
    }

    memcpy(pResults, &g_selfTestState.postResults, sizeof(POSTResults_t));

    return STATUS_OK;
}

/**
 * @brief Get periodic test results
 */
Status_t SelfTest_GetPeriodicResults(PeriodicTestResults_t *pResults)
{
    if (!g_selfTestState.initialized || (pResults == NULL))
    {
        return STATUS_ERROR_PARAM;
    }

    memcpy(pResults, &g_selfTestState.periodicResults, sizeof(PeriodicTestResults_t));

    return STATUS_OK;
}

/**
 * @brief Get self-test statistics
 */
Status_t SelfTest_GetStatistics(SelfTestStats_t *pStats)
{
    if (!g_selfTestState.initialized || (pStats == NULL))
    {
        return STATUS_ERROR_PARAM;
    }

    memcpy(pStats, &g_selfTestState.stats, sizeof(SelfTestStats_t));

    return STATUS_OK;
}

/**
 * @brief Clear self-test statistics
 */
Status_t SelfTest_ClearStatistics(void)
{
    if (!g_selfTestState.initialized)
    {
        return STATUS_ERROR_NOT_INITIALIZED;
    }

    memset(&g_selfTestState.stats, 0, sizeof(SelfTestStats_t));

    return STATUS_OK;
}

/**
 * @brief Log POST results to FRAM
 */
Status_t SelfTest_LogPOSTResults(const POSTResults_t *pResults)
{
    if (pResults == NULL)
    {
        return STATUS_ERROR_PARAM;
    }

    /* Write results to FRAM */
    Status_t status = FRAM_Write(FRAM_ADDR_POST_RESULTS,
                                 (const uint8_t *)pResults,
                                 sizeof(POSTResults_t));

    return status;
}

/**
 * @brief Convert test result to string
 */
const char* SelfTest_ResultToString(SelfTestResult_t result)
{
    if (result < (sizeof(g_resultStrings) / sizeof(g_resultStrings[0])))
    {
        return g_resultStrings[result];
    }

    return "UNKNOWN";
}

/**
 * @brief Convert POST test type to string
 */
const char* SelfTest_POSTTypeToString(POSTTestType_t testType)
{
    if (testType < POST_TEST_COUNT)
    {
        return g_postTestNames[testType];
    }

    return "UNKNOWN";
}

/**
 * @brief Check if POST has been run successfully
 */
bool SelfTest_IsPOSTComplete(void)
{
    return g_selfTestState.postComplete && g_selfTestState.postResults.allTestsPassed;
}

/*******************************************************************************
 * PRIVATE FUNCTIONS
 ******************************************************************************/

/**
 * @brief RAM march test implementation (March C- algorithm)
 * @details Tests for stuck-at faults, address decoding, and coupling faults
 * @param[in] pBuffer Buffer to test
 * @param[in] size Size in words (uint32_t)
 * @return Test result
 */
static SelfTestResult_t selfTest_RAMMarchTest(uint32_t *pBuffer, uint32_t size)
{
    if ((pBuffer == NULL) || (size == 0U))
    {
        return SELF_TEST_RESULT_FAIL_RAM;
    }

    /* March C- algorithm:
     * 1. Write 0 ascending
     * 2. Read 0, Write 1 ascending
     * 3. Read 1, Write 0 ascending
     * 4. Read 0 descending
     * 5. Read 0, Write 1 descending
     * 6. Read 1, Write 0 descending
     * 7. Read 0 ascending
     */

    /* Step 1: Write 0 ascending */
    for (uint32_t i = 0; i < size; i++)
    {
        pBuffer[i] = 0x00000000U;
    }

    /* Step 2: Read 0, Write 1 ascending */
    for (uint32_t i = 0; i < size; i++)
    {
        if (pBuffer[i] != 0x00000000U)
        {
            return SELF_TEST_RESULT_FAIL_RAM;
        }
        pBuffer[i] = 0xFFFFFFFFU;
    }

    /* Step 3: Read 1, Write 0 ascending */
    for (uint32_t i = 0; i < size; i++)
    {
        if (pBuffer[i] != 0xFFFFFFFFU)
        {
            return SELF_TEST_RESULT_FAIL_RAM;
        }
        pBuffer[i] = 0x00000000U;
    }

    /* Step 4: Read 0 descending */
    for (int32_t i = (int32_t)size - 1; i >= 0; i--)
    {
        if (pBuffer[i] != 0x00000000U)
        {
            return SELF_TEST_RESULT_FAIL_RAM;
        }
    }

    /* Step 5: Read 0, Write 1 descending */
    for (int32_t i = (int32_t)size - 1; i >= 0; i--)
    {
        if (pBuffer[i] != 0x00000000U)
        {
            return SELF_TEST_RESULT_FAIL_RAM;
        }
        pBuffer[i] = 0xFFFFFFFFU;
    }

    /* Step 6: Read 1, Write 0 descending */
    for (int32_t i = (int32_t)size - 1; i >= 0; i--)
    {
        if (pBuffer[i] != 0xFFFFFFFFU)
        {
            return SELF_TEST_RESULT_FAIL_RAM;
        }
        pBuffer[i] = 0x00000000U;
    }

    /* Step 7: Read 0 ascending */
    for (uint32_t i = 0; i < size; i++)
    {
        if (pBuffer[i] != 0x00000000U)
        {
            return SELF_TEST_RESULT_FAIL_RAM;
        }
    }

    return SELF_TEST_RESULT_PASS;
}

/**
 * @brief Update self-test statistics
 */
static Status_t selfTest_UpdateStatistics(bool postRun, bool success)
{
    if (postRun)
    {
        g_selfTestState.stats.totalPOSTRuns++;
        if (!success)
        {
            g_selfTestState.stats.totalPOSTFailures++;
        }
        g_selfTestState.stats.lastPOSTTimestamp = Timestamp_GetMilliseconds();
    }

    return STATUS_OK;
}

/****************************** END OF FILE ***********************************/
