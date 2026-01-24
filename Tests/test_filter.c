/**
 * @file    test_filter.c
 * @brief   Unit tests for Filter module
 * @author  BCU Development Team
 * @date    2026-01-24
 * @version 1.0.0
 */

#include "Unity/unity.h"

/*============================================================================*/
/* TEST CONFIGURATION                                                         */
/*============================================================================*/

#ifndef TARGET_STM32
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

/* Status type */
typedef enum {
    STATUS_OK = 0,
    STATUS_ERROR_PARAM,
    STATUS_ERROR
} Status_t;

#define FILTER_MA_MAX_SIZE  (32U)

/* Moving average filter structure */
typedef struct {
    int32_t buffer[FILTER_MA_MAX_SIZE];
    uint8_t size;
    uint8_t index;
    uint8_t count;
    int32_t sum;
} Filter_MovingAverage_t;

/* IIR filter structure */
typedef struct {
    int32_t output;
    uint8_t alpha;
    uint8_t alphaComplement;
    bool initialized;
} Filter_IIR_t;

/* Debounce filter structure */
typedef struct {
    bool state;
    bool lastRaw;
    uint32_t counter;
    uint32_t threshold;
    uint32_t samplePeriod;
} Filter_Debounce_t;

/* Moving average functions */
Status_t Filter_MA_Init(Filter_MovingAverage_t *pFilter, uint8_t size)
{
    Status_t status = STATUS_ERROR_PARAM;
    if ((pFilter != NULL) && (size > 0U) && (size <= FILTER_MA_MAX_SIZE))
    {
        memset(pFilter, 0, sizeof(Filter_MovingAverage_t));
        pFilter->size = size;
        status = STATUS_OK;
    }
    return status;
}

int32_t Filter_MA_Update(Filter_MovingAverage_t *pFilter, int32_t sample)
{
    int32_t output = sample;
    if (pFilter != NULL)
    {
        if (pFilter->count >= pFilter->size)
        {
            pFilter->sum -= pFilter->buffer[pFilter->index];
        }
        else
        {
            pFilter->count++;
        }
        pFilter->buffer[pFilter->index] = sample;
        pFilter->sum += sample;
        pFilter->index++;
        if (pFilter->index >= pFilter->size)
        {
            pFilter->index = 0U;
        }
        output = pFilter->sum / (int32_t)pFilter->count;
    }
    return output;
}

void Filter_MA_Reset(Filter_MovingAverage_t *pFilter)
{
    if (pFilter != NULL)
    {
        uint8_t size = pFilter->size;
        memset(pFilter, 0, sizeof(Filter_MovingAverage_t));
        pFilter->size = size;
    }
}

/* IIR filter functions */
Status_t Filter_IIR_Init(Filter_IIR_t *pFilter, uint8_t alpha, uint8_t scale)
{
    Status_t status = STATUS_ERROR_PARAM;
    if ((pFilter != NULL) && (alpha <= scale) && (scale > 0U))
    {
        pFilter->output = 0;
        pFilter->alpha = alpha;
        pFilter->alphaComplement = scale - alpha;
        pFilter->initialized = false;
        status = STATUS_OK;
    }
    return status;
}

int32_t Filter_IIR_Update(Filter_IIR_t *pFilter, int32_t sample)
{
    int32_t output = sample;
    if (pFilter != NULL)
    {
        if (!pFilter->initialized)
        {
            pFilter->output = sample;
            pFilter->initialized = true;
        }
        else
        {
            int32_t scale = (int32_t)pFilter->alpha + (int32_t)pFilter->alphaComplement;
            pFilter->output = ((int32_t)pFilter->alpha * sample +
                              (int32_t)pFilter->alphaComplement * pFilter->output) / scale;
        }
        output = pFilter->output;
    }
    return output;
}

void Filter_IIR_Reset(Filter_IIR_t *pFilter)
{
    if (pFilter != NULL)
    {
        pFilter->output = 0;
        pFilter->initialized = false;
    }
}

/* Debounce filter functions */
Status_t Filter_Debounce_Init(Filter_Debounce_t *pFilter, uint32_t threshold_ms, uint32_t samplePeriod_ms)
{
    Status_t status = STATUS_ERROR_PARAM;
    if ((pFilter != NULL) && (samplePeriod_ms > 0U))
    {
        pFilter->state = false;
        pFilter->lastRaw = false;
        pFilter->counter = 0U;
        pFilter->threshold = threshold_ms;
        pFilter->samplePeriod = samplePeriod_ms;
        status = STATUS_OK;
    }
    return status;
}

bool Filter_Debounce_Update(Filter_Debounce_t *pFilter, bool rawState)
{
    bool output = false;
    if (pFilter != NULL)
    {
        if (rawState == pFilter->lastRaw)
        {
            pFilter->counter += pFilter->samplePeriod;
            if (pFilter->counter >= pFilter->threshold)
            {
                pFilter->state = rawState;
                pFilter->counter = pFilter->threshold;
            }
        }
        else
        {
            pFilter->counter = 0U;
            pFilter->lastRaw = rawState;
        }
        output = pFilter->state;
    }
    return output;
}

void Filter_Debounce_Reset(Filter_Debounce_t *pFilter)
{
    if (pFilter != NULL)
    {
        pFilter->state = false;
        pFilter->lastRaw = false;
        pFilter->counter = 0U;
    }
}

#endif /* TARGET_STM32 */

/*============================================================================*/
/* TEST VARIABLES                                                             */
/*============================================================================*/

static Filter_MovingAverage_t maFilter;
static Filter_IIR_t iirFilter;
static Filter_Debounce_t debounceFilter;

/*============================================================================*/
/* SETUP / TEARDOWN                                                           */
/*============================================================================*/

void setUp(void)
{
    memset(&maFilter, 0, sizeof(maFilter));
    memset(&iirFilter, 0, sizeof(iirFilter));
    memset(&debounceFilter, 0, sizeof(debounceFilter));
}

void tearDown(void)
{
    /* Nothing to tear down */
}

/*============================================================================*/
/* MOVING AVERAGE TESTS                                                       */
/*============================================================================*/

void test_MA_Init_ValidParams(void)
{
    Status_t status = Filter_MA_Init(&maFilter, 8);
    
    TEST_ASSERT_EQUAL(STATUS_OK, status);
    TEST_ASSERT_EQUAL(8, maFilter.size);
    TEST_ASSERT_EQUAL(0, maFilter.index);
    TEST_ASSERT_EQUAL(0, maFilter.count);
}

void test_MA_Init_NullFilter(void)
{
    Status_t status = Filter_MA_Init(NULL, 8);
    TEST_ASSERT_EQUAL(STATUS_ERROR_PARAM, status);
}

void test_MA_Init_ZeroSize(void)
{
    Status_t status = Filter_MA_Init(&maFilter, 0);
    TEST_ASSERT_EQUAL(STATUS_ERROR_PARAM, status);
}

void test_MA_Init_TooLargeSize(void)
{
    Status_t status = Filter_MA_Init(&maFilter, 64);  /* > FILTER_MA_MAX_SIZE */
    TEST_ASSERT_EQUAL(STATUS_ERROR_PARAM, status);
}

void test_MA_SingleSample(void)
{
    Filter_MA_Init(&maFilter, 4);
    
    int32_t output = Filter_MA_Update(&maFilter, 100);
    
    TEST_ASSERT_EQUAL(100, output);  /* First sample = output */
}

void test_MA_FillingBuffer(void)
{
    Filter_MA_Init(&maFilter, 4);
    
    Filter_MA_Update(&maFilter, 100);  /* Avg = 100/1 = 100 */
    Filter_MA_Update(&maFilter, 200);  /* Avg = 300/2 = 150 */
    Filter_MA_Update(&maFilter, 300);  /* Avg = 600/3 = 200 */
    int32_t output = Filter_MA_Update(&maFilter, 400);  /* Avg = 1000/4 = 250 */
    
    TEST_ASSERT_EQUAL(250, output);
}

void test_MA_SlidingWindow(void)
{
    Filter_MA_Init(&maFilter, 4);
    
    /* Fill buffer: 100, 200, 300, 400 -> sum = 1000, avg = 250 */
    Filter_MA_Update(&maFilter, 100);
    Filter_MA_Update(&maFilter, 200);
    Filter_MA_Update(&maFilter, 300);
    Filter_MA_Update(&maFilter, 400);
    
    /* Add 500: removes 100, adds 500 -> sum = 1400, avg = 350 */
    int32_t output = Filter_MA_Update(&maFilter, 500);
    
    TEST_ASSERT_EQUAL(350, output);
}

void test_MA_ConstantValue(void)
{
    Filter_MA_Init(&maFilter, 4);
    
    for (int i = 0; i < 10; i++)
    {
        int32_t output = Filter_MA_Update(&maFilter, 100);
        TEST_ASSERT_EQUAL(100, output);
    }
}

void test_MA_NegativeValues(void)
{
    Filter_MA_Init(&maFilter, 4);
    
    Filter_MA_Update(&maFilter, -100);
    Filter_MA_Update(&maFilter, -200);
    Filter_MA_Update(&maFilter, -300);
    int32_t output = Filter_MA_Update(&maFilter, -400);
    
    TEST_ASSERT_EQUAL(-250, output);
}

void test_MA_Reset(void)
{
    Filter_MA_Init(&maFilter, 4);
    
    Filter_MA_Update(&maFilter, 100);
    Filter_MA_Update(&maFilter, 200);
    
    Filter_MA_Reset(&maFilter);
    
    TEST_ASSERT_EQUAL(4, maFilter.size);  /* Size preserved */
    TEST_ASSERT_EQUAL(0, maFilter.count);
    TEST_ASSERT_EQUAL(0, maFilter.sum);
    
    /* First sample after reset */
    int32_t output = Filter_MA_Update(&maFilter, 50);
    TEST_ASSERT_EQUAL(50, output);
}

/*============================================================================*/
/* IIR FILTER TESTS                                                           */
/*============================================================================*/

void test_IIR_Init_ValidParams(void)
{
    Status_t status = Filter_IIR_Init(&iirFilter, 10, 100);
    
    TEST_ASSERT_EQUAL(STATUS_OK, status);
    TEST_ASSERT_EQUAL(10, iirFilter.alpha);
    TEST_ASSERT_EQUAL(90, iirFilter.alphaComplement);
    TEST_ASSERT_FALSE(iirFilter.initialized);
}

void test_IIR_Init_NullFilter(void)
{
    Status_t status = Filter_IIR_Init(NULL, 10, 100);
    TEST_ASSERT_EQUAL(STATUS_ERROR_PARAM, status);
}

void test_IIR_Init_AlphaGreaterThanScale(void)
{
    Status_t status = Filter_IIR_Init(&iirFilter, 150, 100);
    TEST_ASSERT_EQUAL(STATUS_ERROR_PARAM, status);
}

void test_IIR_FirstSample(void)
{
    Filter_IIR_Init(&iirFilter, 10, 100);
    
    int32_t output = Filter_IIR_Update(&iirFilter, 1000);
    
    TEST_ASSERT_EQUAL(1000, output);  /* First sample = output */
    TEST_ASSERT_TRUE(iirFilter.initialized);
}

void test_IIR_LowPassBehavior(void)
{
    /* alpha = 10, so 10% new sample + 90% old output */
    Filter_IIR_Init(&iirFilter, 10, 100);
    
    Filter_IIR_Update(&iirFilter, 0);     /* output = 0 */
    int32_t output = Filter_IIR_Update(&iirFilter, 1000);  /* 10*1000 + 90*0 / 100 = 100 */
    
    TEST_ASSERT_EQUAL(100, output);
}

void test_IIR_HighAlphaFastResponse(void)
{
    /* alpha = 90, so 90% new sample + 10% old output */
    Filter_IIR_Init(&iirFilter, 90, 100);
    
    Filter_IIR_Update(&iirFilter, 0);
    int32_t output = Filter_IIR_Update(&iirFilter, 1000);  /* 90*1000 + 10*0 / 100 = 900 */
    
    TEST_ASSERT_EQUAL(900, output);
}

void test_IIR_ConvergesToConstant(void)
{
    Filter_IIR_Init(&iirFilter, 50, 100);
    
    /* Apply constant input many times */
    int32_t output = 0;
    for (int i = 0; i < 50; i++)
    {
        output = Filter_IIR_Update(&iirFilter, 1000);
    }
    
    /* Should converge close to 1000 */
    TEST_ASSERT_INT_WITHIN(10, 1000, output);
}

void test_IIR_Reset(void)
{
    Filter_IIR_Init(&iirFilter, 10, 100);
    
    Filter_IIR_Update(&iirFilter, 500);
    Filter_IIR_Update(&iirFilter, 600);
    
    Filter_IIR_Reset(&iirFilter);
    
    TEST_ASSERT_FALSE(iirFilter.initialized);
    TEST_ASSERT_EQUAL(0, iirFilter.output);
}

/*============================================================================*/
/* DEBOUNCE FILTER TESTS                                                      */
/*============================================================================*/

void test_Debounce_Init_ValidParams(void)
{
    Status_t status = Filter_Debounce_Init(&debounceFilter, 50, 10);
    
    TEST_ASSERT_EQUAL(STATUS_OK, status);
    TEST_ASSERT_EQUAL(50, debounceFilter.threshold);
    TEST_ASSERT_EQUAL(10, debounceFilter.samplePeriod);
    TEST_ASSERT_FALSE(debounceFilter.state);
}

void test_Debounce_Init_NullFilter(void)
{
    Status_t status = Filter_Debounce_Init(NULL, 50, 10);
    TEST_ASSERT_EQUAL(STATUS_ERROR_PARAM, status);
}

void test_Debounce_Init_ZeroSamplePeriod(void)
{
    Status_t status = Filter_Debounce_Init(&debounceFilter, 50, 0);
    TEST_ASSERT_EQUAL(STATUS_ERROR_PARAM, status);
}

void test_Debounce_NoChangeUntilThreshold(void)
{
    Filter_Debounce_Init(&debounceFilter, 50, 10);  /* 50ms threshold, 10ms period */
    
    /* Apply TRUE for 4 samples (40ms < 50ms threshold) */
    Filter_Debounce_Update(&debounceFilter, true);
    Filter_Debounce_Update(&debounceFilter, true);
    Filter_Debounce_Update(&debounceFilter, true);
    bool output = Filter_Debounce_Update(&debounceFilter, true);
    
    TEST_ASSERT_FALSE(output);  /* Still false, threshold not reached */
}

void test_Debounce_ChangeAfterThreshold(void)
{
    Filter_Debounce_Init(&debounceFilter, 50, 10);
    
    /* Apply TRUE for 6 samples (60ms >= 50ms threshold) */
    for (int i = 0; i < 5; i++)
    {
        Filter_Debounce_Update(&debounceFilter, true);
    }
    bool output = Filter_Debounce_Update(&debounceFilter, true);
    
    TEST_ASSERT_TRUE(output);  /* Threshold reached */
}

void test_Debounce_GlitchRejection(void)
{
    Filter_Debounce_Init(&debounceFilter, 50, 10);
    
    /* Apply TRUE for 3 samples, then FALSE (glitch) */
    Filter_Debounce_Update(&debounceFilter, true);
    Filter_Debounce_Update(&debounceFilter, true);
    Filter_Debounce_Update(&debounceFilter, true);
    bool output = Filter_Debounce_Update(&debounceFilter, false);  /* Counter resets */
    
    TEST_ASSERT_FALSE(output);  /* Glitch rejected */
    TEST_ASSERT_EQUAL(0, debounceFilter.counter);  /* Counter reset */
}

void test_Debounce_StableAfterChange(void)
{
    Filter_Debounce_Init(&debounceFilter, 30, 10);
    
    /* Reach TRUE state */
    for (int i = 0; i < 5; i++)
    {
        Filter_Debounce_Update(&debounceFilter, true);
    }
    
    TEST_ASSERT_TRUE(debounceFilter.state);
    
    /* Continue with TRUE - should stay TRUE */
    for (int i = 0; i < 5; i++)
    {
        bool output = Filter_Debounce_Update(&debounceFilter, true);
        TEST_ASSERT_TRUE(output);
    }
}

void test_Debounce_TransitionBackToFalse(void)
{
    Filter_Debounce_Init(&debounceFilter, 30, 10);
    
    /* Get to TRUE state */
    for (int i = 0; i < 5; i++)
    {
        Filter_Debounce_Update(&debounceFilter, true);
    }
    
    /* Now transition back to FALSE */
    for (int i = 0; i < 5; i++)
    {
        Filter_Debounce_Update(&debounceFilter, false);
    }
    
    TEST_ASSERT_FALSE(debounceFilter.state);
}

void test_Debounce_Reset(void)
{
    Filter_Debounce_Init(&debounceFilter, 30, 10);
    
    /* Get to TRUE state */
    for (int i = 0; i < 5; i++)
    {
        Filter_Debounce_Update(&debounceFilter, true);
    }
    
    Filter_Debounce_Reset(&debounceFilter);
    
    TEST_ASSERT_FALSE(debounceFilter.state);
    TEST_ASSERT_FALSE(debounceFilter.lastRaw);
    TEST_ASSERT_EQUAL(0, debounceFilter.counter);
}

/*============================================================================*/
/* MAIN                                                                       */
/*============================================================================*/

int main(void)
{
    UNITY_BEGIN();
    
    /* Moving Average tests */
    RUN_TEST(test_MA_Init_ValidParams);
    RUN_TEST(test_MA_Init_NullFilter);
    RUN_TEST(test_MA_Init_ZeroSize);
    RUN_TEST(test_MA_Init_TooLargeSize);
    RUN_TEST(test_MA_SingleSample);
    RUN_TEST(test_MA_FillingBuffer);
    RUN_TEST(test_MA_SlidingWindow);
    RUN_TEST(test_MA_ConstantValue);
    RUN_TEST(test_MA_NegativeValues);
    RUN_TEST(test_MA_Reset);
    
    /* IIR Filter tests */
    RUN_TEST(test_IIR_Init_ValidParams);
    RUN_TEST(test_IIR_Init_NullFilter);
    RUN_TEST(test_IIR_Init_AlphaGreaterThanScale);
    RUN_TEST(test_IIR_FirstSample);
    RUN_TEST(test_IIR_LowPassBehavior);
    RUN_TEST(test_IIR_HighAlphaFastResponse);
    RUN_TEST(test_IIR_ConvergesToConstant);
    RUN_TEST(test_IIR_Reset);
    
    /* Debounce Filter tests */
    RUN_TEST(test_Debounce_Init_ValidParams);
    RUN_TEST(test_Debounce_Init_NullFilter);
    RUN_TEST(test_Debounce_Init_ZeroSamplePeriod);
    RUN_TEST(test_Debounce_NoChangeUntilThreshold);
    RUN_TEST(test_Debounce_ChangeAfterThreshold);
    RUN_TEST(test_Debounce_GlitchRejection);
    RUN_TEST(test_Debounce_StableAfterChange);
    RUN_TEST(test_Debounce_TransitionBackToFalse);
    RUN_TEST(test_Debounce_Reset);
    
    UNITY_END();
    
    return Unity.NumberOfFailures;
}

/*============================================================================*/
/* END OF FILE                                                                */
/*============================================================================*/
