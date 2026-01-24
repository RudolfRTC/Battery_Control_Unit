/**
 * @file    unity.h
 * @brief   Unity Test Framework - Minimal Implementation
 * @version 1.0.0
 * @note    Based on ThrowTheSwitch Unity framework
 * @note    Simplified for BCU firmware testing
 */

#ifndef UNITY_H
#define UNITY_H

#ifdef __cplusplus
extern "C" {
#endif

/*============================================================================*/
/* INCLUDES                                                                   */
/*============================================================================*/
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

/*============================================================================*/
/* CONFIGURATION                                                              */
/*============================================================================*/
#ifndef UNITY_OUTPUT_CHAR
#define UNITY_OUTPUT_CHAR(c)    putchar(c)
#endif

#ifndef UNITY_OUTPUT_STRING
#define UNITY_OUTPUT_STRING(s)  printf("%s", s)
#endif

/*============================================================================*/
/* GLOBAL STATE                                                               */
/*============================================================================*/
typedef struct {
    const char* TestFile;
    const char* TestFunc;
    uint32_t    TestLine;
    uint32_t    NumberOfTests;
    uint32_t    NumberOfFailures;
    uint32_t    NumberOfIgnored;
    uint8_t     CurrentTestFailed;
    uint8_t     CurrentTestIgnored;
} Unity_t;

extern Unity_t Unity;

/*============================================================================*/
/* MACROS - TEST FRAMEWORK                                                    */
/*============================================================================*/

#define TEST_PASS()         UnityPass()
#define TEST_FAIL()         UnityFail(__LINE__, "Test failed")
#define TEST_FAIL_MESSAGE(msg) UnityFail(__LINE__, msg)
#define TEST_IGNORE()       UnityIgnore(__LINE__, "Test ignored")
#define TEST_IGNORE_MESSAGE(msg) UnityIgnore(__LINE__, msg)

/*============================================================================*/
/* MACROS - ASSERTIONS                                                        */
/*============================================================================*/

/* Boolean assertions */
#define TEST_ASSERT(condition) \
    do { if (!(condition)) { UnityFail(__LINE__, #condition " was FALSE"); } } while(0)

#define TEST_ASSERT_TRUE(condition) \
    do { if (!(condition)) { UnityFail(__LINE__, #condition " expected TRUE was FALSE"); } } while(0)

#define TEST_ASSERT_FALSE(condition) \
    do { if (condition) { UnityFail(__LINE__, #condition " expected FALSE was TRUE"); } } while(0)

#define TEST_ASSERT_NULL(pointer) \
    do { if ((pointer) != NULL) { UnityFail(__LINE__, #pointer " expected NULL"); } } while(0)

#define TEST_ASSERT_NOT_NULL(pointer) \
    do { if ((pointer) == NULL) { UnityFail(__LINE__, #pointer " was NULL"); } } while(0)

/* Integer equality assertions */
#define TEST_ASSERT_EQUAL(expected, actual) \
    TEST_ASSERT_EQUAL_INT((expected), (actual))

#define TEST_ASSERT_EQUAL_INT(expected, actual) \
    do { if ((int)(expected) != (int)(actual)) { \
        UnityFailIntEqual(__LINE__, (int)(expected), (int)(actual), #actual); \
    } } while(0)

#define TEST_ASSERT_EQUAL_INT8(expected, actual) \
    do { if ((int8_t)(expected) != (int8_t)(actual)) { \
        UnityFailIntEqual(__LINE__, (int)(expected), (int)(actual), #actual); \
    } } while(0)

#define TEST_ASSERT_EQUAL_INT16(expected, actual) \
    do { if ((int16_t)(expected) != (int16_t)(actual)) { \
        UnityFailIntEqual(__LINE__, (int)(expected), (int)(actual), #actual); \
    } } while(0)

#define TEST_ASSERT_EQUAL_INT32(expected, actual) \
    do { if ((int32_t)(expected) != (int32_t)(actual)) { \
        UnityFailIntEqual(__LINE__, (int32_t)(expected), (int32_t)(actual), #actual); \
    } } while(0)

#define TEST_ASSERT_EQUAL_UINT(expected, actual) \
    TEST_ASSERT_EQUAL_UINT32((expected), (actual))

#define TEST_ASSERT_EQUAL_UINT8(expected, actual) \
    do { if ((uint8_t)(expected) != (uint8_t)(actual)) { \
        UnityFailUIntEqual(__LINE__, (uint32_t)(expected), (uint32_t)(actual), #actual); \
    } } while(0)

#define TEST_ASSERT_EQUAL_UINT16(expected, actual) \
    do { if ((uint16_t)(expected) != (uint16_t)(actual)) { \
        UnityFailUIntEqual(__LINE__, (uint32_t)(expected), (uint32_t)(actual), #actual); \
    } } while(0)

#define TEST_ASSERT_EQUAL_UINT32(expected, actual) \
    do { if ((uint32_t)(expected) != (uint32_t)(actual)) { \
        UnityFailUIntEqual(__LINE__, (uint32_t)(expected), (uint32_t)(actual), #actual); \
    } } while(0)

#define TEST_ASSERT_EQUAL_HEX(expected, actual) \
    TEST_ASSERT_EQUAL_HEX32((expected), (actual))

#define TEST_ASSERT_EQUAL_HEX8(expected, actual) \
    do { if ((uint8_t)(expected) != (uint8_t)(actual)) { \
        UnityFailHexEqual(__LINE__, (uint32_t)(expected), (uint32_t)(actual), 8, #actual); \
    } } while(0)

#define TEST_ASSERT_EQUAL_HEX16(expected, actual) \
    do { if ((uint16_t)(expected) != (uint16_t)(actual)) { \
        UnityFailHexEqual(__LINE__, (uint32_t)(expected), (uint32_t)(actual), 16, #actual); \
    } } while(0)

#define TEST_ASSERT_EQUAL_HEX32(expected, actual) \
    do { if ((uint32_t)(expected) != (uint32_t)(actual)) { \
        UnityFailHexEqual(__LINE__, (uint32_t)(expected), (uint32_t)(actual), 32, #actual); \
    } } while(0)

/* Comparison assertions */
#define TEST_ASSERT_GREATER_THAN(threshold, actual) \
    do { if ((int)(actual) <= (int)(threshold)) { \
        UnityFailGreater(__LINE__, (int)(threshold), (int)(actual), #actual); \
    } } while(0)

#define TEST_ASSERT_LESS_THAN(threshold, actual) \
    do { if ((int)(actual) >= (int)(threshold)) { \
        UnityFailLess(__LINE__, (int)(threshold), (int)(actual), #actual); \
    } } while(0)

#define TEST_ASSERT_INT_WITHIN(delta, expected, actual) \
    do { \
        int _diff = (int)(actual) - (int)(expected); \
        if (_diff < 0) _diff = -_diff; \
        if (_diff > (int)(delta)) { \
            UnityFailWithin(__LINE__, (int)(delta), (int)(expected), (int)(actual), #actual); \
        } \
    } while(0)

/* Memory comparison assertions */
#define TEST_ASSERT_EQUAL_MEMORY(expected, actual, len) \
    do { if (memcmp((expected), (actual), (len)) != 0) { \
        UnityFailMemory(__LINE__, (expected), (actual), (len)); \
    } } while(0)

#define TEST_ASSERT_EQUAL_STRING(expected, actual) \
    do { if (strcmp((expected), (actual)) != 0) { \
        UnityFailString(__LINE__, (expected), (actual)); \
    } } while(0)

/*============================================================================*/
/* RUN TEST MACROS                                                            */
/*============================================================================*/

#define RUN_TEST(func) \
    do { \
        Unity.TestFile = __FILE__; \
        Unity.TestFunc = #func; \
        Unity.CurrentTestFailed = 0; \
        Unity.CurrentTestIgnored = 0; \
        setUp(); \
        func(); \
        tearDown(); \
        Unity.NumberOfTests++; \
        if (Unity.CurrentTestIgnored) { \
            Unity.NumberOfIgnored++; \
            printf("[IGNORE] %s\n", #func); \
        } else if (Unity.CurrentTestFailed) { \
            Unity.NumberOfFailures++; \
            printf("[FAIL] %s\n", #func); \
        } else { \
            printf("[PASS] %s\n", #func); \
        } \
    } while(0)

#define UNITY_BEGIN() \
    do { \
        memset(&Unity, 0, sizeof(Unity)); \
        printf("\n=============================================================\n"); \
        printf("BCU Firmware Unit Tests\n"); \
        printf("=============================================================\n\n"); \
    } while(0)

#define UNITY_END() \
    do { \
        printf("\n=============================================================\n"); \
        printf("TEST SUMMARY\n"); \
        printf("=============================================================\n"); \
        printf("Tests run:    %u\n", Unity.NumberOfTests); \
        printf("Tests passed: %u\n", Unity.NumberOfTests - Unity.NumberOfFailures - Unity.NumberOfIgnored); \
        printf("Tests failed: %u\n", Unity.NumberOfFailures); \
        printf("Tests ignored: %u\n", Unity.NumberOfIgnored); \
        printf("=============================================================\n"); \
        if (Unity.NumberOfFailures == 0) { \
            printf("ALL TESTS PASSED!\n"); \
        } else { \
            printf("SOME TESTS FAILED!\n"); \
        } \
    } while(0)

/*============================================================================*/
/* FUNCTION PROTOTYPES                                                        */
/*============================================================================*/

void UnityPass(void);
void UnityFail(uint32_t line, const char* message);
void UnityIgnore(uint32_t line, const char* message);
void UnityFailIntEqual(uint32_t line, int expected, int actual, const char* name);
void UnityFailUIntEqual(uint32_t line, uint32_t expected, uint32_t actual, const char* name);
void UnityFailHexEqual(uint32_t line, uint32_t expected, uint32_t actual, uint8_t bits, const char* name);
void UnityFailGreater(uint32_t line, int threshold, int actual, const char* name);
void UnityFailLess(uint32_t line, int threshold, int actual, const char* name);
void UnityFailWithin(uint32_t line, int delta, int expected, int actual, const char* name);
void UnityFailMemory(uint32_t line, const void* expected, const void* actual, uint32_t len);
void UnityFailString(uint32_t line, const char* expected, const char* actual);

/* User must implement these */
void setUp(void);
void tearDown(void);

#ifdef __cplusplus
}
#endif

#endif /* UNITY_H */
