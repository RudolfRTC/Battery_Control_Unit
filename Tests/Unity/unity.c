/**
 * @file    unity.c
 * @brief   Unity Test Framework - Implementation
 * @version 1.0.0
 */

#include "unity.h"

/*============================================================================*/
/* GLOBAL VARIABLES                                                           */
/*============================================================================*/

Unity_t Unity;

/*============================================================================*/
/* FUNCTION IMPLEMENTATIONS                                                   */
/*============================================================================*/

void UnityPass(void)
{
    /* Nothing to do - test passed */
}

void UnityFail(uint32_t line, const char* message)
{
    Unity.CurrentTestFailed = 1;
    printf("  FAIL at line %u: %s\n", line, message);
}

void UnityIgnore(uint32_t line, const char* message)
{
    Unity.CurrentTestIgnored = 1;
    printf("  IGNORE at line %u: %s\n", line, message);
}

void UnityFailIntEqual(uint32_t line, int expected, int actual, const char* name)
{
    Unity.CurrentTestFailed = 1;
    printf("  FAIL at line %u: %s expected %d was %d\n", line, name, expected, actual);
}

void UnityFailUIntEqual(uint32_t line, uint32_t expected, uint32_t actual, const char* name)
{
    Unity.CurrentTestFailed = 1;
    printf("  FAIL at line %u: %s expected %u was %u\n", line, name, expected, actual);
}

void UnityFailHexEqual(uint32_t line, uint32_t expected, uint32_t actual, uint8_t bits, const char* name)
{
    Unity.CurrentTestFailed = 1;
    if (bits == 8) {
        printf("  FAIL at line %u: %s expected 0x%02X was 0x%02X\n", line, name, expected, actual);
    } else if (bits == 16) {
        printf("  FAIL at line %u: %s expected 0x%04X was 0x%04X\n", line, name, expected, actual);
    } else {
        printf("  FAIL at line %u: %s expected 0x%08X was 0x%08X\n", line, name, expected, actual);
    }
}

void UnityFailGreater(uint32_t line, int threshold, int actual, const char* name)
{
    Unity.CurrentTestFailed = 1;
    printf("  FAIL at line %u: %s expected > %d was %d\n", line, name, threshold, actual);
}

void UnityFailLess(uint32_t line, int threshold, int actual, const char* name)
{
    Unity.CurrentTestFailed = 1;
    printf("  FAIL at line %u: %s expected < %d was %d\n", line, name, threshold, actual);
}

void UnityFailWithin(uint32_t line, int delta, int expected, int actual, const char* name)
{
    Unity.CurrentTestFailed = 1;
    printf("  FAIL at line %u: %s expected %d +/- %d was %d\n", line, name, expected, delta, actual);
}

void UnityFailMemory(uint32_t line, const void* expected, const void* actual, uint32_t len)
{
    Unity.CurrentTestFailed = 1;
    printf("  FAIL at line %u: Memory mismatch in %u bytes\n", line, len);
    
    /* Show first difference */
    const uint8_t* pExp = (const uint8_t*)expected;
    const uint8_t* pAct = (const uint8_t*)actual;
    for (uint32_t i = 0; i < len; i++) {
        if (pExp[i] != pAct[i]) {
            printf("    First diff at offset %u: expected 0x%02X was 0x%02X\n", i, pExp[i], pAct[i]);
            break;
        }
    }
}

void UnityFailString(uint32_t line, const char* expected, const char* actual)
{
    Unity.CurrentTestFailed = 1;
    printf("  FAIL at line %u: String mismatch\n", line);
    printf("    Expected: \"%s\"\n", expected);
    printf("    Actual:   \"%s\"\n", actual);
}

/*============================================================================*/
/* END OF FILE                                                                */
/*============================================================================*/
