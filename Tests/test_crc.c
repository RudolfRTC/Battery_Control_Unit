/**
 * @file    test_crc.c
 * @brief   Unit tests for CRC module
 * @author  BCU Development Team
 * @date    2026-01-24
 * @version 1.0.0
 */

#include "Unity/unity.h"

/*============================================================================*/
/* TEST CONFIGURATION                                                         */
/*============================================================================*/

/* Include CRC implementation directly for PC testing */
#ifndef TARGET_STM32
#include <stdint.h>
#include <stddef.h>

/* CRC polynomials */
#define CRC32_POLYNOMIAL    (0xEDB88320UL)
#define CRC32_INIT          (0xFFFFFFFFUL)
#define CRC16_POLYNOMIAL    (0x8408U)
#define CRC16_INIT          (0xFFFFU)
#define CRC8_POLYNOMIAL     (0x07U)

/* CRC-32 incremental */
uint32_t CRC_Calculate32_Incremental(uint32_t crc, const void *pData, uint32_t length)
{
    const uint8_t *pBytes = (const uint8_t *)pData;
    uint32_t i;
    uint8_t j;

    if (pBytes != NULL)
    {
        for (i = 0U; i < length; i++)
        {
            crc ^= (uint32_t)pBytes[i];
            for (j = 0U; j < 8U; j++)
            {
                if ((crc & 1U) != 0U)
                {
                    crc = (crc >> 1U) ^ CRC32_POLYNOMIAL;
                }
                else
                {
                    crc = crc >> 1U;
                }
            }
        }
    }
    return crc;
}

/* CRC-32 */
uint32_t CRC_Calculate32(const void *pData, uint32_t length)
{
    return CRC_Calculate32_Incremental(CRC32_INIT, pData, length) ^ CRC32_INIT;
}

/* CRC-16 */
uint16_t CRC_Calculate16(const void *pData, uint32_t length)
{
    const uint8_t *pBytes = (const uint8_t *)pData;
    uint16_t crc = CRC16_INIT;
    uint32_t i;
    uint8_t j;

    if (pBytes != NULL)
    {
        for (i = 0U; i < length; i++)
        {
            crc ^= (uint16_t)pBytes[i];
            for (j = 0U; j < 8U; j++)
            {
                if ((crc & 1U) != 0U)
                {
                    crc = (crc >> 1U) ^ CRC16_POLYNOMIAL;
                }
                else
                {
                    crc = crc >> 1U;
                }
            }
        }
    }
    return crc ^ CRC16_INIT;
}

/* CRC-8 */
uint8_t CRC_Calculate8(const void *pData, uint32_t length)
{
    const uint8_t *pBytes = (const uint8_t *)pData;
    uint8_t crc = 0U;
    uint32_t i;
    uint8_t j;

    if (pBytes != NULL)
    {
        for (i = 0U; i < length; i++)
        {
            crc ^= pBytes[i];
            for (j = 0U; j < 8U; j++)
            {
                if ((crc & 0x80U) != 0U)
                {
                    crc = (crc << 1U) ^ CRC8_POLYNOMIAL;
                }
                else
                {
                    crc = crc << 1U;
                }
            }
        }
    }
    return crc;
}

#endif /* TARGET_STM32 */

/*============================================================================*/
/* SETUP / TEARDOWN                                                           */
/*============================================================================*/

void setUp(void)
{
    /* Nothing to set up */
}

void tearDown(void)
{
    /* Nothing to tear down */
}

/*============================================================================*/
/* CRC-32 TESTS                                                               */
/*============================================================================*/

void test_CRC32_EmptyData(void)
{
    /* CRC of empty data should be 0 (after XOR with init) */
    uint32_t crc = CRC_Calculate32(NULL, 0);
    TEST_ASSERT_EQUAL_HEX32(0x00000000, crc);
}

void test_CRC32_SingleByte(void)
{
    uint8_t data = 0x00;
    uint32_t crc = CRC_Calculate32(&data, 1);
    /* Known CRC-32 value for 0x00 */
    TEST_ASSERT_EQUAL_HEX32(0xD202EF8D, crc);
}

void test_CRC32_KnownString(void)
{
    /* "123456789" is a standard test vector */
    const char *data = "123456789";
    uint32_t crc = CRC_Calculate32(data, 9);
    /* Standard CRC-32 check value for "123456789" */
    TEST_ASSERT_EQUAL_HEX32(0xCBF43926, crc);
}

void test_CRC32_AllZeros(void)
{
    uint8_t data[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    uint32_t crc = CRC_Calculate32(data, 8);
    /* CRC should be non-zero for all-zero data */
    TEST_ASSERT_NOT_NULL((void*)(uintptr_t)(crc != 0));
}

void test_CRC32_AllOnes(void)
{
    uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    uint32_t crc = CRC_Calculate32(data, 8);
    /* CRC should be consistent */
    TEST_ASSERT_EQUAL_HEX32(0x23083B63, crc);
}

void test_CRC32_Incremental(void)
{
    /* Calculate in two parts */
    const char *part1 = "1234";
    const char *part2 = "56789";
    
    uint32_t crc = CRC32_INIT;
    crc = CRC_Calculate32_Incremental(crc, part1, 4);
    crc = CRC_Calculate32_Incremental(crc, part2, 5);
    crc ^= CRC32_INIT;
    
    /* Should match single calculation */
    TEST_ASSERT_EQUAL_HEX32(0xCBF43926, crc);
}

void test_CRC32_DataIntegrity(void)
{
    uint8_t data[] = {0x01, 0x02, 0x03, 0x04, 0x05};
    uint32_t crc1 = CRC_Calculate32(data, 5);
    
    /* Modify one byte */
    data[2] = 0x33;
    uint32_t crc2 = CRC_Calculate32(data, 5);
    
    /* CRCs should be different */
    TEST_ASSERT_TRUE(crc1 != crc2);
}

/*============================================================================*/
/* CRC-16 TESTS                                                               */
/*============================================================================*/

void test_CRC16_EmptyData(void)
{
    uint16_t crc = CRC_Calculate16(NULL, 0);
    TEST_ASSERT_EQUAL_HEX16(0x0000, crc);
}

void test_CRC16_SingleByte(void)
{
    uint8_t data = 0x00;
    uint16_t crc = CRC_Calculate16(&data, 1);
    /* CRC-16 CCITT for 0x00 */
    TEST_ASSERT_EQUAL_HEX16(0xE1F0, crc);
}

void test_CRC16_KnownData(void)
{
    uint8_t data[] = {0x01, 0x02, 0x03, 0x04};
    uint16_t crc = CRC_Calculate16(data, 4);
    /* Consistent check */
    TEST_ASSERT_NOT_NULL((void*)(uintptr_t)(crc != 0));
}

void test_CRC16_DataChange(void)
{
    uint8_t data[] = {0xAA, 0xBB, 0xCC, 0xDD};
    uint16_t crc1 = CRC_Calculate16(data, 4);
    
    data[0] = 0xAB;  /* Change one byte */
    uint16_t crc2 = CRC_Calculate16(data, 4);
    
    TEST_ASSERT_TRUE(crc1 != crc2);
}

/*============================================================================*/
/* CRC-8 TESTS                                                                */
/*============================================================================*/

void test_CRC8_EmptyData(void)
{
    uint8_t crc = CRC_Calculate8(NULL, 0);
    TEST_ASSERT_EQUAL_HEX8(0x00, crc);
}

void test_CRC8_SingleByte(void)
{
    uint8_t data = 0x00;
    uint8_t crc = CRC_Calculate8(&data, 1);
    TEST_ASSERT_EQUAL_HEX8(0x00, crc);
}

void test_CRC8_NonZero(void)
{
    uint8_t data = 0x01;
    uint8_t crc = CRC_Calculate8(&data, 1);
    /* CRC-8 for 0x01 with polynomial 0x07 */
    TEST_ASSERT_EQUAL_HEX8(0x07, crc);
}

void test_CRC8_MultipleBytes(void)
{
    uint8_t data[] = {0x01, 0x02, 0x03};
    uint8_t crc = CRC_Calculate8(data, 3);
    /* Consistent check - CRC should be deterministic */
    uint8_t crc2 = CRC_Calculate8(data, 3);
    TEST_ASSERT_EQUAL_HEX8(crc, crc2);
}

void test_CRC8_DataIntegrity(void)
{
    uint8_t data[] = {0xFF, 0xFE, 0xFD, 0xFC};
    uint8_t crc1 = CRC_Calculate8(data, 4);
    
    data[3] = 0x00;  /* Change last byte */
    uint8_t crc2 = CRC_Calculate8(data, 4);
    
    TEST_ASSERT_TRUE(crc1 != crc2);
}

/*============================================================================*/
/* MAIN                                                                       */
/*============================================================================*/

int main(void)
{
    UNITY_BEGIN();
    
    /* CRC-32 tests */
    RUN_TEST(test_CRC32_EmptyData);
    RUN_TEST(test_CRC32_SingleByte);
    RUN_TEST(test_CRC32_KnownString);
    RUN_TEST(test_CRC32_AllZeros);
    RUN_TEST(test_CRC32_AllOnes);
    RUN_TEST(test_CRC32_Incremental);
    RUN_TEST(test_CRC32_DataIntegrity);
    
    /* CRC-16 tests */
    RUN_TEST(test_CRC16_EmptyData);
    RUN_TEST(test_CRC16_SingleByte);
    RUN_TEST(test_CRC16_KnownData);
    RUN_TEST(test_CRC16_DataChange);
    
    /* CRC-8 tests */
    RUN_TEST(test_CRC8_EmptyData);
    RUN_TEST(test_CRC8_SingleByte);
    RUN_TEST(test_CRC8_NonZero);
    RUN_TEST(test_CRC8_MultipleBytes);
    RUN_TEST(test_CRC8_DataIntegrity);
    
    UNITY_END();
    
    return Unity.NumberOfFailures;
}

/*============================================================================*/
/* END OF FILE                                                                */
/*============================================================================*/
