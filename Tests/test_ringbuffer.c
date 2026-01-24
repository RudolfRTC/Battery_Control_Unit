/**
 * @file    test_ringbuffer.c
 * @brief   Unit tests for Ring Buffer module
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
#include <stddef.h>
#include <stdbool.h>
#include <string.h>

/* Status type for PC testing */
typedef enum {
    STATUS_OK = 0,
    STATUS_ERROR_PARAM,
    STATUS_ERROR_OVERFLOW,
    STATUS_ERROR_UNDERFLOW,
    STATUS_ERROR
} Status_t;

/* Ring buffer structure */
typedef struct {
    uint8_t *pBuffer;
    uint32_t size;
    volatile uint32_t head;
    volatile uint32_t tail;
} RingBuffer_t;

/* Ring buffer functions */
Status_t RingBuffer_Init(RingBuffer_t *pRB, uint8_t *pBuffer, uint32_t size)
{
    Status_t status = STATUS_ERROR_PARAM;
    if ((pRB != NULL) && (pBuffer != NULL) && (size > 0U))
    {
        pRB->pBuffer = pBuffer;
        pRB->size = size;
        pRB->head = 0U;
        pRB->tail = 0U;
        status = STATUS_OK;
    }
    return status;
}

uint32_t RingBuffer_Available(const RingBuffer_t *pRB)
{
    uint32_t available = 0U;
    if (pRB != NULL)
    {
        if (pRB->head >= pRB->tail)
        {
            available = pRB->head - pRB->tail;
        }
        else
        {
            available = pRB->size - pRB->tail + pRB->head;
        }
    }
    return available;
}

uint32_t RingBuffer_Free(const RingBuffer_t *pRB)
{
    uint32_t free_space = 0U;
    if (pRB != NULL)
    {
        free_space = pRB->size - RingBuffer_Available(pRB) - 1U;
    }
    return free_space;
}

Status_t RingBuffer_Write(RingBuffer_t *pRB, const uint8_t *pData, uint32_t length)
{
    Status_t status = STATUS_ERROR_PARAM;
    if ((pRB != NULL) && (pData != NULL) && (length > 0U))
    {
        uint32_t free_space = RingBuffer_Free(pRB);
        if (length <= free_space)
        {
            uint32_t i;
            for (i = 0U; i < length; i++)
            {
                pRB->pBuffer[pRB->head] = pData[i];
                pRB->head = (pRB->head + 1U) % pRB->size;
            }
            status = STATUS_OK;
        }
        else
        {
            status = STATUS_ERROR_OVERFLOW;
        }
    }
    return status;
}

Status_t RingBuffer_Read(RingBuffer_t *pRB, uint8_t *pData, uint32_t length)
{
    Status_t status = STATUS_ERROR_PARAM;
    if ((pRB != NULL) && (pData != NULL) && (length > 0U))
    {
        uint32_t available = RingBuffer_Available(pRB);
        if (length <= available)
        {
            uint32_t i;
            for (i = 0U; i < length; i++)
            {
                pData[i] = pRB->pBuffer[pRB->tail];
                pRB->tail = (pRB->tail + 1U) % pRB->size;
            }
            status = STATUS_OK;
        }
        else
        {
            status = STATUS_ERROR_UNDERFLOW;
        }
    }
    return status;
}

Status_t RingBuffer_Peek(const RingBuffer_t *pRB, uint8_t *pData, uint32_t length)
{
    Status_t status = STATUS_ERROR_PARAM;
    if ((pRB != NULL) && (pData != NULL) && (length > 0U))
    {
        uint32_t available = RingBuffer_Available(pRB);
        if (length <= available)
        {
            uint32_t i;
            uint32_t idx = pRB->tail;
            for (i = 0U; i < length; i++)
            {
                pData[i] = pRB->pBuffer[idx];
                idx = (idx + 1U) % pRB->size;
            }
            status = STATUS_OK;
        }
        else
        {
            status = STATUS_ERROR_UNDERFLOW;
        }
    }
    return status;
}

bool RingBuffer_IsEmpty(const RingBuffer_t *pRB)
{
    bool empty = true;
    if (pRB != NULL)
    {
        empty = (pRB->head == pRB->tail);
    }
    return empty;
}

bool RingBuffer_IsFull(const RingBuffer_t *pRB)
{
    bool full = false;
    if (pRB != NULL)
    {
        full = (((pRB->head + 1U) % pRB->size) == pRB->tail);
    }
    return full;
}

void RingBuffer_Clear(RingBuffer_t *pRB)
{
    if (pRB != NULL)
    {
        pRB->head = 0U;
        pRB->tail = 0U;
    }
}

#endif /* TARGET_STM32 */

/*============================================================================*/
/* TEST VARIABLES                                                             */
/*============================================================================*/

static RingBuffer_t testRB;
static uint8_t testBuffer[64];

/*============================================================================*/
/* SETUP / TEARDOWN                                                           */
/*============================================================================*/

void setUp(void)
{
    memset(&testRB, 0, sizeof(testRB));
    memset(testBuffer, 0, sizeof(testBuffer));
    RingBuffer_Init(&testRB, testBuffer, sizeof(testBuffer));
}

void tearDown(void)
{
    /* Nothing to tear down */
}

/*============================================================================*/
/* INITIALIZATION TESTS                                                       */
/*============================================================================*/

void test_RB_Init_ValidParams(void)
{
    RingBuffer_t rb;
    uint8_t buf[16];
    
    Status_t status = RingBuffer_Init(&rb, buf, 16);
    
    TEST_ASSERT_EQUAL(STATUS_OK, status);
    TEST_ASSERT_EQUAL(0, rb.head);
    TEST_ASSERT_EQUAL(0, rb.tail);
    TEST_ASSERT_EQUAL(16, rb.size);
}

void test_RB_Init_NullRB(void)
{
    uint8_t buf[16];
    Status_t status = RingBuffer_Init(NULL, buf, 16);
    TEST_ASSERT_EQUAL(STATUS_ERROR_PARAM, status);
}

void test_RB_Init_NullBuffer(void)
{
    RingBuffer_t rb;
    Status_t status = RingBuffer_Init(&rb, NULL, 16);
    TEST_ASSERT_EQUAL(STATUS_ERROR_PARAM, status);
}

void test_RB_Init_ZeroSize(void)
{
    RingBuffer_t rb;
    uint8_t buf[16];
    Status_t status = RingBuffer_Init(&rb, buf, 0);
    TEST_ASSERT_EQUAL(STATUS_ERROR_PARAM, status);
}

/*============================================================================*/
/* WRITE TESTS                                                                */
/*============================================================================*/

void test_RB_Write_SingleByte(void)
{
    uint8_t data = 0xAB;
    Status_t status = RingBuffer_Write(&testRB, &data, 1);
    
    TEST_ASSERT_EQUAL(STATUS_OK, status);
    TEST_ASSERT_EQUAL(1, RingBuffer_Available(&testRB));
    TEST_ASSERT_EQUAL_HEX8(0xAB, testBuffer[0]);
}

void test_RB_Write_MultipleBytes(void)
{
    uint8_t data[] = {0x01, 0x02, 0x03, 0x04, 0x05};
    Status_t status = RingBuffer_Write(&testRB, data, 5);
    
    TEST_ASSERT_EQUAL(STATUS_OK, status);
    TEST_ASSERT_EQUAL(5, RingBuffer_Available(&testRB));
}

void test_RB_Write_Overflow(void)
{
    uint8_t data[100];
    memset(data, 0xAA, sizeof(data));
    
    /* Buffer is 64 bytes, usable is 63 */
    Status_t status = RingBuffer_Write(&testRB, data, 100);
    
    TEST_ASSERT_EQUAL(STATUS_ERROR_OVERFLOW, status);
    TEST_ASSERT_EQUAL(0, RingBuffer_Available(&testRB));
}

void test_RB_Write_ExactlyFull(void)
{
    uint8_t data[63];  /* 64 - 1 = 63 usable */
    memset(data, 0xBB, sizeof(data));
    
    Status_t status = RingBuffer_Write(&testRB, data, 63);
    
    TEST_ASSERT_EQUAL(STATUS_OK, status);
    TEST_ASSERT_TRUE(RingBuffer_IsFull(&testRB));
}

void test_RB_Write_NullParams(void)
{
    uint8_t data = 0x00;
    
    TEST_ASSERT_EQUAL(STATUS_ERROR_PARAM, RingBuffer_Write(NULL, &data, 1));
    TEST_ASSERT_EQUAL(STATUS_ERROR_PARAM, RingBuffer_Write(&testRB, NULL, 1));
    TEST_ASSERT_EQUAL(STATUS_ERROR_PARAM, RingBuffer_Write(&testRB, &data, 0));
}

/*============================================================================*/
/* READ TESTS                                                                 */
/*============================================================================*/

void test_RB_Read_SingleByte(void)
{
    uint8_t writeData = 0xCD;
    uint8_t readData = 0x00;
    
    RingBuffer_Write(&testRB, &writeData, 1);
    Status_t status = RingBuffer_Read(&testRB, &readData, 1);
    
    TEST_ASSERT_EQUAL(STATUS_OK, status);
    TEST_ASSERT_EQUAL_HEX8(0xCD, readData);
    TEST_ASSERT_EQUAL(0, RingBuffer_Available(&testRB));
}

void test_RB_Read_MultipleBytes(void)
{
    uint8_t writeData[] = {0x11, 0x22, 0x33, 0x44};
    uint8_t readData[4] = {0};
    
    RingBuffer_Write(&testRB, writeData, 4);
    Status_t status = RingBuffer_Read(&testRB, readData, 4);
    
    TEST_ASSERT_EQUAL(STATUS_OK, status);
    TEST_ASSERT_EQUAL_MEMORY(writeData, readData, 4);
}

void test_RB_Read_Underflow(void)
{
    uint8_t data[10];
    
    /* Try to read from empty buffer */
    Status_t status = RingBuffer_Read(&testRB, data, 10);
    
    TEST_ASSERT_EQUAL(STATUS_ERROR_UNDERFLOW, status);
}

void test_RB_Read_PartialUnderflow(void)
{
    uint8_t writeData[] = {0x01, 0x02, 0x03};
    uint8_t readData[10];
    
    RingBuffer_Write(&testRB, writeData, 3);
    
    /* Try to read more than available */
    Status_t status = RingBuffer_Read(&testRB, readData, 10);
    
    TEST_ASSERT_EQUAL(STATUS_ERROR_UNDERFLOW, status);
    /* Data should still be in buffer */
    TEST_ASSERT_EQUAL(3, RingBuffer_Available(&testRB));
}

/*============================================================================*/
/* PEEK TESTS                                                                 */
/*============================================================================*/

void test_RB_Peek_DoesNotRemove(void)
{
    uint8_t writeData[] = {0xAA, 0xBB, 0xCC};
    uint8_t peekData[3] = {0};
    
    RingBuffer_Write(&testRB, writeData, 3);
    
    Status_t status = RingBuffer_Peek(&testRB, peekData, 3);
    
    TEST_ASSERT_EQUAL(STATUS_OK, status);
    TEST_ASSERT_EQUAL_MEMORY(writeData, peekData, 3);
    /* Data should still be available */
    TEST_ASSERT_EQUAL(3, RingBuffer_Available(&testRB));
}

void test_RB_Peek_Underflow(void)
{
    uint8_t data[10];
    
    Status_t status = RingBuffer_Peek(&testRB, data, 10);
    
    TEST_ASSERT_EQUAL(STATUS_ERROR_UNDERFLOW, status);
}

/*============================================================================*/
/* WRAPAROUND TESTS                                                           */
/*============================================================================*/

void test_RB_Wraparound(void)
{
    uint8_t writeData[32];
    uint8_t readData[32];
    
    /* Fill with pattern 1 */
    for (int i = 0; i < 32; i++) writeData[i] = (uint8_t)i;
    RingBuffer_Write(&testRB, writeData, 32);
    
    /* Read half */
    RingBuffer_Read(&testRB, readData, 16);
    
    /* Write more - this will wrap around */
    for (int i = 0; i < 32; i++) writeData[i] = (uint8_t)(i + 100);
    RingBuffer_Write(&testRB, writeData, 32);
    
    /* Read all remaining */
    RingBuffer_Read(&testRB, readData, 16);  /* Rest of first batch */
    RingBuffer_Read(&testRB, readData, 32);  /* Second batch */
    
    /* Verify second batch */
    for (int i = 0; i < 32; i++)
    {
        TEST_ASSERT_EQUAL_UINT8((uint8_t)(i + 100), readData[i]);
    }
}

/*============================================================================*/
/* STATUS TESTS                                                               */
/*============================================================================*/

void test_RB_IsEmpty_Initial(void)
{
    TEST_ASSERT_TRUE(RingBuffer_IsEmpty(&testRB));
}

void test_RB_IsEmpty_AfterWrite(void)
{
    uint8_t data = 0x00;
    RingBuffer_Write(&testRB, &data, 1);
    
    TEST_ASSERT_FALSE(RingBuffer_IsEmpty(&testRB));
}

void test_RB_IsEmpty_AfterReadAll(void)
{
    uint8_t data = 0x00;
    RingBuffer_Write(&testRB, &data, 1);
    RingBuffer_Read(&testRB, &data, 1);
    
    TEST_ASSERT_TRUE(RingBuffer_IsEmpty(&testRB));
}

void test_RB_IsFull_WhenFull(void)
{
    uint8_t data[63];
    RingBuffer_Write(&testRB, data, 63);
    
    TEST_ASSERT_TRUE(RingBuffer_IsFull(&testRB));
}

void test_RB_IsFull_WhenNotFull(void)
{
    uint8_t data = 0x00;
    RingBuffer_Write(&testRB, &data, 1);
    
    TEST_ASSERT_FALSE(RingBuffer_IsFull(&testRB));
}

void test_RB_Clear(void)
{
    uint8_t data[30];
    RingBuffer_Write(&testRB, data, 30);
    
    RingBuffer_Clear(&testRB);
    
    TEST_ASSERT_TRUE(RingBuffer_IsEmpty(&testRB));
    TEST_ASSERT_EQUAL(0, RingBuffer_Available(&testRB));
}

void test_RB_FreeSpace(void)
{
    TEST_ASSERT_EQUAL(63, RingBuffer_Free(&testRB));  /* 64 - 1 */
    
    uint8_t data[10];
    RingBuffer_Write(&testRB, data, 10);
    
    TEST_ASSERT_EQUAL(53, RingBuffer_Free(&testRB));
}

/*============================================================================*/
/* MAIN                                                                       */
/*============================================================================*/

int main(void)
{
    UNITY_BEGIN();
    
    /* Initialization tests */
    RUN_TEST(test_RB_Init_ValidParams);
    RUN_TEST(test_RB_Init_NullRB);
    RUN_TEST(test_RB_Init_NullBuffer);
    RUN_TEST(test_RB_Init_ZeroSize);
    
    /* Write tests */
    RUN_TEST(test_RB_Write_SingleByte);
    RUN_TEST(test_RB_Write_MultipleBytes);
    RUN_TEST(test_RB_Write_Overflow);
    RUN_TEST(test_RB_Write_ExactlyFull);
    RUN_TEST(test_RB_Write_NullParams);
    
    /* Read tests */
    RUN_TEST(test_RB_Read_SingleByte);
    RUN_TEST(test_RB_Read_MultipleBytes);
    RUN_TEST(test_RB_Read_Underflow);
    RUN_TEST(test_RB_Read_PartialUnderflow);
    
    /* Peek tests */
    RUN_TEST(test_RB_Peek_DoesNotRemove);
    RUN_TEST(test_RB_Peek_Underflow);
    
    /* Wraparound tests */
    RUN_TEST(test_RB_Wraparound);
    
    /* Status tests */
    RUN_TEST(test_RB_IsEmpty_Initial);
    RUN_TEST(test_RB_IsEmpty_AfterWrite);
    RUN_TEST(test_RB_IsEmpty_AfterReadAll);
    RUN_TEST(test_RB_IsFull_WhenFull);
    RUN_TEST(test_RB_IsFull_WhenNotFull);
    RUN_TEST(test_RB_Clear);
    RUN_TEST(test_RB_FreeSpace);
    
    UNITY_END();
    
    return Unity.NumberOfFailures;
}

/*============================================================================*/
/* END OF FILE                                                                */
/*============================================================================*/
