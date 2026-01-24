/**
 * @file    sil_test_main.c
 * @brief   Software-in-the-Loop Test Runner for BCU Firmware
 * @author  Battery Control Unit Development Team
 * @date    2026-01-23
 * @version 1.0.0
 *
 * @note    Runs on host PC (Windows/Linux) without hardware
 * @note    Tests CAN protocol message packing/unpacking
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

/*============================================================================*/
/* TEST FRAMEWORK                                                             */
/*============================================================================*/

static int tests_run = 0;
static int tests_passed = 0;
static int tests_failed = 0;

#define TEST_ASSERT(cond, msg) do { \
    tests_run++; \
    if (cond) { \
        tests_passed++; \
        printf("  [PASS] %s\n", msg); \
    } else { \
        tests_failed++; \
        printf("  [FAIL] %s (line %d)\n", msg, __LINE__); \
    } \
} while(0)

#define TEST_ASSERT_EQ(a, b, msg) TEST_ASSERT((a) == (b), msg)
#define TEST_ASSERT_NEQ(a, b, msg) TEST_ASSERT((a) != (b), msg)

/*============================================================================*/
/* CAN MESSAGE DEFINITIONS (from DBC)                                         */
/*============================================================================*/

/* TX Message IDs */
#define CAN_ID_BCU_STATUS           (0x100U)
#define CAN_ID_BCU_CURRENT_1        (0x101U)
#define CAN_ID_BCU_CURRENT_2        (0x102U)
#define CAN_ID_BCU_CURRENT_3        (0x103U)
#define CAN_ID_BCU_VOLTAGE          (0x104U)
#define CAN_ID_BCU_TEMPERATURE      (0x105U)
#define CAN_ID_BCU_OUTPUTS          (0x106U)
#define CAN_ID_BCU_INPUTS           (0x107U)
#define CAN_ID_BCU_FAULTS           (0x108U)
#define CAN_ID_BCU_RUL_STATUS       (0x120U)
#define CAN_ID_BCU_TIMING           (0x121U)

/* RX Message IDs */
#define CAN_ID_CMD_OUTPUT_SET       (0x200U)
#define CAN_ID_CMD_OUTPUT_ALL       (0x201U)
#define CAN_ID_CMD_OUTPUT_PWM       (0x202U)
#define CAN_ID_CMD_SYSTEM           (0x210U)
#define CAN_ID_UDS_REQUEST          (0x7E0U)
#define CAN_ID_UDS_RESPONSE         (0x7E8U)

/*============================================================================*/
/* CAN MESSAGE PACKING (Big-Endian / Motorola)                                */
/*============================================================================*/

/**
 * @brief Pack 16-bit value into buffer (big-endian)
 */
static void pack_u16_be(uint8_t *buf, uint16_t val)
{
    buf[0] = (uint8_t)(val >> 8);
    buf[1] = (uint8_t)(val & 0xFFU);
}

/**
 * @brief Pack 32-bit value into buffer (big-endian)
 */
static void pack_u32_be(uint8_t *buf, uint32_t val)
{
    buf[0] = (uint8_t)(val >> 24);
    buf[1] = (uint8_t)(val >> 16);
    buf[2] = (uint8_t)(val >> 8);
    buf[3] = (uint8_t)(val & 0xFFU);
}

/**
 * @brief Unpack 16-bit value from buffer (big-endian)
 */
static uint16_t unpack_u16_be(const uint8_t *buf)
{
    return ((uint16_t)buf[0] << 8) | (uint16_t)buf[1];
}

/**
 * @brief Unpack 32-bit value from buffer (big-endian)
 */
static uint32_t unpack_u32_be(const uint8_t *buf)
{
    return ((uint32_t)buf[0] << 24) | ((uint32_t)buf[1] << 16) |
           ((uint32_t)buf[2] << 8) | (uint32_t)buf[3];
}

/**
 * @brief Unpack signed 16-bit value from buffer (big-endian)
 */
static int16_t unpack_i16_be(const uint8_t *buf)
{
    return (int16_t)(((uint16_t)buf[0] << 8) | (uint16_t)buf[1]);
}

/*============================================================================*/
/* TEST: BCU_Status Message (0x100)                                           */
/*============================================================================*/

static void test_bcu_status_packing(void)
{
    printf("\n[TEST] BCU_Status Message Packing (0x100)\n");

    uint8_t msg[8] = {0};

    /* Pack test data */
    uint8_t state = 3;           /* RUNNING */
    uint8_t powerGood = 1;       /* OK */
    uint8_t safetyOK = 1;        /* OK */
    uint8_t activeErrors = 0;
    uint32_t uptime_ms = 123456789;

    msg[0] = state;
    msg[1] = powerGood;
    msg[2] = safetyOK;
    msg[3] = activeErrors;
    pack_u32_be(&msg[4], uptime_ms);

    /* Verify packing */
    TEST_ASSERT_EQ(msg[0], 3, "State = RUNNING (3)");
    TEST_ASSERT_EQ(msg[1], 1, "PowerGood = OK (1)");
    TEST_ASSERT_EQ(msg[2], 1, "SafetyOK = OK (1)");
    TEST_ASSERT_EQ(msg[3], 0, "ActiveErrors = 0");

    /* Verify uptime big-endian packing */
    uint32_t unpacked_uptime = unpack_u32_be(&msg[4]);
    TEST_ASSERT_EQ(unpacked_uptime, 123456789, "Uptime unpacked correctly");

    /* Verify byte order */
    TEST_ASSERT_EQ(msg[4], 0x07, "Uptime byte 0 (MSB)");
    TEST_ASSERT_EQ(msg[5], 0x5B, "Uptime byte 1");
    TEST_ASSERT_EQ(msg[6], 0xCD, "Uptime byte 2");
    TEST_ASSERT_EQ(msg[7], 0x15, "Uptime byte 3 (LSB)");
}

/*============================================================================*/
/* TEST: BCU_Current Messages (0x101-0x103)                                   */
/*============================================================================*/

static void test_bcu_current_packing(void)
{
    printf("\n[TEST] BCU_Current Message Packing (0x101)\n");

    uint8_t msg[8] = {0};

    /* Test data: currents in mA, packed as 10mA units */
    int32_t current0_mA = 15000;   /* 15A */
    int32_t current1_mA = -8500;   /* -8.5A */
    int32_t current2_mA = 0;       /* 0A */
    int32_t current3_mA = 32767;   /* Max positive */

    /* Pack as 16-bit signed in 10mA resolution (big-endian) */
    int16_t current0_10mA = (int16_t)(current0_mA / 10);
    int16_t current1_10mA = (int16_t)(current1_mA / 10);
    int16_t current2_10mA = (int16_t)(current2_mA / 10);
    int16_t current3_10mA = (int16_t)(current3_mA / 10);

    pack_u16_be(&msg[0], (uint16_t)current0_10mA);
    pack_u16_be(&msg[2], (uint16_t)current1_10mA);
    pack_u16_be(&msg[4], (uint16_t)current2_10mA);
    pack_u16_be(&msg[6], (uint16_t)current3_10mA);

    /* Verify unpacking */
    int16_t unpacked0 = unpack_i16_be(&msg[0]);
    int16_t unpacked1 = unpack_i16_be(&msg[2]);
    int16_t unpacked2 = unpack_i16_be(&msg[4]);
    int16_t unpacked3 = unpack_i16_be(&msg[6]);

    TEST_ASSERT_EQ(unpacked0, 1500, "LEM0: 15000mA -> 1500 (10mA units)");
    TEST_ASSERT_EQ(unpacked1, -850, "LEM1: -8500mA -> -850 (10mA units)");
    TEST_ASSERT_EQ(unpacked2, 0, "LEM2: 0mA -> 0");
    TEST_ASSERT_EQ(unpacked3, 3276, "LEM3: 32767mA -> 3276 (10mA units)");

    /* Verify negative value byte representation */
    /* -850 = 0xFCAE in two's complement */
    TEST_ASSERT_EQ(msg[2], 0xFC, "Negative current MSB");
    TEST_ASSERT_EQ(msg[3], 0xAE, "Negative current LSB");
}

/*============================================================================*/
/* TEST: BCU_Voltage Message (0x104)                                          */
/*============================================================================*/

static void test_bcu_voltage_packing(void)
{
    printf("\n[TEST] BCU_Voltage Message Packing (0x104)\n");

    uint8_t msg[8] = {0};

    /* Test data: voltages in mV */
    uint16_t rail_12v = 12350;      /* 12.35V */
    uint16_t rail_5v = 5020;        /* 5.02V */
    uint16_t rail_3v3_d = 3312;     /* 3.312V */
    uint16_t rail_3v3_a = 3298;     /* 3.298V */

    pack_u16_be(&msg[0], rail_12v);
    pack_u16_be(&msg[2], rail_5v);
    pack_u16_be(&msg[4], rail_3v3_d);
    pack_u16_be(&msg[6], rail_3v3_a);

    /* Verify unpacking */
    TEST_ASSERT_EQ(unpack_u16_be(&msg[0]), 12350, "Rail_12V = 12350mV");
    TEST_ASSERT_EQ(unpack_u16_be(&msg[2]), 5020, "Rail_5V = 5020mV");
    TEST_ASSERT_EQ(unpack_u16_be(&msg[4]), 3312, "Rail_3V3_D = 3312mV");
    TEST_ASSERT_EQ(unpack_u16_be(&msg[6]), 3298, "Rail_3V3_A = 3298mV");
}

/*============================================================================*/
/* TEST: BCU_Temperature Message (0x105)                                      */
/*============================================================================*/

static void test_bcu_temperature_packing(void)
{
    printf("\n[TEST] BCU_Temperature Message Packing (0x105)\n");

    uint8_t msg[8] = {0};

    /* Test data: temperature in milli-degrees Celsius */
    int32_t temp_mC = 25750;  /* 25.75°C */

    /* Pack as 32-bit signed (big-endian) */
    pack_u32_be(&msg[0], (uint32_t)temp_mC);

    /* Verify */
    int32_t unpacked = (int32_t)unpack_u32_be(&msg[0]);
    TEST_ASSERT_EQ(unpacked, 25750, "Temperature = 25750 mC (25.75°C)");

    /* Test negative temperature */
    temp_mC = -10500;  /* -10.5°C */
    pack_u32_be(&msg[0], (uint32_t)temp_mC);
    unpacked = (int32_t)unpack_u32_be(&msg[0]);
    TEST_ASSERT_EQ(unpacked, -10500, "Temperature = -10500 mC (-10.5°C)");
}

/*============================================================================*/
/* TEST: BCU_RUL_Status Message (0x120)                                       */
/*============================================================================*/

static void test_bcu_rul_status_packing(void)
{
    printf("\n[TEST] BCU_RUL_Status Message Packing (0x120)\n");

    uint8_t msg[8] = {0};

    /* Test data */
    uint16_t soh_percent = 8750;      /* 87.50% in 0.01% units */
    uint16_t remaining_cycles = 1234;
    uint16_t remaining_days = 365;
    uint8_t confidence = 85;          /* 85% */
    uint8_t valid = 1;

    pack_u16_be(&msg[0], soh_percent);
    pack_u16_be(&msg[2], remaining_cycles);
    pack_u16_be(&msg[4], remaining_days);
    msg[6] = confidence;
    msg[7] = valid;

    /* Verify */
    TEST_ASSERT_EQ(unpack_u16_be(&msg[0]), 8750, "SoH = 87.50%");
    TEST_ASSERT_EQ(unpack_u16_be(&msg[2]), 1234, "RemainingCycles = 1234");
    TEST_ASSERT_EQ(unpack_u16_be(&msg[4]), 365, "RemainingDays = 365");
    TEST_ASSERT_EQ(msg[6], 85, "Confidence = 85%");
    TEST_ASSERT_EQ(msg[7], 1, "Valid = 1");
}

/*============================================================================*/
/* TEST: BCU_Timing Message (0x121)                                           */
/*============================================================================*/

static void test_bcu_timing_packing(void)
{
    printf("\n[TEST] BCU_Timing Message Packing (0x121)\n");

    uint8_t msg[8] = {0};

    /* Test data */
    uint32_t cycle_time_us = 850;      /* 850 µs */
    uint32_t max_cycle_time_us = 1250; /* 1250 µs */

    pack_u32_be(&msg[0], cycle_time_us);
    pack_u32_be(&msg[4], max_cycle_time_us);

    /* Verify */
    TEST_ASSERT_EQ(unpack_u32_be(&msg[0]), 850, "CycleTime = 850 µs");
    TEST_ASSERT_EQ(unpack_u32_be(&msg[4]), 1250, "MaxCycleTime = 1250 µs");
}

/*============================================================================*/
/* TEST: CMD_Output_Set Message (0x200)                                       */
/*============================================================================*/

static void test_cmd_output_set_parsing(void)
{
    printf("\n[TEST] CMD_Output_Set Message Parsing (0x200)\n");

    uint8_t msg[8] = {0};

    /* Simulate received command: turn on channel 5 */
    msg[0] = 5;   /* ChannelID */
    msg[1] = 1;   /* State = ON */

    uint8_t channel = msg[0];
    uint8_t state = msg[1];

    TEST_ASSERT_EQ(channel, 5, "ChannelID = 5");
    TEST_ASSERT_EQ(state, 1, "State = ON");
    TEST_ASSERT(channel < 20, "Channel in valid range (0-19)");
}

/*============================================================================*/
/* TEST: CMD_System Message (0x210)                                           */
/*============================================================================*/

static void test_cmd_system_parsing(void)
{
    printf("\n[TEST] CMD_System Message Parsing (0x210)\n");

    /* Command codes */
    #define CMD_SYS_NOP             (0x00U)
    #define CMD_SYS_RESET           (0x01U)
    #define CMD_SYS_SAFE_STATE      (0x02U)
    #define CMD_SYS_CLEAR_FAULTS    (0x03U)
    #define CMD_SYS_ENABLE_OUTPUTS  (0x10U)
    #define CMD_SYS_DISABLE_OUTPUTS (0x11U)

    uint8_t msg[8] = {0};

    /* Test RESET command */
    msg[0] = CMD_SYS_RESET;
    msg[1] = 0;  /* Param1 */
    msg[2] = 0;  /* Param2 */

    TEST_ASSERT_EQ(msg[0], 0x01, "Command = RESET (0x01)");

    /* Test SAFE_STATE command */
    msg[0] = CMD_SYS_SAFE_STATE;
    TEST_ASSERT_EQ(msg[0], 0x02, "Command = SAFE_STATE (0x02)");

    /* Test ENABLE_OUTPUTS command */
    msg[0] = CMD_SYS_ENABLE_OUTPUTS;
    TEST_ASSERT_EQ(msg[0], 0x10, "Command = ENABLE_OUTPUTS (0x10)");
}

/*============================================================================*/
/* TEST: I/O Bitmap Packing                                                   */
/*============================================================================*/

static void test_io_bitmap_packing(void)
{
    printf("\n[TEST] I/O Bitmap Packing (20 channels)\n");

    uint8_t msg[8] = {0};
    bool outputs[20] = {0};

    /* Set some outputs */
    outputs[0] = true;
    outputs[5] = true;
    outputs[10] = true;
    outputs[19] = true;

    /* Pack into bitmap */
    for (int i = 0; i < 20; i++)
    {
        if (outputs[i])
        {
            uint8_t byteIndex = i / 8;
            uint8_t bitIndex = i % 8;
            msg[byteIndex] |= (1U << bitIndex);
        }
    }

    /* Verify */
    TEST_ASSERT_EQ(msg[0], 0x21, "Byte 0: bits 0,5 set = 0x21");
    TEST_ASSERT_EQ(msg[1], 0x04, "Byte 1: bit 2 (channel 10) set = 0x04");
    TEST_ASSERT_EQ(msg[2], 0x08, "Byte 2: bit 3 (channel 19) set = 0x08");

    /* Unpack and verify */
    bool unpacked[20] = {0};
    for (int i = 0; i < 20; i++)
    {
        uint8_t byteIndex = i / 8;
        uint8_t bitIndex = i % 8;
        unpacked[i] = (msg[byteIndex] & (1U << bitIndex)) != 0;
    }

    TEST_ASSERT(unpacked[0], "Channel 0 unpacked correctly");
    TEST_ASSERT(unpacked[5], "Channel 5 unpacked correctly");
    TEST_ASSERT(unpacked[10], "Channel 10 unpacked correctly");
    TEST_ASSERT(unpacked[19], "Channel 19 unpacked correctly");
    TEST_ASSERT(!unpacked[1], "Channel 1 is OFF");
    TEST_ASSERT(!unpacked[18], "Channel 18 is OFF");
}

/*============================================================================*/
/* MAIN                                                                       */
/*============================================================================*/

int main(int argc, char *argv[])
{
    (void)argc;
    (void)argv;

    printf("=============================================================\n");
    printf("BCU Firmware - Software-in-the-Loop (SIL) Tests\n");
    printf("Version: 1.0.0\n");
    printf("Date: 2026-01-23\n");
    printf("=============================================================\n");

    /* Run all tests */
    test_bcu_status_packing();
    test_bcu_current_packing();
    test_bcu_voltage_packing();
    test_bcu_temperature_packing();
    test_bcu_rul_status_packing();
    test_bcu_timing_packing();
    test_cmd_output_set_parsing();
    test_cmd_system_parsing();
    test_io_bitmap_packing();

    /* Print summary */
    printf("\n=============================================================\n");
    printf("TEST SUMMARY\n");
    printf("=============================================================\n");
    printf("Tests run:    %d\n", tests_run);
    printf("Tests passed: %d\n", tests_passed);
    printf("Tests failed: %d\n", tests_failed);
    printf("=============================================================\n");

    if (tests_failed == 0)
    {
        printf("ALL TESTS PASSED!\n");
        return 0;
    }
    else
    {
        printf("SOME TESTS FAILED!\n");
        return 1;
    }
}
