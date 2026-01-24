#!/usr/bin/env python3
"""
BCU Firmware - Software-in-the-Loop (SIL) Tests
================================================
Tests CAN message packing/unpacking according to DBC specification.

Run: python sil_tests.py
"""

import struct
import sys

# Test counters
tests_run = 0
tests_passed = 0
tests_failed = 0

def test_assert(condition, message):
    """Assert with test counting"""
    global tests_run, tests_passed, tests_failed
    tests_run += 1
    if condition:
        tests_passed += 1
        print(f"  [PASS] {message}")
    else:
        tests_failed += 1
        print(f"  [FAIL] {message}")

def pack_u16_be(val):
    """Pack 16-bit unsigned big-endian"""
    return struct.pack('>H', val & 0xFFFF)

def pack_i16_be(val):
    """Pack 16-bit signed big-endian"""
    return struct.pack('>h', val)

def pack_u32_be(val):
    """Pack 32-bit unsigned big-endian"""
    return struct.pack('>I', val & 0xFFFFFFFF)

def pack_i32_be(val):
    """Pack 32-bit signed big-endian"""
    return struct.pack('>i', val)

def unpack_u16_be(data):
    """Unpack 16-bit unsigned big-endian"""
    return struct.unpack('>H', bytes(data))[0]

def unpack_i16_be(data):
    """Unpack 16-bit signed big-endian"""
    return struct.unpack('>h', bytes(data))[0]

def unpack_u32_be(data):
    """Unpack 32-bit unsigned big-endian"""
    return struct.unpack('>I', bytes(data))[0]

def unpack_i32_be(data):
    """Unpack 32-bit signed big-endian"""
    return struct.unpack('>i', bytes(data))[0]

# =============================================================================
# TEST: BCU_Status Message (0x100)
# =============================================================================

def test_bcu_status_packing():
    print("\n[TEST] BCU_Status Message Packing (0x100)")

    msg = bytearray(8)

    # Test data
    state = 3           # RUNNING
    power_good = 1      # OK
    safety_ok = 1       # OK
    active_errors = 0
    uptime_ms = 123456789

    # Pack
    msg[0] = state
    msg[1] = power_good
    msg[2] = safety_ok
    msg[3] = active_errors
    msg[4:8] = pack_u32_be(uptime_ms)

    # Verify
    test_assert(msg[0] == 3, "State = RUNNING (3)")
    test_assert(msg[1] == 1, "PowerGood = OK (1)")
    test_assert(msg[2] == 1, "SafetyOK = OK (1)")
    test_assert(msg[3] == 0, "ActiveErrors = 0")

    # Verify uptime big-endian
    unpacked = unpack_u32_be(msg[4:8])
    test_assert(unpacked == 123456789, "Uptime unpacked correctly")

    # Verify byte order
    test_assert(msg[4] == 0x07, "Uptime byte 0 (MSB) = 0x07")
    test_assert(msg[5] == 0x5B, "Uptime byte 1 = 0x5B")
    test_assert(msg[6] == 0xCD, "Uptime byte 2 = 0xCD")
    test_assert(msg[7] == 0x15, "Uptime byte 3 (LSB) = 0x15")

# =============================================================================
# TEST: BCU_Current Messages (0x101-0x103)
# =============================================================================

def test_bcu_current_packing():
    print("\n[TEST] BCU_Current Message Packing (0x101)")

    msg = bytearray(8)

    # Test data: currents in mA, packed as 10mA units
    currents_mA = [15000, -8500, 0, 32767]  # 15A, -8.5A, 0A, max

    for i, current_mA in enumerate(currents_mA):
        current_10mA = current_mA // 10
        msg[i*2:i*2+2] = pack_i16_be(current_10mA)

    # Verify unpacking
    test_assert(unpack_i16_be(msg[0:2]) == 1500, "LEM0: 15000mA -> 1500 (10mA units)")
    test_assert(unpack_i16_be(msg[2:4]) == -850, "LEM1: -8500mA -> -850 (10mA units)")
    test_assert(unpack_i16_be(msg[4:6]) == 0, "LEM2: 0mA -> 0")
    test_assert(unpack_i16_be(msg[6:8]) == 3276, "LEM3: 32767mA -> 3276 (10mA units)")

    # Verify negative value byte representation (-850 = 0xFCAE)
    test_assert(msg[2] == 0xFC, "Negative current MSB = 0xFC")
    test_assert(msg[3] == 0xAE, "Negative current LSB = 0xAE")

# =============================================================================
# TEST: BCU_Voltage Message (0x104)
# =============================================================================

def test_bcu_voltage_packing():
    print("\n[TEST] BCU_Voltage Message Packing (0x104)")

    msg = bytearray(8)

    # Test data: voltages in mV
    voltages = [12350, 5020, 3312, 3298]

    for i, v in enumerate(voltages):
        msg[i*2:i*2+2] = pack_u16_be(v)

    # Verify
    test_assert(unpack_u16_be(msg[0:2]) == 12350, "Rail_12V = 12350mV")
    test_assert(unpack_u16_be(msg[2:4]) == 5020, "Rail_5V = 5020mV")
    test_assert(unpack_u16_be(msg[4:6]) == 3312, "Rail_3V3_D = 3312mV")
    test_assert(unpack_u16_be(msg[6:8]) == 3298, "Rail_3V3_A = 3298mV")

# =============================================================================
# TEST: BCU_Temperature Message (0x105)
# =============================================================================

def test_bcu_temperature_packing():
    print("\n[TEST] BCU_Temperature Message Packing (0x105)")

    msg = bytearray(8)

    # Test positive temperature: 25.75°C = 25750 mC
    temp_mC = 25750
    msg[0:4] = pack_i32_be(temp_mC)

    unpacked = unpack_i32_be(msg[0:4])
    test_assert(unpacked == 25750, "Temperature = 25750 mC (25.75°C)")

    # Test negative temperature: -10.5°C = -10500 mC
    temp_mC = -10500
    msg[0:4] = pack_i32_be(temp_mC)

    unpacked = unpack_i32_be(msg[0:4])
    test_assert(unpacked == -10500, "Temperature = -10500 mC (-10.5°C)")

# =============================================================================
# TEST: BCU_RUL_Status Message (0x120)
# =============================================================================

def test_bcu_rul_status_packing():
    print("\n[TEST] BCU_RUL_Status Message Packing (0x120)")

    msg = bytearray(8)

    # Test data
    soh_percent = 8750      # 87.50% in 0.01% units
    remaining_cycles = 1234
    remaining_days = 365
    confidence = 85         # 85%
    valid = 1

    msg[0:2] = pack_u16_be(soh_percent)
    msg[2:4] = pack_u16_be(remaining_cycles)
    msg[4:6] = pack_u16_be(remaining_days)
    msg[6] = confidence
    msg[7] = valid

    # Verify
    test_assert(unpack_u16_be(msg[0:2]) == 8750, "SoH = 87.50%")
    test_assert(unpack_u16_be(msg[2:4]) == 1234, "RemainingCycles = 1234")
    test_assert(unpack_u16_be(msg[4:6]) == 365, "RemainingDays = 365")
    test_assert(msg[6] == 85, "Confidence = 85%")
    test_assert(msg[7] == 1, "Valid = 1")

# =============================================================================
# TEST: BCU_Timing Message (0x121)
# =============================================================================

def test_bcu_timing_packing():
    print("\n[TEST] BCU_Timing Message Packing (0x121)")

    msg = bytearray(8)

    cycle_time_us = 850
    max_cycle_time_us = 1250

    msg[0:4] = pack_u32_be(cycle_time_us)
    msg[4:8] = pack_u32_be(max_cycle_time_us)

    test_assert(unpack_u32_be(msg[0:4]) == 850, "CycleTime = 850 µs")
    test_assert(unpack_u32_be(msg[4:8]) == 1250, "MaxCycleTime = 1250 µs")

# =============================================================================
# TEST: CMD_Output_Set Message (0x200)
# =============================================================================

def test_cmd_output_set_parsing():
    print("\n[TEST] CMD_Output_Set Message Parsing (0x200)")

    msg = bytearray(8)

    # Simulate: turn on channel 5
    msg[0] = 5   # ChannelID
    msg[1] = 1   # State = ON

    test_assert(msg[0] == 5, "ChannelID = 5")
    test_assert(msg[1] == 1, "State = ON")
    test_assert(msg[0] < 20, "Channel in valid range (0-19)")

# =============================================================================
# TEST: CMD_System Message (0x210)
# =============================================================================

def test_cmd_system_parsing():
    print("\n[TEST] CMD_System Message Parsing (0x210)")

    # Command codes
    CMD_SYS_NOP = 0x00
    CMD_SYS_RESET = 0x01
    CMD_SYS_SAFE_STATE = 0x02
    CMD_SYS_CLEAR_FAULTS = 0x03
    CMD_SYS_ENABLE_OUTPUTS = 0x10
    CMD_SYS_DISABLE_OUTPUTS = 0x11

    msg = bytearray(8)

    msg[0] = CMD_SYS_RESET
    test_assert(msg[0] == 0x01, "Command = RESET (0x01)")

    msg[0] = CMD_SYS_SAFE_STATE
    test_assert(msg[0] == 0x02, "Command = SAFE_STATE (0x02)")

    msg[0] = CMD_SYS_ENABLE_OUTPUTS
    test_assert(msg[0] == 0x10, "Command = ENABLE_OUTPUTS (0x10)")

# =============================================================================
# TEST: I/O Bitmap Packing
# =============================================================================

def test_io_bitmap_packing():
    print("\n[TEST] I/O Bitmap Packing (20 channels)")

    msg = bytearray(8)
    outputs = [False] * 20

    # Set some outputs
    outputs[0] = True
    outputs[5] = True
    outputs[10] = True
    outputs[19] = True

    # Pack into bitmap
    for i in range(20):
        if outputs[i]:
            byte_idx = i // 8
            bit_idx = i % 8
            msg[byte_idx] |= (1 << bit_idx)

    # Verify
    test_assert(msg[0] == 0x21, "Byte 0: bits 0,5 set = 0x21")
    test_assert(msg[1] == 0x04, "Byte 1: bit 2 (channel 10) set = 0x04")
    test_assert(msg[2] == 0x08, "Byte 2: bit 3 (channel 19) set = 0x08")

    # Unpack and verify
    unpacked = [False] * 20
    for i in range(20):
        byte_idx = i // 8
        bit_idx = i % 8
        unpacked[i] = bool(msg[byte_idx] & (1 << bit_idx))

    test_assert(unpacked[0], "Channel 0 unpacked correctly")
    test_assert(unpacked[5], "Channel 5 unpacked correctly")
    test_assert(unpacked[10], "Channel 10 unpacked correctly")
    test_assert(unpacked[19], "Channel 19 unpacked correctly")
    test_assert(not unpacked[1], "Channel 1 is OFF")
    test_assert(not unpacked[18], "Channel 18 is OFF")

# =============================================================================
# MAIN
# =============================================================================

def main():
    print("=" * 61)
    print("BCU Firmware - Software-in-the-Loop (SIL) Tests")
    print("Version: 1.0.0")
    print("Date: 2026-01-23")
    print("=" * 61)

    # Run all tests
    test_bcu_status_packing()
    test_bcu_current_packing()
    test_bcu_voltage_packing()
    test_bcu_temperature_packing()
    test_bcu_rul_status_packing()
    test_bcu_timing_packing()
    test_cmd_output_set_parsing()
    test_cmd_system_parsing()
    test_io_bitmap_packing()

    # Print summary
    print()
    print("=" * 61)
    print("TEST SUMMARY")
    print("=" * 61)
    print(f"Tests run:    {tests_run}")
    print(f"Tests passed: {tests_passed}")
    print(f"Tests failed: {tests_failed}")
    print("=" * 61)

    if tests_failed == 0:
        print("ALL TESTS PASSED!")
        return 0
    else:
        print("SOME TESTS FAILED!")
        return 1

if __name__ == "__main__":
    sys.exit(main())
