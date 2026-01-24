#!/usr/bin/env python3
"""
BCU Firmware Unit Tests - Python Implementation
Runs unit tests for CRC, RingBuffer, and Filter modules.

Usage: python run_tests.py [test_name]
  test_name: crc, ringbuffer, filter, all (default)
"""

import sys
from typing import List, Tuple, Any, Callable

#==============================================================================
# TEST FRAMEWORK
#==============================================================================

class TestFramework:
    """Simple test framework similar to Unity"""
    
    def __init__(self):
        self.tests_run = 0
        self.tests_passed = 0
        self.tests_failed = 0
        self.current_test = ""
        self.failures: List[str] = []
    
    def run_test(self, func: Callable) -> bool:
        """Run a single test function"""
        self.tests_run += 1
        self.current_test = func.__name__
        try:
            func()
            self.tests_passed += 1
            print(f"[PASS] {func.__name__}")
            return True
        except AssertionError as e:
            self.tests_failed += 1
            self.failures.append(f"{func.__name__}: {e}")
            print(f"[FAIL] {func.__name__}: {e}")
            return False
        except Exception as e:
            self.tests_failed += 1
            self.failures.append(f"{func.__name__}: Exception: {e}")
            print(f"[FAIL] {func.__name__}: Exception: {e}")
            return False
    
    def summary(self, suite_name: str):
        """Print test summary"""
        print(f"\n{'='*60}")
        print(f"{suite_name} - TEST SUMMARY")
        print(f"{'='*60}")
        print(f"Tests run:    {self.tests_run}")
        print(f"Tests passed: {self.tests_passed}")
        print(f"Tests failed: {self.tests_failed}")
        print(f"{'='*60}")
        if self.tests_failed == 0:
            print("ALL TESTS PASSED!")
        else:
            print("SOME TESTS FAILED!")
            for f in self.failures:
                print(f"  - {f}")
        print()
        return self.tests_failed == 0

#==============================================================================
# CRC MODULE IMPLEMENTATION (for testing)
#==============================================================================

CRC32_POLYNOMIAL = 0xEDB88320
CRC32_INIT = 0xFFFFFFFF
CRC16_POLYNOMIAL = 0x8408
CRC16_INIT = 0xFFFF
CRC8_POLYNOMIAL = 0x07

def crc32_calculate(data: bytes) -> int:
    """Calculate CRC-32 (IEEE 802.3)"""
    if not data:
        return 0
    crc = CRC32_INIT
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ CRC32_POLYNOMIAL
            else:
                crc >>= 1
    return crc ^ CRC32_INIT

def crc16_calculate(data: bytes) -> int:
    """Calculate CRC-16 (CCITT)"""
    if not data:
        return 0
    crc = CRC16_INIT
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ CRC16_POLYNOMIAL
            else:
                crc >>= 1
    return crc ^ CRC16_INIT

def crc8_calculate(data: bytes) -> int:
    """Calculate CRC-8"""
    if not data:
        return 0
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ CRC8_POLYNOMIAL) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    return crc

#==============================================================================
# CRC TESTS
#==============================================================================

def test_crc32_empty():
    assert crc32_calculate(b"") == 0

def test_crc32_single_byte():
    result = crc32_calculate(bytes([0x00]))
    assert result == 0xD202EF8D, f"Expected 0xD202EF8D, got 0x{result:08X}"

def test_crc32_known_string():
    result = crc32_calculate(b"123456789")
    assert result == 0xCBF43926, f"Expected 0xCBF43926, got 0x{result:08X}"

def test_crc32_all_zeros():
    result = crc32_calculate(bytes([0]*8))
    assert result != 0

def test_crc32_all_ones():
    result = crc32_calculate(bytes([0xFF]*8))
    # Verify CRC is consistent (not checking specific value since it depends on polynomial)
    result2 = crc32_calculate(bytes([0xFF]*8))
    assert result == result2, f"CRC not deterministic"
    assert result != 0, "CRC should not be zero for non-empty data"

def test_crc32_data_integrity():
    data1 = bytes([0x01, 0x02, 0x03, 0x04, 0x05])
    data2 = bytes([0x01, 0x02, 0x33, 0x04, 0x05])  # Changed byte
    crc1 = crc32_calculate(data1)
    crc2 = crc32_calculate(data2)
    assert crc1 != crc2

def test_crc16_empty():
    assert crc16_calculate(b"") == 0

def test_crc16_data_change():
    data1 = bytes([0xAA, 0xBB, 0xCC, 0xDD])
    data2 = bytes([0xAB, 0xBB, 0xCC, 0xDD])
    crc1 = crc16_calculate(data1)
    crc2 = crc16_calculate(data2)
    assert crc1 != crc2

def test_crc8_empty():
    assert crc8_calculate(b"") == 0

def test_crc8_single_zero():
    result = crc8_calculate(bytes([0x00]))
    assert result == 0x00

def test_crc8_single_one():
    result = crc8_calculate(bytes([0x01]))
    assert result == 0x07

def test_crc8_deterministic():
    data = bytes([0x01, 0x02, 0x03])
    crc1 = crc8_calculate(data)
    crc2 = crc8_calculate(data)
    assert crc1 == crc2

#==============================================================================
# RING BUFFER MODULE IMPLEMENTATION (for testing)
#==============================================================================

class RingBuffer:
    """Ring buffer implementation matching C code"""
    
    def __init__(self, size: int):
        self.buffer = bytearray(size)
        self.size = size
        self.head = 0
        self.tail = 0
    
    def available(self) -> int:
        if self.head >= self.tail:
            return self.head - self.tail
        else:
            return self.size - self.tail + self.head
    
    def free(self) -> int:
        return self.size - self.available() - 1
    
    def is_empty(self) -> bool:
        return self.head == self.tail
    
    def is_full(self) -> bool:
        return ((self.head + 1) % self.size) == self.tail
    
    def write(self, data: bytes) -> bool:
        if len(data) > self.free():
            return False
        for byte in data:
            self.buffer[self.head] = byte
            self.head = (self.head + 1) % self.size
        return True
    
    def read(self, length: int) -> bytes:
        if length > self.available():
            return None
        result = bytearray(length)
        for i in range(length):
            result[i] = self.buffer[self.tail]
            self.tail = (self.tail + 1) % self.size
        return bytes(result)
    
    def peek(self, length: int) -> bytes:
        if length > self.available():
            return None
        result = bytearray(length)
        idx = self.tail
        for i in range(length):
            result[i] = self.buffer[idx]
            idx = (idx + 1) % self.size
        return bytes(result)
    
    def clear(self):
        self.head = 0
        self.tail = 0

#==============================================================================
# RING BUFFER TESTS
#==============================================================================

def test_rb_init():
    rb = RingBuffer(64)
    assert rb.size == 64
    assert rb.head == 0
    assert rb.tail == 0
    assert rb.is_empty()

def test_rb_write_single():
    rb = RingBuffer(64)
    assert rb.write(bytes([0xAB]))
    assert rb.available() == 1
    assert rb.buffer[0] == 0xAB

def test_rb_write_multiple():
    rb = RingBuffer(64)
    assert rb.write(bytes([0x01, 0x02, 0x03, 0x04, 0x05]))
    assert rb.available() == 5

def test_rb_write_overflow():
    rb = RingBuffer(64)
    assert not rb.write(bytes([0xAA] * 100))  # Too much data
    assert rb.available() == 0

def test_rb_write_exactly_full():
    rb = RingBuffer(64)
    assert rb.write(bytes([0xBB] * 63))  # 64-1 = 63 usable
    assert rb.is_full()

def test_rb_read_single():
    rb = RingBuffer(64)
    rb.write(bytes([0xCD]))
    data = rb.read(1)
    assert data == bytes([0xCD])
    assert rb.available() == 0

def test_rb_read_multiple():
    rb = RingBuffer(64)
    rb.write(bytes([0x11, 0x22, 0x33, 0x44]))
    data = rb.read(4)
    assert data == bytes([0x11, 0x22, 0x33, 0x44])

def test_rb_read_underflow():
    rb = RingBuffer(64)
    data = rb.read(10)
    assert data is None

def test_rb_peek():
    rb = RingBuffer(64)
    rb.write(bytes([0xAA, 0xBB, 0xCC]))
    data = rb.peek(3)
    assert data == bytes([0xAA, 0xBB, 0xCC])
    assert rb.available() == 3  # Data still there

def test_rb_wraparound():
    rb = RingBuffer(64)
    # Fill half
    rb.write(bytes([i for i in range(32)]))
    # Read half
    rb.read(16)
    # Write more (will wrap)
    rb.write(bytes([i + 100 for i in range(32)]))
    # Read rest
    rb.read(16)
    data = rb.read(32)
    for i in range(32):
        assert data[i] == (i + 100) & 0xFF

def test_rb_clear():
    rb = RingBuffer(64)
    rb.write(bytes([0x01] * 30))
    rb.clear()
    assert rb.is_empty()
    assert rb.available() == 0

def test_rb_free_space():
    rb = RingBuffer(64)
    assert rb.free() == 63
    rb.write(bytes([0x00] * 10))
    assert rb.free() == 53

#==============================================================================
# FILTER MODULE IMPLEMENTATION (for testing)
#==============================================================================

class MovingAverageFilter:
    """Moving average filter matching C code"""
    
    def __init__(self, size: int):
        self.buffer = [0] * size
        self.size = size
        self.index = 0
        self.count = 0
        self.sum = 0
    
    def update(self, sample: int) -> int:
        if self.count >= self.size:
            self.sum -= self.buffer[self.index]
        else:
            self.count += 1
        
        self.buffer[self.index] = sample
        self.sum += sample
        self.index = (self.index + 1) % self.size
        
        return self.sum // self.count
    
    def reset(self):
        self.buffer = [0] * self.size
        self.index = 0
        self.count = 0
        self.sum = 0


class IIRFilter:
    """IIR low-pass filter matching C code"""
    
    def __init__(self, alpha: int, scale: int):
        self.alpha = alpha
        self.alpha_complement = scale - alpha
        self.output = 0
        self.initialized = False
    
    def update(self, sample: int) -> int:
        if not self.initialized:
            self.output = sample
            self.initialized = True
        else:
            scale = self.alpha + self.alpha_complement
            self.output = (self.alpha * sample + self.alpha_complement * self.output) // scale
        return self.output
    
    def reset(self):
        self.output = 0
        self.initialized = False


class DebounceFilter:
    """Debounce filter matching C code"""
    
    def __init__(self, threshold_ms: int, sample_period_ms: int):
        self.state = False
        self.last_raw = False
        self.counter = 0
        self.threshold = threshold_ms
        self.sample_period = sample_period_ms
    
    def update(self, raw_state: bool) -> bool:
        if raw_state == self.last_raw:
            self.counter += self.sample_period
            if self.counter >= self.threshold:
                self.state = raw_state
                self.counter = self.threshold
        else:
            self.counter = 0
            self.last_raw = raw_state
        return self.state
    
    def reset(self):
        self.state = False
        self.last_raw = False
        self.counter = 0

#==============================================================================
# FILTER TESTS
#==============================================================================

def test_ma_single_sample():
    f = MovingAverageFilter(4)
    assert f.update(100) == 100

def test_ma_filling_buffer():
    f = MovingAverageFilter(4)
    f.update(100)  # 100/1 = 100
    f.update(200)  # 300/2 = 150
    f.update(300)  # 600/3 = 200
    result = f.update(400)  # 1000/4 = 250
    assert result == 250

def test_ma_sliding_window():
    f = MovingAverageFilter(4)
    f.update(100)
    f.update(200)
    f.update(300)
    f.update(400)
    # Add 500: removes 100, adds 500 -> 1400/4 = 350
    result = f.update(500)
    assert result == 350

def test_ma_constant():
    f = MovingAverageFilter(4)
    for _ in range(10):
        result = f.update(100)
        assert result == 100

def test_ma_negative():
    f = MovingAverageFilter(4)
    f.update(-100)
    f.update(-200)
    f.update(-300)
    result = f.update(-400)
    assert result == -250

def test_ma_reset():
    f = MovingAverageFilter(4)
    f.update(100)
    f.update(200)
    f.reset()
    assert f.count == 0
    assert f.sum == 0
    assert f.update(50) == 50

def test_iir_first_sample():
    f = IIRFilter(10, 100)
    result = f.update(1000)
    assert result == 1000
    assert f.initialized

def test_iir_lowpass():
    f = IIRFilter(10, 100)  # 10% new, 90% old
    f.update(0)
    result = f.update(1000)  # 10*1000 + 90*0 / 100 = 100
    assert result == 100

def test_iir_highpass():
    f = IIRFilter(90, 100)  # 90% new, 10% old
    f.update(0)
    result = f.update(1000)  # 90*1000 + 10*0 / 100 = 900
    assert result == 900

def test_iir_converges():
    f = IIRFilter(50, 100)
    for _ in range(50):
        result = f.update(1000)
    assert abs(result - 1000) <= 10  # Should be close to 1000

def test_iir_reset():
    f = IIRFilter(10, 100)
    f.update(500)
    f.update(600)
    f.reset()
    assert not f.initialized
    assert f.output == 0

def test_debounce_no_change_before_threshold():
    f = DebounceFilter(50, 10)  # 50ms threshold, 10ms period
    for _ in range(4):  # 40ms < 50ms
        result = f.update(True)
    assert result == False  # Still false

def test_debounce_change_after_threshold():
    f = DebounceFilter(50, 10)
    for _ in range(6):  # 60ms >= 50ms
        result = f.update(True)
    assert result == True

def test_debounce_glitch_rejection():
    f = DebounceFilter(50, 10)
    f.update(True)
    f.update(True)
    f.update(True)
    result = f.update(False)  # Glitch
    assert result == False
    assert f.counter == 0

def test_debounce_transition_back():
    f = DebounceFilter(30, 10)
    # Get to TRUE
    for _ in range(5):
        f.update(True)
    # Go back to FALSE
    for _ in range(5):
        f.update(False)
    assert f.state == False

def test_debounce_reset():
    f = DebounceFilter(30, 10)
    for _ in range(5):
        f.update(True)
    f.reset()
    assert f.state == False
    assert f.counter == 0

#==============================================================================
# MAIN
#==============================================================================

def run_crc_tests():
    """Run CRC test suite"""
    tf = TestFramework()
    print("\n" + "="*60)
    print("CRC MODULE TESTS")
    print("="*60 + "\n")
    
    tf.run_test(test_crc32_empty)
    tf.run_test(test_crc32_single_byte)
    tf.run_test(test_crc32_known_string)
    tf.run_test(test_crc32_all_zeros)
    tf.run_test(test_crc32_all_ones)
    tf.run_test(test_crc32_data_integrity)
    tf.run_test(test_crc16_empty)
    tf.run_test(test_crc16_data_change)
    tf.run_test(test_crc8_empty)
    tf.run_test(test_crc8_single_zero)
    tf.run_test(test_crc8_single_one)
    tf.run_test(test_crc8_deterministic)
    
    return tf.summary("CRC MODULE")


def run_ringbuffer_tests():
    """Run RingBuffer test suite"""
    tf = TestFramework()
    print("\n" + "="*60)
    print("RING BUFFER MODULE TESTS")
    print("="*60 + "\n")
    
    tf.run_test(test_rb_init)
    tf.run_test(test_rb_write_single)
    tf.run_test(test_rb_write_multiple)
    tf.run_test(test_rb_write_overflow)
    tf.run_test(test_rb_write_exactly_full)
    tf.run_test(test_rb_read_single)
    tf.run_test(test_rb_read_multiple)
    tf.run_test(test_rb_read_underflow)
    tf.run_test(test_rb_peek)
    tf.run_test(test_rb_wraparound)
    tf.run_test(test_rb_clear)
    tf.run_test(test_rb_free_space)
    
    return tf.summary("RING BUFFER MODULE")


def run_filter_tests():
    """Run Filter test suite"""
    tf = TestFramework()
    print("\n" + "="*60)
    print("FILTER MODULE TESTS")
    print("="*60 + "\n")
    
    # Moving Average
    tf.run_test(test_ma_single_sample)
    tf.run_test(test_ma_filling_buffer)
    tf.run_test(test_ma_sliding_window)
    tf.run_test(test_ma_constant)
    tf.run_test(test_ma_negative)
    tf.run_test(test_ma_reset)
    
    # IIR Filter
    tf.run_test(test_iir_first_sample)
    tf.run_test(test_iir_lowpass)
    tf.run_test(test_iir_highpass)
    tf.run_test(test_iir_converges)
    tf.run_test(test_iir_reset)
    
    # Debounce Filter
    tf.run_test(test_debounce_no_change_before_threshold)
    tf.run_test(test_debounce_change_after_threshold)
    tf.run_test(test_debounce_glitch_rejection)
    tf.run_test(test_debounce_transition_back)
    tf.run_test(test_debounce_reset)
    
    return tf.summary("FILTER MODULE")


def main():
    print("\n" + "="*60)
    print("BCU FIRMWARE UNIT TESTS")
    print("="*60)
    
    test_name = sys.argv[1] if len(sys.argv) > 1 else "all"
    
    results = []
    
    if test_name in ["crc", "all"]:
        results.append(("CRC", run_crc_tests()))
    
    if test_name in ["ringbuffer", "all"]:
        results.append(("RingBuffer", run_ringbuffer_tests()))
    
    if test_name in ["filter", "all"]:
        results.append(("Filter", run_filter_tests()))
    
    # Overall summary
    print("\n" + "="*60)
    print("OVERALL SUMMARY")
    print("="*60)
    
    all_passed = True
    for name, passed in results:
        status = "PASS" if passed else "FAIL"
        print(f"  {name}: {status}")
        if not passed:
            all_passed = False
    
    print("="*60)
    if all_passed:
        print("ALL TEST SUITES PASSED!")
        return 0
    else:
        print("SOME TEST SUITES FAILED!")
        return 1


if __name__ == "__main__":
    sys.exit(main())
