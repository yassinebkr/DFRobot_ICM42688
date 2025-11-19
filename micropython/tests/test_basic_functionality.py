# SPDX-FileCopyrightText: 2025 Yassine Bekkari
# SPDX-License-Identifier: MIT

"""
Basic functionality tests for ICM42688 MicroPython driver.

This test script validates core functionality of the ICM42688 driver:
- Sensor initialization
- WHO_AM_I verification
- Basic sensor readings (accel, gyro, temp)
- Range configuration
- Data rate configuration
- Power mode switching

Run this on hardware to verify the port is working correctly.
"""

import time
from machine import I2C, Pin
import sys
import gc

# Add icm42688 module to path
sys.path.append('/home/user/DFRobot_ICM42688/micropython')

import icm42688
from icm42688 import registers as reg

# ========================================================================
# Test Configuration
# ========================================================================

I2C_SDA_PIN = 21
I2C_SCL_PIN = 22
I2C_FREQ = 400000
ICM_ADDRESS = 0x69

# Test results tracking
tests_passed = 0
tests_failed = 0
test_results = []

def test_passed(test_name, details=""):
    """Record a passed test."""
    global tests_passed
    tests_passed += 1
    result = f"✓ PASS: {test_name}"
    if details:
        result += f" - {details}"
    print(result)
    test_results.append(result)

def test_failed(test_name, error):
    """Record a failed test."""
    global tests_failed
    tests_failed += 1
    result = f"✗ FAIL: {test_name} - {error}"
    print(result)
    test_results.append(result)

def assert_equal(actual, expected, test_name):
    """Assert that two values are equal."""
    if actual == expected:
        test_passed(test_name, f"{actual} == {expected}")
        return True
    else:
        test_failed(test_name, f"{actual} != {expected}")
        return False

def assert_in_range(value, min_val, max_val, test_name):
    """Assert that a value is within a range."""
    if min_val <= value <= max_val:
        test_passed(test_name, f"{min_val} <= {value} <= {max_val}")
        return True
    else:
        test_failed(test_name, f"{value} not in range [{min_val}, {max_val}]")
        return False

def assert_not_none(value, test_name):
    """Assert that a value is not None."""
    if value is not None:
        test_passed(test_name, f"Value exists: {value}")
        return True
    else:
        test_failed(test_name, "Value is None")
        return False

# ========================================================================
# Test Suite
# ========================================================================

print("=" * 80)
print("ICM42688 MicroPython Driver - Basic Functionality Tests")
print("=" * 80)
print(f"Platform: {sys.platform}")
print(f"MicroPython version: {sys.version}")
print(f"Free memory: {gc.mem_free()} bytes")
print()

# ------------------------------------------------------------------------
# Test 1: I2C Initialization
# ------------------------------------------------------------------------

print("\n[Test 1] I2C Initialization")
print("-" * 80)

try:
    i2c = I2C(0, scl=Pin(I2C_SCL_PIN), sda=Pin(I2C_SDA_PIN), freq=I2C_FREQ)
    test_passed("I2C bus initialized")

    # Scan for devices
    devices = i2c.scan()
    print(f"I2C devices found: {[hex(d) for d in devices]}")

    if ICM_ADDRESS in devices:
        test_passed("ICM42688 found on I2C bus", f"Address: {hex(ICM_ADDRESS)}")
    else:
        test_failed("ICM42688 not found", f"Expected address {hex(ICM_ADDRESS)}")

except Exception as e:
    test_failed("I2C initialization", str(e))
    sys.exit(1)

# ------------------------------------------------------------------------
# Test 2: Sensor Initialization
# ------------------------------------------------------------------------

print("\n[Test 2] Sensor Initialization")
print("-" * 80)

try:
    icm = icm42688.ICM42688(i2c, address=ICM_ADDRESS)
    test_passed("ICM42688 object created")

    # Sensor should be initialized with WHO_AM_I check in __init__
    test_passed("Sensor initialized (WHO_AM_I verified)")

except Exception as e:
    test_failed("Sensor initialization", str(e))
    sys.exit(1)

# ------------------------------------------------------------------------
# Test 3: Temperature Reading
# ------------------------------------------------------------------------

print("\n[Test 3] Temperature Reading")
print("-" * 80)

try:
    temp = icm.temperature
    assert_not_none(temp, "Temperature reading")

    # Temperature should be reasonable (0-50°C for indoor testing)
    assert_in_range(temp, 0, 50, "Temperature in valid range")

    print(f"  Temperature: {temp:.2f} °C")

except Exception as e:
    test_failed("Temperature reading", str(e))

# ------------------------------------------------------------------------
# Test 4: Accelerometer Reading
# ------------------------------------------------------------------------

print("\n[Test 4] Accelerometer Reading")
print("-" * 80)

try:
    accel_x, accel_y, accel_z = icm.acceleration
    assert_not_none(accel_x, "Accel X reading")
    assert_not_none(accel_y, "Accel Y reading")
    assert_not_none(accel_z, "Accel Z reading")

    print(f"  Acceleration: X={accel_x:.2f} Y={accel_y:.2f} Z={accel_z:.2f} m/s²")

    # Check that magnitude is close to 1g (sensor at rest)
    # Allow 0.5g to 1.5g range for different orientations
    accel_mag = (accel_x**2 + accel_y**2 + accel_z**2)**0.5
    assert_in_range(accel_mag, 0.5 * 9.81, 1.5 * 9.81, "Accel magnitude near 1g")

    print(f"  Magnitude: {accel_mag:.2f} m/s² ({accel_mag/9.81:.2f} g)")

except Exception as e:
    test_failed("Accelerometer reading", str(e))

# ------------------------------------------------------------------------
# Test 5: Gyroscope Reading
# ------------------------------------------------------------------------

print("\n[Test 5] Gyroscope Reading")
print("-" * 80)

try:
    gyro_x, gyro_y, gyro_z = icm.gyro
    assert_not_none(gyro_x, "Gyro X reading")
    assert_not_none(gyro_y, "Gyro Y reading")
    assert_not_none(gyro_z, "Gyro Z reading")

    print(f"  Gyroscope: X={gyro_x:.3f} Y={gyro_y:.3f} Z={gyro_z:.3f} rad/s")

    # Gyro should be near zero when stationary (allow ±0.2 rad/s for noise/offset)
    assert_in_range(abs(gyro_x), 0, 0.2, "Gyro X near zero (stationary)")
    assert_in_range(abs(gyro_y), 0, 0.2, "Gyro Y near zero (stationary)")
    assert_in_range(abs(gyro_z), 0, 0.2, "Gyro Z near zero (stationary)")

    # Convert to deg/s for display
    print(f"  Gyroscope: X={gyro_x*57.3:.2f} Y={gyro_y*57.3:.2f} Z={gyro_z*57.3:.2f} °/s")

except Exception as e:
    test_failed("Gyroscope reading", str(e))

# ------------------------------------------------------------------------
# Test 6: Accelerometer Range Configuration
# ------------------------------------------------------------------------

print("\n[Test 6] Accelerometer Range Configuration")
print("-" * 80)

try:
    # Test each range setting
    ranges = [
        (reg.ACCEL_RANGE_2G, "±2g"),
        (reg.ACCEL_RANGE_4G, "±4g"),
        (reg.ACCEL_RANGE_8G, "±8g"),
        (reg.ACCEL_RANGE_16G, "±16g"),
    ]

    for range_val, range_name in ranges:
        icm.accelerometer_range = range_val
        time.sleep(0.01)  # Allow settling
        readback = icm.accelerometer_range
        assert_equal(readback, range_val, f"Accel range set to {range_name}")

    # Restore default
    icm.accelerometer_range = reg.ACCEL_RANGE_16G

except Exception as e:
    test_failed("Accelerometer range configuration", str(e))

# ------------------------------------------------------------------------
# Test 7: Gyroscope Range Configuration
# ------------------------------------------------------------------------

print("\n[Test 7] Gyroscope Range Configuration")
print("-" * 80)

try:
    # Test a few range settings
    ranges = [
        (reg.GYRO_RANGE_2000_DPS, "±2000 dps"),
        (reg.GYRO_RANGE_1000_DPS, "±1000 dps"),
        (reg.GYRO_RANGE_500_DPS, "±500 dps"),
    ]

    for range_val, range_name in ranges:
        icm.gyro_range = range_val
        time.sleep(0.01)
        readback = icm.gyro_range
        assert_equal(readback, range_val, f"Gyro range set to {range_name}")

    # Restore default
    icm.gyro_range = reg.GYRO_RANGE_2000_DPS

except Exception as e:
    test_failed("Gyroscope range configuration", str(e))

# ------------------------------------------------------------------------
# Test 8: Data Rate Configuration
# ------------------------------------------------------------------------

print("\n[Test 8] Output Data Rate Configuration")
print("-" * 80)

try:
    # Test ODR settings
    odrs = [
        (reg.ODR_1KHZ, "1000 Hz"),
        (reg.ODR_200HZ, "200 Hz"),
        (reg.ODR_100HZ, "100 Hz"),
    ]

    for odr_val, odr_name in odrs:
        icm.accelerometer_data_rate = odr_val
        time.sleep(0.01)
        readback = icm.accelerometer_data_rate
        assert_equal(readback, odr_val, f"Accel ODR set to {odr_name}")

        icm.gyro_data_rate = odr_val
        time.sleep(0.01)
        readback = icm.gyro_data_rate
        assert_equal(readback, odr_val, f"Gyro ODR set to {odr_name}")

    # Restore default
    icm.accelerometer_data_rate = reg.ODR_1KHZ
    icm.gyro_data_rate = reg.ODR_1KHZ

except Exception as e:
    test_failed("Data rate configuration", str(e))

# ------------------------------------------------------------------------
# Test 9: Power Mode Switching
# ------------------------------------------------------------------------

print("\n[Test 9] Power Mode Switching")
print("-" * 80)

try:
    # Test power mode changes
    icm.set_power_mode(accel_mode=reg.ACCEL_MODE_LP, gyro_mode=reg.GYRO_MODE_OFF)
    time.sleep(0.05)
    test_passed("Power mode set to LP accel, gyro off")

    # Read data in LP mode
    accel = icm.acceleration
    assert_not_none(accel[0], "Accel reading in LP mode")

    # Restore high performance mode
    icm.set_power_mode(accel_mode=reg.ACCEL_MODE_LN, gyro_mode=reg.GYRO_MODE_LN)
    time.sleep(0.05)
    test_passed("Power mode restored to LN")

except Exception as e:
    test_failed("Power mode switching", str(e))

# ------------------------------------------------------------------------
# Test 10: Soft Reset
# ------------------------------------------------------------------------

print("\n[Test 10] Soft Reset")
print("-" * 80)

try:
    # Perform soft reset
    icm.reset()
    test_passed("Soft reset executed")

    # Verify sensor still works after reset
    temp = icm.temperature
    assert_not_none(temp, "Temperature reading after reset")

    accel = icm.acceleration
    assert_not_none(accel[0], "Accel reading after reset")

except Exception as e:
    test_failed("Soft reset", str(e))

# ------------------------------------------------------------------------
# Test 11: Multiple Rapid Readings (Performance)
# ------------------------------------------------------------------------

print("\n[Test 11] Multiple Rapid Readings (Performance Test)")
print("-" * 80)

try:
    start_time = time.ticks_ms()
    num_samples = 100

    for i in range(num_samples):
        accel = icm.acceleration
        gyro = icm.gyro
        temp = icm.temperature

    end_time = time.ticks_ms()
    elapsed = time.ticks_diff(end_time, start_time) / 1000  # Convert to seconds

    sample_rate = num_samples / elapsed
    test_passed("Multiple rapid readings", f"{num_samples} samples in {elapsed:.2f}s")
    print(f"  Sample rate: {sample_rate:.1f} Hz")

    if sample_rate >= 50:
        test_passed("Performance acceptable", f"Rate: {sample_rate:.1f} Hz >= 50 Hz")
    else:
        test_failed("Performance", f"Rate: {sample_rate:.1f} Hz < 50 Hz")

except Exception as e:
    test_failed("Multiple rapid readings", str(e))

# ------------------------------------------------------------------------
# Test 12: Memory Leak Check
# ------------------------------------------------------------------------

print("\n[Test 12] Memory Leak Check")
print("-" * 80)

try:
    gc.collect()
    mem_before = gc.mem_free()

    # Perform 100 read cycles
    for i in range(100):
        accel = icm.acceleration
        gyro = icm.gyro
        temp = icm.temperature

    gc.collect()
    mem_after = gc.mem_free()

    mem_diff = mem_after - mem_before

    print(f"  Memory before: {mem_before} bytes")
    print(f"  Memory after:  {mem_after} bytes")
    print(f"  Difference:    {mem_diff:+d} bytes")

    # Allow small variations, but no significant leaks
    if abs(mem_diff) < 1000:
        test_passed("No memory leak detected", f"Diff: {mem_diff:+d} bytes")
    else:
        test_failed("Possible memory leak", f"Diff: {mem_diff:+d} bytes")

except Exception as e:
    test_failed("Memory leak check", str(e))

# ========================================================================
# Test Summary
# ========================================================================

print("\n" + "=" * 80)
print("TEST SUMMARY")
print("=" * 80)
print(f"Tests passed: {tests_passed}")
print(f"Tests failed: {tests_failed}")
print(f"Total tests:  {tests_passed + tests_failed}")
print(f"Success rate: {100 * tests_passed / max(tests_passed + tests_failed, 1):.1f}%")
print()

if tests_failed == 0:
    print("✓ ALL TESTS PASSED!")
else:
    print(f"✗ {tests_failed} TEST(S) FAILED")
    print("\nFailed tests:")
    for result in test_results:
        if result.startswith("✗"):
            print(f"  {result}")

print("\n" + "=" * 80)
print(f"Final free memory: {gc.mem_free()} bytes")
print("=" * 80)
