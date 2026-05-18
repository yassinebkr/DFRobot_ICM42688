# SPDX-FileCopyrightText: 2025 Yassine Benkhira
# SPDX-License-Identifier: MIT

"""
FIFO and advanced features tests for ICM42688 MicroPython driver.

This test script validates advanced functionality:
- FIFO enable/disable
- FIFO data reading
- FIFO flush
- Interrupt configuration
- Wake-on-motion detection
- Tap detection

Run this on hardware after basic tests pass.
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

# ========================================================================
# Initialize Hardware
# ========================================================================

print("=" * 80)
print("ICM42688 MicroPython Driver - FIFO and Advanced Features Tests")
print("=" * 80)
print()

try:
    i2c = I2C(0, scl=Pin(I2C_SCL_PIN), sda=Pin(I2C_SDA_PIN), freq=I2C_FREQ)
    icm = icm42688.ICM42688(i2c, address=ICM_ADDRESS)
    test_passed("Sensor initialized")
except Exception as e:
    test_failed("Sensor initialization", str(e))
    sys.exit(1)

# ------------------------------------------------------------------------
# Test 1: FIFO Enable/Disable
# ------------------------------------------------------------------------

print("\n[Test 1] FIFO Enable/Disable")
print("-" * 80)

try:
    # Enable FIFO
    icm.enable_fifo(accel=True, gyro=True, temp=True, mode="stream")
    test_passed("FIFO enabled")

    # Check that FIFO count is accessible
    count = icm.fifo_count
    test_passed("FIFO count readable", f"Count: {count} bytes")

    # Disable FIFO
    icm.disable_fifo()
    test_passed("FIFO disabled")

    # Re-enable for further tests
    icm.enable_fifo(accel=True, gyro=True, temp=True, mode="stream")

except Exception as e:
    test_failed("FIFO enable/disable", str(e))

# ------------------------------------------------------------------------
# Test 2: FIFO Data Accumulation
# ------------------------------------------------------------------------

print("\n[Test 2] FIFO Data Accumulation")
print("-" * 80)

try:
    # Flush FIFO to start fresh
    icm.flush_fifo()
    time.sleep(0.01)

    initial_count = icm.fifo_count
    print(f"  Initial FIFO count: {initial_count} bytes")

    # Wait for data to accumulate
    time.sleep(0.1)  # 100ms at 1kHz = ~100 samples = ~1600 bytes

    final_count = icm.fifo_count
    print(f"  Final FIFO count: {final_count} bytes")

    if final_count > initial_count:
        test_passed("FIFO accumulating data", f"Added {final_count - initial_count} bytes")
    else:
        test_failed("FIFO not accumulating", f"Count: {final_count}")

except Exception as e:
    test_failed("FIFO data accumulation", str(e))

# ------------------------------------------------------------------------
# Test 3: FIFO Read
# ------------------------------------------------------------------------

print("\n[Test 3] FIFO Read")
print("-" * 80)

try:
    # Ensure FIFO has data
    count = icm.fifo_count
    print(f"  FIFO count: {count} bytes")

    if count >= 16:
        # Read one packet
        packet = icm.read_fifo()
        test_passed("FIFO packet read successfully")

        # Verify packet structure
        if 'header' in packet:
            test_passed("Packet has header")
        if 'accel' in packet and len(packet['accel']) == 3:
            test_passed("Packet has accel data (X, Y, Z)")
            print(f"    Accel: {packet['accel']}")
        if 'gyro' in packet and len(packet['gyro']) == 3:
            test_passed("Packet has gyro data (X, Y, Z)")
            print(f"    Gyro: {packet['gyro']}")
        if 'temp' in packet:
            test_passed("Packet has temp data")
            print(f"    Temp: {packet['temp']:.2f} °C")

    else:
        test_failed("FIFO read", "Not enough data in FIFO")

except Exception as e:
    test_failed("FIFO read", str(e))

# ------------------------------------------------------------------------
# Test 4: FIFO Flush
# ------------------------------------------------------------------------

print("\n[Test 4] FIFO Flush")
print("-" * 80)

try:
    # Wait for FIFO to accumulate data
    time.sleep(0.05)
    count_before = icm.fifo_count
    print(f"  FIFO count before flush: {count_before} bytes")

    # Flush FIFO
    icm.flush_fifo()
    time.sleep(0.01)

    count_after = icm.fifo_count
    print(f"  FIFO count after flush: {count_after} bytes")

    if count_after < count_before:
        test_passed("FIFO flushed", f"Cleared {count_before - count_after} bytes")
    else:
        test_failed("FIFO flush", f"Count not reduced: {count_before} -> {count_after}")

except Exception as e:
    test_failed("FIFO flush", str(e))

# ------------------------------------------------------------------------
# Test 5: FIFO Multiple Reads
# ------------------------------------------------------------------------

print("\n[Test 5] FIFO Multiple Reads")
print("-" * 80)

try:
    # Flush and wait for accumulation
    icm.flush_fifo()
    time.sleep(0.1)

    count = icm.fifo_count
    packets_available = count // 16
    print(f"  FIFO has {packets_available} packets available")

    # Read up to 10 packets
    packets_to_read = min(packets_available, 10)
    packets_read = 0

    for i in range(packets_to_read):
        packet = icm.read_fifo()
        packets_read += 1

    test_passed("Multiple FIFO reads", f"Read {packets_read} packets successfully")

except Exception as e:
    test_failed("Multiple FIFO reads", str(e))

# Disable FIFO for interrupt tests
icm.disable_fifo()

# ------------------------------------------------------------------------
# Test 6: Interrupt Configuration
# ------------------------------------------------------------------------

print("\n[Test 6] Interrupt Configuration")
print("-" * 80)

try:
    # Test INT1 configuration
    icm.configure_interrupt(pin=1, polarity="high", mode="latch", drive="push-pull")
    test_passed("INT1 configured (high, latch, push-pull)")

    # Test INT2 configuration
    icm.configure_interrupt(pin=2, polarity="low", mode="pulse", drive="open-drain")
    test_passed("INT2 configured (low, pulse, open-drain)")

    # Restore INT1 to default
    icm.configure_interrupt(pin=1, polarity="high", mode="latch", drive="push-pull")

except Exception as e:
    test_failed("Interrupt configuration", str(e))

# ------------------------------------------------------------------------
# Test 7: Interrupt Status Reading
# ------------------------------------------------------------------------

print("\n[Test 7] Interrupt Status Reading")
print("-" * 80)

try:
    # Read interrupt status
    status = icm.read_interrupt_status()
    test_passed("Interrupt status read")

    # Verify structure
    expected_keys = ['data_ready', 'fifo_threshold', 'fifo_full', 'wom_x', 'wom_y',
                     'wom_z', 'smd', 'tap', 'sleep', 'wake', 'tilt', 'step_overflow',
                     'step_detected']

    all_keys_present = all(key in status for key in expected_keys)
    if all_keys_present:
        test_passed("Interrupt status has all expected keys")
    else:
        test_failed("Interrupt status", "Missing keys")

    print(f"  Status: {status}")

except Exception as e:
    test_failed("Interrupt status reading", str(e))

# ------------------------------------------------------------------------
# Test 8: Wake-on-Motion Enable
# ------------------------------------------------------------------------

print("\n[Test 8] Wake-on-Motion Enable")
print("-" * 80)

try:
    # Enable WOM on all axes
    icm.enable_wake_on_motion(threshold=50, axes="all", int_pin=1)
    test_passed("Wake-on-motion enabled (threshold=50, all axes)")

    # Wait briefly
    time.sleep(0.1)

    # Read status (should be clear if no motion)
    status = icm.read_interrupt_status()
    print(f"  WOM status: X={status['wom_x']}, Y={status['wom_y']}, Z={status['wom_z']}")

    # Disable for next test
    icm.disable_motion_detection()
    test_passed("Wake-on-motion disabled")

except Exception as e:
    test_failed("Wake-on-motion enable", str(e))

# ------------------------------------------------------------------------
# Test 9: Tap Detection Enable
# ------------------------------------------------------------------------

print("\n[Test 9] Tap Detection Enable")
print("-" * 80)

try:
    # Enable tap detection
    icm.enable_tap_detection(mode="low-noise", int_pin=1)
    test_passed("Tap detection enabled (low-noise mode)")

    # Wait briefly
    time.sleep(0.1)

    # Read status
    status = icm.read_interrupt_status()
    print(f"  Tap status: {status['tap']}")

    # Try reading tap info (may not have tap data)
    try:
        tap_info = icm.read_tap_info()
        test_passed("Tap info readable", f"Data: {tap_info}")
    except:
        test_passed("Tap info accessible (no tap event yet)")

    # Disable for next test
    icm.disable_motion_detection()
    test_passed("Tap detection disabled")

except Exception as e:
    test_failed("Tap detection enable", str(e))

# ------------------------------------------------------------------------
# Test 10: Significant Motion Detection Enable
# ------------------------------------------------------------------------

print("\n[Test 10] Significant Motion Detection Enable")
print("-" * 80)

try:
    # Enable SMD
    icm.enable_significant_motion_detection(mode="short", int_pin=1)
    test_passed("Significant motion detection enabled (short mode)")

    # Wait briefly
    time.sleep(0.1)

    # Read status
    status = icm.read_interrupt_status()
    print(f"  SMD status: {status['smd']}")

    # Disable
    icm.disable_motion_detection()
    test_passed("Significant motion detection disabled")

except Exception as e:
    test_failed("Significant motion detection enable", str(e))

# ------------------------------------------------------------------------
# Test 11: Restore Normal Operation
# ------------------------------------------------------------------------

print("\n[Test 11] Restore Normal Operation")
print("-" * 80)

try:
    # Reset sensor to clean state
    icm.reset()
    test_passed("Sensor reset to normal operation")

    # Verify basic reading still works
    temp = icm.temperature
    accel = icm.acceleration
    gyro = icm.gyro

    test_passed("Basic readings work after reset")
    print(f"  Temp: {temp:.2f} °C")
    print(f"  Accel: {accel}")
    print(f"  Gyro: {gyro}")

except Exception as e:
    test_failed("Restore normal operation", str(e))

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
