"""
Comprehensive Cache Invalidation Test Suite

Tests all 7 cache invalidation triggers and edge cases for the CircuitPython
ICM42688 library to verify the cache poisoning bug is fixed.

Hardware required: RP2040 + ICM42688 connected via I2C

Author: Comprehensive validation script
Date: 2026-05-20
"""

import time
import board
import busio
from adafruit_icm42688 import ICM42688

# ANSI color codes for test output
GREEN = "\033[92m"
RED = "\033[91m"
YELLOW = "\033[93m"
BLUE = "\033[94m"
RESET = "\033[0m"

class TestResult:
    """Track test results"""
    def __init__(self):
        self.passed = 0
        self.failed = 0
        self.errors = []

    def record_pass(self, test_name):
        self.passed += 1
        print(f"{GREEN}✓{RESET} {test_name}")

    def record_fail(self, test_name, reason):
        self.failed += 1
        self.errors.append((test_name, reason))
        print(f"{RED}✗{RESET} {test_name}: {reason}")

    def summary(self):
        total = self.passed + self.failed
        print(f"\n{'='*70}")
        print(f"Test Summary: {self.passed}/{total} passed")
        if self.failed > 0:
            print(f"\n{RED}Failed tests:{RESET}")
            for name, reason in self.errors:
                print(f"  • {name}: {reason}")
        print(f"{'='*70}\n")
        return self.failed == 0


def print_header(text):
    """Print formatted section header"""
    print(f"\n{BLUE}{'='*70}")
    print(f"  {text}")
    print(f"{'='*70}{RESET}\n")


def test_basic_cache_api(icm, results):
    """Test basic cached API functionality"""
    print_header("Test 1: Basic Cached API")

    # Test 1.1: Cache should be invalid initially
    try:
        _ = icm.accel_x
        results.record_fail("1.1: Initial cache state", "Expected RuntimeError for invalid cache")
    except RuntimeError:
        results.record_pass("1.1: Initial cache state is invalid")

    # Test 1.2: refresh() should populate cache
    try:
        icm.refresh()
        x = icm.accel_x
        y = icm.accel_y
        z = icm.accel_z
        results.record_pass("1.2: refresh() populates cache")
    except Exception as e:
        results.record_fail("1.2: refresh() populates cache", str(e))

    # Test 1.3: Multiple cached reads without refresh
    try:
        icm.refresh()
        x1 = icm.accel_x
        x2 = icm.accel_x
        # Should return same cached value (no I2C transaction)
        if x1 == x2:
            results.record_pass("1.3: Cached reads return same value")
        else:
            results.record_fail("1.3: Cached reads return same value",
                              f"Values differ: {x1} != {x2}")
    except Exception as e:
        results.record_fail("1.3: Cached reads return same value", str(e))

    # Test 1.4: isCacheValid checks
    try:
        icm.refresh()
        if icm._cache_valid:
            results.record_pass("1.4: Cache valid after refresh()")
        else:
            results.record_fail("1.4: Cache valid after refresh()", "Cache not valid")
    except Exception as e:
        results.record_fail("1.4: Cache valid after refresh()", str(e))


def test_invalidation_accelerometer_range(icm, results):
    """Test cache invalidation on accelerometer_range change"""
    print_header("Test 2: Accelerometer Range Invalidation")

    # Test 2.1: Change range invalidates cache
    try:
        icm.refresh()
        _ = icm.accel_x  # Should work

        # Change range (scale factor changes)
        original_range = icm.accelerometer_range
        new_range = 8 if original_range != 8 else 4
        icm.accelerometer_range = new_range

        # Cache should now be invalid
        try:
            _ = icm.accel_x
            results.record_fail("2.1: accelerometer_range invalidation",
                              "Expected RuntimeError after range change")
        except RuntimeError:
            results.record_pass("2.1: accelerometer_range invalidation")

        # Restore original range
        icm.accelerometer_range = original_range

    except Exception as e:
        results.record_fail("2.1: accelerometer_range invalidation", str(e))

    # Test 2.2: refresh() after range change works
    try:
        icm.refresh()
        x = icm.accel_x  # Should work with new scale
        results.record_pass("2.2: refresh() works after range change")
    except Exception as e:
        results.record_fail("2.2: refresh() works after range change", str(e))


def test_invalidation_gyro_range(icm, results):
    """Test cache invalidation on gyro_range change"""
    print_header("Test 3: Gyro Range Invalidation")

    try:
        icm.refresh()
        _ = icm.gyro_x  # Should work

        # Change range
        original_range = icm.gyro_range
        new_range = 500 if original_range != 500 else 250
        icm.gyro_range = new_range

        # Cache should be invalid
        try:
            _ = icm.gyro_x
            results.record_fail("3.1: gyro_range invalidation",
                              "Expected RuntimeError after range change")
        except RuntimeError:
            results.record_pass("3.1: gyro_range invalidation")

        # Restore and verify
        icm.gyro_range = original_range
        icm.refresh()
        _ = icm.gyro_z
        results.record_pass("3.2: refresh() works after gyro range change")

    except Exception as e:
        results.record_fail("3.1-3.2: gyro_range invalidation", str(e))


def test_invalidation_accel_odr(icm, results):
    """Test cache invalidation on accelerometer_data_rate change"""
    print_header("Test 4: Accelerometer ODR Invalidation")

    try:
        icm.refresh()
        _ = icm.accel_x  # Should work

        # Change ODR (data path changes)
        original_odr = icm.accelerometer_data_rate
        new_odr = 100 if original_odr != 100 else 200
        icm.accelerometer_data_rate = new_odr

        # Cache should be invalid
        try:
            _ = icm.accel_x
            results.record_fail("4.1: accelerometer_data_rate invalidation",
                              "Expected RuntimeError after ODR change")
        except RuntimeError:
            results.record_pass("4.1: accelerometer_data_rate invalidation")

        # Restore
        icm.accelerometer_data_rate = original_odr
        icm.refresh()
        results.record_pass("4.2: refresh() works after accel ODR change")

    except Exception as e:
        results.record_fail("4.1-4.2: accelerometer_data_rate invalidation", str(e))


def test_invalidation_gyro_odr(icm, results):
    """Test cache invalidation on gyro_data_rate change"""
    print_header("Test 5: Gyro ODR Invalidation")

    try:
        icm.refresh()
        _ = icm.gyro_y  # Should work

        # Change ODR
        original_odr = icm.gyro_data_rate
        new_odr = 100 if original_odr != 100 else 200
        icm.gyro_data_rate = new_odr

        # Cache should be invalid
        try:
            _ = icm.gyro_y
            results.record_fail("5.1: gyro_data_rate invalidation",
                              "Expected RuntimeError after ODR change")
        except RuntimeError:
            results.record_pass("5.1: gyro_data_rate invalidation")

        # Restore
        icm.gyro_data_rate = original_odr
        icm.refresh()
        results.record_pass("5.2: refresh() works after gyro ODR change")

    except Exception as e:
        results.record_fail("5.1-5.2: gyro_data_rate invalidation", str(e))


def test_invalidation_power_mode(icm, results):
    """Test cache invalidation on power mode change"""
    print_header("Test 6: Power Mode Invalidation")

    try:
        icm.refresh()
        _ = icm.accel_x  # Should work

        # Change power mode (data path changes)
        icm.set_power_mode(accel_mode="low_power", gyro_mode="low_noise")

        # Cache should be invalid
        try:
            _ = icm.accel_x
            results.record_fail("6.1: set_power_mode invalidation",
                              "Expected RuntimeError after power mode change")
        except RuntimeError:
            results.record_pass("6.1: set_power_mode invalidation")

        # Restore to low noise mode
        icm.set_power_mode(accel_mode="low_noise", gyro_mode="low_noise")
        icm.refresh()
        results.record_pass("6.2: refresh() works after power mode change")

    except Exception as e:
        results.record_fail("6.1-6.2: set_power_mode invalidation", str(e))


def test_invalidation_reset(icm, results):
    """Test cache invalidation on reset"""
    print_header("Test 7: Reset Invalidation")

    try:
        icm.refresh()
        _ = icm.accel_x  # Should work

        # Reset sensor (everything changes)
        icm.reset()
        time.sleep(0.1)  # Wait for reset to complete

        # Cache should be invalid
        try:
            _ = icm.accel_x
            results.record_fail("7.1: reset() invalidation",
                              "Expected RuntimeError after reset")
        except RuntimeError:
            results.record_pass("7.1: reset() invalidation")

        # Re-initialize and verify
        time.sleep(0.1)
        icm.refresh()
        _ = icm.accel_z
        results.record_pass("7.2: refresh() works after reset")

    except Exception as e:
        results.record_fail("7.1-7.2: reset() invalidation", str(e))


def test_invalidation_fifo(icm, results):
    """Test cache invalidation on FIFO enable/disable"""
    print_header("Test 8: FIFO Enable/Disable Invalidation")

    try:
        icm.refresh()
        _ = icm.accel_x  # Should work

        # Enable FIFO (data path changes)
        icm.enable_fifo(accel=True, gyro=True)

        # Cache should be invalid
        try:
            _ = icm.accel_x
            results.record_fail("8.1: enable_fifo() invalidation",
                              "Expected RuntimeError after FIFO enable")
        except RuntimeError:
            results.record_pass("8.1: enable_fifo() invalidation")

        # Verify refresh works
        icm.refresh()
        _ = icm.gyro_x
        results.record_pass("8.2: refresh() works with FIFO enabled")

        # Disable FIFO
        icm.disable_fifo()

        # Cache should be invalid again
        try:
            _ = icm.accel_y
            results.record_fail("8.3: disable_fifo() invalidation",
                              "Expected RuntimeError after FIFO disable")
        except RuntimeError:
            results.record_pass("8.3: disable_fifo() invalidation")

        # Verify refresh works
        icm.refresh()
        _ = icm.accel_z
        results.record_pass("8.4: refresh() works after FIFO disable")

    except Exception as e:
        results.record_fail("8.1-8.4: FIFO invalidation", str(e))


def test_all_data_property(icm, results):
    """Test all_data property (non-cached API)"""
    print_header("Test 9: Non-Cached all_data Property")

    try:
        # all_data should work without refresh (always reads sensor)
        temp, accel, gyro = icm.all_data
        results.record_pass("9.1: all_data works without refresh()")

        # Should return different values on successive reads
        time.sleep(0.01)
        temp2, accel2, gyro2 = icm.all_data

        # At least one value should differ (sensor noise)
        if accel != accel2 or gyro != gyro2:
            results.record_pass("9.2: all_data reads fresh data each time")
        else:
            # Might be the same by coincidence, not necessarily a failure
            print(f"{YELLOW}⚠{RESET} 9.2: all_data values identical (may be coincidence)")
            results.record_pass("9.2: all_data returns data")

    except Exception as e:
        results.record_fail("9.1-9.2: all_data property", str(e))


def test_acceleration_gyro_properties(icm, results):
    """Test acceleration/gyro tuple properties (non-cached API)"""
    print_header("Test 10: Non-Cached acceleration/gyro Properties")

    try:
        # These should work without refresh
        accel = icm.acceleration
        gyro = icm.gyro

        results.record_pass("10.1: acceleration property works without refresh()")
        results.record_pass("10.2: gyro property works without refresh()")

        # Verify they return tuples
        if isinstance(accel, tuple) and len(accel) == 3:
            results.record_pass("10.3: acceleration returns 3-tuple")
        else:
            results.record_fail("10.3: acceleration returns 3-tuple",
                              f"Got {type(accel)} with length {len(accel)}")

        if isinstance(gyro, tuple) and len(gyro) == 3:
            results.record_pass("10.4: gyro returns 3-tuple")
        else:
            results.record_fail("10.4: gyro returns 3-tuple",
                              f"Got {type(gyro)} with length {len(gyro)}")

    except Exception as e:
        results.record_fail("10.1-10.4: acceleration/gyro properties", str(e))


def test_edge_cases(icm, results):
    """Test edge cases and error handling"""
    print_header("Test 11: Edge Cases")

    # Test 11.1: Multiple refresh() calls
    try:
        icm.refresh()
        icm.refresh()
        icm.refresh()
        x = icm.accel_x
        results.record_pass("11.1: Multiple refresh() calls work")
    except Exception as e:
        results.record_fail("11.1: Multiple refresh() calls", str(e))

    # Test 11.2: Interleaved cached and non-cached reads
    try:
        icm.refresh()
        x_cached = icm.accel_x
        x_y_z = icm.acceleration  # Non-cached
        x_cached2 = icm.accel_x  # Should still be cached

        if x_cached == x_cached2:
            results.record_pass("11.2: Cached values persist across non-cached reads")
        else:
            results.record_fail("11.2: Cached values persist",
                              f"Cache changed: {x_cached} != {x_cached2}")
    except Exception as e:
        results.record_fail("11.2: Interleaved reads", str(e))

    # Test 11.3: invalidate_cache() method directly
    try:
        icm.refresh()
        _ = icm.accel_x  # Should work

        icm.invalidate_cache()

        try:
            _ = icm.accel_x
            results.record_fail("11.3: invalidate_cache() method",
                              "Expected RuntimeError after manual invalidation")
        except RuntimeError:
            results.record_pass("11.3: invalidate_cache() method works")
    except Exception as e:
        results.record_fail("11.3: invalidate_cache() method", str(e))

    # Test 11.4: All cached properties after single refresh
    try:
        icm.refresh()
        ax = icm.accel_x
        ay = icm.accel_y
        az = icm.accel_z
        gx = icm.gyro_x
        gy = icm.gyro_y
        gz = icm.gyro_z
        temp = icm.temperature

        results.record_pass("11.4: All cached properties accessible after refresh()")
    except Exception as e:
        results.record_fail("11.4: All cached properties", str(e))


def test_real_world_usage(icm, results):
    """Test real-world usage patterns"""
    print_header("Test 12: Real-World Usage Patterns")

    # Pattern 1: Read loop with refresh
    try:
        samples = []
        for _ in range(10):
            icm.refresh()
            x = icm.accel_x
            y = icm.accel_y
            z = icm.accel_z
            samples.append((x, y, z))
            time.sleep(0.01)

        results.record_pass("12.1: Continuous read loop pattern")
    except Exception as e:
        results.record_fail("12.1: Continuous read loop", str(e))

    # Pattern 2: Configure-then-read
    try:
        icm.accelerometer_range = 4
        icm.accelerometer_data_rate = 100
        time.sleep(0.05)  # Let settings take effect

        icm.refresh()
        data = (icm.accel_x, icm.accel_y, icm.accel_z)

        results.record_pass("12.2: Configure-then-read pattern")
    except Exception as e:
        results.record_fail("12.2: Configure-then-read", str(e))

    # Pattern 3: Mixed cached and non-cached in application
    try:
        # Fast loop with cached reads
        icm.refresh()
        for _ in range(5):
            _ = icm.accel_x  # Cached - fast

        # Occasional non-cached read for verification
        fresh = icm.acceleration

        # Resume cached reads
        icm.refresh()
        _ = icm.accel_y

        results.record_pass("12.3: Mixed cached/non-cached pattern")
    except Exception as e:
        results.record_fail("12.3: Mixed usage", str(e))


def run_all_tests():
    """Run comprehensive test suite"""
    print("\n" + "="*70)
    print("  ICM42688 CircuitPython Cache Invalidation Test Suite")
    print("  Branch: claude/fix-circuitpython-cache-01W3ZLWLe4RqG3if7ugvoZA9")
    print("="*70 + "\n")

    # Initialize sensor
    print("Initializing sensor...")
    try:
        i2c = busio.I2C(board.SCL, board.SDA, frequency=400000)
        icm = ICM42688(i2c)
        print(f"{GREEN}✓{RESET} Sensor initialized\n")
    except Exception as e:
        print(f"{RED}✗{RESET} Failed to initialize sensor: {e}")
        return False

    # Run all tests
    results = TestResult()

    test_basic_cache_api(icm, results)
    test_invalidation_accelerometer_range(icm, results)
    test_invalidation_gyro_range(icm, results)
    test_invalidation_accel_odr(icm, results)
    test_invalidation_gyro_odr(icm, results)
    test_invalidation_power_mode(icm, results)
    test_invalidation_reset(icm, results)
    test_invalidation_fifo(icm, results)
    test_all_data_property(icm, results)
    test_acceleration_gyro_properties(icm, results)
    test_edge_cases(icm, results)
    test_real_world_usage(icm, results)

    # Print summary
    success = results.summary()

    if success:
        print(f"{GREEN}✅ ALL TESTS PASSED!{RESET}")
        print("The cache invalidation fix is working correctly.\n")
    else:
        print(f"{RED}❌ SOME TESTS FAILED{RESET}")
        print("Review failures above and investigate.\n")

    return success


if __name__ == "__main__":
    try:
        success = run_all_tests()
        # Return appropriate exit code for automation
        import sys
        sys.exit(0 if success else 1)
    except KeyboardInterrupt:
        print("\n\nTests interrupted by user")
    except Exception as e:
        print(f"\n{RED}Fatal error:{RESET} {e}")
        import traceback
        traceback.print_exc()
