"""
Comprehensive Cache Invalidation Test Suite

Tests all cache invalidation triggers and edge cases for the CircuitPython
ICM42688 library to verify the cache poisoning bug is fixed.

Hardware required: RP2040 + ICM42688 connected via I2C

NOTE ON API VALUES: The library's range/ODR/mode setters take *register codes*,
not physical values. e.g. ACCEL_RANGE_8G == 1 (not 8), ODR_100HZ == 8 (not 100),
ACCEL_MODE_LP == 2 (not "low_power"). Always pass the `reg.*` constants.

Author: Comprehensive validation script
Date: 2026-05-20
"""

import time
import board
import busio
from adafruit_icm42688 import ICM42688
from adafruit_icm42688 import registers as reg

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
        print(f"{GREEN}+{RESET} {test_name}")

    def record_fail(self, test_name, reason):
        self.failed += 1
        self.errors.append((test_name, reason))
        print(f"{RED}x{RESET} {test_name}: {reason}")

    def summary(self):
        total = self.passed + self.failed
        print(f"\n{'='*70}")
        print(f"Test Summary: {self.passed}/{total} passed")
        if self.failed > 0:
            print(f"\n{RED}Failed tests:{RESET}")
            for name, reason in self.errors:
                print(f"  - {name}: {reason}")
        print(f"{'='*70}\n")
        return self.failed == 0


def print_header(text):
    """Print formatted section header"""
    print(f"\n{BLUE}{'='*70}")
    print(f"  {text}")
    print(f"{'='*70}{RESET}\n")


def expect_runtime_error(fn):
    """Return True if calling fn() raises RuntimeError, False otherwise."""
    try:
        fn()
        return False
    except RuntimeError:
        return True


def test_basic_cache_api(icm, results):
    """Test basic cached API functionality"""
    print_header("Test 1: Basic Cached API")

    # 1.1: Cache should be invalid initially
    if expect_runtime_error(lambda: icm.accel_x):
        results.record_pass("1.1: Initial cache state is invalid")
    else:
        results.record_fail("1.1: Initial cache state", "Expected RuntimeError for invalid cache")

    # 1.2: refresh() should populate cache
    try:
        icm.refresh()
        _ = icm.accel_x
        _ = icm.accel_y
        _ = icm.accel_z
        results.record_pass("1.2: refresh() populates cache")
    except Exception as e:
        results.record_fail("1.2: refresh() populates cache", str(e))

    # 1.3: Multiple cached reads return same value (no re-read)
    try:
        icm.refresh()
        x1 = icm.accel_x
        x2 = icm.accel_x
        if x1 == x2:
            results.record_pass("1.3: Cached reads return same value")
        else:
            results.record_fail("1.3: Cached reads return same value", f"{x1} != {x2}")
    except Exception as e:
        results.record_fail("1.3: Cached reads return same value", str(e))

    # 1.4: Cache valid flag set after refresh
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

    try:
        original = icm.accelerometer_range
        new = reg.ACCEL_RANGE_8G if original != reg.ACCEL_RANGE_8G else reg.ACCEL_RANGE_4G

        icm.refresh()
        _ = icm.accel_x  # works

        icm.accelerometer_range = new
        if expect_runtime_error(lambda: icm.accel_x):
            results.record_pass("2.1: accelerometer_range invalidation")
        else:
            results.record_fail("2.1: accelerometer_range invalidation",
                              "Expected RuntimeError after range change")

        icm.refresh()
        _ = icm.accel_x
        results.record_pass("2.2: refresh() works after range change")

        icm.accelerometer_range = original  # restore
    except Exception as e:
        results.record_fail("2.1-2.2: accelerometer_range invalidation", str(e))


def test_invalidation_gyro_range(icm, results):
    """Test cache invalidation on gyro_range change"""
    print_header("Test 3: Gyro Range Invalidation")

    try:
        original = icm.gyro_range
        new = reg.GYRO_RANGE_500_DPS if original != reg.GYRO_RANGE_500_DPS else reg.GYRO_RANGE_250_DPS

        icm.refresh()
        _ = icm.gyro_x  # works

        icm.gyro_range = new
        if expect_runtime_error(lambda: icm.gyro_x):
            results.record_pass("3.1: gyro_range invalidation")
        else:
            results.record_fail("3.1: gyro_range invalidation",
                              "Expected RuntimeError after range change")

        icm.refresh()
        _ = icm.gyro_z
        results.record_pass("3.2: refresh() works after gyro range change")

        icm.gyro_range = original  # restore
    except Exception as e:
        results.record_fail("3.1-3.2: gyro_range invalidation", str(e))


def test_invalidation_accel_odr(icm, results):
    """Test cache invalidation on accelerometer_data_rate change"""
    print_header("Test 4: Accelerometer ODR Invalidation")

    try:
        original = icm.accelerometer_data_rate
        new = reg.ODR_100HZ if original != reg.ODR_100HZ else reg.ODR_200HZ

        icm.refresh()
        _ = icm.accel_x  # works

        icm.accelerometer_data_rate = new
        if expect_runtime_error(lambda: icm.accel_x):
            results.record_pass("4.1: accelerometer_data_rate invalidation")
        else:
            results.record_fail("4.1: accelerometer_data_rate invalidation",
                              "Expected RuntimeError after ODR change")

        icm.refresh()
        results.record_pass("4.2: refresh() works after accel ODR change")

        icm.accelerometer_data_rate = original  # restore
    except Exception as e:
        results.record_fail("4.1-4.2: accelerometer_data_rate invalidation", str(e))


def test_invalidation_gyro_odr(icm, results):
    """Test cache invalidation on gyro_data_rate change"""
    print_header("Test 5: Gyro ODR Invalidation")

    try:
        original = icm.gyro_data_rate
        new = reg.ODR_100HZ if original != reg.ODR_100HZ else reg.ODR_200HZ

        icm.refresh()
        _ = icm.gyro_y  # works

        icm.gyro_data_rate = new
        if expect_runtime_error(lambda: icm.gyro_y):
            results.record_pass("5.1: gyro_data_rate invalidation")
        else:
            results.record_fail("5.1: gyro_data_rate invalidation",
                              "Expected RuntimeError after ODR change")

        icm.refresh()
        results.record_pass("5.2: refresh() works after gyro ODR change")

        icm.gyro_data_rate = original  # restore
    except Exception as e:
        results.record_fail("5.1-5.2: gyro_data_rate invalidation", str(e))


def test_invalidation_power_mode(icm, results):
    """Test cache invalidation on power mode change"""
    print_header("Test 6: Power Mode Invalidation")

    try:
        icm.refresh()
        _ = icm.accel_x  # works

        # Switch accel to low-power (data path changes)
        icm.set_power_mode(accel_mode=reg.ACCEL_MODE_LP, gyro_mode=reg.GYRO_MODE_LN)
        if expect_runtime_error(lambda: icm.accel_x):
            results.record_pass("6.1: set_power_mode invalidation")
        else:
            results.record_fail("6.1: set_power_mode invalidation",
                              "Expected RuntimeError after power mode change")

        # Restore to low-noise
        icm.set_power_mode(accel_mode=reg.ACCEL_MODE_LN, gyro_mode=reg.GYRO_MODE_LN)
        icm.refresh()
        results.record_pass("6.2: refresh() works after power mode change")
    except Exception as e:
        results.record_fail("6.1-6.2: set_power_mode invalidation", str(e))


def test_invalidation_reset(icm, results):
    """Test cache invalidation on reset"""
    print_header("Test 7: Reset Invalidation")

    try:
        icm.refresh()
        _ = icm.accel_x  # works

        icm.reset()  # full reset + re-init (includes its own delays)

        if expect_runtime_error(lambda: icm.accel_x):
            results.record_pass("7.1: reset() invalidation")
        else:
            results.record_fail("7.1: reset() invalidation",
                              "Expected RuntimeError after reset")

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
        _ = icm.accel_x  # works

        icm.enable_fifo(accel=True, gyro=True)
        if expect_runtime_error(lambda: icm.accel_x):
            results.record_pass("8.1: enable_fifo() invalidation")
        else:
            results.record_fail("8.1: enable_fifo() invalidation",
                              "Expected RuntimeError after FIFO enable")

        icm.refresh()
        _ = icm.gyro_x
        results.record_pass("8.2: refresh() works with FIFO enabled")

        icm.disable_fifo()
        if expect_runtime_error(lambda: icm.accel_y):
            results.record_pass("8.3: disable_fifo() invalidation")
        else:
            results.record_fail("8.3: disable_fifo() invalidation",
                              "Expected RuntimeError after FIFO disable")

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
        # Note: do this after invalidating to prove it's independent of the cache.
        icm.invalidate_cache()
        temp, accel, gyro = icm.all_data
        results.record_pass("9.1: all_data works without refresh()")

        # 9.2: values must be physically plausible (real assertion, not a no-op).
        # Stationary board: temp roughly room/sensor temp, |accel| ~ 1 g.
        mag = (accel[0]**2 + accel[1]**2 + accel[2]**2) ** 0.5
        plausible = (0.0 < temp < 70.0) and (5.0 < mag < 15.0)
        if plausible:
            results.record_pass(f"9.2: all_data values plausible (temp={temp:.1f}C, |a|={mag:.2f})")
        else:
            results.record_fail("9.2: all_data values plausible",
                              f"temp={temp:.1f}C, |accel|={mag:.2f} m/s^2 (board must be still)")
    except Exception as e:
        results.record_fail("9.1-9.2: all_data property", str(e))


def test_acceleration_gyro_properties(icm, results):
    """Test acceleration/gyro tuple properties (non-cached API)"""
    print_header("Test 10: Non-Cached acceleration/gyro Properties")

    try:
        icm.invalidate_cache()  # prove these don't depend on the cache
        accel = icm.acceleration
        gyro = icm.gyro
        results.record_pass("10.1: acceleration property works without refresh()")
        results.record_pass("10.2: gyro property works without refresh()")

        if isinstance(accel, tuple) and len(accel) == 3:
            results.record_pass("10.3: acceleration returns 3-tuple")
        else:
            results.record_fail("10.3: acceleration returns 3-tuple",
                              f"Got {type(accel)} length {len(accel)}")

        if isinstance(gyro, tuple) and len(gyro) == 3:
            results.record_pass("10.4: gyro returns 3-tuple")
        else:
            results.record_fail("10.4: gyro returns 3-tuple",
                              f"Got {type(gyro)} length {len(gyro)}")
    except Exception as e:
        results.record_fail("10.1-10.4: acceleration/gyro properties", str(e))


def test_edge_cases(icm, results):
    """Test edge cases and error handling"""
    print_header("Test 11: Edge Cases")

    # 11.1: Multiple refresh() calls
    try:
        icm.refresh()
        icm.refresh()
        icm.refresh()
        _ = icm.accel_x
        results.record_pass("11.1: Multiple refresh() calls work")
    except Exception as e:
        results.record_fail("11.1: Multiple refresh() calls", str(e))

    # 11.2: cached value is a frozen snapshot, not live data. Several live
    # (non-cached) reads in between must NOT change the cached value at all.
    try:
        icm.refresh()
        x_cached = icm.accel_x
        for _ in range(5):
            _ = icm.acceleration  # live reads; sensor noise changes these
        x_cached2 = icm.accel_x  # must be byte-for-byte identical
        if x_cached == x_cached2:
            results.record_pass("11.2: cached value is a frozen snapshot across live reads")
        else:
            results.record_fail("11.2: snapshot semantics",
                              f"cached value drifted: {x_cached} != {x_cached2}")
    except Exception as e:
        results.record_fail("11.2: snapshot semantics", str(e))

    # 11.3: invalidate_cache() method directly
    try:
        icm.refresh()
        _ = icm.accel_x
        icm.invalidate_cache()
        if expect_runtime_error(lambda: icm.accel_x):
            results.record_pass("11.3: invalidate_cache() method works")
        else:
            results.record_fail("11.3: invalidate_cache() method",
                              "Expected RuntimeError after manual invalidation")
    except Exception as e:
        results.record_fail("11.3: invalidate_cache() method", str(e))

    # 11.4: All cached properties accessible after single refresh
    try:
        icm.refresh()
        _ = icm.accel_x
        _ = icm.accel_y
        _ = icm.accel_z
        _ = icm.gyro_x
        _ = icm.gyro_y
        _ = icm.gyro_z
        _ = icm.temp_cached
        results.record_pass("11.4: All cached properties accessible after refresh()")
    except Exception as e:
        results.record_fail("11.4: All cached properties", str(e))

    # 11.5: temp_cached also raises when invalid
    try:
        icm.invalidate_cache()
        if expect_runtime_error(lambda: icm.temp_cached):
            results.record_pass("11.5: temp_cached raises when cache invalid")
        else:
            results.record_fail("11.5: temp_cached raises when cache invalid",
                              "Expected RuntimeError")
    except Exception as e:
        results.record_fail("11.5: temp_cached invalid", str(e))


def test_real_world_usage(icm, results):
    """Test real-world usage patterns"""
    print_header("Test 12: Real-World Usage Patterns")

    # 12.1: Read loop with refresh
    try:
        for _ in range(10):
            icm.refresh()
            _ = (icm.accel_x, icm.accel_y, icm.accel_z)
            time.sleep(0.01)
        results.record_pass("12.1: Continuous read loop pattern")
    except Exception as e:
        results.record_fail("12.1: Continuous read loop", str(e))

    # 12.2: Configure-then-read
    try:
        icm.accelerometer_range = reg.ACCEL_RANGE_4G
        icm.accelerometer_data_rate = reg.ODR_100HZ
        time.sleep(0.05)
        icm.refresh()
        _ = (icm.accel_x, icm.accel_y, icm.accel_z)
        results.record_pass("12.2: Configure-then-read pattern")
        # restore defaults
        icm.accelerometer_range = reg.ACCEL_RANGE_16G
        icm.accelerometer_data_rate = reg.ODR_1KHZ
    except Exception as e:
        results.record_fail("12.2: Configure-then-read", str(e))

    # 12.3: Mixed cached and non-cached in application
    try:
        icm.refresh()
        for _ in range(5):
            _ = icm.accel_x  # cached
        _ = icm.acceleration  # occasional non-cached
        icm.refresh()
        _ = icm.accel_y
        results.record_pass("12.3: Mixed cached/non-cached pattern")
    except Exception as e:
        results.record_fail("12.3: Mixed usage", str(e))


def test_scale_correctness(icm, results):
    """
    Verify the converted gravity vector stays ~1 g across all accel ranges.

    Changing the range changes the raw LSB/g, and the library must update its
    scale factor so the reported m/s^2 stays consistent. If a range change
    failed to update the scale (a bug class adjacent to the cache fix, since
    both happen in the same setter), the magnitude would jump by up to 8x.

    Board must be stationary for this test.
    """
    print_header("Test 13: Scale-Factor Correctness Across Ranges")

    GRAVITY = 9.80665
    ranges = [
        (reg.ACCEL_RANGE_2G, "+/-2g"),
        (reg.ACCEL_RANGE_4G, "+/-4g"),
        (reg.ACCEL_RANGE_8G, "+/-8g"),
        (reg.ACCEL_RANGE_16G, "+/-16g"),
    ]
    try:
        for code, label in ranges:
            icm.accelerometer_range = code
            time.sleep(0.02)
            icm.refresh()
            ax, ay, az = icm.accel_x, icm.accel_y, icm.accel_z
            mag = (ax*ax + ay*ay + az*az) ** 0.5
            # Generous tolerance: catches an 8x scale error, tolerates board tilt.
            if abs(mag - GRAVITY) < 2.0:
                results.record_pass(f"13: {label} -> |accel|={mag:.2f} m/s^2 (~1 g)")
            else:
                results.record_fail(f"13: {label} scale correctness",
                                  f"|accel|={mag:.2f} m/s^2, expected ~{GRAVITY:.2f} (board still?)")
    except Exception as e:
        results.record_fail("13: scale correctness", str(e))
    finally:
        icm.accelerometer_range = reg.ACCEL_RANGE_16G  # restore default


def run_all_tests():
    """Run comprehensive test suite"""
    print("\n" + "="*70)
    print("  ICM42688 CircuitPython Cache Invalidation Test Suite")
    print("  Branch: claude/fix-circuitpython-cache-01W3ZLWLe4RqG3if7ugvoZA9")
    print("="*70 + "\n")

    print("Initializing sensor...")
    try:
        i2c = busio.I2C(board.SCL, board.SDA, frequency=400000)
        icm = ICM42688(i2c)
        print(f"{GREEN}+{RESET} Sensor initialized\n")
    except Exception as e:
        print(f"{RED}x{RESET} Failed to initialize sensor: {e}")
        return False

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
    test_scale_correctness(icm, results)

    success = results.summary()

    if success:
        print(f"{GREEN}ALL TESTS PASSED!{RESET}")
        print("The cache invalidation fix is working correctly.\n")
    else:
        print(f"{RED}SOME TESTS FAILED{RESET}")
        print("Review failures above and investigate.\n")

    return success


if __name__ == "__main__":
    try:
        ok = run_all_tests()
        print("RESULT:", "PASS" if ok else "FAIL")
    except KeyboardInterrupt:
        print("\n\nTests interrupted by user")
    except Exception as e:
        print(f"\n{RED}Fatal error:{RESET} {e}")
        import traceback
        traceback.print_exc()
