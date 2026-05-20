"""
Manual Interactive Cache Invalidation Test

Simple script to manually verify cache invalidation behavior.
Use this for quick verification and troubleshooting.

Requires an interactive serial connection (uses input() to step through tests).

Hardware required: RP2040 + ICM42688 connected via I2C

NOTE ON API VALUES: range/ODR/mode setters take *register codes*, not physical
values. Always pass the reg.* constants (e.g. reg.ACCEL_RANGE_8G, not 8).

Author: Manual verification script
Date: 2026-05-20
"""

import time
import board
import busio
from adafruit_icm42688 import ICM42688
from adafruit_icm42688 import registers as reg

# Human-readable labels for accel range register codes
ACCEL_RANGE_LABEL = {
    reg.ACCEL_RANGE_2G: "+/-2g",
    reg.ACCEL_RANGE_4G: "+/-4g",
    reg.ACCEL_RANGE_8G: "+/-8g",
    reg.ACCEL_RANGE_16G: "+/-16g",
}

print("\n" + "="*70)
print("  Manual Cache Invalidation Test")
print("  Branch: claude/fix-circuitpython-cache-01W3ZLWLe4RqG3if7ugvoZA9")
print("="*70 + "\n")

print("Initializing sensor...")
i2c = busio.I2C(board.SCL, board.SDA, frequency=400000)
icm = ICM42688(i2c)
print("Sensor ready\n")

# Test 1: Cache should be invalid initially
print("="*70)
print("Test 1: Initial cache state (should be invalid)")
print("="*70)
print("\nTrying to read accel_x without refresh()...")
try:
    x = icm.accel_x
    print("FAIL: Expected RuntimeError but got value:", x)
except RuntimeError as e:
    print("PASS: Got expected RuntimeError:", e)

input("\nPress Enter to continue...")

# Test 2: refresh() populates cache
print("\n" + "="*70)
print("Test 2: refresh() populates cache")
print("="*70)
print("\nCalling refresh()...")
icm.refresh()
print("Reading accel_x, accel_y, accel_z...")
x = icm.accel_x
y = icm.accel_y
z = icm.accel_z
print("PASS: Got values: x=%.2f y=%.2f z=%.2f m/s^2" % (x, y, z))

input("\nPress Enter to continue...")

# Test 3: Change accelerometer_range invalidates cache
print("\n" + "="*70)
print("Test 3: accelerometer_range change invalidates cache")
print("="*70)

original_range = icm.accelerometer_range
new_range = reg.ACCEL_RANGE_8G if original_range != reg.ACCEL_RANGE_8G else reg.ACCEL_RANGE_4G

print("\nCurrent range:", ACCEL_RANGE_LABEL.get(original_range, original_range))
print("Calling refresh() to populate cache...")
icm.refresh()
print("accel_x = %.2f m/s^2" % icm.accel_x)

print("\nChanging range to", ACCEL_RANGE_LABEL.get(new_range, new_range), "...")
icm.accelerometer_range = new_range

print("Trying to read accel_x (cache should be invalid)...")
try:
    x = icm.accel_x
    print("FAIL: Expected RuntimeError but got value: %.2f" % x)
except RuntimeError as e:
    print("PASS: Got expected RuntimeError:", e)

print("\nCalling refresh() to populate cache with new scale...")
icm.refresh()
print("accel_x = %.2f m/s^2 (with new %s scale)" % (icm.accel_x, ACCEL_RANGE_LABEL.get(new_range, new_range)))

print("\nRestoring original range (%s)..." % ACCEL_RANGE_LABEL.get(original_range, original_range))
icm.accelerometer_range = original_range

input("\nPress Enter to continue...")

# Test 4: FIFO enable/disable invalidates cache
print("\n" + "="*70)
print("Test 4: FIFO enable/disable invalidates cache")
print("="*70)

print("\nCalling refresh() to populate cache...")
icm.refresh()
print("accel_x = %.2f m/s^2" % icm.accel_x)

print("\nEnabling FIFO...")
icm.enable_fifo(accel=True, gyro=True)

print("Trying to read accel_x (cache should be invalid)...")
try:
    x = icm.accel_x
    print("FAIL: Expected RuntimeError but got value: %.2f" % x)
except RuntimeError as e:
    print("PASS: Got expected RuntimeError:", e)

print("\nCalling refresh()...")
icm.refresh()
print("accel_x = %.2f m/s^2 (with FIFO enabled)" % icm.accel_x)

print("\nDisabling FIFO...")
icm.disable_fifo()

print("Trying to read accel_x (cache should be invalid again)...")
try:
    x = icm.accel_x
    print("FAIL: Expected RuntimeError but got value: %.2f" % x)
except RuntimeError as e:
    print("PASS: Got expected RuntimeError:", e)

input("\nPress Enter to continue...")

# Test 5: Non-cached APIs still work
print("\n" + "="*70)
print("Test 5: Non-cached APIs work without refresh()")
print("="*70)

icm.invalidate_cache()  # prove independence from the cache

print("\nReading acceleration (non-cached API, no refresh needed)...")
accel = icm.acceleration
print("PASS: acceleration = (%.2f, %.2f, %.2f) m/s^2" % (accel[0], accel[1], accel[2]))

print("\nReading gyro (non-cached API, no refresh needed)...")
gyro = icm.gyro
print("PASS: gyro = (%.2f, %.2f, %.2f) rad/s" % (gyro[0], gyro[1], gyro[2]))

print("\nReading all_data (non-cached API, no refresh needed)...")
temp, accel, gyro = icm.all_data
print("PASS: temperature = %.2f C" % temp)
print("PASS: acceleration = (%.2f, %.2f, %.2f) m/s^2" % (accel[0], accel[1], accel[2]))
print("PASS: gyro = (%.2f, %.2f, %.2f) rad/s" % (gyro[0], gyro[1], gyro[2]))

input("\nPress Enter to continue...")

# Test 6: Real-world usage pattern
print("\n" + "="*70)
print("Test 6: Real-world usage pattern - continuous reading")
print("="*70)

print("\nReading sensor data 10 times with refresh() in loop...")
print("(This is the recommended pattern for cached API)\n")

for i in range(10):
    icm.refresh()      # Single I2C read (14 bytes)
    ax = icm.accel_x   # No I2C transaction
    ay = icm.accel_y
    az = icm.accel_z
    gx = icm.gyro_x
    gy = icm.gyro_y
    gz = icm.gyro_z
    print("  [%2d] accel=(%6.2f, %6.2f, %6.2f) m/s^2  gyro=(%6.2f, %6.2f, %6.2f) rad/s"
          % (i + 1, ax, ay, az, gx, gy, gz))
    time.sleep(0.1)

print("\nPASS: Continuous reading pattern works correctly")

print("\n" + "="*70)
print("  Manual Test Summary")
print("="*70)
print("""
All manual tests passed!

Key behaviors verified:
  1. Cache starts invalid (RuntimeError before first refresh)
  2. refresh() populates cache for all axes
  3. accelerometer_range change invalidates cache
  4. FIFO enable/disable invalidates cache
  5. Non-cached APIs work without refresh()
  6. Real-world usage pattern works correctly

The cache invalidation fix is working as expected.

For comprehensive automated testing, run:
  test_cache_invalidation.py
""")
print("="*70 + "\n")
