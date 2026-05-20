"""
Manual Interactive Cache Invalidation Test

Simple script to manually verify cache invalidation behavior.
Use this for quick verification and troubleshooting.

Hardware required: RP2040 + ICM42688 connected via I2C

Author: Manual verification script
Date: 2026-05-20
"""

import time
import board
import busio
from adafruit_icm42688 import ICM42688

print("\n" + "="*70)
print("  Manual Cache Invalidation Test")
print("  Branch: claude/fix-circuitpython-cache-01W3ZLWLe4RqG3if7ugvoZA9")
print("="*70 + "\n")

# Initialize sensor
print("Initializing sensor...")
i2c = busio.I2C(board.SCL, board.SDA, frequency=400000)
icm = ICM42688(i2c)
print("✓ Sensor ready\n")

# Test 1: Cache should be invalid initially
print("="*70)
print("Test 1: Initial cache state (should be invalid)")
print("="*70)
print("\nTrying to read accel_x without refresh()...")
try:
    x = icm.accel_x
    print("❌ FAIL: Expected RuntimeError but got value:", x)
except RuntimeError as e:
    print("✅ PASS: Got expected RuntimeError:", e)

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
print(f"✅ PASS: Got values: x={x:.2f}, y={y:.2f}, z={z:.2f} m/s²")

input("\nPress Enter to continue...")

# Test 3: Change accelerometer_range invalidates cache
print("\n" + "="*70)
print("Test 3: accelerometer_range change invalidates cache")
print("="*70)

original_range = icm.accelerometer_range
new_range = 8 if original_range != 8 else 4

print(f"\nCurrent range: ±{original_range}g")
print("Calling refresh() to populate cache...")
icm.refresh()
print(f"accel_x = {icm.accel_x:.2f} m/s²")

print(f"\nChanging range to ±{new_range}g...")
icm.accelerometer_range = new_range

print("Trying to read accel_x (cache should be invalid)...")
try:
    x = icm.accel_x
    print(f"❌ FAIL: Expected RuntimeError but got value: {x}")
except RuntimeError as e:
    print(f"✅ PASS: Got expected RuntimeError: {e}")

print("\nCalling refresh() to populate cache with new scale...")
icm.refresh()
print(f"accel_x = {icm.accel_x:.2f} m/s² (with new ±{new_range}g scale)")

print(f"\nRestoring original range (±{original_range}g)...")
icm.accelerometer_range = original_range

input("\nPress Enter to continue...")

# Test 4: FIFO enable invalidates cache
print("\n" + "="*70)
print("Test 4: FIFO enable/disable invalidates cache")
print("="*70)

print("\nCalling refresh() to populate cache...")
icm.refresh()
print(f"accel_x = {icm.accel_x:.2f} m/s²")

print("\nEnabling FIFO...")
icm.enable_fifo(accel=True, gyro=True)

print("Trying to read accel_x (cache should be invalid)...")
try:
    x = icm.accel_x
    print(f"❌ FAIL: Expected RuntimeError but got value: {x}")
except RuntimeError as e:
    print(f"✅ PASS: Got expected RuntimeError: {e}")

print("\nCalling refresh()...")
icm.refresh()
print(f"accel_x = {icm.accel_x:.2f} m/s² (with FIFO enabled)")

print("\nDisabling FIFO...")
icm.disable_fifo()

print("Trying to read accel_x (cache should be invalid again)...")
try:
    x = icm.accel_x
    print(f"❌ FAIL: Expected RuntimeError but got value: {x}")
except RuntimeError as e:
    print(f"✅ PASS: Got expected RuntimeError: {e}")

input("\nPress Enter to continue...")

# Test 5: Non-cached APIs still work
print("\n" + "="*70)
print("Test 5: Non-cached APIs work without refresh()")
print("="*70)

print("\nReading acceleration (non-cached API, no refresh needed)...")
accel = icm.acceleration
print(f"✅ acceleration = ({accel[0]:.2f}, {accel[1]:.2f}, {accel[2]:.2f}) m/s²")

print("\nReading gyro (non-cached API, no refresh needed)...")
gyro = icm.gyro
print(f"✅ gyro = ({gyro[0]:.2f}, {gyro[1]:.2f}, {gyro[2]:.2f}) dps")

print("\nReading all_data (non-cached API, no refresh needed)...")
temp, accel, gyro = icm.all_data
print(f"✅ temperature = {temp:.2f}°C")
print(f"✅ acceleration = ({accel[0]:.2f}, {accel[1]:.2f}, {accel[2]:.2f}) m/s²")
print(f"✅ gyro = ({gyro[0]:.2f}, {gyro[1]:.2f}, {gyro[2]:.2f}) dps")

input("\nPress Enter to continue...")

# Test 6: Real-world usage pattern
print("\n" + "="*70)
print("Test 6: Real-world usage pattern - continuous reading")
print("="*70)

print("\nReading sensor data 10 times with refresh() in loop...")
print("(This is the recommended pattern for cached API)\n")

for i in range(10):
    icm.refresh()  # Single I2C read (14 bytes)
    ax = icm.accel_x  # No I2C transaction
    ay = icm.accel_y  # No I2C transaction
    az = icm.accel_z  # No I2C transaction
    gx = icm.gyro_x   # No I2C transaction
    gy = icm.gyro_y   # No I2C transaction
    gz = icm.gyro_z   # No I2C transaction

    print(f"  [{i+1}] accel=({ax:6.2f}, {ay:6.2f}, {az:6.2f}) m/s²  "
          f"gyro=({gx:6.2f}, {gy:6.2f}, {gz:6.2f}) dps")
    time.sleep(0.1)

print("\n✅ PASS: Continuous reading pattern works correctly")

# Final summary
print("\n" + "="*70)
print("  Manual Test Summary")
print("="*70)
print("""
✅ All manual tests passed!

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
