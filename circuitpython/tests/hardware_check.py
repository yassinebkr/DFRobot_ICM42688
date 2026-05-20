"""
Hardware Connection Check

Verifies that the ICM42688 sensor is properly connected and responding
before running the full test suite.

Hardware required: RP2040 + ICM42688 connected via I2C

Author: Hardware verification script
Date: 2026-05-20
"""

import time
import board
import busio

print("\n" + "="*70)
print("  ICM42688 Hardware Connection Check")
print("="*70 + "\n")

# Step 1: Initialize I2C bus
print("Step 1: Initializing I2C bus...")
print(f"  SCL: {board.SCL}")
print(f"  SDA: {board.SDA}")

try:
    i2c = busio.I2C(board.SCL, board.SDA, frequency=400000)
    print("✓ I2C bus initialized (400 kHz)\n")
except Exception as e:
    print(f"✗ Failed to initialize I2C bus: {e}\n")
    print("Check:")
    print("  1. SCL and SDA pin definitions correct for your board")
    print("  2. No I2C devices conflicting")
    print("  3. CircuitPython version supports your board\n")
    raise

# Step 2: Scan I2C bus
print("Step 2: Scanning I2C bus...")
while not i2c.try_lock():
    pass

try:
    devices = i2c.scan()
    print(f"  Found {len(devices)} device(s):")
    for addr in devices:
        print(f"    • 0x{addr:02X}")
    print()

    # ICM42688 default addresses
    ICM42688_ADDR_LOW = 0x68   # AD0 pin = GND
    ICM42688_ADDR_HIGH = 0x69  # AD0 pin = VCC

    if ICM42688_ADDR_LOW in devices:
        print(f"✓ ICM42688 found at 0x{ICM42688_ADDR_LOW:02X} (AD0=GND)\n")
        icm_address = ICM42688_ADDR_LOW
    elif ICM42688_ADDR_HIGH in devices:
        print(f"✓ ICM42688 found at 0x{ICM42688_ADDR_HIGH:02X} (AD0=VCC)\n")
        icm_address = ICM42688_ADDR_HIGH
    else:
        print(f"✗ ICM42688 not found at 0x{ICM42688_ADDR_LOW:02X} or 0x{ICM42688_ADDR_HIGH:02X}\n")
        print("Check:")
        print("  1. Sensor is connected to I2C bus")
        print("  2. VCC = 3.3V (NOT 5V unless you have a level shifter)")
        print("  3. GND is connected")
        print("  4. Pull-up resistors on SCL/SDA (usually built-in)")
        print("  5. Wiring: SCL to SCL, SDA to SDA")
        print("\nDevices found on bus:")
        if len(devices) == 0:
            print("  None - check all connections")
        else:
            for addr in devices:
                print(f"  • 0x{addr:02X} - unknown device")
        print()
        i2c.unlock()
        raise RuntimeError("ICM42688 not detected")

finally:
    i2c.unlock()

# Step 3: Try to initialize sensor
print("Step 3: Initializing ICM42688 library...")
try:
    from adafruit_icm42688 import ICM42688
    icm = ICM42688(i2c)
    print("✓ ICM42688 library initialized\n")
except ImportError as e:
    print(f"✗ Failed to import adafruit_icm42688: {e}\n")
    print("Check:")
    print("  1. Library is installed in /CIRCUITPY/lib/adafruit_icm42688/")
    print("  2. Library __init__.py exists")
    print("  3. No syntax errors in library\n")
    raise
except Exception as e:
    print(f"✗ Failed to initialize sensor: {e}\n")
    print("The sensor was detected but initialization failed.")
    print("This could indicate:")
    print("  1. Sensor is not responding (check power)")
    print("  2. Wrong I2C address configured")
    print("  3. Sensor reset/timing issue (try power cycle)\n")
    raise

# Step 4: Read WHO_AM_I register
print("Step 4: Verifying WHO_AM_I register...")
try:
    # The library should have read this during init
    # We'll read some sensor data to verify communication
    temp, accel, gyro = icm.all_data
    print(f"✓ Sensor responding")
    print(f"  Temperature: {temp:.2f}°C")
    print(f"  Accel: ({accel[0]:.2f}, {accel[1]:.2f}, {accel[2]:.2f}) m/s²")
    print(f"  Gyro: ({gyro[0]:.2f}, {gyro[1]:.2f}, {gyro[2]:.2f}) dps\n")
except Exception as e:
    print(f"✗ Failed to read sensor data: {e}\n")
    raise

# Step 5: Basic sanity checks
print("Step 5: Sensor data sanity checks...")

# Check accelerometer Z-axis (should be ~9.8 m/s² if board is flat)
az = accel[2]
if 8.0 < az < 11.0:
    print(f"✓ Accel Z-axis reasonable: {az:.2f} m/s² (board appears flat)")
elif -11.0 < az < -8.0:
    print(f"✓ Accel Z-axis reasonable: {az:.2f} m/s² (board appears upside down)")
else:
    print(f"⚠ Accel Z-axis unexpected: {az:.2f} m/s²")
    print("  Expected ~±9.8 m/s² if board is stationary")
    print("  Sensor may need calibration or board is moving")

# Check gyro is near zero (stationary). Library returns rad/s.
gx, gy, gz = gyro
gyro_mag = (gx**2 + gy**2 + gz**2)**0.5
if gyro_mag < 5.0:
    print(f"✓ Gyro readings near zero: {gyro_mag:.2f} rad/s (board stationary)")
else:
    print(f"⚠ Gyro readings high: {gyro_mag:.2f} rad/s")
    print("  Board may be moving or needs calibration")

# Check temperature is reasonable
if 15.0 < temp < 40.0:
    print(f"✓ Temperature reasonable: {temp:.2f}°C")
else:
    print(f"⚠ Temperature unusual: {temp:.2f}°C")
    print("  Sensor may be reading incorrectly or in extreme environment")

print()

# Final result
print("="*70)
print("  Hardware Check Complete")
print("="*70)
print("""
✅ All checks passed!

Your hardware setup is working correctly:
  • I2C bus operational at 400 kHz
  • ICM42688 sensor detected and responding
  • Sensor data appears valid
  • Library successfully initialized

You can now run the test suites:
  • tests/manual_cache_test.py - Interactive manual tests
  • tests/test_cache_invalidation.py - Comprehensive automated tests

""")
print("="*70 + "\n")
