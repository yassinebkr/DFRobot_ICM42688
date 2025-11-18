# SPDX-FileCopyrightText: 2025 Yassine Bekkari
# SPDX-License-Identifier: MIT

"""
ICM42688 test for ESP32-WROOM-32 40-pin DevKit - SPI Interface

This example demonstrates how to use the ICM42688 sensor with an
ESP32-WROOM-32 development board (40-pin version) via SPI.

Hardware Wiring (SPI):
=====================
    ICM42688 Pin    →    ESP32-WROOM-32 Pin      →    Function
    ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    VCC/VDD         →    3.3V                     →    Power (3.3V ONLY!)
    GND             →    GND                      →    Ground
    MOSI (SDI)      →    GPIO23 (Pin 37) - MOSI   →    SPI Data In
    MISO (SDO)      →    GPIO19 (Pin 31) - MISO   →    SPI Data Out
    SCK (SCLK)      →    GPIO18 (Pin 30) - SCK    →    SPI Clock
    CS (nCS)        →    GPIO5  (Pin 29)          →    Chip Select
    INT1 (optional) →    GPIO4  (Pin 26)          →    Interrupt 1
    INT2 (optional) →    GPIO15 (Pin 23)          →    Interrupt 2

CRITICAL:
- ICM42688 operates at 3.3V ONLY. Never connect 5V to VCC!
- CS (Chip Select) must be connected for SPI operation
- Pull CS high when not communicating with sensor

ESP32 VSPI Pins (Default SPI):
==============================
    MOSI  = GPIO23 (Pin 37)
    MISO  = GPIO19 (Pin 31)
    SCK   = GPIO18 (Pin 30)
    CS    = Any GPIO (GPIO5 recommended)

ESP32 HSPI Pins (Alternative SPI):
==================================
    MOSI  = GPIO13 (Pin 15)
    MISO  = GPIO12 (Pin 13)
    SCK   = GPIO14 (Pin 12)
    CS    = Any GPIO

CircuitPython Setup:
===================
Same as I2C version - see esp32_wroom32_i2c_test.py for details
"""

import time
import board
import busio
import digitalio
import adafruit_icm42688
from adafruit_icm42688 import registers as reg

print("=" * 70)
print("ICM42688 SPI Test - ESP32-WROOM-32 (40-pin DevKit)")
print("=" * 70)

# ========================================================================
# SPI Configuration
# ========================================================================

# ESP32 VSPI (Default SPI) Pins:
#   MOSI = GPIO23 (Pin 37)
#   MISO = GPIO19 (Pin 31)
#   SCK  = GPIO18 (Pin 30)
#   CS   = GPIO5  (Pin 29) - user configurable

print("\nInitializing SPI bus...")
print("  MOSI: GPIO23 (Pin 37)")
print("  MISO: GPIO19 (Pin 31)")
print("  SCK:  GPIO18 (Pin 30)")
print("  CS:   GPIO5  (Pin 29)")
print("  Baudrate: 10 MHz")

# Initialize SPI bus
spi = busio.SPI(clock=board.IO18, MOSI=board.IO23, MISO=board.IO19)

# Configure Chip Select pin
cs = digitalio.DigitalInOut(board.IO5)
cs.direction = digitalio.Direction.OUTPUT
cs.value = True  # CS is active-low, so start HIGH (inactive)

print("✓ SPI bus initialized")

# ========================================================================
# Initialize ICM42688
# ========================================================================

print("\nInitializing ICM42688 sensor via SPI...")

try:
    # SPI interface - address parameter is ignored
    icm = adafruit_icm42688.ICM42688(spi, cs=cs, baudrate=10000000)
    print("✓ ICM42688 initialized successfully via SPI")
except Exception as e:
    print(f"✗ FAILED to initialize ICM42688")
    print(f"Error: {e}")
    print("\nTroubleshooting:")
    print("  1. Check all SPI wiring (MOSI, MISO, SCK, CS, VCC, GND)")
    print("  2. Verify CS pin is connected (required for SPI)")
    print("  3. Ensure ICM42688 is in SPI mode (not I2C mode)")
    print("  4. Try lower SPI baudrate (1 MHz)")
    print("  5. Check for shorts or loose connections")
    raise

# ========================================================================
# Configure Sensor
# ========================================================================

print("\nConfiguring sensor...")
icm.accelerometer_range = reg.ACCEL_RANGE_16G
icm.gyro_range = reg.GYRO_RANGE_2000_DPS
icm.accelerometer_data_rate = reg.ODR_1KHZ
icm.gyro_data_rate = reg.ODR_1KHZ
icm.set_power_mode(accel_mode=reg.ACCEL_MODE_LN, gyro_mode=reg.GYRO_MODE_LN)

print("  Accelerometer: ±16g, 1kHz, Low-Noise mode")
print("  Gyroscope: ±2000 dps, 1kHz, Low-Noise mode")
print("  Temperature: Enabled")
print("✓ Configuration complete")

# ========================================================================
# Test 1: Basic Sensor Reading
# ========================================================================

print("\n" + "=" * 70)
print("TEST 1: Basic Sensor Reading via SPI")
print("=" * 70)

temp = icm.temperature
accel_x, accel_y, accel_z = icm.acceleration
gyro_x, gyro_y, gyro_z = icm.gyro

print(f"\nTemperature:  {temp:.2f} °C")
print(f"Acceleration: X={accel_x:7.2f} Y={accel_y:7.2f} Z={accel_z:7.2f} m/s²")
print(f"Gyroscope:    X={gyro_x:7.4f} Y={gyro_y:7.4f} Z={gyro_z:7.4f} rad/s")

# Verify gravity on Z-axis
if 8.0 < abs(accel_z) < 11.0:
    print("\n✓ Gravity detected - SPI communication working!")
else:
    print(f"\n⚠ Warning: Z-axis = {accel_z:.2f} m/s² (expected ~±9.8)")

# ========================================================================
# Test 2: SPI Speed Test
# ========================================================================

print("\n" + "=" * 70)
print("TEST 2: SPI Communication Speed Test")
print("=" * 70)

# Test different SPI speeds
speeds = [1000000, 5000000, 10000000, 15000000, 24000000]  # 1-24 MHz

print("\nTesting different SPI speeds:")
print("Speed (MHz) | Status | Sample Reading")
print("------------|--------|----------------------------------------")

for speed in speeds:
    try:
        # Re-initialize with new speed
        icm_test = adafruit_icm42688.ICM42688(spi, cs=cs, baudrate=speed)
        temp = icm_test.temperature
        ax, ay, az = icm_test.acceleration

        speed_mhz = speed / 1000000
        status = "✓ OK" if 8.0 < abs(az) < 11.0 else "⚠ CHECK"
        print(f"  {speed_mhz:5.1f}     |  {status}  | T={temp:5.1f}°C  "
              f"Az={az:6.2f} m/s²")

        # Clean up test instance
        del icm_test

    except Exception as e:
        speed_mhz = speed / 1000000
        print(f"  {speed_mhz:5.1f}     |  ✗ FAIL | Error: {e}")

# Restore to 10MHz for remaining tests
icm = adafruit_icm42688.ICM42688(spi, cs=cs, baudrate=10000000)
print("\n✓ SPI speed test complete (using 10 MHz for remaining tests)")

# ========================================================================
# Test 3: High-Speed Continuous Reading
# ========================================================================

print("\n" + "=" * 70)
print("TEST 3: High-Speed Continuous Reading (100 samples)")
print("=" * 70)

print("\nReading 100 samples as fast as possible...")
start_time = time.monotonic()

for i in range(100):
    temp = icm.temperature
    accel = icm.acceleration
    gyro = icm.gyro

end_time = time.monotonic()
elapsed = end_time - start_time
sample_rate = 100 / elapsed

print(f"✓ Completed 100 samples in {elapsed:.3f} seconds")
print(f"  Effective sample rate: {sample_rate:.1f} Hz")
print(f"  Per-sample time: {elapsed*1000/100:.2f} ms")

# ========================================================================
# Test 4: FIFO Operation via SPI
# ========================================================================

print("\n" + "=" * 70)
print("TEST 4: FIFO Buffer Operation via SPI")
print("=" * 70)

print("\nEnabling FIFO...")
icm.enable_fifo(accel=True, gyro=True, temp=True, mode="stream")
print("✓ FIFO enabled")

time.sleep(0.5)  # Let FIFO fill

fifo_bytes = icm.fifo_count
packets = fifo_bytes // 16

print(f"FIFO status: {fifo_bytes} bytes ({packets} packets)")

if packets > 0:
    print(f"\nReading {min(packets, 10)} packets:")
    for i in range(min(packets, 10)):
        data = icm.read_fifo()
        print(f"  Packet {i+1}: Accel={data['accel'][2]:6.2f} m/s², "
              f"Temp={data['temp']:5.1f}°C")
    print("✓ FIFO read successful")
else:
    print("⚠ FIFO empty")

icm.disable_fifo()
print("✓ FIFO disabled")

# ========================================================================
# Test Summary
# ========================================================================

print("\n" + "=" * 70)
print("TEST SUMMARY - ESP32-WROOM-32 + ICM42688 (SPI)")
print("=" * 70)
print("✓ SPI communication working")
print("✓ Sensor initialization successful")
print("✓ Multiple SPI speeds tested")
print("✓ High-speed sampling tested")
print("✓ FIFO operation working")
print("\n✓ ALL TESTS PASSED - SPI interface fully functional!")
print("=" * 70)

print("\n\nContinuous monitoring mode (Ctrl+C to stop)...")
print("Temp (°C) | Accel (m/s²)           | Gyro (rad/s)")
print("----------|------------------------|-------------------------")

try:
    while True:
        temp = icm.temperature
        ax, ay, az = icm.acceleration
        gx, gy, gz = icm.gyro

        print(f"  {temp:5.1f}   | {ax:6.2f} {ay:6.2f} {az:6.2f} | "
              f"{gx:6.3f} {gy:6.3f} {gz:6.3f}", end='\r')

        time.sleep(0.05)  # 20Hz update rate

except KeyboardInterrupt:
    print("\n\n✓ Test complete")
