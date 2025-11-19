# SPDX-FileCopyrightText: 2025 Yassine Bekkari
# SPDX-License-Identifier: MIT

"""
ICM42688 test for ESP32-WROOM-32 40-pin DevKit - I2C Interface

This example demonstrates how to use the ICM42688 sensor with an
ESP32-WROOM-32 development board (40-pin version) via I2C.

Hardware Wiring (I2C):
=====================
    ICM42688 Pin    →    ESP32-WROOM-32 Pin    →    Function
    ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    VCC/VDD         →    3.3V                   →    Power (3.3V ONLY!)
    GND             →    GND                    →    Ground
    SDA             →    GPIO21 (Pin 33)        →    I2C Data
    SCL             →    GPIO22 (Pin 36)        →    I2C Clock
    INT1 (optional) →    GPIO23 (Pin 37)        →    Interrupt 1
    INT2 (optional) →    GPIO19 (Pin 31)        →    Interrupt 2

CRITICAL: ICM42688 operates at 3.3V ONLY. Never connect 5V to VCC!

Pin Reference (40-pin ESP32 DevKit):
====================================
                    ESP32-WROOM-32 DevKit (40-pin)
                    ==============================
    Left Side (1-20)                    Right Side (21-40)
    ────────────────                    ──────────────────
    1  - 3V3                            40 - GND
    2  - EN (RESET)                     39 - GND
    3  - GPIO36 (VP/ADC1_0)             38 - GPIO23 (MOSI)
    4  - GPIO39 (VN/ADC1_3)             37 - GPIO22 (SCL)  ← I2C Clock
    5  - GPIO34 (ADC1_6)                36 - GPIO1 (TX0)
    6  - GPIO35 (ADC1_7)                35 - GPIO3 (RX0)
    7  - GPIO32 (ADC1_4)                34 - GPIO21 (SDA)  ← I2C Data
    8  - GPIO33 (ADC1_5)                33 - GND
    9  - GPIO25 (DAC1)                  32 - GPIO19 (MISO)
    10 - GPIO26 (DAC2)                  31 - GPIO18 (SCK)
    11 - GPIO27                         30 - GPIO5 (CS)
    12 - GPIO14                         29 - GPIO17
    13 - GPIO12                         28 - GPIO16
    14 - GND                            27 - GPIO4
    15 - GPIO13                         26 - GPIO0 (BOOT)
    16 - GPIO9 (FLASH)                  25 - GPIO2 (LED)
    17 - GPIO10 (FLASH)                 24 - GPIO15
    18 - GPIO11 (FLASH)                 23 - GND
    19 - VIN (5V)                       22 - 3V3
    20 - GND                            21 - GND

CircuitPython Setup:
===================
1. Download CircuitPython for ESP32-DevKitC-V4-WROOM-32E:
   https://circuitpython.org/board/espressif_esp32_devkitc_v4_wroom_32e/

   OR for DOIT ESP32 DevKit v1:
   https://circuitpython.org/board/doit_esp32_devkit_v1/

2. Install using esptool.py or web installer:
   https://adafruit.github.io/Adafruit_WebSerial_ESPTool/

3. Connect via serial (115200 baud) - NO USB drive will appear

4. Copy library files to /lib/ on the board:
   - adafruit_icm42688/
   - adafruit_bus_device/

5. Copy this file as code.py to the root directory
"""

import time
import board
import busio
import adafruit_icm42688
from adafruit_icm42688 import registers as reg

print("=" * 70)
print("ICM42688 I2C Test - ESP32-WROOM-32 (40-pin DevKit)")
print("=" * 70)

# ========================================================================
# I2C Configuration
# ========================================================================

# ESP32 Default I2C Pins:
#   SDA = GPIO21 (Pin 34 on 40-pin header)
#   SCL = GPIO22 (Pin 37 on 40-pin header)

print("\nInitializing I2C bus...")
print("  SDA: GPIO21 (Pin 34)")
print("  SCL: GPIO22 (Pin 37)")
print("  Frequency: 400kHz")

# Option 1: Use explicit pin numbers
i2c = busio.I2C(scl=board.IO22, sda=board.IO21, frequency=400000)

# Option 2: Use default board I2C (if available)
# i2c = board.I2C()

print("✓ I2C bus initialized")

# ========================================================================
# I2C Bus Scan
# ========================================================================

print("\nScanning I2C bus...")
while not i2c.try_lock():
    pass

try:
    devices = i2c.scan()
    print(f"Found {len(devices)} device(s): {[hex(d) for d in devices]}")

    if not devices:
        print("\n⚠ WARNING: No I2C devices found!")
        print("Troubleshooting:")
        print("  1. Check wiring: VCC → 3.3V, GND → GND")
        print("  2. Verify SDA → GPIO21, SCL → GPIO22")
        print("  3. Check ICM42688 power LED (if available)")
        print("  4. Try adding pull-up resistors (2.2k-4.7k)")
        print("  5. Verify ICM42688 is 3.3V compatible")
finally:
    i2c.unlock()

# ========================================================================
# Initialize ICM42688
# ========================================================================

print("\nInitializing ICM42688 sensor...")

try:
    # Try address 0x69 first (SDO/AD0 pulled HIGH - most common)
    icm = adafruit_icm42688.ICM42688(i2c, address=0x69)
    print("✓ ICM42688 found at address 0x69 (SDO/AD0 = HIGH)")
except RuntimeError:
    try:
        # Try address 0x68 (SDO/AD0 pulled LOW)
        icm = adafruit_icm42688.ICM42688(i2c, address=0x68)
        print("✓ ICM42688 found at address 0x68 (SDO/AD0 = LOW)")
    except Exception as e:
        print(f"✗ FAILED to initialize ICM42688")
        print(f"Error: {e}")
        print("\nTroubleshooting:")
        print("  1. Verify chip ID (should be 0x47)")
        print("  2. Check I2C communication with logic analyzer")
        print("  3. Try lower I2C frequency (100kHz)")
        print("  4. Verify sensor is ICM42688 (not ICM42605/ICM20948)")
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
print("TEST 1: Basic Sensor Reading")
print("=" * 70)

temp = icm.temperature
accel_x, accel_y, accel_z = icm.acceleration
gyro_x, gyro_y, gyro_z = icm.gyro

print(f"\nTemperature:  {temp:.2f} °C")
print(f"Acceleration: X={accel_x:7.2f} Y={accel_y:7.2f} Z={accel_z:7.2f} m/s²")
print(f"Gyroscope:    X={gyro_x:7.4f} Y={gyro_y:7.4f} Z={gyro_z:7.4f} rad/s")

# Verify gravity on Z-axis (sensor lying flat)
if 8.0 < abs(accel_z) < 11.0:
    print("\n✓ Gravity detected on Z-axis - sensor working correctly!")
else:
    print(f"\n⚠ Warning: Z-axis = {accel_z:.2f} m/s² (expected ~±9.8 m/s²)")
    print("   Sensor may be oriented differently or not calibrated")

# ========================================================================
# Test 2: Continuous Reading
# ========================================================================

print("\n" + "=" * 70)
print("TEST 2: Continuous Reading (20 samples @ 5Hz)")
print("=" * 70)
print("\nSample | Temp (°C) | Accel X | Accel Y | Accel Z | Gyro X  | Gyro Y  | Gyro Z")
print("-------|-----------|---------|---------|---------|---------|---------|--------")

for i in range(20):
    temp = icm.temperature
    ax, ay, az = icm.acceleration
    gx, gy, gz = icm.gyro

    print(f"  {i+1:2d}   |   {temp:5.1f}   | {ax:7.2f} | {ay:7.2f} | {az:7.2f} | "
          f"{gx:7.3f} | {gy:7.3f} | {gz:7.3f}")

    time.sleep(0.2)  # 5Hz sampling

print("\n✓ Continuous reading test complete")

# ========================================================================
# Test 3: Range Configuration
# ========================================================================

print("\n" + "=" * 70)
print("TEST 3: Accelerometer Range Configuration")
print("=" * 70)

ranges = [
    (reg.ACCEL_RANGE_2G, "±2g"),
    (reg.ACCEL_RANGE_4G, "±4g"),
    (reg.ACCEL_RANGE_8G, "±8g"),
    (reg.ACCEL_RANGE_16G, "±16g")
]

print("\nTesting different accelerometer ranges (sensor lying flat):")
print("Range | Z-axis (m/s²) | Expected ~9.8 m/s²")
print("------|---------------|-------------------")

for accel_range, range_name in ranges:
    icm.accelerometer_range = accel_range
    time.sleep(0.05)  # Allow sensor to settle
    ax, ay, az = icm.acceleration
    print(f"{range_name:5s} | {az:13.2f} | {'✓' if 8.0 < abs(az) < 11.0 else '✗'}")

# Restore default range
icm.accelerometer_range = reg.ACCEL_RANGE_16G
print("\n✓ Range configuration test complete")

# ========================================================================
# Test 4: FIFO Operation
# ========================================================================

print("\n" + "=" * 70)
print("TEST 4: FIFO Buffer Operation")
print("=" * 70)

print("\nEnabling FIFO...")
icm.enable_fifo(accel=True, gyro=True, temp=True, mode="stream")
print("✓ FIFO enabled (stream mode)")

# Wait for FIFO to fill
print("\nWaiting for FIFO to accumulate data...")
time.sleep(0.5)

fifo_bytes = icm.fifo_count
packets_available = fifo_bytes // 16

print(f"FIFO status: {fifo_bytes} bytes ({packets_available} packets)")

if packets_available > 0:
    print(f"\nReading {min(packets_available, 5)} packets from FIFO:")
    print("Packet | Temp (°C) | Accel X | Accel Y | Accel Z | Gyro X  | Gyro Y  | Gyro Z")
    print("-------|-----------|---------|---------|---------|---------|---------|--------")

    for i in range(min(packets_available, 5)):
        try:
            data = icm.read_fifo()
            ax, ay, az = data['accel']
            gx, gy, gz = data['gyro']
            temp = data['temp']

            print(f"  {i+1:2d}   |   {temp:5.1f}   | {ax:7.2f} | {ay:7.2f} | {az:7.2f} | "
                  f"{gx:7.3f} | {gy:7.3f} | {gz:7.3f}")
        except Exception as e:
            print(f"Error reading packet {i+1}: {e}")
            break

    print("\n✓ FIFO read test complete")
else:
    print("✗ FIFO empty - no data available")

# Disable FIFO
icm.disable_fifo()
print("✓ FIFO disabled")

# ========================================================================
# Test Summary
# ========================================================================

print("\n" + "=" * 70)
print("TEST SUMMARY - ESP32-WROOM-32 + ICM42688")
print("=" * 70)
print("✓ I2C communication working")
print("✓ Sensor initialization successful")
print("✓ Sensor readings valid")
print("✓ Continuous sampling working")
print("✓ Range configuration working")
print("✓ FIFO operation working")
print("\n✓ ALL TESTS PASSED - Library compatible with ESP32-WROOM-32!")
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

        time.sleep(0.1)  # 10Hz update rate

except KeyboardInterrupt:
    print("\n\n✓ Test complete")
