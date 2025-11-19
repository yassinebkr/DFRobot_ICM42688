# SPDX-FileCopyrightText: 2025 Yassine Bekkari
# SPDX-License-Identifier: MIT

"""
Basic I2C example for ICM42688 on ESP32.

This example demonstrates basic usage of the ICM42688 sensor to read
accelerometer, gyroscope, and temperature data using I2C communication.

Hardware Setup (ESP32 WROOM / Lolin D32):
-----------------------------------------
ICM42688        ESP32
---------       -----
VCC      <-->   3.3V
GND      <-->   GND
SDA      <-->   GPIO21 (I2C SDA)
SCL      <-->   GPIO22 (I2C SCL)
SDO      <-->   3.3V (for address 0x69) or GND (for address 0x68)

Note: ESP32 has built-in pull-ups on I2C pins, but external 4.7k pull-ups
are recommended for reliable operation, especially with longer wires.
"""

import time
from machine import I2C, Pin
import sys

# Add icm42688 module to path if not installed
sys.path.append('/home/user/DFRobot_ICM42688/micropython')

import icm42688
from icm42688 import registers as reg

# ========================================================================
# I2C Configuration for ESP32
# ========================================================================

# ESP32 WROOM and Lolin D32 default I2C pins:
# - GPIO21: SDA (data line)
# - GPIO22: SCL (clock line)

I2C_SDA_PIN = 21
I2C_SCL_PIN = 22
I2C_FREQ = 400000  # 400kHz (fast mode)

# ICM42688 I2C address:
# - 0x69 (default): SDO pin pulled HIGH or floating
# - 0x68: SDO pin pulled LOW
ICM_ADDRESS = 0x69

print("=" * 60)
print("ICM42688 Basic I2C Example - ESP32")
print("=" * 60)

# Initialize I2C bus
try:
    i2c = I2C(0, scl=Pin(I2C_SCL_PIN), sda=Pin(I2C_SDA_PIN), freq=I2C_FREQ)
    print(f"I2C initialized on SDA=GPIO{I2C_SDA_PIN}, SCL=GPIO{I2C_SCL_PIN}")

    # Scan I2C bus to verify sensor is connected
    devices = i2c.scan()
    print(f"I2C devices found: {[hex(d) for d in devices]}")

    if ICM_ADDRESS not in devices:
        print(f"WARNING: ICM42688 not found at address {hex(ICM_ADDRESS)}")
        print("Check wiring and I2C address (SDO pin state)")

except Exception as e:
    print(f"ERROR: Failed to initialize I2C: {e}")
    sys.exit(1)

# ========================================================================
# Initialize ICM42688 sensor
# ========================================================================

try:
    icm = icm42688.ICM42688(i2c, address=ICM_ADDRESS)
    print("ICM42688 sensor initialized successfully!")
except Exception as e:
    print(f"ERROR: Failed to initialize ICM42688: {e}")
    sys.exit(1)

# ========================================================================
# Configure sensor settings
# ========================================================================

print("\nConfiguring sensor...")

# Set accelerometer range
# Options: ACCEL_RANGE_2G, ACCEL_RANGE_4G, ACCEL_RANGE_8G, ACCEL_RANGE_16G
icm.accelerometer_range = reg.ACCEL_RANGE_16G
print(f"  Accelerometer range: ±16g")

# Set gyroscope range
# Options: GYRO_RANGE_2000_DPS, GYRO_RANGE_1000_DPS, GYRO_RANGE_500_DPS, etc.
icm.gyro_range = reg.GYRO_RANGE_2000_DPS
print(f"  Gyroscope range: ±2000 dps")

# Set output data rates (ODR)
# Options: ODR_1KHZ, ODR_200HZ, ODR_100HZ, ODR_50HZ, etc.
icm.accelerometer_data_rate = reg.ODR_1KHZ
icm.gyro_data_rate = reg.ODR_1KHZ
print(f"  Sample rate: 1000 Hz")

# Set power modes for high performance
# Accel modes: ACCEL_MODE_OFF, ACCEL_MODE_LP (low-power), ACCEL_MODE_LN (low-noise)
# Gyro modes: GYRO_MODE_OFF, GYRO_MODE_STANDBY, GYRO_MODE_LN (low-noise)
icm.set_power_mode(accel_mode=reg.ACCEL_MODE_LN, gyro_mode=reg.GYRO_MODE_LN)
print(f"  Power mode: Low-noise (high performance)")

print("\n" + "=" * 60)
print("Reading sensor data (press Ctrl+C to stop)")
print("=" * 60)

# ========================================================================
# Main loop - read and display sensor data
# ========================================================================

try:
    sample_count = 0

    while True:
        # Read all sensor data
        accel_x, accel_y, accel_z = icm.acceleration  # m/s²
        gyro_x, gyro_y, gyro_z = icm.gyro  # rad/s
        temp = icm.temperature  # °C

        sample_count += 1

        # Display data
        print(f"\n--- Sample #{sample_count} ---")
        print(f"Temperature:  {temp:6.2f} °C")
        print(f"Acceleration: X={accel_x:7.2f} Y={accel_y:7.2f} Z={accel_z:7.2f} m/s²")
        print(f"Gyroscope:    X={gyro_x:7.3f} Y={gyro_y:7.3f} Z={gyro_z:7.3f} rad/s")

        # Convert gyro to degrees per second for easier interpretation
        gyro_x_dps = gyro_x * 57.2958  # rad/s to deg/s
        gyro_y_dps = gyro_y * 57.2958
        gyro_z_dps = gyro_z * 57.2958
        print(f"Gyroscope:    X={gyro_x_dps:7.2f} Y={gyro_y_dps:7.2f} Z={gyro_z_dps:7.2f} °/s")

        # Calculate magnitude of acceleration (including gravity)
        accel_mag = (accel_x**2 + accel_y**2 + accel_z**2)**0.5
        print(f"Accel magnitude: {accel_mag:7.2f} m/s² ({accel_mag/9.81:5.2f} g)")

        time.sleep(1.0)  # Read at 1 Hz

except KeyboardInterrupt:
    print("\n\nStopped by user")
    print(f"Total samples collected: {sample_count}")

except Exception as e:
    print(f"\nERROR during operation: {e}")
    import sys
    sys.print_exception(e)

print("\nExample finished.")
