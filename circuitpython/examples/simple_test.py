# SPDX-FileCopyrightText: 2025 Yassine Bekkari
# SPDX-License-Identifier: MIT

"""
Simple test example for ICM42688.

This example demonstrates basic usage of the ICM42688 sensor to read
accelerometer, gyroscope, and temperature data at 1Hz.

Hardware setup:
- Connect ICM42688 to I2C or SPI on your Feather RP2040
- For I2C: Connect SDA, SCL, VCC, GND
- For SPI: Connect MOSI, MISO, SCK, CS, VCC, GND
"""

import time
import board
import adafruit_icm42688
from adafruit_icm42688 import registers as reg

# ========================================================================
# I2C Setup (default)
# ========================================================================
# For I2C, the address depends on SDO pin state:
#   SDO pulled high (default): 0x69
#   SDO pulled low: 0x68

i2c = board.I2C()  # Uses board.SCL and board.SDA
icm = adafruit_icm42688.ICM42688(i2c, address=0x69)

# ========================================================================
# SPI Setup (alternative - uncomment to use)
# ========================================================================
# import digitalio
#
# spi = board.SPI()  # Uses board.SCK, board.MOSI, board.MISO
# cs = digitalio.DigitalInOut(board.D10)  # Change pin as needed
# icm = adafruit_icm42688.ICM42688(spi, cs=cs)

# ========================================================================
# Configure sensor
# ========================================================================

print("ICM42688 Simple Test")
print("=" * 50)

# Set accelerometer range and data rate
# Options: ACCEL_RANGE_2G, ACCEL_RANGE_4G, ACCEL_RANGE_8G, ACCEL_RANGE_16G
icm.accelerometer_range = reg.ACCEL_RANGE_16G

# Set gyroscope range and data rate
# Options: GYRO_RANGE_2000_DPS, GYRO_RANGE_1000_DPS, GYRO_RANGE_500_DPS, etc.
icm.gyro_range = reg.GYRO_RANGE_2000_DPS

# Set output data rates (both accel and gyro)
# Options: ODR_1KHZ, ODR_200HZ, ODR_100HZ, ODR_50HZ, etc.
icm.accelerometer_data_rate = reg.ODR_1KHZ
icm.gyro_data_rate = reg.ODR_1KHZ

# Set power modes for high performance
# Options for accel: ACCEL_MODE_OFF, ACCEL_MODE_LP (low-power), ACCEL_MODE_LN (low-noise)
# Options for gyro: GYRO_MODE_OFF, GYRO_MODE_STANDBY, GYRO_MODE_LN (low-noise)
icm.set_power_mode(accel_mode=reg.ACCEL_MODE_LN, gyro_mode=reg.GYRO_MODE_LN)

print("Configuration:")
print(f"  Accelerometer: ±{[2, 4, 8, 16][icm.accelerometer_range]}g")
print(f"  Gyroscope: ±{[2000, 1000, 500, 250, 125, 62.5, 31.25, 15.625][icm.gyro_range]} dps")
print(f"  Data rate: 1000 Hz")
print(f"  Mode: Low-noise (high performance)")
print()
print("Reading sensor data (press Ctrl+C to stop)...")
print("=" * 50)

# ========================================================================
# Main loop - read and display sensor data
# ========================================================================

while True:
    # Method 1: Read all data individually
    accel_x, accel_y, accel_z = icm.acceleration
    gyro_x, gyro_y, gyro_z = icm.gyro
    temp = icm.temperature

    print("\n" + "=" * 50)
    print(f"Temperature:  {temp:6.2f} °C")
    print(f"Acceleration: X={accel_x:7.2f} Y={accel_y:7.2f} Z={accel_z:7.2f} m/s²")
    print(f"Gyroscope:    X={gyro_x:7.2f} Y={gyro_y:7.2f} Z={gyro_z:7.2f} rad/s")

    # Convert gyro to degrees per second for easier reading
    gyro_x_dps = gyro_x * (180.0 / 3.14159)
    gyro_y_dps = gyro_y * (180.0 / 3.14159)
    gyro_z_dps = gyro_z * (180.0 / 3.14159)
    print(f"Gyroscope:    X={gyro_x_dps:7.2f} Y={gyro_y_dps:7.2f} Z={gyro_z_dps:7.2f} °/s")

    time.sleep(1.0)  # Read at 1 Hz
