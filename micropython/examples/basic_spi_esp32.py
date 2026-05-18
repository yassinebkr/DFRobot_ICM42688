# SPDX-FileCopyrightText: 2025 Yassine Benkhira
# SPDX-License-Identifier: MIT

"""
Basic SPI example for ICM42688 on ESP32.

This example demonstrates basic usage of the ICM42688 sensor to read
accelerometer, gyroscope, and temperature data using SPI communication.

Hardware Setup (ESP32 WROOM / Lolin D32):
-----------------------------------------
ICM42688        ESP32 VSPI
---------       ----------
VCC      <-->   3.3V
GND      <-->   GND
MOSI     <-->   GPIO23 (VSPI MOSI)
MISO     <-->   GPIO19 (VSPI MISO)
SCK      <-->   GPIO18 (VSPI SCK)
CS       <-->   GPIO5  (or any GPIO)

Note: ESP32 has two SPI buses:
- VSPI (SPI3): Default pins GPIO23 (MOSI), GPIO19 (MISO), GPIO18 (SCK)
- HSPI (SPI2): GPIO13 (MOSI), GPIO12 (MISO), GPIO14 (SCK)

This example uses VSPI (bus 2 in MicroPython).
"""

import time
from machine import SPI, Pin
import sys

# Add icm42688 module to path if not installed
sys.path.append('/home/user/DFRobot_ICM42688/micropython')

import icm42688
from icm42688 import registers as reg

# ========================================================================
# SPI Configuration for ESP32
# ========================================================================

# ESP32 VSPI pins (SPI bus 2):
SPI_MOSI_PIN = 23
SPI_MISO_PIN = 19
SPI_SCK_PIN = 18
SPI_CS_PIN = 5

# SPI configuration
SPI_BAUDRATE = 10000000  # 10 MHz (ICM42688 supports up to 24 MHz)
SPI_BUS_ID = 2  # VSPI on ESP32

print("=" * 60)
print("ICM42688 Basic SPI Example - ESP32")
print("=" * 60)

# Initialize SPI bus
try:
    spi = SPI(
        SPI_BUS_ID,
        baudrate=SPI_BAUDRATE,
        polarity=0,  # CPOL = 0
        phase=0,  # CPHA = 0
        sck=Pin(SPI_SCK_PIN),
        mosi=Pin(SPI_MOSI_PIN),
        miso=Pin(SPI_MISO_PIN)
    )
    print(f"SPI initialized:")
    print(f"  Bus: VSPI (ID {SPI_BUS_ID})")
    print(f"  SCK:  GPIO{SPI_SCK_PIN}")
    print(f"  MOSI: GPIO{SPI_MOSI_PIN}")
    print(f"  MISO: GPIO{SPI_MISO_PIN}")
    print(f"  CS:   GPIO{SPI_CS_PIN}")
    print(f"  Baudrate: {SPI_BAUDRATE/1000000:.1f} MHz")

    # Initialize CS pin (chip select)
    cs = Pin(SPI_CS_PIN, Pin.OUT)
    cs.value(1)  # CS idle high
    print(f"CS pin configured (idle HIGH)")

except Exception as e:
    print(f"ERROR: Failed to initialize SPI: {e}")
    sys.exit(1)

# ========================================================================
# Initialize ICM42688 sensor
# ========================================================================

try:
    icm = icm42688.ICM42688(spi, cs=cs)
    print("ICM42688 sensor initialized successfully!")
except Exception as e:
    print(f"ERROR: Failed to initialize ICM42688: {e}")
    sys.exit(1)

# ========================================================================
# Configure sensor settings
# ========================================================================

print("\nConfiguring sensor...")

# Set accelerometer range
icm.accelerometer_range = reg.ACCEL_RANGE_16G
print(f"  Accelerometer range: ±16g")

# Set gyroscope range
icm.gyro_range = reg.GYRO_RANGE_2000_DPS
print(f"  Gyroscope range: ±2000 dps")

# Set output data rates (ODR)
icm.accelerometer_data_rate = reg.ODR_1KHZ
icm.gyro_data_rate = reg.ODR_1KHZ
print(f"  Sample rate: 1000 Hz")

# Set power modes for high performance
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

        # Convert gyro to degrees per second
        gyro_x_dps = gyro_x * 57.2958
        gyro_y_dps = gyro_y * 57.2958
        gyro_z_dps = gyro_z * 57.2958
        print(f"Gyroscope:    X={gyro_x_dps:7.2f} Y={gyro_y_dps:7.2f} Z={gyro_z_dps:7.2f} °/s")

        # Calculate magnitude of acceleration
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

finally:
    # Clean up: set CS high
    cs.value(1)

print("\nExample finished.")
