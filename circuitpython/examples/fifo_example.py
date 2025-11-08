# SPDX-FileCopyrightText: 2025 Yassine Bekkari
# SPDX-License-Identifier: MIT

"""
FIFO example for ICM42688.

This example demonstrates how to use the FIFO buffer to efficiently read
sensor data in batches. This is useful for reducing CPU wake-ups and power
consumption in battery-powered applications.

The FIFO can store up to 2KB of sensor data (up to 128 packets of 16 bytes each).

Hardware setup:
- Connect ICM42688 to I2C or SPI on your Feather RP2040
"""

import time
import board
import adafruit_icm42688
from adafruit_icm42688 import registers as reg

# ========================================================================
# Setup sensor (using I2C)
# ========================================================================

i2c = board.I2C()
icm = adafruit_icm42688.ICM42688(i2c, address=0x69)

print("ICM42688 FIFO Example")
print("=" * 70)

# ========================================================================
# Configure sensor for FIFO operation
# ========================================================================

# Set ranges and data rates
icm.accelerometer_range = reg.ACCEL_RANGE_16G
icm.gyro_range = reg.GYRO_RANGE_2000_DPS
icm.accelerometer_data_rate = reg.ODR_1KHZ  # 1kHz sample rate
icm.gyro_data_rate = reg.ODR_1KHZ

# Enable sensors in low-noise mode
icm.set_power_mode(accel_mode=reg.ACCEL_MODE_LN, gyro_mode=reg.GYRO_MODE_LN)

# Enable FIFO with accelerometer, gyroscope, and temperature
# Mode options:
#   "stream": Overwrites old data when full (default)
#   "stop": Stops collecting when full
icm.enable_fifo(accel=True, gyro=True, temp=True, high_res=False, mode="stream")

print("FIFO Configuration:")
print("  Sensors: Accelerometer + Gyroscope + Temperature")
print("  Mode: Stream (overwrites when full)")
print("  Packet size: 16 bytes")
print("  Max packets: ~128 (2048 bytes / 16)")
print("  Sample rate: 1000 Hz")
print()
print("Collecting data from FIFO (press Ctrl+C to stop)...")
print("=" * 70)

# ========================================================================
# Main loop - read FIFO data in batches
# ========================================================================

packet_count = 0

while True:
    # Check how many bytes are in the FIFO
    fifo_bytes = icm.fifo_count

    # Each packet is 16 bytes, so calculate number of complete packets
    packets_available = fifo_bytes // 16

    if packets_available > 0:
        print(f"\nFIFO has {fifo_bytes} bytes ({packets_available} packets)")

        # Read all available packets
        for i in range(packets_available):
            try:
                # Read one packet from FIFO
                data = icm.read_fifo()

                packet_count += 1

                # Extract sensor data from packet
                accel_x, accel_y, accel_z = data['accel']  # m/s²
                gyro_x, gyro_y, gyro_z = data['gyro']  # rad/s
                temp = data['temp']  # °C

                # Display every 10th packet to avoid screen spam
                if packet_count % 10 == 0:
                    print(f"Packet #{packet_count:4d}:")
                    print(f"  Temp: {temp:6.2f} °C")
                    print(f"  Accel: X={accel_x:7.2f} Y={accel_y:7.2f} Z={accel_z:7.2f} m/s²")

                    # Convert gyro to degrees/second for display
                    gyro_x_dps = gyro_x * (180.0 / 3.14159)
                    gyro_y_dps = gyro_y * (180.0 / 3.14159)
                    gyro_z_dps = gyro_z * (180.0 / 3.14159)
                    print(f"  Gyro:  X={gyro_x_dps:7.2f} Y={gyro_y_dps:7.2f} Z={gyro_z_dps:7.2f} °/s")

            except Exception as e:
                print(f"Error reading FIFO: {e}")
                icm.flush_fifo()  # Clear FIFO on error
                break

    # Wait a bit before checking FIFO again
    # At 1kHz ODR, FIFO fills with 16 packets every 16ms
    time.sleep(0.02)  # Check every 20ms

# ========================================================================
# Additional FIFO operations (for reference)
# ========================================================================

# To flush (clear) the FIFO:
# icm.flush_fifo()

# To disable FIFO:
# icm.disable_fifo()

# To check FIFO count:
# count = icm.fifo_count
# print(f"FIFO has {count} bytes")
