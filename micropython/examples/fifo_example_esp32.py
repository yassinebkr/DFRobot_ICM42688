# SPDX-FileCopyrightText: 2025 Yassine Benkhira
# SPDX-License-Identifier: MIT

"""
FIFO example for ICM42688 on ESP32.

This example demonstrates how to use the FIFO buffer to efficiently read
sensor data in batches. This is useful for:
- Reducing CPU wake-ups in battery-powered applications
- High-frequency data logging without losing samples
- Buffering data during processing tasks

The ICM42688 has a 2KB FIFO that can store up to 128 packets (16 bytes each).

Hardware Setup: Same as basic_i2c_esp32.py
"""

import time
from machine import I2C, Pin
import sys
import gc

# Add icm42688 module to path if not installed
sys.path.append('/home/user/DFRobot_ICM42688/micropython')

import icm42688
from icm42688 import registers as reg

# ========================================================================
# Configuration
# ========================================================================

I2C_SDA_PIN = 21
I2C_SCL_PIN = 22
I2C_FREQ = 400000
ICM_ADDRESS = 0x69

print("=" * 70)
print("ICM42688 FIFO Example - ESP32")
print("=" * 70)

# ========================================================================
# Initialize I2C and sensor
# ========================================================================

try:
    i2c = I2C(0, scl=Pin(I2C_SCL_PIN), sda=Pin(I2C_SDA_PIN), freq=I2C_FREQ)
    print(f"I2C initialized on SDA=GPIO{I2C_SDA_PIN}, SCL=GPIO{I2C_SCL_PIN}")

    icm = icm42688.ICM42688(i2c, address=ICM_ADDRESS)
    print("ICM42688 sensor initialized successfully!")

except Exception as e:
    print(f"ERROR: {e}")
    sys.exit(1)

# ========================================================================
# Configure sensor for FIFO operation
# ========================================================================

print("\nConfiguring sensor for FIFO...")

# Set ranges and data rates
icm.accelerometer_range = reg.ACCEL_RANGE_16G
icm.gyro_range = reg.GYRO_RANGE_2000_DPS
icm.accelerometer_data_rate = reg.ODR_1KHZ  # 1kHz sample rate
icm.gyro_data_rate = reg.ODR_1KHZ

# Enable sensors in low-noise mode
icm.set_power_mode(accel_mode=reg.ACCEL_MODE_LN, gyro_mode=reg.GYRO_MODE_LN)

# Enable FIFO with accelerometer, gyroscope, and temperature
# Mode options:
#   "stream": Overwrites old data when full (default) - good for continuous logging
#   "stop": Stops collecting when full - good for burst capture
icm.enable_fifo(accel=True, gyro=True, temp=True, high_res=False, mode="stream")

print("FIFO Configuration:")
print("  Sensors: Accelerometer + Gyroscope + Temperature")
print("  Mode: Stream (overwrites when full)")
print("  Packet size: 16 bytes")
print("  Max packets: ~128 (2048 bytes / 16)")
print("  Sample rate: 1000 Hz")
print("  Fill rate: ~16 KB/s (16 bytes × 1000 Hz)")
print()
print("At 1kHz, FIFO fills completely in ~128ms")
print("Reading every 100ms to prevent overflow")
print()

# Show initial memory status
print(f"Free memory: {gc.mem_free()} bytes")
print()

print("=" * 70)
print("Collecting data from FIFO (press Ctrl+C to stop)")
print("=" * 70)

# ========================================================================
# Main loop - read FIFO data in batches
# ========================================================================

packet_count = 0
total_bytes_read = 0
loop_count = 0
start_time = time.ticks_ms()

try:
    while True:
        loop_count += 1

        # Check how many bytes are in the FIFO
        fifo_bytes = icm.fifo_count

        # Each packet is 16 bytes, so calculate number of complete packets
        packets_available = fifo_bytes // 16

        if packets_available > 0:
            print(f"\n--- Loop #{loop_count} ---")
            print(f"FIFO: {fifo_bytes} bytes ({packets_available} packets available)")

            # Track time to read all packets
            read_start = time.ticks_ms()

            # Read all available packets
            packets_read = 0
            for i in range(packets_available):
                try:
                    # Read one packet from FIFO
                    data = icm.read_fifo()

                    packet_count += 1
                    packets_read += 1
                    total_bytes_read += 16

                    # Extract sensor data from packet
                    accel_x, accel_y, accel_z = data['accel']  # m/s²
                    gyro_x, gyro_y, gyro_z = data['gyro']  # rad/s
                    temp = data['temp']  # °C

                    # Display every 10th packet to avoid screen spam
                    if packet_count % 10 == 0:
                        print(f"\n  Packet #{packet_count}:")
                        print(f"    Temp:  {temp:6.2f} °C")
                        print(f"    Accel: X={accel_x:7.2f} Y={accel_y:7.2f} Z={accel_z:7.2f} m/s²")

                        # Convert gyro to degrees/second for display
                        gyro_x_dps = gyro_x * 57.2958
                        gyro_y_dps = gyro_y * 57.2958
                        gyro_z_dps = gyro_z * 57.2958
                        print(f"    Gyro:  X={gyro_x_dps:7.2f} Y={gyro_y_dps:7.2f} Z={gyro_z_dps:7.2f} °/s")

                except Exception as e:
                    print(f"  ERROR reading packet {i+1}: {e}")
                    icm.flush_fifo()  # Clear FIFO on error
                    break

            # Calculate read performance
            read_time = time.ticks_diff(time.ticks_ms(), read_start)
            print(f"\n  Read {packets_read} packets in {read_time} ms")
            if read_time > 0:
                print(f"  Read rate: {packets_read*1000/read_time:.1f} packets/sec")

        # Show statistics every 10 loops
        if loop_count % 10 == 0:
            elapsed = time.ticks_diff(time.ticks_ms(), start_time) / 1000
            print(f"\n=== Statistics (after {elapsed:.1f}s) ===")
            print(f"Total packets: {packet_count}")
            print(f"Total bytes: {total_bytes_read}")
            print(f"Average rate: {packet_count/elapsed:.1f} packets/sec")
            print(f"Free memory: {gc.mem_free()} bytes")

        # Wait before checking FIFO again
        # At 1kHz ODR, FIFO accumulates 16 packets every 16ms
        # Reading every 100ms allows batching without overflow
        time.sleep(0.1)  # 100ms

except KeyboardInterrupt:
    print("\n\nStopped by user")

    # Print final statistics
    elapsed = time.ticks_diff(time.ticks_ms(), start_time) / 1000
    print("\n" + "=" * 70)
    print("FINAL STATISTICS")
    print("=" * 70)
    print(f"Run time: {elapsed:.1f} seconds")
    print(f"Total packets collected: {packet_count}")
    print(f"Total bytes read: {total_bytes_read}")
    print(f"Average packet rate: {packet_count/elapsed:.1f} packets/sec")
    print(f"Average data rate: {total_bytes_read/elapsed:.1f} bytes/sec")
    print(f"Free memory: {gc.mem_free()} bytes")

except Exception as e:
    print(f"\nERROR during operation: {e}")
    import sys
    sys.print_exception(e)

finally:
    # Disable FIFO on exit
    try:
        icm.disable_fifo()
        print("\nFIFO disabled")
    except:
        pass

print("\nExample finished.")

# ========================================================================
# Additional FIFO operations (for reference)
# ========================================================================

# To manually flush (clear) the FIFO:
# icm.flush_fifo()

# To disable FIFO:
# icm.disable_fifo()

# To check FIFO count without reading:
# count = icm.fifo_count
# print(f"FIFO has {count} bytes")

# FIFO overflow handling:
# If you read slower than the sensor fills the FIFO, it will overflow
# In "stream" mode, old data is overwritten (no error)
# In "stop" mode, new data is discarded when full
# Best practice: Read FIFO faster than it fills (at 1kHz, read every <128ms)
