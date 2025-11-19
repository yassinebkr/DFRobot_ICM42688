# SPDX-FileCopyrightText: 2025 Yassine Bekkari
# SPDX-License-Identifier: MIT

"""
Wake-on-motion detection example for ICM42688 on ESP32.

This example demonstrates how to use the wake-on-motion (WOM) feature to detect
when the sensor experiences motion above a configurable threshold. This is ideal
for low-power applications where you want to wake the system only when motion
is detected.

The WOM detector can:
- Monitor motion on X, Y, Z axes independently or together
- Trigger an interrupt when motion exceeds threshold
- Operate in low-power mode (only ~5µA in standby)

Hardware Setup:
-----------------------------------------
ICM42688        ESP32
---------       -----
VCC      <-->   3.3V
GND      <-->   GND
SDA      <-->   GPIO21 (I2C SDA)
SCL      <-->   GPIO22 (I2C SCL)
INT1     <-->   GPIO4  (or any GPIO - optional for interrupt-driven wake)

Note: The INT1 pin connection is optional for this polled example.
For ultra-low-power applications, connect INT1 to wake from deep sleep.
"""

import time
from machine import I2C, Pin
import sys

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

# Wake-on-motion threshold: 0-255, where each LSB ≈ 3.9mg
# threshold = 50 means ~195mg of acceleration will trigger WOM
# Adjust based on your application:
#   - Lower values (10-30): Very sensitive, detects small movements
#   - Medium values (30-80): Normal sensitivity for device pick-up
#   - Higher values (80-150): Only detects significant motion
WOM_THRESHOLD = 50  # ~195mg

print("=" * 70)
print("ICM42688 Wake-on-Motion Detection - ESP32")
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
# Configure interrupt pin (optional - for interrupt-driven wake)
# ========================================================================

# Configure INT1 pin for WOM interrupts
# For polled operation (this example), this is optional
# For deep sleep wake, you would attach an IRQ handler to this pin

icm.configure_interrupt(
    pin=1,  # Use INT1
    polarity="high",  # Active high
    mode="latch",  # Latched (stays high until status read)
    drive="push-pull"  # Push-pull output
)

print("\nInterrupt Configuration:")
print("  Pin: INT1")
print("  Polarity: Active high")
print("  Mode: Latched (stays high until interrupt cleared)")
print("  Drive: Push-pull")

# ========================================================================
# Configure wake-on-motion detection
# ========================================================================

# Enable WOM on all axes
# Axes options: "x", "y", "z", or "all"
# When using "all", motion on ANY axis triggers the interrupt

icm.enable_wake_on_motion(
    threshold=WOM_THRESHOLD,
    axes="all",  # Monitor all three axes
    int_pin=1  # Route interrupt to INT1
)

print("\nWake-on-Motion Configuration:")
print(f"  Threshold: {WOM_THRESHOLD} (~{WOM_THRESHOLD * 3.9:.1f} mg)")
print(f"  Axes: All (X, Y, Z)")
print(f"  Mode: Low-power (50Hz)")
print(f"  Comparison: Motion vs. previous sample")
print(f"  Power consumption: ~5µA in standby")

print("\n" + "=" * 70)
print("Monitoring for motion... (press Ctrl+C to stop)")
print("Try moving, tapping, or rotating the sensor!")
print("=" * 70)
print()

# ========================================================================
# Main loop - monitor for wake-on-motion events
# ========================================================================

motion_count = 0
last_motion_time = time.ticks_ms()

try:
    while True:
        # Read interrupt status (this clears the latched interrupt)
        status = icm.read_interrupt_status()

        # Check which axes detected motion
        if status['wom_x'] or status['wom_y'] or status['wom_z']:
            motion_count += 1
            current_time = time.ticks_ms()
            time_since_last = time.ticks_diff(current_time, last_motion_time) / 1000
            last_motion_time = current_time

            # Display motion information
            print(f"\n{'='*70}")
            print(f"MOTION #{motion_count} DETECTED!")
            print(f"{'='*70}")
            print(f"Time since last motion: {time_since_last:.2f}s")

            # Show which axes detected motion
            axes_detected = []
            if status['wom_x']:
                axes_detected.append('X')
            if status['wom_y']:
                axes_detected.append('Y')
            if status['wom_z']:
                axes_detected.append('Z')

            print(f"Axes triggered: {', '.join(axes_detected)}")

            # Read current acceleration to see motion magnitude
            # Note: Sensor is in LP mode, so readings may be noisier
            try:
                accel_x, accel_y, accel_z = icm.acceleration
                print(f"Current acceleration:")
                print(f"  X: {accel_x:7.2f} m/s² ({accel_x/9.81:6.3f} g)")
                print(f"  Y: {accel_y:7.2f} m/s² ({accel_y/9.81:6.3f} g)")
                print(f"  Z: {accel_z:7.2f} m/s² ({accel_z/9.81:6.3f} g)")

                # Calculate total acceleration magnitude
                accel_mag = (accel_x**2 + accel_y**2 + accel_z**2)**0.5
                print(f"  Magnitude: {accel_mag:7.2f} m/s² ({accel_mag/9.81:6.3f} g)")
            except Exception as e:
                print(f"  (Could not read acceleration: {e})")

            print()

            # In a real application, you would trigger your wake action here:
            # - Turn on display
            # - Start data logging
            # - Send network packet
            # - Wake from deep sleep
            # - etc.

        # Poll at reasonable rate
        # Note: In ultra-low-power applications, you would use deep sleep
        # and wake on the physical INT pin instead of polling
        time.sleep(0.1)  # 100ms poll interval

except KeyboardInterrupt:
    print("\n\nStopped by user")
    print(f"Total motion events detected: {motion_count}")

except Exception as e:
    print(f"\nERROR during operation: {e}")
    import sys
    sys.print_exception(e)

finally:
    # Disable motion detection on exit
    try:
        icm.disable_motion_detection()
        print("\nMotion detection disabled")
    except:
        pass

print("\nExample finished.")

# ========================================================================
# Power Optimization Notes
# ========================================================================

# For maximum power savings in production applications:
#
# 1. Use the INT pin to wake from deep sleep instead of polling:
#    - Connect INT1 to a wake-capable GPIO (e.g., GPIO4)
#    - Configure the interrupt as shown above
#    - Enter deep sleep with: machine.deepsleep()
#    - The sensor will wake the ESP32 when motion is detected
#    - Power consumption: <10µA (ESP32 deep sleep + ICM42688 WOM mode)
#
# 2. The sensor automatically uses low-power mode when WOM is enabled:
#    - Accelerometer: Low-power mode at 50Hz
#    - Gyroscope: Disabled to save power
#    - Current consumption: ~5µA
#
# 3. Adjust threshold to reduce false wake-ups:
#    - Higher threshold = fewer wake-ups = lower average power
#    - Test threshold based on your application needs
#
# 4. Use axis selection to reduce sensitivity:
#    - Monitor only relevant axes (e.g., Z-axis for vertical motion)
#    - Use axes="z" instead of axes="all"
#
# Example deep sleep code (commented out):
#
# from machine import deepsleep
# from machine import Pin
#
# # Configure wake pin
# wake_pin = Pin(4, Pin.IN, Pin.PULL_DOWN)
#
# print("Entering deep sleep. Will wake on motion...")
# time.sleep(0.1)
#
# # Enter deep sleep - will wake when INT1 goes high
# deepsleep()
#
# # After wake, the ESP32 will restart from the beginning
# # Check reset cause to determine if woken by external interrupt
