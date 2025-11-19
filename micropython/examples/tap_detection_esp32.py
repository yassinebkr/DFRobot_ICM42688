# SPDX-FileCopyrightText: 2025 Yassine Bekkari
# SPDX-License-Identifier: MIT

"""
Tap detection example for ICM42688 on ESP32.

This example demonstrates how to use the tap detection feature to detect
single and double taps on the sensor. This is useful for:
- User interface interactions (tap to wake, double-tap to activate)
- Impact detection
- Gesture recognition

The tap detector can distinguish between single and double taps and
identify which axis the tap occurred on.

Hardware Setup: Same as wake_on_motion_esp32.py
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

print("=" * 70)
print("ICM42688 Tap Detection Example - ESP32")
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
# Configure interrupt pin
# ========================================================================

icm.configure_interrupt(
    pin=1,  # Use INT1
    polarity="high",
    mode="latch",
    drive="push-pull"
)

print("\nInterrupt Configuration:")
print("  Pin: INT1 (active high, latched, push-pull)")

# ========================================================================
# Configure tap detection
# ========================================================================

# Enable tap detection
# Mode options:
#   "low-noise": 1kHz sampling, better sensitivity (higher power)
#   "low-power": 500Hz sampling, lower power consumption

icm.enable_tap_detection(
    mode="low-noise",  # Use low-noise mode for better sensitivity
    int_pin=1  # Route interrupt to INT1
)

print("\nTap Detection Configuration:")
print("  Mode: Low-noise (1kHz sampling)")
print("  Sensitivity: High")
print("  Detection: Single and double taps")
print("  Axis detection: Enabled")

print("\n" + "=" * 70)
print("Monitoring for taps... (press Ctrl+C to stop)")
print("Try tapping the sensor gently on different sides!")
print("=" * 70)
print()

# ========================================================================
# Main loop - monitor for tap events
# ========================================================================

tap_count = 0
single_tap_count = 0
double_tap_count = 0
last_tap_time = time.ticks_ms()

# Track taps per axis
axis_counts = {'x': 0, 'y': 0, 'z': 0}

try:
    while True:
        # Read interrupt status
        status = icm.read_interrupt_status()

        # Check if tap was detected
        if status['tap']:
            tap_count += 1
            current_time = time.ticks_ms()
            time_since_last = time.ticks_diff(current_time, last_tap_time) / 1000
            last_tap_time = current_time

            # Read detailed tap information
            tap_info = icm.read_tap_info()

            # Count tap types
            if tap_info['count'] == 'single':
                single_tap_count += 1
            else:
                double_tap_count += 1

            # Count taps per axis
            axis_counts[tap_info['axis']] += 1

            # Display tap information
            print(f"\n{'='*70}")
            print(f"TAP #{tap_count} DETECTED!")
            print(f"{'='*70}")
            print(f"Type: {tap_info['count'].upper()} TAP")
            print(f"Axis: {tap_info['axis'].upper()}-axis")
            print(f"Direction: {tap_info['direction']}")
            print(f"Time since last tap: {time_since_last:.3f}s")

            # Display running statistics
            print(f"\nRunning Statistics:")
            print(f"  Total taps: {tap_count}")
            print(f"  Single taps: {single_tap_count}")
            print(f"  Double taps: {double_tap_count}")
            print(f"  X-axis: {axis_counts['x']}, Y-axis: {axis_counts['y']}, Z-axis: {axis_counts['z']}")

            # Read current acceleration to show sensor orientation
            try:
                accel_x, accel_y, accel_z = icm.acceleration
                print(f"\nCurrent orientation (gravity vector):")
                print(f"  X: {accel_x/9.81:+6.2f} g")
                print(f"  Y: {accel_y/9.81:+6.2f} g")
                print(f"  Z: {accel_z/9.81:+6.2f} g")
            except:
                pass

            print()

            # In a real application, you could trigger different actions:
            # - Single tap: Wake display
            # - Double tap: Activate feature
            # - Tap on specific axis: Context-aware actions

        # Poll at reasonable rate
        time.sleep(0.05)  # 50ms poll interval (20 Hz)

except KeyboardInterrupt:
    print("\n\nStopped by user")
    print("\n" + "=" * 70)
    print("FINAL STATISTICS")
    print("=" * 70)
    print(f"Total taps detected: {tap_count}")
    print(f"  Single taps: {single_tap_count} ({100*single_tap_count/max(tap_count,1):.1f}%)")
    print(f"  Double taps: {double_tap_count} ({100*double_tap_count/max(tap_count,1):.1f}%)")
    print(f"\nTaps by axis:")
    print(f"  X-axis: {axis_counts['x']} ({100*axis_counts['x']/max(tap_count,1):.1f}%)")
    print(f"  Y-axis: {axis_counts['y']} ({100*axis_counts['y']/max(tap_count,1):.1f}%)")
    print(f"  Z-axis: {axis_counts['z']} ({100*axis_counts['z']/max(tap_count,1):.1f}%)")

except Exception as e:
    print(f"\nERROR during operation: {e}")
    import sys
    sys.print_exception(e)

finally:
    # Disable motion detection on exit
    try:
        icm.disable_motion_detection()
        print("\nTap detection disabled")
    except:
        pass

print("\nExample finished.")

# ========================================================================
# Tap Detection Tips
# ========================================================================

# For best tap detection results:
#
# 1. Mounting:
#    - Mount sensor firmly to the device
#    - Avoid flexible mounting that dampens vibrations
#    - Orient sensor so taps align with X, Y, or Z axis
#
# 2. Sensitivity tuning:
#    - Use "low-noise" mode for better sensitivity
#    - Adjust thresholds in APEX_CONFIG7/8 if needed
#    - Default settings work well for most applications
#
# 3. Double-tap timing:
#    - Double-tap requires two taps within ~600ms
#    - Too slow = two single taps
#    - Too fast = may be detected as single tap
#
# 4. False positive reduction:
#    - Mount sensor away from sources of vibration
#    - Use higher thresholds in noisy environments
#    - Filter taps based on axis if needed
#
# 5. Power consumption:
#    - Low-noise mode: ~350µA
#    - Low-power mode: ~5µA
#    - Choose based on your power budget
