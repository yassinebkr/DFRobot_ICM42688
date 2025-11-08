# SPDX-FileCopyrightText: 2025 Yassine Bekkari
# SPDX-License-Identifier: MIT

"""
Wake-on-motion detection example for ICM42688.

This example demonstrates how to use the wake-on-motion (WOM) feature to detect
when the sensor experiences motion above a configurable threshold. This is ideal
for low-power applications where you want to wake the system only when motion
is detected.

The WOM detector can:
- Monitor motion on X, Y, Z axes independently or together
- Trigger an interrupt when motion exceeds threshold
- Operate in low-power mode for battery applications

Hardware setup:
- Connect ICM42688 to I2C or SPI on your Feather RP2040
- Optionally connect INT1 or INT2 pin to a GPIO for interrupt-driven wake
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

print("ICM42688 Wake-on-Motion Detection Example")
print("=" * 70)

# ========================================================================
# Configure interrupt pin
# ========================================================================

# Configure INT1 pin for WOM interrupts
icm.configure_interrupt(
    pin=1,  # Use INT1
    polarity="high",
    mode="latch",
    drive="push-pull"
)

print("Interrupt Configuration:")
print("  Pin: INT1")
print("  Polarity: Active high")
print("  Mode: Latched (stays high until interrupt cleared)")
print()

# ========================================================================
# Configure wake-on-motion detection
# ========================================================================

# WOM threshold: 0-255, where each LSB ≈ 3.9mg
# threshold = 50 means ~195mg of acceleration will trigger WOM
# Adjust this based on your application:
#   - Lower values (10-30): Very sensitive, detects small movements
#   - Medium values (30-80): Normal sensitivity for device pick-up
#   - Higher values (80-150): Only detects significant motion

WOM_THRESHOLD = 50  # ~195mg threshold

# Enable WOM on all axes
# Axes options: "x", "y", "z", or "all"
# When using "all", motion on ANY axis triggers the interrupt
icm.enable_wake_on_motion(
    threshold=WOM_THRESHOLD,
    axes="all",  # Monitor all three axes
    int_pin=1  # Route interrupt to INT1
)

print("Wake-on-Motion Configuration:")
print(f"  Threshold: {WOM_THRESHOLD} (~{WOM_THRESHOLD * 3.9:.1f} mg)")
print("  Axes: All (X, Y, Z)")
print("  Mode: Low-power (50Hz)")
print("  Comparison: Motion vs. previous sample")
print()
print("Monitoring for motion... (press Ctrl+C to stop)")
print("Try moving or tapping the sensor!")
print("=" * 70)
print()

# ========================================================================
# Main loop - monitor for wake-on-motion events
# ========================================================================

motion_count = 0
last_motion_time = time.monotonic()

while True:
    # Read interrupt status (this clears the interrupt)
    status = icm.read_interrupt_status()

    # Check which axes detected motion
    if status['wom_x'] or status['wom_y'] or status['wom_z']:
        motion_count += 1
        current_time = time.monotonic()
        time_since_last = current_time - last_motion_time
        last_motion_time = current_time

        # Display motion information
        print(f"MOTION #{motion_count} DETECTED!")
        print(f"  Time since last motion: {time_since_last:.2f}s")

        # Show which axes detected motion
        axes_detected = []
        if status['wom_x']:
            axes_detected.append('X')
        if status['wom_y']:
            axes_detected.append('Y')
        if status['wom_z']:
            axes_detected.append('Z')

        print(f"  Axes: {', '.join(axes_detected)}")

        # Read current acceleration to see motion magnitude
        accel_x, accel_y, accel_z = icm.acceleration
        print(f"  Current acceleration:")
        print(f"    X: {accel_x:7.2f} m/s²")
        print(f"    Y: {accel_y:7.2f} m/s²")
        print(f"    Z: {accel_z:7.2f} m/s²")
        print()

        # In a real application, you would trigger your wake action here
        # For example:
        # - Turn on display
        # - Start data logging
        # - Send network packet
        # - etc.

    # Poll at reasonable rate
    # Note: In ultra-low-power applications, you would use deep sleep
    # and wake on the physical INT pin instead of polling
    time.sleep(0.1)  # 100ms poll interval

# ========================================================================
# Power optimization notes
# ========================================================================

# For maximum power savings:
#
# 1. Use the INT pin to wake from deep sleep instead of polling:
#    - Connect INT1/INT2 to a wake-capable GPIO
#    - Configure the interrupt
#    - Enter deep sleep
#    - The sensor will wake the MCU when motion detected
#
# 2. The sensor automatically uses low-power mode when WOM is enabled:
#    - Accelerometer: Low-power mode at 50Hz
#    - Gyroscope: Disabled to save power
#    - This consumes only ~5µA in standby
#
# 3. Adjust threshold to reduce false wake-ups:
#    - Higher threshold = fewer wake-ups = lower average power
#    - Test threshold based on your application needs
#
# 4. Use axis selection to reduce sensitivity:
#    - Monitor only relevant axes (e.g., Z-axis for vertical motion)
#    - axes="z" instead of axes="all"

# ========================================================================
# Alternative: Interrupt-driven wake (for reference)
# ========================================================================

# If you have the INT pin connected, you can use it for wake-up:

# import digitalio
# import alarm
#
# # Setup interrupt pin for wake
# int_pin_alarm = alarm.pin.PinAlarm(pin=board.D5, value=True, pull=False)
#
# print("Entering deep sleep. Will wake on motion...")
#
# # Enter deep sleep - will wake when INT pin goes high
# alarm.exit_and_deep_sleep_until_alarms(int_pin_alarm)
#
# # Code here runs after wake-up
# print("Woke up due to motion!")
# status = icm.read_interrupt_status()

# ========================================================================
# Additional operations (for reference)
# ========================================================================

# To disable WOM:
# icm.disable_motion_detection()

# To change threshold without reconfiguring:
# (Requires bank switching - see advanced usage)

# To configure individual axes:
# icm.enable_wake_on_motion(threshold=50, axes="z", int_pin=1)  # Z-axis only
