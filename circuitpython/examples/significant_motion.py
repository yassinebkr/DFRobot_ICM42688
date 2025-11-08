# SPDX-FileCopyrightText: 2025 Yassine Bekkari
# SPDX-License-Identifier: MIT

"""
Significant motion detection example for ICM42688.

This example demonstrates how to use the significant motion detection (SMD)
feature to detect meaningful motion events. Unlike wake-on-motion which triggers
on any motion above threshold, SMD requires two separate motion events within
a time window, making it ideal for detecting intentional device movement while
ignoring brief vibrations or accidental bumps.

SMD is perfect for:
- Detecting device pick-up or put-down
- Detecting when user starts using the device
- Avoiding false wake-ups from vibrations
- Battery-powered applications requiring high confidence wake

Hardware setup:
- Connect ICM42688 to I2C or SPI on your Feather RP2040
- Optionally connect INT1 or INT2 pin to a GPIO for interrupt-driven detection
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

print("ICM42688 Significant Motion Detection Example")
print("=" * 70)

# ========================================================================
# Configure interrupt pin
# ========================================================================

# Configure INT1 pin for SMD interrupts
icm.configure_interrupt(
    pin=1,
    polarity="high",
    mode="latch",
    drive="push-pull"
)

print("Interrupt Configuration:")
print("  Pin: INT1")
print("  Polarity: Active high")
print("  Mode: Latched")
print()

# ========================================================================
# Configure significant motion detection
# ========================================================================

# SMD has two modes:
#   "short": Requires 2 WOM events 1 second apart
#   "long": Requires 2 WOM events 3 seconds apart
#
# Choose mode based on your application:
#   - "short": Faster response, more sensitive to quick movements
#   - "long": Slower response, better for detecting sustained motion

SMD_MODE = "short"  # Change to "long" for 3-second window

icm.enable_significant_motion_detection(mode=SMD_MODE, int_pin=1)

print("Significant Motion Detection Configuration:")
print(f"  Mode: {SMD_MODE.upper()}")
print(f"  Time window: {1 if SMD_MODE == 'short' else 3} second(s)")
print(f"  WOM threshold: 50 (~195mg)")
print("  Detection: Requires 2 separate motion events within time window")
print()
print("How it works:")
print("  1. First motion event triggers WOM")
print("  2. Sensor waits for second motion event")
print(f"  3. If second event occurs within {1 if SMD_MODE == 'short' else 3}s → SMD interrupt!")
print(f"  4. If no second event within {1 if SMD_MODE == 'short' else 3}s → Reset, wait for next motion")
print()
print("Try these tests:")
print("  ✓ Pick up and shake the sensor → Should trigger SMD")
print("  ✓ Tap once, wait, tap again quickly → Should trigger SMD")
print("  ✗ Single tap → Should NOT trigger SMD")
print("  ✗ Gentle continuous vibration → Should NOT trigger SMD")
print()
print("Monitoring for significant motion... (press Ctrl+C to stop)")
print("=" * 70)
print()

# ========================================================================
# Main loop - monitor for significant motion events
# ========================================================================

smd_count = 0
wom_count = 0
last_event_time = time.monotonic()

while True:
    # Read interrupt status
    status = icm.read_interrupt_status()

    # Check for significant motion detection
    if status['smd']:
        smd_count += 1
        current_time = time.monotonic()
        time_since_last = current_time - last_event_time
        last_event_time = current_time

        print(f"\n{'*' * 70}")
        print(f"SIGNIFICANT MOTION #{smd_count} DETECTED!")
        print(f"{'*' * 70}")
        print(f"  Time since last SMD: {time_since_last:.2f}s")

        # Read current acceleration
        accel_x, accel_y, accel_z = icm.acceleration
        magnitude = (accel_x**2 + accel_y**2 + accel_z**2) ** 0.5

        print(f"  Current acceleration magnitude: {magnitude:.2f} m/s²")
        print(f"  Current acceleration:")
        print(f"    X: {accel_x:7.2f} m/s²")
        print(f"    Y: {accel_y:7.2f} m/s²")
        print(f"    Z: {accel_z:7.2f} m/s²")
        print()

        # In a real application, trigger significant action here:
        # - Wake main system from deep sleep
        # - Start user interaction mode
        # - Begin data logging session
        # - Send high-priority notification
        # etc.

    # Check for individual wake-on-motion events (for debugging)
    if status['wom_x'] or status['wom_y'] or status['wom_z']:
        wom_count += 1
        axes = []
        if status['wom_x']:
            axes.append('X')
        if status['wom_y']:
            axes.append('Y')
        if status['wom_z']:
            axes.append('Z')

        print(f"  WOM event #{wom_count}: {', '.join(axes)} axis")
        print(f"    (Waiting for second event within {1 if SMD_MODE == 'short' else 3}s...)")

    # Poll at reasonable rate
    time.sleep(0.1)  # 100ms

# ========================================================================
# Comparison: WOM vs SMD
# ========================================================================

# Wake-on-Motion (WOM):
#   - Triggers on ANY motion above threshold
#   - Single event detection
#   - Fast response time
#   - May have false positives from vibration
#   - Use for: Quick wake-up, high sensitivity needed
#
# Significant Motion Detection (SMD):
#   - Triggers on TWO motion events within time window
#   - Dual event detection
#   - Delayed response time (1-3 seconds)
#   - Filters out brief vibrations and accidental bumps
#   - Use for: Reliable wake-up, avoiding false triggers

# ========================================================================
# Power consumption notes
# ========================================================================

# SMD mode uses the same low-power configuration as WOM:
#   - Accelerometer in LP mode at 50Hz (~5µA)
#   - Gyroscope disabled
#   - Perfect for battery-powered applications
#
# Typical power consumption:
#   - Active (all sensors on): ~1.2mA
#   - WOM/SMD mode: ~5µA
#   - Deep sleep with SMD: ~5µA + MCU sleep current
#
# Battery life example with CR2032 (220mAh):
#   - Continuous active: ~9 days
#   - SMD with 1% active time: ~2 years

# ========================================================================
# Alternative: Using SMD for deep sleep wake
# ========================================================================

# For ultra-low-power applications, use SMD with deep sleep:

# import digitalio
# import alarm
#
# # Configure SMD
# icm.enable_significant_motion_detection(mode="short", int_pin=1)
#
# # Setup wake pin
# int_pin_alarm = alarm.pin.PinAlarm(pin=board.D5, value=True, pull=False)
#
# print("Entering deep sleep. Will wake on significant motion...")
# print("This provides ultra-low-power operation!")
#
# # Enter deep sleep
# alarm.exit_and_deep_sleep_until_alarms(int_pin_alarm)
#
# # Execution continues here after wake
# print("Woke up! Significant motion detected.")
# status = icm.read_interrupt_status()
# # Process motion event...

# ========================================================================
# Additional operations (for reference)
# ========================================================================

# To disable SMD:
# icm.disable_motion_detection()

# To switch modes:
# icm.enable_significant_motion_detection(mode="long", int_pin=1)

# To read status without waiting for interrupt:
# status = icm.read_interrupt_status()
# if status['smd']:
#     print("Significant motion occurred!")
