# SPDX-FileCopyrightText: 2025 Yassine Bekkari
# SPDX-License-Identifier: MIT

"""
Tap detection example for ICM42688.

This example demonstrates how to use the tap detection feature to detect
single and double taps on the sensor. This is useful for user interface
applications where tapping the device triggers an action.

The tap detector can identify:
- Single tap vs double tap
- Which axis the tap occurred on (X, Y, or Z)
- Tap direction (positive or negative)

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

print("ICM42688 Tap Detection Example")
print("=" * 70)

# ========================================================================
# Configure interrupt pin (optional)
# ========================================================================

# Configure INT1 pin for tap interrupts
# Polarity: "high" = active high, "low" = active low
# Mode: "latch" = stays asserted until cleared, "pulse" = brief pulse
# Drive: "push-pull" or "open-drain"
icm.configure_interrupt(
    pin=1,  # Use INT1
    polarity="high",
    mode="latch",
    drive="push-pull"
)

print("Interrupt Configuration:")
print("  Pin: INT1")
print("  Polarity: Active high")
print("  Mode: Latched")
print("  Drive: Push-pull")
print()

# ========================================================================
# Enable tap detection
# ========================================================================

# Enable tap detection
# Mode options:
#   "low-noise": High accuracy, 1kHz sampling (higher power)
#   "low-power": Lower power, 500Hz sampling
icm.enable_tap_detection(mode="low-noise", int_pin=1)

print("Tap Detection Configuration:")
print("  Mode: Low-noise (high accuracy)")
print("  Sensitivity: Default (suitable for most applications)")
print("  Detection: Single and double taps")
print()
print("Try tapping the sensor! (press Ctrl+C to stop)")
print("=" * 70)
print()

# ========================================================================
# Main loop - poll for tap events
# ========================================================================

tap_count_total = 0
last_tap_time = time.monotonic()

while True:
    # Read interrupt status (this also clears the interrupt)
    status = icm.read_interrupt_status()

    # Check if tap detected
    if status['tap']:
        # Read detailed tap information
        tap_info = icm.read_tap_info()

        tap_count_total += 1
        current_time = time.monotonic()
        time_since_last = current_time - last_tap_time
        last_tap_time = current_time

        # Display tap information
        print(f"TAP #{tap_count_total} DETECTED!")
        print(f"  Type: {tap_info['count'].upper()}")
        print(f"  Axis: {tap_info['axis'].upper()}")
        print(f"  Direction: {'Positive' if tap_info['direction'] else 'Negative'}")
        print(f"  Time since last tap: {time_since_last:.2f}s")
        print()

        # You can use this to trigger actions based on tap type
        if tap_info['count'] == 'single':
            print("  → Action: Single tap detected!")
        elif tap_info['count'] == 'double':
            print("  → Action: Double tap detected!")
        print()

    # Poll at reasonable rate (tap detection is handled by the sensor)
    time.sleep(0.05)  # 50ms poll interval

# ========================================================================
# Alternative: Interrupt-driven detection (for reference)
# ========================================================================

# If you have the INT pin connected to a GPIO, you can use interrupts
# instead of polling. Here's example code:

# import digitalio
#
# # Setup interrupt pin
# int_pin = digitalio.DigitalInOut(board.D5)  # Change to your connected pin
# int_pin.direction = digitalio.Direction.INPUT
# int_pin.pull = digitalio.Pull.DOWN  # or UP depending on configuration
#
# while True:
#     # Wait for interrupt pin to go high
#     if int_pin.value:
#         status = icm.read_interrupt_status()
#         if status['tap']:
#             tap_info = icm.read_tap_info()
#             print(f"Tap detected: {tap_info['count']} on {tap_info['axis']} axis")
#
#     time.sleep(0.01)

# ========================================================================
# Additional operations (for reference)
# ========================================================================

# To disable tap detection:
# icm.disable_motion_detection()

# To adjust sensitivity (advanced - requires register access):
# Bank 4, registers APEX_CONFIG7 and APEX_CONFIG8
# See datasheet for details on tap threshold and timing parameters
