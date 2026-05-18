"""
High-Speed ICM42688P Reading Example

Demonstrates how to achieve maximum sample rates:
- I2C: 100-200 Hz (with optimizations)
- SPI: 300-800 Hz (Arduino-like performance)

Quick Fix: Replace this SLOW pattern:
    accel = icm.acceleration
    gyro = icm.gyro

With this FAST pattern:
    temp, accel, gyro = icm.all_data

Author: Auto-generated for performance optimization
"""

import time
import board
import busio

# Try importing SPI components (may not be available on all boards)
try:
    import digitalio
    HAS_SPI = True
except ImportError:
    HAS_SPI = False

from adafruit_icm42688 import ICM42688

# ============================================================================
# Configuration - Choose your interface
# ============================================================================

USE_SPI = False  # Set to True for maximum performance (300-800 Hz)
USE_FAST_I2C = True  # Set to True for 1 MHz I2C (if supported)

# ============================================================================
# Example 1: Optimized I2C (100-200 Hz)
# ============================================================================

def demo_optimized_i2c():
    """
    Optimized I2C reading pattern.

    Achieves 100-200 Hz by:
    1. Using .all_data property (single read)
    2. Using fast I2C speed (1 MHz)
    """
    print("\n" + "=" * 60)
    print("  Optimized I2C Demo")
    print("=" * 60)

    # Use fast I2C if supported
    i2c_speed = 1000000 if USE_FAST_I2C else 400000
    print(f"\nInitializing I2C @ {i2c_speed // 1000} kHz...")

    i2c = busio.I2C(board.SCL, board.SDA, frequency=i2c_speed)
    icm = ICM42688(i2c)

    print("✓ Sensor initialized")
    print("\nReading sensor data (press Ctrl+C to stop)...")
    print("Format: Temp | Accel (m/s²) | Gyro (rad/s) | Rate\n")

    # Performance measurement
    last_time = time.monotonic()
    count = 0
    rate_display_interval = 1.0  # Update rate every second

    try:
        while True:
            # ✓ EFFICIENT: Single read gets all data
            temp, accel, gyro = icm.all_data

            count += 1
            current_time = time.monotonic()
            elapsed = current_time - last_time

            # Display rate every second
            if elapsed >= rate_display_interval:
                rate = count / elapsed

                # Format output
                print(f"{temp:6.1f}°C | "
                      f"({accel[0]:7.2f}, {accel[1]:7.2f}, {accel[2]:7.2f}) | "
                      f"({gyro[0]:6.2f}, {gyro[1]:6.2f}, {gyro[2]:6.2f}) | "
                      f"{rate:6.1f} Hz")

                # Reset counters
                last_time = current_time
                count = 0

    except KeyboardInterrupt:
        print("\n\nStopped by user")

    i2c.deinit()

# ============================================================================
# Example 2: High-Speed SPI (300-800 Hz)
# ============================================================================

def demo_high_speed_spi():
    """
    High-speed SPI reading pattern.

    Achieves 300-800 Hz using:
    1. SPI interface @ 24 MHz
    2. .all_data property (single read)

    Requires SPI wiring (see FEATHER_RP2040_WIRING_GUIDE.md)
    """
    print("\n" + "=" * 60)
    print("  High-Speed SPI Demo")
    print("=" * 60)

    if not HAS_SPI:
        print("\n✗ SPI not available on this board")
        return

    print("\nInitializing SPI @ 24 MHz...")
    print("Wiring check:")
    print("  ICM42688P SDA  → Board MOSI")
    print("  ICM42688P SDO  → Board MISO")
    print("  ICM42688P SCL  → Board SCK")
    print("  ICM42688P CS   → Board D5\n")

    try:
        spi = busio.SPI(board.SCK, MOSI=board.MOSI, MISO=board.MISO)
        cs = digitalio.DigitalInOut(board.D5)
        icm = ICM42688(spi, cs=cs, baudrate=24000000)

        print("✓ Sensor initialized")
        print("\nReading at maximum speed (press Ctrl+C to stop)...")
        print("Format: Temp | Accel (m/s²) | Gyro (rad/s) | Rate\n")

        # Performance measurement
        last_time = time.monotonic()
        count = 0
        rate_display_interval = 1.0

        try:
            while True:
                # Read all data in single transaction
                temp, accel, gyro = icm.all_data

                count += 1
                current_time = time.monotonic()
                elapsed = current_time - last_time

                # Display rate every second
                if elapsed >= rate_display_interval:
                    rate = count / elapsed

                    print(f"{temp:6.1f}°C | "
                          f"({accel[0]:7.2f}, {accel[1]:7.2f}, {accel[2]:7.2f}) | "
                          f"({gyro[0]:6.2f}, {gyro[1]:6.2f}, {gyro[2]:6.2f}) | "
                          f"{rate:6.1f} Hz")

                    last_time = current_time
                    count = 0

        except KeyboardInterrupt:
            print("\n\nStopped by user")

        spi.deinit()

    except Exception as e:
        print(f"\n✗ SPI initialization failed: {e}")
        print("  Check wiring or set USE_SPI = False")

# ============================================================================
# Comparison: Slow vs Fast Reading
# ============================================================================

def demo_slow_vs_fast_comparison():
    """
    Side-by-side comparison of slow and fast reading patterns.

    Shows the performance difference clearly.
    """
    print("\n" + "=" * 60)
    print("  Performance Comparison: Slow vs Fast")
    print("=" * 60)

    i2c = busio.I2C(board.SCL, board.SDA, frequency=400000)
    icm = ICM42688(i2c)

    duration = 3.0  # Test for 3 seconds each

    # Test 1: SLOW method (separate reads)
    print("\n📊 Test 1: SLOW - Separate reads")
    print("   Code: accel = icm.acceleration; gyro = icm.gyro")

    start = time.monotonic()
    count_slow = 0

    while time.monotonic() - start < duration:
        accel = icm.acceleration  # Read #1
        gyro = icm.gyro           # Read #2 (redundant!)
        count_slow += 1

    rate_slow = count_slow / duration
    print(f"   Result: {rate_slow:.1f} Hz")

    # Test 2: FAST method (all_data)
    print("\n📊 Test 2: FAST - Single read")
    print("   Code: temp, accel, gyro = icm.all_data")

    start = time.monotonic()
    count_fast = 0

    while time.monotonic() - start < duration:
        temp, accel, gyro = icm.all_data  # Single read
        count_fast += 1

    rate_fast = count_fast / duration
    print(f"   Result: {rate_fast:.1f} Hz")

    # Show improvement
    improvement = (rate_fast / rate_slow - 1) * 100
    print("\n" + "=" * 60)
    print(f"  FAST method is {improvement:.0f}% faster ({rate_fast / rate_slow:.2f}x speedup)!")
    print("=" * 60)

    i2c.deinit()

# ============================================================================
# Main Program
# ============================================================================

if __name__ == "__main__":
    print("\n")
    print("╔════════════════════════════════════════════════════════════╗")
    print("║     ICM42688P High-Speed Reading Examples                  ║")
    print("╚════════════════════════════════════════════════════════════╝")

    print("\nAvailable demos:")
    print("  1. Optimized I2C (100-200 Hz)")
    print("  2. High-Speed SPI (300-800 Hz)")
    print("  3. Slow vs Fast comparison")

    # Run comparison first to show the difference
    demo_slow_vs_fast_comparison()

    print("\n" + "=" * 60)
    print("Starting continuous reading demo...")
    print("=" * 60)

    # Run appropriate demo based on configuration
    if USE_SPI and HAS_SPI:
        demo_high_speed_spi()
    else:
        demo_optimized_i2c()

    print("\n✓ Demo complete!")
    print("\n💡 Tips:")
    print("  • For Arduino-like performance (300+ Hz), use SPI")
    print("  • Always use .all_data instead of separate properties")
    print("  • Increase I2C speed to 1 MHz if your wiring supports it")
    print("  • See CIRCUITPYTHON_PERFORMANCE_ANALYSIS.md for details\n")
