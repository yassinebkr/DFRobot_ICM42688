"""
Manual Refresh Mode Demo

Demonstrates the manual refresh pattern for per-axis access without
performance penalty. After calling refresh(), you can access individual
axes (accel_x, gyro_z, etc.) without additional I2C/SPI reads.

Use case: High-performance loops where you need flexibility to access
different axes on different iterations without double-read overhead.

Author: Auto-generated manual refresh example
"""

import time
import board
import busio
from adafruit_icm42688 import ICM42688

def print_header(text):
    """Print formatted section header"""
    print("\n" + "=" * 60)
    print(f"  {text}")
    print("=" * 60)

def demo_manual_refresh():
    """Demonstrate manual refresh mode usage"""
    print("\n")
    print("╔════════════════════════════════════════════════════════════╗")
    print("║       Manual Refresh Mode Demo                            ║")
    print("╚════════════════════════════════════════════════════════════╝")

    # Initialize sensor
    print("\nInitializing sensor...")
    i2c = busio.I2C(board.SCL, board.SDA, frequency=400000)
    icm = ICM42688(i2c)
    print("✓ Sensor ready\n")

    # =========================================================================
    # Pattern 1: Basic Manual Refresh
    # =========================================================================
    print_header("Pattern 1: Basic Manual Refresh")
    print("\nEfficient pattern for per-axis access:")
    print("  icm.refresh()    # Single 14-byte read")
    print("  x = icm.accel_x  # No I2C read")
    print("  y = icm.accel_y  # No I2C read")
    print("  z = icm.gyro_z   # No I2C read")

    print("\nReading 5 samples...")
    for i in range(5):
        # Single sensor read
        icm.refresh()

        # Access any axes without additional reads
        ax = icm.accel_x
        ay = icm.accel_y
        az = icm.accel_z
        gx = icm.gyro_x
        gy = icm.gyro_y
        gz = icm.gyro_z
        temp = icm.temp_cached

        print(f"\nSample {i+1}:")
        print(f"  Accel: X={ax:7.2f}  Y={ay:7.2f}  Z={az:7.2f} m/s²")
        print(f"  Gyro:  X={gx:7.3f}  Y={gy:7.3f}  Z={gz:7.3f} rad/s")
        print(f"  Temp:  {temp:.1f}°C")

        time.sleep(0.2)

    # =========================================================================
    # Pattern 2: Conditional Axis Access (Arduino-like)
    # =========================================================================
    print_header("Pattern 2: Conditional Axis Access")
    print("\nFlexible per-axis access (similar to Arduino API):")

    print("\nScenario: Only read Z-axis when magnitude exceeds threshold")

    for i in range(10):
        icm.refresh()

        # Check magnitude first
        ax = icm.accel_x
        ay = icm.accel_y
        magnitude = (ax**2 + ay**2)**0.5

        if magnitude > 2.0:  # Threshold: 2 m/s²
            # Only access Z-axis if needed
            az = icm.accel_z
            print(f"Sample {i+1}: Magnitude={magnitude:.2f} → Z={az:.2f} m/s²")
        else:
            print(f"Sample {i+1}: Magnitude={magnitude:.2f} → Below threshold")

        time.sleep(0.1)

    # =========================================================================
    # Pattern 3: Loop with Selective Axis Reads
    # =========================================================================
    print_header("Pattern 3: Selective Axis Reads in Loop")
    print("\nRead different axes on different iterations:")

    for i in range(6):
        icm.refresh()

        if i % 3 == 0:
            # Read X-axis every 3rd iteration
            x = icm.accel_x
            print(f"Iteration {i}: Accel X = {x:.2f} m/s²")
        elif i % 3 == 1:
            # Read Y-axis
            y = icm.accel_y
            print(f"Iteration {i}: Accel Y = {y:.2f} m/s²")
        else:
            # Read Z-axis
            z = icm.accel_z
            print(f"Iteration {i}: Accel Z = {z:.2f} m/s²")

        time.sleep(0.1)

    # =========================================================================
    # Pattern 4: High-Rate Loop (Aerospace-style)
    # =========================================================================
    print_header("Pattern 4: High-Rate Control Loop")
    print("\nAerospace control loop pattern (refresh once, use many times):")

    print("\nSimulating 2-second control loop...")
    start = time.monotonic()
    iterations = 0

    while time.monotonic() - start < 2.0:
        # Refresh sensor data once per loop
        icm.refresh()

        # Access different axes for different calculations
        # (No additional I2C transactions)

        # Attitude calculation (needs all gyro axes)
        roll_rate = icm.gyro_x
        pitch_rate = icm.gyro_y
        yaw_rate = icm.gyro_z

        # Vertical acceleration (Z-axis only)
        vertical_accel = icm.accel_z

        # Simple control logic
        if abs(pitch_rate) > 0.1:
            # Imaginary pitch correction
            pass

        if vertical_accel < 8.0:
            # Imaginary altitude hold
            pass

        iterations += 1

    elapsed = time.monotonic() - start
    rate = iterations / elapsed

    print(f"\n  Completed {iterations} iterations in {elapsed:.2f} seconds")
    print(f"  Control loop rate: {rate:.1f} Hz")
    print(f"  ✓ Each iteration = 1 sensor read + multiple axis accesses")

    # =========================================================================
    # Comparison: Manual Refresh vs. All_Data
    # =========================================================================
    print_header("Performance Comparison")

    print("\n📊 Method 1: all_data property")
    start = time.monotonic()
    for _ in range(100):
        temp, accel, gyro = icm.all_data
        # Use all data
        _ = accel[0] + accel[1] + accel[2]
    elapsed_all_data = time.monotonic() - start
    rate_all_data = 100 / elapsed_all_data

    print(f"  100 reads: {elapsed_all_data:.3f} seconds ({rate_all_data:.1f} Hz)")

    print("\n📊 Method 2: Manual refresh + per-axis")
    start = time.monotonic()
    for _ in range(100):
        icm.refresh()
        # Access same axes
        _ = icm.accel_x + icm.accel_y + icm.accel_z
    elapsed_refresh = time.monotonic() - start
    rate_refresh = 100 / elapsed_refresh

    print(f"  100 reads: {elapsed_refresh:.3f} seconds ({rate_refresh:.1f} Hz)")

    print("\n  ✅ Performance: Manual refresh ≈ all_data")
    print(f"     (Both use single 14-byte read per iteration)")

    # =========================================================================
    # When NOT to Use Manual Refresh
    # =========================================================================
    print_header("When NOT to Use Manual Refresh")

    print("\n⚠️  Don't use manual refresh if:")
    print("  1. You always read all axes together")
    print("     → Use icm.all_data instead (cleaner)")

    print("\n  2. You're porting simple Arduino code")
    print("     → all_data is simpler and just as fast")

    print("\n  3. You need guaranteed fresh data each access")
    print("     → Manual refresh caches data from last refresh()")

    print_header("When TO Use Manual Refresh")

    print("\n✅ Use manual refresh when:")
    print("  1. Porting Arduino code with per-axis access")
    print("     → Matches Arduino's icm.getAccelDataX() API pattern")

    print("\n  2. Conditional axis access based on logic")
    print("     → Example: Only read Z if X/Y exceed threshold")

    print("\n  3. Different axes needed at different times")
    print("     → Example: X/Y for navigation, Z for altitude")

    print("\n  4. Building state machines or complex control")
    print("     → refresh() at top of loop, access as needed")

    # =========================================================================
    # Summary
    # =========================================================================
    print_header("Summary")

    print("\n📚 Manual Refresh API:")
    print("  icm.refresh()      # Read and cache all sensor data")
    print("  icm.accel_x        # X-axis acceleration (cached)")
    print("  icm.accel_y        # Y-axis acceleration (cached)")
    print("  icm.accel_z        # Z-axis acceleration (cached)")
    print("  icm.gyro_x         # X-axis gyro (cached)")
    print("  icm.gyro_y         # Y-axis gyro (cached)")
    print("  icm.gyro_z         # Z-axis gyro (cached)")
    print("  icm.temp_cached    # Temperature (cached)")

    print("\n🚀 Performance:")
    print("  • Same speed as all_data (single 14-byte read)")
    print("  • No double-read penalty")
    print("  • Flexible per-axis access")

    print("\n💡 Best Practice:")
    print("  • Call refresh() once at top of loop")
    print("  • Access any axes as needed")
    print("  • No additional I2C/SPI transactions")

    print("\n" + "=" * 60)
    print("Demo complete!")
    print("=" * 60 + "\n")

# Run demo
if __name__ == "__main__":
    demo_manual_refresh()
