"""
Aerospace Filters Configuration Demo

Demonstrates the signal processing filters critical for aerospace applications:
1. Anti-Aliasing Filter (AAF) - Hardware analog filter
2. UI Low-Pass Filter - Software filter with selectable bandwidth
3. Gyro Notch Filter - Reject specific vibration frequencies

These filters are essential for vibrating platforms like drones, vehicles,
and machinery to prevent aliasing and reduce noise.

Author: Auto-generated aerospace example
"""

import time
import board
import busio
from adafruit_icm42688 import ICM42688

def print_header(text):
    """Print formatted section header"""
    print("\n" + "=" * 70)
    print(f"  {text}")
    print("=" * 70)

def demo_filter_configurations():
    """Demonstrate different filter configurations for various scenarios"""
    print("\n")
    print("╔══════════════════════════════════════════════════════════════════════╗")
    print("║       ICM42688P Aerospace Filters Configuration Demo                 ║")
    print("╚══════════════════════════════════════════════════════════════════════╝")

    # Initialize sensor
    print("\nInitializing I2C @ 400 kHz...")
    i2c = busio.I2C(board.SCL, board.SDA, frequency=400000)
    icm = ICM42688(i2c)
    print("✓ Sensor initialized\n")

    # =========================================================================
    # Scenario 1: Quadcopter / Drone (High vibration, known propeller frequency)
    # =========================================================================
    print_header("Scenario 1: Quadcopter / Drone Configuration")

    print("\nTypical quadcopter challenges:")
    print("  • Propeller vibrations: 120-240 Hz (depends on motor RPM)")
    print("  • High-frequency noise from ESCs")
    print("  • Need for real-time responsiveness")

    print("\nRecommended filter settings:")

    # AAF: Enable with moderate bandwidth
    print("  1. Anti-Aliasing Filter (AAF):")
    print("     - Enabled, bandwidth_index=15 (~213 Hz)")
    print("     - Rejects high-frequency aliasing artifacts")
    icm.set_aaf_filter("both", enabled=True, bandwidth_index=15)
    print("     ✓ Configured")

    # UI Filter: 2nd order, moderate bandwidth
    print("\n  2. UI Low-Pass Filter:")
    print("     - 2nd order filter (good noise rejection)")
    print("     - Bandwidth index=3 (moderate filtering)")
    print("     - Preserves flight dynamics response")
    icm.set_ui_filter("both", filter_order=2, bandwidth_index=3)
    print("     ✓ Configured")

    # Notch Filter: Reject propeller vibration
    print("\n  3. Gyro Notch Filter:")
    print("     - Center frequency: 120 Hz (example propeller freq)")
    print("     - Narrow bandwidth (Q=1) for precise rejection")
    print("     - Applied to all axes")
    icm.set_gyro_notch_filter(frequency_hz=120.0, bandwidth=0, axis="all")
    print("     ✓ Configured")

    print("\n  📊 Reading filtered data...")
    for i in range(5):
        temp, accel, gyro = icm.all_data
        mag = (accel[0]**2 + accel[1]**2 + accel[2]**2)**0.5
        print(f"     Sample {i+1}: Accel magnitude = {mag:.2f} m/s² (expect ~9.81)")
        time.sleep(0.1)

    print("\n  ✓ Quadcopter configuration complete")

    time.sleep(2)

    # =========================================================================
    # Scenario 2: Fixed-Wing Aircraft (Lower vibration, emphasis on precision)
    # =========================================================================
    print_header("Scenario 2: Fixed-Wing Aircraft Configuration")

    print("\nTypical fixed-wing challenges:")
    print("  • Engine/propeller vibrations: 80-150 Hz")
    print("  • Airframe flutter at high speeds")
    print("  • Need for precise attitude estimation")

    print("\nRecommended filter settings:")

    # AAF: Enable with wider bandwidth
    print("  1. Anti-Aliasing Filter (AAF):")
    print("     - Enabled, bandwidth_index=20 (wider bandwidth)")
    print("     - Minimal phase lag for precise control")
    icm.set_aaf_filter("both", enabled=True, bandwidth_index=20)
    print("     ✓ Configured")

    # UI Filter: 3rd order, low-latency bandwidth
    print("\n  2. UI Low-Pass Filter:")
    print("     - 3rd order filter (maximum noise rejection)")
    print("     - Bandwidth index=14 (low-latency option)")
    print("     - Good balance for slow dynamics")
    icm.set_ui_filter("both", filter_order=3, bandwidth_index=14)
    print("     ✓ Configured")

    # Notch Filter: Reject engine vibration
    print("\n  3. Gyro Notch Filter:")
    print("     - Center frequency: 100 Hz (example engine freq)")
    print("     - Moderate bandwidth")
    icm.set_gyro_notch_filter(frequency_hz=100.0, bandwidth=1, axis="all")
    print("     ✓ Configured")

    print("\n  📊 Reading filtered data...")
    for i in range(5):
        temp, accel, gyro = icm.all_data
        gyro_mag = (gyro[0]**2 + gyro[1]**2 + gyro[2]**2)**0.5
        print(f"     Sample {i+1}: Gyro magnitude = {gyro_mag:.3f} rad/s (expect ~0 when still)")
        time.sleep(0.1)

    print("\n  ✓ Fixed-wing configuration complete")

    time.sleep(2)

    # =========================================================================
    # Scenario 3: Ground Vehicle (Variable vibrations, harsh environment)
    # =========================================================================
    print_header("Scenario 3: Ground Vehicle Configuration")

    print("\nTypical ground vehicle challenges:")
    print("  • Engine vibrations: 20-100 Hz (depends on RPM)")
    print("  • Road/terrain vibrations: Wide frequency spectrum")
    print("  • Shock and impact events")

    print("\nRecommended filter settings:")

    # AAF: Enable with narrow bandwidth (more filtering)
    print("  1. Anti-Aliasing Filter (AAF):")
    print("     - Enabled, bandwidth_index=10 (narrower bandwidth)")
    print("     - Strong anti-aliasing for harsh environments")
    icm.set_aaf_filter("both", enabled=True, bandwidth_index=10)
    print("     ✓ Configured")

    # UI Filter: 2nd order, aggressive filtering
    print("\n  2. UI Low-Pass Filter:")
    print("     - 2nd order filter")
    print("     - Bandwidth index=6 (aggressive low-pass)")
    print("     - Smooth out road vibrations")
    icm.set_ui_filter("both", filter_order=2, bandwidth_index=6)
    print("     ✓ Configured")

    # Notch Filter: Reject engine idle frequency
    print("\n  3. Gyro Notch Filter:")
    print("     - Center frequency: 60 Hz (example engine idle)")
    print("     - Wide bandwidth to cover RPM variations")
    icm.set_gyro_notch_filter(frequency_hz=60.0, bandwidth=2, axis="all")
    print("     ✓ Configured")

    print("\n  📊 Reading filtered data...")
    for i in range(5):
        temp, accel, gyro = icm.all_data
        print(f"     Sample {i+1}: Temp={temp:.1f}°C, Accel Z={accel[2]:.2f} m/s²")
        time.sleep(0.1)

    print("\n  ✓ Ground vehicle configuration complete")

    time.sleep(2)

    # =========================================================================
    # Scenario 4: Low-Vibration Lab Environment (Minimal filtering)
    # =========================================================================
    print_header("Scenario 4: Lab/Benchtop Configuration")

    print("\nTypical lab environment:")
    print("  • Minimal vibrations")
    print("  • Emphasis on raw measurement fidelity")
    print("  • Maximum bandwidth desired")

    print("\nRecommended filter settings:")

    # AAF: Disable for maximum bandwidth
    print("  1. Anti-Aliasing Filter (AAF):")
    print("     - DISABLED (maximum bandwidth)")
    print("     - Only use when you understand aliasing risks")
    icm.set_aaf_filter("both", enabled=False)
    print("     ✓ Configured")

    # UI Filter: Bypass or 1st order minimal
    print("\n  2. UI Low-Pass Filter:")
    print("     - 1st order filter (minimal filtering)")
    print("     - Bandwidth index=0 (widest: ODR/2)")
    print("     - Near-raw measurements")
    icm.set_ui_filter("both", filter_order=1, bandwidth_index=0)
    print("     ✓ Configured")

    # Notch Filter: Disabled
    print("\n  3. Gyro Notch Filter:")
    print("     - DISABLED (no specific vibration sources)")
    icm.set_gyro_notch_filter(frequency_hz=None)  # Disable
    print("     ✓ Configured")

    print("\n  📊 Reading unfiltered data...")
    for i in range(5):
        temp, accel, gyro = icm.all_data
        print(f"     Sample {i+1}: Accel=({accel[0]:.2f}, {accel[1]:.2f}, {accel[2]:.2f}) m/s²")
        time.sleep(0.1)

    print("\n  ✓ Lab configuration complete")

    # =========================================================================
    # Summary
    # =========================================================================
    print_header("Filter Configuration Summary")

    print("\n📚 Key Takeaways:")
    print("  1. AAF (Anti-Aliasing Filter):")
    print("     - Hardware analog filter before ADC")
    print("     - Enable when ODR > 1kHz or high-frequency noise present")
    print("     - Bandwidth: 1=narrow (42Hz) → 63=wide (max)")

    print("\n  2. UI Low-Pass Filter:")
    print("     - Software digital filter in sensor")
    print("     - Order: 0=bypass, 1=fast, 2=balanced, 3=smooth")
    print("     - Bandwidth: 0=wide → 7=narrow, 14-15=low-latency")

    print("\n  3. Gyro Notch Filter:")
    print("     - Rejects specific frequency (e.g., propeller @ 120Hz)")
    print("     - Critical for drones and vehicles")
    print("     - Per-axis or all-axes configuration")

    print("\n  4. General Guidelines:")
    print("     - High vibration → Enable all filters, narrow bandwidths")
    print("     - Low vibration → Minimal filtering, wide bandwidths")
    print("     - Known vibration source → Use notch filter at that frequency")
    print("     - Real-time control → Use low-latency filter options")

    print("\n🔗 References:")
    print("  • ICM42688 Datasheet: Sections 6.2-6.5 (Filters)")
    print("  • Application Note: 'Vibration Rejection for IMUs'")
    print("  • See: docs/CIRCUITPYTHON_PERFORMANCE_ANALYSIS.md")

    print("\n" + "=" * 70)
    print("Demo complete! Experiment with different configurations for your application.")
    print("=" * 70 + "\n")

# Run demo
if __name__ == "__main__":
    demo_filter_configurations()
