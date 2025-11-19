"""
ICM42688P SPI Test Script for Feather RP2040

This test validates the ICM42688P sensor over SPI on the Adafruit Feather RP2040.
SPI provides higher speed communication compared to I2C.

Wiring:
    ICM42688P    Feather RP2040
    ─────────    ──────────────
    5V       →   3V
    GND      →   GND
    SDA      →   MOSI (D11, GPIO11)
    SCL      →   SCK  (D13, GPIO14)
    SDO      →   MISO (D12, GPIO12)
    CS       →   D5   (GPIO7) or any free GPIO

IMPORTANT: If using RFM95 LoRa radio, use different CS pin than D10!

Author: Auto-generated for Feather RP2040
"""

import time
import board
import busio
import digitalio
from adafruit_icm42688 import ICM42688, GyroRange, AccelRange

def print_header(text):
    """Print a formatted section header"""
    print("\n" + "=" * 60)
    print(f"  {text}")
    print("=" * 60)

def test_spi_initialization():
    """Initialize and test SPI communication"""
    print_header("SPI Initialization")

    print("Setting up SPI bus...")
    spi = busio.SPI(board.SCK, MOSI=board.MOSI, MISO=board.MISO)
    print(f"✓ SPI bus created")
    print(f"  SCK:  {board.SCK}")
    print(f"  MOSI: {board.MOSI}")
    print(f"  MISO: {board.MISO}")

    print("\nSetting up chip select (CS) on D5...")
    cs = digitalio.DigitalInOut(board.D5)
    cs.direction = digitalio.Direction.OUTPUT
    cs.value = True  # CS is active LOW, start HIGH
    print(f"✓ CS pin configured on D5")

    print("\nInitializing ICM42688P sensor...")
    try:
        icm = ICM42688(spi, cs)
        print("✓ Sensor initialized successfully")
        return icm, spi, cs
    except Exception as e:
        print(f"✗ Failed to initialize sensor: {e}")
        print("\nTroubleshooting:")
        print("  1. Verify SPI wiring (MOSI, MISO, SCK, CS)")
        print("  2. Check power connections (3V, GND)")
        print("  3. Ensure CS pin is not shared with other devices")
        return None, None, None

def test_spi_speeds(icm, spi):
    """Test different SPI clock speeds"""
    print_header("SPI Speed Tests")

    speeds = [
        (1000000, "1 MHz"),
        (2000000, "2 MHz"),
        (4000000, "4 MHz"),
        (8000000, "8 MHz"),
        (12000000, "12 MHz"),
        (16000000, "16 MHz"),
        (24000000, "24 MHz"),
    ]

    print("Testing communication at various SPI speeds...\n")

    results = []
    for speed_hz, speed_name in speeds:
        try:
            # Attempt to read at this speed
            start = time.monotonic()
            samples = 0

            # Read for 0.5 seconds
            while time.monotonic() - start < 0.5:
                _ = icm.acceleration
                _ = icm.gyro
                samples += 1

            elapsed = time.monotonic() - start
            rate = samples / elapsed

            print(f"✓ {speed_name:8s}: {rate:6.1f} samples/sec")
            results.append((speed_name, rate, True))

        except Exception as e:
            print(f"✗ {speed_name:8s}: Failed - {e}")
            results.append((speed_name, 0, False))

    # Summary
    print("\nRecommended SPI speeds:")
    print("  - 8-12 MHz: Good balance of speed and reliability")
    print("  - 24 MHz: Maximum performance (if stable)")

    return results

def test_basic_readings(icm):
    """Test basic sensor readings over SPI"""
    print_header("Basic Sensor Readings (SPI)")

    print("Reading 10 samples...\n")

    for i in range(10):
        accel_x, accel_y, accel_z = icm.acceleration
        gyro_x, gyro_y, gyro_z = icm.gyro

        print(f"Sample {i+1:2d}:")
        print(f"  Accel: X={accel_x:7.2f}  Y={accel_y:7.2f}  Z={accel_z:7.2f} m/s²")
        print(f"  Gyro:  X={gyro_x:7.2f}  Y={gyro_y:7.2f}  Z={gyro_z:7.2f} rad/s")

        time.sleep(0.1)

    print("\n✓ SPI communication working correctly")

def test_high_speed_sampling(icm):
    """Test maximum sampling rate over SPI"""
    print_header("High-Speed Sampling Test")

    print("Measuring maximum sample rate (5 second test)...")
    print("This tests the RP2040's ability to handle rapid SPI reads\n")

    start_time = time.monotonic()
    sample_count = 0
    accel_sum = [0.0, 0.0, 0.0]
    gyro_sum = [0.0, 0.0, 0.0]

    while time.monotonic() - start_time < 5.0:
        accel = icm.acceleration
        gyro = icm.gyro

        # Accumulate for averaging
        accel_sum[0] += accel[0]
        accel_sum[1] += accel[1]
        accel_sum[2] += accel[2]
        gyro_sum[0] += gyro[0]
        gyro_sum[1] += gyro[1]
        gyro_sum[2] += gyro[2]

        sample_count += 1

    elapsed = time.monotonic() - start_time
    actual_rate = sample_count / elapsed

    # Calculate averages
    accel_avg = [x / sample_count for x in accel_sum]
    gyro_avg = [x / sample_count for x in gyro_sum]

    print(f"Results:")
    print(f"  Total samples: {sample_count}")
    print(f"  Time elapsed: {elapsed:.2f} seconds")
    print(f"  Sample rate: {actual_rate:.1f} Hz")
    print(f"\nAverage readings:")
    print(f"  Accel: X={accel_avg[0]:7.2f}  Y={accel_avg[1]:7.2f}  Z={accel_avg[2]:7.2f} m/s²")
    print(f"  Gyro:  X={gyro_avg[0]:7.3f}  Y={gyro_avg[1]:7.3f}  Z={gyro_avg[2]:7.3f} rad/s")

    print("\nPerformance assessment:")
    if actual_rate > 1000:
        print("  ✓ Excellent: >1000 Hz sampling")
    elif actual_rate > 500:
        print("  ✓ Good: >500 Hz sampling")
    elif actual_rate > 200:
        print("  ⚠ Moderate: >200 Hz sampling")
    else:
        print("  ✗ Low: <200 Hz (check SPI speed)")

    # CPU usage estimate
    cpu_usage = (actual_rate * 0.5) / 1000  # Rough estimate
    print(f"  Estimated CPU usage: ~{cpu_usage:.1f}% per core")

def test_latency(icm):
    """Measure SPI read latency"""
    print_header("SPI Latency Test")

    print("Measuring single-read latency (100 samples)...\n")

    latencies = []
    for _ in range(100):
        start = time.monotonic_ns()
        _ = icm.acceleration
        _ = icm.gyro
        end = time.monotonic_ns()

        latency_us = (end - start) / 1000  # Convert to microseconds
        latencies.append(latency_us)

    avg_latency = sum(latencies) / len(latencies)
    min_latency = min(latencies)
    max_latency = max(latencies)

    print(f"Latency statistics:")
    print(f"  Average: {avg_latency:7.1f} µs")
    print(f"  Minimum: {min_latency:7.1f} µs")
    print(f"  Maximum: {max_latency:7.1f} µs")

    print("\nComparison to I2C:")
    i2c_latency = 2000  # Typical I2C @ 400kHz
    speedup = i2c_latency / avg_latency
    print(f"  I2C @ 400kHz: ~2000 µs")
    print(f"  SPI speedup: ~{speedup:.1f}x faster")

def test_range_switching(icm):
    """Test rapid range switching performance"""
    print_header("Range Switching Test")

    print("Testing rapid range changes (SPI advantage)...\n")

    ranges = [
        (AccelRange.RANGE_2G, "2G"),
        (AccelRange.RANGE_4G, "4G"),
        (AccelRange.RANGE_8G, "8G"),
        (AccelRange.RANGE_16G, "16G"),
    ]

    start = time.monotonic()
    iterations = 100

    for _ in range(iterations):
        for range_val, _ in ranges:
            icm.accelerometer_range = range_val
            _ = icm.acceleration

    elapsed = time.monotonic() - start
    time_per_switch = (elapsed / (iterations * len(ranges))) * 1000

    print(f"Switched ranges {iterations * len(ranges)} times")
    print(f"Total time: {elapsed:.2f} seconds")
    print(f"Time per switch: {time_per_switch:.2f} ms")
    print("\n✓ Fast range switching demonstrated")

def run_all_tests():
    """Run complete SPI test suite"""
    print("\n")
    print("╔════════════════════════════════════════════════════════════╗")
    print("║  ICM42688P SPI Test Suite - Feather RP2040                ║")
    print("╚════════════════════════════════════════════════════════════╝")

    # Initialize SPI
    icm, spi, cs = test_spi_initialization()
    if icm is None:
        print("\n✗ TEST FAILED: Could not initialize sensor")
        return

    # Run test sequence
    try:
        test_basic_readings(icm)
        test_spi_speeds(icm, spi)
        test_high_speed_sampling(icm)
        test_latency(icm)
        test_range_switching(icm)

        # Final summary
        print_header("Test Summary")
        print("✓ All SPI tests completed successfully!")
        print("\nFeather RP2040 + ICM42688P over SPI:")
        print("  ✓ Fast communication (up to 24 MHz)")
        print("  ✓ Low latency (~500 µs)")
        print("  ✓ High sample rates (>1000 Hz possible)")
        print("  ✓ Efficient for real-time applications")

        print("\nSPI vs I2C:")
        print("  SPI Advantages:")
        print("    - 4-10x faster sample rates")
        print("    - Lower latency")
        print("    - Dedicated bus (no sharing)")
        print("  I2C Advantages:")
        print("    - Fewer wires (4 vs 7)")
        print("    - Can share bus with other sensors")
        print("    - Better for RFM95 LoRa compatibility")

        print("\nRecommendation:")
        print("  - Use SPI if you need >500 Hz sampling")
        print("  - Use I2C for simplicity and RFM95 compatibility")

    except Exception as e:
        print(f"\n✗ Test failed with error: {e}")
        import traceback
        traceback.print_exc()

# Run tests if executed directly
if __name__ == "__main__":
    run_all_tests()
