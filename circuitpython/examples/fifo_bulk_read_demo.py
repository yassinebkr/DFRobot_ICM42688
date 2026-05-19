"""
FIFO Bulk Read Demo

Demonstrates the high-performance bulk FIFO read capability.
Instead of reading one packet at a time (slow), read all packets
at once (10-100x faster).

Critical for:
- Data logging to SD card
- High-rate sensor fusion
- ML training data collection
- Batch processing

Author: Auto-generated bulk FIFO example
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

def demo_fifo_bulk_read():
    """Demonstrate FIFO bulk read performance"""
    print("\n")
    print("╔══════════════════════════════════════════════════════════════════════╗")
    print("║       FIFO Bulk Read Demo - High-Performance Data Logging           ║")
    print("╚══════════════════════════════════════════════════════════════════════╝")

    # Initialize sensor
    print("\nInitializing sensor...")
    i2c = busio.I2C(board.SCL, board.SDA, frequency=400000)
    icm = ICM42688(i2c)
    print("✓ Sensor ready")

    # =========================================================================
    # Setup FIFO
    # =========================================================================
    print_header("FIFO Configuration")

    print("\nConfiguring FIFO:")
    print("  • Mode: Stream (overwrite old data when full)")
    print("  • Contents: Accel + Gyro + Temperature")
    print("  • Size: 2048 bytes = 128 packets max")

    icm.enable_fifo(accel=True, gyro=True, temp=True, mode="stream")
    print("✓ FIFO enabled")

    # Flush any old data
    icm.flush_fifo()
    print("✓ FIFO flushed")

    # =========================================================================
    # Pattern 1: Periodic Bulk Read (Data Logging)
    # =========================================================================
    print_header("Pattern 1: Periodic Bulk Read (Data Logging)")

    print("\nScenario: Log sensor data every 100ms")
    print("  • Sensor ODR: 1 kHz → ~100 samples per batch")
    print("  • Bulk read: Read all 100 packets at once")
    print("  • Write to SD card / storage")

    total_packets = 0
    total_time = 0

    print("\nCollecting data for 5 batches...")
    for batch_num in range(5):
        # Wait for data to accumulate
        time.sleep(0.1)  # 100ms

        # Read FIFO count
        fifo_bytes = icm.fifo_count
        num_packets = fifo_bytes // 16

        # Bulk read all packets
        start = time.monotonic()
        packets = icm.read_fifo_bulk(max_packets=num_packets)
        read_time = time.monotonic() - start

        total_packets += len(packets)
        total_time += read_time

        print(f"\n  Batch {batch_num + 1}:")
        print(f"    • Packets available: {num_packets}")
        print(f"    • Packets read: {len(packets)}")
        print(f"    • Read time: {read_time*1000:.2f} ms")
        print(f"    • Throughput: {len(packets)/read_time:.1f} packets/sec")

        # Example: Print first and last packet
        if packets:
            first = packets[0]
            last = packets[-1]
            print(f"    • First accel: ({first['accel'][0]:.2f}, {first['accel'][1]:.2f}, {first['accel'][2]:.2f})")
            print(f"    • Last accel:  ({last['accel'][0]:.2f}, {last['accel'][1]:.2f}, {last['accel'][2]:.2f})")

    avg_read_time = (total_time / 5) * 1000
    print(f"\n  📊 Average batch read time: {avg_read_time:.2f} ms")
    print(f"  📊 Total packets collected: {total_packets}")
    print(f"  ✓ Data logging pattern demonstrated")

    # Clear FIFO
    icm.flush_fifo()
    time.sleep(0.5)

    # =========================================================================
    # Pattern 2: Continuous High-Rate Collection
    # =========================================================================
    print_header("Pattern 2: Continuous High-Rate Collection")

    print("\nScenario: Collect maximum data for 3 seconds")
    print("  • Read FIFO as fast as possible")
    print("  • Simulate ML training data collection")

    all_packets = []
    start_time = time.monotonic()
    reads = 0

    print("\nCollecting...")
    while time.monotonic() - start_time < 3.0:
        # Check if FIFO has data
        if icm.fifo_count >= 16:
            packets = icm.read_fifo_bulk(max_packets=64)
            all_packets.extend(packets)
            reads += 1

        # Small delay to prevent bus saturation
        time.sleep(0.01)

    elapsed = time.monotonic() - start_time

    print(f"\n  📊 Results:")
    print(f"    • Total packets: {len(all_packets)}")
    print(f"    • Collection time: {elapsed:.2f} seconds")
    print(f"    • Average rate: {len(all_packets)/elapsed:.1f} packets/sec")
    print(f"    • Number of bulk reads: {reads}")
    print(f"    • Average packets per read: {len(all_packets)/reads:.1f}")

    # Example analysis
    if all_packets:
        # Calculate statistics
        accel_z_values = [p['accel'][2] for p in all_packets]
        avg_accel_z = sum(accel_z_values) / len(accel_z_values)
        min_accel_z = min(accel_z_values)
        max_accel_z = max(accel_z_values)

        print(f"\n  📈 Accel Z-axis statistics:")
        print(f"    • Mean: {avg_accel_z:.2f} m/s²")
        print(f"    • Min:  {min_accel_z:.2f} m/s²")
        print(f"    • Max:  {max_accel_z:.2f} m/s²")
        print(f"    • Range: {max_accel_z - min_accel_z:.2f} m/s²")

    print(f"  ✓ High-rate collection demonstrated")

    # Clear FIFO
    icm.flush_fifo()
    time.sleep(0.5)

    # =========================================================================
    # Pattern 3: Event-Triggered Bulk Read
    # =========================================================================
    print_header("Pattern 3: Event-Triggered Bulk Read")

    print("\nScenario: Read FIFO when watermark threshold reached")
    print("  • Wait for 32+ packets (512 bytes)")
    print("  • Bulk read when threshold met")
    print("  • Efficient for power-saving")

    watermark_packets = 32
    print(f"\n  Watermark: {watermark_packets} packets")
    print("  Waiting for FIFO to fill...")

    event_count = 0
    max_events = 3

    while event_count < max_events:
        # Check FIFO level
        fifo_bytes = icm.fifo_count
        num_packets = fifo_bytes // 16

        if num_packets >= watermark_packets:
            # Threshold reached - bulk read
            start = time.monotonic()
            packets = icm.read_fifo_bulk(max_packets=num_packets)
            read_time = time.monotonic() - start

            event_count += 1

            print(f"\n  Event {event_count}:")
            print(f"    • Packets ready: {num_packets}")
            print(f"    • Packets read: {len(packets)}")
            print(f"    • Read time: {read_time*1000:.2f} ms")

            # Example: Detect motion events
            if packets:
                # Calculate max acceleration magnitude in batch
                max_mag = 0
                for p in packets:
                    ax, ay, az = p['accel']
                    mag = (ax**2 + ay**2 + az**2)**0.5
                    if mag > max_mag:
                        max_mag = mag

                print(f"    • Max accel magnitude: {max_mag:.2f} m/s²")

                if max_mag > 12.0:
                    print(f"    ⚠️  Motion detected! (> 12 m/s²)")

        time.sleep(0.05)  # Check every 50ms

    print(f"\n  ✓ Event-triggered pattern demonstrated")

    # =========================================================================
    # Performance Comparison
    # =========================================================================
    print_header("Performance Comparison: Bulk vs. Single Read")

    print("\n📊 Test: Read 64 packets two different ways")

    # Fill FIFO
    icm.flush_fifo()
    time.sleep(0.1)  # Let it fill

    # Method 1: Bulk read
    print("\n  Method 1: read_fifo_bulk()")
    start = time.monotonic()
    packets_bulk = icm.read_fifo_bulk(max_packets=64)
    time_bulk = time.monotonic() - start
    print(f"    • Read {len(packets_bulk)} packets")
    print(f"    • Time: {time_bulk*1000:.2f} ms")
    print(f"    • Rate: {len(packets_bulk)/time_bulk:.1f} packets/sec")

    # Refill FIFO
    icm.flush_fifo()
    time.sleep(0.1)

    # Method 2: Single read in loop (traditional approach)
    print("\n  Method 2: read_fifo() in loop")
    packets_single = []
    start = time.monotonic()
    for _ in range(64):
        if icm.fifo_count >= 16:
            packet = icm.read_fifo()
            packets_single.append(packet)
        else:
            break
    time_single = time.monotonic() - start
    print(f"    • Read {len(packets_single)} packets")
    print(f"    • Time: {time_single*1000:.2f} ms")
    print(f"    • Rate: {len(packets_single)/time_single:.1f} packets/sec")

    # Comparison
    if time_single > 0:
        speedup = time_single / time_bulk
        print(f"\n  ✅ Bulk read is {speedup:.1f}x faster!")
        print(f"     ({time_single/len(packets_single)*1000:.2f} ms/packet → {time_bulk/len(packets_bulk)*1000:.2f} ms/packet)")

    # =========================================================================
    # Cleanup
    # =========================================================================
    print_header("Cleanup")

    icm.disable_fifo()
    print("✓ FIFO disabled")

    # =========================================================================
    # Summary
    # =========================================================================
    print_header("Summary")

    print("\n📚 FIFO Bulk Read API:")
    print("  packets = icm.read_fifo_bulk(max_packets=128)")
    print("  # Returns list of packet dictionaries")

    print("\n🚀 Performance Benefits:")
    print("  • 10-100x faster than single-packet reads")
    print("  • Single I2C/SPI transaction for all packets")
    print("  • Minimal CPU overhead")
    print("  • Ideal for batch processing")

    print("\n💡 Use Cases:")
    print("  1. Data logging to SD card")
    print("     → Accumulate 100-200ms, bulk read, batch write")

    print("\n  2. ML training data collection")
    print("     → High-rate collection with minimal overhead")

    print("\n  3. Event detection in batches")
    print("     → Read watermark threshold, process all at once")

    print("\n  4. Power-efficient operation")
    print("     → Sleep until FIFO fills, wake and bulk read")

    print("\n  5. High-throughput sensor fusion")
    print("     → Minimize bus transactions for max rate")

    print("\n⚠️  Important Notes:")
    print("  • FIFO size: 2048 bytes = 128 packets max")
    print("  • Packet size: 16 bytes (basic mode)")
    print("  • Call enable_fifo() before using bulk read")
    print("  • Use flush_fifo() to reset FIFO")

    print("\n🔗 See Also:")
    print("  • circuitpython/examples/icm42688_simpletest.py")
    print("  • docs/CIRCUITPYTHON_PERFORMANCE_ANALYSIS.md")

    print("\n" + "=" * 70)
    print("Demo complete! Try bulk FIFO read in your data logging application.")
    print("=" * 70 + "\n")

# Run demo
if __name__ == "__main__":
    demo_fifo_bulk_read()
