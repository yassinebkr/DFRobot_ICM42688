# ICM42688 MicroPython Driver

MicroPython driver for the TDK InvenSense ICM42688 6-axis IMU (Inertial Measurement Unit) sensor. This is a complete port of the CircuitPython driver, optimized for ESP32, RP2040, and other MicroPython-compatible microcontrollers.

## Features

- **Full sensor access**: 3-axis accelerometer, 3-axis gyroscope, temperature sensor
- **I2C and SPI support**: Choose the interface that works best for your project
- **Configurable ranges**:
  - Accelerometer: ±2g, ±4g, ±8g, ±16g
  - Gyroscope: ±15.625 dps to ±2000 dps
- **Flexible data rates**: 1.5Hz to 32kHz output data rate
- **Power modes**: Low-power and low-noise modes for different applications
- **FIFO buffer**: 2KB FIFO for batch reading and low-power operation
- **Advanced features**:
  - Wake-on-motion detection
  - Tap detection (single and double tap)
  - Significant motion detection
  - Configurable interrupts
- **Memory optimized**: Pre-allocated buffers, minimal GC pressure
- **ESP32 optimized**: Uses `micropython.const()` for better performance

## Hardware Compatibility

**Tested Platforms:**
- ESP32 (WROOM, ESP32-C3, Lolin D32)
- RP2040 (Raspberry Pi Pico)
- Any MicroPython board with I2C or SPI support

**Software Requirements:**
- MicroPython firmware v1.19 or later
- `machine` module (I2C, SPI, Pin)
- `struct` module (standard library)

## Installation

### Option 1: Copy to board

Copy the `icm42688` folder to your MicroPython board:

```bash
# Using mpremote (recommended)
mpremote cp -r icm42688 :

# Or using ampy
ampy -p /dev/ttyUSB0 put icm42688
```

### Option 2: Use in project

Add the `icm42688` folder to your project directory and import:

```python
import icm42688
from icm42688 import registers as reg
```

## Quick Start

### I2C Example (ESP32)

```python
from machine import I2C, Pin
import icm42688

# Initialize I2C (ESP32 WROOM default pins)
i2c = I2C(0, scl=Pin(22), sda=Pin(21), freq=400000)

# Create sensor object
icm = icm42688.ICM42688(i2c, address=0x69)

# Read sensor data
accel_x, accel_y, accel_z = icm.acceleration  # m/s²
gyro_x, gyro_y, gyro_z = icm.gyro  # rad/s
temp = icm.temperature  # °C

print(f"Temperature: {temp:.2f} °C")
print(f"Acceleration: X={accel_x:.2f} Y={accel_y:.2f} Z={accel_z:.2f} m/s²")
print(f"Gyroscope: X={gyro_x:.3f} Y={gyro_y:.3f} Z={gyro_z:.3f} rad/s")
```

### SPI Example (ESP32)

```python
from machine import SPI, Pin
import icm42688

# Initialize SPI (ESP32 VSPI pins)
spi = SPI(2, baudrate=10000000, sck=Pin(18), mosi=Pin(23), miso=Pin(19))
cs = Pin(5, Pin.OUT)
cs.value(1)  # CS idle high

# Create sensor object
icm = icm42688.ICM42688(spi, cs=cs)

# Read sensor data (same API as I2C)
accel = icm.acceleration
gyro = icm.gyro
temp = icm.temperature
```

## Configuration

### Set Ranges

```python
from icm42688 import registers as reg

# Set accelerometer range
icm.accelerometer_range = reg.ACCEL_RANGE_16G  # ±16g

# Set gyroscope range
icm.gyro_range = reg.GYRO_RANGE_2000_DPS  # ±2000 dps
```

### Set Data Rates

```python
# Set output data rate
icm.accelerometer_data_rate = reg.ODR_1KHZ  # 1000 Hz
icm.gyro_data_rate = reg.ODR_1KHZ  # 1000 Hz
```

### Set Power Modes

```python
# Low-noise mode (high performance)
icm.set_power_mode(accel_mode=reg.ACCEL_MODE_LN, gyro_mode=reg.GYRO_MODE_LN)

# Low-power mode (battery applications)
icm.set_power_mode(accel_mode=reg.ACCEL_MODE_LP, gyro_mode=reg.GYRO_MODE_OFF)
```

## FIFO Usage

```python
# Enable FIFO
icm.enable_fifo(accel=True, gyro=True, temp=True, mode="stream")

# Check FIFO count
count = icm.fifo_count
print(f"FIFO has {count} bytes ({count//16} packets)")

# Read FIFO packet
if count >= 16:
    packet = icm.read_fifo()
    accel = packet['accel']  # m/s²
    gyro = packet['gyro']  # rad/s
    temp = packet['temp']  # °C

# Flush FIFO
icm.flush_fifo()

# Disable FIFO
icm.disable_fifo()
```

## Motion Detection

### Wake-on-Motion

```python
# Configure interrupt pin
icm.configure_interrupt(pin=1, polarity="high", mode="latch", drive="push-pull")

# Enable wake-on-motion (threshold in units of ~3.9mg)
icm.enable_wake_on_motion(threshold=50, axes="all", int_pin=1)

# Check for motion
status = icm.read_interrupt_status()
if status['wom_x'] or status['wom_y'] or status['wom_z']:
    print("Motion detected!")

# Disable motion detection
icm.disable_motion_detection()
```

### Tap Detection

```python
# Enable tap detection
icm.enable_tap_detection(mode="low-noise", int_pin=1)

# Check for taps
status = icm.read_interrupt_status()
if status['tap']:
    tap_info = icm.read_tap_info()
    print(f"{tap_info['count']} tap on {tap_info['axis']} axis")
```

## Pin Mapping

### ESP32 WROOM / Lolin D32

**I2C (Default):**
- SDA: GPIO21
- SCL: GPIO22

**SPI (VSPI):**
- MOSI: GPIO23
- MISO: GPIO19
- SCK: GPIO18
- CS: GPIO5 (configurable)

### RP2040 (Raspberry Pi Pico)

**I2C:**
- I2C0: SDA=GP0, SCL=GP1
- I2C1: SDA=GP2, SCL=GP3

**SPI:**
- SPI0: MOSI=GP3, MISO=GP4, SCK=GP2
- SPI1: MOSI=GP11, MISO=GP12, SCK=GP10

## API Differences from CircuitPython

The MicroPython port maintains API compatibility with the CircuitPython version, with only minor differences in initialization:

### Import Changes

**CircuitPython:**
```python
import board
import busio
i2c = busio.I2C(board.SCL, board.SDA)
```

**MicroPython:**
```python
from machine import I2C, Pin
i2c = I2C(0, scl=Pin(22), sda=Pin(21), freq=400000)
```

### Pin Specification

**CircuitPython:** Uses symbolic names (`board.GP1`)
**MicroPython:** Uses numeric pins (`Pin(1)`)

### Error Types

- I2C/SPI errors raise `OSError` in MicroPython (vs `RuntimeError` in CircuitPython)

All other APIs (properties, methods, constants) are identical.

## Examples

See the `examples/` directory for complete examples:

- `basic_i2c_esp32.py` - Basic I2C usage
- `basic_spi_esp32.py` - Basic SPI usage
- `fifo_example_esp32.py` - FIFO buffer usage
- `wake_on_motion_esp32.py` - Wake-on-motion detection
- `tap_detection_esp32.py` - Tap detection

## Testing

Run the test scripts to validate your hardware setup:

```bash
# Basic functionality tests
mpremote run tests/test_basic_functionality.py

# FIFO and advanced features tests
mpremote run tests/test_fifo_and_advanced.py
```

## Performance

Tested on ESP32 @ 240MHz:
- Read rate: 50-100 Hz for all sensors
- FIFO read rate: ~1000 packets/sec
- Memory usage: ~15KB (including buffers)

## Memory Optimization

The driver uses several techniques to minimize memory usage:
- Pre-allocated buffers (no allocations in read loops)
- `micropython.const()` for ESP32 constant optimization
- Reused buffers for repeated operations
- Minimal temporary object creation

For aerospace and real-time applications, this ensures predictable GC behavior.

## Troubleshooting

### Sensor not found

- Check wiring (SDA, SCL for I2C; MOSI, MISO, SCK, CS for SPI)
- Verify I2C address (0x69 if SDO high, 0x68 if SDO low)
- Check power supply (3.3V, not 5V)
- Try I2C bus scan: `i2c.scan()`

### Incorrect readings

- Ensure sensor is stationary for gyro calibration
- Check range settings match your application
- Verify ODR is compatible with power mode
- Allow settling time after power mode changes

### FIFO issues

- Ensure reading faster than FIFO fill rate
- At 1kHz ODR, FIFO fills in ~128ms
- Use FIFO flush if overflow suspected
- Check FIFO mode (stream vs stop)

## License

MIT License - See LICENSE file

## Credits

- Original CircuitPython driver: Yassine Bekkari
- Based on DFRobot_ICM42688
- MicroPython port: Yassine Bekkari

## Support

For issues, questions, or contributions:
- GitHub: https://github.com/yassinebkr/DFRobot_ICM42688
- Issues: https://github.com/yassinebkr/DFRobot_ICM42688/issues

## References

- ICM-42688-P Datasheet: [TDK InvenSense](https://invensense.tdk.com/products/motion-tracking/6-axis/icm-42688-p/)
- MicroPython Documentation: [docs.micropython.org](https://docs.micropython.org)
