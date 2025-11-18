# Feather RP2040 + ICM42688P Detailed Wiring Guide

Complete wiring reference for connecting ICM42688P sensor to Adafruit Feather RP2040.

## Table of Contents
- [Board Pinout](#board-pinout)
- [I2C Wiring (Recommended)](#i2c-wiring-recommended)
- [SPI Wiring (Advanced)](#spi-wiring-advanced)
- [RFM95 LoRa Compatibility](#rfm95-lora-compatibility)
- [Example Code](#example-code)

## Board Pinout

### Feather RP2040 Full Pinout

```
                ┌─────────[USB-C]─────────┐
                │                         │
            RST ○ 1                   28 ○ VBUS (5V from USB)
   Power →   3V ○ 2                   27 ○ EN (Enable)
           ARef ○ 3                   26 ○ VBAT (Battery)
  Ground →  GND ○ 4                   25 ○ D13 (LED, SCK)
             A0 ○ 5                   24 ○ D12 (MISO)
             A1 ○ 6                   23 ○ D11 (MOSI)
             A2 ○ 7                   22 ○ D10
             A3 ○ 8                   21 ○ D9
            D24 ○ 9                   20 ○ D6
            D25 ○ 10                  19 ○ D5
            SCK ○ 11                  18 ○ SCL (GPIO3) ← I2C Clock
           MOSI ○ 12                  17 ○ SDA (GPIO2) ← I2C Data
           MISO ○ 13                  16 ○
                │                         │
                └─────────────────────────┘
```

### ICM42688P Breakout Board Pinout

```
┌──────────────────┐
│   ICM42688P      │
│   Breakout       │
├──────────────────┤
│ 5V (labeled 5V)  │ ← Power input (accepts 3.3V-5V, has onboard regulator)
│ GND              │ ← Ground
│ SDA              │ ← I2C Data / SPI MOSI
│ SCL              │ ← I2C Clock / SPI SCK
│ SDO              │ ← SPI MISO (not used in I2C mode)
│ CS               │ ← SPI Chip Select (tie HIGH for I2C)
│ AD0              │ ← I2C Address Select (HIGH=0x69, LOW=0x68)
│ INT1             │ ← Interrupt 1 (optional)
│ INT2             │ ← Interrupt 2 (optional)
└──────────────────┘
```

## I2C Wiring (Recommended)

### Why I2C?

✅ **Simplest**: Only 4 wires needed
✅ **Compatible with RFM95**: No pin conflicts
✅ **Sufficient speed**: 400kHz handles most applications
✅ **Multi-device**: Can share bus with other I2C sensors

### Basic I2C Connection

```
ICM42688P Pin             Feather RP2040 Pin
─────────────────────     ──────────────────
"5V" (labeled 5V)    ──── 3V (Pin 2) ← Use 3.3V, not VBUS!
GND                  ──── GND (Pin 4)
SDA                  ──── SDA (Pin 17, GPIO2)
SCL                  ──── SCL (Pin 18, GPIO3)
CS                   ──── 3V (tie HIGH for I2C mode)
SDO                  ──── (leave disconnected)
AD0                  ──── 3V (for 0x69) or GND (for 0x68)
```

### Visual Wiring Diagram (I2C)

```
   ICM42688P                          Feather RP2040
   ─────────                          ──────────────

   [5V]────────────────────────────── [3V] Pin 2
   [GND]───────────────────────────── [GND] Pin 4
   [SDA]───────────────────────────── [SDA] Pin 17 (GPIO2)
   [SCL]───────────────────────────── [SCL] Pin 18 (GPIO3)
   [CS]────────┐
               └─────────────────────  [3V] Pin 2 (tie HIGH)
   [AD0]───────┐
               └─────────────────────  [3V] Pin 2 (for address 0x69)
                                       or [GND] (for address 0x68)
```

### I2C Example Code

```python
import board
import busio
from adafruit_icm42688 import ICM42688

# Initialize I2C bus
i2c = busio.I2C(board.SCL, board.SDA, frequency=400000)

# Create sensor object
# Default address is 0x69 (AD0 HIGH)
icm = ICM42688(i2c)

# Or specify address if AD0 is LOW:
# icm = ICM42688(i2c, address=0x68)

# Read sensor data
while True:
    accel_x, accel_y, accel_z = icm.acceleration
    gyro_x, gyro_y, gyro_z = icm.gyro

    print(f"Accel: X={accel_x:6.2f} Y={accel_y:6.2f} Z={accel_z:6.2f} m/s²")
    print(f"Gyro:  X={gyro_x:6.2f} Y={gyro_y:6.2f} Z={gyro_z:6.2f} rad/s")
```

## SPI Wiring (Advanced)

### Why SPI?

✅ **Faster**: Up to 24MHz clock (vs 1MHz I2C max)
✅ **Lower latency**: Direct communication
❌ **More wires**: 7 connections vs 4 for I2C
❌ **Conflicts with RFM95**: Both use SPI bus (requires CS management)

### Basic SPI Connection

```
ICM42688P Pin             Feather RP2040 Pin
─────────────────────     ──────────────────
"5V" (labeled 5V)    ──── 3V (Pin 2)
GND                  ──── GND (Pin 4)
SDA (MOSI)           ──── MOSI (Pin 12, D11, GPIO11)
SCL (SCK)            ──── SCK (Pin 11, D13, GPIO14)
SDO (MISO)           ──── MISO (Pin 13, D12, GPIO12)
CS                   ──── D5 (Pin 19, GPIO7) ← or any free GPIO
```

### Visual Wiring Diagram (SPI)

```
   ICM42688P                          Feather RP2040
   ─────────                          ──────────────

   [5V]────────────────────────────── [3V] Pin 2
   [GND]───────────────────────────── [GND] Pin 4
   [SDA]───────────────────────────── [MOSI] Pin 12 (D11, GPIO11)
   [SCL]───────────────────────────── [SCK] Pin 11 (D13, GPIO14)
   [SDO]───────────────────────────── [MISO] Pin 13 (D12, GPIO12)
   [CS]────────────────────────────── [D5] Pin 19 (GPIO7)
```

### SPI Example Code

```python
import board
import busio
import digitalio
from adafruit_icm42688 import ICM42688

# Initialize SPI bus
spi = busio.SPI(board.SCK, MOSI=board.MOSI, MISO=board.MISO)

# Create chip select pin
cs = digitalio.DigitalInOut(board.D5)
cs.direction = digitalio.Direction.OUTPUT
cs.value = True  # CS is active LOW

# Create sensor object
icm = ICM42688(spi, cs)

# Read sensor data
while True:
    accel_x, accel_y, accel_z = icm.acceleration
    gyro_x, gyro_y, gyro_z = icm.gyro

    print(f"Accel: X={accel_x:6.2f} Y={accel_y:6.2f} Z={accel_z:6.2f} m/s²")
    print(f"Gyro:  X={gyro_x:6.2f} Y={gyro_y:6.2f} Z={gyro_z:6.2f} rad/s")
```

## RFM95 LoRa Compatibility

### Using Both ICM42688P and RFM95 LoRa Radio

#### Configuration 1: I2C for ICM42688P (Recommended)

```
RFM95 LoRa FeatherWing (SPI):     ICM42688P (I2C):
────────────────────────────      ────────────────
SCK  (D13) ← SPI Clock            SDA (GPIO2) ← I2C Data
MOSI (D11) ← SPI MOSI             SCL (GPIO3) ← I2C Clock
MISO (D12) ← SPI MISO             3V  ← Power
CS   (D10) ← LoRa Chip Select     GND ← Ground
RST  (D9)  ← LoRa Reset
IRQ  (D6)  ← LoRa Interrupt

✅ NO CONFLICTS - Different buses
```

#### Configuration 2: SPI for Both (Advanced)

```
Shared SPI Bus:
───────────────
SCK  (D13) ← Shared by both
MOSI (D11) ← Shared by both
MISO (D12) ← Shared by both

Chip Selects (MUST BE DIFFERENT):
──────────────────────────────────
RFM95:      CS = D10 (LoRa)
ICM42688P:  CS = D5  (IMU)  ← Use different pin!

⚠️  IMPORTANT: Never activate both CS pins simultaneously
```

### Example: I2C IMU + SPI LoRa

```python
import board
import busio
import digitalio
from adafruit_icm42688 import ICM42688
import adafruit_rfm9x

# Initialize I2C for ICM42688P
i2c = busio.I2C(board.SCL, board.SDA, frequency=400000)
imu = ICM42688(i2c)

# Initialize SPI for RFM95 LoRa
spi = busio.SPI(board.SCK, MOSI=board.MOSI, MISO=board.MISO)
cs = digitalio.DigitalInOut(board.D10)
rst = digitalio.DigitalInOut(board.D9)
lora = adafruit_rfm9x.RFM9x(spi, cs, rst, 915.0)  # 915MHz for US

# Read IMU data
accel = imu.acceleration
gyro = imu.gyro

# Send via LoRa
message = f"A:{accel[0]:.1f},{accel[1]:.1f},{accel[2]:.1f}"
lora.send(bytes(message, "utf-8"))
print(f"Sent: {message}")
```

## Example Code

### Complete I2C Test Script

See: `circuitpython/examples/feather_rp2040_i2c_test.py`

This includes:
- I2C bus scanning
- Basic sensor readings
- Range configuration tests
- Data rate tests
- FIFO buffer tests

### Complete SPI Test Script

See: `circuitpython/examples/feather_rp2040_spi_test.py`

This includes:
- SPI speed tests (1MHz - 24MHz)
- High-speed sampling
- Performance benchmarks

## Troubleshooting

### I2C Issues

**No device found at 0x69:**
1. Check AD0 pin - HIGH for 0x69, LOW for 0x68
2. Verify CS pin is tied HIGH (3V) for I2C mode
3. Run I2C scan to find actual address

**Unreliable readings:**
1. Add 4.7kΩ pull-up resistors on SDA/SCL (usually not needed)
2. Shorten wires (keep under 6 inches for 400kHz)
3. Add 0.1µF bypass capacitor near sensor

### SPI Issues

**No communication:**
1. Verify CS pin stays HIGH when idle
2. Check MISO connection (often forgotten)
3. Try lower SPI speed (start at 1MHz)

**Conflicts with RFM95:**
1. Use different CS pins
2. Ensure only one CS is LOW at a time
3. Consider using I2C for IMU instead

### Power Issues

**Sensor not responding:**
1. Measure voltage at sensor: should be 3.0-3.6V
2. Check GND connection
3. Add 0.1µF capacitor between 3V and GND near sensor

**RP2040 resets randomly:**
- Power supply insufficient - use USB 2.0 port (500mA minimum)
- Battery too low - charge or replace

## Performance Benchmarks

### I2C Performance (400kHz)

- **Read latency**: ~2ms per reading
- **Max sample rate**: ~400 Hz (both accel + gyro)
- **CPU usage**: <1% at 400Hz

### SPI Performance (24MHz)

- **Read latency**: ~0.5ms per reading
- **Max sample rate**: ~1500 Hz (both accel + gyro)
- **CPU usage**: <2% at 1500Hz

### RP2040 Specifications

- **CPU**: Dual-core ARM Cortex-M0+ @ 133MHz
- **SRAM**: 264KB
- **Flash**: 8MB (Feather RP2040)
- **I2C**: Up to 1MHz (400kHz recommended)
- **SPI**: Up to 62.5MHz (24MHz safe for ICM42688P)

## Pin Reference Table

### Feather RP2040 Pin Names

| Physical Pin | Silkscreen Label | CircuitPython Name | GPIO Number | Function          |
|--------------|------------------|--------------------|-------------|-------------------|
| 2            | 3V               | -                  | -           | 3.3V Power Output |
| 4            | GND              | -                  | -           | Ground            |
| 17           | SDA              | board.SDA          | GPIO2       | I2C Data          |
| 18           | SCL              | board.SCL          | GPIO3       | I2C Clock         |
| 11           | SCK              | board.SCK          | GPIO14      | SPI Clock         |
| 12           | MOSI             | board.MOSI         | GPIO11      | SPI MOSI          |
| 13           | MISO             | board.MISO         | GPIO12      | SPI MISO          |
| 19           | D5               | board.D5           | GPIO7       | Digital I/O       |
| 20           | D6               | board.D6           | GPIO8       | Digital I/O       |
| 21           | D9               | board.D9           | GPIO9       | Digital I/O       |
| 22           | D10              | board.D10          | GPIO10      | Digital I/O       |
| 28           | VBUS             | -                  | -           | 5V from USB       |

## Safety Notes

⚠️ **Voltage Levels**: Feather RP2040 is 3.3V - do NOT connect 5V to GPIO pins
⚠️ **5V Pin Label**: ICM42688P board may have "5V" label - connect to **3V**, not VBUS
⚠️ **Current Limits**: 3V pin can supply ~500mA total (sufficient for ICM42688P)
⚠️ **Static Protection**: Handle RP2040 board with ESD precautions

## Additional Resources

- **CircuitPython Downloads**: https://circuitpython.org/board/adafruit_feather_rp2040/
- **Feather RP2040 Guide**: https://learn.adafruit.com/adafruit-feather-rp2040-pico
- **ICM42688 Datasheet**: https://invensense.tdk.com/products/motion-tracking/6-axis/icm-42688-p/
- **Library Documentation**: See `circuitpython/README.md`
