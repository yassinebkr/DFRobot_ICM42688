# Feather RP2040 + ICM42688P Quick Start Guide

Quick reference for connecting the ICM42688P sensor to the **Adafruit Feather RP2040** board.

## Hardware Overview

- **Feather RP2040**: RP2040-based board with CircuitPython support, 264KB SRAM, 8MB Flash
- **ICM42688P**: 6-axis IMU (accelerometer + gyroscope) with I2C/SPI interface
- **RFM95 LoRa Radio** (optional): Can coexist with ICM42688P using different CS pins

## Wiring (I2C - Recommended)

### Simple 4-Wire Connection

```
ICM42688P Board   Feather RP2040
───────────────   ──────────────
5V (pin label)──  3V           ← Connect board's "5V" pin to Feather's 3.3V!
GND           ──  GND
SDA           ──  SDA (GPIO2)
SCL           ──  SCL (GPIO3)
```

**CRITICAL**: Despite the "5V" label, connect to Feather's **3V** pin (3.3V). The ICM42688P board has an onboard regulator.

### Pin Locations on Feather RP2040

```
      ┌─────────[USB]─────────┐
      │                       │
  RST ○                       ○ VBUS (5V)
  3V  ○ ← Power               ○ EN
  ARef○                       ○ VBAT
  GND ○ ← Ground              ○ D13 (LED)
  A0  ○                       ○ D12
  A1  ○                       ○ D11
  A2  ○                       ○ D10
  A3  ○                       ○ D9
  D24 ○                       ○ D6
  D25 ○                       ○ D5
  SCK ○                       ○ SCL (GPIO3) ← I2C Clock
  MOSI○                       ○ SDA (GPIO2) ← I2C Data
  MISO○                       ○
      └───────────────────────┘
```

## Installation Steps

### 1. Install CircuitPython

Download CircuitPython **8.0.0 or newer** for Feather RP2040:
- Visit: https://circuitpython.org/board/adafruit_feather_rp2040/
- Hold BOOTSEL button while plugging USB
- Copy `.uf2` file to RPI-RP2 drive
- Board reboots as `CIRCUITPY` drive

### 2. Install Library

Copy the library to your Feather:

```bash
# Copy library folder to CIRCUITPY drive
cp -r circuitpython/adafruit_icm42688 /media/CIRCUITPY/lib/
```

Or use `circup`:
```bash
circup install adafruit_icm42688
```

### 3. Test Connection (I2C)

Create `code.py` on CIRCUITPY drive:

```python
import board
import busio
from adafruit_icm42688 import ICM42688

# Initialize I2C (uses board.SCL and board.SDA)
i2c = busio.I2C(board.SCL, board.SDA, frequency=400000)
icm = ICM42688(i2c)

print("ICM42688P Test on Feather RP2040")
print(f"Accelerometer: {icm.acceleration} m/s²")
print(f"Gyroscope: {icm.gyro} rad/s")
```

**Expected I2C Address**:
- `0x69` if AD0 pin is HIGH (default on most breakouts)
- `0x68` if AD0 pin is LOW (or connected to GND)

## RFM95 LoRa Radio Compatibility

If using RFM95 LoRa FeatherWing, **I2C is the best choice** for ICM42688P:

✅ **I2C Mode**: No conflicts - LoRa uses SPI, IMU uses I2C
❌ **SPI Mode**: Requires careful CS pin management

### Pin Usage with RFM95

```
RFM95 LoRa (SPI):     ICM42688P (I2C):
- D11 (MOSI)          - SDA (GPIO2)
- D12 (MISO)          - SCL (GPIO3)
- D13 (SCK)           - No SPI pins used
- D10 (CS)            - No conflicts
- D9  (RST)
- D6  (IRQ)
```

## Troubleshooting

### "No I2C device found at 0x69"

1. **Check wiring**: Verify SDA/SCL connections
2. **Check address**: Try 0x68 if AD0 is grounded
3. **Scan I2C bus**:
   ```python
   import board
   import busio

   i2c = busio.I2C(board.SCL, board.SDA)
   while not i2c.try_lock():
       pass
   print("I2C addresses:", [hex(x) for x in i2c.scan()])
   i2c.unlock()
   ```

### Memory Issues

Feather RP2040 has **264KB SRAM** - plenty for this library. If using many libraries:
- Disable unused features
- Use `.mpy` compiled libraries instead of `.py`

### Power Issues

- **Don't use VBUS (5V)** - Use **3V pin only**
- If sensor behaves erratically: Add 0.1µF capacitor between 3V and GND near sensor

## Performance

- **I2C Speed**: 400kHz (default), up to 1MHz possible
- **Sample Rate**: Up to 32kHz (accelerometer), 8kHz (gyroscope)
- **RP2040 CPU**: 133MHz dual-core (plenty of headroom)

## Next Steps

- **Detailed wiring**: See `circuitpython/examples/FEATHER_RP2040_WIRING_GUIDE.md`
- **Example code**: See `circuitpython/examples/feather_rp2040_i2c_test.py`
- **SPI mode**: Available if needed (see wiring guide)

## Safety Notes

⚠️ **Voltage**: ICM42688P is 3.3V logic - compatible with Feather RP2040
⚠️ **5V Pin**: Despite label, connect to Feather's **3V**, not VBUS (5V)
⚠️ **First Use**: Read SAFETY_CHECK_PROCEDURE.md if unsure about your breakout board
