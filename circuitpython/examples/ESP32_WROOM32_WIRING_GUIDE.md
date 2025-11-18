# ICM42688 + ESP32-WROOM-32 Wiring Guide

Complete hardware setup guide for testing the ICM42688 library with ESP32-WROOM-32 40-pin development boards.

## Board Compatibility

**Supported ESP32-WROOM-32 Boards:**
- ESP32-DevKitC V4 (WROOM-32E/32UE)
- DOIT ESP32 DevKit v1
- ESP32-WROOM-32 Generic 40-pin boards
- NodeMCU-32S
- Any ESP32-WROOM-32 based 40-pin development board

**CircuitPython Downloads:**
- ESP32-DevKitC V4: https://circuitpython.org/board/espressif_esp32_devkitc_v4_wroom_32e/
- DOIT ESP32 DevKit v1: https://circuitpython.org/board/doit_esp32_devkit_v1/

---

## ESP32-WROOM-32 40-Pin Pinout Reference

```
                    ESP32-WROOM-32 DevKit (40-pin)
                    ==============================

         ┌─────────────────────────────────┐
         │   ┌───────────────────────┐     │
         │   │    ESP32-WROOM-32     │     │
         │   │       Module          │     │
         │   └───────────────────────┘     │
         │                                 │
    3V3  │ 1  ○                      ○ 40 │ GND
    EN   │ 2  ○                      ○ 39 │ GND
   VP36  │ 3  ○                      ○ 38 │ IO23 (MOSI)
   VN39  │ 4  ○                      ○ 37 │ IO22 (SCL) ←─ I2C
   IO34  │ 5  ○                      ○ 36 │ TX0
   IO35  │ 6  ○                      ○ 35 │ RX0
   IO32  │ 7  ○                      ○ 34 │ IO21 (SDA) ←─ I2C
   IO33  │ 8  ○                      ○ 33 │ GND
   IO25  │ 9  ○                      ○ 32 │ IO19 (MISO)
   IO26  │ 10 ○                      ○ 31 │ IO18 (SCK)
   IO27  │ 11 ○                      ○ 30 │ IO5
   IO14  │ 12 ○  [USB]    [BOOT]     ○ 29 │ IO17
   IO12  │ 13 ○   [○]       [○]      ○ 28 │ IO16
    GND  │ 14 ○                      ○ 27 │ IO4
   IO13  │ 15 ○                      ○ 26 │ IO0
    SD2  │ 16 ○                      ○ 25 │ IO2 (LED)
    SD3  │ 17 ○                      ○ 24 │ IO15
    CMD  │ 18 ○                      ○ 23 │ GND
    VIN  │ 19 ○                      ○ 22 │ 3V3
    GND  │ 20 ○                      ○ 21 │ GND
         └─────────────────────────────────┘
```

---

## I2C Wiring Diagram

### Standard I2C Connection

```
ICM42688 Breakout          ESP32-WROOM-32 DevKit
┌────────────────┐         ┌──────────────────────┐
│                │         │                      │
│  VCC/VDD  ○────┼─────────┤ Pin 1 (3.3V)         │
│  GND      ○────┼─────────┤ Pin 14/20/21 (GND)   │
│  SDA      ○────┼─────────┤ Pin 34 (IO21/SDA)    │
│  SCL      ○────┼─────────┤ Pin 37 (IO22/SCL)    │
│  INT1     ○────┼─────────┤ Pin 38 (IO23)        │ (optional)
│  INT2     ○────┼─────────┤ Pin 32 (IO19)        │ (optional)
│  SDO/AD0  ○ ┬  │         │                      │
│           │ │  │         Pull HIGH for 0x69     │
│           │ └──┼─────────┤ 3.3V (default)       │
│           │    │    OR   │                      │
│           └────┼─────────┤ GND (for 0x68)       │
│                │         │                      │
└────────────────┘         └──────────────────────┘

Connections:
  VCC     → 3.3V  (Pin 1 or 22)
  GND     → GND   (Pin 14, 20, 21, 23, 33, 39, or 40)
  SDA     → IO21  (Pin 34)
  SCL     → IO22  (Pin 37)
  INT1    → IO23  (Pin 38) - Optional for interrupts
  INT2    → IO19  (Pin 32) - Optional for interrupts
  SDO/AD0 → 3.3V  (Address 0x69) or GND (Address 0x68)
```

### I2C Pin Details

| Function | GPIO | Physical Pin | Notes |
|----------|------|--------------|-------|
| SDA | GPIO21 | Pin 34 | Default I2C Data |
| SCL | GPIO22 | Pin 37 | Default I2C Clock |
| Power | 3.3V | Pin 1 or 22 | **NEVER USE 5V!** |
| Ground | GND | Pin 14, 20, 21, 23, 33, 39, 40 | Multiple GND pins available |

**I2C Frequency Options:**
- 100 kHz: Standard mode (conservative)
- 400 kHz: Fast mode (recommended)
- 1 MHz: Fast-mode Plus (maximum)

**Pull-up Resistors:**
- Most ICM42688 breakouts have built-in 10kΩ pull-ups
- If needed, add external 2.2kΩ - 4.7kΩ resistors to SDA and SCL

---

## SPI Wiring Diagram

### VSPI Connection (Default Hardware SPI)

```
ICM42688 Breakout          ESP32-WROOM-32 DevKit
┌────────────────┐         ┌──────────────────────┐
│                │         │                      │
│  VCC      ○────┼─────────┤ Pin 1 (3.3V)         │
│  GND      ○────┼─────────┤ Pin 14/20/21 (GND)   │
│  MOSI/SDI ○────┼─────────┤ Pin 38 (IO23/MOSI)   │
│  MISO/SDO ○────┼─────────┤ Pin 32 (IO19/MISO)   │
│  SCK/SCLK ○────┼─────────┤ Pin 31 (IO18/SCK)    │
│  CS/nCS   ○────┼─────────┤ Pin 29 (IO5)         │
│  INT1     ○────┼─────────┤ Pin 27 (IO4)         │ (optional)
│  INT2     ○────┼─────────┤ Pin 24 (IO15)        │ (optional)
│                │         │                      │
└────────────────┘         └──────────────────────┘

Connections:
  VCC      → 3.3V  (Pin 1 or 22)
  GND      → GND   (Pin 14, 20, 21, 23, 33, 39, or 40)
  MOSI/SDI → IO23  (Pin 38)
  MISO/SDO → IO19  (Pin 32)
  SCK      → IO18  (Pin 31)
  CS       → IO5   (Pin 29) - Any GPIO works
  INT1     → IO4   (Pin 27) - Optional
  INT2     → IO15  (Pin 24) - Optional
```

### SPI Pin Details

| Function | GPIO | Physical Pin | SPI Bus | Notes |
|----------|------|--------------|---------|-------|
| MOSI | GPIO23 | Pin 38 | VSPI | SPI Data Out (ESP32 → Sensor) |
| MISO | GPIO19 | Pin 32 | VSPI | SPI Data In (Sensor → ESP32) |
| SCK | GPIO18 | Pin 31 | VSPI | SPI Clock |
| CS | GPIO5 | Pin 29 | - | Chip Select (any GPIO) |

**Alternative: HSPI Pins (if VSPI conflicts)**
| Function | GPIO | Physical Pin |
|----------|------|--------------|
| MOSI | GPIO13 | Pin 15 |
| MISO | GPIO12 | Pin 13 |
| SCK | GPIO14 | Pin 12 |
| CS | Any GPIO | User choice |

**SPI Speed Options:**
- 1 MHz: Conservative (always works)
- 10 MHz: Recommended default
- 24 MHz: Maximum ICM42688 speed

---

## Physical Wiring Examples

### Breadboard Layout (I2C)

```
    ESP32-WROOM-32                    ICM42688
    DevKit (40-pin)                   Breakout

    Pin 1  (3.3V) ──────┬─────────── VCC
                        │
    Pin 34 (IO21) ──────┼─────────── SDA
                        │
    Pin 37 (IO22) ──────┼─────────── SCL
                        │
    Pin 14 (GND)  ──────┴─────────── GND


Connect on breadboard:
  Row 1:  3.3V  → Red power rail
  Row 2:  GND   → Blue/Black ground rail
  Row 5:  IO21  → ICM42688 SDA
  Row 7:  IO22  → ICM42688 SCL
  Row 8:  VCC   → Red power rail
  Row 9:  GND   → Ground rail
```

### Soldered Connection Recommendations

For aerospace/flight applications, **DO NOT USE BREADBOARDS**. Use one of:

1. **Perfboard with soldered connections**
   - Solder all connections
   - Add strain relief for wires
   - Use heat-shrink tubing

2. **Custom PCB**
   - Design PCB with ESP32 module socket
   - Surface-mount ICM42688
   - Add filtering capacitors

3. **Wire-to-board connectors**
   - JST-SH or JST-PH connectors
   - Secure with hot glue after testing
   - Use twisted-pair shielded cable for I2C/SPI

---

## Power Considerations

### Voltage Requirements

**CRITICAL:** ICM42688 is **3.3V ONLY**

```
✓ CORRECT:
  ESP32 3.3V pin → ICM42688 VCC

✗ WRONG (WILL DAMAGE SENSOR):
  ESP32 VIN/5V → ICM42688 VCC
  USB 5V → ICM42688 VCC
```

### Power Consumption

| Mode | ESP32 | ICM42688 | Total |
|------|-------|----------|-------|
| Active (WiFi) | ~160 mA | 2.5 mA | ~163 mA |
| Active (No WiFi) | ~80 mA | 2.5 mA | ~83 mA |
| Low Power | ~15 mA | 0.5 mA | ~16 mA |
| Deep Sleep | ~10 µA | 1 µA | ~11 µA |

**Recommended Power Supply:**
- USB power: 500 mA minimum
- Battery: LiPo 3.7V with 500+ mAh
- External: 3.3V regulated, 200+ mA

### Decoupling Capacitors

**For reliable operation, add capacitors:**

```
ESP32 3.3V pin
     │
     ├─── 100µF electrolytic (bulk)
     │
     ├─── 10µF ceramic (near ESP32)
     │
     ├─── 0.1µF ceramic (near ICM42688)
     │
     └─── ICM42688 VCC pin
```

Place capacitors as close to IC pins as possible.

---

## I2C Address Configuration

The ICM42688 supports two I2C addresses:

| SDO/AD0 Pin State | I2C Address | Typical Use |
|-------------------|-------------|-------------|
| Connected to VCC (3.3V) | 0x69 | **Default** on most breakouts |
| Connected to GND | 0x68 | Alternative address |
| Floating | 0x69 | Usually has internal pull-up |

**To change address:**
1. Locate SDO/AD0 pin on breakout (may be labeled AD0, SA0, or ADR)
2. Connect to GND for 0x68
3. Connect to 3.3V for 0x69 (default)

---

## Interrupt Wiring (Optional)

The ICM42688 has two interrupt pins for motion detection, tap detection, etc.

### INT1 Wiring

```
ICM42688 INT1 → ESP32 GPIO23 (Pin 38)
```

**Code example:**
```python
import digitalio
int1_pin = digitalio.DigitalInOut(board.IO23)
int1_pin.direction = digitalio.Direction.INPUT
```

### INT2 Wiring

```
ICM42688 INT2 → ESP32 GPIO19 (Pin 32)
```

**Configure in library:**
```python
icm.configure_interrupt(pin=1, polarity="high", mode="latch")
icm.enable_wake_on_motion(threshold=50, int_pin=1)
```

---

## Common Wiring Issues

### Problem: I2C scan finds no devices

**Solutions:**
1. Check VCC is 3.3V (NOT 5V!)
2. Verify GND connection
3. Check SDA on GPIO21, SCL on GPIO22
4. Swap SDA and SCL if wired backwards
5. Add pull-up resistors (2.2kΩ - 4.7kΩ)
6. Try lower I2C frequency (100kHz)
7. Check ICM42688 power LED (if available)

### Problem: SPI initialization fails

**Solutions:**
1. Verify CS pin is connected
2. Check MOSI/MISO not swapped
3. Ensure SCK is connected
4. Lower SPI baudrate to 1MHz
5. Add short wires (<10cm for breadboard)
6. Check CS pin is correct GPIO in code

### Problem: Sensor reads 0x00 for all values

**Solutions:**
1. Sensor not powered - check VCC
2. Wrong I2C address - try 0x68 instead of 0x69
3. SPI mode mismatch - check CS pin
4. Bad solder joint - reflow connections
5. Damaged sensor - try different ICM42688

### Problem: Gravity not showing ~9.8 m/s²

**Solutions:**
1. Sensor is rotated - check Z-axis orientation
2. Wrong accelerometer range - verify ±16g setting
3. Sensor not initialized - check WHO_AM_I register
4. Need calibration - run calibration routine

---

## Testing Checklist

### Pre-Connection Checks
- [ ] Verify ESP32 board is 3.3V output
- [ ] Check ICM42688 breakout voltage rating
- [ ] Confirm CircuitPython installed on ESP32
- [ ] Libraries copied to /lib/ folder
- [ ] Measure ESP32 3.3V pin with multimeter (should be 3.2-3.4V)

### Post-Connection Checks (I2C)
- [ ] Measure VCC at ICM42688 (should be 3.2-3.4V)
- [ ] Check continuity: ESP32 GND to ICM42688 GND
- [ ] Run I2C scan, confirm 0x68 or 0x69 detected
- [ ] Run esp32_wroom32_i2c_test.py
- [ ] Verify Z-axis reads ~9.8 m/s² when flat

### Post-Connection Checks (SPI)
- [ ] Measure VCC at ICM42688 (3.2-3.4V)
- [ ] Verify CS pin goes LOW during communication (oscilloscope/logic analyzer)
- [ ] Run esp32_wroom32_spi_test.py
- [ ] Test multiple SPI speeds (1MHz, 10MHz, 24MHz)
- [ ] Verify FIFO read works

---

## Next Steps

1. **Run I2C test:** `esp32_wroom32_i2c_test.py`
2. **Run SPI test:** `esp32_wroom32_spi_test.py`
3. **Test FIFO:** Verify high-speed data collection
4. **Test interrupts:** Wake-on-motion, tap detection
5. **Long-term test:** Run for 24+ hours
6. **Document results:** Report any issues found

---

## Safety Notes

⚠️ **CRITICAL WARNINGS:**

1. **NEVER apply 5V to ICM42688 VCC** - Permanent damage will occur
2. **Ground ESP32 and sensor to same GND** - Different grounds cause communication failure
3. **Keep wires short** - Long wires (>30cm) cause I2C/SPI errors
4. **Add decoupling capacitors** - Prevents power supply noise
5. **Check connections before power** - Use continuity tester

---

## Resources

- **CircuitPython Docs:** https://docs.circuitpython.org/
- **ESP32 Pinout:** https://docs.espressif.com/projects/esp-idf/en/latest/esp32/hw-reference/
- **ICM42688 Datasheet:** https://invensense.tdk.com/products/motion-tracking/6-axis/icm-42688-p/
- **Adafruit Bus Device:** https://github.com/adafruit/Adafruit_CircuitPython_BusDevice

---

**Document Version:** 1.0
**Last Updated:** 2025
**Author:** Yassine Bekkari
