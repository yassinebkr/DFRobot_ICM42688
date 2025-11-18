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
                    Your Board Pinout

         ┌─────────────────────────────────┐
         │   ┌───────────────────────┐     │
         │   │    ESP32-WROOM-32     │     │
         │   │       Module          │     │
         │   └───────────────────────┘     │
         │                                 │
    GND  │ 1  ○                      ○ 40 │ 3V3   ←─ Power
    P23  │ 2  ○                      ○ 39 │ EN
    P22  │ 3  ○  [USB]    [BOOT]     ○ 38 │ SVP
    TX   │ 4  ○   [○]       [○]      ○ 37 │ SVN
    RX   │ 5  ○                      ○ 36 │ P34
    P21  │ 6  ○                      ○ 35 │ P35
    GND  │ 7  ○                      ○ 34 │ P32
    P19  │ 8  ○                      ○ 33 │ P33
    P18  │ 9  ○                      ○ 32 │ P25
    P5   │ 10 ○                      ○ 31 │ P26
    P17  │ 11 ○                      ○ 30 │ P27
    P16  │ 12 ○                      ○ 29 │ P14
    P4   │ 13 ○                      ○ 28 │ P12
    P0   │ 14 ○                      ○ 27 │ GND
    P2   │ 15 ○                      ○ 26 │ P13
    P15  │ 16 ○                      ○ 25 │ SD2
    SD1  │ 17 ○                      ○ 24 │ SD3
    SD0  │ 18 ○                      ○ 23 │ GND
    CLK  │ 19 ○                      ○ 22 │ 5V
    GND  │ 20 ○                      ○ 21 │ GND
         └─────────────────────────────────┘

I2C Default Pins:
  P21 = GPIO21 (SDA) - LEFT side, Pin 6
  P22 = GPIO22 (SCL) - LEFT side, Pin 3

SPI Default Pins:
  P23 = GPIO23 (MOSI) - LEFT side, Pin 2
  P19 = GPIO19 (MISO) - LEFT side, Pin 8
  P18 = GPIO18 (SCK)  - LEFT side, Pin 9
  P5  = GPIO5  (CS)   - LEFT side, Pin 10
```

---

## I2C Wiring Diagram

### Standard I2C Connection

```
ICM42688 Breakout          ESP32-WROOM-32 DevKit (Your Board)
┌────────────────┐         ┌──────────────────────────────┐
│                │         │                              │
│  VCC/VDD  ○────┼─────────┤ RIGHT Pin 40 (3V3)           │
│  GND      ○────┼─────────┤ LEFT Pin 1,7,20 or RIGHT 21,23,27 (GND) │
│  SDA      ○────┼─────────┤ LEFT Pin 6 (P21)             │
│  SCL      ○────┼─────────┤ LEFT Pin 3 (P22)             │
│  INT1     ○────┼─────────┤ LEFT Pin 2 (P23) - Optional  │
│  INT2     ○────┼─────────┤ LEFT Pin 8 (P19) - Optional  │
│  SDO/AD0  ○ ┬  │         │                              │
│           │ │  │         Pull HIGH for 0x69             │
│           │ └──┼─────────┤ 3V3 (default)                │
│           │    │    OR   │                              │
│           └────┼─────────┤ GND (for 0x68)               │
│                │         │                              │
└────────────────┘         └──────────────────────────────┘

Connections:
  VCC     → 3V3   (RIGHT side, Pin 40 - top pin)
  GND     → GND   (Multiple: LEFT 1,7,20  RIGHT 21,23,27)
  SDA     → P21   (LEFT side, Pin 6)
  SCL     → P22   (LEFT side, Pin 3)
  INT1    → P23   (LEFT side, Pin 2) - Optional
  INT2    → P19   (LEFT side, Pin 8) - Optional
  SDO/AD0 → 3V3   (Address 0x69) or GND (Address 0x68)
```

### I2C Pin Details

| Function | GPIO | Physical Pin | Side | Notes |
|----------|------|--------------|------|-------|
| SDA | GPIO21 (P21) | Pin 6 | LEFT | Default I2C Data |
| SCL | GPIO22 (P22) | Pin 3 | LEFT | Default I2C Clock |
| Power | 3.3V (3V3) | Pin 40 | RIGHT | **NEVER USE 5V!** |
| Ground | GND | Pins 1,7,20 (LEFT) or 21,23,27 (RIGHT) | Both | Multiple GND pins available |

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
ICM42688 Breakout          ESP32-WROOM-32 DevKit (Your Board)
┌────────────────┐         ┌────────────────────────────┐
│                │         │                            │
│  VCC      ○────┼─────────┤ RIGHT Pin 40 (3V3)         │
│  GND      ○────┼─────────┤ LEFT Pin 1,7,20 or RIGHT 21,23,27 (GND) │
│  MOSI/SDI ○────┼─────────┤ LEFT Pin 2 (P23/MOSI)      │
│  MISO/SDO ○────┼─────────┤ LEFT Pin 8 (P19/MISO)      │
│  SCK/SCLK ○────┼─────────┤ LEFT Pin 9 (P18/SCK)       │
│  CS/nCS   ○────┼─────────┤ LEFT Pin 10 (P5)           │
│  INT1     ○────┼─────────┤ LEFT Pin 13 (P4) - Optional│
│  INT2     ○────┼─────────┤ LEFT Pin 16 (P15) - Optional│
│                │         │                            │
└────────────────┘         └────────────────────────────┘

Connections:
  VCC      → 3V3  (RIGHT Pin 40)
  GND      → GND  (LEFT 1,7,20 or RIGHT 21,23,27)
  MOSI/SDI → P23  (LEFT Pin 2)
  MISO/SDO → P19  (LEFT Pin 8)
  SCK      → P18  (LEFT Pin 9)
  CS       → P5   (LEFT Pin 10) - Any GPIO works
  INT1     → P4   (LEFT Pin 13) - Optional
  INT2     → P15  (LEFT Pin 16) - Optional
```

### SPI Pin Details

| Function | GPIO | Physical Pin | Side | SPI Bus | Notes |
|----------|------|--------------|------|---------|-------|
| MOSI | GPIO23 (P23) | Pin 2 | LEFT | VSPI | SPI Data Out (ESP32 → Sensor) |
| MISO | GPIO19 (P19) | Pin 8 | LEFT | VSPI | SPI Data In (Sensor → ESP32) |
| SCK | GPIO18 (P18) | Pin 9 | LEFT | VSPI | SPI Clock |
| CS | GPIO5 (P5) | Pin 10 | LEFT | - | Chip Select (any GPIO) |

**Alternative: HSPI Pins (if VSPI conflicts)**
| Function | GPIO | Physical Pin | Side |
|----------|------|--------------|------|
| MOSI | GPIO13 (P13) | Pin 26 | RIGHT |
| MISO | GPIO12 (P12) | Pin 28 | RIGHT |
| SCK | GPIO14 (P14) | Pin 29 | RIGHT |
| CS | Any GPIO | User choice | - |

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
    Your Board

    RIGHT Pin 40 (3V3) ──────┬─────────── VCC
                             │
    LEFT  Pin 6  (P21) ──────┼─────────── SDA
                             │
    LEFT  Pin 3  (P22) ──────┼─────────── SCL
                             │
    LEFT  Pin 1  (GND) ──────┴─────────── GND


Connect on breadboard:
  RIGHT Pin 40:  3V3   → Red power rail
  LEFT Pin 1:    GND   → Blue/Black ground rail
  LEFT Pin 6:    P21   → ICM42688 SDA
  LEFT Pin 3:    P22   → ICM42688 SCL
  ICM42688 VCC:        → Red power rail
  ICM42688 GND:        → Ground rail
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
