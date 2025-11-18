# ICM42688P 5V Breakout Board Guide

**CRITICAL INFORMATION:** Your ICM42688P breakout board (Model: 3031CM42681) has a **5V power input** with onboard voltage regulation.

## Your Board Pinout

```
Board Label: 3031CM42681 (ICM42688P)
Pin Order (top to bottom):
━━━━━━━━━━━━━━━━━━━━━
INT2              ← Interrupt 2 output
INT1              ← Interrupt 1 output
CS                ← SPI Chip Select
SCL/SCLK          ← I2C Clock / SPI Clock
SDA/MOSI/AD0/MISO ← I2C Data / SPI MOSI / I2C Address / SPI MISO
GND               ← Ground
V (5V)            ← Power Input (5V tolerant, outputs 3.3V internally)
```

---

## CRITICAL: Voltage Regulator Board

### What This Means

Your board has an **onboard 5V-to-3.3V voltage regulator**. This means:

✓ **SAFE:** You can connect "V" pin to 5V power
✓ **SAFE:** You can connect "V" pin to 3.3V power
✓ **BETTER:** ESP32's 3.3V is recommended (more efficient)

The regulator powers the ICM42688P chip at 3.3V internally, even when you supply 5V.

### ⚠️ SIGNAL VOLTAGE WARNING

**CRITICAL QUESTION:** Are the signal pins (SDA, SCL, CS, INT1, INT2) 5V tolerant?

Most breakout boards with 5V input have **level shifters** for signals, but some don't.

**ESP32 GPIO pins are NOT 5V tolerant!** They will be damaged by 5V signals.

---

## Safe Connection Strategy

### Option 1: Power with 3.3V (SAFEST FOR ESP32)

Connect "V" pin to ESP32's 3.3V output:

```
ICM42688P Board        ESP32-WROOM-32
───────────────        ──────────────
V (labeled 5V)    ──── 3.3V (Pin 1)    ← SAFE, regulator accepts 3.3V input
GND               ──── GND (Pin 14/20)
SDA/MOSI/AD0/MISO ──── IO21 (Pin 34)   ← Signals will be 3.3V
SCL/SCLK          ──── IO22 (Pin 37)   ← Signals will be 3.3V
CS                ──── Not connected (for I2C mode)
INT1              ──── IO23 (Pin 38)   ← Optional
INT2              ──── IO19 (Pin 32)   ← Optional
```

**Advantages:**
- No risk of 5V damage to ESP32
- Signals are guaranteed 3.3V
- Simpler wiring
- Lower power consumption

**Disadvantage:**
- Voltage regulator may drop out if input is too low (check if board works)

### Option 2: Power with 5V (Need to verify signal levels)

**ONLY IF** your board has level shifters (measure first!):

```
ICM42688P Board        ESP32-WROOM-32 + External 5V
───────────────        ─────────────────────────────
V                 ──── 5V external supply (USB, battery, etc.)
GND               ──── Common ground (ESP32 GND + 5V GND)
SDA/MOSI/AD0/MISO ──── IO21 (Pin 34) ⚠️ ONLY if signals are 3.3V!
SCL/SCLK          ──── IO22 (Pin 37) ⚠️ ONLY if signals are 3.3V!
```

**⚠️ WARNING:** DO NOT connect signal pins until you verify voltage levels (see testing procedure below)!

---

## Pin Function Analysis

### Combined SDA/MOSI/AD0/MISO Pin

This is unusual. This suggests the board uses **one pin for multiple functions**:

**I2C Mode:**
- SDA: I2C Data (bidirectional)
- AD0: I2C Address select (when idle/pulled up or down)

**SPI Mode:**
- MOSI: SPI Master Out, Slave In (from ESP32 to sensor)
- MISO: SPI Master In, Slave Out (from sensor to ESP32)

**This is NOT standard.** It suggests:
1. The board auto-detects I2C vs SPI mode
2. OR you need to configure mode selection somehow
3. OR the board has internal switching circuitry

### Most Likely Scenario

Based on typical breakout board designs, I believe:

**For I2C mode (default):**
- This pin acts as **SDA** (I2C data)
- AD0 function determines I2C address (internal pull-up/down)

**For SPI mode:**
- You would need to short/cut a jumper to enable SPI
- Pin would switch to MOSI/MISO function

---

## BEFORE YOU CONNECT: Voltage Verification

### Required Tool
- Multimeter (voltage measurement mode)

### Procedure

**Step 1: Verify power regulator output**

1. Power the board:
   ```
   ICM42688P "V" pin  → ESP32 3.3V (Pin 1)
   ICM42688P "GND"    → ESP32 GND (Pin 14)
   ```

2. Measure voltage at ICM42688P chip VCC pin:
   - Set multimeter to DC voltage (20V range)
   - Black probe to GND
   - Red probe to small capacitor near chip or chip VCC pin
   - **Expected:** 3.2V - 3.4V
   - **If reading:** 5V or >3.5V → Board has NO regulator (DANGER!)

**Step 2: Verify signal pin voltage**

With board powered at 3.3V, measure signal pins:

1. Measure SCL/SCLK pin (should be pulled up):
   - Black probe to GND
   - Red probe to SCL/SCLK pin
   - **Expected:** ~3.3V (pull-up resistor)
   - **If reading:** >3.5V → Signal is 5V (DANGER for ESP32!)

2. Measure SDA pin (should be pulled up):
   - Black probe to GND
   - Red probe to SDA/MOSI/AD0/MISO pin
   - **Expected:** ~3.3V
   - **If reading:** >3.5V → Signal is 5V (DANGER!)

### ✓ SAFE TO CONNECT IF:
- Chip VCC = 3.2V - 3.4V
- Signal pins = 3.0V - 3.4V when idle

### ✗ DO NOT CONNECT IF:
- Chip VCC = 5V (no regulator, will damage sensor)
- Signal pins > 3.5V (will damage ESP32 GPIO)

If signals are 5V, you need **level shifters** between board and ESP32.

---

## I2C Connection (Recommended for First Test)

### Wiring for I2C Mode

```
ICM42688P Pin          ESP32-WROOM-32 Pin
─────────────          ──────────────────
V                 ───► 3.3V (Pin 1 or 22)
GND               ───► GND (Pin 14 or 20)
SDA/MOSI/AD0/MISO ───► IO21 (Pin 34) - I2C Data
SCL/SCLK          ───► IO22 (Pin 37) - I2C Clock
CS                ───► Not connected (leave floating for I2C mode)
INT1              ───► IO23 (Pin 38) - Optional
INT2              ───► (Leave disconnected for now)
```

### Visual Diagram

```
   ICM42688P Breakout              ESP32 DevKit
   (Side view)                     (40-pin)

   INT2  ○
   INT1  ○ ─────────────────────► Pin 38 (IO23) - Optional
   CS    ○ (floating)
   SCL   ○ ─────────────────────► Pin 37 (IO22)
   SDA   ○ ─────────────────────► Pin 34 (IO21)
   GND   ○ ─────────────────────► Pin 14 (GND)
   V     ○ ─────────────────────► Pin 1  (3.3V)
```

### I2C Address

With SDA/MOSI/AD0/MISO pin functioning as **SDA** in I2C mode, the **AD0** function sets address:

- **AD0 internally pulled HIGH**: Address = **0x69** (most common)
- **AD0 internally pulled LOW**: Address = **0x68**

Your board likely has 0x69 as default. Try both in code:

```python
# Try address 0x69 first
try:
    icm = adafruit_icm42688.ICM42688(i2c, address=0x69)
except:
    # If fails, try 0x68
    icm = adafruit_icm42688.ICM42688(i2c, address=0x68)
```

---

## Test Code for Your Board

```python
# test_icm42688p_5v_board.py
# For ICM42688P breakout board with 5V regulator

import board
import busio
import time

print("=" * 60)
print("ICM42688P Breakout Board Test (Model: 3031CM42681)")
print("=" * 60)

# Initialize I2C
print("\nInitializing I2C...")
i2c = busio.I2C(scl=board.IO22, sda=board.IO21, frequency=400000)
print("✓ I2C initialized")

# Scan I2C bus
print("\nScanning I2C bus...")
while not i2c.try_lock():
    pass
try:
    devices = i2c.scan()
    print(f"Found {len(devices)} device(s): {[hex(d) for d in devices]}")

    if 0x68 in devices:
        print("  → ICM42688P at 0x68 (AD0 = LOW)")
    if 0x69 in devices:
        print("  → ICM42688P at 0x69 (AD0 = HIGH)")

    if not devices:
        print("⚠ WARNING: No I2C devices found!")
        print("\nTroubleshooting:")
        print("  1. Check V pin connected to 3.3V")
        print("  2. Verify GND connection")
        print("  3. Check SDA on IO21, SCL on IO22")
        print("  4. Measure voltage at SDA/SCL (should be ~3.3V)")
        print("  5. Try swapping SDA and SCL")
finally:
    i2c.unlock()

# Test sensor
if devices:
    print("\nInitializing ICM42688P sensor...")
    import adafruit_icm42688
    from adafruit_icm42688 import registers as reg

    # Try both addresses
    icm = None
    for addr in [0x69, 0x68]:
        try:
            icm = adafruit_icm42688.ICM42688(i2c, address=addr)
            print(f"✓ ICM42688P initialized at address {hex(addr)}")
            break
        except Exception as e:
            print(f"  Address {hex(addr)}: {e}")

    if icm:
        # Read sensor data
        print("\n" + "=" * 60)
        print("SENSOR TEST")
        print("=" * 60)

        temp = icm.temperature
        accel = icm.acceleration
        gyro = icm.gyro

        print(f"\nTemperature:  {temp:.2f} °C")
        print(f"Acceleration: X={accel[0]:7.2f} Y={accel[1]:7.2f} Z={accel[2]:7.2f} m/s²")
        print(f"Gyroscope:    X={gyro[0]:7.4f} Y={gyro[1]:7.4f} Z={gyro[2]:7.4f} rad/s")

        if 8.0 < abs(accel[2]) < 11.0:
            print("\n✓ SUCCESS: Gravity detected! Sensor working correctly.")
        else:
            print(f"\n⚠ Z-axis = {accel[2]:.2f} m/s² (expected ~±9.8)")

        print("\n" + "=" * 60)
        print("✓ ALL TESTS PASSED")
        print("=" * 60)
    else:
        print("\n✗ FAILED: Could not initialize sensor")

print("\nTest complete.")
```

---

## SPI Mode (Advanced - Not Recommended Initially)

### Potential SPI Wiring

**⚠️ WARNING:** The combined SDA/MOSI/AD0/MISO pin makes SPI mode unclear.

**Possible configuration:**
```
ICM42688P Pin          ESP32 Pin
─────────────          ─────────
V                 ───► 3.3V
GND               ───► GND
SCL/SCLK          ───► IO18 (SCK)
SDA/MOSI/AD0/MISO ───► IO23 (MOSI) - Data to sensor
???               ───► IO19 (MISO) - Data from sensor (WHERE?)
CS                ───► IO5  (CS)
```

**Problem:** No separate MISO pin! This suggests:
1. Board uses a jumper or solder bridge to switch modes
2. MISO is shared with MOSI (half-duplex SPI - unusual)
3. Board documentation needed

**Recommendation:** Start with I2C mode. SPI requires board documentation.

---

## Troubleshooting

### Issue: No I2C devices found

**Checklist:**
- [ ] V pin connected to ESP32 3.3V
- [ ] GND connected
- [ ] SDA/MOSI/AD0/MISO pin on IO21
- [ ] SCL/SCLK pin on IO22
- [ ] CS pin left floating (not connected)
- [ ] Measured voltage: SDA and SCL show ~3.3V when idle
- [ ] Try swapping SDA and SCL connections
- [ ] Try both addresses (0x68 and 0x69)

### Issue: Sensor initializes but reads zeros

**Solutions:**
- Check WHO_AM_I register manually (should be 0x47)
- Verify chip is ICM42688P (not ICM42605 or different chip)
- Measure actual VCC at ICM chip on board (should be 3.3V)
- Try lower I2C frequency: `frequency=100000`

### Issue: Voltage measurements show 5V on signals

**You NEED level shifters!**

Add a **bidirectional I2C level shifter** between board and ESP32:
- Board side: Connect to ICM42688P signals
- ESP32 side: Connect to GPIO pins
- Example: SparkFun BOB-12009, Adafruit 757

---

## Summary for Your Board

**Board:** ICM42688P (3031CM42681)
**Power:** V pin accepts 3.3V - 5V (has onboard regulator)
**Recommended:** Power with ESP32's 3.3V for safety
**I2C Mode:** Use SDA/MOSI/AD0/MISO pin as SDA, SCL/SCLK as SCL
**Default Address:** Probably 0x69 (try 0x68 if not found)
**CS Pin:** Leave floating for I2C mode

**BEFORE CONNECTING:**
1. Measure voltage at signal pins with multimeter
2. Verify signals are 3.3V (NOT 5V)
3. If 5V, use level shifters

**SAFE CONNECTION:**
- V → ESP32 3.3V
- GND → ESP32 GND
- SDA/MOSI/AD0/MISO → IO21
- SCL/SCLK → IO22
- CS → Not connected

---

## Request for More Information

To provide better help, please provide:

1. **Photo of the board** (front and back)
2. **Measured voltages:**
   - SDA/MOSI/AD0/MISO pin voltage when powered
   - SCL/SCLK pin voltage when powered
3. **Board markings:** Any other text on PCB
4. **Purchase link:** Where did you get this board?

This will help me create exact wiring instructions for your specific board.

---

**Next Step:** Measure voltages with multimeter before connecting to ESP32!
