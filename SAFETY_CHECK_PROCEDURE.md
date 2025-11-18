# SAFETY CHECK PROCEDURE - ICM42688P 5V Breakout Board

**CRITICAL:** Follow this procedure BEFORE connecting your ICM42688P board to ESP32 to prevent damage.

---

## Required Equipment

- ESP32-WROOM-32 board (powered via USB)
- ICM42688P breakout board (Model: 3031CM42681)
- Digital multimeter (any cheap one works)
- Jumper wires
- Breadboard (optional)

---

## Step 1: Visual Inspection

### Check Your Board

Look at your ICM42688P board and verify:

```
Expected pin labels (top to bottom):
┌─────────────────┐
│  INT2           │
│  INT1           │
│  CS             │
│  SCL/SCLK       │ ← Check this label
│  SDA/MOSI/AD0/MISO │ ← Check this label
│  GND            │
│  V              │ ← Should say "5V" or just "V"
└─────────────────┘
```

**Questions to answer:**
- [ ] Does your board have 7 pins in this order?
- [ ] Is there a small IC near the power pin (voltage regulator)?
- [ ] Are there any jumpers or solder bridges on the back?
- [ ] Can you see the ICM42688P chip itself?

Take a photo if unsure!

---

## Step 2: Power Board Safely

### Initial Power-Up Test

**Setup:**
1. Do NOT connect any signal pins yet
2. ONLY connect power and ground

```
ICM42688P Board     ESP32-WROOM-32 (Your Board)
───────────────     ───────────────────────────
V (5V pin)     ──── 3V3 (RIGHT Pin 40)    ← Power input
GND            ──── GND (LEFT Pin 1)      ← Ground
(All other pins disconnected)
```

**⚠️ USE 3.3V, NOT 5V!** This is safer for first test.

### Visual Check

Power the ESP32 via USB. Board should:
- Get warm (not hot)
- No smoke (obviously!)
- No burning smell
- LED may illuminate (if board has one)

**If anything smells, gets hot, or smokes → DISCONNECT IMMEDIATELY!**

---

## Step 3: Voltage Measurements

### Required Multimeter Settings

```
Multimeter Setup:
┌────────────────┐
│  Function: VDC │ ← DC Voltage
│  Range: 20V    │ ← Or auto-range
│  COM: Black    │ ← Common/negative probe
│  V: Red        │ ← Voltage probe
└────────────────┘
```

### Measurement 1: Power Input Verification

**What:** Verify board receives 3.3V power

**How:**
1. Keep board powered (V pin to ESP32 3.3V, GND to ESP32 GND)
2. Set multimeter to DC Volts (20V range)
3. Touch black probe to ICM42688P GND pin
4. Touch red probe to ICM42688P V pin
5. Read voltage

**Expected:** 3.25V - 3.35V
**If reading:**
- 0V → Bad connection, check wiring
- 5V → Wrong pin! Connected to 5V instead of 3.3V
- <3.0V → Weak power supply

```
Measurement point:
   ICM42688P Breakout
   ┌─────────────┐
   │ INT2      ○ │
   │ INT1      ○ │
   │ CS        ○ │
   │ SCL       ○ │
   │ SDA       ○ │
   │ GND       ○ │ ← Black probe HERE
   │ V         ○ │ ← Red probe HERE
   └─────────────┘

Expected reading: ~3.3V
```

### Measurement 2: SCL Signal Voltage (CRITICAL!)

**What:** Check if SCL signal is 3.3V or 5V

**Why:** ESP32 GPIO pins are damaged by 5V!

**How:**
1. Keep board powered
2. Set multimeter to DC Volts
3. Black probe to ICM42688P GND pin
4. Red probe to ICM42688P **SCL/SCLK** pin
5. Read voltage

**Expected:** 3.0V - 3.4V (pulled up by resistor)
**If reading:**
- 3.0-3.4V → ✓ SAFE for ESP32
- 4.5-5.5V → ✗ DANGER! Need level shifter
- 0V - 0.5V → May be OK (check SDA too)

```
Measurement point:
   ICM42688P Breakout
   ┌─────────────┐
   │ INT2      ○ │
   │ INT1      ○ │
   │ CS        ○ │
   │ SCL       ○ │ ← Red probe HERE
   │ SDA       ○ │
   │ GND       ○ │ ← Black probe HERE
   │ V         ○ │
   └─────────────┘

Expected reading: ~3.3V
If >3.5V → STOP! Need level shifter!
```

### Measurement 3: SDA Signal Voltage (CRITICAL!)

**What:** Check if SDA signal is 3.3V or 5V

**How:**
1. Keep board powered
2. Black probe to GND
3. Red probe to **SDA/MOSI/AD0/MISO** pin
4. Read voltage

**Expected:** 3.0V - 3.4V
**If reading:**
- 3.0-3.4V → ✓ SAFE for ESP32
- 4.5-5.5V → ✗ DANGER! Need level shifter
- 0V - 0.5V → May be OK, pull-up may be weak

```
Measurement point:
   ICM42688P Breakout
   ┌─────────────┐
   │ INT2      ○ │
   │ INT1      ○ │
   │ CS        ○ │
   │ SCL       ○ │
   │ SDA       ○ │ ← Red probe HERE
   │ GND       ○ │ ← Black probe HERE
   │ V         ○ │
   └─────────────┘

Expected reading: ~3.3V
If >3.5V → STOP! Need level shifter!
```

### Measurement 4: Chip VCC (Advanced)

**What:** Measure actual voltage at ICM42688P chip

**Why:** Verify onboard regulator is working

**How:**
1. Locate tiny chip on board (5mm x 5mm square)
2. Find small capacitor next to chip
3. Measure voltage across capacitor:
   - Black probe to GND side
   - Red probe to VCC side
4. Or touch red probe directly to chip VCC pin (very small!)

**Expected:** 3.2V - 3.4V (regulated down from input)
**If reading:**
- 3.2-3.4V → ✓ Regulator working perfectly
- 5.0V → ✗ No regulator! Board is 5V-only!
- <3.0V → Regulator failing

```
   Top view of board:
   ┌──────────────────┐
   │  ┌──┐  ┌──┐     │
   │  │░░│  │▓▓│ ← Capacitor
   │  │░░│  └┬─┘     │
   │  └──┘   │       │
   │  ICM    └─ VCC (measure here)
   │ 42688P          │
   └──────────────────┘

Measure at capacitor:
Black probe → Capacitor negative (usually has "-" marking)
Red probe → Capacitor positive
Expected: 3.2-3.4V
```

---

## Step 4: Decision Matrix

Based on your measurements:

### ✓ SAFE TO CONNECT - All conditions met:

- [ ] V pin voltage = 3.25V - 3.35V
- [ ] SCL voltage = 3.0V - 3.4V
- [ ] SDA voltage = 3.0V - 3.4V
- [ ] Chip VCC = 3.2V - 3.4V (if measured)

**Action:** Proceed to connect signal pins

```
Full I2C connection:
ICM42688P          ESP32
─────────          ─────
V            ────► 3.3V (Pin 1)
GND          ────► GND (Pin 14)
SDA          ────► IO21 (Pin 34)  ← NOW SAFE
SCL          ────► IO22 (Pin 37)  ← NOW SAFE
CS           ────► (not connected)
```

### ⚠️ NEEDS LEVEL SHIFTER - Any condition:

- [ ] SCL voltage > 3.5V
- [ ] SDA voltage > 3.5V
- [ ] Chip VCC = 5.0V

**Action:** Use bidirectional level shifter

```
Required: I2C Level Shifter (e.g., SparkFun BOB-12009)

Connection with level shifter:
ICM42688P → [Level Shifter HV side] → [Level Shifter LV side] → ESP32
5V signals     (5V reference)            (3.3V reference)      3.3V GPIO
```

### ✗ PROBLEM DETECTED - Any condition:

- [ ] V pin = 0V (no power)
- [ ] Any voltage > 5.5V
- [ ] Board gets hot (>50°C)
- [ ] Voltage readings are erratic

**Action:** STOP! Troubleshoot wiring or board may be damaged

---

## Step 5: First I2C Test

### After Confirming Safe Voltages

**Full wiring:**
```
ICM42688P Pin          ESP32 Pin (Your Board)
─────────────          ──────────────────────
V                 ──── 3V3 (RIGHT Pin 40)
GND               ──── GND (LEFT Pin 1, 7, or 20)
SDA/MOSI/AD0/MISO ──── P21 (LEFT Pin 6)
SCL/SCLK          ──── P22 (LEFT Pin 3)
CS                ──── (floating - not connected)
INT1              ──── (optional - leave for now)
INT2              ──── (optional - leave for now)
```

**Test script:**
```python
import board
import busio

# Initialize I2C
i2c = busio.I2C(scl=board.IO22, sda=board.IO21, frequency=400000)

# Scan bus
while not i2c.try_lock():
    pass
try:
    devices = i2c.scan()
    print(f"I2C devices found: {[hex(d) for d in devices]}")
finally:
    i2c.unlock()

# Expected output:
# I2C devices found: ['0x69']  or  ['0x68']
```

**If scan finds 0x68 or 0x69 → SUCCESS!**

---

## Troubleshooting Measurement Issues

### Multimeter shows "OL" or "1"

**Meaning:** Over-limit, voltage out of range
**Solution:** Change to higher voltage range (200V)

### Multimeter shows negative voltage

**Meaning:** Probes are reversed
**Solution:** Swap red and black probes

### Voltage fluctuates rapidly

**Possible causes:**
1. Poor connection → Check probe contact
2. Noisy power supply → Add 10µF capacitor across V and GND
3. Bad ground → Ensure ESP32 GND is solid

### SDA/SCL voltage is 0V

**Possible causes:**
1. No pull-up resistors on board → Board may be faulty
2. Pins configured as outputs → Normal before I2C starts
3. Board not powered → Check V pin has voltage

**Test:** Try measuring resistance from SDA to V pin:
- Should be 2kΩ - 10kΩ (pull-up resistor)
- If >100kΩ → No pull-up, board may need external resistors

---

## Summary Checklist

Before connecting signal pins to ESP32:

- [ ] Step 1: Visual inspection complete
- [ ] Step 2: Board powers up without issues
- [ ] Step 3.1: V pin measures 3.25-3.35V
- [ ] Step 3.2: SCL measures 3.0-3.4V (NOT >3.5V!)
- [ ] Step 3.3: SDA measures 3.0-3.4V (NOT >3.5V!)
- [ ] Step 3.4: Chip VCC measures 3.2-3.4V (optional)
- [ ] Step 4: Decision made (safe / needs level shifter / problem)
- [ ] Step 5: Ready for I2C test

**If all checked → Proceed with full wiring!**

---

## What If I Don't Have a Multimeter?

### Budget Option (~$10-20)

Buy a basic digital multimeter from:
- Amazon: Search "digital multimeter"
- Harbor Freight: Free with coupon
- Hardware store: Any $10-20 model works

**ANY multimeter can measure DC voltage!**

### No Multimeter Available

**Risky but possible:**
1. Connect ONLY power pins (V and GND) using 3.3V
2. Connect SDA and SCL
3. Run I2C scan immediately
4. If ESP32 still works after 5 minutes → probably safe
5. If ESP32 crashes or gets hot → DISCONNECT! 5V signals detected

**⚠️ This risks damaging your ESP32! Multimeter is $15, ESP32 is $7. Worth it.**

---

## Emergency: I Already Connected Without Checking!

### If ESP32 Still Works

1. DISCONNECT immediately
2. Check ESP32 for heat (feel the main chip)
3. Follow measurement procedure above
4. If voltages are safe (3.3V), you got lucky
5. If voltages are 5V, your ESP32 may be damaged

### If ESP32 Stopped Working

Possible damage:
- GPIO pins burned out (may still boot, but I2C won't work)
- Voltage regulator damaged (won't power up)
- Main chip damaged (dead board)

**Test:**
1. Disconnect everything
2. Try uploading simple blink sketch
3. If upload works → GPIO may be damaged, chip OK
4. If upload fails → Board likely dead

**Prevention:** Always measure voltages first!

---

**Next:** After confirming safe voltages, proceed to `ESP32_WROOM32_QUICKSTART.md` for I2C testing.
