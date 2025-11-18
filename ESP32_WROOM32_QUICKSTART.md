# ESP32-WROOM-32 + ICM42688 Quick Start Guide

Get started testing the ICM42688 library with your ESP32-WROOM-32 40-pin board in 5 minutes.

## What You Need

### Hardware
- ESP32-WROOM-32 40-pin development board (ESP32-DevKitC, DOIT ESP32 DevKit v1, or similar)
- ICM42688 6-axis IMU breakout board
- USB cable (Micro-USB or USB-C depending on board)
- Breadboard and jumper wires (or soldering equipment)

### Software
- Computer with Chrome/Edge browser (for web installer)
- CircuitPython firmware for ESP32
- This ICM42688 library

---

## Step 1: Install CircuitPython on ESP32

### Option A: Web Installer (Easiest)

1. Go to CircuitPython downloads:
   - ESP32-DevKitC V4: https://circuitpython.org/board/espressif_esp32_devkitc_v4_wroom_32e/
   - DOIT ESP32 DevKit v1: https://circuitpython.org/board/doit_esp32_devkit_v1/

2. Click "OPEN INSTALLER" (requires Chrome browser)

3. Connect ESP32 via USB

4. Select port and click "Install"

5. Wait for installation (1-2 minutes)

### Option B: Command Line (Advanced)

```bash
# Install esptool
pip install esptool

# Download CircuitPython .bin file from link above

# Erase flash
esptool.py --chip esp32 --port /dev/ttyUSB0 erase_flash

# Flash CircuitPython
esptool.py --chip esp32 --port /dev/ttyUSB0 write_flash -z 0x1000 circuitpython-esp32.bin
```

Replace `/dev/ttyUSB0` with your port (Windows: `COM3`, macOS: `/dev/cu.usbserial-*`)

---

## Step 2: Connect to ESP32 Serial REPL

**IMPORTANT:** ESP32 has no USB drive! You must use serial connection.

### Linux/Mac:
```bash
screen /dev/ttyUSB0 115200
# Or use: picocom /dev/ttyUSB0 -b 115200
```

### Windows:
- Open PuTTY or Tera Term
- Port: COM3 (check Device Manager)
- Baud: 115200

You should see:
```
Auto-reload is on. Simply save files over USB to run them or enter REPL to disable.

Press any key to enter the REPL. Use CTRL-D to reload.
```

Press Enter to get Python REPL:
```python
>>>
```

---

## Step 3: Install Library Files

### Method 1: Via REPL (No CIRCUITPY drive)

Since ESP32 has no USB drive, upload files via REPL:

```python
# In REPL, press Ctrl+E to enter paste mode
# Paste these commands:

import os
os.mkdir('/lib')
os.mkdir('/lib/adafruit_icm42688')

# Exit paste mode with Ctrl+D
```

Then use a file transfer tool:
- **ampy**: `ampy --port /dev/ttyUSB0 put circuitpython/adafruit_icm42688 /lib/adafruit_icm42688`
- **rshell**: Connect with rshell and use `cp` commands
- **Thonny IDE**: Use built-in file manager

### Method 2: Via Web Workflow (CircuitPython 8+)

1. Enable web workflow in REPL:
```python
import wifi
import socketpool
wifi.radio.connect("YourWiFi", "password")
print(wifi.radio.ipv4_address)
```

2. Open browser: `http://<ESP32_IP>`
3. Upload files via web interface

### Required Files

Copy these to ESP32's `/lib/` folder:
```
/lib/
├── adafruit_icm42688/
│   ├── __init__.py
│   └── registers.py
└── adafruit_bus_device/
    ├── __init__.py
    ├── i2c_device.py
    └── spi_device.py
```

Get `adafruit_bus_device` from:
https://github.com/adafruit/Adafruit_CircuitPython_Bundle/releases

---

## Step 4: Wire ICM42688 to ESP32

### I2C Wiring (Recommended)

```
ICM42688          ESP32-WROOM-32
────────          ──────────────
VCC      ────────  3.3V  (Pin 1)
GND      ────────  GND   (Pin 14 or 20)
SDA      ────────  IO21  (Pin 34)
SCL      ────────  IO22  (Pin 37)
```

**CRITICAL:** Use 3.3V only! Never 5V!

### Visual Reference

```
ESP32 (Top View)              ICM42688
  ┌─────────┐
  │  Pin 1  │ 3.3V ────────── VCC
  │  Pin 14 │ GND  ────────── GND
  │  Pin 34 │ IO21 ────────── SDA
  │  Pin 37 │ IO22 ────────── SCL
  └─────────┘
```

See `circuitpython/examples/ESP32_WROOM32_WIRING_GUIDE.md` for detailed diagrams.

---

## Step 5: Run First Test

### Upload Test Script

Transfer `circuitpython/examples/esp32_wroom32_i2c_test.py` to ESP32 as `code.py`:

```bash
# Using ampy:
ampy --port /dev/ttyUSB0 put circuitpython/examples/esp32_wroom32_i2c_test.py /code.py

# Using rshell:
rshell --port /dev/ttyUSB0
> cp circuitpython/examples/esp32_wroom32_i2c_test.py /pyboard/code.py
```

### Run Test

Press Ctrl+D in REPL to reboot and run code.py:

```
======================================================================
ICM42688 I2C Test - ESP32-WROOM-32 (40-pin DevKit)
======================================================================

Initializing I2C bus...
  SDA: GPIO21 (Pin 34)
  SCL: GPIO22 (Pin 37)
  Frequency: 400kHz
✓ I2C bus initialized

Scanning I2C bus...
Found 1 device(s): ['0x69']

Initializing ICM42688 sensor...
✓ ICM42688 found at address 0x69 (SDO/AD0 = HIGH)

Configuring sensor...
  Accelerometer: ±16g, 1kHz, Low-Noise mode
  Gyroscope: ±2000 dps, 1kHz, Low-Noise mode
  Temperature: Enabled
✓ Configuration complete

======================================================================
TEST 1: Basic Sensor Reading
======================================================================

Temperature:  25.34 °C
Acceleration: X=  -0.23 Y=   0.45 Z=   9.78 m/s²
Gyroscope:    X= 0.0012 Y=-0.0034 Z= 0.0001 rad/s

✓ Gravity detected on Z-axis - sensor working correctly!
```

**Success!** If you see gravity (~9.8 m/s²) on Z-axis, everything is working.

---

## Step 6: Test Your Application

### Basic Reading Example

```python
import board
import busio
import adafruit_icm42688

# Initialize I2C
i2c = busio.I2C(scl=board.IO22, sda=board.IO21)

# Create sensor instance
icm = adafruit_icm42688.ICM42688(i2c, address=0x69)

# Read sensors
temp = icm.temperature
accel = icm.acceleration
gyro = icm.gyro

print(f"Temperature: {temp:.1f}°C")
print(f"Acceleration: X={accel[0]:.2f} Y={accel[1]:.2f} Z={accel[2]:.2f} m/s²")
print(f"Gyroscope: X={gyro[0]:.3f} Y={gyro[1]:.3f} Z={gyro[2]:.3f} rad/s")
```

### SPI Example

```python
import board
import busio
import digitalio
import adafruit_icm42688

# Initialize SPI
spi = busio.SPI(clock=board.IO18, MOSI=board.IO23, MISO=board.IO19)
cs = digitalio.DigitalInOut(board.IO5)

# Create sensor instance
icm = adafruit_icm42688.ICM42688(spi, cs=cs, baudrate=10000000)

# Read sensors (same API as I2C)
temp = icm.temperature
accel = icm.acceleration
```

---

## Troubleshooting

### Problem: No I2C devices found

**Check:**
1. Wiring: VCC=3.3V, GND=GND, SDA=IO21, SCL=IO22
2. Power LED on ICM42688 (if available)
3. Try different I2C address: `ICM42688(i2c, address=0x68)`
4. Lower I2C speed: `I2C(scl=board.IO22, sda=board.IO21, frequency=100000)`

### Problem: Import error - no module named 'adafruit_icm42688'

**Solution:**
Library not installed. Copy `adafruit_icm42688/` folder to `/lib/` on ESP32.

### Problem: ESP32 not appearing as USB device

**This is normal!** ESP32 (original) has no USB drive. Use serial connection.

For ESP32-S2/S3, you WOULD see a drive, but ESP32-WROOM-32 requires serial only.

### Problem: Permission denied on /dev/ttyUSB0 (Linux)

```bash
sudo usermod -a -G dialout $USER
# Log out and back in
```

### Problem: Sensor reads all zeros

**Check:**
1. ICM42688 powered (measure VCC with multimeter)
2. I2C address correct (try 0x68 and 0x69)
3. SDA/SCL not swapped
4. No loose connections

---

## Next Steps

1. ✓ Basic test working
2. Run full test suite: `esp32_wroom32_i2c_test.py`
3. Test SPI interface: `esp32_wroom32_spi_test.py`
4. Test FIFO for high-speed data logging
5. Test interrupts for motion detection
6. Integrate into your application

---

## Reference Documents

- **Detailed Wiring:** `circuitpython/examples/ESP32_WROOM32_WIRING_GUIDE.md`
- **I2C Test Script:** `circuitpython/examples/esp32_wroom32_i2c_test.py`
- **SPI Test Script:** `circuitpython/examples/esp32_wroom32_spi_test.py`
- **Library Examples:** `circuitpython/examples/`

---

## Support

**Issues with library?**
Report at: https://github.com/yassinebkr/DFRobot_ICM42688/issues

**CircuitPython questions?**
Forum: https://forums.adafruit.com/viewforum.php?f=60

**ESP32 CircuitPython guide:**
https://learn.adafruit.com/circuitpython-with-esp32-quick-start

---

**Happy Making! 🚀**
