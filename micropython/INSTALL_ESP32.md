# MicroPython Installation Guide for ESP32

Complete guide to installing MicroPython on ESP32 boards and setting up the ICM42688 driver.

## Table of Contents

1. [Prerequisites](#prerequisites)
2. [Install MicroPython Tools](#install-micropython-tools)
3. [Download MicroPython Firmware](#download-micropython-firmware)
4. [Flash MicroPython to ESP32](#flash-micropython-to-esp32)
5. [Connect to ESP32](#connect-to-esp32)
6. [Install ICM42688 Library](#install-icm42688-library)
7. [Verify Installation](#verify-installation)
8. [Troubleshooting](#troubleshooting)

---

## Prerequisites

### Hardware Required
- ESP32 development board (ESP32 WROOM, ESP32-C3, Lolin D32, etc.)
- USB cable (typically USB-A to micro-USB or USB-C)
- ICM42688 breakout board (optional, for testing)

### Software Required
- Python 3.7 or later
- USB-to-serial drivers (usually automatic on modern OS)
- Terminal/command prompt

### Supported Operating Systems
- Windows 10/11
- macOS 10.14+
- Linux (Ubuntu, Debian, Fedora, etc.)

---

## Install MicroPython Tools

### Option 1: Using pip (Recommended)

Install the required tools globally:

```bash
# Install esptool (for flashing firmware)
pip install esptool

# Install mpremote (for file transfer and REPL)
pip install mpremote

# Optional: Install ampy (alternative file transfer tool)
pip install adafruit-ampy
```

### Option 2: Using Python virtual environment

```bash
# Create virtual environment
python3 -m venv micropython-env

# Activate virtual environment
# On Windows:
micropython-env\Scripts\activate
# On macOS/Linux:
source micropython-env/bin/activate

# Install tools
pip install esptool mpremote adafruit-ampy
```

### Verify Installation

```bash
esptool version
mpremote version
```

---

## Download MicroPython Firmware

### ESP32 (Generic, WROOM, Lolin D32)

1. Visit the MicroPython downloads page:
   https://micropython.org/download/ESP32_GENERIC/

2. Download the latest stable firmware (e.g., `ESP32_GENERIC-20240105-v1.22.1.bin`)

3. Or use wget/curl:
   ```bash
   wget https://micropython.org/resources/firmware/ESP32_GENERIC-20240105-v1.22.1.bin
   ```

### ESP32-C3

1. Visit: https://micropython.org/download/ESP32_GENERIC_C3/

2. Download the latest firmware (e.g., `ESP32_GENERIC_C3-20240105-v1.22.1.bin`)

### ESP32-S3

1. Visit: https://micropython.org/download/ESP32_GENERIC_S3/

2. Download the latest firmware

**Note:** Always use the latest stable version (v1.19 or later required for ICM42688 driver)

---

## Flash MicroPython to ESP32

### Step 1: Connect ESP32

1. Connect your ESP32 to your computer via USB
2. Note the serial port:
   - **Windows:** `COM3`, `COM4`, etc. (check Device Manager)
   - **macOS:** `/dev/cu.usbserial-*` or `/dev/cu.SLAB_USBtoUART`
   - **Linux:** `/dev/ttyUSB0` or `/dev/ttyACM0`

### Step 2: Erase Flash (Recommended)

Before flashing, erase the existing flash:

```bash
# Replace /dev/ttyUSB0 with your port
esptool.py --chip esp32 --port /dev/ttyUSB0 erase_flash
```

**Windows example:**
```bash
esptool.py --chip esp32 --port COM3 erase_flash
```

### Step 3: Flash MicroPython Firmware

```bash
# ESP32 Generic/WROOM/Lolin D32
esptool.py --chip esp32 --port /dev/ttyUSB0 --baud 460800 write_flash -z 0x1000 ESP32_GENERIC-20240105-v1.22.1.bin

# ESP32-C3
esptool.py --chip esp32c3 --port /dev/ttyUSB0 --baud 460800 write_flash -z 0x0 ESP32_GENERIC_C3-20240105-v1.22.1.bin
```

**Parameters explained:**
- `--chip esp32`: Specify chip type (esp32, esp32c3, esp32s3)
- `--port /dev/ttyUSB0`: Serial port
- `--baud 460800`: Baud rate (460800 is fastest, use 115200 if issues)
- `write_flash`: Write firmware
- `-z`: Compress data
- `0x1000`: Flash offset (0x1000 for ESP32, 0x0 for ESP32-C3)

### Step 4: Verify Flash

After flashing completes, press the RESET button on your ESP32 or reconnect USB.

The output should show:
```
Hash of data verified.

Leaving...
Hard resetting via RTS pin...
```

---

## Connect to ESP32

### Option 1: Using mpremote (Recommended)

```bash
# Connect to REPL
mpremote

# You should see:
# MicroPython v1.22.1 on 2024-01-05; ESP32 module with ESP32
# Type "help()" for more information.
# >>>
```

Test Python:
```python
>>> print("Hello from MicroPython!")
Hello from MicroPython!
>>> import sys
>>> print(sys.platform)
esp32
```

Exit REPL: Press `Ctrl+]`

### Option 2: Using screen (macOS/Linux)

```bash
screen /dev/ttyUSB0 115200
```

Exit: Press `Ctrl+A` then `K`, then `Y`

### Option 3: Using PuTTY (Windows)

1. Download PuTTY: https://www.putty.org/
2. Set connection type: Serial
3. Serial line: COM3 (your port)
4. Speed: 115200
5. Click "Open"

### Option 4: Using Thonny IDE (Beginner-Friendly)

1. Download Thonny: https://thonny.org/
2. Open Thonny
3. Go to: Run → Select Interpreter
4. Choose "MicroPython (ESP32)"
5. Select your port
6. Click "OK"

---

## Install ICM42688 Library

### Method 1: Using mpremote (Recommended)

```bash
# Navigate to the repository root
cd /path/to/DFRobot_ICM42688

# Copy the entire icm42688 module to ESP32
mpremote cp -r micropython/icm42688 :

# Verify installation
mpremote ls
# Should show: icm42688/

# Check module contents
mpremote ls icm42688
# Should show: __init__.py, registers.py
```

### Method 2: Using ampy

```bash
# Set port (adjust for your system)
export AMPY_PORT=/dev/ttyUSB0  # Linux/macOS
# Or Windows: set AMPY_PORT=COM3

# Copy library
ampy put micropython/icm42688 icm42688

# Verify
ampy ls
```

### Method 3: Manual File Copy (Thonny)

1. Open Thonny
2. Connect to ESP32 (View → Files)
3. Navigate to `micropython/icm42688/` on your computer
4. Right-click the `icm42688` folder
5. Select "Upload to /"
6. Wait for upload to complete

### Method 4: WebREPL (WiFi Transfer)

1. Enable WebREPL on ESP32:
   ```python
   import webrepl_setup
   # Follow prompts to enable and set password
   ```

2. Access WebREPL: http://micropython.org/webrepl/

3. Upload files via web interface

---

## Verify Installation

### Test 1: Import Library

```bash
mpremote
```

In the REPL:
```python
>>> import icm42688
>>> print(icm42688.__version__)
1.0.0
>>> from icm42688 import registers as reg
>>> print(reg.ICM42688_CHIP_ID)
71
```

If no errors, the library is installed correctly!

### Test 2: Connect ICM42688 Hardware

**Wiring (ESP32 WROOM I2C):**
```
ICM42688    ESP32
--------    -----
VCC    →    3.3V
GND    →    GND
SDA    →    GPIO21
SCL    →    GPIO22
```

**Test code:**
```python
from machine import I2C, Pin
import icm42688

# Initialize I2C
i2c = I2C(0, scl=Pin(22), sda=Pin(21), freq=400000)

# Scan for devices
devices = i2c.scan()
print("I2C devices:", [hex(d) for d in devices])
# Should show: I2C devices: ['0x69'] or ['0x68']

# Initialize sensor
icm = icm42688.ICM42688(i2c, address=0x69)

# Read data
print("Temperature:", icm.temperature, "°C")
print("Acceleration:", icm.acceleration)
print("Gyroscope:", icm.gyro)
```

### Test 3: Run Test Scripts

```bash
# Copy and run basic test
mpremote run micropython/tests/test_basic_functionality.py

# Run example
mpremote run micropython/examples/basic_i2c_esp32.py
```

If all tests pass, installation is complete! 🎉

---

## Troubleshooting

### Issue: "Could not open port"

**Cause:** Port already in use or permission denied

**Solutions:**
- **Linux:** Add user to dialout group:
  ```bash
  sudo usermod -a -G dialout $USER
  # Log out and back in
  ```
- **macOS:** Check System Preferences → Security & Privacy
- **Windows:** Close Arduino IDE, PuTTY, or other serial programs

### Issue: "Failed to connect to ESP32"

**Solutions:**
1. Hold BOOT button while connecting USB
2. Try lower baud rate: `--baud 115200`
3. Check USB cable (some are power-only)
4. Install CH340/CP2102 drivers if needed

### Issue: "Timed out waiting for packet header"

**Solutions:**
1. Press and hold BOOT button during flash
2. Try: `esptool.py --chip esp32 --port /dev/ttyUSB0 --before default_reset --after hard_reset write_flash ...`
3. Check power supply (use powered USB hub if needed)

### Issue: "OSError: [Errno 19] ENODEV" when using I2C

**Solutions:**
1. Check wiring (SDA to GPIO21, SCL to GPIO22)
2. Add external pull-up resistors (4.7kΩ on SDA and SCL)
3. Try different I2C frequency: `freq=100000`
4. Verify 3.3V power supply
5. Check sensor with I2C scan:
   ```python
   from machine import I2C, Pin
   i2c = I2C(0, scl=Pin(22), sda=Pin(21), freq=400000)
   print(i2c.scan())  # Should show [104] or [105]
   ```

### Issue: "MemoryError" or "RuntimeError: Failed to find ICM42688"

**Solutions:**
1. Reset ESP32: Press RESET button
2. Check I2C address (0x69 if SDO high, 0x68 if SDO low)
3. Verify ICM42688 power (3.3V, NOT 5V)
4. Check for short circuits in wiring

### Issue: Module not found after upload

**Solutions:**
1. Verify upload: `mpremote ls`
2. Reset ESP32 after upload
3. Check file paths (should be `icm42688/__init__.py`)
4. Re-upload with `mpremote cp -r`

### Issue: Slow upload speed

**Solutions:**
1. Use mpremote instead of ampy (faster)
2. Close other serial connections
3. Reduce file size (already optimized in this library)

---

## Quick Reference

### Common Commands

```bash
# Flash firmware
esptool.py --chip esp32 --port /dev/ttyUSB0 erase_flash
esptool.py --chip esp32 --port /dev/ttyUSB0 write_flash -z 0x1000 firmware.bin

# Connect to REPL
mpremote

# List files on ESP32
mpremote ls

# Copy files to ESP32
mpremote cp file.py :
mpremote cp -r folder/ :

# Run Python script
mpremote run script.py

# Get file from ESP32
mpremote cp :file.py .

# Remove file from ESP32
mpremote rm file.py

# Reset ESP32
mpremote reset
```

### Pin Reference (ESP32 WROOM / Lolin D32)

| Function | GPIO | Alt Function |
|----------|------|--------------|
| I2C SDA  | 21   | Default      |
| I2C SCL  | 22   | Default      |
| SPI MOSI | 23   | VSPI         |
| SPI MISO | 19   | VSPI         |
| SPI SCK  | 18   | VSPI         |
| SPI CS   | 5    | Any GPIO     |

---

## Additional Resources

### Official Documentation
- MicroPython: https://docs.micropython.org/
- ESP32 Quick Reference: https://docs.micropython.org/en/latest/esp32/quickref.html
- ICM42688 Datasheet: https://invensense.tdk.com/products/motion-tracking/6-axis/icm-42688-p/

### Tools
- esptool: https://github.com/espressif/esptool
- mpremote: https://docs.micropython.org/en/latest/reference/mpremote.html
- Thonny IDE: https://thonny.org/

### Community
- MicroPython Forum: https://forum.micropython.org/
- ESP32 Forum: https://www.esp32.com/

---

## Next Steps

1. ✓ MicroPython installed and verified
2. ✓ ICM42688 library installed
3. ✓ Hardware connected and tested

**Now you can:**
- Run the examples in `micropython/examples/`
- Integrate into your own projects
- Read the API documentation in `micropython/README.md`
- Explore advanced features (FIFO, motion detection, interrupts)

Happy coding! 🚀
