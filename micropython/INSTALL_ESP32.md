# MicroPython Installation Guide for ESP32

Complete guide to installing MicroPython on ESP32 boards and setting up the ICM42688 driver.

## ⚠️ Experiencing "invalid header" Boot Errors?

If you're seeing this error after flashing:
```
rst:0x10 (RTCWDT_RTC_RESET),boot:0x13 (SPI_FAST_FLASH_BOOT)
invalid header: 0x5b1b6769
invalid header: 0x5b1b6769
```

**Jump directly to:** [Troubleshooting: "invalid header" Boot Errors](#issue-invalid-header-boot-errors-critical)

**Quick fix:** You likely need to:
1. Use **115200 baud** instead of 460800
2. **Physically disconnect/reconnect USB** after flashing
3. **Verify boot** with serial monitor before using `mpremote`

---

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


## Flash MicroPython to ESP32

### ⚠️ CRITICAL: Read This Before Flashing

**Common mistakes that cause boot failures:**
1. Not waiting for flash to complete before disconnecting
2. Not physically resetting ESP32 after flashing
3. Trying to connect with `mpremote` immediately without verifying boot
4. Using baud rate too high (460800 can fail on some systems)
5. Not checking for error messages during flashing

**If you see `invalid header` errors after flashing, you MUST reflash - see troubleshooting section below.**

---

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
esptool --chip esp32 --port /dev/ttyUSB0 erase_flash
```

**Windows example:**
```bash
esptool --chip esp32 --port COM3 erase_flash
```

**Expected output:**
```
esptool.py v4.x
Serial port /dev/ttyUSB0
Connecting....
Chip is ESP32-D0WDQ6 (revision X)
...
Erasing flash (this may take a while)...
Chip erase completed successfully in X.Xs
Hard resetting via RTS pin...
```

✅ **Success indicator:** Look for "Chip erase completed successfully"

### Step 3: Flash MicroPython Firmware

**IMPORTANT: Start with 115200 baud for reliability. Only use 460800 if 115200 is too slow and you've had success with lower speed first.**

#### Recommended (Reliable) Method:

```bash
# ESP32 Generic/WROOM/Lolin D32 - Use 115200 baud for reliability
esptool --chip esp32 --port /dev/ttyUSB0 --baud 115200 write_flash -z 0x1000 ESP32_GENERIC-20240105-v1.22.1.bin

# ESP32-C3 - Use 115200 baud for reliability
esptool --chip esp32c3 --port /dev/ttyUSB0 --baud 115200 write_flash -z 0x0 ESP32_GENERIC_C3-20240105-v1.22.1.bin
```

#### Fast Method (Use only if 115200 works reliably):

```bash
# ESP32 Generic/WROOM/Lolin D32 - Faster but can fail on some systems
esptool --chip esp32 --port /dev/ttyUSB0 --baud 460800 write_flash -z 0x1000 ESP32_GENERIC-20240105-v1.22.1.bin

# ESP32-C3 - Faster but can fail on some systems
esptool --chip esp32c3 --port /dev/ttyUSB0 --baud 460800 write_flash -z 0x0 ESP32_GENERIC_C3-20240105-v1.22.1.bin
```

**Parameters explained:**
- `--chip esp32`: Specify chip type (esp32, esp32c3, esp32s3)
- `--port /dev/ttyUSB0`: Serial port
- `--baud 115200`: Baud rate (115200 is reliable, 460800 is faster but riskier)
- `write_flash`: Write firmware
- `-z`: Compress data during transfer
- `0x1000`: Flash offset (0x1000 for ESP32, 0x0 for ESP32-C3)

**Expected output during flashing:**
```
esptool.py v4.x
Serial port /dev/ttyUSB0
Connecting....
Chip is ESP32-D0WDQ6 (revision X)
...
Configuring flash size...
Compressed XXXXX bytes to XXXXX...
Writing at 0x00001000... (X %)
Writing at 0x00002000... (X %)
...
Wrote XXXXX bytes (XXXXX compressed) at 0x00001000 in X.X seconds (effective XXX.X kbit/s)...
Hash of data verified.

Leaving...
Hard resetting via RTS pin...
```

✅ **Success indicators to look for:**
- "Connecting...." followed by chip detection
- Progress percentages (0% to 100%)
- "Hash of data verified." ← **MOST IMPORTANT**
- "Hard resetting via RTS pin..."

❌ **Failure indicators:**
- "Failed to connect"
- "Timed out waiting for packet header"
- No "Hash of data verified" message
- Error messages during writing

### Step 4: CRITICAL - Reset ESP32 Properly

**IMMEDIATELY after flashing completes, you MUST reset the ESP32:**

**Option 1: Physical reset button (Recommended)**
1. Press the **RESET** or **EN** button on your ESP32 board
2. Release after 1 second
3. Wait 3-5 seconds for boot

**Option 2: Reconnect USB**
1. Disconnect USB cable from ESP32
2. Wait 2 seconds
3. Reconnect USB cable
4. Wait 3-5 seconds for boot

**Option 3: Power cycle**
1. Unplug ESP32 from power
2. Wait 2 seconds
3. Reconnect power
4. Wait 3-5 seconds for boot

### Step 5: Verify Successful Boot

**Before trying to connect with mpremote, verify the ESP32 boots correctly:**

#### Method 1: Serial Monitor (Recommended)

```bash
# Linux/macOS - use screen
screen /dev/ttyUSB0 115200

# Windows - use PuTTY or:
# Open COM port at 115200 baud
```

**What you should see after pressing RESET:**
```
rst:0x1 (POWERON_RESET),boot:0x13 (SPI_FAST_FLASH_BOOT)
configsip: 0, SPIWP:0xee
clk_drv:0x00,q_drv:0x00,d_drv:0x00,cs0_drv:0x00,hd_drv:0x00,wp_drv:0x00
mode:DIO, clock div:2
load:0x3fff0030,len:XXXX
load:0x40078000,len:XXXX
load:0x40080400,len:XXXX
entry 0x400805e4
MicroPython v1.22.1 on 2024-01-05; ESP32 module with ESP32
Type "help()" for more information.
>>>
```

✅ **Success:** You see the MicroPython banner and `>>>` prompt

❌ **FAILURE - You see "invalid header" errors:**
```
rst:0x10 (RTCWDT_RTC_RESET),boot:0x13 (SPI_FAST_FLASH_BOOT)
invalid header: 0x5b1b6769
invalid header: 0x5b1b6769
invalid header: 0x5b1b6769
```

**If you see "invalid header", STOP and go to the troubleshooting section below.**

#### Method 2: Using mpremote

Only try this AFTER confirming boot with serial monitor:

```bash
mpremote
```

**Expected output:**
```
Connected to MicroPython at /dev/ttyUSB0
Use Ctrl-] to exit this shell
MicroPython v1.22.1 on 2024-01-05; ESP32 module with ESP32
>>>
```


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


## Troubleshooting

### Issue: "invalid header" Boot Errors (CRITICAL)

**Error message:**
```
rst:0x10 (RTCWDT_RTC_RESET),boot:0x13 (SPI_FAST_FLASH_BOOT)
invalid header: 0x5b1b6769
invalid header: 0x5b1b6769
invalid header: 0x5b1b6769
ets Jul 29 2019 12:21:46
```

**What it means:** The ESP32 bootloader cannot find valid MicroPython firmware in flash memory. This means the flash either failed or is corrupted.

**Root causes:**
1. Flash write failed but you didn't notice the error
2. Baud rate too high (460800) caused data corruption
3. Poor USB connection or unreliable USB cable
4. Flash was erased but write command never completed
5. ESP32 not reset properly after flashing
6. Wrong firmware file for your ESP32 variant

**SOLUTION - Complete reflash procedure:**

#### Step 1: Close all serial connections
```bash
# Kill any screen/mpremote sessions
# Press Ctrl+A then K (screen) or Ctrl+] (mpremote)

# On Linux, check what's using the port:
lsof | grep ttyUSB0
# Kill those processes if needed
```

#### Step 2: Try with reliable 115200 baud rate

```bash
# 1. Erase flash completely
esptool --chip esp32 --port /dev/ttyUSB0 erase_flash

# 2. Wait for "Chip erase completed successfully"

# 3. Disconnect and reconnect USB cable
# Wait 2 seconds

# 4. Flash with SLOW, reliable baud rate
esptool --chip esp32 --port /dev/ttyUSB0 --baud 115200 write_flash -z 0x1000 ESP32_GENERIC-20240105-v1.22.1.bin

# 5. VERIFY you see "Hash of data verified."
```

#### Step 3: Physically reset ESP32

**IMPORTANT:** After flashing completes:
1. Disconnect USB cable completely
2. Wait 5 seconds
3. Reconnect USB cable
4. Wait 5 seconds for boot

#### Step 4: Verify with serial monitor

```bash
# Connect with screen/PuTTY at 115200 baud
screen /dev/ttyUSB0 115200

# Press RESET button on ESP32
# You MUST see MicroPython banner
```

#### Step 5: If still failing, try these:

**A. Verify firmware file integrity**
```bash
# Check file size (should be ~1.5MB for ESP32)
ls -lh ESP32_GENERIC-20240105-v1.22.1.bin

# Re-download if suspicious
rm ESP32_GENERIC-20240105-v1.22.1.bin
wget https://micropython.org/resources/firmware/ESP32_GENERIC-20240105-v1.22.1.bin
```

**B. Try different USB port/cable**
- Use a different USB port on your computer
- Try a different USB cable (must be data cable, not charge-only)
- Avoid USB hubs - connect directly to computer

**C. Hold BOOT button during flash**
```bash
# 1. Hold BOOT/FLASH button on ESP32
# 2. Run erase_flash command while holding
# 3. Keep holding until "Connecting...." appears
# 4. Release BOOT button
# 5. Wait for erase to complete
# 6. Repeat for write_flash command
```

**D. Use manual reset timing**
```bash
esptool --chip esp32 --port /dev/ttyUSB0 --before default_reset --after hard_reset --baud 115200 write_flash -z 0x1000 ESP32_GENERIC-20240105-v1.22.1.bin
```

**E. Check for hardware issues**
```bash
# Read ESP32 chip info to verify hardware
esptool --chip esp32 --port /dev/ttyUSB0 chip_id
esptool --chip esp32 --port /dev/ttyUSB0 flash_id

# Should show chip type and flash size
# If these fail, hardware may be damaged
```

**F. Last resort - Try older MicroPython version**
```bash
# Download older stable version
wget https://micropython.org/resources/firmware/esp32-20220618-v1.19.1.bin

# Flash with 115200 baud
esptool --chip esp32 --port /dev/ttyUSB0 --baud 115200 write_flash -z 0x1000 esp32-20220618-v1.19.1.bin
```

---

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


## Quick Reference

### Complete Flash Procedure (Recommended)

```bash
# 1. Erase flash
esptool --chip esp32 --port /dev/ttyUSB0 erase_flash
# Wait for "Chip erase completed successfully"

# 2. Disconnect and reconnect USB (IMPORTANT!)
# Wait 2 seconds

# 3. Flash firmware with RELIABLE baud rate (115200)
esptool --chip esp32 --port /dev/ttyUSB0 --baud 115200 write_flash -z 0x1000 ESP32_GENERIC-20240105-v1.22.1.bin
# MUST see "Hash of data verified."

# 4. Disconnect and reconnect USB again (CRITICAL!)
# Wait 5 seconds

# 5. Verify boot with serial monitor
screen /dev/ttyUSB0 115200
# Press RESET button - should see MicroPython banner

# 6. Now safe to use mpremote
mpremote
```

### Common Commands

```bash
# Flash firmware (ESP32 Generic)
esptool --chip esp32 --port /dev/ttyUSB0 erase_flash
esptool --chip esp32 --port /dev/ttyUSB0 --baud 115200 write_flash -z 0x1000 firmware.bin

# Flash firmware (ESP32-C3)
esptool --chip esp32c3 --port /dev/ttyUSB0 --baud 115200 write_flash -z 0x0 firmware.bin

# Verify ESP32 hardware
esptool --chip esp32 --port /dev/ttyUSB0 chip_id
esptool --chip esp32 --port /dev/ttyUSB0 flash_id

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
