# DFRobot_ICM42688

A comprehensive multi-platform library for the **TDK InvenSense ICM-42688-P** 6-axis MEMS MotionTracking device, combining a 3-axis gyroscope and 3-axis accelerometer with I2C and SPI support.

> Translations: [中文版](./docs/README_CN.md)

![Product Image](./resources/images/SEN0452.jpg)

**Product Link**: [DFRobot SEN0452](https://www.dfrobot.com/) — SKU: SEN0452

---

## Features

- 6-axis motion tracking (3-axis accel + 3-axis gyro)
- Configurable ranges: ±2g/±4g/±8g/±16g, ±15.625dps to ±2000dps
- Output Data Rate (ODR) from 1.5625 Hz up to 32 kHz
- 2 KB hardware FIFO with watermark interrupts & bulk read (CircuitPython/MicroPython)
- **Advanced signal processing filters** (CircuitPython/MicroPython):
  - Anti-Aliasing Filter (AAF) — Hardware analog filter
  - UI Low-Pass Filter — Selectable software filter
  - Gyro Notch Filter — Reject specific vibration frequencies
- Wake-on-Motion (WoM) and Significant Motion Detection (SMD)
- Tap detection (single/double)
- Temperature sensor
- Ultra-low-power operation
- **Performance optimized** CircuitPython driver (50-200% faster than naive implementation)

---

## Repository Structure

```
DFRobot_ICM42688/
├── README.md                  ← You are here
├── LICENSE
├── arduino/                   ← Original Arduino C++ library
│   ├── DFRobot_ICM42688.cpp
│   ├── DFRobot_ICM42688.h
│   ├── library.properties
│   ├── keywords.txt
│   └── examples/              ← Arduino example sketches
│
├── circuitpython/             ← CircuitPython driver (NEW)
│   ├── adafruit_icm42688/     ← Library module
│   ├── examples/              ← Working examples
│   └── tests/                 ← Test suite
│
├── micropython/               ← MicroPython driver (NEW)
│   ├── icm42688/              ← Library module
│   ├── examples/              ← Working examples
│   └── tests/                 ← Test suite
│
├── python/                    ← Python (Raspberry Pi) driver
│   └── raspberrypi/
│
├── docs/                      ← All documentation
│   ├── README_CN.md
│   ├── ESP32_WROOM32_QUICKSTART.md
│   ├── FEATHER_RP2040_QUICKSTART.md
│   ├── ICM42688P_5V_BREAKOUT_GUIDE.md
│   ├── SAFETY_CHECK_PROCEDURE.md
│   ├── CIRCUITPYTHON_PERFORMANCE_ANALYSIS.md
│   └── LIBRARY_OPTIMIZATION_OPPORTUNITIES.md
│
└── resources/                 ← Images and assets
```

---

## Supported Platforms

| Platform                    | Language       | Location               | Status     |
|-----------------------------|----------------|------------------------|------------|
| **Arduino**                 | C++            | `arduino/`             | ✅ Stable  |
| **CircuitPython**           | Python         | `circuitpython/`       | ✅ Stable  |
| **MicroPython** (ESP32)     | Python         | `micropython/`         | ✅ Stable  |
| **Raspberry Pi / Linux**    | Python         | `python/raspberrypi/`  | ✅ Stable  |

---

## Quick Start

### Arduino

```cpp
#include <DFRobot_ICM42688.h>

// I2C: DFRobot_ICM42688_I2C icm(DFRobot_ICM42688_I2C_H_ADDR);
DFRobot_ICM42688_SPI icm(/* csPin= */5);

void setup() {
  Serial.begin(9600);
  while (icm.begin() != 0) delay(1000);
  icm.setODRAndFSR(ACCEL, ODR_1KHZ, FSR_0);
  icm.startAccelMeasure(LN_MODE);
  icm.startGyroMeasure(LN_MODE);
}

void loop() {
  Serial.print(icm.getAccelDataX());
  Serial.print(", ");
  Serial.println(icm.getGyroDataX());
  delay(100);
}
```

See [arduino/examples/](./arduino/examples/) for more.

### CircuitPython

```python
import board
import busio
from adafruit_icm42688 import ICM42688

i2c = busio.I2C(board.SCL, board.SDA, frequency=400000)
icm = ICM42688(i2c)

# Configure filters for aerospace applications (drones, vehicles)
icm.set_aaf_filter("both", enabled=True, bandwidth_index=15)  # Anti-aliasing
icm.set_ui_filter("both", filter_order=2, bandwidth_index=3)   # Low-pass
icm.set_gyro_notch_filter(frequency_hz=120.0, axis="all")      # Reject propeller vibe

# Efficient reading options:
# 1. Single efficient read (recommended for general use)
temp, accel, gyro = icm.all_data

# 2. Manual refresh mode (per-axis access without performance penalty)
icm.refresh()  # Single read
x = icm.accel_x  # No additional I2C transaction
z = icm.gyro_z   # No additional I2C transaction

# 3. Bulk FIFO read (data logging, 10-100x faster)
icm.enable_fifo()
packets = icm.read_fifo_bulk(max_packets=64)  # Read all at once
```

See [circuitpython/examples/](./circuitpython/examples/) for more.

### MicroPython (ESP32)

```python
from machine import I2C, Pin
from icm42688 import ICM42688

i2c = I2C(0, scl=Pin(22), sda=Pin(21), freq=400000)
icm = ICM42688(i2c)

print(icm.acceleration, icm.gyro)
```

See [micropython/examples/](./micropython/examples/) and [docs/](./docs/) for installation.

### Raspberry Pi (Python)

See [python/raspberrypi/](./python/raspberrypi/).

---

## Documentation

All detailed documentation lives in [`docs/`](./docs/):

### Hardware Guides
- [ESP32-WROOM-32 Quick Start](./docs/ESP32_WROOM32_QUICKSTART.md) — Wiring & setup for ESP32-WROOM-32
- [Feather RP2040 Quick Start](./docs/FEATHER_RP2040_QUICKSTART.md) — Wiring & setup for Feather RP2040 (+ RFM95)
- [ICM42688P 5V Breakout Guide](./docs/ICM42688P_5V_BREAKOUT_GUIDE.md) — Notes for boards with "5V" labelled power pin
- [Safety Check Procedure](./docs/SAFETY_CHECK_PROCEDURE.md) — Voltage verification before connecting hardware

### Performance Tuning
- [CircuitPython Performance Analysis](./docs/CIRCUITPYTHON_PERFORMANCE_ANALYSIS.md) — Why Python is slower & how to close the gap
- [Library Optimization Opportunities](./docs/LIBRARY_OPTIMIZATION_OPPORTUNITIES.md) — Internal optimizations applied to the driver

### Wiring References (in-place)
- [CircuitPython ESP32-WROOM-32 Wiring Guide](./circuitpython/examples/ESP32_WROOM32_WIRING_GUIDE.md)
- [CircuitPython Feather RP2040 Wiring Guide](./circuitpython/examples/FEATHER_RP2040_WIRING_GUIDE.md)
- [MicroPython ESP32 Installation](./micropython/INSTALL_ESP32.md)

---

## Installation

### Arduino
1. Open the Arduino IDE → **Tools → Manage Libraries**
2. Search for `DFRobot_ICM42688` and install
3. Or copy [`arduino/`](./arduino/) contents to your `Arduino/libraries/DFRobot_ICM42688/` folder

### CircuitPython
```bash
# Copy library to CIRCUITPY drive
cp -r circuitpython/adafruit_icm42688 /media/CIRCUITPY/lib/
```

### MicroPython
```bash
# Copy library to your board (e.g. via ampy or mpremote)
mpremote cp -r micropython/icm42688 :
```

---

## Compatibility

| MCU                 | Arduino | CircuitPython | MicroPython | Notes               |
|---------------------|:-------:|:-------------:|:-----------:|---------------------|
| FireBeetle ESP32    |   ✅    |      ✅       |     ✅      |                     |
| ESP32-WROOM-32      |   ✅    |      ✅       |     ✅      | See docs/ guides    |
| ESP32-C3 Super Mini |   ✅    |      ✅       |     ✅      | 3.3V logic only     |
| Feather RP2040      |   ✅    |      ✅       |     —       | RFM95 LoRa friendly |
| FireBeetle ESP8266  |   ✅    |      —        |     —       |                     |
| FireBeetle M0       |   ✅    |      —        |     —       |                     |
| Raspberry Pi        |   —     |      —        |     —       | Use python/         |
| Arduino UNO / MEGA  |   ⚠️    |      —        |     —       | 3.3V level required |

---

## Arduino API Reference

The Arduino library exposes the original DFRobot API. Key methods:

| Method                                                     | Description                                       |
|------------------------------------------------------------|---------------------------------------------------|
| `int begin(void)`                                          | Initialize the sensor                             |
| `float getTemperature(void)`                               | Read temperature (°C)                             |
| `float getAccelDataX/Y/Z(void)`                            | Read accelerometer axis (mg)                      |
| `float getGyroDataX/Y/Z(void)`                             | Read gyroscope axis (dps)                         |
| `bool setODRAndFSR(uint8_t who, uint8_t ODR, uint8_t FSR)` | Configure ODR and full-scale range                |
| `void startAccelMeasure(uint8_t mode)`                     | Enable accelerometer (LN/LP mode)                 |
| `void startGyroMeasure(uint8_t mode)`                      | Enable gyroscope (LN/STANDBY mode)                |
| `void startTempMeasure()`                                  | Enable temperature sensor                         |
| `void tapDetectionInit(uint8_t accelMode)`                 | Initialize tap detection                          |
| `void wakeOnMotionInit()`                                  | Initialize wake-on-motion                         |
| `void setWOMTh(uint8_t axis, uint8_t threshold)`           | Set wake-on-motion threshold (1g/256 resolution)  |
| `void setWOMInterrupt(uint8_t axis)`                       | Enable wake-on-motion interrupt                   |
| `void enableSMDInterrupt(uint8_t mode)`                    | Enable significant motion detection               |
| `void startFIFOMode() / sotpFIFOMode()`                    | Enable / disable FIFO                             |
| `void getFIFOData()`                                       | Read FIFO buffer                                  |
| `void setINTMode(...)`                                     | Configure interrupt pin mode                      |

Full headers and source: [`arduino/DFRobot_ICM42688.h`](./arduino/DFRobot_ICM42688.h)

---

## Performance Notes

- **Arduino (C++)**: 300+ Hz easily over SPI
- **CircuitPython (optimized)**: 150–200 Hz over I2C @ 400 kHz, 800–1200 Hz over SPI @ 24 MHz
- **MicroPython**: Similar to CircuitPython

For details, see [docs/CIRCUITPYTHON_PERFORMANCE_ANALYSIS.md](./docs/CIRCUITPYTHON_PERFORMANCE_ANALYSIS.md).

---

## History

- **2021/09/28** — v1.0.0: Initial Arduino release
- **2025**     — CircuitPython port added
- **2025**     — MicroPython port added
- **2025**     — Multi-platform documentation, performance optimizations, repository reorganization

---

## Credits

- Original Arduino library: **yangfeng** ([feng.yang@dfrobot.com](mailto:feng.yang@dfrobot.com)), DFRobot — [website](https://www.dfrobot.com/)
- CircuitPython / MicroPython ports and performance work: **Yassine Benkhira**

---

## License

[MIT License](./LICENSE)
