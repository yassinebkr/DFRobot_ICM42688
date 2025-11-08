# SPDX-FileCopyrightText: 2025 Yassine Bekkari
# SPDX-License-Identifier: MIT

"""
CircuitPython driver for the ICM42688 6-axis IMU sensor.

This library provides a CircuitPython interface for the TDK InvenSense ICM42688
high-performance 6-axis MEMS motion tracking device. It supports both I2C and SPI
communication and provides access to all sensor features including FIFO, interrupts,
and advanced motion detection.

* Author(s): Yassine Bekkari (based on DFRobot_ICM42688)

Implementation Notes
--------------------

**Hardware:**

* `Adafruit Feather RP2040 with RFM95 LoRa <https://www.adafruit.com/product/5714>`_
* ICM42688 6-Axis IMU breakout board

**Software and Dependencies:**

* Adafruit CircuitPython firmware for the supported boards:
  https://circuitpython.org/downloads

* Adafruit's Bus Device library:
  https://github.com/adafruit/Adafruit_CircuitPython_BusDevice

* Adafruit's Register library (optional):
  https://github.com/adafruit/Adafruit_CircuitPython_Register
"""

import time
import struct
from micropython import const

try:
    from typing import Optional, Tuple, Union
    from busio import I2C, SPI
    from digitalio import DigitalInOut
except ImportError:
    pass

from adafruit_bus_device import i2c_device, spi_device

# Import all register definitions and constants
from . import registers as reg

__version__ = "1.0.0"
__repo__ = "https://github.com/yassinebkr/DFRobot_ICM42688.git"


class ICM42688:
    """
    Driver for the ICM42688 6-axis IMU sensor.

    Supports both I2C and SPI communication. Provides access to 3-axis accelerometer,
    3-axis gyroscope, temperature sensor, FIFO, interrupts, and motion detection.

    :param i2c_or_spi: The I2C or SPI bus the ICM42688 is connected to
    :param address: I2C address (default 0x69). Ignored for SPI.
    :param cs: Chip select pin (required for SPI, ignored for I2C)
    :param baudrate: SPI baudrate (default 10MHz). Ignored for I2C.

    **Quickstart: Importing and using the device**

    Here is an example of using the :class:`ICM42688` class with I2C.
    First you will need to import the libraries to use the sensor

    .. code-block:: python

        import board
        import adafruit_icm42688

    Once this is done you can define your `board.I2C` object and define your sensor object

    .. code-block:: python

        i2c = board.I2C()  # uses board.SCL and board.SDA
        icm = adafruit_icm42688.ICM42688(i2c)

    Now you have access to the sensor's properties

    .. code-block:: python

        accel_x, accel_y, accel_z = icm.acceleration
        gyro_x, gyro_y, gyro_z = icm.gyro
        temp = icm.temperature
    """

    def __init__(
        self,
        i2c_or_spi: Union[I2C, SPI],
        address: int = reg.ICM42688_I2C_ADDR_HIGH,
        cs: Optional[DigitalInOut] = None,
        baudrate: int = 10000000,
    ) -> None:
        """Initialize the ICM42688 sensor."""

        # Determine if using I2C or SPI
        self._spi = None
        self._i2c = None

        if hasattr(i2c_or_spi, "writeto"):  # I2C interface
            self._i2c = i2c_device.I2CDevice(i2c_or_spi, address)
            self._use_spi = False
        elif hasattr(i2c_or_spi, "configure"):  # SPI interface
            if cs is None:
                raise ValueError("Chip select (cs) pin required for SPI")
            self._spi = spi_device.SPIDevice(
                i2c_or_spi, cs, baudrate=baudrate, polarity=0, phase=0
            )
            self._use_spi = True
        else:
            raise ValueError("Expected I2C or SPI bus object")

        # Internal state tracking
        self._current_bank = 0  # Track current register bank
        self._accel_range = reg.ACCEL_RANGE_16G  # Default ±16g
        self._gyro_range = reg.GYRO_RANGE_2000_DPS  # Default ±2000dps
        self._accel_odr = reg.ODR_1KHZ  # Default 1kHz
        self._gyro_odr = reg.ODR_1KHZ  # Default 1kHz

        # Pre-allocated buffers for efficiency (avoid allocations in read loops)
        self._buffer = bytearray(20)  # Max FIFO packet size
        self._data_buffer = bytearray(14)  # All sensor data (temp + accel + gyro)

        # Initialize the sensor
        self._init_sensor()

    def _init_sensor(self) -> None:
        """
        Initialize the ICM42688 sensor.

        Performs sensor ID check, soft reset, and configures defaults.
        Raises RuntimeError if sensor not found or initialization fails.
        """
        # Switch to bank 0
        self._set_bank(0)

        # Check WHO_AM_I register
        chip_id = self._read_register_byte(reg.REG_WHO_AM_I)
        if chip_id != reg.ICM42688_CHIP_ID:
            raise RuntimeError(
                f"Failed to find ICM42688! Chip ID 0x{chip_id:02X} != 0x{reg.ICM42688_CHIP_ID:02X}"
            )

        # Perform soft reset
        self._write_register_byte(reg.REG_DEVICE_CONFIG, 0x01)
        time.sleep(reg.RESET_DELAY)

        # Switch back to bank 0 (reset changes to bank 0)
        self._set_bank(0)

        # Configure default settings
        # - Accelerometer: ±16g, 1kHz ODR, Low-Noise mode
        # - Gyroscope: ±2000dps, 1kHz ODR, Low-Noise mode
        # - Temperature sensor enabled

        # Set accelerometer range and ODR
        accel_config = (self._accel_range << 5) | (self._accel_odr & 0x0F)
        self._write_register_byte(reg.REG_ACCEL_CONFIG0, accel_config)

        # Set gyroscope range and ODR
        gyro_config = (self._gyro_range << 5) | (self._gyro_odr & 0x0F)
        self._write_register_byte(reg.REG_GYRO_CONFIG0, gyro_config)

        # Enable accelerometer and gyroscope in Low-Noise mode, temperature sensor enabled
        # PWR_MGMT0: TEMP_DIS=0, IDLE=0, GYRO_MODE=11 (LN), ACCEL_MODE=11 (LN)
        pwr_mgmt = (reg.GYRO_MODE_LN << 2) | reg.ACCEL_MODE_LN
        self._write_register_byte(reg.REG_PWR_MGMT0, pwr_mgmt)

        # Wait for mode transition (200µs required)
        time.sleep(reg.MODE_CHANGE_DELAY)

        # Additional 45ms for gyroscope startup
        time.sleep(reg.GYRO_STARTUP_TIME)

    # ========================================================================
    # Low-level register access methods
    # ========================================================================

    def _set_bank(self, bank: int) -> None:
        """
        Switch to the specified register bank (0-4).

        The ICM42688 has a bank-switched register architecture.
        This method optimizes by only switching if necessary.

        :param bank: Bank number (0-4)
        """
        if bank < 0 or bank > 4:
            raise ValueError("Bank must be 0-4")

        if self._current_bank != bank:
            # Bank select register is always in bank 0 address space
            # Can be written from any bank without switching first
            if self._use_spi:
                with self._spi as spi:
                    spi.write(bytes([reg.REG_BANK_SEL, bank]))
            else:
                with self._i2c as i2c:
                    i2c.write(bytes([reg.REG_BANK_SEL, bank]))

            self._current_bank = bank

    def _read_register_byte(self, register: int) -> int:
        """
        Read a single byte from a register.

        :param register: Register address
        :return: Register value (0-255)
        """
        if self._use_spi:
            self._buffer[0] = register | 0x80  # Set read bit for SPI
            with self._spi as spi:
                spi.write_readinto(self._buffer, self._buffer, end=2)
            return self._buffer[1]
        else:
            with self._i2c as i2c:
                i2c.write_then_readinto(bytes([register]), self._buffer, out_end=1, in_end=1)
            return self._buffer[0]

    def _read_register_bytes(self, register: int, length: int, buf: Optional[bytearray] = None) -> bytearray:
        """
        Read multiple bytes from consecutive registers.

        :param register: Starting register address
        :param length: Number of bytes to read
        :param buf: Optional buffer to read into (must be >= length)
        :return: bytearray containing read data
        """
        if buf is None:
            buf = bytearray(length)
        elif len(buf) < length:
            raise ValueError("Buffer too small")

        if self._use_spi:
            # SPI: write register address with read bit, then read data
            self._buffer[0] = register | 0x80
            with self._spi as spi:
                spi.write_readinto(self._buffer, buf, out_end=1, in_start=0, in_end=length)
                # Shift data down by 1 (first byte is dummy)
                for i in range(length):
                    buf[i] = buf[i + 1] if i + 1 < len(buf) else 0
        else:
            # I2C: write register address, then read data
            with self._i2c as i2c:
                i2c.write_then_readinto(bytes([register]), buf, out_end=1, in_end=length)

        return buf

    def _write_register_byte(self, register: int, value: int) -> None:
        """
        Write a single byte to a register.

        :param register: Register address
        :param value: Value to write (0-255)
        """
        if value < 0 or value > 255:
            raise ValueError("Value must be 0-255")

        if self._use_spi:
            with self._spi as spi:
                spi.write(bytes([register & 0x7F, value]))  # Clear read bit for SPI
        else:
            with self._i2c as i2c:
                i2c.write(bytes([register, value]))

    def _write_register_bytes(self, register: int, data: bytes) -> None:
        """
        Write multiple bytes to consecutive registers.

        :param register: Starting register address
        :param data: Data to write
        """
        if self._use_spi:
            # SPI: send register address + data
            buf = bytearray(len(data) + 1)
            buf[0] = register & 0x7F  # Clear read bit
            buf[1:] = data
            with self._spi as spi:
                spi.write(buf)
        else:
            # I2C: send register address + data
            buf = bytearray(len(data) + 1)
            buf[0] = register
            buf[1:] = data
            with self._i2c as i2c:
                i2c.write(buf)

    def _read_sensor_data(self) -> Tuple[float, Tuple[float, float, float], Tuple[float, float, float]]:
        """
        Read all sensor data in a single burst read (optimized).

        Reads 14 bytes starting from TEMP_DATA1:
        - Temperature (2 bytes)
        - Accelerometer X, Y, Z (6 bytes)
        - Gyroscope X, Y, Z (6 bytes)

        :return: Tuple of (temperature, (accel_x, accel_y, accel_z), (gyro_x, gyro_y, gyro_z))
        """
        # Ensure we're in bank 0
        self._set_bank(0)

        # Read all 14 bytes in one transaction
        self._read_register_bytes(reg.REG_TEMP_DATA1, 14, self._data_buffer)

        # Unpack big-endian signed 16-bit values
        # Format: >7h = big-endian, 7 signed shorts
        temp_raw, ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw = struct.unpack(
            '>7h', self._data_buffer
        )

        # Convert temperature: (RAW / 132.48) + 25°C
        temperature = (temp_raw / reg.TEMP_SENSITIVITY) + reg.TEMP_OFFSET

        # Convert accelerometer to m/s²
        accel_sensitivity = reg.ACCEL_SENSITIVITY[self._accel_range]
        accel_x = (ax_raw / accel_sensitivity) * reg.GRAVITY_EARTH
        accel_y = (ay_raw / accel_sensitivity) * reg.GRAVITY_EARTH
        accel_z = (az_raw / accel_sensitivity) * reg.GRAVITY_EARTH

        # Convert gyroscope to rad/s
        gyro_sensitivity = reg.GYRO_SENSITIVITY[self._gyro_range]
        gyro_x = (gx_raw / gyro_sensitivity) * reg.DEG_TO_RAD
        gyro_y = (gy_raw / gyro_sensitivity) * reg.DEG_TO_RAD
        gyro_z = (gz_raw / gyro_sensitivity) * reg.DEG_TO_RAD

        return temperature, (accel_x, accel_y, accel_z), (gyro_x, gyro_y, gyro_z)

    # ========================================================================
    # Public properties for sensor readings (Adafruit API pattern)
    # ========================================================================

    @property
    def acceleration(self) -> Tuple[float, float, float]:
        """
        Acceleration measured by the sensor in m/s².

        Returns a 3-tuple of X, Y, Z axis values in meters per second squared.

        .. code-block:: python

            accel_x, accel_y, accel_z = icm.acceleration
        """
        _, accel, _ = self._read_sensor_data()
        return accel

    @property
    def gyro(self) -> Tuple[float, float, float]:
        """
        Rotation rates measured by the gyroscope in rad/s.

        Returns a 3-tuple of X, Y, Z axis values in radians per second.

        .. code-block:: python

            gyro_x, gyro_y, gyro_z = icm.gyro
        """
        _, _, gyro = self._read_sensor_data()
        return gyro

    @property
    def temperature(self) -> float:
        """
        Temperature measured by the sensor's internal thermometer in degrees Celsius.

        .. code-block:: python

            temp_c = icm.temperature
        """
        temp, _, _ = self._read_sensor_data()
        return temp

    # ========================================================================
    # Configuration methods
    # ========================================================================

    @property
    def accelerometer_range(self) -> int:
        """
        Accelerometer full-scale range setting.

        Returns one of:
        - ACCEL_RANGE_2G: ±2g
        - ACCEL_RANGE_4G: ±4g
        - ACCEL_RANGE_8G: ±8g
        - ACCEL_RANGE_16G: ±16g (default)
        """
        return self._accel_range

    @accelerometer_range.setter
    def accelerometer_range(self, value: int) -> None:
        """Set accelerometer full-scale range."""
        if value not in reg.ACCEL_SENSITIVITY:
            raise ValueError(
                "Range must be ACCEL_RANGE_2G, ACCEL_RANGE_4G, ACCEL_RANGE_8G, or ACCEL_RANGE_16G"
            )

        self._set_bank(0)

        # Read current config, modify range bits only
        current = self._read_register_byte(reg.REG_ACCEL_CONFIG0)
        new_config = (current & 0x1F) | (value << 5)
        self._write_register_byte(reg.REG_ACCEL_CONFIG0, new_config)

        self._accel_range = value

    @property
    def gyro_range(self) -> int:
        """
        Gyroscope full-scale range setting.

        Returns one of:
        - GYRO_RANGE_15_625_DPS: ±15.625 dps
        - GYRO_RANGE_31_25_DPS: ±31.25 dps
        - GYRO_RANGE_62_5_DPS: ±62.5 dps
        - GYRO_RANGE_125_DPS: ±125 dps
        - GYRO_RANGE_250_DPS: ±250 dps
        - GYRO_RANGE_500_DPS: ±500 dps
        - GYRO_RANGE_1000_DPS: ±1000 dps
        - GYRO_RANGE_2000_DPS: ±2000 dps (default)
        """
        return self._gyro_range

    @gyro_range.setter
    def gyro_range(self, value: int) -> None:
        """Set gyroscope full-scale range."""
        if value not in reg.GYRO_SENSITIVITY:
            raise ValueError("Invalid gyro range")

        self._set_bank(0)

        # Read current config, modify range bits only
        current = self._read_register_byte(reg.REG_GYRO_CONFIG0)
        new_config = (current & 0x1F) | (value << 5)
        self._write_register_byte(reg.REG_GYRO_CONFIG0, new_config)

        self._gyro_range = value

    @property
    def accelerometer_data_rate(self) -> int:
        """Accelerometer output data rate (ODR)."""
        return self._accel_odr

    @accelerometer_data_rate.setter
    def accelerometer_data_rate(self, value: int) -> None:
        """
        Set accelerometer output data rate (ODR).

        Valid values: ODR_1KHZ, ODR_200HZ, ODR_100HZ, ODR_50HZ, etc.
        See registers.py for full list.
        """
        if value < 1 or value > 15:
            raise ValueError("Invalid ODR value")

        self._set_bank(0)

        # Read current config, modify ODR bits only
        current = self._read_register_byte(reg.REG_ACCEL_CONFIG0)
        new_config = (current & 0xF0) | (value & 0x0F)
        self._write_register_byte(reg.REG_ACCEL_CONFIG0, new_config)

        self._accel_odr = value

    @property
    def gyro_data_rate(self) -> int:
        """Gyroscope output data rate (ODR)."""
        return self._gyro_odr

    @gyro_data_rate.setter
    def gyro_data_rate(self, value: int) -> None:
        """
        Set gyroscope output data rate (ODR).

        Valid values: ODR_1KHZ, ODR_200HZ, ODR_100HZ, ODR_50HZ, etc.
        See registers.py for full list.
        """
        if value < 1 or value > 15:
            raise ValueError("Invalid ODR value")

        self._set_bank(0)

        # Read current config, modify ODR bits only
        current = self._read_register_byte(reg.REG_GYRO_CONFIG0)
        new_config = (current & 0xF0) | (value & 0x0F)
        self._write_register_byte(reg.REG_GYRO_CONFIG0, new_config)

        self._gyro_odr = value

    def set_power_mode(
        self,
        accel_mode: Optional[int] = None,
        gyro_mode: Optional[int] = None,
        temp_enabled: bool = True
    ) -> None:
        """
        Set power modes for accelerometer and gyroscope.

        :param accel_mode: Accelerometer mode (ACCEL_MODE_OFF, ACCEL_MODE_LP, ACCEL_MODE_LN)
        :param gyro_mode: Gyroscope mode (GYRO_MODE_OFF, GYRO_MODE_STANDBY, GYRO_MODE_LN)
        :param temp_enabled: Enable temperature sensor (default True)

        .. code-block:: python

            from adafruit_icm42688 import registers as reg

            # Low power mode for battery applications
            icm.set_power_mode(accel_mode=reg.ACCEL_MODE_LP, gyro_mode=reg.GYRO_MODE_OFF)

            # High performance mode
            icm.set_power_mode(accel_mode=reg.ACCEL_MODE_LN, gyro_mode=reg.GYRO_MODE_LN)
        """
        self._set_bank(0)

        # Read current power management register
        current = self._read_register_byte(reg.REG_PWR_MGMT0)

        # Build new config
        new_config = current

        if accel_mode is not None:
            if accel_mode not in (reg.ACCEL_MODE_OFF, reg.ACCEL_MODE_LP, reg.ACCEL_MODE_LN):
                raise ValueError("Invalid accelerometer mode")
            new_config = (new_config & 0xFC) | accel_mode  # Clear bits 0-1, set new value

        if gyro_mode is not None:
            if gyro_mode not in (reg.GYRO_MODE_OFF, reg.GYRO_MODE_STANDBY, reg.GYRO_MODE_LN):
                raise ValueError("Invalid gyroscope mode")
            new_config = (new_config & 0xF3) | (gyro_mode << 2)  # Clear bits 2-3, set new value

        # Temperature sensor control (bit 5)
        if temp_enabled:
            new_config &= ~reg.TEMP_DIS_BIT  # Clear bit to enable
        else:
            new_config |= reg.TEMP_DIS_BIT  # Set bit to disable

        # Write new config
        self._write_register_byte(reg.REG_PWR_MGMT0, new_config)

        # Wait for mode transition
        time.sleep(reg.MODE_CHANGE_DELAY)

        # If enabling gyro, wait for startup
        if gyro_mode == reg.GYRO_MODE_LN:
            time.sleep(reg.GYRO_STARTUP_TIME)

    def reset(self) -> None:
        """
        Perform a soft reset of the sensor.

        This resets all registers to their default values and requires
        re-initialization. The sensor will be in bank 0 after reset.
        """
        self._set_bank(0)
        self._write_register_byte(reg.REG_DEVICE_CONFIG, 0x01)
        time.sleep(reg.RESET_DELAY)
        self._current_bank = 0

        # Re-initialize with default settings
        self._init_sensor()

    # ========================================================================
    # FIFO Methods
    # ========================================================================

    def enable_fifo(
        self,
        accel: bool = True,
        gyro: bool = True,
        temp: bool = True,
        high_res: bool = False,
        mode: str = "stream"
    ) -> None:
        """
        Enable and configure the FIFO buffer.

        The FIFO can store up to 2KB of sensor data, useful for batch reading
        and reducing CPU wake-ups in low-power applications.

        :param accel: Include accelerometer data in FIFO
        :param gyro: Include gyroscope data in FIFO
        :param temp: Include temperature data in FIFO
        :param high_res: Enable 20-bit high-resolution mode (default: 16-bit)
        :param mode: FIFO mode - "stream" (overwrite old data) or "stop" (stop when full)

        .. code-block:: python

            # Enable FIFO with all sensors
            icm.enable_fifo()

            # Read FIFO data
            while True:
                count = icm.fifo_count
                if count >= 16:  # At least one packet available
                    data = icm.read_fifo()
                    print(f"Accel: {data['accel']}, Gyro: {data['gyro']}")
                time.sleep(0.1)
        """
        self._set_bank(0)

        # Configure FIFO packet format (FIFO_CONFIG1)
        fifo_config1 = 0
        if accel:
            fifo_config1 |= reg.FIFO_ACCEL_EN
        if gyro:
            fifo_config1 |= reg.FIFO_GYRO_EN
        if temp:
            fifo_config1 |= reg.FIFO_TEMP_EN
        if high_res:
            fifo_config1 |= reg.FIFO_HIRES_EN

        self._write_register_byte(reg.REG_FIFO_CONFIG1, fifo_config1)

        # Set FIFO mode (FIFO_CONFIG)
        if mode == "stream":
            fifo_mode = reg.FIFO_MODE_STREAM
        elif mode == "stop":
            fifo_mode = reg.FIFO_MODE_STOP_ON_FULL
        else:
            raise ValueError("Mode must be 'stream' or 'stop'")

        self._write_register_byte(reg.REG_FIFO_CONFIG, fifo_mode)

    def disable_fifo(self) -> None:
        """
        Disable the FIFO buffer and set to bypass mode.
        """
        self._set_bank(0)
        self._write_register_byte(reg.REG_FIFO_CONFIG, reg.FIFO_MODE_BYPASS)

    def flush_fifo(self) -> None:
        """
        Clear all data from the FIFO buffer.
        """
        self._set_bank(0)

        # Read SIGNAL_PATH_RESET, set FIFO flush bit, write back
        # Bit 1 is FIFO flush (write 1 to flush)
        self._write_register_byte(reg.REG_SIGNAL_PATH_RESET, 0x02)

    @property
    def fifo_count(self) -> int:
        """
        Number of bytes currently stored in the FIFO buffer.

        Returns the count in bytes. Each packet is 16 bytes (basic mode)
        or 20 bytes (high-resolution mode).
        """
        self._set_bank(0)

        # Read 2-byte count (big-endian)
        count_high = self._read_register_byte(reg.REG_FIFO_COUNTH)
        count_low = self._read_register_byte(reg.REG_FIFO_COUNTL)
        return (count_high << 8) | count_low

    def read_fifo(self) -> dict:
        """
        Read one packet from the FIFO buffer.

        Returns a dictionary with keys 'header', 'accel', 'gyro', 'temp' (if enabled).
        Call this when fifo_count >= 16 (or 20 for high-res mode).

        :return: Dictionary with sensor data from FIFO packet

        .. code-block:: python

            if icm.fifo_count >= 16:
                packet = icm.read_fifo()
                ax, ay, az = packet['accel']  # m/s²
                gx, gy, gz = packet['gyro']   # rad/s
                temp = packet['temp']         # °C
        """
        self._set_bank(0)

        # Read 16-byte FIFO packet (basic mode)
        # Packet format:
        #   Byte 0: Header
        #   Bytes 1-2: Accel X
        #   Bytes 3-4: Accel Y
        #   Bytes 5-6: Accel Z
        #   Bytes 7-8: Gyro X
        #   Bytes 9-10: Gyro Y
        #   Bytes 11-12: Gyro Z
        #   Byte 13: Temp
        #   Bytes 14-15: Timestamp/reserved

        self._read_register_bytes(reg.REG_FIFO_DATA, 16, self._buffer)

        # Parse header
        header = self._buffer[0]

        # Unpack data (big-endian signed 16-bit)
        ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw = struct.unpack(
            '>6h', self._buffer[1:13]
        )
        temp_raw = struct.unpack('>b', bytes([self._buffer[13]]))[0]  # Signed 8-bit

        # Convert to physical units
        accel_sensitivity = reg.ACCEL_SENSITIVITY[self._accel_range]
        accel = (
            (ax_raw / accel_sensitivity) * reg.GRAVITY_EARTH,
            (ay_raw / accel_sensitivity) * reg.GRAVITY_EARTH,
            (az_raw / accel_sensitivity) * reg.GRAVITY_EARTH
        )

        gyro_sensitivity = reg.GYRO_SENSITIVITY[self._gyro_range]
        gyro = (
            (gx_raw / gyro_sensitivity) * reg.DEG_TO_RAD,
            (gy_raw / gyro_sensitivity) * reg.DEG_TO_RAD,
            (gz_raw / gyro_sensitivity) * reg.DEG_TO_RAD
        )

        # Temperature from FIFO uses different formula: RAW/2.07 + 25
        temperature = (temp_raw / 2.07) + 25.0

        return {
            'header': header,
            'accel': accel,
            'gyro': gyro,
            'temp': temperature
        }

    # ========================================================================
    # Interrupt Configuration
    # ========================================================================

    def configure_interrupt(
        self,
        pin: int = 1,
        polarity: str = "high",
        mode: str = "pulse",
        drive: str = "push-pull"
    ) -> None:
        """
        Configure interrupt pin behavior.

        :param pin: Interrupt pin number (1 or 2)
        :param polarity: "high" for active-high, "low" for active-low
        :param mode: "pulse" for pulse mode, "latch" for latched mode
        :param drive: "push-pull" or "open-drain"

        .. code-block:: python

            # Configure INT1 as active-high, latched, push-pull
            icm.configure_interrupt(pin=1, polarity="high", mode="latch", drive="push-pull")
        """
        if pin not in (1, 2):
            raise ValueError("Pin must be 1 or 2")
        if polarity not in ("high", "low"):
            raise ValueError("Polarity must be 'high' or 'low'")
        if mode not in ("pulse", "latch"):
            raise ValueError("Mode must be 'latch' or 'pulse'")
        if drive not in ("push-pull", "open-drain"):
            raise ValueError("Drive must be 'push-pull' or 'open-drain'")

        self._set_bank(0)

        # Read current config
        current = self._read_register_byte(reg.REG_INT_CONFIG)
        config = current

        # Configure INT1
        if pin == 1:
            if polarity == "high":
                config |= reg.INT1_POLARITY_HIGH
            else:
                config &= ~reg.INT1_POLARITY_HIGH

            if drive == "push-pull":
                config |= reg.INT1_DRIVE_PP
            else:
                config &= ~reg.INT1_DRIVE_PP

            if mode == "latch":
                config |= reg.INT1_MODE_LATCH
            else:
                config &= ~reg.INT1_MODE_LATCH

        # Configure INT2
        else:
            if polarity == "high":
                config |= reg.INT2_POLARITY_HIGH
            else:
                config &= ~reg.INT2_POLARITY_HIGH

            if drive == "push-pull":
                config |= reg.INT2_DRIVE_PP
            else:
                config &= ~reg.INT2_DRIVE_PP

            if mode == "latch":
                config |= reg.INT2_MODE_LATCH
            else:
                config &= ~reg.INT2_MODE_LATCH

        self._write_register_byte(reg.REG_INT_CONFIG, config)

    def read_interrupt_status(self) -> dict:
        """
        Read and clear interrupt status.

        Returns a dictionary with interrupt flags.

        :return: Dictionary with interrupt status flags

        .. code-block:: python

            status = icm.read_interrupt_status()
            if status['wom_x']:
                print("Wake-on-motion detected on X axis")
            if status['tap']:
                print("Tap detected")
        """
        self._set_bank(0)

        # Read interrupt status registers (reading clears them)
        status = self._read_register_byte(reg.REG_INT_STATUS)
        status2 = self._read_register_byte(reg.REG_INT_STATUS2)
        status3 = self._read_register_byte(reg.REG_INT_STATUS3)

        return {
            'data_ready': bool(status & reg.INT_STATUS_DRDY),
            'fifo_threshold': bool(status & reg.INT_STATUS_FIFO_THS),
            'fifo_full': bool(status & reg.INT_STATUS_FIFO_FULL),
            'wom_x': bool(status2 & reg.INT_STATUS_WOM_X),
            'wom_y': bool(status2 & reg.INT_STATUS_WOM_Y),
            'wom_z': bool(status2 & reg.INT_STATUS_WOM_Z),
            'smd': bool(status2 & reg.INT_STATUS_SMD),
            'tap': bool(status3 & reg.INT_STATUS_TAP),
            'sleep': bool(status3 & reg.INT_STATUS_SLEEP),
            'wake': bool(status3 & reg.INT_STATUS_WAKE),
            'tilt': bool(status3 & reg.INT_STATUS_TILT),
            'step_overflow': bool(status3 & reg.INT_STATUS_STEP_CNT_OVF),
            'step_detected': bool(status3 & reg.INT_STATUS_STEP_DET)
        }

    # ========================================================================
    # Motion Detection Features
    # ========================================================================

    def enable_wake_on_motion(
        self,
        threshold: int = 50,
        axes: str = "all",
        int_pin: int = 1
    ) -> None:
        """
        Enable wake-on-motion (WOM) detection.

        :param threshold: Motion threshold (0-255). Each LSB = ~3.9mg
        :param axes: Which axes to monitor: "x", "y", "z", "all"
        :param int_pin: Interrupt pin to use (1 or 2)

        .. code-block:: python

            # Trigger interrupt when motion > 50 * 3.9mg = ~195mg on any axis
            icm.enable_wake_on_motion(threshold=50, axes="all", int_pin=1)
        """
        if threshold < 0 or threshold > 255:
            raise ValueError("Threshold must be 0-255")
        if axes not in ("x", "y", "z", "all"):
            raise ValueError("Axes must be 'x', 'y', 'z', or 'all'")
        if int_pin not in (1, 2):
            raise ValueError("Pin must be 1 or 2")

        # Configure accelerometer for wake-on-motion
        # WOM requires LP mode with ODR = 50Hz recommended
        self._set_bank(0)

        # Set accel to 50Hz LP mode
        self.accelerometer_data_rate = reg.ODR_50HZ
        self.set_power_mode(accel_mode=reg.ACCEL_MODE_LP)

        # Switch to bank 4 to set thresholds
        self._set_bank(4)

        if axes in ("x", "all"):
            self._write_register_byte(reg.REG_ACCEL_WOM_X_THR, threshold)
        if axes in ("y", "all"):
            self._write_register_byte(reg.REG_ACCEL_WOM_Y_THR, threshold)
        if axes in ("z", "all"):
            self._write_register_byte(reg.REG_ACCEL_WOM_Z_THR, threshold)

        # Configure WOM mode and interrupt routing
        self._set_bank(0)

        # Configure SMD_CONFIG: WOM mode = compare to previous, OR mode
        smd_config = reg.WOM_MODE_CMP_PREV | reg.WOM_INT_MODE_OR | 0x01  # SMD mode = 1
        self._write_register_byte(reg.REG_SMD_CONFIG, smd_config)

        # Route WOM interrupts to specified pin
        int_source_reg = reg.REG_INT_SOURCE1 if int_pin == 1 else reg.REG_INT_SOURCE4

        # Enable interrupts for selected axes
        int_mask = 0
        if axes in ("x", "all"):
            int_mask |= reg.INT_STATUS_WOM_X
        if axes in ("y", "all"):
            int_mask |= reg.INT_STATUS_WOM_Y
        if axes in ("z", "all"):
            int_mask |= reg.INT_STATUS_WOM_Z

        self._write_register_byte(int_source_reg, int_mask)

    def enable_tap_detection(
        self,
        mode: str = "low-noise",
        int_pin: int = 1
    ) -> None:
        """
        Enable tap detection (single and double tap).

        :param mode: Accelerometer mode - "low-noise" (1kHz) or "low-power" (500Hz)
        :param int_pin: Interrupt pin to use (1 or 2)

        .. code-block:: python

            icm.enable_tap_detection(mode="low-noise", int_pin=1)

            # Check for taps
            status = icm.read_interrupt_status()
            if status['tap']:
                tap_info = icm.read_tap_info()
                print(f"Tap: {tap_info['count']} on axis {tap_info['axis']}")
        """
        if mode not in ("low-noise", "low-power"):
            raise ValueError("Mode must be 'low-noise' or 'low-power'")
        if int_pin not in (1, 2):
            raise ValueError("Pin must be 1 or 2")

        self._set_bank(0)

        # Configure accelerometer for tap detection
        if mode == "low-power":
            self.accelerometer_data_rate = reg.ODR_500HZ
            self.set_power_mode(accel_mode=reg.ACCEL_MODE_LP)
        else:  # low-noise
            self.accelerometer_data_rate = reg.ODR_1KHZ
            self.set_power_mode(accel_mode=reg.ACCEL_MODE_LN)

        # Configure accel filter for tap detection
        # 2nd order filter, BW = ODR/2
        accel_config1 = (2 << 1)  # ACCEL_UI_FILT_ORD = 2 (2nd order)
        self._write_register_byte(reg.REG_ACCEL_CONFIG1, accel_config1)

        # Set GYRO_ACCEL_CONFIG0 for filter bandwidth
        self._write_register_byte(reg.REG_GYRO_ACCEL_CONFIG0, 0x00)  # BW = ODR/2

        # Switch to bank 4 for APEX configuration
        self._set_bank(4)

        # Configure tap timing (APEX_CONFIG8)
        # TAP_TMIN=3, TAP_TAVG=3, TAP_TMAX=2
        tap_config8 = 0x03 | (0x03 << 3) | (0x02 << 5)
        self._write_register_byte(reg.REG_APEX_CONFIG8, tap_config8)

        # Configure tap thresholds (APEX_CONFIG7)
        # TAP_MIN_JERK_THR=17, TAP_MAX_PEAK_TOL=1
        tap_config7 = 0x01 | (17 << 2)
        self._write_register_byte(reg.REG_APEX_CONFIG7, tap_config7)

        # Route tap interrupt to specified pin
        int_source_reg = reg.REG_INT_SOURCE6 if int_pin == 1 else reg.REG_INT_SOURCE7
        self._write_register_byte(int_source_reg, 0x01)  # Enable tap interrupt (bit 0)

        # Enable tap detection in APEX_CONFIG0
        self._set_bank(0)
        apex_config0 = self._read_register_byte(reg.REG_APEX_CONFIG0)
        apex_config0 |= reg.TAP_ENABLE
        self._write_register_byte(reg.REG_APEX_CONFIG0, apex_config0)

    def read_tap_info(self) -> dict:
        """
        Read tap detection information.

        Returns a dictionary with tap details after a tap interrupt.

        :return: Dictionary with 'count' ("single" or "double"), 'axis' ("x", "y", or "z"),
                 and 'direction' (0 or 1)

        .. code-block:: python

            if icm.read_interrupt_status()['tap']:
                info = icm.read_tap_info()
                print(f"{info['count']} tap on {info['axis']} axis")
        """
        self._set_bank(0)

        # Read APEX_DATA4 register
        data = self._read_register_byte(reg.REG_APEX_DATA4)

        # Parse tap information
        tap_num = data & reg.TAP_NUM_MASK
        tap_count = "double" if tap_num == reg.TAP_DOUBLE else "single"

        tap_axis_bits = data & reg.TAP_AXIS_MASK
        if tap_axis_bits == reg.TAP_AXIS_X:
            axis = "x"
        elif tap_axis_bits == reg.TAP_AXIS_Y:
            axis = "y"
        else:
            axis = "z"

        direction = bool(data & reg.TAP_DIR_MASK)

        return {
            'count': tap_count,
            'axis': axis,
            'direction': direction,
            'raw': data
        }

    def enable_significant_motion_detection(
        self,
        mode: str = "short",
        int_pin: int = 1
    ) -> None:
        """
        Enable significant motion detection (SMD).

        SMD detects two wake-on-motion events separated by 1 sec (short) or 3 sec (long).
        Useful for detecting device pick-up or movement.

        :param mode: "short" (1 sec wait) or "long" (3 sec wait)
        :param int_pin: Interrupt pin to use (1 or 2)

        .. code-block:: python

            icm.enable_significant_motion_detection(mode="short", int_pin=1)
        """
        if mode not in ("short", "long"):
            raise ValueError("Mode must be 'short' or 'long'")
        if int_pin not in (1, 2):
            raise ValueError("Pin must be 1 or 2")

        # First configure wake-on-motion (SMD requires WOM)
        self.enable_wake_on_motion(threshold=50, axes="all", int_pin=int_pin)

        # Configure SMD mode
        self._set_bank(0)

        smd_mode_val = reg.SMD_MODE_SHORT if mode == "short" else reg.SMD_MODE_LONG
        smd_config = smd_mode_val | reg.WOM_MODE_CMP_PREV | reg.WOM_INT_MODE_OR
        self._write_register_byte(reg.REG_SMD_CONFIG, smd_config)

        # Route SMD interrupt
        int_source_reg = reg.REG_INT_SOURCE1 if int_pin == 1 else reg.REG_INT_SOURCE4
        self._write_register_byte(int_source_reg, reg.INT_STATUS_SMD)

    def disable_motion_detection(self) -> None:
        """
        Disable all motion detection features (WOM, SMD, tap).
        """
        self._set_bank(0)

        # Disable SMD/WOM
        self._write_register_byte(reg.REG_SMD_CONFIG, reg.SMD_MODE_OFF)

        # Disable APEX features
        self._write_register_byte(reg.REG_APEX_CONFIG0, 0x00)

        # Clear interrupt sources
        self._write_register_byte(reg.REG_INT_SOURCE1, 0x00)
        self._write_register_byte(reg.REG_INT_SOURCE4, 0x00)

        # Switch to bank 4 and clear thresholds
        self._set_bank(4)
        self._write_register_byte(reg.REG_ACCEL_WOM_X_THR, 0x00)
        self._write_register_byte(reg.REG_ACCEL_WOM_Y_THR, 0x00)
        self._write_register_byte(reg.REG_ACCEL_WOM_Z_THR, 0x00)

        self._set_bank(0)
