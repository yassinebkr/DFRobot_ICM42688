# SPDX-FileCopyrightText: 2025 Yassine Bekkari
# SPDX-License-Identifier: MIT

"""
Pytest configuration and fixtures for ICM42688 testing.

This module provides mocked hardware interfaces to test the ICM42688 library
without requiring actual hardware (Software-In-The-Loop simulation).
"""

import sys
from unittest.mock import MagicMock, Mock
import pytest

# ========================================================================
# Mock CircuitPython modules (not available in standard Python)
# ========================================================================

# Mock busio module
busio_mock = MagicMock()
busio_mock.I2C = Mock
busio_mock.SPI = Mock
sys.modules['busio'] = busio_mock

# Mock digitalio module
digitalio_mock = MagicMock()
digitalio_mock.DigitalInOut = Mock
digitalio_mock.Direction = MagicMock()
digitalio_mock.Pull = MagicMock()
sys.modules['digitalio'] = digitalio_mock

# Mock board module
board_mock = MagicMock()
board_mock.I2C = Mock
board_mock.SPI = Mock
sys.modules['board'] = board_mock

# Mock micropython module
micropython_mock = MagicMock()
sys.modules['micropython'] = micropython_mock

# Mock adafruit_bus_device
bus_device_mock = MagicMock()
sys.modules['adafruit_bus_device'] = bus_device_mock
sys.modules['adafruit_bus_device.i2c_device'] = MagicMock()
sys.modules['adafruit_bus_device.spi_device'] = MagicMock()

# ========================================================================
# Mock Hardware Classes
# ========================================================================

class MockI2CDevice:
    """Mock I2C device for testing."""

    def __init__(self, i2c, address):
        self.i2c = i2c
        self.address = address
        self.register_map = {}
        self._init_register_map()

    def _init_register_map(self):
        """Initialize register map with default values."""
        # WHO_AM_I register
        self.register_map[0x75] = 0x47  # ICM42688 chip ID

        # Default power management (sensors off)
        self.register_map[0x4E] = 0x00

        # Default ODR and range registers
        self.register_map[0x4F] = 0x06  # Gyro: 1kHz, ±2000dps
        self.register_map[0x50] = 0x06  # Accel: 1kHz, ±16g

        # Temperature data (25°C)
        self.register_map[0x1D] = 0x00
        self.register_map[0x1E] = 0x00

        # Accelerometer data (0g on X/Y, 1g on Z)
        self.register_map[0x1F] = 0x00  # X MSB
        self.register_map[0x20] = 0x00  # X LSB
        self.register_map[0x21] = 0x00  # Y MSB
        self.register_map[0x22] = 0x00  # Y LSB
        self.register_map[0x23] = 0x08  # Z MSB (~1g)
        self.register_map[0x24] = 0x00  # Z LSB

        # Gyroscope data (0 dps)
        for addr in range(0x25, 0x2B):
            self.register_map[addr] = 0x00

        # FIFO count
        self.register_map[0x2E] = 0x00  # FIFO_COUNTH
        self.register_map[0x2F] = 0x00  # FIFO_COUNTL

        # Interrupt status
        self.register_map[0x37] = 0x00  # INT_STATUS2
        self.register_map[0x38] = 0x00  # INT_STATUS3

        # Bank select
        self.register_map[0x76] = 0x00

    def __enter__(self):
        return self

    def __exit__(self, *args):
        pass

    def write(self, buffer):
        """Write to I2C device."""
        reg = buffer[0]
        if len(buffer) > 1:
            # Write data to register
            for i, value in enumerate(buffer[1:]):
                self.register_map[reg + i] = value

    def write_then_readinto(self, out_buffer, in_buffer, out_end=None, in_end=None):
        """Write then read from I2C device."""
        if out_end is None:
            out_end = len(out_buffer)
        if in_end is None:
            in_end = len(in_buffer)

        # Extract register address
        reg = out_buffer[0]

        # Read from register map
        for i in range(in_end):
            in_buffer[i] = self.register_map.get(reg + i, 0x00)


class MockSPIDevice:
    """Mock SPI device for testing."""

    def __init__(self, spi, cs, baudrate=10000000, polarity=0, phase=0):
        self.spi = spi
        self.cs = cs
        self.baudrate = baudrate
        self.register_map = {}
        self._init_register_map()

    def _init_register_map(self):
        """Initialize register map with default values."""
        # Same as I2C mock
        self.register_map[0x75] = 0x47  # WHO_AM_I
        self.register_map[0x4E] = 0x00  # PWR_MGMT0
        self.register_map[0x4F] = 0x06  # GYRO_CONFIG0
        self.register_map[0x50] = 0x06  # ACCEL_CONFIG0

        # Temperature and sensor data (same as I2C)
        for addr in range(0x1D, 0x2B):
            self.register_map[addr] = 0x00

        # Special case: Z-axis accel = ~1g
        self.register_map[0x23] = 0x08

        # FIFO and interrupts
        self.register_map[0x2E] = 0x00
        self.register_map[0x2F] = 0x00
        self.register_map[0x37] = 0x00
        self.register_map[0x38] = 0x00
        self.register_map[0x76] = 0x00

    def __enter__(self):
        return self

    def __exit__(self, *args):
        pass

    def write(self, buffer):
        """Write to SPI device."""
        reg = buffer[0] & 0x7F  # Clear read bit
        if len(buffer) > 1:
            for i, value in enumerate(buffer[1:]):
                self.register_map[reg + i] = value

    def write_readinto(self, out_buffer, in_buffer, *, out_start=0, out_end=None, in_start=0, in_end=None):
        """Write and read from SPI device."""
        reg = out_buffer[0] & 0x7F  # Extract register address
        is_read = (out_buffer[0] & 0x80) != 0

        if is_read:
            # Read operation
            if in_end is None:
                in_end = len(in_buffer)

            # First byte is dummy for SPI reads
            in_buffer[0] = 0x00

            # Read data from register map (starting from index 1)
            for i in range(1, min(in_end, len(in_buffer))):
                in_buffer[i] = self.register_map.get(reg + i - 1, 0x00)


# ========================================================================
# Pytest Fixtures
# ========================================================================

@pytest.fixture
def mock_i2c_device(monkeypatch):
    """Provide a mock I2C device."""
    mock_device = MockI2CDevice(None, 0x69)

    # Patch the I2CDevice class
    monkeypatch.setattr(
        'adafruit_bus_device.i2c_device.I2CDevice',
        lambda i2c, address: mock_device
    )

    return mock_device


@pytest.fixture
def mock_spi_device(monkeypatch):
    """Provide a mock SPI device."""
    mock_device = MockSPIDevice(None, None)

    # Patch the SPIDevice class
    monkeypatch.setattr(
        'adafruit_bus_device.spi_device.SPIDevice',
        lambda spi, cs, baudrate, polarity, phase: mock_device
    )

    return mock_device


@pytest.fixture
def mock_i2c():
    """Provide a mock I2C bus."""
    i2c = Mock()
    i2c.writeto = Mock()
    return i2c


@pytest.fixture
def mock_spi():
    """Provide a mock SPI bus."""
    spi = Mock()
    spi.configure = Mock()
    return spi


@pytest.fixture
def mock_cs_pin():
    """Provide a mock chip select pin."""
    cs = Mock()
    return cs


@pytest.fixture
def icm_i2c(mock_i2c_device, mock_i2c):
    """Provide an ICM42688 instance with I2C interface."""
    # Import after mocks are set up
    from adafruit_icm42688 import ICM42688

    return ICM42688(mock_i2c, address=0x69)


@pytest.fixture
def icm_spi(mock_spi_device, mock_spi, mock_cs_pin):
    """Provide an ICM42688 instance with SPI interface."""
    from adafruit_icm42688 import ICM42688

    return ICM42688(mock_spi, cs=mock_cs_pin)


@pytest.fixture(params=['i2c', 'spi'])
def icm(request, icm_i2c, icm_spi):
    """Provide ICM42688 instance (parameterized for both I2C and SPI)."""
    if request.param == 'i2c':
        return icm_i2c
    else:
        return icm_spi


# ========================================================================
# Helper Functions for Tests
# ========================================================================

def simulate_sensor_data(mock_device, accel_xyz, gyro_xyz, temp_c):
    """
    Simulate sensor data in the mock device.

    :param mock_device: MockI2CDevice or MockSPIDevice
    :param accel_xyz: Tuple of (x, y, z) acceleration in m/s²
    :param gyro_xyz: Tuple of (x, y, z) gyro in rad/s
    :param temp_c: Temperature in °C
    """
    from adafruit_icm42688 import registers as reg

    # Convert temperature to raw value
    temp_raw = int((temp_c - reg.TEMP_OFFSET) * reg.TEMP_SENSITIVITY)
    mock_device.register_map[0x1D] = (temp_raw >> 8) & 0xFF
    mock_device.register_map[0x1E] = temp_raw & 0xFF

    # Read current accelerometer range from ACCEL_CONFIG0 register (0x50)
    accel_config0 = mock_device.register_map.get(0x50, 0x00)
    accel_range = (accel_config0 >> 5) & 0x07  # Bits 5-7

    # Get sensitivity based on current range
    if accel_range == 0x00:  # ±16g
        accel_sensitivity = reg.ACCEL_SENSITIVITY[reg.ACCEL_RANGE_16G]
    elif accel_range == 0x01:  # ±8g
        accel_sensitivity = reg.ACCEL_SENSITIVITY[reg.ACCEL_RANGE_8G]
    elif accel_range == 0x02:  # ±4g
        accel_sensitivity = reg.ACCEL_SENSITIVITY[reg.ACCEL_RANGE_4G]
    elif accel_range == 0x03:  # ±2g
        accel_sensitivity = reg.ACCEL_SENSITIVITY[reg.ACCEL_RANGE_2G]
    else:
        accel_sensitivity = 2048.0  # Default to ±16g

    ax_raw = int((accel_xyz[0] / reg.GRAVITY_EARTH) * accel_sensitivity)
    ay_raw = int((accel_xyz[1] / reg.GRAVITY_EARTH) * accel_sensitivity)
    az_raw = int((accel_xyz[2] / reg.GRAVITY_EARTH) * accel_sensitivity)

    mock_device.register_map[0x1F] = (ax_raw >> 8) & 0xFF
    mock_device.register_map[0x20] = ax_raw & 0xFF
    mock_device.register_map[0x21] = (ay_raw >> 8) & 0xFF
    mock_device.register_map[0x22] = ay_raw & 0xFF
    mock_device.register_map[0x23] = (az_raw >> 8) & 0xFF
    mock_device.register_map[0x24] = az_raw & 0xFF

    # Read current gyroscope range from GYRO_CONFIG0 register (0x4F)
    gyro_config0 = mock_device.register_map.get(0x4F, 0x00)
    gyro_range = (gyro_config0 >> 5) & 0x07  # Bits 5-7

    # Get sensitivity based on current range
    if gyro_range == 0x00:  # ±2000 dps
        gyro_sensitivity = reg.GYRO_SENSITIVITY[reg.GYRO_RANGE_2000_DPS]
    elif gyro_range == 0x01:  # ±1000 dps
        gyro_sensitivity = reg.GYRO_SENSITIVITY[reg.GYRO_RANGE_1000_DPS]
    elif gyro_range == 0x02:  # ±500 dps
        gyro_sensitivity = reg.GYRO_SENSITIVITY[reg.GYRO_RANGE_500_DPS]
    elif gyro_range == 0x03:  # ±250 dps
        gyro_sensitivity = reg.GYRO_SENSITIVITY[reg.GYRO_RANGE_250_DPS]
    else:
        gyro_sensitivity = 16.4  # Default to ±2000 dps

    gx_raw = int((gyro_xyz[0] / reg.DEG_TO_RAD) * gyro_sensitivity)
    gy_raw = int((gyro_xyz[1] / reg.DEG_TO_RAD) * gyro_sensitivity)
    gz_raw = int((gyro_xyz[2] / reg.DEG_TO_RAD) * gyro_sensitivity)

    mock_device.register_map[0x25] = (gx_raw >> 8) & 0xFF
    mock_device.register_map[0x26] = gx_raw & 0xFF
    mock_device.register_map[0x27] = (gy_raw >> 8) & 0xFF
    mock_device.register_map[0x28] = gy_raw & 0xFF
    mock_device.register_map[0x29] = (gz_raw >> 8) & 0xFF
    mock_device.register_map[0x2A] = gz_raw & 0xFF


def simulate_interrupt(mock_device, interrupt_type):
    """
    Simulate an interrupt in the mock device.

    :param mock_device: MockI2CDevice or MockSPIDevice
    :param interrupt_type: 'wom_x', 'wom_y', 'wom_z', 'tap', 'smd', etc.
    """
    if interrupt_type == 'wom_x':
        mock_device.register_map[0x37] |= 0x01
    elif interrupt_type == 'wom_y':
        mock_device.register_map[0x37] |= 0x02
    elif interrupt_type == 'wom_z':
        mock_device.register_map[0x37] |= 0x04
    elif interrupt_type == 'smd':
        mock_device.register_map[0x37] |= 0x08
    elif interrupt_type == 'tap':
        mock_device.register_map[0x38] |= 0x01
        # Also set tap info (single tap on Z axis)
        mock_device.register_map[0x35] = 0x08 | 0x04  # Single tap, Z axis
    elif interrupt_type == 'tilt':
        mock_device.register_map[0x38] |= 0x08


def get_register_value(mock_device, reg_addr):
    """Get register value from mock device."""
    return mock_device.register_map.get(reg_addr, 0x00)
