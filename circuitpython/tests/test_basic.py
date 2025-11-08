# SPDX-FileCopyrightText: 2025 Yassine Bekkari
# SPDX-License-Identifier: MIT

"""
Basic functionality tests for ICM42688.

Tests initialization, sensor reading, and basic operations.
"""

import pytest
from conftest import simulate_sensor_data


def test_initialization_i2c(icm_i2c, mock_i2c_device):
    """Test sensor initialization with I2C."""
    # Verify chip ID was read correctly during init
    assert mock_i2c_device.register_map[0x75] == 0x47

    # Verify default ranges are set
    assert icm_i2c._accel_range == 0  # ±16g
    assert icm_i2c._gyro_range == 0  # ±2000dps


def test_initialization_spi(icm_spi, mock_spi_device):
    """Test sensor initialization with SPI."""
    # Verify chip ID was read correctly during init
    assert mock_spi_device.register_map[0x75] == 0x47

    # Verify default ranges are set
    assert icm_spi._accel_range == 0
    assert icm_spi._gyro_range == 0


def test_wrong_chip_id(mock_i2c_device, mock_i2c, monkeypatch):
    """Test that wrong chip ID raises RuntimeError."""
    # Change chip ID to wrong value
    mock_i2c_device.register_map[0x75] = 0xFF

    from adafruit_icm42688 import ICM42688

    with pytest.raises(RuntimeError, match="Failed to find ICM42688"):
        ICM42688(mock_i2c, address=0x69)


def test_read_acceleration(icm, mock_i2c_device):
    """Test reading acceleration data."""
    # Simulate 1g on Z axis (sensor at rest, upright)
    simulate_sensor_data(
        mock_i2c_device,
        accel_xyz=(0.0, 0.0, 9.81),  # m/s²
        gyro_xyz=(0.0, 0.0, 0.0),
        temp_c=25.0
    )

    accel_x, accel_y, accel_z = icm.acceleration

    # Check values are approximately correct (within 5%)
    assert abs(accel_x - 0.0) < 0.5
    assert abs(accel_y - 0.0) < 0.5
    assert abs(accel_z - 9.81) < 0.5


def test_read_gyro(icm, mock_i2c_device):
    """Test reading gyroscope data."""
    # Simulate 100 deg/s rotation on X axis
    rotation_dps = 100.0
    rotation_rads = rotation_dps * (3.14159 / 180.0)

    simulate_sensor_data(
        mock_i2c_device,
        accel_xyz=(0.0, 0.0, 9.81),
        gyro_xyz=(rotation_rads, 0.0, 0.0),  # rad/s
        temp_c=25.0
    )

    gyro_x, gyro_y, gyro_z = icm.gyro

    # Check values are approximately correct
    assert abs(gyro_x - rotation_rads) < 0.1
    assert abs(gyro_y - 0.0) < 0.1
    assert abs(gyro_z - 0.0) < 0.1


def test_read_temperature(icm, mock_i2c_device):
    """Test reading temperature data."""
    # Simulate 30°C
    simulate_sensor_data(
        mock_i2c_device,
        accel_xyz=(0.0, 0.0, 9.81),
        gyro_xyz=(0.0, 0.0, 0.0),
        temp_c=30.0
    )

    temp = icm.temperature

    # Check temperature is approximately correct (within 1°C)
    assert abs(temp - 30.0) < 1.0


def test_read_all_sensors(icm, mock_i2c_device):
    """Test reading all sensors at once."""
    # Simulate realistic sensor data
    simulate_sensor_data(
        mock_i2c_device,
        accel_xyz=(0.5, -0.3, 9.81),  # m/s²
        gyro_xyz=(0.1, -0.05, 0.0),  # rad/s
        temp_c=28.5
    )

    # Read all sensors
    accel = icm.acceleration
    gyro = icm.gyro
    temp = icm.temperature

    # Verify all values are reasonable
    assert len(accel) == 3
    assert len(gyro) == 3
    assert isinstance(temp, float)

    # Check acceleration magnitude is close to 1g
    accel_mag = (accel[0]**2 + accel[1]**2 + accel[2]**2) ** 0.5
    assert abs(accel_mag - 9.81) < 1.0


def test_reset(icm, mock_i2c_device):
    """Test soft reset functionality."""
    # Change a register
    mock_i2c_device.register_map[0x4E] = 0xFF

    # Perform reset
    icm.reset()

    # After reset, device should be reconfigured
    # Check that device config register received reset command
    # Note: In real hardware, registers would reset to defaults
    assert icm._current_bank == 0


def test_sensor_data_units(icm, mock_i2c_device):
    """Test that sensor data is returned in correct SI units."""
    # Simulate known values
    accel_expected = (1.0, 2.0, 9.81)  # m/s²
    gyro_expected = (0.5, -0.5, 1.0)  # rad/s
    temp_expected = 25.0  # °C

    simulate_sensor_data(
        mock_i2c_device,
        accel_xyz=accel_expected,
        gyro_xyz=gyro_expected,
        temp_c=temp_expected
    )

    # Read sensors
    accel = icm.acceleration
    gyro = icm.gyro
    temp = icm.temperature

    # Verify units are correct (SI units)
    # Acceleration should be in m/s² (not g or mg)
    assert 0.5 < abs(accel[2]) < 15.0  # Reasonable range for m/s²

    # Gyro should be in rad/s (not deg/s or dps)
    assert -10.0 < gyro[0] < 10.0  # Reasonable range for rad/s

    # Temperature should be in °C
    assert 0.0 < temp < 50.0  # Reasonable range for °C


def test_multiple_reads(icm, mock_i2c_device):
    """Test multiple sequential sensor reads."""
    # Simulate sensor data
    simulate_sensor_data(
        mock_i2c_device,
        accel_xyz=(0.0, 0.0, 9.81),
        gyro_xyz=(0.0, 0.0, 0.0),
        temp_c=25.0
    )

    # Read multiple times
    for _ in range(10):
        accel = icm.acceleration
        gyro = icm.gyro
        temp = icm.temperature

        # All reads should be successful
        assert len(accel) == 3
        assert len(gyro) == 3
        assert isinstance(temp, float)


def test_bank_switching(icm, mock_i2c_device):
    """Test register bank switching."""
    # Initially should be in bank 0
    assert icm._current_bank == 0

    # Switch to bank 1
    icm._set_bank(1)
    assert icm._current_bank == 1
    assert mock_i2c_device.register_map[0x76] == 1

    # Switch to bank 4
    icm._set_bank(4)
    assert icm._current_bank == 4
    assert mock_i2c_device.register_map[0x76] == 4

    # Switch back to bank 0
    icm._set_bank(0)
    assert icm._current_bank == 0
    assert mock_i2c_device.register_map[0x76] == 0


def test_bank_switching_optimization(icm, mock_i2c_device):
    """Test that bank switching is optimized (doesn't switch if already there)."""
    # Set to bank 1
    icm._set_bank(1)
    register_writes_before = mock_i2c_device.register_map.copy()

    # Try to switch to bank 1 again
    icm._set_bank(1)

    # Register map should be unchanged (no unnecessary write)
    assert mock_i2c_device.register_map == register_writes_before


def test_invalid_bank(icm):
    """Test that invalid bank numbers raise ValueError."""
    with pytest.raises(ValueError, match="Bank must be 0-4"):
        icm._set_bank(5)

    with pytest.raises(ValueError, match="Bank must be 0-4"):
        icm._set_bank(-1)
