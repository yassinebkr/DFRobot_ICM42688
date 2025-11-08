# SPDX-FileCopyrightText: 2025 Yassine Bekkari
# SPDX-License-Identifier: MIT

"""
Configuration tests for ICM42688.

Tests range, ODR, and power mode configuration.
"""

import pytest
from adafruit_icm42688 import registers as reg
from conftest import get_register_value


def test_set_accelerometer_range(icm, mock_i2c_device):
    """Test setting accelerometer range."""
    # Test each valid range
    ranges = [
        reg.ACCEL_RANGE_2G,
        reg.ACCEL_RANGE_4G,
        reg.ACCEL_RANGE_8G,
        reg.ACCEL_RANGE_16G
    ]

    for range_val in ranges:
        icm.accelerometer_range = range_val
        assert icm.accelerometer_range == range_val
        assert icm._accel_range == range_val

        # Verify register was written correctly
        config_reg = get_register_value(mock_i2c_device, 0x50)  # ACCEL_CONFIG0
        assert (config_reg >> 5) & 0x07 == range_val


def test_set_gyro_range(icm, mock_i2c_device):
    """Test setting gyroscope range."""
    # Test each valid range
    ranges = [
        reg.GYRO_RANGE_15_625_DPS,
        reg.GYRO_RANGE_31_25_DPS,
        reg.GYRO_RANGE_62_5_DPS,
        reg.GYRO_RANGE_125_DPS,
        reg.GYRO_RANGE_250_DPS,
        reg.GYRO_RANGE_500_DPS,
        reg.GYRO_RANGE_1000_DPS,
        reg.GYRO_RANGE_2000_DPS
    ]

    for range_val in ranges:
        icm.gyro_range = range_val
        assert icm.gyro_range == range_val
        assert icm._gyro_range == range_val

        # Verify register was written correctly
        config_reg = get_register_value(mock_i2c_device, 0x4F)  # GYRO_CONFIG0
        assert (config_reg >> 5) & 0x07 == range_val


def test_invalid_accelerometer_range(icm):
    """Test that invalid accelerometer range raises ValueError."""
    with pytest.raises(ValueError):
        icm.accelerometer_range = 99  # Invalid range


def test_invalid_gyro_range(icm):
    """Test that invalid gyroscope range raises ValueError."""
    with pytest.raises(ValueError):
        icm.gyro_range = 99  # Invalid range


def test_set_accelerometer_odr(icm, mock_i2c_device):
    """Test setting accelerometer output data rate."""
    # Test various ODR values
    odr_values = [
        reg.ODR_1KHZ,
        reg.ODR_200HZ,
        reg.ODR_100HZ,
        reg.ODR_50HZ,
        reg.ODR_25HZ
    ]

    for odr in odr_values:
        icm.accelerometer_data_rate = odr
        assert icm.accelerometer_data_rate == odr
        assert icm._accel_odr == odr

        # Verify register was written correctly
        config_reg = get_register_value(mock_i2c_device, 0x50)  # ACCEL_CONFIG0
        assert (config_reg & 0x0F) == odr


def test_set_gyro_odr(icm, mock_i2c_device):
    """Test setting gyroscope output data rate."""
    # Test various ODR values
    odr_values = [
        reg.ODR_1KHZ,
        reg.ODR_200HZ,
        reg.ODR_100HZ,
        reg.ODR_50HZ
    ]

    for odr in odr_values:
        icm.gyro_data_rate = odr
        assert icm.gyro_data_rate == odr
        assert icm._gyro_odr == odr

        # Verify register was written correctly
        config_reg = get_register_value(mock_i2c_device, 0x4F)  # GYRO_CONFIG0
        assert (config_reg & 0x0F) == odr


def test_invalid_odr(icm):
    """Test that invalid ODR raises ValueError."""
    with pytest.raises(ValueError):
        icm.accelerometer_data_rate = 0  # Invalid

    with pytest.raises(ValueError):
        icm.gyro_data_rate = 16  # Invalid


def test_set_power_mode_accel_only(icm, mock_i2c_device):
    """Test setting accelerometer power mode."""
    # Test low-power mode
    icm.set_power_mode(accel_mode=reg.ACCEL_MODE_LP)

    pwr_reg = get_register_value(mock_i2c_device, 0x4E)  # PWR_MGMT0
    assert (pwr_reg & 0x03) == reg.ACCEL_MODE_LP

    # Test low-noise mode
    icm.set_power_mode(accel_mode=reg.ACCEL_MODE_LN)

    pwr_reg = get_register_value(mock_i2c_device, 0x4E)
    assert (pwr_reg & 0x03) == reg.ACCEL_MODE_LN

    # Test off mode
    icm.set_power_mode(accel_mode=reg.ACCEL_MODE_OFF)

    pwr_reg = get_register_value(mock_i2c_device, 0x4E)
    assert (pwr_reg & 0x03) == reg.ACCEL_MODE_OFF


def test_set_power_mode_gyro_only(icm, mock_i2c_device):
    """Test setting gyroscope power mode."""
    # Test low-noise mode
    icm.set_power_mode(gyro_mode=reg.GYRO_MODE_LN)

    pwr_reg = get_register_value(mock_i2c_device, 0x4E)  # PWR_MGMT0
    assert ((pwr_reg >> 2) & 0x03) == reg.GYRO_MODE_LN

    # Test standby mode
    icm.set_power_mode(gyro_mode=reg.GYRO_MODE_STANDBY)

    pwr_reg = get_register_value(mock_i2c_device, 0x4E)
    assert ((pwr_reg >> 2) & 0x03) == reg.GYRO_MODE_STANDBY

    # Test off mode
    icm.set_power_mode(gyro_mode=reg.GYRO_MODE_OFF)

    pwr_reg = get_register_value(mock_i2c_device, 0x4E)
    assert ((pwr_reg >> 2) & 0x03) == reg.GYRO_MODE_OFF


def test_set_power_mode_both(icm, mock_i2c_device):
    """Test setting both accelerometer and gyroscope power modes."""
    icm.set_power_mode(
        accel_mode=reg.ACCEL_MODE_LN,
        gyro_mode=reg.GYRO_MODE_LN
    )

    pwr_reg = get_register_value(mock_i2c_device, 0x4E)
    assert (pwr_reg & 0x03) == reg.ACCEL_MODE_LN
    assert ((pwr_reg >> 2) & 0x03) == reg.GYRO_MODE_LN


def test_set_power_mode_temp_control(icm, mock_i2c_device):
    """Test temperature sensor enable/disable."""
    # Disable temperature sensor
    icm.set_power_mode(temp_enabled=False)

    pwr_reg = get_register_value(mock_i2c_device, 0x4E)
    assert (pwr_reg & reg.TEMP_DIS_BIT) != 0  # Bit should be set (disabled)

    # Enable temperature sensor
    icm.set_power_mode(temp_enabled=True)

    pwr_reg = get_register_value(mock_i2c_device, 0x4E)
    assert (pwr_reg & reg.TEMP_DIS_BIT) == 0  # Bit should be clear (enabled)


def test_invalid_power_mode(icm):
    """Test that invalid power modes raise ValueError."""
    with pytest.raises(ValueError, match="Invalid accelerometer mode"):
        icm.set_power_mode(accel_mode=99)

    with pytest.raises(ValueError, match="Invalid gyroscope mode"):
        icm.set_power_mode(gyro_mode=99)


def test_range_affects_sensitivity(icm, mock_i2c_device):
    """Test that changing range affects sensitivity correctly."""
    from conftest import simulate_sensor_data

    # Set to ±2g range
    icm.accelerometer_range = reg.ACCEL_RANGE_2G

    # Simulate 1g acceleration
    simulate_sensor_data(
        mock_i2c_device,
        accel_xyz=(0.0, 0.0, 9.81),
        gyro_xyz=(0.0, 0.0, 0.0),
        temp_c=25.0
    )

    accel1 = icm.acceleration

    # Set to ±16g range
    icm.accelerometer_range = reg.ACCEL_RANGE_16G

    # Same 1g acceleration, but with different sensitivity
    simulate_sensor_data(
        mock_i2c_device,
        accel_xyz=(0.0, 0.0, 9.81),
        gyro_xyz=(0.0, 0.0, 0.0),
        temp_c=25.0
    )

    accel2 = icm.acceleration

    # Both should read approximately 1g (same physical value)
    # even though internal sensitivity changed
    assert abs(accel1[2] - 9.81) < 0.5
    assert abs(accel2[2] - 9.81) < 0.5


def test_config_persistence(icm, mock_i2c_device):
    """Test that configuration persists across reads."""
    # Set configuration
    icm.accelerometer_range = reg.ACCEL_RANGE_4G
    icm.gyro_range = reg.GYRO_RANGE_500_DPS
    icm.accelerometer_data_rate = reg.ODR_200HZ
    icm.gyro_data_rate = reg.ODR_200HZ

    # Read sensors multiple times
    for _ in range(5):
        _ = icm.acceleration
        _ = icm.gyro

    # Configuration should still be the same
    assert icm.accelerometer_range == reg.ACCEL_RANGE_4G
    assert icm.gyro_range == reg.GYRO_RANGE_500_DPS
    assert icm.accelerometer_data_rate == reg.ODR_200HZ
    assert icm.gyro_data_rate == reg.ODR_200HZ


def test_multiple_config_changes(icm, mock_i2c_device):
    """Test multiple configuration changes in sequence."""
    # Change configuration multiple times
    for range_val in [reg.ACCEL_RANGE_2G, reg.ACCEL_RANGE_4G, reg.ACCEL_RANGE_8G, reg.ACCEL_RANGE_16G]:
        icm.accelerometer_range = range_val
        assert icm.accelerometer_range == range_val

        # Read sensor to ensure it still works
        accel = icm.acceleration
        assert len(accel) == 3


def test_config_after_reset(icm, mock_i2c_device):
    """Test that reset restores default configuration."""
    # Change configuration
    icm.accelerometer_range = reg.ACCEL_RANGE_2G
    icm.gyro_range = reg.GYRO_RANGE_500_DPS

    # Reset sensor
    icm.reset()

    # Configuration should be back to defaults
    assert icm._accel_range == reg.ACCEL_RANGE_16G
    assert icm._gyro_range == reg.GYRO_RANGE_2000_DPS
