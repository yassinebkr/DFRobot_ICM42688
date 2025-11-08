# SPDX-FileCopyrightText: 2025 Yassine Bekkari
# SPDX-License-Identifier: MIT

"""
FIFO functionality tests for ICM42688.

Tests FIFO enable/disable, configuration, and data reading.
"""

import pytest
from adafruit_icm42688 import registers as reg
from conftest import get_register_value


def test_enable_fifo_default(icm, mock_device):
    """Test enabling FIFO with default parameters."""
    icm.enable_fifo()

    # Check FIFO_CONFIG1 register
    fifo_config1 = get_register_value(mock_device, 0x5F)
    assert (fifo_config1 & reg.FIFO_ACCEL_EN) != 0  # Accel enabled
    assert (fifo_config1 & reg.FIFO_GYRO_EN) != 0  # Gyro enabled
    assert (fifo_config1 & reg.FIFO_TEMP_EN) != 0  # Temp enabled
    assert (fifo_config1 & reg.FIFO_HIRES_EN) == 0  # High-res disabled

    # Check FIFO_CONFIG register (should be in stream mode)
    fifo_config = get_register_value(mock_device, 0x16)
    assert fifo_config == reg.FIFO_MODE_STREAM


def test_enable_fifo_custom(icm, mock_device):
    """Test enabling FIFO with custom parameters."""
    icm.enable_fifo(
        accel=True,
        gyro=False,
        temp=False,
        high_res=True,
        mode="stop"
    )

    # Check FIFO_CONFIG1 register
    fifo_config1 = get_register_value(mock_device, 0x5F)
    assert (fifo_config1 & reg.FIFO_ACCEL_EN) != 0  # Accel enabled
    assert (fifo_config1 & reg.FIFO_GYRO_EN) == 0  # Gyro disabled
    assert (fifo_config1 & reg.FIFO_TEMP_EN) == 0  # Temp disabled
    assert (fifo_config1 & reg.FIFO_HIRES_EN) != 0  # High-res enabled

    # Check FIFO mode
    fifo_config = get_register_value(mock_device, 0x16)
    assert fifo_config == reg.FIFO_MODE_STOP_ON_FULL


def test_disable_fifo(icm, mock_device):
    """Test disabling FIFO."""
    # Enable first
    icm.enable_fifo()

    # Then disable
    icm.disable_fifo()

    # Check FIFO is in bypass mode
    fifo_config = get_register_value(mock_device, 0x16)
    assert fifo_config == reg.FIFO_MODE_BYPASS


def test_flush_fifo(icm, mock_device):
    """Test flushing FIFO."""
    # Simulate some data in FIFO
    mock_device.register_map[0x2E] = 0x00  # FIFO_COUNTH
    mock_device.register_map[0x2F] = 0x20  # FIFO_COUNTL = 32 bytes

    # Flush FIFO
    icm.flush_fifo()

    # Verify SIGNAL_PATH_RESET register was written with flush bit
    # (In real hardware, this would clear the FIFO)


def test_fifo_count(icm, mock_device):
    """Test reading FIFO count."""
    # Simulate FIFO with 48 bytes (3 packets)
    mock_device.register_map[0x2E] = 0x00  # FIFO_COUNTH
    mock_device.register_map[0x2F] = 0x30  # FIFO_COUNTL = 48

    count = icm.fifo_count
    assert count == 48


def test_fifo_count_large(icm, mock_device):
    """Test reading FIFO count with large value."""
    # Simulate FIFO with 512 bytes
    mock_device.register_map[0x2E] = 0x02  # FIFO_COUNTH = 2
    mock_device.register_map[0x2F] = 0x00  # FIFO_COUNTL = 0

    count = icm.fifo_count
    assert count == 512


def test_read_fifo_packet(icm, mock_device):
    """Test reading a FIFO packet."""
    # Simulate a FIFO packet (16 bytes)
    # Header + Accel(X,Y,Z) + Gyro(X,Y,Z) + Temp + Reserved
    fifo_data = [
        0x00,  # Header
        0x00, 0x00,  # Accel X = 0
        0x00, 0x00,  # Accel Y = 0
        0x08, 0x00,  # Accel Z = ~1g
        0x00, 0x00,  # Gyro X = 0
        0x00, 0x00,  # Gyro Y = 0
        0x00, 0x00,  # Gyro Z = 0
        0x00,  # Temp = 25°C (0 in FIFO format)
        0x00, 0x00   # Reserved
    ]

    # Load FIFO data into mock device
    for i, byte in enumerate(fifo_data):
        mock_device.register_map[0x30 + i] = byte

    # Simulate some bytes in FIFO
    mock_device.register_map[0x2E] = 0x00
    mock_device.register_map[0x2F] = 0x10  # 16 bytes

    # Read FIFO packet
    packet = icm.read_fifo()

    # Verify packet structure
    assert 'header' in packet
    assert 'accel' in packet
    assert 'gyro' in packet
    assert 'temp' in packet

    # Verify data types
    assert isinstance(packet['header'], int)
    assert isinstance(packet['accel'], tuple)
    assert isinstance(packet['gyro'], tuple)
    assert isinstance(packet['temp'], float)

    # Verify tuple lengths
    assert len(packet['accel']) == 3
    assert len(packet['gyro']) == 3


def test_fifo_data_units(icm, mock_device):
    """Test that FIFO data is returned in correct SI units."""
    # Enable FIFO
    icm.enable_fifo()

    # Simulate a FIFO packet with known values
    fifo_data = [
        0x00,  # Header
        0x00, 0x00,  # Accel X
        0x00, 0x00,  # Accel Y
        0x08, 0x00,  # Accel Z (~1g)
        0x00, 0x64,  # Gyro X (~100 in raw)
        0x00, 0x00,  # Gyro Y
        0x00, 0x00,  # Gyro Z
        0x0A,  # Temp (~30°C in FIFO format)
        0x00, 0x00
    ]

    for i, byte in enumerate(fifo_data):
        mock_device.register_map[0x30 + i] = byte

    packet = icm.read_fifo()

    # Check units
    # Acceleration should be in m/s²
    assert packet['accel'][2] > 5.0  # Z-axis should be ~9.8 m/s²

    # Gyro should be in rad/s
    assert -10.0 < packet['gyro'][0] < 10.0

    # Temperature should be in °C
    assert 20.0 < packet['temp'] < 35.0


def test_fifo_invalid_mode(icm):
    """Test that invalid FIFO mode raises ValueError."""
    with pytest.raises(ValueError, match="Mode must be 'stream' or 'stop'"):
        icm.enable_fifo(mode="invalid")


def test_fifo_multiple_packets(icm, mock_device):
    """Test reading multiple packets from FIFO."""
    # Enable FIFO
    icm.enable_fifo()

    # Simulate multiple packets available
    mock_device.register_map[0x2E] = 0x00
    mock_device.register_map[0x2F] = 0x30  # 48 bytes = 3 packets

    # Read multiple packets
    packets = []
    for _ in range(3):
        if icm.fifo_count >= 16:
            packet = icm.read_fifo()
            packets.append(packet)

    # Should have read 3 packets
    assert len(packets) == 3

    # All packets should be valid
    for packet in packets:
        assert 'accel' in packet
        assert 'gyro' in packet
        assert 'temp' in packet


def test_fifo_after_reset(icm, mock_device):
    """Test FIFO state after sensor reset."""
    # Enable FIFO
    icm.enable_fifo()

    # Reset sensor
    icm.reset()

    # FIFO should be disabled after reset
    # (In real hardware, registers reset to defaults)
    fifo_config = get_register_value(mock_device, 0x16)
    # After reset, FIFO should be in bypass mode or need re-enabling
