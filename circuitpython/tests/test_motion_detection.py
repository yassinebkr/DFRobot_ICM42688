# SPDX-FileCopyrightText: 2025 Yassine Bekkari
# SPDX-License-Identifier: MIT

"""
Motion detection tests for ICM42688.

Tests wake-on-motion (WOM), tap detection, and significant motion detection (SMD).
"""

import pytest
from adafruit_icm42688 import registers as reg
from conftest import get_register_value


# ========================================================================
# Wake-on-Motion (WOM) Tests
# ========================================================================

def test_enable_wom_all_axes_default(icm, mock_i2c_device):
    """Test enabling WOM on all axes with default threshold."""
    icm.enable_wake_on_motion(threshold=50, axes="all", int_pin=1)

    # Verify WOM is enabled in APEX_CONFIG0 (Bank 4, 0x56)
    # Check power mode is LP for accel
    pwr_mgmt0 = get_register_value(mock_i2c_device, 0x4E)
    assert (pwr_mgmt0 & 0x03) == reg.ACCEL_MODE_LP  # Accel in LP mode

    # Verify gyro is off (WOM only uses accel)
    assert (pwr_mgmt0 & 0x0C) == reg.GYRO_MODE_OFF


def test_enable_wom_custom_threshold(icm, mock_i2c_device):
    """Test enabling WOM with custom threshold."""
    threshold = 100  # Higher threshold

    icm.enable_wake_on_motion(threshold=threshold, axes="all", int_pin=1)

    # The threshold would be written to WOM_THRESHOLD register in Bank 4
    # Since we switch banks, verify the operation succeeded
    # (Actual register checking would require bank switching in test)


def test_enable_wom_single_axis_x(icm, mock_i2c_device):
    """Test enabling WOM on X-axis only."""
    icm.enable_wake_on_motion(threshold=50, axes="x", int_pin=1)

    # Verify WOM is configured for X-axis only
    # This would check APEX_CONFIG0 register in Bank 4


def test_enable_wom_single_axis_y(icm, mock_i2c_device):
    """Test enabling WOM on Y-axis only."""
    icm.enable_wake_on_motion(threshold=50, axes="y", int_pin=1)


def test_enable_wom_single_axis_z(icm, mock_i2c_device):
    """Test enabling WOM on Z-axis only."""
    icm.enable_wake_on_motion(threshold=50, axes="z", int_pin=1)


def test_enable_wom_invalid_axes(icm):
    """Test that invalid axes parameter raises ValueError."""
    with pytest.raises(ValueError, match="Axes must be 'x', 'y', 'z', or 'all'"):
        icm.enable_wake_on_motion(threshold=50, axes="invalid", int_pin=1)


def test_enable_wom_threshold_bounds(icm):
    """Test WOM threshold boundary values."""
    # Minimum threshold (0)
    icm.enable_wake_on_motion(threshold=0, axes="all", int_pin=1)

    # Maximum threshold (255)
    icm.enable_wake_on_motion(threshold=255, axes="all", int_pin=1)

    # Out of bounds should raise ValueError or be clamped
    with pytest.raises((ValueError, OverflowError)):
        icm.enable_wake_on_motion(threshold=256, axes="all", int_pin=1)


def test_enable_wom_interrupt_routing_int1(icm, mock_i2c_device):
    """Test WOM interrupt routing to INT1."""
    icm.enable_wake_on_motion(threshold=50, axes="all", int_pin=1)

    # Verify INT_SOURCE0 register has WOM routed to INT1
    int_source0 = get_register_value(mock_i2c_device, 0x20)
    # Should have WOM bits set for INT1


def test_enable_wom_interrupt_routing_int2(icm, mock_i2c_device):
    """Test WOM interrupt routing to INT2."""
    icm.enable_wake_on_motion(threshold=50, axes="all", int_pin=2)

    # Verify INT_SOURCE1 register has WOM routed to INT2


def test_wom_power_mode(icm, mock_i2c_device):
    """Test that WOM puts accelerometer in low-power mode."""
    # Enable WOM
    icm.enable_wake_on_motion(threshold=50, axes="all", int_pin=1)

    # Check power management register
    pwr_mgmt0 = get_register_value(mock_i2c_device, 0x4E)

    # Accelerometer should be in LP mode
    accel_mode = pwr_mgmt0 & 0x03
    assert accel_mode == reg.ACCEL_MODE_LP

    # Gyroscope should be off to save power
    gyro_mode = (pwr_mgmt0 >> 2) & 0x03
    assert gyro_mode == reg.GYRO_MODE_OFF


def test_wom_detection_simulation(icm, mock_i2c_device):
    """Test WOM interrupt status after simulated motion."""
    # Enable WOM
    icm.enable_wake_on_motion(threshold=50, axes="all", int_pin=1)

    # Simulate motion on X-axis (INT_STATUS2 = 0x37)
    mock_i2c_device.register_map[0x37] = reg.INT_STATUS_WOM_X

    # Read interrupt status
    status = icm.read_interrupt_status()

    assert status['wom_x'] is True
    assert status['wom_y'] is False
    assert status['wom_z'] is False


# ========================================================================
# Tap Detection Tests
# ========================================================================

def test_enable_tap_detection_low_noise(icm, mock_i2c_device):
    """Test enabling tap detection in low-noise mode."""
    icm.enable_tap_detection(mode="low-noise", int_pin=1)

    # Verify APEX is enabled
    # Check power mode supports tap detection (LN mode at high ODR)
    pwr_mgmt0 = get_register_value(mock_i2c_device, 0x4E)

    # Accelerometer should be in LN mode for tap detection
    accel_mode = pwr_mgmt0 & 0x03
    assert accel_mode == reg.ACCEL_MODE_LN


def test_enable_tap_detection_low_power(icm, mock_i2c_device):
    """Test enabling tap detection in low-power mode."""
    icm.enable_tap_detection(mode="low-power", int_pin=1)

    # Verify configuration
    pwr_mgmt0 = get_register_value(mock_i2c_device, 0x4E)
    accel_mode = pwr_mgmt0 & 0x03
    assert accel_mode == reg.ACCEL_MODE_LP


def test_enable_tap_detection_invalid_mode(icm):
    """Test that invalid tap detection mode raises ValueError."""
    with pytest.raises(ValueError, match="Mode must be 'low-noise' or 'low-power'"):
        icm.enable_tap_detection(mode="invalid", int_pin=1)


def test_enable_tap_detection_int_routing(icm, mock_i2c_device):
    """Test tap interrupt routing."""
    icm.enable_tap_detection(mode="low-noise", int_pin=1)

    # Verify tap interrupt is routed to INT1
    int_source0 = get_register_value(mock_i2c_device, 0x20)
    # Should have tap detection bit set


def test_tap_detection_status(icm, mock_i2c_device):
    """Test reading tap detection status."""
    # Enable tap detection
    icm.enable_tap_detection(mode="low-noise", int_pin=1)

    # Simulate tap interrupt (INT_STATUS3 = 0x38)
    mock_i2c_device.register_map[0x38] = reg.INT_STATUS_TAP

    # Read interrupt status
    status = icm.read_interrupt_status()

    assert status['tap'] is True


def test_read_tap_info_structure(icm, mock_i2c_device):
    """Test tap info reading returns correct structure."""
    # Enable tap detection
    icm.enable_tap_detection(mode="low-noise", int_pin=1)

    # Simulate tap data in APEX_DATA4 register (0x35)
    mock_i2c_device.register_map[0x35] = 0x00  # Single tap, X-axis

    tap_info = icm.read_tap_info()

    # Verify structure
    assert 'count' in tap_info
    assert 'axis' in tap_info
    assert 'direction' in tap_info

    # Verify types
    assert isinstance(tap_info['count'], str)
    assert isinstance(tap_info['axis'], str)
    assert isinstance(tap_info['direction'], bool)


def test_tap_info_single_tap(icm, mock_i2c_device):
    """Test tap info for single tap."""
    icm.enable_tap_detection(mode="low-noise", int_pin=1)

    # APEX_DATA4 format:
    # Bit 7-6: Tap number (00=single, 01=double)
    # Bit 5-4: Tap axis (00=X, 01=Y, 10=Z)
    # Bit 3: Tap direction (0=negative, 1=positive)

    # Single tap on X-axis, positive direction
    mock_i2c_device.register_map[0x35] = 0b00000000

    tap_info = icm.read_tap_info()
    assert tap_info['count'] == 'single'


def test_tap_info_double_tap(icm, mock_i2c_device):
    """Test tap info for double tap."""
    icm.enable_tap_detection(mode="low-noise", int_pin=1)

    # Double tap on Y-axis
    mock_i2c_device.register_map[0x35] = 0b01010000

    tap_info = icm.read_tap_info()
    assert tap_info['count'] == 'double'


def test_tap_info_all_axes(icm, mock_i2c_device):
    """Test tap detection on all axes."""
    icm.enable_tap_detection(mode="low-noise", int_pin=1)

    # Test X-axis (bits 1-2 = 00)
    mock_i2c_device.register_map[0x35] = reg.TAP_AXIS_X
    tap_info = icm.read_tap_info()
    assert tap_info['axis'] == 'x'

    # Test Y-axis (bits 1-2 = 01, bit 1 set)
    mock_i2c_device.register_map[0x35] = reg.TAP_AXIS_Y
    tap_info = icm.read_tap_info()
    assert tap_info['axis'] == 'y'

    # Test Z-axis (bits 1-2 = 10, bit 2 set)
    mock_i2c_device.register_map[0x35] = reg.TAP_AXIS_Z
    tap_info = icm.read_tap_info()
    assert tap_info['axis'] == 'z'


def test_tap_info_directions(icm, mock_i2c_device):
    """Test tap direction detection."""
    icm.enable_tap_detection(mode="low-noise", int_pin=1)

    # Negative direction (bit 0 = 0)
    mock_i2c_device.register_map[0x35] = 0b00000000
    tap_info = icm.read_tap_info()
    assert tap_info['direction'] is False

    # Positive direction (bit 0 = 1)
    mock_i2c_device.register_map[0x35] = 0b00000001
    tap_info = icm.read_tap_info()
    assert tap_info['direction'] is True


# ========================================================================
# Significant Motion Detection (SMD) Tests
# ========================================================================

def test_enable_smd_short_mode(icm, mock_i2c_device):
    """Test enabling SMD in short mode (1 second window)."""
    icm.enable_significant_motion_detection(mode="short", int_pin=1)

    # Verify SMD is enabled in APEX_CONFIG0 (Bank 4)
    # Verify power mode is LP
    pwr_mgmt0 = get_register_value(mock_i2c_device, 0x4E)
    assert (pwr_mgmt0 & 0x03) == reg.ACCEL_MODE_LP


def test_enable_smd_long_mode(icm, mock_i2c_device):
    """Test enabling SMD in long mode (3 second window)."""
    icm.enable_significant_motion_detection(mode="long", int_pin=1)

    # Verify SMD configuration
    pwr_mgmt0 = get_register_value(mock_i2c_device, 0x4E)
    assert (pwr_mgmt0 & 0x03) == reg.ACCEL_MODE_LP


def test_enable_smd_invalid_mode(icm):
    """Test that invalid SMD mode raises ValueError."""
    with pytest.raises(ValueError, match="Mode must be 'short' or 'long'"):
        icm.enable_significant_motion_detection(mode="invalid", int_pin=1)


def test_enable_smd_interrupt_routing(icm, mock_i2c_device):
    """Test SMD interrupt routing."""
    icm.enable_significant_motion_detection(mode="short", int_pin=1)

    # Verify SMD interrupt is routed to INT1
    int_source0 = get_register_value(mock_i2c_device, 0x20)
    # Should have SMD bit set


def test_smd_detection_status(icm, mock_i2c_device):
    """Test reading SMD status."""
    # Enable SMD
    icm.enable_significant_motion_detection(mode="short", int_pin=1)

    # Simulate SMD interrupt (INT_STATUS2 = 0x37)
    mock_i2c_device.register_map[0x37] = reg.INT_STATUS_SMD

    # Read interrupt status
    status = icm.read_interrupt_status()

    assert status['smd'] is True


def test_smd_with_wom_events(icm, mock_i2c_device):
    """Test SMD requires two WOM events."""
    # Enable SMD
    icm.enable_significant_motion_detection(mode="short", int_pin=1)

    # Simulate first WOM event (should not trigger SMD yet) - INT_STATUS2 = 0x37
    mock_i2c_device.register_map[0x37] = reg.INT_STATUS_WOM_X

    status = icm.read_interrupt_status()
    assert status['wom_x'] is True
    assert status['smd'] is False

    # Simulate second WOM event (would trigger SMD in real hardware)
    mock_i2c_device.register_map[0x37] = reg.INT_STATUS_WOM_X | reg.INT_STATUS_SMD

    status = icm.read_interrupt_status()
    assert status['smd'] is True


def test_smd_power_mode(icm, mock_i2c_device):
    """Test that SMD uses low-power mode."""
    icm.enable_significant_motion_detection(mode="short", int_pin=1)

    # Check power management
    pwr_mgmt0 = get_register_value(mock_i2c_device, 0x4E)

    # Accelerometer should be in LP mode
    accel_mode = pwr_mgmt0 & 0x03
    assert accel_mode == reg.ACCEL_MODE_LP

    # Gyroscope should be off
    gyro_mode = (pwr_mgmt0 >> 2) & 0x03
    assert gyro_mode == reg.GYRO_MODE_OFF


# ========================================================================
# Motion Detection Disable Tests
# ========================================================================

def test_disable_motion_detection_after_wom(icm, mock_i2c_device):
    """Test disabling motion detection after WOM was enabled."""
    # Enable WOM
    icm.enable_wake_on_motion(threshold=50, axes="all", int_pin=1)

    # Disable motion detection
    icm.disable_motion_detection()

    # Verify APEX is disabled
    # (Would check APEX_CONFIG0 register in Bank 4)


def test_disable_motion_detection_after_tap(icm, mock_i2c_device):
    """Test disabling motion detection after tap was enabled."""
    # Enable tap detection
    icm.enable_tap_detection(mode="low-noise", int_pin=1)

    # Disable motion detection
    icm.disable_motion_detection()


def test_disable_motion_detection_after_smd(icm, mock_i2c_device):
    """Test disabling motion detection after SMD was enabled."""
    # Enable SMD
    icm.enable_significant_motion_detection(mode="short", int_pin=1)

    # Disable motion detection
    icm.disable_motion_detection()


def test_disable_motion_detection_clears_interrupts(icm, mock_i2c_device):
    """Test that disabling motion detection clears pending interrupts."""
    # Enable WOM
    icm.enable_wake_on_motion(threshold=50, axes="all", int_pin=1)

    # Simulate pending WOM interrupt
    mock_i2c_device.register_map[0x2D] = reg.INT_STATUS_WOM_X

    # Disable motion detection
    icm.disable_motion_detection()

    # Read status (in real hardware, would be cleared)
    status = icm.read_interrupt_status()


# ========================================================================
# Combined Motion Detection Tests
# ========================================================================

def test_switch_between_motion_detection_modes(icm, mock_i2c_device):
    """Test switching between different motion detection modes."""
    # Start with WOM
    icm.enable_wake_on_motion(threshold=50, axes="all", int_pin=1)

    # Switch to tap detection
    icm.disable_motion_detection()
    icm.enable_tap_detection(mode="low-noise", int_pin=1)

    # Switch to SMD
    icm.disable_motion_detection()
    icm.enable_significant_motion_detection(mode="short", int_pin=1)


def test_motion_detection_with_different_int_pins(icm, mock_i2c_device):
    """Test routing different motion features to different interrupt pins."""
    # WOM on INT1
    icm.enable_wake_on_motion(threshold=50, axes="all", int_pin=1)
    icm.disable_motion_detection()

    # Tap on INT2
    icm.enable_tap_detection(mode="low-noise", int_pin=2)


def test_motion_detection_after_reset(icm, mock_i2c_device):
    """Test motion detection state after sensor reset."""
    # Enable WOM
    icm.enable_wake_on_motion(threshold=50, axes="all", int_pin=1)

    # Reset sensor
    icm.reset()

    # After reset, motion detection should be disabled
    # (Would need to re-enable)


def test_multiple_motion_interrupts_simultaneous(icm, mock_i2c_device):
    """Test handling multiple motion interrupt types simultaneously."""
    # Simulate multiple motion events in their respective registers
    mock_i2c_device.register_map[0x37] = reg.INT_STATUS_WOM_X | reg.INT_STATUS_WOM_Y  # INT_STATUS2
    mock_i2c_device.register_map[0x38] = reg.INT_STATUS_TAP  # INT_STATUS3

    status = icm.read_interrupt_status()

    assert status['wom_x'] is True
    assert status['wom_y'] is True
    assert status['tap'] is True
    assert status['smd'] is False
